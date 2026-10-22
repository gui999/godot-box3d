#include "box3d_physics_direct_space_state_3d.hpp"

#include "../shapes/box3d_concave_polygon_shape_impl_3d.hpp"

#include "../misc/box3d_shape_proxy.hpp"
#include "../misc/type_conversions.hpp"
#include "../objects/box3d_area_impl_3d.hpp"
#include "../objects/box3d_body_impl_3d.hpp"
#include "../objects/box3d_shaped_object_impl_3d.hpp"
#include "../servers/box3d_physics_server_3d.hpp"
#include "../shapes/box3d_shape_impl_3d.hpp"
#include "box3d_motion_core.hpp"
#include "box3d_query_filter_3d.hpp"
#include "box3d_space_3d.hpp"

#include <box3d/box3d.h>
#include "../shapes/box3d_voxel_module_cache.hpp"
#include <mutex>
#include <deque>
#include <cfloat>
#include <chrono>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/classes/worker_thread_pool.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

struct OverlapContext {
	const Box3DQueryFilter3D* filter = nullptr;
	PhysicsServer3DExtensionShapeResult* results = nullptr;
	int32_t max_results = 0;
	int32_t count = 0;
	// When set, a shape is only reported when it really is within margin of this proxy: b3World_OverlapShape also answers for
	// shapes a speculative distance away.
	const b3ShapeProxy* exact_proxy = nullptr;
	float exact_margin = 0.0f;
	b3WorldId world = b3_nullWorldId;
};

bool should_report(void* p_user_data, const Box3DQueryFilter3D& p_filter, Box3DShapedObjectImpl3D*& r_object) {
	auto* object = static_cast<Box3DShapedObjectImpl3D*>(p_user_data);
	if (object == nullptr) {
		return false;
	}
	const bool is_area = dynamic_cast<Box3DAreaImpl3D*>(object) != nullptr;
	if (is_area && !p_filter.collide_with_areas) {
		return false;
	}
	if (!is_area && !p_filter.collide_with_bodies) {
		return false;
	}
	if (p_filter.should_exclude(object->get_rid())) {
		return false;
	}
	r_object = object;
	return true;
}

int32_t find_shape_index(const Box3DShapedObjectImpl3D& p_object, b3ShapeId p_shape_id) {
	return p_object.find_shape_index(p_shape_id);
}

// The result structs are read by the engine, so they take the engine's own object pointer, not
// godot-cpp's wrapper (which ObjectDB::get_instance returns here).
Object* collider_object(const Box3DShapedObjectImpl3D& p_object) {
	const uint64_t id = p_object.get_instance_id();
	return id == 0 ? nullptr : reinterpret_cast<Object*>(internal::gdextension_interface_object_get_instance_from_id(id));
}

// A shape cast hits a mesh from either side (Box3D's casts are two sided). A mesh without backface collision is only solid on
// the front of its triangles, so a hit whose normal points out of a triangle's back is not a hit.
bool is_backside_mesh_hit(b3ShapeId p_shape_id, int p_triangle_index, const b3Vec3& p_normal) {
	if (p_triangle_index < 0 || b3Shape_GetType(p_shape_id) != b3_meshShape) {
		return false;
	}
	const b3Mesh mesh = b3Shape_GetMesh(p_shape_id);
	if (mesh.data->doubleSided != 0 || p_triangle_index >= mesh.data->triangleCount) {
		return false;
	}
	const b3MeshTriangle triangle = b3GetMeshTriangles(mesh.data)[p_triangle_index];
	const b3Vec3* vertices = b3GetMeshVertices(mesh.data);
	const b3Vec3 face_normal = b3Cross(
			b3Sub(vertices[triangle.index2], vertices[triangle.index1]),
			b3Sub(vertices[triangle.index3], vertices[triangle.index1]));
	const b3WorldTransform body_transform = b3Body_GetTransform(b3Shape_GetBody(p_shape_id));
	return b3Dot(b3RotateVector(body_transform.q, face_normal), p_normal) < 0.0f;
}

bool overlap_result_fcn(b3ShapeId p_shape_id, void* p_context) {
	auto* ctx = static_cast<OverlapContext*>(p_context);
	if (ctx->count >= ctx->max_results) {
		return false;
	}

	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	if (!should_report(b3Body_GetUserData(body_id), *ctx->filter, object)) {
		return true;
	}

	if (ctx->exact_proxy != nullptr) {
		b3m::Params params;
		params.world = ctx->world;
		params.margin = ctx->exact_margin;
		std::vector<b3m::Contact> contacts;
		if (b3m::shape_contacts(params, *ctx->exact_proxy, 0, p_shape_id, contacts, true) && contacts.empty()) {
			return true;
		}
	}

	PhysicsServer3DExtensionShapeResult& result = ctx->results[ctx->count];
	result.rid = object->get_rid();
	result.collider_id = object->get_instance_id();
	result.collider = collider_object(*object);
	result.shape = MAX(find_shape_index(*object, p_shape_id), 0);
	ctx->count++;
	return true;
}

struct CollideShapeContext {
	const Box3DQueryFilter3D* filter = nullptr;
	const b3ShapeProxy* query_proxy = nullptr;
	Vector3* results = nullptr;
	int32_t max_results = 0;
	int32_t count = 0;
	float margin = 0.0f;
};

// Reports the closest points between the query shape and one overlapping shape. Godot wants
// world-space pairs, and b3ShapeDistance runs in frame A, which is world space here because
// Box3DShapeProxy3D already bakes the transform into its points.
bool collide_shape_result_fcn(b3ShapeId p_shape_id, void* p_context) {
	auto* ctx = static_cast<CollideShapeContext*>(p_context);
	if (ctx->count >= ctx->max_results) {
		return false;
	}

	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	if (!should_report(b3Body_GetUserData(body_id), *ctx->filter, object)) {
		return true;
	}

	const Transform3D object_transform = object->get_transform();
	for (int32_t i = 0; i < object->get_shape_count(); i++) {
		if (!object->has_shape_id(i) || !B3_ID_EQUALS(object->get_shape_id(i), p_shape_id)) {
			continue;
		}

		const Box3DShapeProxy3D other_proxy(object->get_shape(i), object_transform * object->get_shape_transform(i));
		if (!other_proxy.is_supported()) {
			return true;
		}

		b3DistanceInput input{};
		input.proxyA = *ctx->query_proxy;
		input.proxyB = other_proxy.get_proxy();
		input.transform = b3Transform_identity;
		input.useRadii = true;

		b3SimplexCache cache{};
		const b3DistanceOutput output = b3ShapeDistance(&input, &cache, nullptr, 0);
		if (output.distance > ctx->margin) {
			return true; // within the speculative distance, not within the margin
		}

		// GJK cannot recover penetration depth, so an overlapping pair reports its witness
		// point for both sides rather than a fabricated depth.
		ctx->results[ctx->count * 2 + 0] = b3_to_godot(output.pointA);
		ctx->results[ctx->count * 2 + 1] = b3_to_godot(output.pointB);
		ctx->count++;
		return true;
	}
	return true;
}

struct RayContext {
	const Box3DQueryFilter3D* filter = nullptr;
	bool hit_from_inside = false;
	bool has_hit = false;
	bool is_ray = false;
	b3ShapeId shape_id = b3_nullShapeId;
	b3Pos point{};
	b3Vec3 normal{};
	float fraction = 1.0f;
	int triangle_index = -1;
};

float cast_result_fcn(b3ShapeId p_shape_id, b3Pos p_point, b3Vec3 p_normal, float p_fraction, uint64_t, int p_triangle_index, int, void* p_context) {
	auto* ctx = static_cast<RayContext*>(p_context);

	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	if (!should_report(b3Body_GetUserData(body_id), *ctx->filter, object)) {
		return -1.0f;
	}

	// Box3D reports a ray that starts inside a solid as a hit at the origin (fraction 0, zero normal).
	// Godot only reports that when hit_from_inside is set.
	if (ctx->is_ray && !ctx->hit_from_inside && p_fraction <= 0.0f && b3LengthSquared(p_normal) == 0.0f) {
		return -1.0f;
	}

	if (!ctx->is_ray && is_backside_mesh_hit(p_shape_id, p_triangle_index, p_normal)) {
		return -1.0f;
	}

	ctx->has_hit = true;
	ctx->triangle_index = p_triangle_index;
	ctx->shape_id = p_shape_id;
	ctx->point = p_point;
	ctx->normal = p_normal;
	ctx->fraction = p_fraction;
	return p_fraction;
}

// The motion core asks this about every shape it meets: only the shapes the filter lets through take part.
bool motion_accept(b3ShapeId p_shape_id, void* p_context) {
	const auto* filter = static_cast<const Box3DQueryFilter3D*>(p_context);
	Box3DShapedObjectImpl3D* object = nullptr;
	return should_report(b3Body_GetUserData(b3Shape_GetBody(p_shape_id)), *filter, object);
}

void fill_motion_collision(const b3m::Collision& p_source, Box3DShapedObjectImpl3D& p_object, int32_t p_local_shape, PhysicsServer3DExtensionMotionCollision& r_target) {
	const Vector3 position = b3_to_godot(p_source.point);
	r_target.position = position;
	r_target.normal = b3_to_godot(p_source.normal).normalized();
	r_target.depth = p_source.depth;
	r_target.local_shape = p_local_shape;
	r_target.collider = p_object.get_rid();
	r_target.collider_id = p_object.get_instance_id();
	r_target.collider_shape = find_shape_index(p_object, p_source.shape);

	auto* body = dynamic_cast<Box3DBodyImpl3D*>(&p_object);
	if (body != nullptr) {
		r_target.collider_angular_velocity = body->get_angular_velocity();
		const Vector3 center_of_mass = body->get_transform().xform(body->get_center_of_mass());
		r_target.collider_velocity =
				body->get_linear_velocity() + body->get_angular_velocity().cross(position - center_of_mass);
	}
}

} // namespace

bool Box3DPhysicsDirectSpaceState3D::_intersect_ray(
		const Vector3& p_from,
		const Vector3& p_to,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		bool p_hit_from_inside,
		bool p_hit_back_faces,
		bool p_pick_ray,
		PhysicsServer3DExtensionRayResult* p_result) {
	ERR_FAIL_NULL_V(space, false);

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	RayContext context;
	context.filter = &filter;
	context.hit_from_inside = p_hit_from_inside;
	context.is_ray = true;

	const b3Vec3 origin = godot_to_b3(p_from);
	const b3Vec3 translation = godot_to_b3(p_to - p_from);

	b3World_CastRay(space->get_world_id(), origin, translation, filter.filter, cast_result_fcn, &context);

	if (!context.has_hit) {
		return false;
	}

	const b3BodyId body_id = b3Shape_GetBody(context.shape_id);
	auto* object = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(body_id));
	if (object == nullptr) {
		return false;
	}

	p_result->position = b3_to_godot(context.point);
	p_result->normal = b3_to_godot(context.normal);
	p_result->rid = object->get_rid();
	p_result->collider_id = object->get_instance_id();
	p_result->collider = collider_object(*object);
	p_result->shape = MAX(find_shape_index(*object, context.shape_id), 0);
	p_result->face_index = context.triangle_index;
	return true;
}

int32_t Box3DPhysicsDirectSpaceState3D::_intersect_point(
		const Vector3& p_position,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_results,
		int32_t p_max_results) {
	ERR_FAIL_NULL_V(space, 0);

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	const b3Vec3 point = godot_to_b3(p_position);
	b3ShapeProxy proxy;
	proxy.points = &point;
	proxy.count = 1;
	proxy.radius = 0.0f;

	OverlapContext context;
	context.filter = &filter;
	context.results = p_results;
	context.max_results = p_max_results;

	b3World_OverlapShape(space->get_world_id(), b3Vec3_zero, &proxy, filter.filter, overlap_result_fcn, &context);

	return context.count;
}

int32_t Box3DPhysicsDirectSpaceState3D::_intersect_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_results,
		int32_t p_max_results) {
	ERR_FAIL_NULL_V(space, 0);

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, 0);

	// A concave query shape has no overlap proxy, but querying with it is how a worker thread asks
	// for its Box3D mesh to be prebuilt (so attaching it to a body later is cheap). Build and return;
	// this touches only the shape's own mutex-guarded cache, no world or server state.
	if (shape->get_type() == PhysicsServer3D::SHAPE_CONCAVE_POLYGON) {
		static_cast<Box3DConcavePolygonShapeImpl3D*>(shape)->get_mesh();
		return 0;
	}

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		return 0;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	const float margin = MAX(0.0f, (float)p_margin);
	const b3ShapeProxy query{ shape_proxy.get_proxy().points, shape_proxy.get_proxy().count, shape_proxy.get_proxy().radius + margin };

	OverlapContext context;
	context.filter = &filter;
	context.results = p_results;
	context.max_results = p_max_results;
	context.exact_proxy = &shape_proxy.get_proxy();
	context.exact_margin = margin;
	context.world = space->get_world_id();

	b3World_OverlapShape(space->get_world_id(), b3Vec3_zero, &query, filter.filter, overlap_result_fcn, &context);

	return context.count;
}

bool Box3DPhysicsDirectSpaceState3D::_cast_motion(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		float* p_closest_safe,
		float* p_closest_unsafe,
		PhysicsServer3DExtensionShapeRestInfo* p_info) {
	ERR_FAIL_NULL_V(space, false);

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		*p_closest_safe = 1.0;
		*p_closest_unsafe = 1.0;
		return false;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	// Godot (and Jolt) answer a cast that hits nothing with [1, 1]; false means the query failed. The sweep is the motion core's,
	// on exact separations: Box3D's own shape cast reports contact early (within its linear slop).
	const b3ShapeProxy& proxy = shape_proxy.get_proxy();
	b3m::BodyShape body_shape;
	body_shape.points.assign(proxy.points, proxy.points + proxy.count);
	body_shape.radius = proxy.radius;

	b3m::Params params;
	params.world = space->get_world_id();
	params.filter = filter.filter;
	params.accept = motion_accept;
	params.accept_context = &filter;
	params.margin = MAX(0.0f, (float)p_margin);
	params.motion = godot_to_b3(p_motion);

	float safe = 1.0f;
	float unsafe = 1.0f;
	b3m::cast_shapes({ body_shape }, params, safe, unsafe);
	*p_closest_safe = safe;
	*p_closest_unsafe = unsafe;
	return true;
}

bool Box3DPhysicsDirectSpaceState3D::_collide_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		void* p_results,
		int32_t p_max_results,
		int32_t* p_result_count) {
	*p_result_count = 0;
	ERR_FAIL_NULL_V(space, false);
	if (p_max_results <= 0) {
		return false;
	}

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		return false;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	const float margin = MAX(0.0f, (float)p_margin);
	const b3ShapeProxy query{ shape_proxy.get_proxy().points, shape_proxy.get_proxy().count, shape_proxy.get_proxy().radius + margin };

	CollideShapeContext context;
	context.filter = &filter;
	context.query_proxy = &shape_proxy.get_proxy();
	context.results = static_cast<Vector3*>(p_results);
	context.max_results = p_max_results;
	context.margin = margin;

	b3World_OverlapShape(space->get_world_id(), b3Vec3_zero, &query, filter.filter, collide_shape_result_fcn, &context);

	*p_result_count = context.count;
	return context.count > 0;
}

bool Box3DPhysicsDirectSpaceState3D::_rest_info(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeRestInfo* p_info) {
	ERR_FAIL_NULL_V(space, false);

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		return false;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	// The closest contact within the margin at the given pose (the motion plays no part, as on Jolt).
	b3m::Params params;
	params.world = space->get_world_id();
	params.filter = filter.filter;
	params.accept = motion_accept;
	params.accept_context = &filter;
	params.margin = MAX(0.0f, (float)p_margin);

	std::vector<b3m::Contact> contacts;
	b3m::collect_contacts(params, shape_proxy.get_proxy(), 0, contacts);
	const b3m::Contact* closest = nullptr;
	for (const b3m::Contact& contact : contacts) {
		if (closest == nullptr || contact.separation < closest->separation) {
			closest = &contact;
		}
	}
	if (closest == nullptr) {
		return false;
	}

	auto* object = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(b3Shape_GetBody(closest->shape)));
	if (object == nullptr) {
		return false;
	}

	p_info->point = b3_to_godot(closest->point);
	p_info->normal = b3_to_godot(closest->normal);
	p_info->rid = object->get_rid();
	p_info->collider_id = object->get_instance_id();
	p_info->shape = MAX(find_shape_index(*object, closest->shape), 0);

	auto* body = dynamic_cast<Box3DBodyImpl3D*>(object);
	if (body != nullptr) {
		p_info->linear_velocity = body->get_linear_velocity();
	}

	return true;
}

Vector3 Box3DPhysicsDirectSpaceState3D::_get_closest_point_to_object_volume(const RID& p_object, const Vector3& p_point) const {
	Box3DShapedObjectImpl3D* object = Box3DPhysicsServer3D::get_singleton()->get_body(p_object);
	if (object == nullptr) {
		object = Box3DPhysicsServer3D::get_singleton()->get_area(p_object);
	}
	if (object == nullptr || !object->has_body_id()) {
		return p_point;
	}

	b3Vec3 result_point{};
	b3Body_GetClosestPoint(object->get_body_id(), &result_point, godot_to_b3(p_point));
	return b3_to_godot(result_point);
}

namespace {

constexpr int32_t MOVER_MAX_PLANES = 64;
constexpr int32_t MOVER_PLANE_STRIDE = 8;
// move_actors' layouts (its contract is at the method).
constexpr int32_t MOVER_REQUEST_STRIDE = 20;
constexpr int32_t MOVER_RESULT_STRIDE = 16;

// The mover queries filter like test_body_motion: the body's mask, minus the body and its collision exceptions.
bool make_mover_filter(const Box3DPhysicsDirectSpaceState3D* p_state, const RID& p_body, Box3DQueryFilter3D& r_filter) {
	Box3DBodyImpl3D* body = Box3DPhysicsServer3D::get_singleton()->get_body(p_body);
	ERR_FAIL_NULL_V_MSG(body, false, "Box3D mover query: the RID is not a body.");
	r_filter.set_collision_mask(body->get_collision_mask());
	r_filter.direct_state = p_state;
	r_filter.exclude.insert(p_body);
	for (const KeyValue<RID, Box3DFilterJointImpl3D*>& entry : body->get_collision_exceptions()) {
		r_filter.exclude.insert(entry.key);
	}
	return true;
}

// A mover only touches solid bodies (no areas, no sensors) that the filter does not exclude.
bool mover_accepts(b3ShapeId p_shape_id, const Box3DQueryFilter3D& p_filter) {
	if (b3Shape_IsSensor(p_shape_id)) {
		return false;
	}
	Box3DShapedObjectImpl3D* object = nullptr;
	return should_report(b3Body_GetUserData(b3Shape_GetBody(p_shape_id)), p_filter, object);
}

struct CollideMoverContext {
	const Box3DQueryFilter3D* filter = nullptr;
	b3Vec3 origin = b3Vec3_zero;
	float* planes = nullptr;
	int32_t count = 0;
};

bool collide_mover_result_fcn(b3ShapeId p_shape_id, const b3PlaneResult* p_planes, int p_plane_count, void* p_context) {
	auto* ctx = static_cast<CollideMoverContext*>(p_context);
	if (!mover_accepts(p_shape_id, *ctx->filter)) {
		return true;
	}
	// b3BodyType values are 0 static, 1 kinematic, 2 dynamic.
	const float mode = (float)b3Body_GetType(b3Shape_GetBody(p_shape_id));
	for (int i = 0; i < p_plane_count && ctx->count < MOVER_MAX_PLANES; i++) {
		const b3PlaneResult& result = p_planes[i];
		const b3Vec3 point = b3Add(ctx->origin, result.point);
		float* out = ctx->planes + ctx->count * MOVER_PLANE_STRIDE;
		out[0] = result.plane.normal.x;
		out[1] = result.plane.normal.y;
		out[2] = result.plane.normal.z;
		out[3] = result.plane.offset;
		out[4] = point.x;
		out[5] = point.y;
		out[6] = point.z;
		out[7] = mode;
		ctx->count++;
	}
	return ctx->count < MOVER_MAX_PLANES;
}

bool cast_mover_filter_fcn(b3ShapeId p_shape_id, void* p_context) {
	return mover_accepts(p_shape_id, *static_cast<const Box3DQueryFilter3D*>(p_context));
}

// The capsule goes to Box3D relative to its midpoint to keep the coordinates small.
b3Capsule mover_capsule(const Vector3& p_center1, const Vector3& p_center2, float p_radius, b3Vec3& r_origin) {
	const b3Vec3 center1 = godot_to_b3(p_center1);
	const b3Vec3 center2 = godot_to_b3(p_center2);
	r_origin = b3MulSV(0.5f, b3Add(center1, center2));
	return { b3Sub(center1, r_origin), b3Sub(center2, r_origin), p_radius };
}

// collide_mover's planes into r_planes (room for MOVER_MAX_PLANES), their count returned.
int32_t collide_mover_into(b3WorldId p_world, const Box3DQueryFilter3D& p_filter, const Vector3& p_center1, const Vector3& p_center2,
		float p_radius, float* r_planes) {
	b3Vec3 origin;
	const b3Capsule mover = mover_capsule(p_center1, p_center2, p_radius, origin);
	CollideMoverContext context;
	context.filter = &p_filter;
	context.origin = origin;
	context.planes = r_planes;
	b3World_CollideMover(p_world, origin, &mover, p_filter.filter, collide_mover_result_fcn, &context);
	return context.count;
}

float cast_mover_with(b3WorldId p_world, const Box3DQueryFilter3D& p_filter, const Vector3& p_center1, const Vector3& p_center2,
		float p_radius, const Vector3& p_translation) {
	b3Vec3 origin;
	const b3Capsule mover = mover_capsule(p_center1, p_center2, p_radius, origin);
	return b3World_CastMover(p_world, origin, &mover, godot_to_b3(p_translation), p_filter.filter, cast_mover_filter_fcn,
			const_cast<Box3DQueryFilter3D*>(&p_filter));
}

} // namespace

void Box3DPhysicsDirectSpaceState3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("collide_mover", "body", "center1", "center2", "radius"), &Box3DPhysicsDirectSpaceState3D::collide_mover);
	ClassDB::bind_method(D_METHOD("cast_mover", "body", "center1", "center2", "radius", "translation"), &Box3DPhysicsDirectSpaceState3D::cast_mover);
	ClassDB::bind_method(D_METHOD("move_actors", "bodies", "requests", "workers"), &Box3DPhysicsDirectSpaceState3D::move_actors);
	ClassDB::bind_method(D_METHOD("box3d_byte_count"), &Box3DPhysicsDirectSpaceState3D::box3d_byte_count);
	ClassDB::bind_method(D_METHOD("world_counters"), &Box3DPhysicsDirectSpaceState3D::world_counters);
	ClassDB::bind_method(D_METHOD("prepare_voxel_modules", "boxes", "box_offsets", "cell_meters", "cell_voxels"), &Box3DPhysicsDirectSpaceState3D::prepare_voxel_modules);
	ClassDB::bind_method(D_METHOD("set_shape_max_normal_impulse", "body", "shape_index", "impulse"), &Box3DPhysicsDirectSpaceState3D::set_shape_max_normal_impulse);
	ClassDB::bind_method(D_METHOD("get_shape_max_normal_impulse", "body", "shape_index"), &Box3DPhysicsDirectSpaceState3D::get_shape_max_normal_impulse);
	ClassDB::bind_method(D_METHOD("set_limit_pass_through", "enabled"), &Box3DPhysicsDirectSpaceState3D::set_limit_pass_through);
	ClassDB::bind_method(D_METHOD("set_voxel_point_contacts", "enabled"), &Box3DPhysicsDirectSpaceState3D::set_voxel_point_contacts);
	ClassDB::bind_method(D_METHOD("is_voxel_point_contacts"), &Box3DPhysicsDirectSpaceState3D::is_voxel_point_contacts);
	ClassDB::bind_method(D_METHOD("shape_contact_impulses", "body", "shape_index"), &Box3DPhysicsDirectSpaceState3D::shape_contact_impulses);
}

// 8 floats per plane: normal xyz, plane offset (as Box3D reports it, relative to the capsule midpoint), contact point xyz in
// world space, collider body mode (0 static, 1 kinematic, 2 dynamic). The capsule goes to Box3D relative to its midpoint to
// keep the coordinates small.
PackedFloat32Array Box3DPhysicsDirectSpaceState3D::collide_mover(
		const RID& p_body, const Vector3& p_center1, const Vector3& p_center2, float p_radius) {
	PackedFloat32Array planes;
	ERR_FAIL_NULL_V(space, planes);
	Box3DQueryFilter3D filter;
	if (!make_mover_filter(this, p_body, filter)) {
		return planes;
	}
	planes.resize(MOVER_MAX_PLANES * MOVER_PLANE_STRIDE);
	const int32_t count = collide_mover_into(space->get_world_id(), filter, p_center1, p_center2, p_radius, planes.ptrw());
	planes.resize(count * MOVER_PLANE_STRIDE);
	return planes;
}

// The free fraction of the translation, 1 when nothing is hit.
float Box3DPhysicsDirectSpaceState3D::cast_mover(
		const RID& p_body, const Vector3& p_center1, const Vector3& p_center2, float p_radius, const Vector3& p_translation) {
	ERR_FAIL_NULL_V(space, 1.0f);
	Box3DQueryFilter3D filter;
	if (!make_mover_filter(this, p_body, filter)) {
		return 1.0f;
	}
	return cast_mover_with(space->get_world_id(), filter, p_center1, p_center2, p_radius, p_translation);
}

namespace {

// The batched actor mover (Terra's crowd prototype, docs/89 row 3, D817): every actor's move for one tick, each
// exactly as the game's C# motor makes it (TerraPrime's ActorMotor.Move over CapsuleMover: Box3D's samples/mover.cpp
// plane mover, MIT, Erin Catto), the same float operations in the same order, so the answers match the C# path bit
// for bit. Each actor moves against the world as it stands when the call begins (the other actors' start-of-tick
// poses, D817): the call moves no body, every query only reads the world, and each actor's answer lands in its own
// slot, so it is the same on any number of workers. The plugin never records a Box3D world (b3World's recording
// writer is not safe for parallel queries).
namespace mover {

constexpr int32_t ITERATIONS = 5;
constexpr float SLOP = 0.005f;
constexpr float CLEARANCE = 0.001f;
constexpr float FLOOR_RISE = 0.7f;
constexpr float TOUCHING = 4 * SLOP;
constexpr float TOLERANCE = 0.001f;
const Vector3 UP(0, 1, 0);
const Vector3 DOWN(0, -1, 0);

struct Plane {
	Vector3 normal, solving, point;
	float offset = 0, push = 0;
	int32_t mode = 0;
};

// One actor's scratch: its query filter, the planes last collided and those the last move's solve pushed on.
struct Actor {
	Box3DQueryFilter3D filter;
	Plane planes[MOVER_MAX_PLANES];
	int32_t count = 0;
	Vector3 pushed[MOVER_MAX_PLANES];
	int32_t pushed_count = 0;
	int32_t iterated = 0;
	bool on_floor = false, on_wall = false, on_ceiling = false;
	Vector3 floor_normal = UP, wall_normal;
};

struct Batch {
	b3WorldId world;
	const float* requests = nullptr;
	float* results = nullptr;
	float* planes = nullptr;
	Actor* actors = nullptr;
};

void centres(const Vector3& p_origin, const Vector3& p_shape_offset, float p_height, float p_radius, Vector3& r_low, Vector3& r_high) {
	const Vector3 centre = p_origin + p_shape_offset + DOWN * SLOP;
	const float half = std::max(0.0f, p_height / 2 - p_radius);
	r_low = centre + DOWN * half;
	r_high = centre + UP * half;
}

void collide(const Batch& p_batch, Actor& r_actor, const Vector3& p_origin, const Vector3& p_shape_offset, float p_height, float p_radius) {
	Vector3 low, high;
	centres(p_origin, p_shape_offset, p_height, p_radius, low, high);
	float data[MOVER_MAX_PLANES * MOVER_PLANE_STRIDE];
	r_actor.count = collide_mover_into(p_batch.world, r_actor.filter, low, high, p_radius + TOUCHING, data);
	for (int32_t i = 0; i < r_actor.count; i++) {
		const float* at = data + i * MOVER_PLANE_STRIDE;
		Plane& plane = r_actor.planes[i];
		plane.normal = Vector3(at[0], at[1], at[2]);
		plane.offset = at[3] - TOUCHING;
		plane.point = Vector3(at[4], at[5], at[6]);
		plane.mode = (int32_t)at[7];
		plane.solving = Vector3();
		plane.push = 0;
	}
}

float cast(const Batch& p_batch, const Actor& p_actor, const Vector3& p_origin, const Vector3& p_shape_offset, float p_height, float p_radius,
		const Vector3& p_motion) {
	if (p_motion.length_squared() < 1e-12f) {
		return 1;
	}
	Vector3 low, high;
	centres(p_origin, p_shape_offset, p_height, p_radius, low, high);
	const float fraction = cast_mover_with(p_batch.world, p_actor.filter, low, high, p_radius, p_motion);
	return fraction >= 1 ? 1 : std::max(0.0f, fraction - CLEARANCE / p_motion.length());
}

Vector3 solve(Actor& r_actor, Vector3 p_delta, float p_block_cos) {
	for (int32_t i = 0; i < r_actor.count; i++) {
		Plane& plane = r_actor.planes[i];
		const float rise = plane.normal.y;
		const Vector3 across(plane.normal.x, 0, plane.normal.z);
		plane.solving = rise > 0 && rise < p_block_cos && across.length_squared() > 1e-6f ? across.normalized() : plane.normal;
	}
	for (int32_t i = 0; i < r_actor.count; i++) {
		r_actor.planes[i].push = 0;
	}
	int32_t iteration;
	for (iteration = 0; iteration < 20; iteration++) {
		float total = 0;
		for (int32_t i = 0; i < r_actor.count; i++) {
			Plane& plane = r_actor.planes[i];
			const float separation = plane.solving.dot(p_delta) - plane.offset + SLOP;
			const float accumulated = plane.push;
			plane.push = std::max(0.0f, plane.push - separation);
			const float push = plane.push - accumulated;
			p_delta += plane.solving * push;
			total += std::abs(push);
		}
		if (total < SLOP) {
			break;
		}
	}
	r_actor.iterated = std::max(r_actor.iterated, iteration);
	return p_delta;
}

Vector3 clip(const Actor& p_actor, Vector3 p_velocity) {
	for (int32_t i = 0; i < p_actor.pushed_count; i++) {
		p_velocity -= p_actor.pushed[i] * std::min(0.0f, p_velocity.dot(p_actor.pushed[i]));
	}
	return p_velocity;
}

Vector3 move(const Batch& p_batch, Actor& r_actor, const Vector3& p_origin, const Vector3& p_shape_offset, float p_height, float p_radius,
		const Vector3& p_motion, float p_block_cos) {
	r_actor.iterated = 0;
	Vector3 position = p_origin;
	const Vector3 target = p_origin + p_motion;
	for (int32_t iteration = 0; iteration < ITERATIONS; iteration++) {
		collide(p_batch, r_actor, position, p_shape_offset, p_height, p_radius);
		Vector3 delta = solve(r_actor, target - position, p_block_cos);
		delta *= cast(p_batch, r_actor, position, p_shape_offset, p_height, p_radius, delta);
		position += delta;
		if (delta.length_squared() < TOLERANCE * TOLERANCE) {
			break;
		}
	}
	r_actor.pushed_count = 0;
	for (int32_t i = 0; i < r_actor.count; i++) {
		if (r_actor.planes[i].push > 0) {
			r_actor.pushed[r_actor.pushed_count++] = r_actor.planes[i].solving;
		}
	}
	// Out of walls and ceilings the body steps back to the clearance; floors keep the solver's slop.
	collide(p_batch, r_actor, position, p_shape_offset, p_height, p_radius);
	for (int32_t i = 0; i < r_actor.count; i++) {
		const float separation = -r_actor.planes[i].offset;
		if (r_actor.planes[i].normal.y < FLOOR_RISE && separation < CLEARANCE && separation > -4 * SLOP) {
			position += r_actor.planes[i].normal * (CLEARANCE - separation);
		}
	}
	return position;
}

Vector3 support_normal(const Actor& p_actor) {
	Vector3 sum;
	for (int32_t i = 0; i < p_actor.count; i++) {
		if (-p_actor.planes[i].offset <= TOUCHING && p_actor.planes[i].normal.dot(UP) > 0) {
			sum += p_actor.planes[i].normal;
		}
	}
	return sum.length_squared() > 1e-8f ? sum.normalized() : Vector3();
}

void classify(Actor& r_actor, float p_floor_cos, const Vector3& p_motion) {
	r_actor.on_floor = r_actor.on_wall = r_actor.on_ceiling = false;
	Vector3 floor;
	float against = FLT_MAX;
	const Vector3 direction = (p_motion - UP * p_motion.dot(UP)).normalized();
	for (int32_t i = 0; i < r_actor.count; i++) {
		const Plane& plane = r_actor.planes[i];
		if (-plane.offset > TOUCHING) {
			continue;
		}
		const float rise = plane.normal.dot(UP);
		if (rise >= p_floor_cos) {
			r_actor.on_floor = true;
			floor += plane.normal;
		} else if (rise <= -p_floor_cos) {
			r_actor.on_ceiling = true;
		} else {
			r_actor.on_wall = true;
			const float facing = direction.is_finite() ? plane.normal.dot(direction) : 0;
			if (facing < against) {
				against = facing;
				r_actor.wall_normal = plane.normal;
			}
		}
	}
	if (!r_actor.on_floor) {
		const Vector3 support = support_normal(r_actor);
		if (support.dot(UP) >= p_floor_cos) {
			r_actor.on_floor = true;
			floor = support;
		}
	}
	r_actor.floor_normal = r_actor.on_floor ? floor.normalized() : UP;
}

void move_actor(void* p_batch, uint32_t p_index) {
	const Batch& batch = *static_cast<const Batch*>(p_batch);
	Actor& actor = batch.actors[p_index];
	const float* in = batch.requests + (size_t)p_index * MOVER_REQUEST_STRIDE;
	const Vector3 start(in[0], in[1], in[2]);
	const Vector3 shape_offset(in[3], in[4], in[5]);
	const float height = in[6], radius = in[7];
	const Vector3 desired(in[8], in[9], in[10]);
	const float delta = in[11], floor_cos = in[12], snap_length = in[13];
	const int32_t flags = (int32_t)in[14];
	const bool was_on_floor = (flags & 1) != 0, snap = (flags & 2) != 0;
	actor.wall_normal = Vector3(in[15], in[16], in[17]);
	actor.on_floor = was_on_floor;

	const float block_cos = was_on_floor ? floor_cos : 2;
	Vector3 motion = desired * delta;
	if (was_on_floor && snap) {
		motion.y = 0;
	}
	Vector3 position = move(batch, actor, start, shape_offset, height, radius, motion, block_cos);
	Vector3 velocity = clip(actor, desired);
	collide(batch, actor, position, shape_offset, height, radius);
	classify(actor, floor_cos, desired);
	if (snap && was_on_floor && !actor.on_floor && snap_length > 0) {
		const Vector3 down = DOWN * snap_length;
		const float fraction = cast(batch, actor, position, shape_offset, height, radius, down);
		if (fraction < 1) {
			const Vector3 snapped = position + down * fraction;
			collide(batch, actor, snapped, shape_offset, height, radius);
			classify(actor, floor_cos, desired);
			if (actor.on_floor) {
				position = snapped;
			} else {
				collide(batch, actor, position, shape_offset, height, radius);
				classify(actor, floor_cos, desired);
			}
		}
	}
	if (actor.on_floor && velocity.y < 0) {
		velocity.y = 0;
	}

	float* out = batch.results + (size_t)p_index * MOVER_RESULT_STRIDE;
	out[0] = position.x, out[1] = position.y, out[2] = position.z;
	out[3] = velocity.x, out[4] = velocity.y, out[5] = velocity.z;
	out[6] = (float)((actor.on_floor ? 1 : 0) | (actor.on_wall ? 2 : 0) | (actor.on_ceiling ? 4 : 0));
	out[7] = actor.floor_normal.x, out[8] = actor.floor_normal.y, out[9] = actor.floor_normal.z;
	out[10] = actor.wall_normal.x, out[11] = actor.wall_normal.y, out[12] = actor.wall_normal.z;
	out[13] = (float)actor.iterated;
	out[14] = (float)actor.count;
	out[15] = 0;
	float* planes = batch.planes + (size_t)p_index * MOVER_MAX_PLANES * MOVER_PLANE_STRIDE;
	for (int32_t i = 0; i < actor.count; i++) {
		const Plane& plane = actor.planes[i];
		float* at = planes + i * MOVER_PLANE_STRIDE;
		at[0] = plane.normal.x, at[1] = plane.normal.y, at[2] = plane.normal.z;
		at[3] = plane.offset;
		at[4] = plane.point.x, at[5] = plane.point.y, at[6] = plane.point.z;
		at[7] = (float)plane.mode;
	}
}

} // namespace mover
} // namespace

// Moves every actor for one tick (docs/89 row 3): bodies one RID per actor, requests MOVER_REQUEST_STRIDE floats per
// actor (start xyz, the capsule shape's offset xyz, height, radius, desired velocity xyz, the tick's seconds, the cosine
// of the floor angle, the floor snap length, flags (1 on a floor before the move, 2 snap to the floor), the wall normal
// before the move xyz, two unused), workers the most threads the moves share (1 on the calling thread; 0 as many as
// the pool gives). Returns [results, planes, plane_starts]: MOVER_RESULT_STRIDE floats per actor (position xyz,
// velocity xyz, flags (1 floor, 2 wall, 4 ceiling), floor normal xyz, wall normal xyz, most solver rounds, plane
// count, unused), the planes the body touches where it stops (collide_mover's layout, the offset less the touching
// window as the C# mover keeps it), and where each actor's planes start (one more entry than actors). Main thread,
// between physics steps; an invalid body fails the whole batch with an empty Array.
Array Box3DPhysicsDirectSpaceState3D::move_actors(const Array& p_bodies, const PackedFloat32Array& p_requests, int p_workers) {
	Array answer;
	ERR_FAIL_NULL_V(space, answer);
	const int64_t count = p_bodies.size();
	ERR_FAIL_COND_V_MSG(p_requests.size() != count * MOVER_REQUEST_STRIDE, answer,
			"Box3D move_actors: requests must hold MOVER_REQUEST_STRIDE floats per body.");
	ERR_FAIL_COND_V_MSG(p_workers < 0, answer, "Box3D move_actors: workers must be 0 (the pool's) or more.");

	// Scratch kept between calls: the method runs on the main thread only.
	static std::vector<mover::Actor> mover_actors;
	static std::vector<float> mover_planes;
	if ((int64_t)mover_actors.size() < count) {
		mover_actors.resize(count);
	}
	mover_planes.resize(count * MOVER_MAX_PLANES * MOVER_PLANE_STRIDE);
	PackedFloat32Array results;
	results.resize(count * MOVER_RESULT_STRIDE);
	for (int64_t i = 0; i < count; i++) {
		mover_actors[i].filter = Box3DQueryFilter3D();
		if (!make_mover_filter(this, p_bodies[i], mover_actors[i].filter)) {
			return answer;
		}
	}

	mover::Batch batch;
	batch.world = space->get_world_id();
	batch.requests = p_requests.ptr();
	batch.results = results.ptrw();
	batch.planes = mover_planes.data();
	batch.actors = mover_actors.data();
	if (p_workers == 1 || count < 2) {
		for (int64_t i = 0; i < count; i++) {
			mover::move_actor(&batch, (uint32_t)i);
		}
	} else {
		WorkerThreadPool* pool = WorkerThreadPool::get_singleton();
		const int tasks = p_workers == 0 ? -1 : (int)std::min<int64_t>(p_workers, count);
		const WorkerThreadPool::GroupID group = pool->add_native_group_task(&mover::move_actor, &batch, (int)count, tasks, true, "Box3D move_actors");
		pool->wait_for_group_task_completion(group);
	}

	PackedInt32Array starts;
	starts.resize(count + 1);
	int32_t total = 0;
	for (int64_t i = 0; i < count; i++) {
		starts.set(i, total);
		total += mover_actors[i].count;
	}
	starts.set(count, total);
	PackedFloat32Array planes;
	planes.resize(total * MOVER_PLANE_STRIDE);
	float* to = planes.ptrw();
	for (int64_t i = 0; i < count; i++) {
		const int32_t floats = mover_actors[i].count * MOVER_PLANE_STRIDE;
		memcpy(to, mover_planes.data() + i * MOVER_MAX_PLANES * MOVER_PLANE_STRIDE, floats * sizeof(float));
		to += floats;
	}
	answer.append(results);
	answer.append(planes);
	answer.append(starts);
	return answer;
}

namespace {

// Modules derived ahead of time on a worker, held until their grid's body has had time to enter.
struct PreparedModules {
	std::chrono::steady_clock::time_point at;
	std::vector<b3VoxelGridModule*> modules;
};
std::mutex prepared_mutex;
std::deque<PreparedModules> prepared;
constexpr double PREPARED_SECONDS = 10.0;

} // namespace

// Derives a voxel grid's modules into the module cache now, on the calling thread (a worker building the
// grid), so the body that takes the grid finds them made instead of deriving them on the main thread as
// it enters the space (a terrain chunk's grid holds thousands of distinct boxes). Boxes and offsets as
// the grid shape's data takes them. Each module is held 10 s, then released (the cache keeps it while a
// grid does). Thread safe.
void Box3DPhysicsDirectSpaceState3D::prepare_voxel_modules(const PackedFloat32Array& p_boxes, const PackedInt32Array& p_box_offsets, float p_cell_meters, int p_cell_voxels) {
	PreparedModules made;
	made.at = std::chrono::steady_clock::now();
	const float* boxes = p_boxes.ptr();
	for (int64_t m = 0; m + 1 < p_box_offsets.size(); m++) {
		const int32_t first = p_box_offsets[m], end = p_box_offsets[m + 1];
		ERR_FAIL_COND_MSG(first < 0 || end < first || (int64_t)end * 6 > p_boxes.size(), "Box3D: prepare_voxel_modules was given offsets past its boxes.");
		made.modules.push_back(Box3DVoxelModuleCache::acquire(boxes + (size_t)first * 6, end - first, p_cell_meters, p_cell_voxels));
	}
	std::vector<b3VoxelGridModule*> expired;
	{
		std::lock_guard<std::mutex> lock(prepared_mutex);
		while (!prepared.empty() && std::chrono::duration<double>(made.at - prepared.front().at).count() > PREPARED_SECONDS) {
			expired.insert(expired.end(), prepared.front().modules.begin(), prepared.front().modules.end());
			prepared.pop_front();
		}
		prepared.push_back(std::move(made));
	}
	for (b3VoxelGridModule* module : expired) {
		b3ReleaseVoxelGridModule(module);
	}
}
// Terra (D604): the most normal impulse in N*s that a contact of the body's shape (its Godot shape index) may apply over
// one step, its sub-steps and restitution included; a contact takes the lower of its two shapes' limits, and each contact
// has the whole limit (two bodies on one shape in a step each meet it). INF is no limit (the default). The body keeps the
// limit across rebuilds of its shape, and it takes effect at the next step: set it between steps, on the main thread
// (Box3D refuses a change while it steps). Only contacts on Box3D's scalar path honour it, which every voxel grid,
// mesh and height field contact is; a contact between two convex shapes (hull, box, sphere, capsule) ignores it.
void Box3DPhysicsDirectSpaceState3D::set_shape_max_normal_impulse(const RID& p_body, int p_shape_index, float p_impulse) {
	Box3DBodyImpl3D* body = Box3DPhysicsServer3D::get_singleton()->get_body(p_body);
	ERR_FAIL_NULL_MSG(body, "Box3D set_shape_max_normal_impulse: the RID is not a body.");
	body->set_shape_max_normal_impulse(p_shape_index, p_impulse);
}

// The body's shape's limit as set_shape_max_normal_impulse set it (FLT_MAX for none).
float Box3DPhysicsDirectSpaceState3D::get_shape_max_normal_impulse(const RID& p_body, int p_shape_index) {
	Box3DBodyImpl3D* body = Box3DPhysicsServer3D::get_singleton()->get_body(p_body);
	ERR_FAIL_NULL_V_MSG(body, FLT_MAX, "Box3D get_shape_max_normal_impulse: the RID is not a body.");
	return body->get_shape_max_normal_impulse(p_shape_index);
}

// Whether continuous collision passes a shape whose limit gave way this step, so a fast body carries on through what it
// broke (on by default). The mark is per shape: every fast body passes that shape in that step. Between steps only.
void Box3DPhysicsDirectSpaceState3D::set_limit_pass_through(bool p_enabled) {
	ERR_FAIL_NULL(space);
	b3World_EnableLimitPassThrough(space->get_world_id(), p_enabled);
}

// Terra: whether voxel grid against voxel grid contacts are point-sampled against occupancy (the project setting
// physics/box3d/voxel_point_contacts gives the start value). Between steps only.
void Box3DPhysicsDirectSpaceState3D::set_voxel_point_contacts(bool p_enabled) {
	ERR_FAIL_NULL(space);
	b3World_EnableVoxelPointContacts(space->get_world_id(), p_enabled);
}

bool Box3DPhysicsDirectSpaceState3D::is_voxel_point_contacts() {
	ERR_FAIL_NULL_V(space, false);
	return b3World_IsVoxelPointContactsEnabled(space->get_world_id());
}

// What each body touching the body's shape (its Godot shape index) took from it over the last step: one Dictionary per
// touching shape pair that applied any normal impulse, with collider (RID), collider_id (the collider's instance id),
// collider_shape (its Godot shape index), impulse (the net normal impulse in N*s over the step, sub-steps and
// restitution included: under a limit at most the limit, and the limit itself when the contact spent it), point (the
// impulse-weighted contact point in world space) and normal (the unit direction the shape pushed the collider). Read
// between steps: after the step the limits were set for, before the next.
Array Box3DPhysicsDirectSpaceState3D::shape_contact_impulses(const RID& p_body, int p_shape_index) {
	Array result;
	Box3DBodyImpl3D* body = Box3DPhysicsServer3D::get_singleton()->get_body(p_body);
	ERR_FAIL_NULL_V_MSG(body, result, "Box3D shape_contact_impulses: the RID is not a body.");
	ERR_FAIL_INDEX_V(p_shape_index, body->get_shape_count(), result);
	if (!body->has_body_id() || !body->has_shape_id(p_shape_index)) {
		return result;
	}

	const b3ShapeId shape_id = body->get_shape_id(p_shape_index);
	const int capacity = b3Shape_GetContactCapacity(shape_id);
	if (capacity <= 0) {
		return result;
	}
	LocalVector<b3ContactData> pairs;
	pairs.resize(capacity);
	const int count = b3Shape_GetContactData(shape_id, pairs.ptr(), capacity);
	const b3Vec3 center = b3Body_GetWorldCenter(body->get_body_id());

	for (int i = 0; i < count; i++) {
		const b3ContactData& pair = pairs[i];
		const bool self_is_a = B3_ID_EQUALS(pair.shapeIdA, shape_id);
		const b3ShapeId other_shape = self_is_a ? pair.shapeIdB : pair.shapeIdA;
		float impulse = 0.0f;
		b3Vec3 point_sum = b3Vec3_zero;
		b3Vec3 normal_sum = b3Vec3_zero;
		for (int m = 0; m < pair.manifoldCount; m++) {
			const b3Manifold& manifold = pair.manifolds[m];
			// The manifold normal points from A to B: it pushes B along it and A against it.
			const b3Vec3 push = self_is_a ? manifold.normal : b3Neg(manifold.normal);
			for (int p = 0; p < manifold.pointCount; p++) {
				const b3ManifoldPoint& point = manifold.points[p];
				const float j = point.appliedNormalImpulse;
				if (j <= 0.0f) {
					continue;
				}
				impulse += j;
				point_sum = b3MulAdd(point_sum, j, b3Add(center, self_is_a ? point.anchorA : point.anchorB));
				normal_sum = b3MulAdd(normal_sum, j, push);
			}
		}
		if (impulse <= 0.0f) {
			continue;
		}

		const b3BodyId other_id = b3Shape_GetBody(other_shape);
		// Areas share the userData slot as a sibling class, so a static_cast alone would yield garbage.
		const auto* other = dynamic_cast<const Box3DBodyImpl3D*>(static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(other_id)));
		Dictionary entry;
		entry["collider"] = other != nullptr ? other->get_rid() : RID();
		entry["collider_id"] = other != nullptr ? (int64_t)other->get_instance_id() : (int64_t)0;
		entry["collider_shape"] = other != nullptr ? other->find_shape_index(other_shape) : -1;
		entry["impulse"] = impulse;
		entry["point"] = b3_to_godot(b3MulSV(1.0f / impulse, point_sum));
		entry["normal"] = b3_to_godot(b3Normalize(normal_sum));
		result.push_back(entry);
	}
	return result;
}

int Box3DPhysicsDirectSpaceState3D::box3d_byte_count() {
	return b3GetByteCount();
}

Dictionary Box3DPhysicsDirectSpaceState3D::world_counters() {
	Dictionary result;
	ERR_FAIL_NULL_V(space, result);
	const b3Counters c = b3World_GetCounters(space->get_world_id());
	result["physics_step_ms"] = b3World_GetProfile(space->get_world_id()).step;
	result["body_count"] = c.bodyCount;
	result["shape_count"] = c.shapeCount;
	result["contact_count"] = c.contactCount;
	result["joint_count"] = c.jointCount;
	result["island_count"] = c.islandCount;
	result["stack_used"] = c.stackUsed;
	result["arena_capacity"] = c.arenaCapacity;
	result["static_tree_height"] = c.staticTreeHeight;
	result["tree_height"] = c.treeHeight;
	result["sat_call_count"] = c.satCallCount;
	result["sat_cache_hit_count"] = c.satCacheHitCount;
	result["voxel_grid_pair_count"] = c.voxelGridPairCount;
	result["voxel_sample_count"] = c.voxelSampleCount;
	result["voxel_point_count"] = c.voxelPointCount;
	result["voxel_min_separation"] = c.voxelMinSeparation;
	result["byte_count"] = c.byteCount;
	result["task_count"] = c.taskCount;
	PackedInt32Array color_counts;
	for (int i = 0; i < 24; i++) {
		color_counts.push_back(c.colorCounts[i]);
	}
	result["color_counts"] = color_counts;
	PackedInt32Array manifold_counts;
	for (int i = 0; i < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; i++) {
		manifold_counts.push_back(c.manifoldCounts[i]);
	}
	result["manifold_counts"] = manifold_counts;
	result["awake_contact_count"] = c.awakeContactCount;
	result["recycled_contact_count"] = c.recycledContactCount;
	result["distance_iterations"] = c.distanceIterations;
	result["push_back_iterations"] = c.pushBackIterations;
	result["root_iterations"] = c.rootIterations;
	return result;
}

bool Box3DPhysicsDirectSpaceState3D::test_body_motion(
		Box3DShapedObjectImpl3D& p_body,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		int32_t p_max_collisions,
		bool p_recovery_as_collision,
		PhysicsServer3DExtensionMotionResult* p_result) const {
	ERR_FAIL_NULL_V(space, false);
	ERR_FAIL_NULL_V(p_result, false);

	p_result->travel = Vector3();
	p_result->remainder = p_motion;
	p_result->collision_depth = 0.0f;
	p_result->collision_safe_fraction = 1.0f;
	p_result->collision_unsafe_fraction = 1.0f;
	p_result->collision_count = 0;

	Box3DQueryFilter3D filter;
	filter.set_collision_mask(p_body.get_collision_mask());
	filter.exclude.insert(p_body.get_rid());
	if (auto* body = dynamic_cast<Box3DBodyImpl3D*>(&p_body)) {
		for (const KeyValue<RID, Box3DFilterJointImpl3D*>& entry : body->get_collision_exceptions()) {
			filter.exclude.insert(entry.key);
		}
	}

	// The body's convex pieces at the start pose; local_shapes maps each back to the body's shape index.
	std::vector<b3m::BodyShape> body_shapes;
	std::vector<int32_t> local_shapes;
	for (int32_t i = 0; i < p_body.get_shape_count(); i++) {
		if (p_body.is_shape_disabled(i)) {
			continue;
		}
		Box3DShapeImpl3D* shape = p_body.get_shape(i);
		if (shape == nullptr) {
			continue;
		}
		const Box3DShapeProxy3D shape_proxy(shape, p_transform * p_body.get_shape_transform(i));
		if (!shape_proxy.is_supported()) {
			continue;
		}
		const b3ShapeProxy& proxy = shape_proxy.get_proxy();
		b3m::BodyShape body_shape;
		body_shape.points.assign(proxy.points, proxy.points + proxy.count);
		body_shape.radius = proxy.radius;
		body_shapes.push_back(std::move(body_shape));
		local_shapes.push_back(i);
	}
	if (body_shapes.empty()) {
		p_result->travel = p_motion;
		p_result->remainder = Vector3();
		return false;
	}

	b3m::Params params;
	params.world = space->get_world_id();
	params.filter = filter.filter;
	params.accept = motion_accept;
	params.accept_context = &filter;
	params.margin = (float)p_margin;
	params.motion = godot_to_b3(p_motion);
	params.max_collisions = MAX(0, p_max_collisions);
	params.recovery_as_collision = p_recovery_as_collision;

	b3m::Result result;
	const bool collided = b3m::test_motion(body_shapes, params, result);

	p_result->travel = b3_to_godot(result.travel);
	p_result->remainder = b3_to_godot(result.remainder);
	p_result->collision_safe_fraction = result.safe_fraction;
	p_result->collision_unsafe_fraction = result.unsafe_fraction;
	p_result->collision_depth = result.depth;

	int32_t count = 0;
	for (int32_t i = 0; i < result.collision_count; i++) {
		const b3m::Collision& collision = result.collisions[i];
		auto* object = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(b3Shape_GetBody(collision.shape)));
		if (object == nullptr) {
			continue;
		}
		fill_motion_collision(collision, *object, local_shapes[collision.local_shape], p_result->collisions[count++]);
	}
	p_result->collision_count = count;
	return collided;
}

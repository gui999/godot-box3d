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

#include <godot_cpp/core/object.hpp>
#include <godot_cpp/templates/local_vector.hpp>

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

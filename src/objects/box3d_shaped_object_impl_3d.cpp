#include "box3d_shaped_object_impl_3d.hpp"

#include "../misc/type_conversions.hpp"
#include "../shapes/box3d_box_shape_impl_3d.hpp"
#include "../shapes/box3d_capsule_shape_impl_3d.hpp"
#include "../shapes/box3d_concave_polygon_shape_impl_3d.hpp"
#include "../shapes/box3d_convex_polygon_shape_impl_3d.hpp"
#include "../shapes/box3d_cylinder_shape_impl_3d.hpp"
#include "../shapes/box3d_heightmap_shape_impl_3d.hpp"
#include "../shapes/box3d_shape_impl_3d.hpp"
#include "../shapes/box3d_sphere_shape_impl_3d.hpp"
#include "../shapes/box3d_voxel_grid_shape_impl_3d.hpp"
#include "../shapes/box3d_world_boundary_shape_impl_3d.hpp"
#include "../spaces/box3d_space_3d.hpp"

#include <box3d/box3d.h>
#include <box3d/collision.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace {

// The shape definition every attached shape starts from. No shape carries b3 userData except a voxel
// grid's cell shapes (see Box3DVoxelGridInstance3D): an instance lives in a LocalVector that can
// reallocate, so a pointer to it must never be stored.
b3ShapeDef make_shape_def(
		uint32_t p_layer,
		uint32_t p_mask,
		bool p_is_sensor,
		bool p_is_concave,
		void* p_user_data,
		float p_friction,
		float p_restitution,
		bool p_is_static) {
	b3ShapeDef def = b3DefaultShapeDef();
	def.userData = p_user_data;
	def.filter = godot_to_b3_filter(p_layer, p_mask);
	def.isSensor = p_is_sensor;
	// A concave shape may be a sensor, but never a sensor visitor (Box3D can't proxy it).
	def.enableSensorEvents = p_is_sensor || !p_is_concave;
	def.enableContactEvents = !p_is_sensor;
	def.density = 1.0f;
	// Terra: the body applies its mass once before the next step (Box3DBodyImpl3D::flush_mass_data).
	def.updateBodyMass = false;
	def.baseMaterial = b3DefaultSurfaceMaterial();
	def.baseMaterial.friction = p_friction;
	def.baseMaterial.restitution = p_restitution;
	// A static shape need not scan the broad phase when it is created: dynamic bodies find it
	// when they move, which makes creating many static shapes cheap.
	def.invokeContactCreation = !p_is_static;
	return def;
}

// Builds and attaches a live b3ShapeId for the given shape instance, dispatching by
// concrete Box3DShapeImpl3D subtype. p_is_sensor marks every shape on an area body as a
// sensor (per the plan's area-bridging design). Returns b3_nullShapeId if the shape has no
// buildable geometry yet (e.g. a convex hull with fewer than 4 points).
b3ShapeId create_box3d_shape(
		b3BodyId p_body_id,
		Box3DShapeInstance3D& p_instance,
		uint32_t p_layer,
		uint32_t p_mask,
		bool p_is_sensor,
		float p_friction,
		float p_restitution,
		bool p_is_static) {
	Box3DShapeImpl3D* shape = p_instance.get_shape();
	if (shape == nullptr || p_instance.is_disabled()) {
		return b3_nullShapeId;
	}

	const PhysicsServer3D::ShapeType type = shape->get_type();
	const bool is_concave = (type == PhysicsServer3D::SHAPE_CONCAVE_POLYGON || type == PhysicsServer3D::SHAPE_HEIGHTMAP);

	b3ShapeDef def = make_shape_def(p_layer, p_mask, p_is_sensor, is_concave, nullptr, p_friction, p_restitution, p_is_static);

	const Transform3D& local = p_instance.get_transform();

	switch (type) {
		case PhysicsServer3D::SHAPE_SPHERE: {
			auto* sphere_shape = static_cast<Box3DSphereShapeImpl3D*>(shape);
			b3Sphere sphere;
			sphere.center = godot_to_b3(local.origin);
			sphere.radius = (float)sphere_shape->get_radius();
			return b3CreateSphereShape(p_body_id, &def, &sphere);
		}

		case PhysicsServer3D::SHAPE_CAPSULE: {
			auto* capsule_shape = static_cast<Box3DCapsuleShapeImpl3D*>(shape);
			const float radius = (float)capsule_shape->get_radius();
			const float height = (float)capsule_shape->get_height();
			const float half_seg = MAX(0.0f, height * 0.5f - radius);
			const Vector3 local_c1 = local.xform(Vector3(0, half_seg, 0));
			const Vector3 local_c2 = local.xform(Vector3(0, -half_seg, 0));
			b3Capsule capsule;
			capsule.center1 = godot_to_b3(local_c1);
			capsule.center2 = godot_to_b3(local_c2);
			capsule.radius = radius;
			return b3CreateCapsuleShape(p_body_id, &def, &capsule);
		}

		case PhysicsServer3D::SHAPE_BOX: {
			auto* box_shape = static_cast<Box3DBoxShapeImpl3D*>(shape);
			const Vector3 half = box_shape->get_half_extents();
			const b3Transform box_transform = godot_to_b3_transform(local);
			b3BoxHull box_hull = b3MakeTransformedBoxHull((float)half.x, (float)half.y, (float)half.z, box_transform);
			return b3CreateHullShape(p_body_id, &def, &box_hull.base);
		}

		case PhysicsServer3D::SHAPE_CYLINDER: {
			auto* cylinder_shape = static_cast<Box3DCylinderShapeImpl3D*>(shape);
			const float height = (float)cylinder_shape->get_height();
			b3HullData* cylinder = b3CreateCylinder(
					height,
					(float)cylinder_shape->get_radius(),
					-0.5f * height,
					Box3DCylinderShapeImpl3D::HULL_SIDES);
			ERR_FAIL_NULL_V(cylinder, b3_nullShapeId);
			const b3Transform cylinder_transform = godot_to_b3_transform(local);
			const b3ShapeId shape_id = b3CreateTransformedHullShape(p_body_id, &def, cylinder, cylinder_transform, b3Vec3{1.0f, 1.0f, 1.0f});
			b3DestroyHull(cylinder);
			return shape_id;
		}

		case PhysicsServer3D::SHAPE_CONVEX_POLYGON: {
			auto* convex_shape = static_cast<Box3DConvexPolygonShapeImpl3D*>(shape);
			const b3HullData* hull = convex_shape->get_hull();
			if (hull == nullptr) {
				return b3_nullShapeId;
			}
			if (local.origin == Vector3() && local.basis.is_equal_approx(Basis())) {
				return b3CreateHullShape(p_body_id, &def, hull);
			}
			const b3Transform hull_transform = godot_to_b3_transform(local);
			return b3CreateTransformedHullShape(p_body_id, &def, hull, hull_transform, b3Vec3{1.0f, 1.0f, 1.0f});
		}

		case PhysicsServer3D::SHAPE_CONCAVE_POLYGON: {
			auto* mesh_shape = static_cast<Box3DConcavePolygonShapeImpl3D*>(shape);

			// b3CreateMeshShape takes no transform, so an offset or rotated instance needs
			// its own mesh with the local transform baked into the vertices.
			if (local.origin == Vector3() && local.basis.is_equal_approx(Basis())) {
				const b3MeshData* mesh = mesh_shape->get_mesh();
				if (mesh == nullptr) {
					return b3_nullShapeId;
				}
				return b3CreateMeshShape(p_body_id, &def, mesh, b3Vec3{1.0f, 1.0f, 1.0f});
			}

			b3MeshData* baked = Box3DConcavePolygonShapeImpl3D::build_mesh(mesh_shape->get_faces(), local);
			if (baked == nullptr) {
				return b3_nullShapeId;
			}
			p_instance.set_owned_mesh(baked);
			return b3CreateMeshShape(p_body_id, &def, baked, b3Vec3{1.0f, 1.0f, 1.0f});
		}

		case PhysicsServer3D::SHAPE_HEIGHTMAP: {
			auto* height_shape = static_cast<Box3DHeightMapShapeImpl3D*>(shape);
			const b3HeightFieldData* height_field = height_shape->get_height_field();
			if (height_field == nullptr) {
				return b3_nullShapeId;
			}
			return b3CreateHeightFieldShape(p_body_id, &def, height_field);
		}

		case PhysicsServer3D::SHAPE_WORLD_BOUNDARY: {
			auto* boundary_shape = static_cast<Box3DWorldBoundaryShapeImpl3D*>(shape);
			const Plane plane = boundary_shape->get_plane();
			const Vector3 normal = plane.normal.normalized();
			// Build a box hull plate centered `distance` along `normal` from the origin,
			// oriented so its local +Y faces along the plane normal.
			const Vector3 up(0, 1, 0);
			Basis basis;
			if (Math::abs(normal.dot(up)) > 0.999f) {
				basis = Basis::from_euler(Vector3(normal.y < 0 ? Math_PI : 0, 0, 0));
			} else {
				const Vector3 axis = up.cross(normal).normalized();
				const real_t angle = Math::acos(CLAMP(up.dot(normal), -1.0, 1.0));
				basis = Basis(axis, angle);
			}
			const Vector3 origin = normal * (real_t)(plane.d - Box3DWorldBoundaryShapeImpl3D::PLATE_HALF_THICKNESS);
			const Transform3D plate_transform(basis, origin);
			const Transform3D combined = local * plate_transform;
			const b3Transform box_transform = godot_to_b3_transform(combined);
			b3BoxHull box_hull = b3MakeTransformedBoxHull(
					Box3DWorldBoundaryShapeImpl3D::PLATE_HALF_EXTENT,
					Box3DWorldBoundaryShapeImpl3D::PLATE_HALF_THICKNESS,
					Box3DWorldBoundaryShapeImpl3D::PLATE_HALF_EXTENT,
					box_transform);
			return b3CreateHullShape(p_body_id, &def, &box_hull.base);
		}

		default: {
			ERR_FAIL_V_MSG(b3_nullShapeId, "Box3D: unsupported shape type used on a body.");
		}
	}
}

} // namespace

Box3DShapedObjectImpl3D::~Box3DShapedObjectImpl3D() {
	_destroy_body_id();
	for (auto& instance : shapes) {
		if (instance.get_shape() != nullptr) {
			instance.get_shape()->remove_owner(this);
		}
	}
}

Transform3D Box3DShapedObjectImpl3D::get_transform() const {
	if (has_body_id()) {
		return b3_to_godot(b3Body_GetTransform(body_id));
	}
	return cached_transform;
}

void Box3DShapedObjectImpl3D::set_transform(const Transform3D& p_transform) {
	cached_transform = p_transform;
	if (has_body_id()) {
		const b3Transform t = godot_to_b3_transform(p_transform);
		b3Body_SetTransform(body_id, t.p, t.q);
	}
}

void Box3DShapedObjectImpl3D::add_shape(Box3DShapeImpl3D* p_shape, const Transform3D& p_transform, bool p_disabled) {
	Box3DShapeInstance3D instance(p_shape, p_transform);
	instance.set_disabled(p_disabled);
	instance.set_index((uint32_t)shapes.size());
	shapes.push_back(instance);

	if (has_body_id()) {
		_create_shape_instance(shapes[shapes.size() - 1]);
		_shapes_changed();
	}
}

void Box3DShapedObjectImpl3D::remove_shape(const Box3DShapeImpl3D* p_shape) {
	for (int32_t i = (int32_t)shapes.size() - 1; i >= 0; i--) {
		if (shapes[i].get_shape() == p_shape) {
			remove_shape(i);
		}
	}
}

void Box3DShapedObjectImpl3D::remove_shape(int32_t p_index) {
	ERR_FAIL_INDEX(p_index, (int32_t)shapes.size());
	Box3DShapeImpl3D* removed = shapes[p_index].get_shape();
	_destroy_shape_instance(shapes[p_index]);
	shapes.remove_at(p_index);
	for (uint32_t i = p_index; i < shapes.size(); i++) {
		shapes[i].set_index(i);
	}
	_release_shape_owner(removed);
	_shapes_changed();
}

void Box3DShapedObjectImpl3D::set_shape(int32_t p_index, Box3DShapeImpl3D* p_shape) {
	ERR_FAIL_INDEX(p_index, (int32_t)shapes.size());
	Box3DShapeImpl3D* replaced = shapes[p_index].get_shape();
	_destroy_shape_instance(shapes[p_index]);
	shapes[p_index].set_shape(p_shape);
	_release_shape_owner(replaced);
	if (has_body_id()) {
		_create_shape_instance(shapes[p_index]);
	}
	_shapes_changed();
}

void Box3DShapedObjectImpl3D::clear_shapes() {
	for (int32_t i = (int32_t)shapes.size() - 1; i >= 0; i--) {
		_destroy_shape_instance(shapes[i]);
		if (shapes[i].get_shape() != nullptr) {
			shapes[i].get_shape()->remove_owner(this);
		}
	}
	shapes.clear();
	_shapes_changed();
}

Box3DShapeImpl3D* Box3DShapedObjectImpl3D::get_shape(int32_t p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int32_t)shapes.size(), nullptr);
	return shapes[p_index].get_shape();
}

Transform3D Box3DShapedObjectImpl3D::get_shape_transform(int32_t p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int32_t)shapes.size(), Transform3D());
	return shapes[p_index].get_transform();
}

b3ShapeId Box3DShapedObjectImpl3D::get_shape_id(int32_t p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int32_t)shapes.size(), b3_nullShapeId);
	return shapes[p_index].get_shape_id();
}

bool Box3DShapedObjectImpl3D::has_shape_id(int32_t p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int32_t)shapes.size(), false);
	return shapes[p_index].has_shape_id();
}

void Box3DShapedObjectImpl3D::set_shape_transform(int32_t p_index, const Transform3D& p_transform) {
	ERR_FAIL_INDEX(p_index, (int32_t)shapes.size());
	Box3DShapeInstance3D& instance = shapes[p_index];
	instance.set_transform(p_transform);
	if (instance.has_shape_id() || instance.get_grid_instance()) {
		_destroy_shape_instance(instance);
		_create_shape_instance(instance);
		_shapes_changed();
	}
}

bool Box3DShapedObjectImpl3D::is_shape_disabled(int32_t p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int32_t)shapes.size(), false);
	return shapes[p_index].is_disabled();
}

void Box3DShapedObjectImpl3D::set_shape_disabled(int32_t p_index, bool p_disabled) {
	ERR_FAIL_INDEX(p_index, (int32_t)shapes.size());
	Box3DShapeInstance3D& instance = shapes[p_index];
	if (instance.is_disabled() == p_disabled) {
		return;
	}
	instance.set_disabled(p_disabled);
	if (p_disabled) {
		_destroy_shape_instance(instance);
	} else if (has_body_id()) {
		_create_shape_instance(instance);
	}
	_shapes_changed();
}

void Box3DShapedObjectImpl3D::set_space(Box3DSpace3D* p_space) {
	if (space == p_space) {
		return;
	}

	if (space != nullptr) {
		cached_transform = get_transform();
		for (auto& instance : shapes) {
			_destroy_shape_instance(instance);
		}
		_destroy_body_id();
	}

	Box3DObjectImpl3D::set_space(p_space);

	if (space != nullptr) {
		body_id = _create_body_id(space->get_world_id());
		rebuild_shapes();
	}
}

void Box3DShapedObjectImpl3D::set_collision_layer(uint32_t p_layer) {
	if (collision_layer == p_layer) {
		return;
	}
	Box3DObjectImpl3D::set_collision_layer(p_layer);
	_refresh_shape_filters();
}

void Box3DShapedObjectImpl3D::set_collision_mask(uint32_t p_mask) {
	if (collision_mask == p_mask) {
		return;
	}
	Box3DObjectImpl3D::set_collision_mask(p_mask);
	_refresh_shape_filters();
}

void Box3DShapedObjectImpl3D::_refresh_shape_filters() {
	if (!has_body_id()) {
		return;
	}
	const b3Filter filter = godot_to_b3_filter(collision_layer, collision_mask);
	for (auto& instance : shapes) {
		if (instance.has_shape_id()) {
			b3Shape_SetFilter(instance.get_shape_id(), filter, true);
		}
		if (instance.get_grid_instance()) {
			for (auto& cell : instance.get_grid_instance()->cell_shapes) {
				for (const b3ShapeId id : cell.second) {
					b3Shape_SetFilter(id, filter, true);
				}
			}
		}
	}
}

void Box3DShapedObjectImpl3D::refresh_shape_materials() {
	if (!has_body_id()) {
		return;
	}
	const float friction = _get_shape_friction();
	const float restitution = _get_shape_restitution();
	for (auto& instance : shapes) {
		if (instance.has_shape_id()) {
			b3Shape_SetFriction(instance.get_shape_id(), friction);
			b3Shape_SetRestitution(instance.get_shape_id(), restitution);
		}
		if (instance.get_grid_instance()) {
			for (auto& cell : instance.get_grid_instance()->cell_shapes) {
				for (const b3ShapeId id : cell.second) {
					b3Shape_SetFriction(id, friction);
					b3Shape_SetRestitution(id, restitution);
				}
			}
		}
	}
}

void Box3DShapedObjectImpl3D::rebuild_shapes() {
	if (!has_body_id()) {
		return;
	}
	for (auto& instance : shapes) {
		if (!instance.is_disabled()) {
			_create_shape_instance(instance);
		}
	}
	_shapes_changed();
}

void Box3DShapedObjectImpl3D::_destroy_body_id() {
	if (has_body_id()) {
		for (auto& instance : shapes) {
			instance.set_shape_id(b3_nullShapeId);
			if (instance.get_grid_instance()) {
				// The body takes its shapes with it.
				for (auto& cell : instance.get_grid_instance()->cell_shapes) {
					Box3DVoxelGridShapeImpl3D::add_live_shapes(-(int64_t)cell.second.size());
				}
				instance.set_grid_instance(nullptr);
			}
		}
		b3DestroyBody(body_id);
		body_id = b3_nullBodyId;
	}
}

void Box3DShapedObjectImpl3D::_create_shape_instance(Box3DShapeInstance3D& p_instance) {
	if (p_instance.has_shape_id() || p_instance.get_grid_instance() || !has_body_id()) {
		return;
	}
	const bool is_static = b3Body_GetType(body_id) == b3_staticBody;
	// The voxel grid is the only custom shape.
	if (p_instance.get_shape() != nullptr && !p_instance.is_disabled() && p_instance.get_shape()->get_type() == PhysicsServer3D::SHAPE_CUSTOM) {
		ERR_FAIL_COND_MSG(!is_static || _is_sensor_body(), "Box3D: a voxel grid shape can only be added to a static body.");
		_create_grid_instance(p_instance);
		return;
	}
	const b3ShapeId shape_id = create_box3d_shape(body_id, p_instance, collision_layer, collision_mask, _is_sensor_body(), _get_shape_friction(), _get_shape_restitution(), is_static);
	p_instance.set_shape_id(shape_id);
}

void Box3DShapedObjectImpl3D::_destroy_shape_instance(Box3DShapeInstance3D& p_instance) {
	if (p_instance.has_shape_id()) {
		b3DestroyShape(p_instance.get_shape_id(), true);
		p_instance.set_shape_id(b3_nullShapeId);
	}
	_destroy_grid_instance(p_instance);
	// Box3D keeps a pointer to mesh data, so free the baked copy only after the shape.
	if (p_instance.get_owned_mesh() != nullptr) {
		b3DestroyMesh(p_instance.get_owned_mesh());
		p_instance.set_owned_mesh(nullptr);
	}
}

void Box3DShapedObjectImpl3D::_release_shape_owner(Box3DShapeImpl3D* p_shape) {
	if (p_shape == nullptr) {
		return;
	}
	for (const auto& instance : shapes) {
		if (instance.get_shape() == p_shape) {
			return;
		}
	}
	p_shape->remove_owner(this);
}

int32_t Box3DShapedObjectImpl3D::find_shape_index(b3ShapeId p_shape_id) const {
	for (int32_t i = 0; i < (int32_t)shapes.size(); i++) {
		if (shapes[i].has_shape_id() && B3_ID_EQUALS(shapes[i].get_shape_id(), p_shape_id)) {
			return i;
		}
	}
	// Only a voxel grid's cell shapes carry userData: the stable state of the attachment they belong to.
	if (b3Shape_IsValid(p_shape_id)) {
		const auto* grid_instance = static_cast<const Box3DVoxelGridInstance3D*>(b3Shape_GetUserData(p_shape_id));
		if (grid_instance != nullptr && grid_instance->owner == this) {
			return (int32_t)grid_instance->index;
		}
	}
	return -1;
}

void Box3DShapedObjectImpl3D::detach_shape(Box3DShapeImpl3D* p_shape) {
	for (auto& instance : shapes) {
		if (instance.get_shape() == p_shape) {
			_destroy_shape_instance(instance);
			instance.set_shape(nullptr);
		}
	}
	p_shape->remove_owner(this);
	_shapes_changed();
}

void Box3DShapedObjectImpl3D::_create_grid_instance(Box3DShapeInstance3D& p_instance) {
	auto state = std::make_shared<Box3DVoxelGridInstance3D>();
	state->owner = this;
	state->index = p_instance.get_index();
	p_instance.set_grid_instance(state);
	const auto* grid = static_cast<const Box3DVoxelGridShapeImpl3D*>(p_instance.get_shape());
	const Box3DVoxelGridData& data = grid->get_grid();
	if (!data.is_valid()) {
		return; // The grid's data arrives later; set_data updates this attachment then.
	}
	// Nothing needs waking: the bodies in the space find the new static shapes as they move.
	for (int32_t cell = 0; cell < data.get_padded_cell_count(); cell++) {
		if (data.get_module_at(cell) >= 0 && !data.is_ring_cell(cell)) {
			b3AABB bounds;
			_rebuild_grid_cell(p_instance, *grid, cell, bounds);
		}
	}
}

void Box3DShapedObjectImpl3D::_destroy_grid_instance(Box3DShapeInstance3D& p_instance) {
	const std::shared_ptr<Box3DVoxelGridInstance3D> state = p_instance.get_grid_instance();
	if (!state) {
		return;
	}
	for (auto& cell : state->cell_shapes) {
		for (const b3ShapeId id : cell.second) {
			if (b3Shape_IsValid(id)) {
				b3DestroyShape(id, false);
			}
		}
		Box3DVoxelGridShapeImpl3D::add_live_shapes(-(int64_t)cell.second.size());
	}
	state->cell_shapes.clear();
	p_instance.set_grid_instance(nullptr);
}

bool Box3DShapedObjectImpl3D::_rebuild_grid_cell(Box3DShapeInstance3D& p_instance, const Box3DVoxelGridShapeImpl3D& p_grid, int32_t p_padded_cell, b3AABB& r_bounds) {
	Box3DVoxelGridInstance3D& state = *p_instance.get_grid_instance();
	bool touched = false;
	auto include = [&](const b3AABB& p_aabb) {
		r_bounds = touched ? b3AABB_Union(r_bounds, p_aabb) : p_aabb;
		touched = true;
	};

	auto existing = state.cell_shapes.find(p_padded_cell);
	if (existing != state.cell_shapes.end()) {
		for (const b3ShapeId id : existing->second) {
			if (b3Shape_IsValid(id)) {
				include(b3Shape_GetAABB(id));
				// A static body has no mass to update; destroying a shape wakes what touched it.
				b3DestroyShape(id, false);
			}
		}
		Box3DVoxelGridShapeImpl3D::add_live_shapes(-(int64_t)existing->second.size());
		state.cell_shapes.erase(existing);
	}

	const Box3DVoxelGridData& data = p_grid.get_grid();
	if (data.is_ring_cell(p_padded_cell)) {
		return touched;
	}
	const int32_t module = data.get_module_at(p_padded_cell);
	int box_count = 0;
	const float* boxes = data.get_module_boxes(module, box_count);
	if (box_count <= 0) {
		return touched;
	}

	int i, j, k;
	data.get_padded_coordinates(p_padded_cell, i, j, k);
	const Vector3 corner = data.get_cell_corner(i, j, k);
	const Transform3D& local = p_instance.get_transform();
	const Quaternion rotation = local.basis.get_rotation_quaternion();
	b3Transform box_transform;
	box_transform.q = godot_to_b3(rotation);

	const b3ShapeDef def = make_shape_def(collision_layer, collision_mask, false, false, &state, _get_shape_friction(), _get_shape_restitution(), true);
	std::vector<b3ShapeId>& created = state.cell_shapes[p_padded_cell];
	created.reserve((size_t)box_count);
	for (int b = 0; b < box_count; b++) {
		const float* box = boxes + (size_t)b * 6;
		const Vector3 half((box[3] - box[0]) * 0.5f, (box[4] - box[1]) * 0.5f, (box[5] - box[2]) * 0.5f);
		if (half.x <= 1e-6f || half.y <= 1e-6f || half.z <= 1e-6f) {
			continue;
		}
		const Vector3 center = corner + Vector3(box[0] + half.x, box[1] + half.y, box[2] + half.z);
		box_transform.p = godot_to_b3(local.xform(center));
		b3BoxHull hull = b3MakeTransformedBoxHull(half.x, half.y, half.z, box_transform);
		const b3ShapeId id = b3CreateHullShape(body_id, &def, &hull.base);
		if (B3_IS_NULL(id)) {
			continue;
		}
		created.push_back(id);
		include(b3Shape_GetAABB(id));
	}
	Box3DVoxelGridShapeImpl3D::add_live_shapes((int64_t)created.size());
	if (created.empty()) {
		state.cell_shapes.erase(p_padded_cell);
	}
	return touched;
}

namespace {

bool collect_overlapping_shape(b3ShapeId p_shape_id, void* p_context) {
	static_cast<std::vector<b3BodyId>*>(p_context)->push_back(b3Shape_GetBody(p_shape_id));
	return true;
}

} // namespace

void Box3DShapedObjectImpl3D::_wake_bodies_in(const b3AABB& p_bounds) {
	if (space == nullptr) {
		return;
	}
	std::vector<b3BodyId> bodies;
	b3World_OverlapAABB(space->get_world_id(), p_bounds, b3DefaultQueryFilter(), collect_overlapping_shape, &bodies);
	for (const b3BodyId id : bodies) {
		if (b3Body_IsValid(id) && b3Body_GetType(id) != b3_staticBody) {
			b3Body_SetAwake(id, true);
		}
	}
}

void Box3DShapedObjectImpl3D::update_voxel_grid(const Box3DVoxelGridShapeImpl3D* p_grid, const std::vector<int32_t>* p_changed_cells) {
	if (!has_body_id()) {
		return;
	}
	for (auto& instance : shapes) {
		if (instance.get_shape() != p_grid || !instance.get_grid_instance()) {
			continue;
		}
		if (p_changed_cells == nullptr) {
			// New grid data: the attachment starts over.
			_destroy_grid_instance(instance);
			_create_grid_instance(instance);
			continue;
		}
		for (const int32_t cell : *p_changed_cells) {
			b3AABB bounds;
			if (_rebuild_grid_cell(instance, *p_grid, cell, bounds)) {
				_wake_bodies_in(bounds);
			}
		}
	}
}

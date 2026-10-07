#pragma once

#include <box3d/box3d.h>
#include <box3d/collision.h>
#include <box3d/types.h>

#include <vector>

// The core of Godot's body_test_motion on Box3D, free of Godot types so a Box3D-only harness can drive it
// (tests/motion). Contract (Jolt's, which the game was tuned on):
//   1. recover: up to a few iterations, each gathering real contacts (normal, signed separation) of every body shape
//      within the margin and pushing the body out along the contact normals to three quarters of the margin;
//   2. cast: sweep the body along the motion by conservative advancement on exact contact separations (never Box3D's shape
//      cast, which reports contact early); the safe fraction is the last pose a skin short of the first surface that closes in
//      on the body (a margin from what it runs into head on, so bodies rest a margin above a floor), the unsafe fraction the
//      first exact overlap after it, both to well under a millimetre;
//   3. collide: at the unsafe pose gather the contacts within the margin that oppose the motion, deepest first.
// Contact normals point from the collider to the body. A collision's depth is the margin plus the penetration.
namespace b3m {

// One convex piece of the body in world space at the start pose: a point cloud wrapped in a radius.
struct BodyShape {
	std::vector<b3Vec3> points;
	float radius = 0.0f;
};

// Return true to let the shape take part. The same function filters overlaps, casts and contacts.
typedef bool AcceptFcn(b3ShapeId p_shape_id, void* p_context);

struct Params {
	b3WorldId world = b3_nullWorldId;
	b3QueryFilter filter{};
	AcceptFcn* accept = nullptr;
	void* accept_context = nullptr;
	float margin = 0.001f;
	b3Vec3 motion{};
	int max_collisions = 1;
	bool recovery_as_collision = false;
	int recovery_iterations = 4;
};

// A contact between one body shape and one collider shape. separation is the signed gap between the two surfaces
// (negative when they overlap).
struct Contact {
	b3ShapeId shape = b3_nullShapeId;
	b3Vec3 normal{};
	b3Vec3 point{};
	float separation = 0.0f;
	int local_shape = -1;
};

constexpr int MAX_COLLISIONS = 32;

struct Collision {
	b3ShapeId shape = b3_nullShapeId;
	b3Vec3 normal{};
	b3Vec3 point{};
	float depth = 0.0f;
	int local_shape = -1;
};

struct Result {
	b3Vec3 recovery{};
	b3Vec3 travel{};
	b3Vec3 remainder{};
	float safe_fraction = 1.0f;
	float unsafe_fraction = 1.0f;
	float depth = 0.0f;
	bool recovered = false;
	int collision_count = 0;
	Collision collisions[MAX_COLLISIONS];
};

// Contacts of one proxy (already in world space) with the world within the margin. Appends to r_contacts.
void collect_contacts(const Params& p_params, const b3ShapeProxy& p_proxy, int p_local_shape, std::vector<Contact>& r_contacts);

// Convex against convex: the signed separation of the two rounded point clouds and the normal from A to B, with a
// witness point on A's surface. Penetrating cores are resolved by EPA. Returns false when the shapes are farther
// apart than p_margin.
bool convex_contact(const b3ShapeProxy& p_a, const b3ShapeProxy& p_b, float p_margin, b3Vec3& r_normal_a_to_b, float& r_separation, b3Vec3& r_point_on_a);

// The shape's contacts with one collider shape within p_params.margin (exact: GJK/EPA for convex shapes, per triangle for meshes,
// per box for voxel grids), appended to r_contacts. Returns false for a collider type with no contact answer (height fields,
// compounds), which leaves the caller to its own test. The accept function is applied. p_any_only: the caller only asks whether
// there is a contact, so the answer may hold just one (a mesh or grid stops at its first).
bool shape_contacts(const Params& p_params, const b3ShapeProxy& p_proxy, int p_local_shape, b3ShapeId p_shape_id, std::vector<Contact>& r_contacts, bool p_any_only = false);

// Whether the shapes (moved by p_offset) come closer than p_params.margin to anything; margin 0 means the surfaces really cross.
bool overlaps_any(const std::vector<BodyShape>& p_shapes, const Params& p_params, b3Vec3 p_offset);

// Godot's cast_motion: the shapes at their given pose swept along p_params.motion, p_params.margin as the clearance kept. The
// safe fraction is the last pose farther than the margin from everything (a body already within it is ignored), the unsafe
// one the first within it, both to about a millimetre. Returns whether anything was hit; [1, 1] otherwise.
bool cast_shapes(const std::vector<BodyShape>& p_shapes, const Params& p_params, float& r_safe, float& r_unsafe);

// The whole motion test. Returns whether the body collided (Godot's body_test_motion result).
bool test_motion(const std::vector<BodyShape>& p_shapes, const Params& p_params, Result& r_result);

} // namespace b3m

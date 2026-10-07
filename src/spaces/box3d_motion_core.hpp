#pragma once

#include <box3d/box3d.h>
#include <box3d/collision.h>
#include <box3d/types.h>

#include <vector>

// The core of Godot's body_test_motion on Box3D, free of Godot types so a Box3D-only harness can drive it
// (tests/motion). Contract (Jolt's, which the game was tuned on):
//   1. recover: up to a few iterations, each gathering real contacts (normal, signed separation) of every body shape
//      within the margin and pushing the body out along the contact normals to half the margin;
//   2. cast: sweep the body along the motion; the safe fraction is the last pose that does not overlap, the unsafe
//      fraction the first that does (a bisection on exact overlaps, so both are millimetre exact);
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

// The whole motion test. Returns whether the body collided (Godot's body_test_motion result).
bool test_motion(const std::vector<BodyShape>& p_shapes, const Params& p_params, Result& r_result);

} // namespace b3m

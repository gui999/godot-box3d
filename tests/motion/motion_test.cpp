// Native proof of the body motion core: a capsule driven by a port of Godot's CharacterBody3D::move_and_slide (grounded
// mode, stop on slope, block on wall, floor snap) over ramps made of a turned box, a convex wedge, a triangle mesh and a
// voxel grid, on Box3D worlds. The Godot twin is Tests/RampWalkTests.cs.
#include "box3d_motion_core.hpp"

#include <box3d/box3d.h>
#include <box3d/collision.h>
#include <box3d/constants.h>
#include <box3d/math_functions.h>

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "stall_grid.inc"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(condition, ...)                                   \
	do {                                                        \
		g_checks++;                                             \
		if (!(condition)) {                                     \
			g_failures++;                                       \
			std::printf("  FAIL %s:%d  ", __FILE__, __LINE__);  \
			std::printf(__VA_ARGS__);                           \
			std::printf("\n");                                  \
		}                                                       \
	} while (0)

// --- a small vector type with Godot's vocabulary ----------------------------------------------------------------------

struct V3 {
	float x = 0, y = 0, z = 0;
	V3() = default;
	V3(float p_x, float p_y, float p_z) : x(p_x), y(p_y), z(p_z) {}
	V3(const b3Vec3& p_v) : x(p_v.x), y(p_v.y), z(p_v.z) {}
	operator b3Vec3() const { return b3Vec3{ x, y, z }; }
	V3 operator+(const V3& o) const { return V3(x + o.x, y + o.y, z + o.z); }
	V3 operator-(const V3& o) const { return V3(x - o.x, y - o.y, z - o.z); }
	V3 operator*(float s) const { return V3(x * s, y * s, z * s); }
	V3 operator-() const { return V3(-x, -y, -z); }
	V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
	V3& operator-=(const V3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
	float dot(const V3& o) const { return x * o.x + y * o.y + z * o.z; }
	V3 cross(const V3& o) const { return V3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); }
	float length() const { return std::sqrt(dot(*this)); }
	V3 normalized() const { const float l = length(); return l > 1e-20f ? *this * (1.0f / l) : V3(); }
	V3 slide(const V3& n) const { return *this - n * dot(n); }
	bool is_zero_approx() const { return std::fabs(x) < 1e-5f && std::fabs(y) < 1e-5f && std::fabs(z) < 1e-5f; }
	bool is_zero() const { return x == 0 && y == 0 && z == 0; }
};

static float deg(float p_degrees) { return p_degrees * 0.017453292519943295f; }
static const V3 UP(0, 1, 0);

// --- levels -----------------------------------------------------------------------------------------------------------

struct Level {
	b3WorldId world;
	std::vector<b3MeshData*> meshes;
	std::vector<b3HullData*> hulls;
	b3VoxelGridModule* module_slab = nullptr;
	b3VoxelGrid* grid = nullptr;

	Level() {
		b3WorldDef def = b3DefaultWorldDef();
		world = b3CreateWorld(&def);
	}
	~Level() {
		b3DestroyWorld(world);
		for (b3MeshData* m : meshes) b3DestroyMesh(m);
		for (b3HullData* h : hulls) b3DestroyHull(h);
		if (grid) b3ReleaseVoxelGrid(grid);
		if (module_slab) b3ReleaseVoxelGridModule(module_slab);
		for (b3VoxelGridModule* m : stall_modules) b3ReleaseVoxelGridModule(m);
	}

	b3BodyId static_body() {
		b3BodyDef def = b3DefaultBodyDef();
		return b3CreateBody(world, &def);
	}

	void add_box(V3 p_center, V3 p_half, float p_roll_degrees = 0.0f) {
		b3Transform xf;
		xf.p = p_center;
		xf.q = b3MakeQuatFromAxisAngle(b3Vec3{ 0, 0, 1 }, deg(p_roll_degrees));
		b3BoxHull box = b3MakeTransformedBoxHull(p_half.x, p_half.y, p_half.z, xf);
		b3ShapeDef shape = b3DefaultShapeDef();
		b3CreateHullShape(static_body(), &shape, &box.base);
	}

	void add_wedge(float p_run, float p_rise, float p_half_width) {
		b3Vec3 points[6] = { { 0, 0, -p_half_width }, { 0, 0, p_half_width }, { p_run, 0, -p_half_width }, { p_run, 0, p_half_width }, { p_run, p_rise, -p_half_width }, { p_run, p_rise, p_half_width } };
		b3HullData* hull = b3CreateHull(points, 6, 64);
		hulls.push_back(hull);
		b3ShapeDef shape = b3DefaultShapeDef();
		b3CreateHullShape(static_body(), &shape, hull);
	}

	void add_mesh_ramp(float p_run, float p_rise, float p_half_width) {
		b3Vec3 vertices[4] = { { 0, 0, -p_half_width }, { p_run, p_rise, -p_half_width }, { p_run, p_rise, p_half_width }, { 0, 0, p_half_width } };
		int32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
		b3MeshDef def = {};
		def.vertices = vertices;
		def.indices = indices;
		def.vertexCount = 4;
		def.triangleCount = 2;
		def.doubleSided = true;
		b3MeshData* mesh = b3CreateMesh(&def, nullptr, 0);
		meshes.push_back(mesh);
		b3ShapeDef shape = b3DefaultShapeDef();
		b3CreateMeshShape(static_body(), &shape, mesh, b3Vec3{ 1, 1, 1 });
	}

	// A flat landing of voxel slabs: cells x 1 x cells_z cells of 2 m, its top at p_top, its corner at (p_x, ., p_z).
	void add_voxel_floor(float p_x, float p_top, float p_z, int p_cells_x, int p_cells_z) {
		const float cell = 2.0f;
		const float slab[6] = { 0, 0, 0, cell, 0.4f, cell };
		module_slab = b3CreateVoxelGridModule(slab, 1, cell, 20);
		std::vector<int> padded((p_cells_x + 2) * 3 * (p_cells_z + 2), -1);
		for (int k = 0; k < p_cells_z; k++) {
			for (int i = 0; i < p_cells_x; i++) {
				padded[((0 + 1) * (p_cells_z + 2) + (k + 1)) * (p_cells_x + 2) + (i + 1)] = 0;
			}
		}
		b3VoxelGridDef def = {};
		def.cellCountX = p_cells_x;
		def.cellCountY = 1;
		def.cellCountZ = p_cells_z;
		def.origin = b3Vec3{ p_x, p_top - 0.4f, p_z };
		def.cellMeters = cell;
		def.cellVoxels = 20;
		def.maxBoxesPerCell = 256;
		b3VoxelGridModule* modules[1] = { module_slab };
		def.modules = modules;
		def.moduleCount = 1;
		def.paddedCells = padded.data();
		grid = b3CreateVoxelGrid(&def);
		b3ShapeDef shape = b3DefaultShapeDef();
		b3CreateVoxelGridShape(static_body(), &shape, grid);
	}

	// A voxel cylinder of 0.1 m columns (axis vertical through (p_x, ., p_z), 2 m tall, radius p_radius), in one 2 m cell. The
	// columns' footprints are returned in r_columns as x0, z0 (each 0.1 m wide).
	void add_voxel_cylinder(float p_x, float p_z, float p_radius, std::vector<float>& r_columns) {
		const float cell = 2.0f;
		std::vector<float> boxes;
		for (int i = 0; i < 20; i++) {
			for (int k = 0; k < 20; k++) {
				const float dx = (i + 0.5f) * 0.1f - 1.0f;
				const float dz = (k + 0.5f) * 0.1f - 1.0f;
				if (dx * dx + dz * dz <= p_radius * p_radius) {
					const float b[6] = { i * 0.1f, 0.0f, k * 0.1f, (i + 1) * 0.1f, 2.0f, (k + 1) * 0.1f };
					boxes.insert(boxes.end(), b, b + 6);
					r_columns.push_back(p_x - 1.0f + i * 0.1f);
					r_columns.push_back(p_z - 1.0f + k * 0.1f);
				}
			}
		}
		module_slab = b3CreateVoxelGridModule(boxes.data(), (int)boxes.size() / 6, cell, 20);
		std::vector<int> padded(3 * 3 * 3, -1);
		padded[(1 * 3 + 1) * 3 + 1] = 0;
		b3VoxelGridDef def = {};
		def.cellCountX = 1;
		def.cellCountY = 1;
		def.cellCountZ = 1;
		def.origin = b3Vec3{ p_x - 1.0f, 0.0f, p_z - 1.0f };
		def.cellMeters = cell;
		def.cellVoxels = 20;
		def.maxBoxesPerCell = 256;
		b3VoxelGridModule* modules[1] = { module_slab };
		def.modules = modules;
		def.moduleCount = 1;
		def.paddedCells = padded.data();
		grid = b3CreateVoxelGrid(&def);
		b3ShapeDef shape = b3DefaultShapeDef();
		b3CreateVoxelGridShape(static_body(), &shape, grid);
	}

	// The dumped landing of stall_grid.inc: modules of many thin boxes (stairs, posts, rails), cells of 2 m.
	std::vector<b3VoxelGridModule*> stall_modules;
	void add_stall_grid() {
		for (int m = 0; m < STALL_MODULE_COUNT; m++) {
			stall_modules.push_back(b3CreateVoxelGridModule(STALL_BOXES + STALL_OFFSETS[m] * 6, STALL_OFFSETS[m + 1] - STALL_OFFSETS[m], 2.0f, 20));
		}
		b3VoxelGridDef def = {};
		def.cellCountX = STALL_CELLS_X;
		def.cellCountY = STALL_CELLS_Y;
		def.cellCountZ = STALL_CELLS_Z;
		def.origin = b3Vec3{ 0, 4, 0 };
		def.cellMeters = 2.0f;
		def.cellVoxels = 20;
		def.maxBoxesPerCell = 256;
		def.modules = stall_modules.data();
		def.moduleCount = (int)stall_modules.size();
		def.paddedCells = STALL_CELLS;
		grid = b3CreateVoxelGrid(&def);
		b3ShapeDef shape = b3DefaultShapeDef();
		b3CreateVoxelGridShape(static_body(), &shape, grid);
	}
};

enum class Kind { Box, Wedge, Mesh };

static const float RUN = 6.0f;

// RampWalkTests.Level: a floor, a ramp of the given kind rising along +x to its top at x = 6, and a landing.
static void build_level(Level& p_level, Kind p_kind, float p_slope_degrees, bool p_voxel_landing = false) {
	p_level.add_box(V3(0, -0.5f, 0), V3(20, 0.5f, 20));
	const float slope = deg(p_slope_degrees);
	const float rise = RUN * std::tan(slope);
	switch (p_kind) {
		case Kind::Box: {
			const float length = RUN / std::cos(slope);
			p_level.add_box(V3(RUN / 2, rise / 2 - 0.2f / std::cos(slope), 0), V3(length / 2, 0.2f, 2), p_slope_degrees);
		} break;
		case Kind::Wedge:
			p_level.add_wedge(RUN, rise, 2);
			break;
		case Kind::Mesh:
			p_level.add_mesh_ramp(RUN, rise, 2);
			break;
	}
	if (p_voxel_landing) {
		// The voxel slab's top is flush with the ramp's top: the seam sits at x = 6.
		p_level.add_voxel_floor(RUN, rise, -2, 3, 2);
	} else {
		p_level.add_box(V3(RUN + 3, rise - 0.2f, 0), V3(3, 0.2f, 2));
	}
}

// --- the character: Godot's CharacterBody3D, grounded ------------------------------------------------------------------

struct Motion {
	V3 travel, remainder;
	float safe = 1, unsafe = 1, depth = 0;
	int count = 0;
	struct C { V3 normal, point; float depth; } c[6];
};

struct Character {
	Level* level;
	V3 position, velocity;
	float radius = 0.35f, height = 1.8f;
	float margin = 0.001f, floor_snap_length = 0.3f, floor_max_angle = deg(45), wall_min_slide_angle = deg(15);
	int max_slides = 6;
	bool floor_stop_on_slope = true, floor_block_on_wall = true;
	bool on_floor = false, on_wall = false, on_ceiling = false;
	V3 floor_normal, wall_normal, ceiling_normal;
	int motion_calls = 0;

	bool test_motion(V3 p_from, V3 p_motion, int p_max_collisions, bool p_recovery_as_collision, Motion& r) {
		motion_calls++;
		b3m::BodyShape shape;
		const float half = std::fmax(0.0f, height * 0.5f - radius);
		shape.points = { b3Vec3(p_from + V3(0, half, 0)), b3Vec3(p_from + V3(0, -half, 0)) };
		shape.radius = radius;
		b3m::Params params;
		params.world = level->world;
		params.filter = b3DefaultQueryFilter();
		params.margin = margin;
		params.motion = p_motion;
		params.max_collisions = p_max_collisions;
		params.recovery_as_collision = p_recovery_as_collision;
		b3m::Result result;
		const bool collided = b3m::test_motion({ shape }, params, result);
		r.travel = result.travel;
		r.remainder = result.remainder;
		r.safe = result.safe_fraction;
		r.unsafe = result.unsafe_fraction;
		r.depth = result.depth;
		r.count = result.collision_count;
		for (int i = 0; i < result.collision_count && i < 6; i++) {
			r.c[i] = { V3(result.collisions[i].normal), V3(result.collisions[i].point), result.collisions[i].depth };
		}
		return collided;
	}

	// PhysicsBody3D::move_and_collide
	bool move_and_collide(V3 p_from, V3 p_motion, int p_max_collisions, Motion& r, bool p_test_only, bool p_cancel_sliding) {
		const bool colliding = test_motion(p_from, p_motion, p_max_collisions, true, r);
		if (p_cancel_sliding) {
			const float motion_length = p_motion.length();
			float precision = 0.001f;
			if (colliding) {
				precision += motion_length * (r.unsafe - r.safe);
				if (r.c[0].depth > margin + precision) {
					p_cancel_sliding = false;
				}
			}
			if (p_cancel_sliding) {
				V3 motion_normal;
				if (motion_length > 1e-5f) {
					motion_normal = p_motion * (1.0f / motion_length);
				}
				const float projected_length = r.travel.dot(motion_normal);
				const V3 recovery = r.travel - motion_normal * projected_length;
				if (recovery.length() < margin + precision) {
					r.travel = motion_normal * projected_length;
					r.remainder = p_motion - r.travel;
				}
			}
		}
		if (!p_test_only) {
			position = p_from + r.travel;
		}
		return colliding;
	}

	struct State { bool floor = false, wall = false, ceiling = false; };

	void set_collision_direction(const Motion& p_result, State& r_state, bool p_apply_floor = true, bool p_apply_ceiling = true, bool p_apply_wall = true) {
		r_state = State();
		float wall_depth = -1, floor_depth = -1;
		const bool was_on_wall = on_wall;
		const V3 prev_wall_normal = wall_normal;
		int wall_collision_count = 0;
		V3 combined_wall_normal, tmp_wall_col;
		for (int i = p_result.count - 1; i >= 0; i--) {
			const auto& c = p_result.c[i];
			const float floor_angle = std::acos(std::fmax(-1.0f, std::fmin(1.0f, c.normal.dot(UP))));
			if (floor_angle <= floor_max_angle + 0.01f) {
				r_state.floor = true;
				if (p_apply_floor && c.depth > floor_depth) {
					on_floor = true;
					floor_normal = c.normal;
					floor_depth = c.depth;
				}
				continue;
			}
			const float ceiling_angle = std::acos(std::fmax(-1.0f, std::fmin(1.0f, c.normal.dot(-UP))));
			if (ceiling_angle <= floor_max_angle + 0.01f) {
				r_state.ceiling = true;
				if (p_apply_ceiling) {
					ceiling_normal = c.normal;
					on_ceiling = true;
				}
				continue;
			}
			r_state.wall = true;
			if (p_apply_wall && c.depth > wall_depth) {
				on_wall = true;
				wall_depth = c.depth;
				wall_normal = c.normal;
			}
			const V3 d = c.normal - tmp_wall_col;
			if (!(std::fabs(d.x) < 1e-5f && std::fabs(d.y) < 1e-5f && std::fabs(d.z) < 1e-5f)) {
				tmp_wall_col = c.normal;
				combined_wall_normal += c.normal;
				wall_collision_count++;
			}
		}
		if (r_state.wall && wall_collision_count > 1 && !r_state.floor) {
			combined_wall_normal = combined_wall_normal.normalized();
			const float floor_angle = std::acos(std::fmax(-1.0f, std::fmin(1.0f, combined_wall_normal.dot(UP))));
			if (floor_angle <= floor_max_angle + 0.01f) {
				r_state.floor = true;
				r_state.wall = false;
				if (p_apply_floor) {
					on_floor = true;
					floor_normal = combined_wall_normal;
				}
				if (p_apply_wall) {
					on_wall = was_on_wall;
					wall_normal = prev_wall_normal;
				}
			}
		}
	}

	void apply_floor_snap() {
		if (on_floor) {
			return;
		}
		const float length = std::fmax(floor_snap_length, margin);
		Motion r;
		if (move_and_collide(position, -UP * length, 4, r, true, false)) {
			State s;
			set_collision_direction(r, s, true, false, false);
			if (s.floor) {
				if (r.travel.length() > margin) {
					r.travel = UP * UP.dot(r.travel);
				} else {
					r.travel = V3();
				}
				position += r.travel;
			}
		}
	}

	void snap_on_floor(bool p_was_on_floor, bool p_vel_dir_facing_up) {
		if (on_floor || !p_was_on_floor || p_vel_dir_facing_up) {
			return;
		}
		apply_floor_snap();
	}

	void move_and_slide(float p_delta) {
		const bool was_on_floor = on_floor;
		on_floor = on_wall = on_ceiling = false;
		move_and_slide_grounded(p_delta, was_on_floor);
	}

	void move_and_slide_grounded(float p_delta, bool p_was_on_floor) {
		V3 motion = velocity * p_delta;
		const V3 motion_slide_up = motion.slide(UP);
		const V3 prev_floor_normal = floor_normal;
		floor_normal = wall_normal = ceiling_normal = V3();

		bool sliding_enabled = !floor_stop_on_slope;
		bool first_slide = true;
		const bool vel_dir_facing_up = velocity.dot(UP) > 0;
		(void)first_slide;

		for (int iteration = 0; iteration < max_slides; ++iteration) {
			Motion result;
			const bool collided = move_and_collide(position, motion, 6, result, false, !sliding_enabled);

			if (collided) {
				const State previous_state = { on_floor, on_wall, on_ceiling };
				State result_state;
				set_collision_direction(result, result_state);

				if (on_floor && floor_stop_on_slope && (velocity.normalized() + UP).length() < 0.01f) {
					if (result.travel.length() <= margin + 1e-5f) {
						position -= result.travel;
					}
					velocity = V3();
					motion = V3();
					break;
				}

				if (result.remainder.is_zero_approx()) {
					motion = V3();
					break;
				}

				bool apply_default_sliding = true;

				if (result_state.wall && (motion_slide_up.dot(wall_normal) <= 0)) {
					if (floor_block_on_wall) {
						const V3 horizontal_motion = motion.slide(UP);
						const V3 horizontal_normal = wall_normal.slide(UP).normalized();
						const float motion_angle = std::fabs(std::acos(-horizontal_normal.dot(horizontal_motion.normalized())));
						if (motion_angle < 0.5f * 3.14159265f) {
							apply_default_sliding = false;
							if (p_was_on_floor && !vel_dir_facing_up) {
								const float travel_total = result.travel.length();
								const float cancel_dist_max = std::fmin(0.1f, margin * 20);
								if (travel_total <= margin + 1e-5f) {
									position -= result.travel;
									result.travel = V3();
								} else if (travel_total < cancel_dist_max) {
									position -= result.travel.slide(UP);
									motion = motion.slide(UP);
									result.travel = V3();
								} else {
									result.travel = result.travel.slide(UP);
									motion = result.remainder;
								}
								snap_on_floor(true, false);
							} else {
								motion = result.remainder;
							}

							const V3 forward = wall_normal.slide(UP).normalized();
							motion = motion.slide(forward);

							if (vel_dir_facing_up) {
								const V3 slide_motion = velocity.slide(result.c[0].normal);
								velocity = UP * UP.dot(velocity) + slide_motion.slide(UP);
							} else {
								velocity = velocity.slide(forward);
							}

							if (p_was_on_floor && !vel_dir_facing_up && (motion.dot(UP) > 0.0f)) {
								const V3 floor_side = prev_floor_normal.cross(wall_normal);
								if (!floor_side.is_zero()) {
									motion = floor_side * motion.dot(floor_side);
								}
							}

							bool stop_all_motion = previous_state.wall && !vel_dir_facing_up;
							if (!on_floor && motion.dot(UP) < 0) {
								const V3 slide_motion = motion.slide(wall_normal);
								if (slide_motion.dot(UP) < 0) {
									stop_all_motion = false;
									motion = slide_motion;
								}
							}
							if (stop_all_motion) {
								motion = V3();
								velocity = V3();
							}
						}
					}
				}

				if (p_was_on_floor && (wall_min_slide_angle > 0.0f) && result_state.wall) {
					const V3 horizontal_normal = wall_normal.slide(UP).normalized();
					const float motion_angle = std::fabs(std::acos(-horizontal_normal.dot(motion_slide_up.normalized())));
					if (motion_angle < wall_min_slide_angle) {
						motion = UP * motion.dot(UP);
						velocity = UP * velocity.dot(UP);
						apply_default_sliding = false;
					}
				}

				if (apply_default_sliding) {
					if (sliding_enabled || !on_floor) {
						const auto& collision = result.c[0];
						V3 slide_motion = result.remainder.slide(collision.normal);
						if (on_floor && !on_wall && !motion_slide_up.is_zero_approx()) {
							const float motion_length = slide_motion.length();
							slide_motion = UP.cross(result.remainder).cross(floor_normal);
							slide_motion = slide_motion.normalized() * motion_length;
						}
						if (slide_motion.dot(velocity) > 0.0f) {
							motion = slide_motion;
						} else {
							motion = V3();
						}
						if (vel_dir_facing_up) {
							velocity = velocity.slide(collision.normal);
						} else {
							velocity = UP * UP.dot(velocity);
						}
					} else {
						motion = result.remainder;
					}
				}
			}

			if (!collided || motion.is_zero_approx()) {
				break;
			}
			sliding_enabled = true;
			first_slide = false;
		}

		snap_on_floor(p_was_on_floor, vel_dir_facing_up);

		if (on_floor && !vel_dir_facing_up) {
			velocity = velocity.slide(UP);
		}
	}
};

// --- scenarios --------------------------------------------------------------------------------------------------------

static const char* kind_name(Kind p_kind) {
	return p_kind == Kind::Box ? "box" : p_kind == Kind::Wedge ? "wedge" : "mesh";
}

// RampWalkTests.ACharacterWalksUpARamp
static void walk_up(Kind p_kind, bool p_voxel_landing) {
	Level level;
	build_level(level, p_kind, 30.0f, p_voxel_landing);
	Character body;
	body.level = &level;
	body.position = V3(-2, 0.95f, 0);
	int tick = 0;
	float min_y = 1e9f;
	for (; tick < 480 && body.position.x < 8; tick++) {
		const float fall = body.on_floor ? -0.1f : body.velocity.y - 9.8f / 60.0f;
		body.velocity = V3(3, fall, 0);
		body.move_and_slide(1.0f / 60.0f);
		min_y = std::fmin(min_y, body.position.y);
	}
	std::printf("walk up %s ramp%s: x %.2f y %.3f z %.3f after %d ticks (%d motion tests)\n", kind_name(p_kind), p_voxel_landing ? " + voxel landing" : "", body.position.x, body.position.y, body.position.z, tick, body.motion_calls);
	CHECK(body.position.x >= 8, "stalled at %.3f %.3f %.3f", body.position.x, body.position.y, body.position.z);
	CHECK(body.position.y > 3, "not on the landing: y %.3f", body.position.y);
	CHECK(std::fabs(body.position.z) < 0.01f, "drifted sideways: z %.3f", body.position.z);
}

// RampWalkTests.ACharacterStandsStillOnARamp
static void stand_still(Kind p_kind, float p_slope_degrees) {
	Level level;
	build_level(level, p_kind, p_slope_degrees);
	Character body;
	body.level = &level;
	const float y = 3 * std::tan(deg(p_slope_degrees)) + 0.95f;
	body.position = V3(3, y + 0.1f, 0);
	auto tick = [&]() {
		body.velocity = V3(0, body.on_floor ? -0.1f : body.velocity.y - 9.8f / 60.0f, 0);
		body.move_and_slide(1.0f / 60.0f);
	};
	for (int i = 0; i < 30; i++) tick();
	const V3 settled = body.position;
	for (int i = 0; i < 60; i++) tick();
	const float drift = (body.position - settled).length();
	std::printf("stand on %s ramp %.0f deg: settled (%.3f %.3f %.3f), drift %.4f m, on floor %d\n", kind_name(p_kind), p_slope_degrees, settled.x, settled.y, settled.z, drift, body.on_floor);
	CHECK(body.on_floor, "not on the floor");
	CHECK(drift < 0.05f, "drifted %.4f", drift);
	CHECK(std::fabs(body.position.z) < 0.01f, "slid sideways to z %.3f", body.position.z);
}

// A slope steeper than the floor angle is a wall: the character comes down it.
static void slide_down(Kind p_kind) {
	Level level;
	build_level(level, p_kind, 50.0f);
	Character body;
	body.level = &level;
	// Half way up the 50 degree ramp, standing just off its surface.
	const float x = 3.0f;
	body.position = V3(x, x * std::tan(deg(50)) + 1.5f, 0);
	float last_y = body.position.y;
	int tick = 0;
	for (; tick < 300; tick++) {
		body.velocity = V3(0, body.on_floor ? -0.1f : body.velocity.y - 9.8f / 60.0f, 0);
		body.move_and_slide(1.0f / 60.0f);
		last_y = body.position.y;
	}
	std::printf("slide down %s 50 deg: end (%.3f %.3f %.3f) on floor %d wall %d\n", kind_name(p_kind), body.position.x, body.position.y, body.position.z, body.on_floor, body.on_wall);
	CHECK(body.position.x < 0.5f, "stuck on the slope at x %.3f", body.position.x);
	CHECK(last_y < 1.1f, "did not reach the floor: y %.3f", last_y);
	CHECK(std::fabs(body.position.z) < 0.01f, "slid sideways to z %.3f", body.position.z);
}

// Recovery from a 2 cm embedding into a turned box leaves along its face normal.
static void embedded_recovery() {
	Level level;
	build_level(level, Kind::Box, 30.0f);
	// The ramp's top face: a point on it and the unit normal.
	const float slope = deg(30);
	const V3 normal(-std::sin(slope), std::cos(slope), 0);
	const V3 on_face(RUN / 2 - 0.2f * std::sin(slope), RUN / 2 * std::tan(slope) / 1.0f - 0.2f / std::cos(slope) + 0.2f * std::cos(slope), 0);
	// A capsule whose lower sphere sinks 2 cm into the face, over the middle of the ramp.
	const float radius = 0.35f, half = 0.55f;
	const V3 lower_center = on_face + normal * (radius - 0.02f);
	const V3 center = lower_center + V3(0, half, 0);
	Character body;
	body.level = &level;
	body.position = center;
	Motion r;
	body.test_motion(center, V3(), 4, false, r);
	const V3 push = r.travel;
	const float angle = std::acos(std::fmin(1.0f, push.normalized().dot(normal)));
	std::printf("recovery from 2 cm: push (%.4f %.4f %.4f) length %.4f, %.3f deg off the face normal\n", push.x, push.y, push.z, push.length(), angle * 57.29578f);
	CHECK(angle < deg(0.5f), "pushed %.2f deg off the face normal", angle * 57.29578f);
	CHECK(std::fabs(push.length() - 0.0205f) < 0.0006f, "pushed %.4f instead of 0.0205", push.length());
}

// The capsule beside a ramp, not touching, must not be pushed by it (the old AABB recovery shoved it along world axes).
static void no_phantom_push() {
	Level level;
	build_level(level, Kind::Box, 30.0f);
	Character body;
	body.level = &level;
	// On the floor in front of the ramp's foot, 0.3 m clear of the ramp's face.
	const V3 center(-0.9f, 0.9f + 0.001f, 0); // resting a margin above the floor
	Motion r;
	const bool collided = body.test_motion(center, V3(), 4, true, r);
	std::printf("beside the ramp foot: travel (%.5f %.5f %.5f) collided %d\n", r.travel.x, r.travel.y, r.travel.z, collided);
	CHECK(r.travel.length() < 1e-4f, "pushed %.5f", r.travel.length());
}

// Standing at rest on a floor: no recovery, and a downward test reports the floor with a proper depth.
static void rest_on_floor() {
	Level level;
	build_level(level, Kind::Box, 30.0f);
	Character body;
	body.level = &level;
	Motion r;
	body.test_motion(V3(-5, 0.9f + 0.0005f, 0), V3(0, -0.01f, 0), 6, true, r);
	std::printf("rest on floor: safe %.4f unsafe %.4f depth %.5f normal (%.3f %.3f %.3f) count %d\n", r.safe, r.unsafe, r.depth, r.c[0].normal.x, r.c[0].normal.y, r.c[0].normal.z, r.count);
	CHECK(r.count == 1, "collisions %d", r.count);
	CHECK(r.c[0].normal.y > 0.999f, "normal y %.4f", r.c[0].normal.y);
	CHECK(r.depth >= 0.001f && r.depth < 0.0025f, "depth %.5f", r.depth);
	CHECK(r.safe < 0.2f, "safe fraction %.4f", r.safe);
}

// The game's player (capsule r 0.34, h 1.76, origin at its centre; margin 0.001) walking and standing on a flat floor rests one
// margin above it, as on Jolt: the foot (origin - 0.88) stays within [+0.0005, +0.002] of the floor top for 120 ticks.
static void rest_height(bool p_voxel) {
	Level level;
	if (p_voxel) {
		level.add_voxel_floor(-3, 0.0f, -2, 3, 2);
	} else {
		level.add_box(V3(0, -0.5f, 0), V3(20, 0.5f, 20));
	}
	for (int walking = 0; walking < 2; walking++) {
		Character body;
		body.level = &level;
		body.radius = 0.34f;
		body.height = 1.76f;
		body.position = V3(-2, 0.88f + 0.05f, 0);
		float min_foot = 1e9f, max_foot = -1e9f;
		for (int tick = 0; tick < 120; tick++) {
			body.velocity = V3(walking ? 1.5f : 0.0f, body.on_floor ? -0.1f : body.velocity.y - 9.8f / 60.0f, 0);
			body.move_and_slide(1.0f / 60.0f);
			if (tick >= 20) {
				min_foot = std::fmin(min_foot, body.position.y - 0.88f);
				max_foot = std::fmax(max_foot, body.position.y - 0.88f);
			}
		}
		std::printf("rest height on %s floor, %s: foot y %.5f .. %.5f\n", p_voxel ? "voxel" : "box", walking ? "walking" : "standing", min_foot, max_foot);
		CHECK(min_foot >= 0.0005f && max_foot <= 0.002f, "foot y %.5f .. %.5f outside [0.0005, 0.002] (%s, %s)", min_foot, max_foot, p_voxel ? "voxel" : "box", walking ? "walking" : "standing");
	}
}

static b3m::BodyShape capsule_shape(V3 p_center, float p_radius, float p_height) {
	const float half = std::fmax(0.0f, p_height * 0.5f - p_radius);
	b3m::BodyShape shape;
	shape.points = { b3Vec3(p_center + V3(0, half, 0)), b3Vec3(p_center + V3(0, -half, 0)) };
	shape.radius = p_radius;
	return shape;
}

static b3m::BodyShape box_shape(V3 p_center, float p_half) {
	b3m::BodyShape shape;
	for (int sx = -1; sx <= 1; sx += 2)
		for (int sy = -1; sy <= 1; sy += 2)
			for (int sz = -1; sz <= 1; sz += 2) shape.points.push_back(b3Vec3(p_center + V3(sx * p_half, sy * p_half, sz * p_half)));
	shape.radius = 0.0f;
	return shape;
}

// PhysicsDirectSpaceState3D.cast_motion (ActiveCover.ClearPath): a capsule (r 0.34, h 1.76) swept past a voxel cylinder of 0.1 m
// columns. 1 cm clear of the columns it reports [1, 1]; 1 cm inside it reports the analytic touch to a millimetre.
static void cast_past_cylinder() {
	Level level;
	std::vector<float> columns;
	level.add_voxel_cylinder(0.0f, 0.0f, 0.5f, columns);
	const float radius = 0.34f;
	float z_extent = -1e9f;
	for (size_t i = 0; i < columns.size(); i += 2) z_extent = std::fmax(z_extent, columns[i + 1] + 0.1f);
	for (int inside = 0; inside < 2; inside++) {
		const float z = z_extent + radius + (inside ? -0.01f : 0.01f);
		const float start_x = -1.5f, length = 3.0f;
		b3m::Params params;
		params.world = level.world;
		params.filter = b3DefaultQueryFilter();
		params.margin = 0.0f;
		params.motion = b3Vec3{ length, 0, 0 };
		float safe = 1, unsafe = 1;
		const bool hit = b3m::cast_shapes({ capsule_shape(V3(start_x, 1.0f, z), radius, 1.76f) }, params, safe, unsafe);
		if (!inside) {
			std::printf("cast 1 cm clear of the cylinder: hit %d safe %.4f unsafe %.4f\n", hit, safe, unsafe);
			CHECK(!hit && safe == 1.0f && unsafe == 1.0f, "hit %d safe %.4f unsafe %.4f (expect [1, 1])", hit, safe, unsafe);
			continue;
		}
		// The first column to touch: the capsule's disc at x reaches a column's rectangle when dx^2 + dz^2 <= r^2.
		float touch_x = 1e9f;
		for (size_t i = 0; i < columns.size(); i += 2) {
			const float dz = std::fmax(0.0f, std::fmax(columns[i + 1] - z, z - (columns[i + 1] + 0.1f)));
			if (dz < radius) touch_x = std::fmin(touch_x, columns[i] - std::sqrt(radius * radius - dz * dz));
		}
		const float safe_x = start_x + safe * length, unsafe_x = start_x + unsafe * length;
		std::printf("cast 1 cm into the cylinder: hit %d safe x %.4f unsafe x %.4f analytic %.4f\n", hit, safe_x, unsafe_x, touch_x);
		CHECK(hit, "no hit");
		CHECK(std::fabs(safe_x - touch_x) < 0.001f && std::fabs(unsafe_x - touch_x) < 0.001f, "safe x %.4f unsafe x %.4f vs analytic %.4f", safe_x, unsafe_x, touch_x);
	}
}

// intersect_shape / collide_shape: a box 5 mm clear of a floor is not touching it, one 5 mm into it is.
static void exact_overlap() {
	for (int voxel = 0; voxel < 2; voxel++) {
		Level level;
		if (voxel) {
			level.add_voxel_floor(-3, 0.0f, -2, 3, 2);
		} else {
			level.add_box(V3(0, -0.5f, 0), V3(20, 0.5f, 20));
		}
		b3m::Params params;
		params.world = level.world;
		params.filter = b3DefaultQueryFilter();
		params.margin = 0.0f;
		const bool clear = b3m::overlaps_any({ box_shape(V3(0, 0.5f + 0.005f, 0), 0.5f) }, params, b3Vec3_zero);
		const bool into = b3m::overlaps_any({ box_shape(V3(0, 0.5f - 0.005f, 0), 0.5f) }, params, b3Vec3_zero);
		std::printf("box 5 mm over a %s floor: %d, 5 mm into it: %d\n", voxel ? "voxel" : "box", clear, into);
		CHECK(!clear, "a box 5 mm clear of the %s floor is reported touching", voxel ? "voxel" : "box");
		CHECK(into, "a box 5 mm into the %s floor is not reported", voxel ? "voxel" : "box");
	}
}

// Core penetration (EPA): a box proxy sunk into a turned box, and a capsule core inside one.
static void deep_penetration() {
	b3Vec3 box_a[8];
	b3Vec3 box_b[8];
	int i = 0;
	for (int sx = -1; sx <= 1; sx += 2)
		for (int sy = -1; sy <= 1; sy += 2)
			for (int sz = -1; sz <= 1; sz += 2) {
				box_a[i] = b3Vec3{ (float)sx, (float)sy, (float)sz };
				box_b[i] = b3Vec3{ 1.5f + 0.5f * sx, 0.1f + 0.5f * sy, 0.2f + 0.5f * sz };
				i++;
			}
	b3Vec3 normal, point;
	float separation;
	const b3ShapeProxy a{ box_a, 8, 0.0f };
	const b3ShapeProxy b{ box_b, 8, 0.0f };
	const bool hit = b3m::convex_contact(a, b, 0.01f, normal, separation, point);
	std::printf("EPA box in box: hit %d separation %.4f normal (%.3f %.3f %.3f)\n", hit, separation, normal.x, normal.y, normal.z);
	CHECK(hit && std::fabs(separation) < 1e-3f, "touching boxes: separation %.4f", separation);
	// B spans x 1..2, A spans x -1..1: they touch. Shift B left by 0.2 for a 0.2 overlap along x.
	for (int k = 0; k < 8; k++) box_b[k].x -= 0.2f;
	const bool hit2 = b3m::convex_contact(a, b, 0.01f, normal, separation, point);
	std::printf("EPA box in box (0.2 along x): hit %d separation %.4f normal (%.3f %.3f %.3f)\n", hit2, separation, normal.x, normal.y, normal.z);
	CHECK(hit2 && std::fabs(separation + 0.2f) < 1e-3f, "separation %.4f", separation);
	CHECK(normal.x > 0.999f, "normal x %.4f", normal.x);

	// A capsule core through the middle of a box, along y: leaves along the nearest side.
	b3Vec3 segment[2] = { b3Vec3{ 0.7f, -2, 0 }, b3Vec3{ 0.7f, 2, 0 } };
	const b3ShapeProxy capsule{ segment, 2, 0.3f };
	const bool hit3 = b3m::convex_contact(a, capsule, 0.01f, normal, separation, point);
	std::printf("EPA capsule through box: hit %d separation %.4f normal (%.3f %.3f %.3f)\n", hit3, separation, normal.x, normal.y, normal.z);
	CHECK(hit3 && std::fabs(separation - (-(0.3f + 0.3f))) < 1e-3f, "separation %.4f (expect -0.6)", separation);
	CHECK(normal.x > 0.999f, "normal x %.4f", normal.x);
}

// A step in the way (what ActorMotor's step-up probes): the capsule's lower sphere meets the step's top edge (normal tilted 8 degrees up), the travel stops at it, and
// moving diagonally into a wall corner reports the wall and the floor.
static void step_and_corner() {
	Level level;
	level.add_box(V3(0, -0.5f, 0), V3(20, 0.5f, 20));
	level.add_box(V3(2.0f, 0.15f, 0), V3(0.5f, 0.15f, 2)); // a 0.3 m step whose face is at x = 1.5
	level.add_box(V3(0, 1.0f, 6.5f), V3(10, 1.0f, 0.5f)); // a wall whose face is at z = 6
	Character body;
	body.level = &level;
	Motion r;
	body.test_motion(V3(0, 0.9f + 0.0005f, 0), V3(2, 0, 0), 1, false, r);
	std::printf("step: travel %.4f safe %.4f unsafe %.4f normal (%.3f %.3f %.3f) depth %.5f count %d\n", r.travel.x, r.safe, r.unsafe, r.c[0].normal.x, r.c[0].normal.y, r.c[0].normal.z, r.depth, r.count);
	CHECK(r.count == 1 && r.c[0].normal.x < -0.98f, "step normal x %.3f count %d", r.c[0].normal.x, r.count);
	CHECK(std::fabs(r.travel.x - 1.1540f) < 0.002f, "stopped at x %.4f (expect 1.154)", r.travel.x);
	body.test_motion(V3(-5, 0.9f + 0.0005f, 5.64f), V3(0.3f, 0, 0.3f), 6, false, r);
	std::printf("corner: count %d normal0 (%.3f %.3f %.3f)\n", r.count, r.c[0].normal.x, r.c[0].normal.y, r.c[0].normal.z);
	CHECK(r.count == 1 && r.c[0].normal.z < -0.999f, "wall normal z %.3f count %d", r.c[0].normal.z, r.count);
	// Falling straight down onto the floor: the floor, and the body ends touching it.
	body.test_motion(V3(-5, 3.0f, 0), V3(0, -5, 0), 6, false, r);
	CHECK(r.count == 1 && r.c[0].normal.y > 0.999f, "floor normal y %.3f", r.c[0].normal.y);
	CHECK(std::fabs(r.travel.y - (-5.0f + 0 + (5 - 2.1f)) ) < 0.002f || std::fabs(3.0f + r.travel.y - 0.9f) < 0.002f, "landed at y %.4f", 3.0f + r.travel.y);
	std::printf("fall: lands at y %.4f\n", 3.0f + r.travel.y);
}

// The game's stall on Box3D: a player standing on a PaintedBuilding landing, 0.22 m beside the corner of a 0.2 m post (box 185
// of the dumped grid, x 4.0..4.2, z 1.8..2.0). Walking -z touches the corner, whose true normal is diagonal; the contact was an
// axis push of the box's bounding box (0, 0, 1), a wall in the way that blocked the slide completely.
static void stall_post_corner() {
	Level level;
	level.add_stall_grid();
	Character body;
	body.level = &level;
	const V3 start(3.7803605f, 4.000139f + 0.9f, 2.2915742f);

	Motion r;
	const bool collided = body.test_motion(start, V3(0, 0, -0.1f), 6, true, r);
	std::printf("stall: collided %d, %d collisions, travel z %.4f, normal (%.3f %.3f %.3f)\n", collided, r.count, r.travel.z, r.c[0].normal.x, r.c[0].normal.y, r.c[0].normal.z);
	CHECK(collided && r.count >= 1, "no contact with the post corner");
	CHECK(std::fabs(r.travel.z + 0.0185f) < 0.004f, "stopped after %.4f instead of 0.0185", -r.travel.z);
	CHECK(r.c[0].normal.z > 0.5f && r.c[0].normal.z < 0.95f && r.c[0].normal.x < -0.3f && std::fabs(r.c[0].normal.y) < 0.05f, "normal (%.3f %.3f %.3f) is not the corner's diagonal", r.c[0].normal.x, r.c[0].normal.y, r.c[0].normal.z);
	CHECK(r.c[0].point.y > 4.0f && std::fabs(r.c[0].point.x - 4.0f) < 0.02f && std::fabs(r.c[0].point.z - 2.0f) < 0.02f, "contact point (%.3f %.3f %.3f) is not the corner (4, ., 2)", r.c[0].point.x, r.c[0].point.y, r.c[0].point.z);

	// Free sides stay free.
	body.test_motion(start, V3(-0.1f, 0, 0), 6, true, r);
	CHECK(r.count == 0, "-x blocked");
	body.test_motion(start, V3(0, 0, 0.1f), 6, true, r);
	CHECK(r.count == 0, "+z blocked");

	// Walking -z with move_and_slide slides off the corner and carries on.
	body.position = start;
	body.on_floor = true;
	for (int tick = 0; tick < 90; tick++) {
		body.velocity = V3(0, -0.1f, -3.0f);
		body.move_and_slide(1.0f / 60.0f);
	}
	std::printf("stall walk: x %.3f y %.3f z %.3f\n", body.position.x, body.position.y - 0.9f, body.position.z);
	CHECK(body.position.z < start.z - 1.0f, "stuck at z %.3f", body.position.z);
	CHECK(std::fabs(body.position.y - 0.9f - 4.0f) < 0.3f, "left the landing: y %.3f", body.position.y);
}

#include "regression_tests.inc"

int main(int argc, char** argv) {
	b3SetLengthUnitsPerMeter(1.0f);
	if (argc > 1 && std::strcmp(argv[1], "bench") == 0) {
		for (int p = 0; p < 4; p++) bench_queries(p % 2 == 1, 20, p >= 2);
		return 0;
	}
	for (Kind kind : { Kind::Box, Kind::Wedge, Kind::Mesh }) {
		walk_up(kind, false);
	}
	for (Kind kind : { Kind::Box, Kind::Wedge, Kind::Mesh }) {
		stand_still(kind, 30.0f);
	}
	for (Kind kind : { Kind::Box, Kind::Wedge, Kind::Mesh }) {
		slide_down(kind);
	}
	walk_up(Kind::Box, true);
	walk_up(Kind::Wedge, true);
	walk_up(Kind::Mesh, true);
	embedded_recovery();
	no_phantom_push();
	rest_on_floor();
	rest_height(false);
	rest_height(true);
	cast_past_cylinder();
	exact_overlap();
	deep_penetration();
	step_and_corner();
	stall_post_corner();
	cast_against_the_grid();
	passage_after_removal();
	long_cast_beside_a_wall();
	std::printf("%d checks, %d failures\n", g_checks, g_failures);
	return g_failures == 0 ? 0 : 1;
}

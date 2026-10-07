// Regression tests of the exact queries (included by motion_test.cpp after its helpers): the game's voxel row as a grid and as
// hulls, grazing casts, an edit in place, and the cost of the queries.

// VoxelGridShapeTests.ShapesTouchAndCastAgainstTheGridAsAgainstTheMesh: the game's 8 x 1 x 8 row (floor 3 x 3, a wall along its
// north edge, a turned wall on its east edge, a railed floor cell) as a voxel grid and as the same boxes in world space (convex
// hulls, which the motion core answers exactly).
struct RowBox { float lo[3], hi[3]; };

static std::vector<std::vector<RowBox>> row_modules() {
	return {
		{ { { 0, 0, 0 }, { 2, 0.2f, 2 } } }, // Floor
		{ { { 0, 0, 0.8f }, { 2, 2, 1.2f } } }, // Wall
		{ { { 0, 0, 0 }, { 2, 0.2f, 2 } }, { { 0, 0.2f, 1.8f }, { 2, 1.1f, 2 } } }, // Railing
	};
}

static RowBox rotate_box(RowBox p_box, int p_quarter_turns, float p_cell) {
	for (int turn = 0; turn < p_quarter_turns; turn++) {
		const RowBox b = p_box;
		p_box.lo[0] = b.lo[2];
		p_box.lo[2] = p_cell - b.hi[0];
		p_box.hi[0] = b.hi[2];
		p_box.hi[2] = p_cell - b.lo[0];
	}
	return p_box;
}

struct RowCell { int x, z, module, turns; };

static std::vector<RowCell> row_cells() {
	std::vector<RowCell> cells;
	for (int z = 0; z < 3; z++)
		for (int x = 0; x < 3; x++) cells.push_back({ x, z, 0, 0 });
	for (int x = 0; x < 3; x++) cells.push_back({ x, 3, 1, 0 });
	for (int z = 0; z < 3; z++) cells.push_back({ 3, z, 1, 1 });
	cells.push_back({ 4, 1, 2, 2 });
	return cells;
}

struct RowWorld {
	std::vector<b3VoxelGridModule*> modules;
	b3VoxelGrid* grid = nullptr;
	b3WorldId world;
	std::vector<RowCell> cells;

	// p_voxel: one voxel grid shape; otherwise every box a hull. p_skip: a cell left out (for the edit in place test).
	explicit RowWorld(bool p_voxel) {
		b3WorldDef world_def = b3DefaultWorldDef();
		world = b3CreateWorld(&world_def);
		const auto base = row_modules();
		cells = row_cells();
		const float cell = 2.0f;
		b3BodyDef body_def = b3DefaultBodyDef();
		b3BodyId body = b3CreateBody(world, &body_def);
		if (p_voxel) {
			std::vector<std::pair<int, int>> keys; // module, turns, in the order first asked for
			const int padded = 10;
			std::vector<int> grid_cells(padded * 3 * padded, -1);
			std::vector<std::vector<float>> module_boxes;
			for (int z = 0; z < 8; z++)
				for (int x = 0; x < 8; x++)
					for (const RowCell& c : cells) {
						if (c.x != x || c.z != z) continue;
						int index = -1;
						for (size_t i = 0; i < keys.size(); i++) {
							if (keys[i] == std::make_pair(c.module, c.turns)) index = (int)i;
						}
						if (index < 0) {
							index = (int)keys.size();
							keys.push_back({ c.module, c.turns });
							std::vector<float> boxes;
							for (const RowBox& b : base[c.module]) {
								const RowBox r = rotate_box(b, c.turns, cell);
								boxes.insert(boxes.end(), { r.lo[0], r.lo[1], r.lo[2], r.hi[0], r.hi[1], r.hi[2] });
							}
							module_boxes.push_back(boxes);
						}
						grid_cells[(padded + z + 1) * padded + x + 1] = index;
					}
			for (const auto& boxes : module_boxes) modules.push_back(b3CreateVoxelGridModule(boxes.data(), (int)boxes.size() / 6, cell, 20));
			b3VoxelGridDef def = {};
			def.cellCountX = 8;
			def.cellCountY = 1;
			def.cellCountZ = 8;
			def.origin = b3Vec3{ 0, 0, 0 };
			def.cellMeters = cell;
			def.cellVoxels = 20;
			def.maxBoxesPerCell = 256;
			def.modules = modules.data();
			def.moduleCount = (int)modules.size();
			def.paddedCells = grid_cells.data();
			grid = b3CreateVoxelGrid(&def);
			b3ShapeDef shape = b3DefaultShapeDef();
			b3CreateVoxelGridShape(body, &shape, grid);
		} else {
			for (const RowCell& c : cells)
				for (const RowBox& b : base[c.module]) {
					const RowBox r = rotate_box(b, c.turns, cell);
					b3Transform xf;
					xf.p = b3Vec3{ c.x * cell + 0.5f * (r.lo[0] + r.hi[0]), 0.5f * (r.lo[1] + r.hi[1]), c.z * cell + 0.5f * (r.lo[2] + r.hi[2]) };
					xf.q = b3Quat_identity;
					b3BoxHull box = b3MakeTransformedBoxHull(0.5f * (r.hi[0] - r.lo[0]), 0.5f * (r.hi[1] - r.lo[1]), 0.5f * (r.hi[2] - r.lo[2]), xf);
					b3ShapeDef shape = b3DefaultShapeDef();
					b3CreateHullShape(body, &shape, &box.base);
				}
		}
	}
	~RowWorld() {
		b3DestroyWorld(world);
		if (grid) b3ReleaseVoxelGrid(grid);
		for (b3VoxelGridModule* m : modules) b3ReleaseVoxelGridModule(m);
	}
};

static b3m::BodyShape sphere_shape(V3 p_center, float p_radius) {
	b3m::BodyShape shape;
	shape.points = { b3Vec3(p_center) };
	shape.radius = p_radius;
	return shape;
}

static bool cast_at(b3WorldId p_world, const b3m::BodyShape& p_shape, V3 p_motion, float p_margin, float& r_safe, float& r_unsafe) {
	b3m::Params params;
	params.world = p_world;
	params.filter = b3DefaultQueryFilter();
	params.margin = p_margin;
	params.motion = b3Vec3(p_motion);
	return b3m::cast_shapes({ p_shape }, params, r_safe, r_unsafe);
}

// A 0.6 m box and a 0.3 m sphere cast along (0.4, -2, 0.3) over the row stop where the same boxes as hulls stop (the game's test
// compares the grid with the equivalent triangle mesh, which the hulls stand in for here).
static void cast_against_the_grid() {
	RowWorld grid(true), reference(false);
	int casts = 0, bad = 0, shown = 0;
	for (int probe = 0; probe < 2; probe++)
		for (float x = 0.15f; x < 10; x += 0.7f)
			for (float z = 0.15f; z < 8; z += 0.7f)
				for (float y = 0.1f; y < 2.6f; y += 0.6f) {
					const b3m::BodyShape shape = probe == 0 ? sphere_shape(V3(x, y, z), 0.3f) : box_shape(V3(x, y, z), 0.3f);
					float safe[2], unsafe[2];
					cast_at(grid.world, shape, V3(0.4f, -2.0f, 0.3f), 0.0f, safe[0], unsafe[0]);
					cast_at(reference.world, shape, V3(0.4f, -2.0f, 0.3f), 0.0f, safe[1], unsafe[1]);
					casts++;
					if (std::fabs(safe[0] - safe[1]) >= 0.01f) {
						bad++;
						if (shown++ < 6) std::printf("  %s cast from (%.3f %.3f %.3f): reference safe %.4f, grid %.4f\n", probe == 0 ? "sphere" : "box", x, y, z, safe[1], safe[0]);
					}
				}
	// The game's failing probe: the box skims the underside of a railed cell's floor slab edge (the slab's bottom edge at z 2, y 0,
	// where the box's top edge arrives exactly as it passes): the triangle mesh reports the touch, so must the grid.
	{
		float safe = 1, unsafe = 1;
		cast_at(grid.world, box_shape(V3(7.85f, 0.7f, 1.55f), 0.3f), V3(0.4f, -2.0f, 0.3f), 0.0f, safe, unsafe);
		std::printf("grazing box cast: safe %.4f unsafe %.4f (the mesh stops at 0.5)\n", safe, unsafe);
		CHECK(std::fabs(safe - 0.5f) < 0.01f, "the box that grazes the slab edge moves %.4f of its cast instead of stopping at 0.5", safe);
	}
	std::printf("grid vs hull casts: %d casts, %d differ\n", casts, bad);
	CHECK(bad == 0, "%d of %d casts against the voxel grid differ from the same boxes as hulls", bad, casts);
}

// --- the cost of the queries ---------------------------------------------------------------------------------------------
//
// cast_motion and intersect_shape of a capsule (the AI's and the cover code's probe) near a voxel stair landing and over a mesh
// floor, in microseconds a query. "box3d" is a plain Box3D cast / overlap (what the plugin answered with before the exact
// queries), "exact" the motion core's.
#include <chrono>

struct Lcg {
	uint32_t state = 12345u;
	float next() {
		state = state * 1664525u + 1013904223u;
		return (float)(state >> 8) / (float)(1 << 24);
	}
};

static float plain_cast_callback(b3ShapeId, b3Pos, b3Vec3, float p_fraction, uint64_t, int, int, void* p_context) {
	float* closest = static_cast<float*>(p_context);
	*closest = std::fmin(*closest, p_fraction);
	return p_fraction;
}

static bool counting_overlap(b3ShapeId, void* p_context) {
	(*static_cast<int*>(p_context))++;
	return true;
}

struct BenchWorld {
	Level level;
	bool mesh_floor;
	explicit BenchWorld(bool p_mesh, float p_roll = 0.15f) : mesh_floor(p_mesh) {
		if (p_mesh) {
			// 64 x 64 cells of 0.5 m, gently rolling, one mesh.
			const int n = 64;
			std::vector<b3Vec3> vertices;
			for (int j = 0; j <= n; j++)
				for (int i = 0; i <= n; i++) vertices.push_back(b3Vec3{ i * 0.5f, 4.0f + p_roll * std::sin(i * 0.3f) * std::cos(j * 0.25f), j * 0.5f });
			std::vector<int32_t> indices;
			for (int j = 0; j < n; j++)
				for (int i = 0; i < n; i++) {
					const int a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
					indices.insert(indices.end(), { a, c, b, b, c, d });
				}
			b3MeshDef def = {};
			def.vertices = vertices.data();
			def.indices = indices.data();
			def.vertexCount = (int)vertices.size();
			def.triangleCount = (int)indices.size() / 3;
			def.doubleSided = true;
			b3MeshData* mesh = b3CreateMesh(&def, nullptr, 0);
			level.meshes.push_back(mesh);
			b3ShapeDef shape = b3DefaultShapeDef();
			b3CreateMeshShape(level.static_body(), &shape, mesh, b3Vec3{ 1, 1, 1 });
		} else {
			level.add_stall_grid();
		}
	}
};

static void bench_queries(bool p_mesh, int p_repeat, bool p_level_casts = false) {
	BenchWorld bw(p_mesh, p_level_casts ? 0.0f : 0.15f);
	const float radius = 0.34f, height = 1.76f;
	struct Query { V3 centre, motion; };
	std::vector<Query> queries;
	Lcg rng;
	for (int i = 0; i < 400; i++) {
		const float span = p_mesh ? 30.0f : 15.0f;
		Query q;
		q.centre = V3(1.0f + rng.next() * span, 4.0f + 0.9f + (p_level_casts ? 0.04f : (p_mesh ? 0.1f : 0.3f) + rng.next() * 0.2f), 1.0f + rng.next() * span);
		const float angle = rng.next() * 6.2831853f;
		const float length = 0.5f + rng.next() * 5.0f;
		q.motion = V3(std::cos(angle) * length, p_level_casts ? 0.0f : -0.3f * length * rng.next(), std::sin(angle) * length);
		queries.push_back(q);
	}
	using Clock = std::chrono::steady_clock;
	b3QueryFilter filter = b3DefaultQueryFilter();
	long hits_plain = 0, hits_exact = 0, overlaps_plain = 0, overlaps_exact = 0;
	double cast_plain = 0, cast_exact = 0, inter_plain = 0, inter_exact = 0;
	for (int repeat = 0; repeat < p_repeat; repeat++) {
		for (const Query& q : queries) {
			const b3m::BodyShape shape = capsule_shape(q.centre, radius, height);
			const b3ShapeProxy proxy{ shape.points.data(), (int)shape.points.size(), shape.radius };
			auto t0 = Clock::now();
			float closest = 1.0f;
			b3World_CastShape(bw.level.world, b3Vec3_zero, &proxy, b3Vec3(q.motion), filter, plain_cast_callback, &closest);
			auto t1 = Clock::now();
			float safe = 1, unsafe = 1;
			const bool hit = cast_at(bw.level.world, shape, q.motion, 0.0f, safe, unsafe);
			auto t2 = Clock::now();
			int count_plain = 0;
			b3World_OverlapShape(bw.level.world, b3Vec3_zero, &proxy, filter, counting_overlap, &count_plain);
			auto t3 = Clock::now();
			// What the plugin's intersect_shape does: the overlap, each candidate confirmed by its exact contacts.
			struct Exact { b3m::Params params; const b3ShapeProxy* proxy; int count; } exact{ {}, &proxy, 0 };
			exact.params.world = bw.level.world;
			exact.params.margin = 0.0f;
			auto callback = [](b3ShapeId p_id, void* p_context) {
				Exact* e = static_cast<Exact*>(p_context);
				std::vector<b3m::Contact> contacts;
				if (b3m::shape_contacts(e->params, *e->proxy, 0, p_id, contacts, true) && contacts.empty()) return true;
				e->count++;
				return true;
			};
			b3World_OverlapShape(bw.level.world, b3Vec3_zero, &proxy, filter, callback, &exact);
			auto t4 = Clock::now();
			cast_plain += std::chrono::duration<double, std::micro>(t1 - t0).count();
			cast_exact += std::chrono::duration<double, std::micro>(t2 - t1).count();
			inter_plain += std::chrono::duration<double, std::micro>(t3 - t2).count();
			inter_exact += std::chrono::duration<double, std::micro>(t4 - t3).count();
			hits_plain += closest < 1.0f;
			hits_exact += hit;
			overlaps_plain += count_plain > 0;
			overlaps_exact += exact.count > 0;
		}
	}
	const double n = (double)queries.size() * p_repeat;
	std::printf("bench %s: cast_motion box3d %.2f us (%ld hits/%d), exact %.2f us (%ld hits/%d); intersect box3d %.2f us (%ld), exact %.2f us (%ld)\n",
			std::string(p_mesh ? "mesh floor" : "stall grid").append(p_level_casts ? " (level casts, 4 cm up)" : " (random heights)").c_str(), cast_plain / n, hits_plain / p_repeat, (int)queries.size(), cast_exact / n, hits_exact / p_repeat, (int)queries.size(),
			inter_plain / n, overlaps_plain / p_repeat, inter_exact / n, overlaps_exact / p_repeat);
}

// ConstructionProof: a partial-cell wall (x 0..1.4 of cell (0,0,0)) and a full block in the next cell leave a 0.6 m gap a standing
// capsule cannot pass; once the block is removed from the shape in place (b3Shape_VoxelGridSetCells) the capsule passes at
// x = 1.8, 7 cm from the wall. ActiveCover.ClearPath: an intersect at the start and the end, a cast along the path, all with a
// 5 mm margin, the capsule 1.76 m tall and 0.33 m wide, 3 cm above the floor.
static bool clear_path(b3WorldId p_world, V3 p_from, V3 p_to) {
	const float radius = 0.33f, height = 1.76f;
	const V3 lift(0, 0.9f + 0.03f, 0);
	b3m::Params params;
	params.world = p_world;
	params.filter = b3DefaultQueryFilter();
	params.margin = 0.005f;
	auto hits = [&](V3 p_centre) {
		const b3m::BodyShape shape = capsule_shape(p_centre, radius, height);
		const b3ShapeProxy proxy{ shape.points.data(), (int)shape.points.size(), shape.radius };
		struct Probe { const b3m::Params* params; const b3ShapeProxy* proxy; bool found; } probe{ &params, &proxy, false };
		const b3ShapeProxy query{ proxy.points, proxy.count, proxy.radius + params.margin };
		b3World_OverlapShape(p_world, b3Vec3_zero, &query, params.filter, [](b3ShapeId p_id, void* p_context) {
			Probe* p = static_cast<Probe*>(p_context);
			std::vector<b3m::Contact> contacts;
			if (b3m::shape_contacts(*p->params, *p->proxy, 0, p_id, contacts, true) && contacts.empty()) return true;
			p->found = true;
			return false;
		}, &probe);
		return probe.found;
	};
	const bool start_hit = hits(p_from + lift);
	float safe = 1, unsafe = 1;
	cast_at(p_world, capsule_shape(p_from + lift, radius, height), p_to - p_from, params.margin, safe, unsafe);
	const bool end_hit = hits(p_to + lift);
	if (false) std::printf("  clear_path: start hit %d, cast safe %.4f unsafe %.4f, end hit %d%c", start_hit, safe, unsafe, end_hit, 10);
	return !start_hit && safe >= 0.999f && !end_hit;
}

static void passage_after_removal() {
	b3WorldDef world_def = b3DefaultWorldDef();
	b3WorldId world = b3CreateWorld(&world_def);
	b3BodyDef body_def = b3DefaultBodyDef();
	b3ShapeDef shape_def = b3DefaultShapeDef();
	{
		b3Transform xf;
		xf.p = b3Vec3{ 0, -0.5f, 0 };
		xf.q = b3Quat_identity;
		b3BoxHull floor = b3MakeTransformedBoxHull(40, 0.5f, 40, xf);
		b3CreateHullShape(b3CreateBody(world, &body_def), &shape_def, &floor.base);
	}
	const float side[6] = { 0, 0, 0, 1.4f, 2, 2 };
	const float block[6] = { 0, 0, 0, 2, 2, 2 };
	b3VoxelGridModule* modules[2] = { b3CreateVoxelGridModule(side, 1, 2.0f, 20), b3CreateVoxelGridModule(block, 1, 2.0f, 20) };
	const int padded = 10;
	std::vector<int> cells(padded * 3 * padded, -1);
	auto index = [&](int x, int z) { return (padded + z + 1) * padded + x + 1; };
	cells[index(0, 0)] = 0;
	b3VoxelGridDef def = {};
	def.cellCountX = 8;
	def.cellCountY = 1;
	def.cellCountZ = 8;
	def.origin = b3Vec3{ 0, 0, 0 };
	def.cellMeters = 2.0f;
	def.cellVoxels = 20;
	def.maxBoxesPerCell = 256;
	def.modules = modules;
	def.moduleCount = 2;
	def.paddedCells = cells.data();
	b3VoxelGrid* grid = b3CreateVoxelGrid(&def);
	const b3ShapeId id = b3CreateVoxelGridShape(b3CreateBody(world, &body_def), &shape_def, grid);

	const V3 from(1.8f, 0, -1), to(1.8f, 0, 3);
	const bool open_before = clear_path(world, from, to);
	// The opposing side placed in place, as the game's edit does.
	int cell = index(1, 0), module = 1;
	b3Shape_VoxelGridSetCells(id, &cell, &module, 1);
	const bool blocked = !clear_path(world, from, to);
	module = -1;
	b3Shape_VoxelGridSetCells(id, &cell, &module, 1);
	const bool clear_after = clear_path(world, from, to);
	std::printf("passage: free with one side %d, blocked with both %d, free after the removal %d\n", open_before, blocked, clear_after);
	CHECK(open_before && blocked && clear_after, "free with one side %d, blocked with both %d, free after the removal %d", open_before, blocked, clear_after);

	b3DestroyWorld(world);
	b3ReleaseVoxelGrid(grid);
	for (b3VoxelGridModule* m : modules) b3ReleaseVoxelGridModule(m);
}

// A long cast along a wall 2 mm away (inside Box3D's slop, which the plain cast cannot sweep) is clear however many steps the exact
// sweep needs. 2 mm into the wall it is clear too: a body the shape already touches takes no part (Jolt's rule).
static void long_cast_beside_a_wall() {
	Level level;
	level.add_box(V3(0, -0.5f, 0), V3(40, 0.5f, 40));
	level.add_box(V3(10, 1, 1), V3(12, 1, 0.5f)); // a wall along x, its face at z = 0.5
	const float radius = 0.33f;
	for (int into = 0; into < 2; into++) {
		float safe = 1, unsafe = 1;
		const float z = 0.5f - radius - (into ? -0.002f : 0.002f);
		const bool hit = cast_at(level.world, capsule_shape(V3(-1, 1.0f, z), radius, 1.76f), V3(16, 0, 0), 0.0f, safe, unsafe);
		std::printf("16 m cast 2 mm %s a wall: hit %d safe %.4f%c", into ? "into" : "beside", hit, safe, 10);
		CHECK(!hit && safe == 1.0f, "hit %d safe %.4f (expect [1, 1])", hit, safe);
	}
}

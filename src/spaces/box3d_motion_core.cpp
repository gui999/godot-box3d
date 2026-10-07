#include "box3d_motion_core.hpp"

#include <box3d/constants.h>
#include <box3d/math_functions.h>

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace b3m {

namespace {

constexpr float CONTACT_EPSILON = 1.0e-5f;
constexpr float DIRECTION_EPSILON = 1.0e-5f;
constexpr float CONTACT_GROWTH = 1.0e-4f; // see collect_callback: meshes and voxel grids ignore a proxy whose radius is 0
constexpr int GJK_SIMPLEX_CAPACITY = 40; // Box3D runs at most 32 GJK iterations

b3Vec3 centroid(const b3Vec3* p_points, int p_count) {
	b3Vec3 sum = b3Vec3_zero;
	for (int i = 0; i < p_count; i++) {
		sum = b3Add(sum, p_points[i]);
	}
	return b3MulSV(1.0f / (float)p_count, sum);
}

int support_index(const b3Vec3* p_points, int p_count, b3Vec3 p_direction) {
	int best = 0;
	float best_dot = b3Dot(p_points[0], p_direction);
	for (int i = 1; i < p_count; i++) {
		const float d = b3Dot(p_points[i], p_direction);
		if (d > best_dot) {
			best_dot = d;
			best = i;
		}
	}
	return best;
}

// --- EPA over the Minkowski difference B - A of two point clouds -------------------------------------------------------

struct EpaVertex {
	b3Vec3 w; // b - a
	b3Vec3 a; // the point of A
};

struct EpaFace {
	int index[3];
	b3Vec3 normal; // outward, unit
	float distance; // from the origin along the normal
	bool alive;
};

struct Epa {
	const b3ShapeProxy* a;
	const b3ShapeProxy* b;
	EpaVertex vertices[160];
	int vertex_count = 0;
	EpaFace faces[400];
	int face_count = 0;

	EpaVertex support(b3Vec3 p_direction) const {
		EpaVertex v;
		const int ib = support_index(b->points, b->count, p_direction);
		const int ia = support_index(a->points, a->count, b3Neg(p_direction));
		v.a = a->points[ia];
		v.w = b3Sub(b->points[ib], v.a);
		return v;
	}

	bool add_face(int p_i0, int p_i1, int p_i2) {
		if (face_count >= 400) {
			return false;
		}
		const b3Vec3 v0 = vertices[p_i0].w;
		const b3Vec3 v1 = vertices[p_i1].w;
		const b3Vec3 v2 = vertices[p_i2].w;
		b3Vec3 n = b3Cross(b3Sub(v1, v0), b3Sub(v2, v0));
		const float length = b3Length(n);
		if (length < 1.0e-12f) {
			return true; // a sliver adds nothing; the neighbours still close the hull
		}
		n = b3MulSV(1.0f / length, n);
		float d = b3Dot(n, v0);
		EpaFace f;
		f.index[0] = p_i0;
		f.index[1] = p_i1;
		f.index[2] = p_i2;
		if (d < 0.0f) {
			// The origin is on the negative side: flip the winding so the normal points away from it.
			f.index[1] = p_i2;
			f.index[2] = p_i1;
			n = b3Neg(n);
			d = -d;
		}
		f.normal = n;
		f.distance = d;
		f.alive = true;
		faces[face_count++] = f;
		return true;
	}

	// p_seed is a tetrahedron around the origin. On success: normal is the outward normal of the closest face of B - A
	// (B has to move by -normal * depth to clear A).
	bool solve(const b3Simplex& p_seed, b3Vec3& r_normal, float& r_depth, b3Vec3& r_point_on_a) {
		for (int i = 0; i < 4; i++) {
			vertices[i].w = p_seed.vertices[i].w;
			vertices[i].a = p_seed.vertices[i].wA;
		}
		vertex_count = 4;
		add_face(0, 1, 2);
		add_face(0, 3, 1);
		add_face(0, 2, 3);
		add_face(1, 3, 2);
		if (face_count < 4) {
			return false;
		}

		for (int iteration = 0; iteration < 64; iteration++) {
			int closest = -1;
			for (int i = 0; i < face_count; i++) {
				if (faces[i].alive && (closest < 0 || faces[i].distance < faces[closest].distance)) {
					closest = i;
				}
			}
			if (closest < 0) {
				return false;
			}

			const EpaFace face = faces[closest];
			const EpaVertex s = support(face.normal);
			const float grown = b3Dot(s.w, face.normal);
			if (grown - face.distance < 1.0e-5f || vertex_count >= 160) {
				r_normal = face.normal;
				r_depth = face.distance;
				// Witness on A: the origin's projection on the face, in barycentric coordinates.
				const b3Vec3 v0 = vertices[face.index[0]].w;
				const b3Vec3 v1 = vertices[face.index[1]].w;
				const b3Vec3 v2 = vertices[face.index[2]].w;
				const b3Vec3 p = b3MulSV(face.distance, face.normal);
				const b3Vec3 e0 = b3Sub(v1, v0);
				const b3Vec3 e1 = b3Sub(v2, v0);
				const b3Vec3 e2 = b3Sub(p, v0);
				const float d00 = b3Dot(e0, e0);
				const float d01 = b3Dot(e0, e1);
				const float d11 = b3Dot(e1, e1);
				const float d20 = b3Dot(e2, e0);
				const float d21 = b3Dot(e2, e1);
				const float denominator = d00 * d11 - d01 * d01;
				float u = 0.0f;
				float v = 0.0f;
				if (std::fabs(denominator) > 1.0e-20f) {
					v = (d11 * d20 - d01 * d21) / denominator;
					u = (d00 * d21 - d01 * d20) / denominator;
				}
				const float w0 = 1.0f - v - u;
				r_point_on_a = b3Add(
						b3Add(b3MulSV(w0, vertices[face.index[0]].a), b3MulSV(v, vertices[face.index[1]].a)),
						b3MulSV(u, vertices[face.index[2]].a));
				return true;
			}

			// Grow the hull: drop every face the new vertex sees and stitch the horizon to it.
			const int new_vertex = vertex_count++;
			vertices[new_vertex] = s;
			int horizon[400][2];
			int horizon_count = 0;
			for (int i = 0; i < face_count; i++) {
				EpaFace& f = faces[i];
				if (!f.alive) {
					continue;
				}
				if (b3Dot(f.normal, b3Sub(s.w, vertices[f.index[0]].w)) > 1.0e-7f) {
					f.alive = false;
					for (int e = 0; e < 3; e++) {
						const int from = f.index[e];
						const int to = f.index[(e + 1) % 3];
						bool shared = false;
						for (int h = 0; h < horizon_count; h++) {
							if (horizon[h][0] == to && horizon[h][1] == from) {
								horizon[h][0] = horizon[horizon_count - 1][0];
								horizon[h][1] = horizon[horizon_count - 1][1];
								horizon_count--;
								shared = true;
								break;
							}
						}
						if (!shared && horizon_count < 400) {
							horizon[horizon_count][0] = from;
							horizon[horizon_count][1] = to;
							horizon_count++;
						}
					}
				}
			}
			for (int h = 0; h < horizon_count; h++) {
				if (!add_face(horizon[h][0], horizon[h][1], new_vertex)) {
					return false;
				}
			}
		}

		// Out of iterations: the closest face is the answer to within the tolerance reached.
		int closest = -1;
		for (int i = 0; i < face_count; i++) {
			if (faces[i].alive && (closest < 0 || faces[i].distance < faces[closest].distance)) {
				closest = i;
			}
		}
		if (closest < 0) {
			return false;
		}
		r_normal = faces[closest].normal;
		r_depth = faces[closest].distance;
		r_point_on_a = vertices[faces[closest].index[0]].a;
		return true;
	}
};

struct AcceptState {
	const Params* params;
};

bool accepted(const Params& p_params, b3ShapeId p_shape_id) {
	return p_params.accept == nullptr || p_params.accept(p_shape_id, p_params.accept_context);
}

// World-space proxy of a convex Box3D shape (sphere, capsule, hull). Returns false for any other type.
bool convex_target_proxy(b3ShapeId p_shape_id, b3Transform p_transform, std::vector<b3Vec3>& r_points, float& r_radius) {
	r_points.clear();
	switch (b3Shape_GetType(p_shape_id)) {
		case b3_sphereShape: {
			const b3Sphere sphere = b3Shape_GetSphere(p_shape_id);
			r_points.push_back(b3TransformPoint(p_transform, sphere.center));
			r_radius = sphere.radius;
			return true;
		}
		case b3_capsuleShape: {
			const b3Capsule capsule = b3Shape_GetCapsule(p_shape_id);
			r_points.push_back(b3TransformPoint(p_transform, capsule.center1));
			r_points.push_back(b3TransformPoint(p_transform, capsule.center2));
			r_radius = capsule.radius;
			return true;
		}
		case b3_hullShape: {
			const b3HullData* hull = b3Shape_GetHull(p_shape_id);
			const b3Vec3* points = b3GetHullPoints(hull);
			for (int i = 0; i < hull->vertexCount; i++) {
				r_points.push_back(b3TransformPoint(p_transform, points[i]));
			}
			r_radius = 0.0f;
			return !r_points.empty();
		}
		default:
			return false;
	}
}

struct CollectContext {
	const Params* params;
	const b3ShapeProxy* body;
	int local_shape;
	std::vector<Contact>* contacts;
};

bool collect_callback(b3ShapeId p_shape_id, void* p_context) {
	auto* ctx = static_cast<CollectContext*>(p_context);
	const Params& params = *ctx->params;
	if (!accepted(params, p_shape_id)) {
		return true;
	}

	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	const b3Transform body_transform = b3Body_GetTransform(body_id);
	const b3ShapeProxy& body = *ctx->body;
	const float margin = params.margin;

	switch (b3Shape_GetType(p_shape_id)) {
		case b3_sphereShape:
		case b3_capsuleShape:
		case b3_hullShape: {
			std::vector<b3Vec3> points;
			float radius = 0.0f;
			if (!convex_target_proxy(p_shape_id, body_transform, points, radius)) {
				return true;
			}
			const b3ShapeProxy target{ points.data(), (int)points.size(), radius };
			Contact contact;
			if (convex_contact(target, body, margin, contact.normal, contact.separation, contact.point)) {
				contact.shape = p_shape_id;
				contact.local_shape = ctx->local_shape;
				ctx->contacts->push_back(contact);
			}
			return true;
		}

		case b3_meshShape: {
			// The mesh answers per triangle through the proxy grown by the margin (and a hair, which a flat core with no radius
			// needs to be answered at all): depth is then the push that leaves the grown proxy, so the separation is grow - depth.
			const float grow = margin + CONTACT_GROWTH;
			const b3Mesh mesh = b3Shape_GetMesh(p_shape_id);
			const b3ShapeProxy grown{ body.points, body.count, body.radius + grow };
			b3MeshRecoverResult results[64];
			const int count = b3RecoverMesh(&mesh, body_transform, &grown, results, 64);
			for (int i = 0; i < count; i++) {
				Contact contact;
				contact.shape = p_shape_id;
				contact.normal = results[i].normal;
				contact.point = results[i].point;
				contact.separation = grow - results[i].depth;
				if (contact.separation >= margin) {
					continue;
				}
				contact.local_shape = ctx->local_shape;
				ctx->contacts->push_back(contact);
			}
			return true;
		}

		case b3_voxelGridShape: {
			// One exact contact per box (a post's corner leaves along its diagonal, not along an axis of its bounds); depth is
			// the push that leaves the grown proxy, so the separation is grow - depth.
			const float grow = margin + CONTACT_GROWTH;
			const b3ShapeProxy grown{ body.points, body.count, body.radius + grow };
			b3VoxelContact results[128];
			const int count = b3CollideVoxelGrid(b3Shape_GetVoxelGrid(p_shape_id), body_transform, &grown, results, 128);
			for (int i = 0; i < count; i++) {
				Contact contact;
				contact.shape = p_shape_id;
				contact.normal = results[i].normal;
				contact.point = results[i].point;
				contact.separation = grow - results[i].depth;
				if (contact.separation >= margin) {
					continue;
				}
				contact.local_shape = ctx->local_shape;
				ctx->contacts->push_back(contact);
			}
			return true;
		}

		default:
			// Height fields and compounds have no contact answer yet.
			return true;
	}
}

// The proxy of one body shape moved by an offset.
struct MovedShape {
	std::vector<b3Vec3> points;
	b3ShapeProxy proxy;
	MovedShape(const BodyShape& p_shape, b3Vec3 p_offset) {
		points.resize(p_shape.points.size());
		for (size_t i = 0; i < points.size(); i++) {
			points[i] = b3Add(p_shape.points[i], p_offset);
		}
		proxy.points = points.data();
		proxy.count = (int)points.size();
		proxy.radius = p_shape.radius;
	}
};

void gather_contacts(const std::vector<BodyShape>& p_shapes, const Params& p_params, b3Vec3 p_offset, std::vector<Contact>& r_contacts) {
	for (size_t i = 0; i < p_shapes.size(); i++) {
		const MovedShape moved(p_shapes[i], p_offset);
		CollectContext ctx{ &p_params, &moved.proxy, (int)i, &r_contacts };
		const b3ShapeProxy query{ moved.proxy.points, moved.proxy.count, moved.proxy.radius + p_params.margin };
		b3World_OverlapShape(p_params.world, b3Vec3_zero, &query, p_params.filter, collect_callback, &ctx);
	}
}

// Whether the body is closer than p_margin to anything (p_margin 0: the surfaces really cross). The contacts are exact, not
// the speculative tolerance of b3World_OverlapShape.
bool overlaps_any_margin(const std::vector<BodyShape>& p_shapes, const Params& p_params, b3Vec3 p_offset, float p_margin) {
	Params exact = p_params;
	exact.margin = p_margin;
	std::vector<Contact> contacts;
	gather_contacts(p_shapes, exact, p_offset, contacts);
	for (const Contact& contact : contacts) {
		if (contact.separation < p_margin) {
			return true;
		}
	}
	return false;
}

// --- The sweep: conservative advancement on exact separations ----------------------------------------------------------
//
// Box3D's shape cast reports contact early (within the linear slop, and by the rounded proxies' radius handling), and a cast that
// starts within the slop of a surface answers "overlapped at the origin". The motion is therefore swept here with the exact
// contacts of the motion core: at a pose, every piece within the window gives a separating direction n (collider to body) and a
// gap s, and the body can travel s / closing along the motion before that piece is reached, where closing = -motion . n (a piece
// that does not close never blocks). The smallest such step is safe, so the sweep cannot tunnel, and a pose is never skipped.
constexpr float SWEEP_WINDOW = 0.04f;
constexpr float SWEEP_TOLERANCE = 1.0e-5f;
constexpr int SWEEP_ITERATIONS = 96;

// How far a closing piece is kept from the body. A cast query keeps the margin from everything. A body's motion keeps it from what
// it runs into head on (a floor it falls onto, a wall it walks into, which is what makes it rest a margin above the floor as on
// Jolt) and less the more the surface is only grazed: a body wedged between two corners, or sliding up a ramp, moves up to the
// surface as an exact cast would, because there a skin would stop the slide dead. Starting closer than the skin keeps that
// closeness (cap): the body is not asked to back off by travelling.
struct SkinRule {
	float skin = 0.0f;
	float cap = FLT_MAX;
	bool graded = false;
};

struct Sweep {
	const std::vector<BodyShape>* shapes;
	Params params; // margin = the window
	b3Vec3 origin;
	b3Vec3 motion;
	float length;
	float window;
	std::vector<b3BodyId> ignored;

	bool is_ignored(b3ShapeId p_shape) const {
		if (ignored.empty()) {
			return false;
		}
		const b3BodyId body = b3Shape_GetBody(p_shape);
		for (const b3BodyId& other : ignored) {
			if (B3_ID_EQUALS(other, body)) {
				return true;
			}
		}
		return false;
	}

	void contacts_at(float p_fraction, std::vector<Contact>& r_contacts) const {
		r_contacts.clear();
		gather_contacts(*shapes, params, b3MulAdd(origin, p_fraction, motion), r_contacts);
		if (!ignored.empty()) {
			r_contacts.erase(std::remove_if(r_contacts.begin(), r_contacts.end(), [this](const Contact& p_contact) {
				return is_ignored(p_contact.shape);
			}), r_contacts.end());
		}
	}

	// Metres per unit of fraction at which the contact's gap closes (negative: it opens).
	float closing(const Contact& p_contact) const {
		return -b3Dot(motion, p_contact.normal);
	}

	bool closes(const Contact& p_contact) const {
		// Below a ten-thousandth of the motion the normal is numerical noise: a flat floor is not closing on a sideways step.
		return closing(p_contact) > 1.0e-4f * length;
	}
};

void init_sweep(Sweep& r_sweep, const std::vector<BodyShape>& p_shapes, const Params& p_params, b3Vec3 p_origin, b3Vec3 p_motion, float p_skin) {
	r_sweep.shapes = &p_shapes;
	r_sweep.params = p_params;
	r_sweep.window = std::max(SWEEP_WINDOW, p_skin + 0.03f);
	r_sweep.params.margin = r_sweep.window;
	r_sweep.origin = p_origin;
	r_sweep.motion = p_motion;
	r_sweep.length = b3Length(p_motion);
}

float skin_of(const Sweep& p_sweep, const SkinRule& p_rule, const Contact& p_contact) {
	if (p_rule.skin <= 0.0f) {
		return 0.0f;
	}
	float skin = p_rule.skin;
	if (p_rule.graded) {
		const float head_on = p_sweep.closing(p_contact) / p_sweep.length; // 1: straight into the surface
		skin *= std::min(1.0f, std::max(0.0f, (head_on - 0.5f) * 2.0f));
	}
	return std::min(skin, p_rule.cap);
}

// A body that starts closer than the wanted skin keeps the closeness it has (the rule's cap).
SkinRule starting_rule(const Sweep& p_sweep, float p_skin) {
	SkinRule rule;
	rule.skin = p_skin;
	rule.graded = true;
	std::vector<Contact> contacts;
	p_sweep.contacts_at(0.0f, contacts);
	for (const Contact& contact : contacts) {
		if (p_sweep.closes(contact) && skin_of(p_sweep, rule, contact) > 0.0f) {
			rule.cap = std::min(rule.cap, std::max(0.0f, contact.separation));
		}
	}
	return rule;
}

// The first fraction at or after p_start where a closing piece is within p_skin, or -1 when the motion reaches 1 without one.
float advance(const Sweep& p_sweep, float p_start, const SkinRule& p_rule) {
	std::vector<Contact> contacts;
	float t = p_start;
	float last_ok = p_start;
	for (int iteration = 0; iteration < SWEEP_ITERATIONS; iteration++) {
		p_sweep.contacts_at(t, contacts);
		// Anything not in the window is at least a window away.
		float step = p_sweep.window / p_sweep.length;
		bool blocked = false;
		bool overshot = false;
		for (const Contact& contact : contacts) {
			if (!p_sweep.closes(contact)) {
				continue;
			}
			const float gap = contact.separation - skin_of(p_sweep, p_rule, contact);
			if (gap < -SWEEP_TOLERANCE) {
				overshot = true;
			}
			if (gap <= SWEEP_TOLERANCE) {
				blocked = true;
				continue;
			}
			step = std::min(step, gap / p_sweep.closing(contact));
		}
		if (overshot && t > last_ok) {
			// The separating direction was not the closest one and the step went past the skin: bracket it again.
			float lo = last_ok;
			float hi = t;
			for (int i = 0; i < 12; i++) {
				const float mid = 0.5f * (lo + hi);
				p_sweep.contacts_at(mid, contacts);
				bool past = false;
				for (const Contact& contact : contacts) {
					if (p_sweep.closes(contact) && contact.separation - skin_of(p_sweep, p_rule, contact) < -SWEEP_TOLERANCE) {
						past = true;
						break;
					}
				}
				(past ? hi : lo) = mid;
			}
			return lo;
		}
		if (blocked) {
			return t;
		}
		last_ok = t;
		if (t + step >= 1.0f) {
			return -1.0f;
		}
		t += step;
	}
	return last_ok;
}

} // namespace

bool convex_contact(const b3ShapeProxy& p_a, const b3ShapeProxy& p_b, float p_margin, b3Vec3& r_normal_a_to_b, float& r_separation, b3Vec3& r_point_on_a) {
	const float radius_sum = p_a.radius + p_b.radius;

	b3DistanceInput input{};
	input.proxyA = b3ShapeProxy{ p_a.points, p_a.count, 0.0f };
	input.proxyB = b3ShapeProxy{ p_b.points, p_b.count, 0.0f };
	input.transform = b3Transform_identity;
	input.useRadii = false;
	b3SimplexCache cache{};
	b3Simplex simplexes[GJK_SIMPLEX_CAPACITY];
	const b3DistanceOutput output = b3ShapeDistance(&input, &cache, simplexes, GJK_SIMPLEX_CAPACITY);

	if (output.distance > CONTACT_EPSILON) {
		const float separation = output.distance - radius_sum;
		if (separation >= p_margin) {
			return false;
		}
		const b3Vec3 normal = b3MulSV(1.0f / output.distance, b3Sub(output.pointB, output.pointA));
		r_normal_a_to_b = normal;
		r_separation = separation;
		r_point_on_a = b3MulAdd(output.pointA, p_a.radius, normal);
		return true;
	}

	// The cores touch or cross.
	if (output.simplexCount > 0) {
		const b3Simplex& last = simplexes[output.simplexCount - 1];
		if (last.count == 4) {
			Epa* epa = new Epa();
			epa->a = &p_a;
			epa->b = &p_b;
			b3Vec3 normal_m;
			float depth;
			b3Vec3 point_on_a;
			const bool solved = epa->solve(last, normal_m, depth, point_on_a);
			delete epa;
			if (solved) {
				r_normal_a_to_b = b3Neg(normal_m);
				r_separation = -depth - radius_sum;
				r_point_on_a = b3MulAdd(point_on_a, p_a.radius, r_normal_a_to_b);
				return r_separation < p_margin;
			}
		}
	}

	// The cores are flat against each other (or EPA had nothing to grow): leave along the line between the centres. The
	// cores themselves overlap by nothing, so the depth is the radii alone.
	const b3Vec3 center_a = centroid(p_a.points, p_a.count);
	const b3Vec3 center_b = centroid(p_b.points, p_b.count);
	b3Vec3 normal = b3Normalize(b3Sub(center_b, center_a));
	if (b3LengthSquared(normal) < 0.5f) {
		normal = b3Vec3{ 0.0f, 1.0f, 0.0f };
	}
	r_normal_a_to_b = normal;
	r_separation = -radius_sum;
	r_point_on_a = b3MulAdd(center_a, p_a.radius, normal);
	return r_separation < p_margin;
}

void collect_contacts(const Params& p_params, const b3ShapeProxy& p_proxy, int p_local_shape, std::vector<Contact>& r_contacts) {
	CollectContext ctx{ &p_params, &p_proxy, p_local_shape, &r_contacts };
	const b3ShapeProxy query{ p_proxy.points, p_proxy.count, p_proxy.radius + p_params.margin };
	b3World_OverlapShape(p_params.world, b3Vec3_zero, &query, p_params.filter, collect_callback, &ctx);
}

bool test_motion(const std::vector<BodyShape>& p_shapes, const Params& p_params_in, Result& r_result) {
	Params params = p_params_in;
	params.margin = std::max(params.margin, 0.0001f);
	const float margin = params.margin;
	const b3Vec3 motion = params.motion;
	const float motion_length = b3Length(motion);

	r_result = Result();
	r_result.travel = motion;
	if (p_shapes.empty()) {
		return false;
	}

	// 1. Recover. Contacts closer than three quarters of the margin push the body out to it (Jolt recovers to the margin, a
	// share of it per iteration); the pushes of one iteration are applied one after another, each contact seeing the ones
	// before it, so contacts that share a normal do not add up.
	const float rest_gap = 0.75f * margin;
	const float dead_zone = 0.01f * margin;
	b3Vec3 offset = b3Vec3_zero;
	std::vector<Contact> contacts;
	for (int iteration = 0; iteration < params.recovery_iterations; iteration++) {
		contacts.clear();
		gather_contacts(p_shapes, params, offset, contacts);
		b3Vec3 push = b3Vec3_zero;
		bool moved = false;
		for (const Contact& contact : contacts) {
			const float separation = contact.separation + b3Dot(push, contact.normal);
			const float needed = rest_gap - separation;
			if (needed > dead_zone) {
				push = b3MulAdd(push, needed, contact.normal);
				moved = true;
			}
		}
		if (!moved) {
			break;
		}
		offset = b3Add(offset, push);
		r_result.recovered = true;
	}
	r_result.recovery = offset;

	// 2. Cast. The sweep stops the body one skin (the margin, or less when it started closer) short of the first surface that
	// closes in on it, so a body walking down onto a floor rests a margin above it as on Jolt. The unsafe fraction is the first
	// exact overlap after that.
	float safe_fraction = 1.0f;
	float unsafe_fraction = 1.0f;
	bool hit = false;
	if (motion_length > FLT_EPSILON) {
		Sweep sweep;
		init_sweep(sweep, p_shapes, params, offset, motion, margin);
		const float stop = advance(sweep, 0.0f, starting_rule(sweep, margin));
		if (stop >= 0.0f) {
			hit = true;
			safe_fraction = stop;
			const float touch = advance(sweep, stop, SkinRule());
			if (touch < 0.0f) {
				unsafe_fraction = 1.0f; // it only comes within the margin
			} else if (overlaps_any_margin(p_shapes, params, b3MulAdd(offset, touch, motion), 0.0f)) {
				unsafe_fraction = touch;
			} else {
				float lo = touch;
				float hi = std::min(1.0f, touch + 0.002f / motion_length);
				if (overlaps_any_margin(p_shapes, params, b3MulAdd(offset, hi, motion), 0.0f)) {
					for (int i = 0; i < 6; i++) {
						const float mid = 0.5f * (lo + hi);
						if (overlaps_any_margin(p_shapes, params, b3MulAdd(offset, mid, motion), 0.0f)) {
							hi = mid;
						} else {
							lo = mid;
						}
					}
				}
				unsafe_fraction = hi;
			}
			unsafe_fraction = std::max(unsafe_fraction, safe_fraction);
		}
	}

	// 3. Collide at the unsafe pose.
	bool collided = false;
	if ((hit || (r_result.recovered && params.recovery_as_collision)) && params.max_collisions > 0) {
		contacts.clear();
		gather_contacts(p_shapes, params, b3MulAdd(offset, unsafe_fraction, motion), contacts);

		const b3Vec3 direction = motion_length > FLT_EPSILON ? b3MulSV(1.0f / motion_length, motion) : b3Vec3_zero;
		std::vector<Collision> collisions;
		for (const Contact& contact : contacts) {
			const float depth = margin - contact.separation;
			if (depth <= 0.0f) {
				continue;
			}
			// Contacts that do not oppose the motion would only be ghosts for the slide.
			if (motion_length > FLT_EPSILON && b3Dot(direction, contact.normal) >= -DIRECTION_EPSILON) {
				continue;
			}
			// One collision per collider shape and normal: a flat surface made of many triangles reports its deepest.
			bool merged = false;
			for (Collision& existing : collisions) {
				if (B3_ID_EQUALS(existing.shape, contact.shape) && existing.local_shape == contact.local_shape &&
						b3Dot(existing.normal, contact.normal) > 0.9998f) {
					if (depth > existing.depth) {
						existing.depth = depth;
						existing.point = contact.point;
					}
					merged = true;
					break;
				}
			}
			if (!merged) {
				Collision collision;
				collision.shape = contact.shape;
				collision.normal = contact.normal;
				collision.point = contact.point;
				collision.depth = depth;
				collision.local_shape = contact.local_shape;
				collisions.push_back(collision);
			}
		}
		std::stable_sort(collisions.begin(), collisions.end(), [](const Collision& p_a, const Collision& p_b) {
			return p_a.depth > p_b.depth;
		});
		const int count = std::min({ (int)collisions.size(), params.max_collisions, MAX_COLLISIONS });
		for (int i = 0; i < count; i++) {
			r_result.collisions[i] = collisions[i];
		}
		r_result.collision_count = count;
		collided = count > 0;
	}

	if (collided) {
		r_result.travel = b3MulAdd(offset, safe_fraction, motion);
		r_result.remainder = b3MulSV(1.0f - safe_fraction, motion);
		r_result.depth = r_result.collisions[0].depth;
		r_result.safe_fraction = safe_fraction;
		r_result.unsafe_fraction = unsafe_fraction;
	} else {
		r_result.travel = b3Add(offset, motion);
		r_result.remainder = b3Vec3_zero;
		r_result.depth = 0.0f;
		r_result.safe_fraction = 1.0f;
		r_result.unsafe_fraction = 1.0f;
		r_result.collision_count = 0;
	}
	return collided;
}

bool cast_shapes(const std::vector<BodyShape>& p_shapes, const Params& p_params, float& r_safe, float& r_unsafe) {
	r_safe = 1.0f;
	r_unsafe = 1.0f;
	const float skin = std::max(0.0f, p_params.margin);
	const float length = b3Length(p_params.motion);
	if (p_shapes.empty() || length <= FLT_EPSILON) {
		return false;
	}
	Sweep sweep;
	init_sweep(sweep, p_shapes, p_params, b3Vec3_zero, p_params.motion, skin);

	// A body the shape already touches (within the margin) takes no part, as on Jolt.
	{
		std::vector<Contact> contacts;
		sweep.contacts_at(0.0f, contacts);
		for (const Contact& contact : contacts) {
			if (contact.separation < skin) {
				const b3BodyId body = b3Shape_GetBody(contact.shape);
				bool known = false;
				for (const b3BodyId& other : sweep.ignored) {
					known |= B3_ID_EQUALS(other, body);
				}
				if (!known) {
					sweep.ignored.push_back(body);
				}
			}
		}
	}

	SkinRule rule;
	rule.skin = skin;
	const float stop = advance(sweep, 0.0f, rule);
	if (stop < 0.0f) {
		return false;
	}
	r_safe = stop;
	float lo = stop;
	float hi = std::min(1.0f, stop + 0.002f / length);
	std::vector<Contact> contacts;
	auto blocked = [&](float p_fraction) {
		sweep.contacts_at(p_fraction, contacts);
		for (const Contact& contact : contacts) {
			if (contact.separation < skin) {
				return true;
			}
		}
		return false;
	};
	if (blocked(hi)) {
		for (int i = 0; i < 6; i++) {
			const float mid = 0.5f * (lo + hi);
			(blocked(mid) ? hi : lo) = mid;
		}
	}
	r_unsafe = std::max(hi, stop);
	return true;
}

bool shape_contacts(const Params& p_params, const b3ShapeProxy& p_proxy, int p_local_shape, b3ShapeId p_shape_id, std::vector<Contact>& r_contacts) {
	switch (b3Shape_GetType(p_shape_id)) {
		case b3_sphereShape:
		case b3_capsuleShape:
		case b3_hullShape:
		case b3_meshShape:
		case b3_voxelGridShape: {
			CollectContext ctx{ &p_params, &p_proxy, p_local_shape, &r_contacts };
			collect_callback(p_shape_id, &ctx);
			return true;
		}
		default:
			return false;
	}
}

bool overlaps_any(const std::vector<BodyShape>& p_shapes, const Params& p_params, b3Vec3 p_offset) {
	return overlaps_any_margin(p_shapes, p_params, p_offset, std::max(0.0f, p_params.margin));
}

} // namespace b3m

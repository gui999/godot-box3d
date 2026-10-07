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
			// The mesh answers per triangle through the proxy grown by the margin: depth is then the push that leaves the
			// margin, so the separation is margin - depth.
			const b3Mesh mesh = b3Shape_GetMesh(p_shape_id);
			const b3ShapeProxy grown{ body.points, body.count, body.radius + margin };
			b3MeshRecoverResult results[16];
			const int count = b3RecoverMesh(&mesh, body_transform, &grown, results, 16);
			for (int i = 0; i < count; i++) {
				Contact contact;
				contact.shape = p_shape_id;
				contact.normal = results[i].normal;
				contact.point = results[i].point;
				contact.separation = margin - results[i].depth;
				contact.local_shape = ctx->local_shape;
				ctx->contacts->push_back(contact);
			}
			return true;
		}

		case b3_voxelGridShape: {
			const b3ShapeProxy grown{ body.points, body.count, body.radius + margin };
			// One exact contact per box (a post's corner leaves along its diagonal, not along an axis of its bounds); depth is
			// the push that leaves the margin, so the separation is margin - depth.
			b3VoxelContact results[32];
			const int count = b3CollideVoxelGrid(b3Shape_GetVoxelGrid(p_shape_id), body_transform, &grown, results, 32);
			for (int i = 0; i < count; i++) {
				Contact contact;
				contact.shape = p_shape_id;
				contact.normal = results[i].normal;
				contact.point = results[i].point;
				contact.separation = margin - results[i].depth;
				contact.local_shape = ctx->local_shape;
				ctx->contacts->push_back(contact);
			}
			return true;
		}

		default:
			// Height fields and compounds are cast against but have no contact answer yet.
			return true;
	}
}

// A shape cast hits a mesh from either side, but a mesh without backface collision is solid on the front of its triangles only.
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
	const b3Transform body_transform = b3Body_GetTransform(b3Shape_GetBody(p_shape_id));
	return b3Dot(b3RotateVector(body_transform.q, face_normal), p_normal) < 0.0f;
}

struct CastContext {
	const Params* params;
	bool hit = false;
	float fraction = 1.0f;
};

float cast_callback(b3ShapeId p_shape_id, b3Pos, b3Vec3 p_normal, float p_fraction, uint64_t, int p_triangle_index, int, void* p_context) {
	auto* ctx = static_cast<CastContext*>(p_context);
	if (!accepted(*ctx->params, p_shape_id)) {
		return -1.0f;
	}
	if (is_backside_mesh_hit(p_shape_id, p_triangle_index, p_normal)) {
		return -1.0f;
	}
	// Box3D reports a cast that starts overlapped as a hit at the origin with no normal; recovery owns that case.
	if (p_fraction <= FLT_EPSILON && b3LengthSquared(p_normal) < 0.25f) {
		return -1.0f;
	}
	ctx->hit = true;
	ctx->fraction = std::min(ctx->fraction, p_fraction);
	return p_fraction;
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

// Whether the body overlaps anything at all. b3World_OverlapShape tolerates a fraction of a slop, so the exact answer comes
// from the contacts with no margin: a contact then means the surfaces really cross.
bool overlaps_any(const std::vector<BodyShape>& p_shapes, const Params& p_params, b3Vec3 p_offset) {
	Params exact = p_params;
	exact.margin = 0.0f;
	std::vector<Contact> contacts;
	gather_contacts(p_shapes, exact, p_offset, contacts);
	for (const Contact& contact : contacts) {
		if (contact.separation < 0.0f) {
			return true;
		}
	}
	return false;
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
	if (-radius_sum >= p_margin) {
		return false;
	}

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

	// 1. Recover. Contacts closer than half the margin push the body out to half the margin; the pushes of one iteration
	// are applied one after another, each contact seeing the ones before it, so contacts that share a normal do not add up.
	const float rest_gap = 0.5f * margin;
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

	// 2. Cast.
	float safe_fraction = 1.0f;
	float unsafe_fraction = 1.0f;
	bool hit = false;
	if (motion_length > FLT_EPSILON) {
		float cast_fraction = 1.0f;
		for (const BodyShape& shape : p_shapes) {
			const MovedShape moved(shape, offset);
			CastContext ctx{ &params };
			b3World_CastShape(params.world, b3Vec3_zero, &moved.proxy, motion, params.filter, cast_callback, &ctx);
			if (ctx.hit && ctx.fraction < cast_fraction) {
				cast_fraction = ctx.fraction;
				hit = true;
			}
		}

		if (hit) {
			// Box3D's cast stops about a slop short of touching (or a slop into the rounded shapes); refine it to the exact
			// overlap boundary.
			const float slop_fraction = 2.0f * B3_LINEAR_SLOP / motion_length;
			float lo = 0.0f;
			float hi = std::min(1.0f, cast_fraction);
			bool bracketed = overlaps_any(p_shapes, params, b3MulAdd(offset, hi, motion));
			if (!bracketed && hi < 1.0f) {
				hi = std::min(1.0f, hi + slop_fraction);
				bracketed = overlaps_any(p_shapes, params, b3MulAdd(offset, hi, motion));
			}
			if (!bracketed) {
				safe_fraction = unsafe_fraction = cast_fraction;
			} else if (overlaps_any(p_shapes, params, offset)) {
				// Still embedded after recovery: no free travel.
				safe_fraction = 0.0f;
				unsafe_fraction = 0.0f;
			} else {
				const int steps = std::max(4, std::min(16, (int)(std::log(1000.0f * motion_length) / 0.6931472f)));
				for (int i = 0; i < steps; i++) {
					const float mid = 0.5f * (lo + hi);
					if (overlaps_any(p_shapes, params, b3MulAdd(offset, mid, motion))) {
						hi = mid;
					} else {
						lo = mid;
					}
				}
				safe_fraction = lo;
				unsafe_fraction = hi;
			}
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

} // namespace b3m

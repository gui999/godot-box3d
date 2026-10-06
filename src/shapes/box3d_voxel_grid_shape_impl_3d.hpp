#pragma once

#include "box3d_shape_impl_3d.hpp"
#include "box3d_voxel_grid_data.hpp"

#include <box3d/id.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

// Per attachment of a voxel grid shape to a body: the live b3 shapes of every collidable cell. Heap
// allocated and shared by the owner's Box3DShapeInstance3D, so its address is stable and serves as every
// cell shape's b3 userData; `index` is kept equal to the instance's index in its owner (renumbered when
// an earlier shape is removed), which is how a b3 shape found by a query or contact maps back to the one
// Godot shape index of the grid.
struct Box3DVoxelGridInstance3D {
	Box3DShapedObjectImpl3D* owner = nullptr;
	uint32_t index = 0;
	// Padded cell -> the hull shapes of its module's boxes.
	std::unordered_map<int32_t, std::vector<b3ShapeId>> cell_shapes;
};

// A PhysicsServer3D custom shape holding a voxel grid (see Box3DVoxelGridData for the contract). This
// backend builds one Box3D hull per box of every collidable cell, on static bodies only. The data
// handling lives in Box3DVoxelGridData; the Box3D shapes are made by Box3DShapedObjectImpl3D.
class Box3DVoxelGridShapeImpl3D final : public Box3DShapeImpl3D {
public:
	~Box3DVoxelGridShapeImpl3D() override;

	ShapeType get_type() const override { return PhysicsServer3D::SHAPE_CUSTOM; }

	Variant get_data() const override { return data.to_dictionary(); }

	void set_data(const Variant& p_data) override;

	AABB get_aabb() const override { return data.get_aabb(); }

	const Box3DVoxelGridData& get_grid() const { return data; }

	// Live b3 shapes created by every voxel grid shape (the diagnostics line prints it).
	static int64_t get_live_shape_count() { return live_shape_count; }

	static void add_live_shapes(int64_t p_delta) { live_shape_count += p_delta; }

private:
	Box3DVoxelGridData data;

	static inline int64_t live_shape_count = 0;
};

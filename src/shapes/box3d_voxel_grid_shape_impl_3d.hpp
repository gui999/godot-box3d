#pragma once

#include "box3d_shape_impl_3d.hpp"
#include "box3d_voxel_grid_data.hpp"

#include <cstdint>

// A PhysicsServer3D custom shape holding a voxel grid (see Box3DVoxelGridData for the contract). This
// backend makes one Box3D voxel grid shape (b3_voxelGridShape) per attachment to a body, static bodies
// only; the boxes are not shapes. The data handling lives in Box3DVoxelGridData, the shared per-module
// derived data in Box3DVoxelModuleCache and the Box3D shape in Box3DShapedObjectImpl3D.
class Box3DVoxelGridShapeImpl3D final : public Box3DShapeImpl3D {
public:
	~Box3DVoxelGridShapeImpl3D() override;

	ShapeType get_type() const override { return PhysicsServer3D::SHAPE_CUSTOM; }

	Variant get_data() const override { return data.to_dictionary(); }

	void set_data(const Variant& p_data) override;

	AABB get_aabb() const override { return data.get_aabb(); }

	const Box3DVoxelGridData& get_grid() const { return data; }

	// Live b3 voxel grid shapes created by every voxel grid shape (the diagnostics line prints it).
	static int64_t get_live_shape_count() { return live_shape_count; }

	static void add_live_shapes(int64_t p_delta) { live_shape_count += p_delta; }

private:
	Box3DVoxelGridData data;

	static inline int64_t live_shape_count = 0;
};

#include "box3d_voxel_grid_shape_impl_3d.hpp"

#include "../objects/box3d_shaped_object_impl_3d.hpp"

#include <godot_cpp/core/error_macros.hpp>

#include <vector>

Box3DVoxelGridShapeImpl3D::~Box3DVoxelGridShapeImpl3D() {
	// Freeing the shape takes its b3 shapes off every body that still uses it.
	std::vector<Box3DShapedObjectImpl3D*> attached;
	for (Box3DShapedObjectImpl3D* owner : owners) {
		attached.push_back(owner);
	}
	for (Box3DShapedObjectImpl3D* owner : attached) {
		owner->detach_shape(this);
	}
}

void Box3DVoxelGridShapeImpl3D::set_data(const Variant& p_data) {
	ERR_FAIL_COND_MSG(p_data.get_type() != Variant::DICTIONARY, "A voxel grid shape takes a Dictionary.");
	const Dictionary dictionary = p_data;
	std::vector<Box3DShapedObjectImpl3D*> attached;
	for (Box3DShapedObjectImpl3D* owner : owners) {
		attached.push_back(owner);
	}
	if (dictionary.has("update")) {
		std::vector<int32_t> changed_cells;
		if (!data.update(dictionary, changed_cells) || changed_cells.empty()) {
			return;
		}
		for (Box3DShapedObjectImpl3D* owner : attached) {
			owner->update_voxel_grid(this, &changed_cells);
		}
		return;
	}
	if (!data.set(dictionary)) {
		return;
	}
	for (Box3DShapedObjectImpl3D* owner : attached) {
		owner->update_voxel_grid(this, nullptr);
	}
}

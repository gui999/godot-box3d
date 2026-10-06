#include "box3d_shape_instance_3d.hpp"

#include "box3d_voxel_grid_shape_impl_3d.hpp"

void Box3DShapeInstance3D::set_index(uint32_t p_index) {
	index = p_index;
	if (grid_instance) {
		grid_instance->index = p_index;
	}
}

#pragma once

#include <box3d/collision.h>

// Terra Prime: voxel grid modules (a cell's solid boxes and everything Box3D derives from them) shared by
// every grid that places them. Deriving a module is the heavy part of building a voxel grid shape, so it
// happens once per distinct set of boxes: a cache keyed by the boxes' bytes (the counterpart of Jolt's
// JoltCustomVoxelGridShape::get_module).
class Box3DVoxelModuleCache {
public:
	// A module for the boxes (six floats a box: min xyz, max xyz in metres in the cell frame), from the
	// cache or derived now. The caller owns one reference and releases it with b3ReleaseVoxelGridModule.
	// Degenerate boxes are skipped. Never null: a set of boxes Box3D refuses gives an empty module and a
	// printed error. Thread safe: modules may be acquired ahead of time on a worker thread.
	static b3VoxelGridModule* acquire(const float* p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels);
	// A merged body's region module (D755) with the caller's covered faces, from the cache or derived now: a body
	// that leaves the space and returns (its plot out of reach and back) finds its regions as they were. Boxes are
	// kept as given (the faces index them). Null when Box3D refuses the boxes. The caller owns one reference.
	static b3VoxelGridModule* acquire_region(const float* p_boxes, const uint8_t* p_faces, int p_box_count, float p_cell_meters, int p_cell_voxels);
};

#pragma once

#include <godot_cpp/variant/aabb.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/vector3i.hpp>

#include <cstdint>
#include <vector>

using namespace godot;

// Terra Prime voxel grid shape data: the parsed, validated form of the Dictionary a game hands to
// PhysicsServer3D.shape_set_data on a custom shape, independent of how a backend turns it into
// Box3D shapes (a native voxel grid shape per attachment today; it reuses this class unchanged).
// The contract is Godot's Jolt voxel grid shape (jolt_voxel_grid_shape_3d.cpp): cell_size, cell_voxels,
// size, origin, cells (a module index or -1 per cell of the grid padded by a one-cell ring, the ring
// only for face cover and never collidable), box_offsets, boxes (six floats a box: min xyz, max xyz in
// metres in the cell's frame); an update dictionary carries `update` (padded cell, module) pairs plus
// the box_offsets/boxes of modules appended to the table.
// A merged body (Terra, D755) adds its collision boxes merged across pages, in regions of body_region_cells cells a side:
// body_region_offsets (one box offset a region, regions in (j * regions z + k) * regions x + i order, the region count
// plus one), body_region_boxes (six floats a box in its region's frame) and body_region_covered_faces (a mask a box).
// The cells then only give the exact occupancy. Its update dictionary may carry, beside the cells' update, body_regions
// (the regions it replaces) with their body_region_offsets, body_region_boxes and body_region_covered_faces.
class Box3DVoxelGridData {
public:
	bool is_valid() const { return valid; }

	// Replaces everything. On a validation failure prints the error, changes nothing and returns false.
	bool set(const Dictionary& p_data);

	// Applies an `update` dictionary. r_changed_cells receives the distinct padded cells whose module
	// changed. On a validation failure prints the error, changes nothing and returns false.
	bool update(const Dictionary& p_update, std::vector<int32_t>& r_changed_cells);

	// The stored data as the dictionary it was given, updates applied.
	Dictionary to_dictionary() const;

	AABB get_aabb() const { return AABB(origin, Vector3(size.x, size.y, size.z) * cell_size); }

	float get_cell_size() const { return cell_size; }

	int get_cell_voxels() const { return cell_voxels; }

	// The module table's size: the modules a grid shape has, the update dictionaries' additions included.
	int get_module_count() const { return (int)box_offsets.size() - 1; }

	// The padded cells' module indices, get_padded_cell_count() of them, in the layout of get_padded_index.
	const int32_t* get_padded_cells() const { return padded_cells.data(); }

	Vector3i get_size() const { return size; }

	Vector3 get_origin() const { return origin; }

	int get_padded_index(int p_i, int p_j, int p_k) const { return ((p_j + 1) * (size.z + 2) + (p_k + 1)) * (size.x + 2) + (p_i + 1); }

	// A padded cell's grid coordinates; a ring cell has a coordinate of -1 or the size.
	void get_padded_coordinates(int32_t p_padded_cell, int& r_i, int& r_j, int& r_k) const;

	bool is_ring_cell(int32_t p_padded_cell) const;

	// The cell's lower corner in the shape's frame.
	Vector3 get_cell_corner(int p_i, int p_j, int p_k) const { return origin + Vector3(p_i, p_j, p_k) * cell_size; }

	int32_t get_module_at(int32_t p_padded_cell) const { return padded_cells[(size_t)p_padded_cell]; }

	int32_t get_padded_cell_count() const { return (int32_t)padded_cells.size(); }

	// A module's boxes: r_count boxes of six floats (min xyz, max xyz) each.
	const float* get_module_boxes(int32_t p_module, int& r_count) const;
	// A merged body's regions (none when the grid is not a merged body).
	bool is_merged_body() const { return body_region_cells > 0; }
	int get_body_region_cells() const { return body_region_cells; }
	float get_body_region_meters() const { return body_region_cells * cell_size; }
	Vector3i get_body_region_size() const { return body_region_size; }
	int get_body_region_count() const { return (int)region_boxes.size(); }
	const std::vector<float>& get_region_boxes(int p_region) const { return region_boxes[(size_t)p_region]; }
	const std::vector<uint8_t>& get_region_covered_faces(int p_region) const { return region_covered_faces[(size_t)p_region]; }
	// The regions the last update replaced.
	const std::vector<int32_t>& get_changed_regions() const { return changed_regions; }

private:
	bool valid = false;
	float cell_size = 0.0f;
	int cell_voxels = 0;
	Vector3i size;
	Vector3 origin;
	std::vector<int32_t> padded_cells;
	std::vector<int32_t> box_offsets;
	std::vector<float> boxes;
	int body_region_cells = 0;
	Vector3i body_region_size;
	std::vector<std::vector<float>> region_boxes;
	std::vector<std::vector<uint8_t>> region_covered_faces;
	std::vector<int32_t> changed_regions;

	// Parses p_region_count regions of p_data into r_boxes and r_covered: all regions in order, or those p_regions names.
	// A region's boxes must lie in the part of it the grid covers (an edge region is cut by the grid's extent).
	bool parse_regions(const Dictionary& p_data, int p_region_count, const std::vector<int32_t>* p_regions, std::vector<std::vector<float>>& r_boxes,
			std::vector<std::vector<uint8_t>>& r_covered) const;
};

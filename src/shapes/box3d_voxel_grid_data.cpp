#include "box3d_voxel_grid_data.hpp"

#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/variant.hpp>

#include <algorithm>
#include <cmath>

namespace {

template <typename TPacked, typename TValue>
std::vector<TValue> to_vector(const TPacked& p_array) {
	const int64_t count = p_array.size();
	if (count == 0) {
		return {};
	}
	return std::vector<TValue>(p_array.ptr(), p_array.ptr() + count);
}

} // namespace

bool Box3DVoxelGridData::set(const Dictionary& p_data) {
	ERR_FAIL_COND_V(p_data.get("cell_size", Variant()).get_type() != Variant::FLOAT, false);
	ERR_FAIL_COND_V(p_data.get("cell_voxels", Variant()).get_type() != Variant::INT, false);
	ERR_FAIL_COND_V(p_data.get("size", Variant()).get_type() != Variant::VECTOR3I, false);
	ERR_FAIL_COND_V(p_data.get("origin", Variant()).get_type() != Variant::VECTOR3, false);
	ERR_FAIL_COND_V(p_data.get("cells", Variant()).get_type() != Variant::PACKED_INT32_ARRAY, false);
	ERR_FAIL_COND_V(p_data.get("box_offsets", Variant()).get_type() != Variant::PACKED_INT32_ARRAY, false);
	ERR_FAIL_COND_V(p_data.get("boxes", Variant()).get_type() != Variant::PACKED_FLOAT32_ARRAY, false);

	const float new_cell_size = p_data["cell_size"];
	const int new_cell_voxels = p_data["cell_voxels"];
	const Vector3i new_size = p_data["size"];
	const Vector3 new_origin = p_data["origin"];
	const PackedInt32Array cells = p_data["cells"];
	const PackedInt32Array offsets = p_data["box_offsets"];
	const PackedFloat32Array new_boxes = p_data["boxes"];
	ERR_FAIL_COND_V_MSG(new_cell_size <= 0.0f || new_cell_voxels < 1 || new_cell_voxels > 32, false, "A voxel grid shape needs a positive cell size and 1 to 32 voxels a cell side.");
	ERR_FAIL_COND_V_MSG(new_size.x < 1 || new_size.y < 1 || new_size.z < 1, false, "A voxel grid shape needs at least one cell.");
	ERR_FAIL_COND_V_MSG(cells.size() != (int64_t)(new_size.x + 2) * (new_size.y + 2) * (new_size.z + 2), false, "A voxel grid shape's cells must cover the grid and its one-cell ring.");
	ERR_FAIL_COND_V_MSG(offsets.is_empty() || offsets[0] != 0 || (int64_t)offsets[offsets.size() - 1] * 6 != new_boxes.size(), false, "A voxel grid shape's box offsets must start at 0 and end at the box count.");
	const int module_count = (int)offsets.size() - 1;
	for (int m = 0; m < module_count; m++) {
		ERR_FAIL_COND_V_MSG(offsets[m + 1] < offsets[m], false, "A voxel grid shape's box offsets must not decrease.");
	}
	for (int64_t i = 0; i < cells.size(); i++) {
		ERR_FAIL_COND_V_MSG(cells[i] < -1 || cells[i] >= module_count, false, "A voxel grid shape's cell names a module it does not have.");
	}

	int new_region_cells = 0;
	Vector3i new_region_size;
	std::vector<std::vector<float>> new_region_boxes;
	std::vector<std::vector<uint8_t>> new_region_covered;
	if (p_data.has("body_region_cells")) {
		ERR_FAIL_COND_V(p_data["body_region_cells"].get_type() != Variant::INT, false);
		new_region_cells = p_data["body_region_cells"];
		ERR_FAIL_COND_V_MSG(new_region_cells < 1, false, "A merged body's regions need at least one cell a side.");
		new_region_size = Vector3i((new_size.x + new_region_cells - 1) / new_region_cells, (new_size.y + new_region_cells - 1) / new_region_cells,
				(new_size.z + new_region_cells - 1) / new_region_cells);
		Box3DVoxelGridData parsing;
		parsing.cell_size = new_cell_size;
		parsing.size = new_size;
		parsing.body_region_cells = new_region_cells;
		parsing.body_region_size = new_region_size;
		if (!parsing.parse_regions(p_data, new_region_size.x * new_region_size.y * new_region_size.z, nullptr, new_region_boxes, new_region_covered)) {
			return false;
		}
	}
	body_region_cells = new_region_cells;
	body_region_size = new_region_size;
	region_boxes = std::move(new_region_boxes);
	region_covered_faces = std::move(new_region_covered);
	changed_regions.clear();
	cell_size = new_cell_size;
	cell_voxels = new_cell_voxels;
	size = new_size;
	origin = new_origin;
	padded_cells = to_vector<PackedInt32Array, int32_t>(cells);
	box_offsets = to_vector<PackedInt32Array, int32_t>(offsets);
	boxes = to_vector<PackedFloat32Array, float>(new_boxes);
	valid = true;
	return true;
}

bool Box3DVoxelGridData::update(const Dictionary& p_update, std::vector<int32_t>& r_changed_cells) {
	r_changed_cells.clear();
	ERR_FAIL_COND_V_MSG(!valid, false, "A voxel grid shape's cells can only be updated once it has a grid.");
	ERR_FAIL_COND_V(p_update.get("update", Variant()).get_type() != Variant::PACKED_INT32_ARRAY, false);
	ERR_FAIL_COND_V(p_update.get("box_offsets", Variant()).get_type() != Variant::PACKED_INT32_ARRAY, false);
	ERR_FAIL_COND_V(p_update.get("boxes", Variant()).get_type() != Variant::PACKED_FLOAT32_ARRAY, false);
	const PackedInt32Array pairs = p_update["update"];
	const PackedInt32Array added_offsets = p_update["box_offsets"];
	const PackedFloat32Array added_boxes = p_update["boxes"];
	const int had = (int)box_offsets.size() - 1;
	ERR_FAIL_COND_V_MSG(pairs.size() % 2 != 0, false, "A voxel grid shape's update pairs a cell with a module.");
	ERR_FAIL_COND_V_MSG(added_offsets.is_empty() || added_offsets[0] != 0 || (int64_t)added_offsets[added_offsets.size() - 1] * 6 != added_boxes.size(), false, "A voxel grid shape update's box offsets must start at 0 and end at the box count.");
	const int added = (int)added_offsets.size() - 1;
	for (int m = 0; m < added; m++) {
		ERR_FAIL_COND_V_MSG(added_offsets[m + 1] < added_offsets[m], false, "A voxel grid shape update's box offsets must not decrease.");
		ERR_FAIL_COND_V_MSG(added_offsets[m + 1] - added_offsets[m] > cell_voxels * cell_voxels * cell_voxels, false, "A voxel grid shape's module has more boxes than a cell has voxels.");
	}
	for (int64_t p = 0; p < pairs.size(); p += 2) {
		ERR_FAIL_COND_V_MSG(pairs[p] < 0 || pairs[p] >= (int64_t)padded_cells.size(), false, "A voxel grid shape update names a cell outside the grid.");
		ERR_FAIL_COND_V_MSG(pairs[p + 1] < -1 || pairs[p + 1] >= had + added, false, "A voxel grid shape update names a module the shape does not have.");
	}

	std::vector<int32_t> regions;
	std::vector<std::vector<float>> replaced_boxes;
	std::vector<std::vector<uint8_t>> replaced_covered;
	if (p_update.has("body_regions")) {
		ERR_FAIL_COND_V_MSG(!is_merged_body(), false, "Only a merged body's grid has regions to replace.");
		ERR_FAIL_COND_V(p_update["body_regions"].get_type() != Variant::PACKED_INT32_ARRAY, false);
		const PackedInt32Array named = p_update["body_regions"];
		for (int64_t r = 0; r < named.size(); r++) {
			ERR_FAIL_COND_V_MSG(named[r] < 0 || named[r] >= get_body_region_count(), false, "A merged body's update names a region it does not have.");
			regions.push_back(named[r]);
		}
		if (!parse_regions(p_update, (int)regions.size(), &regions, replaced_boxes, replaced_covered)) {
			return false;
		}
	}

	const int32_t base = box_offsets[had];
	for (int m = 1; m <= added; m++) {
		box_offsets.push_back(base + added_offsets[m]);
	}
	if (added_boxes.size() > 0) {
		boxes.insert(boxes.end(), added_boxes.ptr(), added_boxes.ptr() + added_boxes.size());
	}
	for (int64_t p = 0; p < pairs.size(); p += 2) {
		int32_t& module = padded_cells[(size_t)pairs[p]];
		if (module != pairs[p + 1]) {
			module = pairs[p + 1];
			r_changed_cells.push_back(pairs[p]);
		}
	}
	std::sort(r_changed_cells.begin(), r_changed_cells.end());
	r_changed_cells.erase(std::unique(r_changed_cells.begin(), r_changed_cells.end()), r_changed_cells.end());
	changed_regions = regions;
	for (size_t r = 0; r < regions.size(); r++) {
		region_boxes[(size_t)regions[r]] = std::move(replaced_boxes[r]);
		region_covered_faces[(size_t)regions[r]] = std::move(replaced_covered[r]);
	}
	return true;
}

bool Box3DVoxelGridData::parse_regions(const Dictionary& p_data, int p_region_count, const std::vector<int32_t>* p_regions,
		std::vector<std::vector<float>>& r_boxes, std::vector<std::vector<uint8_t>>& r_covered) const {
	ERR_FAIL_COND_V(p_data.get("body_region_offsets", Variant()).get_type() != Variant::PACKED_INT32_ARRAY, false);
	ERR_FAIL_COND_V(p_data.get("body_region_boxes", Variant()).get_type() != Variant::PACKED_FLOAT32_ARRAY, false);
	ERR_FAIL_COND_V(p_data.get("body_region_covered_faces", Variant()).get_type() != Variant::PACKED_BYTE_ARRAY, false);
	const PackedInt32Array offsets = p_data["body_region_offsets"];
	const PackedFloat32Array boxes_in = p_data["body_region_boxes"];
	const PackedByteArray covered = p_data["body_region_covered_faces"];
	ERR_FAIL_COND_V_MSG(offsets.size() != (int64_t)p_region_count + 1 || offsets[0] != 0 || (int64_t)offsets[p_region_count] * 6 != boxes_in.size()
					|| covered.size() != boxes_in.size() / 6,
			false, "A merged body's region offsets must start at 0, give one offset a region and end at the box count, with a cover mask a box.");
	const float region_meters = get_body_region_meters();
	r_boxes.assign((size_t)p_region_count, {});
	r_covered.assign((size_t)p_region_count, {});
	for (int r = 0; r < p_region_count; r++) {
		ERR_FAIL_COND_V_MSG(offsets[r + 1] < offsets[r], false, "A merged body's region offsets must not decrease.");
		// The region's corner, in regions, from its index ((j * regions z + k) * regions x + i).
		const int index = p_regions != nullptr ? (*p_regions)[(size_t)r] : r;
		const int corner[3] = { index % body_region_size.x, index / (body_region_size.x * body_region_size.z), index / body_region_size.x % body_region_size.z };
		for (int32_t b = offsets[r]; b < offsets[r + 1]; b++) {
			for (int a = 0; a < 3; a++) {
				const float extent = std::min(region_meters, size[a] * cell_size - corner[a] * region_meters) + 0.001f;
				const float low = boxes_in[(int64_t)b * 6 + a], high = boxes_in[(int64_t)b * 6 + 3 + a];
				ERR_FAIL_COND_V_MSG(!std::isfinite(low) || !std::isfinite(high) || low < -0.001f || high > extent || high <= low, false,
						"A merged body's region box must lie in the part of its region the grid covers.");
			}
			ERR_FAIL_COND_V(covered[b] > 63, false);
		}
		r_boxes[(size_t)r].assign(boxes_in.ptr() + (int64_t)offsets[r] * 6, boxes_in.ptr() + (int64_t)offsets[r + 1] * 6);
		r_covered[(size_t)r].assign(covered.ptr() + offsets[r], covered.ptr() + offsets[r + 1]);
	}
	return true;
}

Dictionary Box3DVoxelGridData::to_dictionary() const {
	Dictionary data;
	if (!valid) {
		return data;
	}
	PackedInt32Array cells;
	cells.resize((int64_t)padded_cells.size());
	if (!padded_cells.empty()) {
		memcpy(cells.ptrw(), padded_cells.data(), padded_cells.size() * sizeof(int32_t));
	}
	PackedInt32Array offsets;
	offsets.resize((int64_t)box_offsets.size());
	if (!box_offsets.empty()) {
		memcpy(offsets.ptrw(), box_offsets.data(), box_offsets.size() * sizeof(int32_t));
	}
	PackedFloat32Array box_array;
	box_array.resize((int64_t)boxes.size());
	if (!boxes.empty()) {
		memcpy(box_array.ptrw(), boxes.data(), boxes.size() * sizeof(float));
	}
	data["cell_size"] = cell_size;
	data["cell_voxels"] = cell_voxels;
	data["size"] = size;
	data["origin"] = origin;
	data["cells"] = cells;
	data["box_offsets"] = offsets;
	data["boxes"] = box_array;
	if (is_merged_body()) {
		PackedInt32Array region_offsets;
		PackedFloat32Array region_box_array;
		PackedByteArray region_covered;
		region_offsets.push_back(0);
		for (size_t r = 0; r < region_boxes.size(); r++) {
			for (float value : region_boxes[r]) {
				region_box_array.push_back(value);
			}
			for (uint8_t mask : region_covered_faces[r]) {
				region_covered.push_back(mask);
			}
			region_offsets.push_back((int32_t)(region_box_array.size() / 6));
		}
		data["body_region_cells"] = body_region_cells;
		data["body_region_offsets"] = region_offsets;
		data["body_region_boxes"] = region_box_array;
		data["body_region_covered_faces"] = region_covered;
	}
	return data;
}

void Box3DVoxelGridData::get_padded_coordinates(int32_t p_padded_cell, int& r_i, int& r_j, int& r_k) const {
	const int row = size.x + 2;
	const int layer = size.z + 2;
	r_i = (p_padded_cell % row) - 1;
	r_k = ((p_padded_cell / row) % layer) - 1;
	r_j = (p_padded_cell / (row * layer)) - 1;
}

bool Box3DVoxelGridData::is_ring_cell(int32_t p_padded_cell) const {
	int i, j, k;
	get_padded_coordinates(p_padded_cell, i, j, k);
	return i < 0 || j < 0 || k < 0 || i >= size.x || j >= size.y || k >= size.z;
}

const float* Box3DVoxelGridData::get_module_boxes(int32_t p_module, int& r_count) const {
	r_count = 0;
	if (p_module < 0 || p_module + 1 >= (int32_t)box_offsets.size()) {
		return nullptr;
	}
	const int first = box_offsets[p_module];
	r_count = box_offsets[p_module + 1] - first;
	return boxes.data() + (size_t)first * 6;
}

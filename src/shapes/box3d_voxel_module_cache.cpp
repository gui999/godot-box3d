#include "box3d_voxel_module_cache.hpp"

#include <godot_cpp/core/error_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

struct CacheEntry {
	b3VoxelGridModule* module = nullptr;
	float cell_meters = 0.0f;
	int cell_voxels = 0;
	// A merged body's region module (D755) carries the caller's covered faces; an ordinary module derives its own.
	bool region = false;
	std::vector<uint8_t> faces;
};

std::mutex cache_mutex;
std::unordered_multimap<uint64_t, CacheEntry> cache;
size_t cache_sweep_at = 4096;

uint64_t hash_boxes(const float* p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	uint64_t hash = 14695981039346656037ull;
	auto mix = [&hash](const void* p_bytes, size_t p_size) {
		const uint8_t* bytes = static_cast<const uint8_t*>(p_bytes);
		for (size_t i = 0; i < p_size; i++) {
			hash = (hash ^ bytes[i]) * 1099511628211ull;
		}
	};
	mix(&p_cell_meters, sizeof(float));
	mix(&p_cell_voxels, sizeof(int));
	mix(p_boxes, (size_t)p_box_count * 6 * sizeof(float));
	return hash;
}

// A cached module with the same boxes, retained for the caller, or null. Call with the lock held.
b3VoxelGridModule* find(uint64_t p_hash, const float* p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	auto range = cache.equal_range(p_hash);
	for (auto it = range.first; it != range.second; ++it) {
		const CacheEntry& entry = it->second;
		if (!entry.region && entry.cell_meters == p_cell_meters && entry.cell_voxels == p_cell_voxels && b3VoxelGridModule_GetBoxCount(entry.module) == p_box_count &&
				(p_box_count == 0 || std::memcmp(b3VoxelGridModule_GetBoxes(entry.module), p_boxes, (size_t)p_box_count * 6 * sizeof(float)) == 0)) {
			b3RetainVoxelGridModule(entry.module);
			return entry.module;
		}
	}
	return nullptr;
}

} // namespace

b3VoxelGridModule* Box3DVoxelModuleCache::acquire(const float* p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	// Degenerate boxes carry no collision.
	std::vector<float> boxes;
	boxes.reserve((size_t)std::max(p_box_count, 0) * 6);
	for (int b = 0; b < p_box_count; b++) {
		const float* box = p_boxes + (size_t)b * 6;
		if (box[3] - box[0] <= 2e-6f || box[4] - box[1] <= 2e-6f || box[5] - box[2] <= 2e-6f) {
			continue;
		}
		boxes.insert(boxes.end(), box, box + 6);
	}
	const int box_count = (int)(boxes.size() / 6);
	const uint64_t hash = hash_boxes(boxes.data(), box_count, p_cell_meters, p_cell_voxels);
	{
		std::lock_guard<std::mutex> lock(cache_mutex);
		if (b3VoxelGridModule* cached = find(hash, boxes.data(), box_count, p_cell_meters, p_cell_voxels)) {
			return cached;
		}
	}

	// Derived outside the lock: it is the expensive part and Box3D only uses the allocator for it.
	b3VoxelGridModule* module = b3CreateVoxelGridModule(boxes.data(), box_count, p_cell_meters, p_cell_voxels);
	if (module == nullptr) {
		ERR_PRINT("Box3D: a voxel grid module's boxes are not valid (not finite or not positive in size); the module is left empty.");
		return b3CreateVoxelGridModule(nullptr, 0, p_cell_meters, p_cell_voxels);
	}

	std::lock_guard<std::mutex> lock(cache_mutex);
	// Another thread may have derived the same module meanwhile.
	if (b3VoxelGridModule* cached = find(hash, boxes.data(), box_count, p_cell_meters, p_cell_voxels)) {
		b3ReleaseVoxelGridModule(module);
		return cached;
	}
	// Modules no grid holds any more (hole-carved cells replaced since) leave the cache when it grows.
	if (cache.size() >= cache_sweep_at) {
		// Only this cache holds them (it retains under the lock, so none can be taken meanwhile); they are
		// freed on a thread of their own: terrain grids make tens of thousands of distinct modules, and
		// freeing a streamed-out region's at once held the thread that made the next grid 10–110 ms.
		std::vector<b3VoxelGridModule*> dead;
		for (auto it = cache.begin(); it != cache.end();) {
			if (b3VoxelGridModule_GetReferenceCount(it->second.module) == 1) {
				dead.push_back(it->second.module);
				it = cache.erase(it);
			} else {
				++it;
			}
		}
		if (!dead.empty()) {
			std::thread([dead = std::move(dead)]() {
				for (b3VoxelGridModule* module : dead) {
					b3ReleaseVoxelGridModule(module);
				}
			}).detach();
		}
		cache_sweep_at = std::max<size_t>(4096, cache.size() * 2);
	}
	b3RetainVoxelGridModule(module);
	cache.emplace(hash, CacheEntry{ module, p_cell_meters, p_cell_voxels });
	return module;
}

namespace {

// Word-wise FNV-1a: a region's boxes are megabytes, and the byte-wise mix above cost as much as a module build.
uint64_t hash_words(uint64_t p_hash, const void* p_bytes, size_t p_size) {
	const uint8_t* bytes = static_cast<const uint8_t*>(p_bytes);
	size_t i = 0;
	for (; i + 8 <= p_size; i += 8) {
		uint64_t word;
		std::memcpy(&word, bytes + i, 8);
		p_hash = (p_hash ^ word) * 1099511628211ull;
	}
	for (; i < p_size; i++) {
		p_hash = (p_hash ^ bytes[i]) * 1099511628211ull;
	}
	return p_hash;
}

b3VoxelGridModule* find_region(uint64_t p_hash, const float* p_boxes, const uint8_t* p_faces, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	auto range = cache.equal_range(p_hash);
	for (auto it = range.first; it != range.second; ++it) {
		const CacheEntry& entry = it->second;
		if (entry.region && entry.cell_meters == p_cell_meters && entry.cell_voxels == p_cell_voxels && b3VoxelGridModule_GetBoxCount(entry.module) == p_box_count &&
				(p_box_count == 0 || (std::memcmp(b3VoxelGridModule_GetBoxes(entry.module), p_boxes, (size_t)p_box_count * 6 * sizeof(float)) == 0 &&
						std::memcmp(entry.faces.data(), p_faces, (size_t)p_box_count) == 0))) {
			b3RetainVoxelGridModule(entry.module);
			return entry.module;
		}
	}
	return nullptr;
}

} // namespace

b3VoxelGridModule* Box3DVoxelModuleCache::acquire_region(const float* p_boxes, const uint8_t* p_faces, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	uint64_t hash = hash_words(14695981039346656037ull ^ 0x5245474Eull, &p_cell_meters, sizeof(float));
	hash = hash_words(hash, &p_cell_voxels, sizeof(int));
	hash = hash_words(hash, p_boxes, (size_t)p_box_count * 6 * sizeof(float));
	hash = hash_words(hash, p_faces, (size_t)p_box_count);
	{
		std::lock_guard<std::mutex> lock(cache_mutex);
		if (b3VoxelGridModule* cached = find_region(hash, p_boxes, p_faces, p_box_count, p_cell_meters, p_cell_voxels)) {
			return cached;
		}
	}
	b3VoxelGridModule* module = b3CreateVoxelGridModule(p_box_count > 0 ? p_boxes : nullptr, p_box_count, p_cell_meters, p_cell_voxels);
	if (module == nullptr) {
		return nullptr;
	}
	if (p_box_count > 0) {
		b3VoxelGridModule_SetCoveredFaces(module, p_faces, p_box_count);
	}
	std::lock_guard<std::mutex> lock(cache_mutex);
	if (b3VoxelGridModule* cached = find_region(hash, p_boxes, p_faces, p_box_count, p_cell_meters, p_cell_voxels)) {
		b3ReleaseVoxelGridModule(module);
		return cached;
	}
	b3RetainVoxelGridModule(module);
	CacheEntry entry{ module, p_cell_meters, p_cell_voxels, true, std::vector<uint8_t>(p_faces, p_faces + std::max(p_box_count, 0)) };
	cache.emplace(hash, std::move(entry));
	return module;
}

#pragma once

#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <mutex>
#include <shared_mutex>

using namespace godot;

// Ported from godot-jolt's src/containers/rid_owner.hpp. Engine-agnostic RID<->pointer
// registry: a thin HashMap<int64_t, T*> keyed by Godot-allocated RID ids.
template <typename TResource, bool TThreadSafe = false>
// NOLINTNEXTLINE(readability-identifier-naming)
class RID_PtrOwner {
public:
	RID_PtrOwner() = default;

	RID_PtrOwner(const RID_PtrOwner& p_other) = delete;

	RID_PtrOwner(RID_PtrOwner&& p_other) = delete;

	~RID_PtrOwner() {
		if (ptrs_by_id.size() > 0) {
			WARN_PRINT(vformat(
				"%d RIDs in Godot Box3D were found to not have been freed. "
				"This is likely caused by orphaned nodes. "
				"If not, consider reporting this issue.",
				ptrs_by_id.size()
			));
		}
	}

	_FORCE_INLINE_ RID make_rid(TResource* p_ptr) {
		const int64_t id = UtilityFunctions::rid_allocate_id();
		WriteLock lock(mutex);
		ptrs_by_id[id] = p_ptr;
		return UtilityFunctions::rid_from_int64(id);
	}

	_FORCE_INLINE_ TResource* get_or_null(const RID& p_rid) const {
		ReadLock lock(mutex);
		auto iter = ptrs_by_id.find(p_rid.get_id());
		return iter != ptrs_by_id.end() ? iter->value : nullptr;
	}

	_FORCE_INLINE_ void replace(const RID& p_rid, TResource* p_new_ptr) {
		WriteLock lock(mutex);
		auto iter = ptrs_by_id.find(p_rid.get_id());
		ERR_FAIL_COND(iter == ptrs_by_id.end());
		iter->value = p_new_ptr;
	}

	_FORCE_INLINE_ bool owns(const RID& p_rid) const {
		ReadLock lock(mutex);
		return ptrs_by_id.has(p_rid.get_id());
	}

	_FORCE_INLINE_ void free(const RID& p_rid) {
		WriteLock lock(mutex);
		ptrs_by_id.erase(p_rid.get_id());
	}

	RID_PtrOwner& operator=(const RID_PtrOwner& p_other) = delete;

	RID_PtrOwner& operator=(RID_PtrOwner&& p_other) = delete;

private:
	// With TThreadSafe the id map is guarded by a reader/writer lock so worker threads may look a
	// RID up while the main thread allocates or frees others; otherwise the lock is a no-op.
	struct NoLock {
		template <typename T> explicit NoLock(T&) {}
	};
	struct Mutex {};
	using MutexType = std::conditional_t<TThreadSafe, std::shared_mutex, Mutex>;
	using ReadLock = std::conditional_t<TThreadSafe, std::shared_lock<std::shared_mutex>, NoLock>;
	using WriteLock = std::conditional_t<TThreadSafe, std::unique_lock<std::shared_mutex>, NoLock>;
	mutable MutexType mutex;
	HashMap<int64_t, TResource*> ptrs_by_id;
};

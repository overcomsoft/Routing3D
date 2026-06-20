// Spatial Box Index for Routing3D.
// Indexes obstacle AABBs with a uniform-grid broadphase so sparse occupancy can
// answer overlap and nearest-distance queries without voxelizing the full domain.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "routing3d/geometry.hpp"

namespace routing3d {

inline bool aabb_overlap(const Vec3& alo, const Vec3& ahi, const Vec3& blo, const Vec3& bhi) {
    return alo.x < bhi.x && ahi.x > blo.x && alo.y < bhi.y && ahi.y > blo.y &&
           alo.z < bhi.z && ahi.z > blo.z;
}

inline double point_aabb_dist(const Vec3& p, const Vec3& lo, const Vec3& hi) {
    double dx = std::max(std::max(lo.x - p.x, 0.0), p.x - hi.x);
    double dy = std::max(std::max(lo.y - p.y, 0.0), p.y - hi.y);
    double dz = std::max(std::max(lo.z - p.z, 0.0), p.z - hi.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

class SpatialBoxIndex {
public:
    explicit SpatialBoxIndex(double bucket_mm) : bucket_mm_(bucket_mm > 0.0 ? bucket_mm : 1000.0) {}

    long long box_count() const { return static_cast<long long>(lo_.size()); }

    template <class Fn>
    void for_each_box(Fn&& fn) const {
        for (size_t idx = 0; idx < lo_.size(); ++idx)
            fn(lo_[idx], hi_[idx]);
    }

    void add(const Vec3& lo, const Vec3& hi) {
        const int idx = static_cast<int>(lo_.size());
        lo_.push_back(lo);
        hi_.push_back(hi);
        const int bx0 = fl(lo.x), bx1 = fl(hi.x);
        const int by0 = fl(lo.y), by1 = fl(hi.y);
        const int bz0 = fl(lo.z), bz1 = fl(hi.z);
        for (int bz = bz0; bz <= bz1; ++bz)
            for (int by = by0; by <= by1; ++by)
                for (int bx = bx0; bx <= bx1; ++bx)
                    buckets_[key(bx, by, bz)].push_back(idx);
    }

    bool overlaps(const Vec3& qlo, const Vec3& qhi) const {
        const int bx0 = fl(qlo.x), bx1 = fl(qhi.x);
        const int by0 = fl(qlo.y), by1 = fl(qhi.y);
        const int bz0 = fl(qlo.z), bz1 = fl(qhi.z);
        for (int bz = bz0; bz <= bz1; ++bz)
            for (int by = by0; by <= by1; ++by)
                for (int bx = bx0; bx <= bx1; ++bx) {
                    auto it = buckets_.find(key(bx, by, bz));
                    if (it == buckets_.end()) continue;
                    for (int bi : it->second)
                        if (aabb_overlap(qlo, qhi, lo_[static_cast<size_t>(bi)],
                                         hi_[static_cast<size_t>(bi)]))
                            return true;
                }
        return false;
    }

    double nearest_dist(const Vec3& p, double max_dist) const {
        if (lo_.empty()) return max_dist;
        const int bx0 = fl(p.x - max_dist), bx1 = fl(p.x + max_dist);
        const int by0 = fl(p.y - max_dist), by1 = fl(p.y + max_dist);
        const int bz0 = fl(p.z - max_dist), bz1 = fl(p.z + max_dist);
        double best = max_dist;
        for (int bz = bz0; bz <= bz1; ++bz)
            for (int by = by0; by <= by1; ++by)
                for (int bx = bx0; bx <= bx1; ++bx) {
                    auto it = buckets_.find(key(bx, by, bz));
                    if (it == buckets_.end()) continue;
                    for (int bi : it->second) {
                        double d = point_aabb_dist(p, lo_[static_cast<size_t>(bi)],
                                                   hi_[static_cast<size_t>(bi)]);
                        if (d < best) {
                            best = d;
                            if (best <= 0.0) return 0.0;
                        }
                    }
                }
        return best;
    }

private:
    int fl(double w) const { return static_cast<int>(std::floor(w / bucket_mm_)); }

    static uint64_t key(int bx, int by, int bz) {
        constexpr uint64_t B = 1u << 20;
        return ((static_cast<uint64_t>(static_cast<int64_t>(bx) + B) & 0x1FFFFF) << 42) |
               ((static_cast<uint64_t>(static_cast<int64_t>(by) + B) & 0x1FFFFF) << 21) |
               (static_cast<uint64_t>(static_cast<int64_t>(bz) + B) & 0x1FFFFF);
    }

    double bucket_mm_;
    std::vector<Vec3> lo_;
    std::vector<Vec3> hi_;
    std::unordered_map<uint64_t, std::vector<int>> buckets_;
};

}  // namespace routing3d

// 공간 박스 인덱스 (Spatial Box Index) — Routing3D C++ 엔진 (Phase 3, Step 3.11 — sparse 확장 S3)
// =============================================================================
// [이 파일이 하는 일]
//   장애물 AABB(월드 mm) 목록을 **유니폼 그리드 해시(broadphase)** 로 색인해
//   "이 작은 질의 박스(=셀)와 겹치는 장애물이 있는가?" 와 "이 점에서 가장 가까운
//   장애물 표면까지 거리(mm)?" 를 빠르게 답한다.
//
//   왜 필요한가 — 근본 동기(sparse 해법):
//     기존 DenseOccupancy 는 장애물을 셀 격자에 칠해(voxelize) 1B/셀 배열로 저장한다.
//     셀을 줄이면 셀 수가 부피로 폭증(25mm·1.3억셀=130MB, 10mm·20억셀=2GB→크래시).
//     반면 장애물을 **AABB 그대로** 들고 질의로 점유를 판정하면 메모리 = O(장애물 수)로
//     **셀 크기와 완전 무관**. 도메인이 커져도, 셀을 더 줄여도 저장이 폭발하지 않는다.
//
//   질의 의미(보수적):
//     overlaps(q) — 질의 박스 q 와 임의 장애물 AABB 의 '닿음 포함' 중첩(경계 일치도 점유).
//                   ImplicitOccupancy 가 셀 AABB 로 호출하면 DenseOccupancy.add_box 의
//                   복셀화(grid_box_range)와 사실상 동일한 점유 판정을 준다(과소차단 없음).
//     nearest_dist(p, max) — 점 p 에서 가장 가까운 장애물 표면까지 유클리드 거리(내부면 0).
//                   max 를 넘으면 탐색을 끊고 max 를 반환(온디맨드 클리어런스용 상한).
//
//   복잡도: 박스 ~1000개, 질의는 해당 버킷(+근방)만 스캔 → 질의당 상수 시간 수준.
//
// [빌드/실행]  헤더 전용. cmake --build cpp/build --config Release (occupancy.hpp 가 사용)
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "routing3d/geometry.hpp"

namespace routing3d {

// 두 AABB 의 '진짜' 중첩(경계가 정확히 맞닿기만 하면 false). 셀 AABB[c*cell,(c+1)*cell) 와
// 장애물 AABB 에 적용하면 DenseOccupancy.add_box 의 반열린 복셀화(grid_box_range: floor lo /
// ceil hi)와 **동일한 셀 점유**를 준다 → 경계의 인접 빈 셀을 과차단하지 않는다(불변식 O1 일치).
inline bool aabb_overlap(const Vec3& alo, const Vec3& ahi, const Vec3& blo, const Vec3& bhi) {
    return alo.x < bhi.x && ahi.x > blo.x && alo.y < bhi.y && ahi.y > blo.y &&
           alo.z < bhi.z && ahi.z > blo.z;
}

// 점 p 에서 AABB[lo,hi] 표면까지의 유클리드 거리(점이 내부면 0).
inline double point_aabb_dist(const Vec3& p, const Vec3& lo, const Vec3& hi) {
    double dx = std::max(std::max(lo.x - p.x, 0.0), p.x - hi.x);
    double dy = std::max(std::max(lo.y - p.y, 0.0), p.y - hi.y);
    double dz = std::max(std::max(lo.z - p.z, 0.0), p.z - hi.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 장애물 AABB 목록의 유니폼-그리드 broadphase 색인.
//   bucket_mm 한 변의 정육면체 버킷에 각 박스를 '겹치는 모든 버킷' 으로 등록한다.
//   질의 셀은 작아(≤버킷) 자신의 버킷만 스캔하면 그 셀과 겹치는 박스는 반드시 같은 버킷에
//   등록돼 있으므로(박스가 셀을 덮으면 셀의 버킷도 덮음) 누락 없이 찾는다.
class SpatialBoxIndex {
public:
    explicit SpatialBoxIndex(double bucket_mm) : bucket_mm_(bucket_mm > 0.0 ? bucket_mm : 1000.0) {}

    long long box_count() const { return static_cast<long long>(lo_.size()); }

    // 장애물 AABB 추가(월드 mm). lo<hi 가정(호출자가 퇴화 박스는 거른다).
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

    // 질의 박스 q[lo,hi] 와 겹치는 장애물이 하나라도 있는가(닿음 포함).
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

    // 점 p 에서 가장 가까운 장애물 표면까지 거리(mm). max_dist 초과면 탐색 중단 후 max_dist 반환.
    // p 를 중심으로 max_dist 반경이 닿는 버킷들만 스캔한다(온디맨드 클리어런스용).
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
                            if (best <= 0.0) return 0.0;  // 내부 → 더 볼 것 없음.
                        }
                    }
                }
        return best;
    }

private:
    int fl(double w) const { return static_cast<int>(std::floor(w / bucket_mm_)); }

    // 버킷 좌표(음수 가능) → 64비트 키. 축당 21비트 + 바이어스(±2^20). 플랜트 도메인엔 충분.
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

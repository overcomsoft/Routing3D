// ê³µê°„ ë°•ìŠ¤ ?¸ë±??(Spatial Box Index) ??Routing3D C++ ?”ì§„ (Phase 3, Step 3.11 ??sparse ?•ì¥ S3)
// =============================================================================
// [???Œì¼???˜ëŠ” ??
//   ?¥ì• ë¬?AABB(?”ë“œ mm) ëª©ë¡??**? ë‹ˆ??ê·¸ë¦¬???´ì‹œ(broadphase)** ë¡??‰ì¸??
//   "???‘ì? ì§ˆì˜ ë°•ìŠ¤(=?€)?€ ê²¹ì¹˜???¥ì• ë¬¼ì´ ?ˆëŠ”ê°€?" ?€ "???ì—??ê°€??ê°€ê¹Œìš´
//   ?¥ì• ë¬??œë©´ê¹Œì? ê±°ë¦¬(mm)?" ë¥?ë¹ ë¥´ê²??µí•œ??
//
//   ???„ìš”?œê? ??ê·¼ë³¸ ?™ê¸°(sparse ?´ë²•):
//     ê¸°ì¡´ DenseOccupancy ???¥ì• ë¬¼ì„ ?€ ê²©ì??ì¹ í•´(voxelize) 1B/?€ ë°°ì—´ë¡??€?¥í•œ??
//     ?€??ì¤„ì´ë©??€ ?˜ê? ë¶€?¼ë¡œ ??¦(25mmÂ·1.3?µì?=130MB, 10mmÂ·20?µì?=2GB?’í¬?˜ì‹œ).
//     ë°˜ë©´ ?¥ì• ë¬¼ì„ **AABB ê·¸ë?ë¡?* ?¤ê³  ì§ˆì˜ë¡??ìœ ë¥??ì •?˜ë©´ ë©”ëª¨ë¦?= O(?¥ì• ë¬???ë¡?
//     **?€ ?¬ê¸°?€ ?„ì „ ë¬´ê?**. ?„ë©”?¸ì´ ì»¤ì ¸?? ?€????ì¤„ì—¬???€?¥ì´ ??°œ?˜ì? ?ŠëŠ”??
//
//   ì§ˆì˜ ?˜ë?(ë³´ìˆ˜??:
//     overlaps(q) ??ì§ˆì˜ ë°•ìŠ¤ q ?€ ?„ì˜ ?¥ì• ë¬?AABB ??'?¿ìŒ ?¬í•¨' ì¤‘ì²©(ê²½ê³„ ?¼ì¹˜???ìœ ).
//                   ImplicitOccupancy ê°€ ?€ AABB ë¡??¸ì¶œ?˜ë©´ DenseOccupancy.add_box ??
//                   ë³µì???grid_box_range)?€ ?¬ì‹¤???™ì¼???ìœ  ?ì •??ì¤€??ê³¼ì†Œì°¨ë‹¨ ?†ìŒ).
//     nearest_dist(p, max) ????p ?ì„œ ê°€??ê°€ê¹Œìš´ ?¥ì• ë¬??œë©´ê¹Œì? ? í´ë¦¬ë“œ ê±°ë¦¬(?´ë?ë©?0).
//                   max ë¥??˜ìœ¼ë©??ìƒ‰???Šê³  max ë¥?ë°˜í™˜(?¨ë””ë§¨ë“œ ?´ë¦¬?´ëŸ°?¤ìš© ?í•œ).
//
//   ë³µì¡?? ë°•ìŠ¤ ~1000ê°? ì§ˆì˜???´ë‹¹ ë²„í‚·(+ê·¼ë°©)ë§??¤ìº” ??ì§ˆì˜???ìˆ˜ ?œê°„ ?˜ì?.
//
// [ë¹Œë“œ/?¤í–‰]  ?¤ë” ?„ìš©. cmake --build cpp/build --config Release (occupancy.hpp ê°€ ?¬ìš©)
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

// ??AABB ??'ì§„ì§œ' ì¤‘ì²©(ê²½ê³„ê°€ ?•í™•??ë§ë‹¿ê¸°ë§Œ ?˜ë©´ false). ?€ AABB[c*cell,(c+1)*cell) ?€
// ?¥ì• ë¬?AABB ???ìš©?˜ë©´ DenseOccupancy.add_box ??ë°˜ì—´ë¦?ë³µì???grid_box_range: floor lo /
// ceil hi)?€ **?™ì¼???€ ?ìœ **ë¥?ì¤€????ê²½ê³„???¸ì ‘ ë¹??€??ê³¼ì°¨?¨í•˜ì§€ ?ŠëŠ”??ë¶ˆë???O1 ?¼ì¹˜).
inline bool aabb_overlap(const Vec3& alo, const Vec3& ahi, const Vec3& blo, const Vec3& bhi) {
    return alo.x < bhi.x && ahi.x > blo.x && alo.y < bhi.y && ahi.y > blo.y &&
           alo.z < bhi.z && ahi.z > blo.z;
}

// ??p ?ì„œ AABB[lo,hi] ?œë©´ê¹Œì???? í´ë¦¬ë“œ ê±°ë¦¬(?ì´ ?´ë?ë©?0).
inline double point_aabb_dist(const Vec3& p, const Vec3& lo, const Vec3& hi) {
    double dx = std::max(std::max(lo.x - p.x, 0.0), p.x - hi.x);
    double dy = std::max(std::max(lo.y - p.y, 0.0), p.y - hi.y);
    double dz = std::max(std::max(lo.z - p.z, 0.0), p.z - hi.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ?¥ì• ë¬?AABB ëª©ë¡??? ë‹ˆ??ê·¸ë¦¬??broadphase ?‰ì¸.
//   bucket_mm ??ë³€???•ìœ¡ë©´ì²´ ë²„í‚·??ê°?ë°•ìŠ¤ë¥?'ê²¹ì¹˜??ëª¨ë“  ë²„í‚·' ?¼ë¡œ ?±ë¡?œë‹¤.
//   ì§ˆì˜ ?€?€ ?‘ì•„(?¤ë²„?? ?ì‹ ??ë²„í‚·ë§??¤ìº”?˜ë©´ ê·??€ê³?ê²¹ì¹˜??ë°•ìŠ¤??ë°˜ë“œ??ê°™ì? ë²„í‚·??
//   ?±ë¡???ˆìœ¼ë¯€ë¡?ë°•ìŠ¤ê°€ ?€????œ¼ë©??€??ë²„í‚·????Œ) ?„ë½ ?†ì´ ì°¾ëŠ”??
class SpatialBoxIndex {
public:
    explicit SpatialBoxIndex(double bucket_mm) : bucket_mm_(bucket_mm > 0.0 ? bucket_mm : 1000.0) {}

    long long box_count() const { return static_cast<long long>(lo_.size()); }

    template <class Fn>
    void for_each_box(Fn&& fn) const {
        for (size_t idx = 0; idx < lo_.size(); ++idx)
            fn(lo_[idx], hi_[idx]);
    }

    // ?¥ì• ë¬?AABB ì¶”ê?(?”ë“œ mm). lo<hi ê°€???¸ì¶œ?ê? ?´í™” ë°•ìŠ¤??ê±°ë¥¸??.
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

    // ì§ˆì˜ ë°•ìŠ¤ q[lo,hi] ?€ ê²¹ì¹˜???¥ì• ë¬¼ì´ ?˜ë‚˜?¼ë„ ?ˆëŠ”ê°€(?¿ìŒ ?¬í•¨).
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

    // ??p ?ì„œ ê°€??ê°€ê¹Œìš´ ?¥ì• ë¬??œë©´ê¹Œì? ê±°ë¦¬(mm). max_dist ì´ˆê³¼ë©??ìƒ‰ ì¤‘ë‹¨ ??max_dist ë°˜í™˜.
    // p ë¥?ì¤‘ì‹¬?¼ë¡œ max_dist ë°˜ê²½???¿ëŠ” ë²„í‚·?¤ë§Œ ?¤ìº”?œë‹¤(?¨ë””ë§¨ë“œ ?´ë¦¬?´ëŸ°?¤ìš©).
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
                            if (best <= 0.0) return 0.0;  // ?´ë? ????ë³?ê²??†ìŒ.
                        }
                    }
                }
        return best;
    }

private:
    int fl(double w) const { return static_cast<int>(std::floor(w / bucket_mm_)); }

    // ë²„í‚· ì¢Œí‘œ(?Œìˆ˜ ê°€?? ??64ë¹„íŠ¸ ?? ì¶•ë‹¹ 21ë¹„íŠ¸ + ë°”ì´?´ìŠ¤(Â±2^20). ?Œëœ???„ë©”?¸ì—” ì¶©ë¶„.
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

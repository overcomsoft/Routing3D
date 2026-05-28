// 희소 점유맵 구현 — occupancy.hpp 의 SparseOccupancy 참고. 명세 algorithm_spec.md §1,§2.
// 좌표/복셀화는 geometry.hpp 의 공유 함수로 Dense 와 동일(불변식 O1: 질의 결과 동일).
#include "routing3d/occupancy.hpp"

#include <stdexcept>

namespace routing3d {

SparseOccupancy::SparseOccupancy(Cell shape, Vec3 origin, double cell_mm)
    : shape_(shape), origin_(origin), cell_(cell_mm) {
    if (shape.i <= 0 || shape.j <= 0 || shape.k <= 0)
        throw std::invalid_argument("shape must be 3 positive ints");
    if (cell_mm <= 0.0)
        throw std::invalid_argument("cell_mm must be positive");
    // 축당 21비트 패킹 한계(2,097,152) 검사.
    if (shape.i > (1 << 21) || shape.j > (1 << 21) || shape.k > (1 << 21))
        throw std::invalid_argument("SparseOccupancy: 축당 셀 수는 2^21 이하여야 합니다");
}

bool SparseOccupancy::is_blocked(const Cell& c) const {
    if (!in_bounds(c)) return true;  // 격자 밖 = 점유 (불변식 G1)
    return blocked_.find(pack(c)) != blocked_.end();
}

void SparseOccupancy::block_cell(const Cell& c) {
    if (in_bounds(c)) blocked_.insert(pack(c));
}

int SparseOccupancy::add_box(const AABB& box) {
    const CellRange r = grid_box_range(box, origin_, cell_, shape_);
    if (r.empty()) return 0;
    int newly = 0;
    for (int k = r.lo.k; k < r.hi.k; ++k)
        for (int j = r.lo.j; j < r.hi.j; ++j)
            for (int i = r.lo.i; i < r.hi.i; ++i)
                if (blocked_.insert(pack(Cell{i, j, k})).second) ++newly;
    return newly;
}

Cell SparseOccupancy::unlin(int idx) const {
    int i = idx % shape_.i;
    int j = (idx / shape_.i) % shape_.j;
    int k = idx / (shape_.i * shape_.j);
    return Cell{i, j, k};
}

}  // namespace routing3d

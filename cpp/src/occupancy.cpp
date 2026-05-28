// 점유맵 구현 — occupancy.hpp 참고. 명세 algorithm_spec.md §1,§2.
#include "routing3d/occupancy.hpp"

#include <algorithm>
#include <cmath>

namespace routing3d {

DenseOccupancy::DenseOccupancy(Cell shape, Vec3 origin, double cell_mm)
    : shape_(shape), origin_(origin), cell_(cell_mm) {
    if (shape.i <= 0 || shape.j <= 0 || shape.k <= 0)
        throw std::invalid_argument("shape must be 3 positive ints");
    if (cell_mm <= 0.0)
        throw std::invalid_argument("cell_mm must be positive");
    grid_.assign(static_cast<size_t>(size()), 0);
}

bool DenseOccupancy::in_bounds(const Cell& c) const {
    return grid_in_bounds(c, shape_);
}

bool DenseOccupancy::is_blocked(const Cell& c) const {
    if (!in_bounds(c)) return true;  // 격자 밖 = 점유 (불변식 G1)
    return grid_[static_cast<size_t>(lin(c))] != 0;
}

void DenseOccupancy::block_cell(const Cell& c) {
    if (in_bounds(c)) grid_[static_cast<size_t>(lin(c))] = 1;
}

Vec3 DenseOccupancy::to_world(const Cell& c) const {
    return grid_cell_to_world(c, origin_, cell_);
}

Cell DenseOccupancy::to_cell(const Vec3& w) const {
    return grid_world_to_cell(w, origin_, cell_);
}

int DenseOccupancy::add_box(const AABB& box) {
    const CellRange r = grid_box_range(box, origin_, cell_, shape_);
    if (r.empty()) return 0;
    int newly = 0;
    for (int k = r.lo.k; k < r.hi.k; ++k)
        for (int j = r.lo.j; j < r.hi.j; ++j)
            for (int i = r.lo.i; i < r.hi.i; ++i) {
                size_t idx = static_cast<size_t>(lin(Cell{i, j, k}));
                if (grid_[idx] == 0) { grid_[idx] = 1; ++newly; }
            }
    return newly;
}

long long DenseOccupancy::count_blocked() const {
    long long n = 0;
    for (uint8_t v : grid_) n += (v != 0);
    return n;
}

Cell DenseOccupancy::unlin(int idx) const {
    int i = idx % shape_.i;
    int j = (idx / shape_.i) % shape_.j;
    int k = idx / (shape_.i * shape_.j);
    return Cell{i, j, k};
}

}  // namespace routing3d

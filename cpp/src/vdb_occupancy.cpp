// OpenVDB 점유맵 구현 — vdb_occupancy.hpp 참고. OpenVDB BoolGrid 를 점유 비트맵으로 사용.
// 좌표/복셀화는 geometry.hpp 공유 함수로 Dense/Sparse 와 일치(불변식 O1).
#include "routing3d/vdb_occupancy.hpp"

#include <mutex>
#include <stdexcept>

#include <openvdb/openvdb.h>

namespace routing3d {

namespace {
// openvdb::initialize() 는 그리드 사용 전 한 번 호출해야 한다(여러 번 호출은 안전, 스레드세이프).
void ensure_openvdb_initialized() {
    static std::once_flag flag;
    std::call_once(flag, []() { openvdb::initialize(); });
}

openvdb::Coord to_coord(const Cell& c) { return openvdb::Coord(c.i, c.j, c.k); }
}  // namespace

// 내부 구현: BoolGrid + 빠른 질의용 ValueAccessor.
struct VdbOccupancy::Impl {
    openvdb::BoolGrid::Ptr grid;
    // 읽기 가속용 accessor(단일 스레드). is_blocked 가 자주 호출되므로 캐시.
    mutable openvdb::BoolGrid::ConstAccessor accessor;

    explicit Impl(openvdb::BoolGrid::Ptr g) : grid(std::move(g)), accessor(grid->getConstAccessor()) {}
};

VdbOccupancy::VdbOccupancy(Cell shape, Vec3 origin, double cell_mm)
    : shape_(shape), origin_(origin), cell_(cell_mm) {
    if (shape.i <= 0 || shape.j <= 0 || shape.k <= 0)
        throw std::invalid_argument("shape must be 3 positive ints");
    if (cell_mm <= 0.0)
        throw std::invalid_argument("cell_mm must be positive");
    ensure_openvdb_initialized();
    // 배경값 false(비점유). 활성 복셀 = 점유.
    impl_ = std::make_unique<Impl>(openvdb::BoolGrid::create(/*background=*/false));
}

VdbOccupancy::~VdbOccupancy() = default;

VdbOccupancy::VdbOccupancy(const VdbOccupancy& other)
    : shape_(other.shape_), origin_(other.origin_), cell_(other.cell_) {
    // 그리드 깊은 복사.
    openvdb::BoolGrid::Ptr g = other.impl_->grid->deepCopy();
    impl_ = std::make_unique<Impl>(std::move(g));
}

VdbOccupancy& VdbOccupancy::operator=(const VdbOccupancy& other) {
    if (this != &other) {
        shape_ = other.shape_;
        origin_ = other.origin_;
        cell_ = other.cell_;
        impl_ = std::make_unique<Impl>(other.impl_->grid->deepCopy());
    }
    return *this;
}

VdbOccupancy::VdbOccupancy(VdbOccupancy&&) noexcept = default;
VdbOccupancy& VdbOccupancy::operator=(VdbOccupancy&&) noexcept = default;

bool VdbOccupancy::is_blocked(const Cell& c) const {
    if (!in_bounds(c)) return true;  // 격자 밖 = 점유 (불변식 G1)
    return impl_->accessor.isValueOn(to_coord(c));
}

void VdbOccupancy::block_cell(const Cell& c) {
    if (!in_bounds(c)) return;
    // 쓰기는 별도 accessor(읽기 accessor 와 트리 상태 동기화 위해 쓰기 후 재생성).
    impl_->grid->getAccessor().setValueOn(to_coord(c), true);
    impl_->accessor = impl_->grid->getConstAccessor();  // 캐시 무효화.
}

long long VdbOccupancy::add_box(const AABB& box) {
    const CellRange r = grid_box_range(box, origin_, cell_, shape_);
    if (r.empty()) return 0;
    const openvdb::Index64 before = impl_->grid->activeVoxelCount();
    // fill 은 [min,max] 양끝 포함 → hi-1. active=true 로 타일 최적화(꽉 찬 영역=타일).
    openvdb::CoordBBox bbox(openvdb::Coord(r.lo.i, r.lo.j, r.lo.k),
                           openvdb::Coord(r.hi.i - 1, r.hi.j - 1, r.hi.k - 1));
    impl_->grid->fill(bbox, /*value=*/true, /*active=*/true);
    impl_->accessor = impl_->grid->getConstAccessor();  // 캐시 무효화.
    const openvdb::Index64 after = impl_->grid->activeVoxelCount();
    return static_cast<long long>(after - before);
}

long long VdbOccupancy::count_blocked() const {
    return static_cast<long long>(impl_->grid->activeVoxelCount());
}

long long VdbOccupancy::memory_bytes() const {
    return static_cast<long long>(impl_->grid->memUsage());
}

Cell VdbOccupancy::unlin(int idx) const {
    int i = idx % shape_.i;
    int j = (idx / shape_.i) % shape_.j;
    int k = idx / (shape_.i * shape_.j);
    return Cell{i, j, k};
}

}  // namespace routing3d

// FCL 정밀 충돌 장면 구현 — fcl_scene.hpp 참고. 장애물 박스 → FCL broadphase, 캡슐 질의.
// 단위 mm. FCL/Eigen 타입은 이 .cpp 안에만 등장(헤더 pimpl).
#include "routing3d/fcl_scene.hpp"

#include <algorithm>
#include <cmath>

#include <fcl/fcl.h>

namespace routing3d {

struct FclScene::Impl {
    // broadphase 가 raw 포인터를 보유하므로 객체 주소가 안정적이어야 함 → unique_ptr 보관.
    std::vector<std::unique_ptr<fcl::CollisionObjectd>> objs;
    fcl::DynamicAABBTreeCollisionManagerd mgr;
    bool built = false;

    void ensure_built() {
        if (!built) {
            mgr.setup();
            built = true;
        }
    }

    // 질의 기하 1개를 broadphase 와 충돌검사. 충돌하면 true.
    bool query(const std::shared_ptr<fcl::CollisionGeometryd>& geom, const fcl::Transform3d& tf) {
        ensure_built();
        fcl::CollisionObjectd qobj(geom, tf);
        fcl::DefaultCollisionData<double> cdata;
        mgr.collide(&qobj, &cdata, fcl::DefaultCollisionFunction<double>);
        return cdata.result.isCollision();
    }
};

FclScene::FclScene() : impl_(std::make_unique<Impl>()) {}
FclScene::~FclScene() = default;
FclScene::FclScene(FclScene&&) noexcept = default;
FclScene& FclScene::operator=(FclScene&&) noexcept = default;

void FclScene::add_box(const AABB& box) {
    const double sx = box.hi.x - box.lo.x;
    const double sy = box.hi.y - box.lo.y;
    const double sz = box.hi.z - box.lo.z;
    auto geom = std::make_shared<fcl::Boxd>(sx, sy, sz);
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    tf.translation() = fcl::Vector3d(0.5 * (box.lo.x + box.hi.x), 0.5 * (box.lo.y + box.hi.y),
                                     0.5 * (box.lo.z + box.hi.z));
    auto obj = std::make_unique<fcl::CollisionObjectd>(geom, tf);
    impl_->mgr.registerObject(obj.get());
    impl_->objs.push_back(std::move(obj));
    impl_->built = false;  // 토폴로지 변경 → 다음 질의 시 재setup.
}

void FclScene::build() { impl_->ensure_built(); }

size_t FclScene::size() const { return impl_->objs.size(); }

bool FclScene::collides_sphere(const Vec3& center, double radius) const {
    auto geom = std::make_shared<fcl::Sphered>(std::max(radius, 1e-6));
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    tf.translation() = fcl::Vector3d(center.x, center.y, center.z);
    return impl_->query(geom, tf);
}

bool FclScene::collides_point(const Vec3& p) const { return collides_sphere(p, 0.0); }

bool FclScene::segment_clear(const Vec3& a, const Vec3& b, double radius) const {
    const fcl::Vector3d A(a.x, a.y, a.z);
    const fcl::Vector3d B(b.x, b.y, b.z);
    const fcl::Vector3d dir = B - A;
    const double L = dir.norm();
    const double r = std::max(radius, 1e-6);

    fcl::Transform3d tf = fcl::Transform3d::Identity();
    std::shared_ptr<fcl::CollisionGeometryd> geom;
    if (L < 1e-9) {
        geom = std::make_shared<fcl::Sphered>(r);
        tf.translation() = A;
    } else {
        // 캡슐: 로컬 z축 길이 L 실린더 + 반경 r 반구 캡. 중점에 놓고 z축을 dir 로 회전.
        geom = std::make_shared<fcl::Capsuled>(r, L);
        tf.translation() = 0.5 * (A + B);
        const fcl::Quaterniond q =
            fcl::Quaterniond::FromTwoVectors(fcl::Vector3d::UnitZ(), dir / L);
        tf.linear() = q.toRotationMatrix();
    }
    return !impl_->query(geom, tf);  // 충돌 없으면 통과 가능.
}

bool FclScene::path_clear(const std::vector<Vec3>& pts, double radius) const {
    if (pts.size() < 2) return true;
    for (size_t i = 1; i < pts.size(); ++i)
        if (!segment_clear(pts[i - 1], pts[i], radius)) return false;
    return true;
}

}  // namespace routing3d

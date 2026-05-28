// 점유맵 백엔드 테스트 — Routing3D C++ 엔진 (Phase 3, Step 3.6)
// =============================================================================
// [이 파일이 하는 일]
//   1) 불변식 O1: DenseOccupancy 와 SparseOccupancy 가 동일 입력에 대해 **동일 질의 결과**를
//      내는지 검증한다(in_bounds/is_blocked/to_cell/to_world/count_blocked).
//   2) 8,000m 초대형 격자: SparseOccupancy 가 점유 셀 수에 비례한 메모리만 쓰는지(= 격자 전체를
//      할당하지 않음) 확인한다. Dense 로는 불가능한 규모임을 대비로 보인다.
//
//   참고: 이 해시셋 기반 희소맵은 '흩어진' 장애물에는 메모리 효율적이지만, 8,000m 바닥/천장
//   같은 '꽉 찬 2D 시트'는 복셀이 폭증한다 → 타일 압축(OpenVDB)이 본 Step 3.6 의 핵심 목표다.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R occupancy --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>
#include <vector>

#include "routing3d/occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;

static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

// 한 장면(shape/obstacles)에 대해 Dense 와 Sparse 의 모든 질의가 일치하는지 전수 비교.
static void o1_scene(const std::string& name, Cell shape, Vec3 origin, double cell_mm,
                     const std::vector<AABB>& boxes) {
    std::printf("=== O1: %s (shape %d,%d,%d) ===\n", name.c_str(), shape.i, shape.j, shape.k);
    DenseOccupancy dense(shape, origin, cell_mm);
    SparseOccupancy sparse(shape, origin, cell_mm);
    int dnew = 0, snew = 0;
    for (const AABB& b : boxes) { dnew += dense.add_box(b); snew += sparse.add_box(b); }
    check(dnew == snew, "add_box 신규 점유 수 동일");
    check(dense.count_blocked() == sparse.count_blocked(), "count_blocked 동일");

    // 격자 내부 전수 + 경계 밖 일부 셀에 대해 is_blocked/in_bounds 비교.
    long long mism = 0;
    for (int k = -1; k <= shape.k; ++k)
        for (int j = -1; j <= shape.j; ++j)
            for (int i = -1; i <= shape.i; ++i) {
                Cell c{i, j, k};
                if (dense.is_blocked(c) != sparse.is_blocked(c)) ++mism;
                if (dense.in_bounds(c) != sparse.in_bounds(c)) ++mism;
            }
    check(mism == 0, "is_blocked/in_bounds 전수 일치(경계 밖 포함)");

    // 좌표 변환 동일(샘플 월드 포인트).
    long long tmism = 0;
    for (int s = 0; s < 50; ++s) {
        Vec3 w{origin.x + s * 37.3, origin.y + s * 53.1, origin.z + s * 11.7};
        Cell dc = dense.to_cell(w), sc = sparse.to_cell(w);
        if (!(dc == sc)) ++tmism;
        Cell cc{s % shape.i, (s * 3) % shape.j, (s * 7) % shape.k};
        Vec3 dw = dense.to_world(cc), sw = sparse.to_world(cc);
        if (dw.x != sw.x || dw.y != sw.y || dw.z != sw.z) ++tmism;
    }
    check(tmism == 0, "to_cell/to_world 동일");
}

static void test_o1() {
    // golden02 유사: 80^3 격자에 기둥형 장애물.
    o1_scene("single_obstacle", Cell{80, 80, 80}, Vec3{0, 0, 0}, 50.0,
             {AABB(Vec3{1900, 0, 0}, Vec3{2150, 2250, 4000})});
    // golden03 유사: 120x120x60, 바닥 슬래브.
    o1_scene("multi_tier", Cell{120, 120, 60}, Vec3{0, 0, 0}, 50.0,
             {AABB(Vec3{0, 0, 0}, Vec3{6000, 6000, 250})});
    // 여러 박스 + 비정수 origin.
    o1_scene("multi_box", Cell{40, 40, 40}, Vec3{-125.5, 33.0, 7.25}, 50.0,
             {AABB(Vec3{0, 0, 0}, Vec3{500, 500, 500}),
              AABB(Vec3{900, 200, 100}, Vec3{1100, 1900, 300}),
              AABB(Vec3{-50, -50, -50}, Vec3{120, 120, 120})});
}

static void test_8000m_memory() {
    std::printf("=== 8,000m 초대형 격자 (Sparse) ===\n");
    // 8,000m / 50mm = 160,000 셀/축. Dense 라면 160000^3 = 4.096e15 셀(=4 PB) → 불가능.
    const int N = 160000;
    SparseOccupancy occ(Cell{N, N, N}, Vec3{0, 0, 0}, 50.0);
    check(occ.size() == 4096000000000000LL, "size() == 160000^3 (오버플로 없음)");
    check(occ.count_blocked() == 0, "구성 직후 점유 0 (격자 전체 미할당)");

    // 흩어진 기둥 100개(각 500x500x4000mm = 10x10x80 = 8,000 셀)를 공간 전역에 배치.
    long long expect = 0;
    for (int n = 0; n < 100; ++n) {
        double x = 1000.0 + n * 70000.0;          // 70m 간격으로 퍼뜨림(8,000m 범위 내).
        double y = 1000.0 + (n % 10) * 700000.0 / 10.0;
        AABB col(Vec3{x, y, 0.0}, Vec3{x + 500.0, y + 500.0, 4000.0});
        occ.add_box(col);
        expect += 10LL * 10 * 80;
    }
    std::printf("  점유 셀 %lld개\n", occ.count_blocked());
    check(occ.count_blocked() == expect, "흩어진 기둥 100개 복셀 수 일치");

    // 메모리 추정: unordered_set<uint64_t> 노드당 보수적으로 64바이트로 잡아도 32GB 미만.
    double est_bytes = static_cast<double>(occ.count_blocked()) * 64.0;
    std::printf("  추정 메모리 ~%.1f MB (32GB 한도 대비)\n", est_bytes / (1024.0 * 1024.0));
    check(est_bytes < 32.0 * 1024 * 1024 * 1024, "추정 메모리 < 32GB (흩어진 장애물)");
}

int main() {
    test_o1();
    test_8000m_memory();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}

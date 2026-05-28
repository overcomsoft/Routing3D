// OpenVDB 점유맵 테스트 — Routing3D C++ 엔진 (Phase 3, Step 3.6)
// =============================================================================
// [이 파일이 하는 일]
//   1) 불변식 O1: VdbOccupancy 가 DenseOccupancy 와 동일 질의 결과를 내는지 검증.
//   2) 타일 압축(핵심): 8,000m 규모의 '꽉 찬 부피' 장애물을 OpenVDB fill(타일)로 넣으면
//      활성 복셀 수는 수조 개라도 **메모리는 소량**(<32GB)임을 보인다. 해시셋 SparseOccupancy
//      라면 복셀당 저장 → 수십 TB 필요 → OpenVDB 타일 압축의 가치 입증.
//
// [빌드/실행]  (OpenVDB = vcpkg; USE_OPENVDB=ON)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64 -DUSE_OPENVDB=ON `
//     -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R vdb --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/vdb_occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;

static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

// Dense 와 Vdb 의 모든 질의가 일치하는지 전수 비교(O1).
static void o1_scene(const std::string& name, Cell shape, Vec3 origin, double cell_mm,
                     const std::vector<AABB>& boxes) {
    std::printf("=== O1: %s (shape %d,%d,%d) ===\n", name.c_str(), shape.i, shape.j, shape.k);
    DenseOccupancy dense(shape, origin, cell_mm);
    VdbOccupancy vdb(shape, origin, cell_mm);
    long long dnew = 0, vnew = 0;
    for (const AABB& b : boxes) { dnew += dense.add_box(b); vnew += vdb.add_box(b); }
    check(dnew == vnew, "add_box 신규 점유 수 동일");
    check(dense.count_blocked() == vdb.count_blocked(), "count_blocked 동일");

    long long mism = 0;
    for (int k = -1; k <= shape.k; ++k)
        for (int j = -1; j <= shape.j; ++j)
            for (int i = -1; i <= shape.i; ++i) {
                Cell c{i, j, k};
                if (dense.is_blocked(c) != vdb.is_blocked(c)) ++mism;
                if (dense.in_bounds(c) != vdb.in_bounds(c)) ++mism;
            }
    check(mism == 0, "is_blocked/in_bounds 전수 일치(경계 밖 포함)");

    long long tmism = 0;
    for (int s = 0; s < 50; ++s) {
        Vec3 w{origin.x + s * 37.3, origin.y + s * 53.1, origin.z + s * 11.7};
        if (!(dense.to_cell(w) == vdb.to_cell(w))) ++tmism;
        Cell cc{s % shape.i, (s * 3) % shape.j, (s * 7) % shape.k};
        Vec3 dw = dense.to_world(cc), vw = vdb.to_world(cc);
        if (dw.x != vw.x || dw.y != vw.y || dw.z != vw.z) ++tmism;
    }
    check(tmism == 0, "to_cell/to_world 동일");

    // block_cell 후 deepCopy 독립성(copy() 가 원본 불변 보장 — 계약 M2 대비).
    VdbOccupancy v2 = vdb.copy();
    Cell free{shape.i - 1, shape.j - 1, shape.k - 1};
    if (!vdb.is_blocked(free)) {
        v2.block_cell(free);
        check(v2.is_blocked(free) && !vdb.is_blocked(free), "copy() 깊은 복사(원본 불변)");
    }
}

static void test_o1() {
    o1_scene("single_obstacle", Cell{80, 80, 80}, Vec3{0, 0, 0}, 50.0,
             {AABB(Vec3{1900, 0, 0}, Vec3{2150, 2250, 4000})});
    o1_scene("multi_tier", Cell{120, 120, 60}, Vec3{0, 0, 0}, 50.0,
             {AABB(Vec3{0, 0, 0}, Vec3{6000, 6000, 250})});
    o1_scene("multi_box", Cell{40, 40, 40}, Vec3{-125.5, 33.0, 7.25}, 50.0,
             {AABB(Vec3{0, 0, 0}, Vec3{500, 500, 500}),
              AABB(Vec3{900, 200, 100}, Vec3{1100, 1900, 300}),
              AABB(Vec3{-50, -50, -50}, Vec3{120, 120, 120})});
}

static void test_8000m_scattered() {
    // (A) 현실적 희소 장면: 8,000m 범위에 흩어진 기둥 100개. Dense 라면 160000^3 = 4 PB → 불가능.
    //     OpenVDB 는 빈 공간을 할당하지 않으므로 메모리 ∝ 내용물.
    std::printf("=== 8,000m 흩어진 장애물 (VdbOccupancy) ===\n");
    const int N = 160000;  // 8,000m / 50mm.
    VdbOccupancy occ(Cell{N, N, N}, Vec3{0, 0, 0}, 50.0);
    long long expect = 0;
    for (int n = 0; n < 100; ++n) {
        double x = 1000.0 + n * 70000.0;
        double y = 1000.0 + (n % 10) * 70000.0;
        occ.add_box(AABB(Vec3{x, y, 0.0}, Vec3{x + 500.0, y + 500.0, 4000.0}));
        expect += 10LL * 10 * 80;
    }
    const long long mem = occ.memory_bytes();
    std::printf("  점유 복셀 %lld개, 메모리 %.2f MB\n", occ.count_blocked(), mem / (1024.0 * 1024.0));
    check(occ.count_blocked() == expect, "흩어진 기둥 100개 복셀 수 일치");
    check(mem < 32LL * 1024 * 1024 * 1024, "메모리 < 32GB (빈 공간 미할당)");
}

static void test_tile_compression() {
    // (B) 타일 압축 데모: '모든 차원이 큰' 꽉 찬 영역만 상위 타일로 압축된다.
    //     전체 8,000m^3 도메인을 채우면 루트 타일로 표현 → 활성 복셀 4.096e15 인데 메모리는 소량.
    std::printf("=== 타일 압축: 전체 도메인 채우기 (VdbOccupancy) ===\n");
    const int N = 160000;
    VdbOccupancy occ(Cell{N, N, N}, Vec3{0, 0, 0}, 50.0);
    occ.add_box(AABB(Vec3{0, 0, 0}, Vec3{N * 50.0, N * 50.0, N * 50.0}));  // 전체 도메인.
    const long long active = occ.count_blocked();
    const long long mem = occ.memory_bytes();
    std::printf("  활성 복셀 %lld개, 메모리 %.2f MB\n", active, mem / (1024.0 * 1024.0));
    check(active == 4096000000000000LL, "활성 복셀 == 160000^3 (4.096e15)");
    check(mem < 4LL * 1024 * 1024 * 1024, "메모리 < 4GB (타일 압축, 32GB 목표 대비 여유)");
    double hashset_tb = static_cast<double>(active) * 16.0 / (1024.0 * 1024.0 * 1024.0 * 1024.0);
    std::printf("  (참고) 해시셋이라면 ~%.0f TB 필요 → 타일 압축 핵심\n", hashset_tb);
    check(active * 16LL / mem > 1000, "압축비 > 1000배 (복셀당 저장 대비)");
    // 주의: '얇은' 바닥/천장 시트(한 축이 노드 dim 128 미만)는 leaf 타일까지만 압축돼 메모리가
    //       커진다 → 50mm 전역 fine 격자 대신 계층(coarse) 표현 + 로컬 corridor 가 필요(후속 단계).
}

// 백엔드 무관 검증: 골든 입력을 Dense 와 Vdb 로 라우팅하면 지표·경로가 같아야 한다.
static void test_routing_cross_backend() {
    std::printf("=== 라우팅 백엔드 교차검증: Dense vs Vdb ===\n");
    RouteParams bp;  // baseline.

    // 골든02(단일, 장애물 우회): 지표 + 경로 셀 완전 일치.
    {
        const Cell shape{80, 80, 80};
        DenseOccupancy d(shape, Vec3{0, 0, 0}, 50.0);
        VdbOccupancy v(shape, Vec3{0, 0, 0}, 50.0);
        const AABB box(Vec3{1900, 0, 0}, Vec3{2150, 2250, 4000});
        d.add_box(box);
        v.add_box(box);
        AStarResult rd = astar_weighted(d, d.to_cell(Vec3{275, 2025, 2025}),
                                        d.to_cell(Vec3{3725, 2025, 2025}), bp);
        AStarResult rv = astar_weighted(v, v.to_cell(Vec3{275, 2025, 2025}),
                                        v.to_cell(Vec3{3725, 2025, 2025}), bp);
        bool eq = rd.success == rv.success && rd.length_mm == rv.length_mm &&
                  rd.turns == rv.turns && rd.expanded_nodes == rv.expanded_nodes &&
                  rd.path == rv.path;
        check(eq && rv.length_mm == 3950.0 && rv.turns == 2, "golden02 Dense==Vdb (지표+경로)");
    }

    // 골든03(다중 순차): 5/5 성공, 충돌 0, 총길이 28050.
    {
        const Cell shape{120, 120, 60};
        VdbOccupancy v(shape, Vec3{0, 0, 0}, 50.0);
        v.add_box(AABB(Vec3{0, 0, 0}, Vec3{6000, 6000, 250}));
        RouteTask t;
        t.start_mm = Vec3{275, 3025, 1525};
        t.end_mm = Vec3{5725, 3025, 1525};
        std::vector<RouteTask> tasks;
        const char* utils[5][2] = {{"UPW_S", "UPW"}, {"NFW", "Waste Liquid"}, {"PA", "Gas"},
                                   {"NW", "Water"}, {"ACID", "Exhaust"}};
        for (auto& u : utils) { t.utility = u[0]; t.utility_group = u[1]; tasks.push_back(t); }
        auto mr = route_sequential(v, tasks, bp, "longest");
        // 충돌(쌍별 셀 공유) 계산.
        std::vector<std::vector<int>> sets;
        for (const PipeResult& p : mr.pipes) {
            if (!p.result.success) continue;
            std::vector<int> s;
            for (const Cell& c : p.result.path) s.push_back(v.lin(c));
            sets.push_back(std::move(s));
        }
        int collisions = 0;
        for (size_t i = 0; i < sets.size(); ++i)
            for (size_t j = i + 1; j < sets.size(); ++j) {
                bool shared = false;
                for (int a : sets[i]) { for (int b : sets[j]) if (a == b) { shared = true; break; } if (shared) break; }
                if (shared) ++collisions;
            }
        std::printf("  success=%d/%zu total_length=%.1f collisions=%d\n", mr.success_count(),
                    mr.pipes.size(), mr.total_length_mm(), collisions);
        check(mr.success_count() == 5 && mr.total_length_mm() == 28050.0 && collisions == 0,
              "golden03 Vdb 라우팅 (5/5, 28050mm, 충돌0)");
    }
}

int main() {
    test_o1();
    test_routing_cross_backend();
    test_8000m_scattered();
    test_tile_compression();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}

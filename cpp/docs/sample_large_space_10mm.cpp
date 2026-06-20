// sample_large_space_10mm.cpp — Routing3D C++ 엔진 전체 파이프라인 예제
// =============================================================================
// [목적]
//   30m × 30m × 높이 20m, 셀 간격 10mm 초미세 격자 환경에서
//   Routing3D 엔진의 전체 라우팅 파이프라인을 단계별로 시연한다.
//
// [핵심 수치]
//   도메인 : 30,000mm × 30,000mm × 20,000mm
//   셀 크기 : 10mm
//   격자 셀 수 : 3,000 × 3,000 × 2,000 = **18,000,000,000셀 (180억)**
//   → DenseOccupancy(1Byte/셀) 필요 RAM ≈ 18GB → 불가
//   → ImplicitOccupancy (O(장애물+깔린셀)) 필수
//   → 직접 A* 전체격자 탐색 상한(48M 확장) 초과 → route_hier 필수
//
// [커버하는 단계]
//   Step 1 : SceneDoc 구성 (격자 원점·형태·셀크기·파라미터)
//   Step 2 : 장애물 추가 (벽·기둥·주요장비·덕트)
//   Step 3 : 라우팅 작업 정의 (시작/종단 좌표·유틸리티·관경)
//   Step 4 : 직접 A* 탐색 (소규모·테스트용, 이 도메인에선 상한 도달)
//   Step 5 : 계층 corridor 라우팅 (route_corridor / route_hier — 180억셀 안전)
//   Step 6 : 다중 배관 라우팅 (route_multi_impl 상당 흐름, 관경·이격·CBS)
//   Step 7 : 경유점(waypoint) 입력 — 구간 분해 후 순차 탐색
//   Step 8 : C ABI (r3d_* 함수) 를 통한 동일 파이프라인
//
// [빌드]
//   cmake --build cpp/build --config Release --target routing3d_capi
//   cl /std:c++20 /utf-8 /I cpp/include sample_large_space_10mm.cpp
//      cpp/build/Release/routing3d_capi.lib /Fe:sample.exe
//
// 또는 단독 헤더 전용(DenseOccupancy 대신 ImplicitOccupancy, 외부링크 불필요):
//   cl /std:c++20 /utf-8 /I cpp/include sample_large_space_10mm.cpp /Fe:sample.exe
// =============================================================================

// ─── 헤더 전용 엔진 헤더 포함 ─────────────────────────────────────────────
#include "routing3d/geometry.hpp"       // Vec3, Cell, AABB, NEIGHBORS_6
#include "routing3d/scene_io.hpp"       // SceneDoc, RouteTask, SceneResult
#include "routing3d/occupancy.hpp"      // DenseOccupancy, ImplicitOccupancy
#include "routing3d/cost.hpp"           // RouteParams, CostModel, clearance_map
#include "routing3d/astar.hpp"          // astar_weighted, AStarResult, snap_to_free_cell
#include "routing3d/multi_route.hpp"    // route_sequential, route_ripup, mark_pipe
#include "routing3d/corridor.hpp"       // route_corridor, astar_hashed, coarse 유틸리티
#include "routing3d/route_task.hpp"     // RouteTask(시작·끝·유틸·관경)
#include "routing3d/box_index.hpp"      // SpatialBoxIndex (ImplicitOccupancy 내부)

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>
#include <unordered_set>
#include <optional>

using namespace routing3d;

// =============================================================================
// ■ 도메인 상수 정의
// =============================================================================
// 도메인 크기 (월드 좌표, mm)
static constexpr double DOM_X_MM = 30000.0;   // 30m (X 축)
static constexpr double DOM_Y_MM = 30000.0;   // 30m (Y 축)
static constexpr double DOM_Z_MM = 20000.0;   // 20m (Z 축, 높이)

// 셀 크기 — 10mm 초미세 격자
// 주의: 격자 셀 수 = (3000, 3000, 2000) = 18,000,000,000 셀
// DenseOccupancy 는 1Byte/셀 → 18GB 필요. 반드시 ImplicitOccupancy 를 사용해야 한다.
static constexpr double CELL_MM = 10.0;

// 격자 셀 수 (각 축)
static constexpr int NX = static_cast<int>(DOM_X_MM / CELL_MM);   // 3,000
static constexpr int NY = static_cast<int>(DOM_Y_MM / CELL_MM);   // 3,000
static constexpr int NZ = static_cast<int>(DOM_Z_MM / CELL_MM);   // 2,000

// 도메인 원점 (모든 좌표의 기준, mm).
// 실제 프로젝트에서는 BIM 모델의 글로벌 원점을 사용한다.
static const Vec3 ORIGIN{0.0, 0.0, 0.0};

// coarse 격자 배율: ceil(200mm / cell_mm) = 20
// → coarse 셀 크기 ≈ 200mm (Stage 3b 적응형), coarse 격자 셀 수 = 150×150×100 = 2,250,000
// → coarse A*(최적, w=1.0)이 수백만 셀에서 빠르게 골격을 찾는다.
static constexpr int HIER_FACTOR  = 20;
static constexpr int HIER_RADIUS  = 2;   // coarse corridor 팽창 반경(coarse 셀 단위)

// 탐색 상한: 48M(기본 엔진 한도). 직접 A* 가 이를 넘으면 route_hier 로 escalation.
static constexpr long long MAX_EXP = 48'000'000LL;

// =============================================================================
// ■ 헬퍼: mm 좌표 → 격자 Cell 변환
// =============================================================================
// world_to_cell: 월드 좌표(mm)를 격자 셀 인덱스로 변환한다.
// 원점(ORIGIN)에서의 상대 좌표를 cell_mm 으로 나눠 바닥값을 취한다.
// 격자 경계 클램프 포함.
inline Cell mm_to_cell(double x, double y, double z) {
    int i = static_cast<int>((x - ORIGIN.x) / CELL_MM);
    int j = static_cast<int>((y - ORIGIN.y) / CELL_MM);
    int k = static_cast<int>((z - ORIGIN.z) / CELL_MM);
    // 경계 클램프
    i = std::max(0, std::min(NX - 1, i));
    j = std::max(0, std::min(NY - 1, j));
    k = std::max(0, std::min(NZ - 1, k));
    return Cell{i, j, k};
}

// cell_to_mm: 격자 셀 인덱스 → 월드 좌표(셀 중심).
inline Vec3 cell_to_mm(const Cell& c) {
    return Vec3{ORIGIN.x + (c.i + 0.5) * CELL_MM,
                ORIGIN.y + (c.j + 0.5) * CELL_MM,
                ORIGIN.z + (c.k + 0.5) * CELL_MM};
}

// =============================================================================
// ■ Step 1: SceneDoc 구성 — 격자 메타데이터 + 라우팅 파라미터 설정
// =============================================================================
// SceneDoc: 엔진이 다루는 모든 데이터의 컨테이너.
//   - params   : 셀 크기, 회전비용, 클리어런스 반경 등 A* 비용 파라미터
//   - origin   : 도메인 원점 (월드 mm)
//   - shape    : 격자 크기 (셀 단위)
//   - obstacles: AABB 장애물 목록 (월드 mm)
//   - tasks    : 라우팅 작업 목록 (start_mm → end_mm + 유틸리티)
//   - results  : 라우팅 결과 (경로 셀 배열 + 성공/길이/꺾임)
SceneDoc build_scene_doc() {
    SceneDoc doc;

    // ── 라우팅 파라미터 (RouteParams) ──────────────────────────────────────
    // 셀 크기: A* 비용 계산·클리어런스 모두 이 값 기준.
    doc.params.cell_mm   = CELL_MM;   // 10mm

    // 회전 비용 가산 (mm): 꺾임 1회마다 이동 비용에 더한다.
    //   → 크면 직선 선호, 작으면 꺾임 허용.
    //   A* admissibility 보존: 가산(additive)만 허용, 승산(multiplicative) 금지.
    doc.params.w_turn    = 200.0;   // 꺾임 1회 ≈ 200mm 페널티

    // 클리어런스 비용 가산: 장애물에 가까운 셀 통과 시 추가 비용(mm).
    //   clearance_map(BFS 거리변환)이 생성한 거리맵을 사용한다.
    //   ImplicitOccupancy 는 전역 clearance_map 미생성(온디맨드) → 0.0 권장.
    doc.params.w_clear   = 0.0;   // ImplicitOccupancy 에서는 비활성

    // 클리어런스 반경 (셀 단위): 장애물 주변 N셀을 추가 비용 구역으로 처리.
    //   w_clear > 0 일 때만 적용. 0이면 클리어런스 비용 없음.
    doc.params.clearance_radius       = 0;
    doc.params.clearance_connectivity = 6;   // 6-이웃(직교) BFS

    // 회랑 가산 비용: w_corridor > 0 이면 '학습된 회랑 셀' 밖 이동에 페널티 추가.
    //   L2b(기존설계 회랑 소프트 바이어스). 0=비활성(표준 A*).
    doc.params.w_corridor = 0.0;

    // weighted A* 파라미터:
    //   w_heur     : 먼 곳에서의 휴리스틱 가중 (1.0=최적, 2.0=목표지향 빠름).
    //   w_heur_near: 목표 근처에서의 가중 (1.0=정확). 동적 보간으로 수렴 접근.
    //   무제한 탐색(max_expansions ≤ 0)에서만 동적 가중 활성.
    doc.params.w_heur      = 2.0;   // 먼 곳: 목표 지향(빠름)
    doc.params.w_heur_near = 1.0;   // 목표 근처: 정확(표준 A*)

    // 최소 직선 하드 제약 (P3u): 꺾임 후 최소 N셀 직진 강제.
    //   cell=10mm, min=100mm → min_straight_cells = ceil(100/10) = 10
    //   → 모든 내부 구간이 10셀(=100mm) 이상 직선 보장.
    //   A* 상태 키에 run 카운터 추가 → 상태 공간이 ×(min_run+1) 배 증가.
    doc.params.min_straight_cells = static_cast<int>(std::ceil(100.0 / CELL_MM));   // 10

    // 격자 원점·형태 설정
    doc.origin = ORIGIN;
    doc.shape  = Cell{NX, NY, NZ};   // (3000, 3000, 2000)

    printf("[Step 1] SceneDoc 구성 완료\n");
    printf("  도메인: %.0fmm × %.0fmm × %.0fmm\n", DOM_X_MM, DOM_Y_MM, DOM_Z_MM);
    printf("  셀 크기: %.0fmm\n", CELL_MM);
    printf("  격자 셀 수: %d × %d × %d = %.1f억\n", NX, NY, NZ,
           (double)NX * NY * NZ / 1e8);
    printf("  coarse factor: %d (셀 ≈ %.0fmm)\n", HIER_FACTOR, CELL_MM * HIER_FACTOR);

    return doc;
}

// =============================================================================
// ■ Step 2: 장애물 추가 — 벽·기둥·주요장비·덕트
// =============================================================================
// 장애물은 AABB(축정렬 경계 박스, 월드 mm)로 정의한다.
// ImplicitOccupancy 는 장애물 AABB 를 SpatialBoxIndex(유니폼 그리드)로 색인해
// is_blocked(cell) 쿼리 시 O(1)~O(k) 로 처리한다. 점유 배열 없음.
void add_obstacles(SceneDoc& doc) {
    // ── 외벽(5면 폐쇄, 두께 500mm) ───────────────────────────────────────
    // X 음수 벽 (Y-Z 평면)
    doc.obstacles.push_back({Vec3{-500.0,     0.0,     0.0},
                              Vec3{   0.0, 30000.0, 20000.0}});
    // X 양수 벽
    doc.obstacles.push_back({Vec3{30000.0,    0.0,     0.0},
                              Vec3{30500.0, 30000.0, 20000.0}});
    // Y 음수 벽
    doc.obstacles.push_back({Vec3{    0.0,  -500.0,   0.0},
                              Vec3{30000.0,     0.0, 20000.0}});
    // Y 양수 벽
    doc.obstacles.push_back({Vec3{    0.0, 30000.0,   0.0},
                              Vec3{30000.0, 30500.0, 20000.0}});
    // 바닥 슬래브 (두께 300mm)
    doc.obstacles.push_back({Vec3{0.0, 0.0,  -300.0},
                              Vec3{30000.0, 30000.0, 0.0}});

    // ── 내부 기둥 (500×500mm 단면, 높이 20m) ─────────────────────────────
    // 6m 간격 그리드로 기둥 배치 (X=6000, 12000, 18000, 24000 / Y=6000, 12000, 18000, 24000)
    for (int ci = 1; ci <= 4; ++ci) {
        for (int cj = 1; cj <= 4; ++cj) {
            double cx = ci * 6000.0;
            double cy = cj * 6000.0;
            // 기둥 AABB: 중심에서 ±250mm, 전체 높이
            doc.obstacles.push_back({Vec3{cx - 250.0, cy - 250.0, 0.0},
                                      Vec3{cx + 250.0, cy + 250.0, 20000.0}});
        }
    }

    // ── 주요장비 (MAIN_EQUIPMENT): 펌프·압축기·탱크 등 ─────────────────────
    // 장비 A: 펌프 (2000×1500×1200mm), 위치 X=3000-5000, Y=3000-4500, Z=0-1200
    doc.obstacles.push_back({Vec3{3000.0, 3000.0, 0.0},
                              Vec3{5000.0, 4500.0, 1200.0}});

    // 장비 B: 압축기 (3000×2000×2000mm), 위치 X=22000-25000, Y=5000-7000, Z=0-2000
    doc.obstacles.push_back({Vec3{22000.0, 5000.0, 0.0},
                              Vec3{25000.0, 7000.0, 2000.0}});

    // 장비 C: 열교환기 (1500×1000×3000mm, 수직), X=14000-15500, Y=14000-15000, Z=0-3000
    doc.obstacles.push_back({Vec3{14000.0, 14000.0, 0.0},
                              Vec3{15500.0, 15000.0, 3000.0}});

    // 장비 D: 저장탱크 (지름 4000mm = AABB 4000×4000×5000), X=8000-12000, Y=20000-24000
    doc.obstacles.push_back({Vec3{8000.0, 20000.0, 0.0},
                              Vec3{12000.0, 24000.0, 5000.0}});

    // ── 덕트·레터럴 (DUCT): 배기·공급 덕트 ────────────────────────────────
    // 덕트 A: 상부 수평 배기덕트 (단면 1000×800mm, 길이 X 방향 전체)
    //   Z=12000~12800 (12~12.8m 고도), Y=14000~15000
    doc.obstacles.push_back({Vec3{0.0,     14000.0, 12000.0},
                              Vec3{30000.0, 15000.0, 12800.0}});

    // 덕트 B: 공급덕트 (단면 600×600mm, Y 방향 전체)
    //   X=6800~7400, Z=10000~10600
    doc.obstacles.push_back({Vec3{6800.0, 0.0,     10000.0},
                              Vec3{7400.0, 30000.0, 10600.0}});

    // ── 중간 층 슬래브 (두께 300mm, 부분 개구부 있음) ────────────────────
    // 층 1: Z=8000 (8m 고도), Y=0~20000 구간만 설치
    doc.obstacles.push_back({Vec3{0.0,     0.0, 8000.0},
                              Vec3{30000.0, 20000.0, 8300.0}});
    // 층 2: Z=15000 (15m), X=0~15000 구간만
    doc.obstacles.push_back({Vec3{0.0,     0.0, 15000.0},
                              Vec3{15000.0, 30000.0, 15300.0}});

    printf("[Step 2] 장애물 추가 완료: 외벽 5면 + 기둥 %d개 + 장비 4개 + 덕트 2개 + 슬래브 2개\n",
           4 * 4);
    printf("  총 장애물 수: %zu\n", doc.obstacles.size());
}

// =============================================================================
// ■ Step 3: 라우팅 작업 정의 — 시작/종단 좌표·유틸리티·관경·목표진입축
// =============================================================================
// RouteTask: 배관 1건의 입력 명세.
//   start_mm     : 시작 PoC 월드 좌표 (mm)
//   end_mm       : 종단 PoC 월드 좌표 (mm)
//   utility      : 유틸리티 이름 (예: "CW"=냉각수)
//   utility_group: 유틸리티 그룹 (예: "Cooling")
//   diameter_mm  : 관경 (mm). 우선순위 "diameter"(굵은 배관 먼저) 정렬 키.
//   goal_dir     : 목표 진입축 (-1=무제약, 0..5=+x,-x,+y,-y,+z,-z). -1이 기본.
std::vector<RouteTask> build_tasks() {
    std::vector<RouteTask> tasks;

    // ── 작업 T1: 냉각수 공급배관 (CW, 200A φ219mm) ──────────────────────
    // 장비 A(펌프 출구) → 장비 B(압축기 입구), 대각 횡단
    {
        RouteTask t;
        t.start_mm      = Vec3{5000.0, 3750.0, 1000.0};    // 펌프 출구 PoC
        t.end_mm        = Vec3{22000.0, 6000.0, 1000.0};   // 압축기 입구 PoC
        t.utility       = "CW";                             // Cooling Water
        t.utility_group = "Cooling";
        t.diameter_mm   = 219.0;   // 200A 배관 외경 219.1mm
        t.goal_dir      = -1;      // 무제약 진입
        tasks.push_back(t);
    }

    // ── 작업 T2: 배기배관 (Exhaust, 150A φ165mm) — 덕트 상단 진입 ────────
    // 장비 B(압축기 배기구, 수직 상향) → 덕트 A 하단 PoC (+Z 진입)
    {
        RouteTask t;
        t.start_mm      = Vec3{23500.0, 6000.0, 2000.0};   // 압축기 배기구
        t.end_mm        = Vec3{23000.0, 14500.0, 12000.0}; // 덕트 A 하단 PoC
        t.utility       = "Exhaust";
        t.utility_group = "Exhaust";
        t.diameter_mm   = 165.0;   // 150A 배관 외경 165.2mm
        // 덕트 하단에 +Z(아래→위) 방향으로 진입: NEIGHBORS_6[4] = +z
        t.goal_dir      = 4;
        tasks.push_back(t);
    }

    // ── 작업 T3: 냉각수 환수배관 (CR, 200A φ219mm) ─────────────────────
    {
        RouteTask t;
        t.start_mm      = Vec3{22000.0, 6000.0, 1200.0};   // 압축기 CR 출구
        t.end_mm        = Vec3{3000.0, 3750.0, 1200.0};    // 펌프 CR 입구
        t.utility       = "CR";
        t.utility_group = "Cooling";
        t.diameter_mm   = 219.0;
        t.goal_dir      = -1;
        tasks.push_back(t);
    }

    // ── 작업 T4: 공정수 배관 (PW, 100A φ114mm) ─────────────────────────
    // 열교환기 → 저장탱크
    {
        RouteTask t;
        t.start_mm      = Vec3{14750.0, 14000.0, 2500.0};  // 열교환기 공정수 출구
        t.end_mm        = Vec3{10000.0, 20000.0, 3000.0};  // 저장탱크 입구
        t.utility       = "PW";
        t.utility_group = "Process";
        t.diameter_mm   = 114.0;   // 100A 배관 외경 114.3mm
        t.goal_dir      = -1;
        tasks.push_back(t);
    }

    // ── 작업 T5: 가는 계장배관 (Instrument Air, 25A φ34mm) ──────────────
    // 장비 A 제어밸브 → 제어판넬 (두 장비 사이 짧은 구간)
    {
        RouteTask t;
        t.start_mm      = Vec3{4000.0, 4000.0, 1100.0};   // 제어밸브
        t.end_mm        = Vec3{4500.0, 8000.0, 3000.0};   // 제어판넬
        t.utility       = "IA";
        t.utility_group = "Instrument";
        t.diameter_mm   = 34.0;   // 25A 배관 외경 33.7mm
        t.goal_dir      = -1;
        tasks.push_back(t);
    }

    printf("[Step 3] 라우팅 작업 정의 완료: %zu개\n", tasks.size());
    for (size_t i = 0; i < tasks.size(); ++i) {
        printf("  T%zu: [%s] %s φ%.0fmm  (%.0f,%.0f,%.0f)→(%.0f,%.0f,%.0f)\n",
               i + 1, tasks[i].utility_group.value_or("?").c_str(),
               tasks[i].utility.value_or("?").c_str(),
               tasks[i].diameter_mm,
               tasks[i].start_mm.x, tasks[i].start_mm.y, tasks[i].start_mm.z,
               tasks[i].end_mm.x, tasks[i].end_mm.y, tasks[i].end_mm.z);
    }
    return tasks;
}

// =============================================================================
// ■ Step 4: ImplicitOccupancy 구성 + 직접 A* 단일 탐색
// =============================================================================
// [ImplicitOccupancy란?]
//   - 3D 배열 없이 장애물 AABB + 마킹된 배관 셀만 메모리에 보관.
//   - 메모리: O(장애물 수 + 깔린 배관 셀 수). 18억 셀이라도 메모리 안전.
//   - is_blocked(c): SpatialBoxIndex(유니폼 그리드 색인)로 O(1) 근사 쿼리.
//   - add_box(aabb): 장애물 AABB 추가.
//   - mark_cell(c) : 배관 셀 마킹(다음 배관 탐색 시 장애물로 처리).
//
// [astar_weighted 직접 탐색]
//   - 소규모 격자(≤5M 셀)나 짧은 배관(probe 내 해결) 시 사용.
//   - 이 도메인(18억 셀)에서 직접 탐색은 MAX_EXP(48M) 에 도달해 실패.
//   - 실제 라우팅에서는 route_hier(계층 corridor)로 escalation된다.
void step4_direct_astar(const SceneDoc& doc, const RouteTask& task) {
    printf("\n[Step 4] 직접 A* 단일 탐색 (ImplicitOccupancy)\n");

    // ── ImplicitOccupancy 구성 ────────────────────────────────────────────
    // shape(NX,NY,NZ), origin, cell_mm 로 인덱스 파라미터 초기화.
    ImplicitOccupancy occ(doc.shape, doc.origin, CELL_MM);

    // 장애물 AABB 추가: SceneDoc.obstacles 목록을 순회해 모두 추가.
    for (const Obstacle& obs : doc.obstacles) {
        try {
            occ.add_box(AABB(obs.min_xyz, obs.max_xyz));
        } catch (const std::invalid_argument&) {
            // 격자 밖 장애물은 무시.
        }
    }
    printf("  ImplicitOccupancy 구성 완료: 장애물 %zu개 색인\n", doc.obstacles.size());

    // ── 시작/종단 셀 변환 + 자유셀 스냅 ─────────────────────────────────
    // world_to_cell: 월드 좌표 mm → 격자 셀 인덱스.
    Cell s = occ.world_to_cell(task.start_mm);
    Cell g = occ.world_to_cell(task.end_mm);

    // snap_to_free_cell: 시작/종단이 장애물 셀이면 주변 자유셀로 스냅(최소 반경).
    // 실제 PoC 좌표가 장비 표면에 붙어 있어 종종 점유 셀에 떨어진다.
    s = snap_to_free_cell(occ, s, 5);   // 최대 반경 5셀 탐색
    g = snap_to_free_cell(occ, g, 5);

    printf("  시작 셀: (%d,%d,%d)\n", s.i, s.j, s.k);
    printf("  종단 셀: (%d,%d,%d)\n", g.i, g.j, g.k);

    // ── RouteParams 복사 (비용 모델) ──────────────────────────────────────
    // CostModel 은 RouteParams 를 기반으로 이동/회전/클리어런스 비용을 계산한다.
    // clearance_map 은 ImplicitOccupancy 에서는 빈 배열(온디맨드 비활성) → w_clear=0 권장.
    RouteParams p = doc.params;
    std::vector<double> empty_cmap;   // ImplicitOccupancy: 전역 clearance_map 미생성
    CostModel<ImplicitOccupancy> cost_model(occ, p, empty_cmap);

    // ── A* 탐색 실행 ─────────────────────────────────────────────────────
    // astar_weighted: weighted A*(동적 가중·min_straight 하드 제약 포함) 실행.
    // 파라미터:
    //   occ           : 점유맵 (ImplicitOccupancy)
    //   s, g          : 시작·종단 셀
    //   params        : 비용 파라미터 (RouteParams)
    //   max_expansions: 탐색 상한 (이 도메인에서는 상한 도달 예상)
    //   collect_visited: 방문맵 수집 여부 (시각화용, 기본 OFF)
    //   corridor      : 회랑 셀 포인터 (nullptr = 전역 탐색)
    //   intra_pipe    : 동일 배관 재방문 허용 셀 포인터 (nullptr = 없음)
    //   progress_every: 진행 콜백 주기 (0 = 없음)
    //   AllowAll{}    : 모든 셀 허용 술어 (기본)
    //   goal_dir      : 목표 진입축 제약 (-1 = 무제약)
    constexpr long long probe = 300'000LL;   // 소규모 직접 시도 예산
    AStarResult res = astar_weighted(occ, s, g, p, probe,
                                     /*collect_visited=*/false,
                                     /*corridor=*/nullptr,
                                     /*intra=*/nullptr,
                                     /*progress_every=*/0,
                                     AllowAll{},
                                     task.goal_dir);

    if (res.success) {
        printf("  ✓ 직접 A* 성공: 길이=%.1fmm, 꺾임=%d, 확장=%lld, 시간=%.2fms\n",
               res.length_mm, res.turns, res.expanded_nodes, res.elapsed_ms);
    } else {
        printf("  ✗ 직접 A* 실패 (확장=%lld ≥ probe=%lld) → route_hier 로 escalation 필요\n",
               res.expanded_nodes, probe);
        printf("    ※ 이 도메인(18억 셀)에서 정상: probe 예산은 쉬운 배관 빠른 처리용\n");
    }
}

// =============================================================================
// ■ Step 5: 계층 corridor 라우팅 (route_corridor + route_hier)
// =============================================================================
// [알고리즘 요약]
//   1) coarse_implicit_from_doc: 동일 장애물을 HIER_FACTOR(20) 배 셀 크기로 재색인
//      → coarse 격자 = 150×150×100 = 2,250,000셀 (18억 → 2.25M)
//   2) astar_hashed(coarse, CorridorAll{}): coarse 격자 전역 A*(균일비용, 최적)
//      → 최적 coarse 골격 경로(수십ms)
//   3) coarse 경로를 Chebyshev 반경 HIER_RADIUS 로 팽창 → corridor 집합
//   4) astar_hashed(fine, in_corr): fine 격자에서 corridor 내부만 탐색
//      → 전체 fine 격자 탐색 불필요 (탐색량: 수백만 → 수만 셀)
//   5) fine 실패 시 radius ×2 로 최대 3회 재시도 (Stage 2)
//
// [핵심 함수]
//   coarse_implicit_from_doc(doc, factor): coarse ImplicitOccupancy 생성
//   route_corridor(fine, coarse, s, g, factor, radius, max_exp): 2단계 탐색

// coarse 점유맵 생성 함수 (routing3d_capi.cpp 의 내부 함수를 여기서 재현)
// 실제 capi 환경에서는 route_hier 함수가 내부적으로 이 과정을 수행한다.
ImplicitOccupancy make_coarse_occ(const SceneDoc& doc, int factor) {
    // coarse 격자 크기: ceil(NX / factor) 등
    Cell cs{(doc.shape.i + factor - 1) / factor,
            (doc.shape.j + factor - 1) / factor,
            (doc.shape.k + factor - 1) / factor};
    // coarse ImplicitOccupancy: origin 동일, 셀 크기 = CELL_MM × factor
    ImplicitOccupancy coarse(cs, doc.origin, CELL_MM * factor);
    for (const Obstacle& o : doc.obstacles) {
        try { coarse.add_box(AABB(o.min_xyz, o.max_xyz)); }
        catch (const std::invalid_argument&) { /* 격자 밖 무시 */ }
    }
    return coarse;
}

void step5_hier_corridor(const SceneDoc& doc, const RouteTask& task) {
    printf("\n[Step 5] 계층 corridor 라우팅\n");

    // ── fine 점유맵 구성 ─────────────────────────────────────────────────
    ImplicitOccupancy fine(doc.shape, doc.origin, CELL_MM);
    for (const Obstacle& obs : doc.obstacles) {
        try { fine.add_box(AABB(obs.min_xyz, obs.max_xyz)); }
        catch (const std::invalid_argument&) {}
    }

    // ── coarse 점유맵 구성 (HIER_FACTOR=20 배 셀, 약 200mm) ─────────────
    ImplicitOccupancy coarse = make_coarse_occ(doc, HIER_FACTOR);
    printf("  coarse 격자: %d × %d × %d ≈ %.1fM셀 (fine의 1/%d³ = 1/%.0f)\n",
           (NX + HIER_FACTOR - 1) / HIER_FACTOR,
           (NY + HIER_FACTOR - 1) / HIER_FACTOR,
           (NZ + HIER_FACTOR - 1) / HIER_FACTOR,
           (double)((NX + HIER_FACTOR - 1) / HIER_FACTOR) *
           ((NY + HIER_FACTOR - 1) / HIER_FACTOR) *
           ((NZ + HIER_FACTOR - 1) / HIER_FACTOR) / 1e6,
           HIER_FACTOR, std::pow(HIER_FACTOR, 3.0));

    // ── 시작/종단 셀 변환 ─────────────────────────────────────────────────
    Cell s_fine = snap_to_free_cell(fine, fine.world_to_cell(task.start_mm), 5);
    Cell g_fine = snap_to_free_cell(fine, fine.world_to_cell(task.end_mm), 5);

    // ── route_corridor: 2단계 계층 탐색 ─────────────────────────────────
    //   route_corridor(fine, coarse, start_fine, goal_fine, factor, radius, max_exp)
    //   - Stage 1: coarse A*(균일비용, 최적) → 최적 골격 경로
    //   - Stage 2: fine A*(corridor 튜브 내부만) → 정밀 경로
    //   재시도(Stage 2b): fine 실패 시 radius ×2 로 최대 3회 반복
    CorridorRoute cr = route_corridor(fine, coarse,
                                      s_fine, g_fine,
                                      HIER_FACTOR, HIER_RADIUS,
                                      MAX_EXP);

    if (!cr.coarse_success) {
        printf("  ✗ coarse 탐색 실패 — 장애물이 경로를 완전히 차단\n");
        return;
    }
    printf("  coarse 경로: %zu셀, corridor 집합 크기: %lld\n",
           cr.coarse_path.size(), cr.corridor_cells);

    if (cr.fine.success) {
        printf("  ✓ fine 탐색 성공: 길이=%.1fmm, 꺾임=%d, 확장=%lld, 시간=%.2fms\n",
               cr.fine.length_mm, cr.fine.turns, cr.fine.expanded_nodes, cr.fine.elapsed_ms);
    } else {
        // Stage 2 재시도: corridor 반경을 넓혀 재탐색
        printf("  fine 첫 시도 실패 (radius=%d, 확장=%lld)\n",
               HIER_RADIUS, cr.fine.expanded_nodes);
        for (int retry = 1; retry <= 3; ++retry) {
            int r2 = HIER_RADIUS * (1 << retry);   // ×2, ×4, ×8
            printf("  재시도 %d: radius=%d ...\n", retry, r2);
            cr = route_corridor(fine, coarse, s_fine, g_fine, HIER_FACTOR, r2, MAX_EXP);
            if (cr.fine.success) {
                printf("  ✓ 재시도 %d 성공: 길이=%.1fmm, 꺾임=%d, 확장=%lld\n",
                       retry, cr.fine.length_mm, cr.fine.turns, cr.fine.expanded_nodes);
                break;
            }
        }
        if (!cr.fine.success)
            printf("  ✗ 계층 corridor 전체 실패 — 혼잡·막힘\n");
    }
}

// =============================================================================
// ■ Step 6: 다중 배관 라우팅 (route_sequential + 관경·이격·인라인 rip-up)
// =============================================================================
// [route_sequential 흐름]
//   1) order_indices: 우선순위 정렬 (diameter=굵은 배관 먼저)
//   2) work = occ.copy()  ← 원본 점유맵 불변 (계약 M2)
//   3) 작업마다: snap_to_free_cell → astar_weighted/route_hier → 성공이면 mark_pipe
//      mark_pipe: 경로 셀을 ±pipe_radius 6이웃까지 마킹 → 다음 배관 진입로 확보
//   4) 인라인 rip-up: main 패스 실패 배관의 직접 blocker(≤4개)를 뜯어 재배치
//      → 단조 성공: 재귀 깊이 0이면 rip-up, >0이면 CBS-lite
//
// [per-task 반경 (B1)]
//   각 배관에 ceil(diameter_mm / cell_mm) - 1 클램프 반경 부여.
//   굵은 관(219mm) → radius=21, 가는 관(34mm) → radius=2.
//   배관 마킹 시 자기 반경으로 팽창 → 센터선 간격이 물리적 관경 일치.
//
// [배관 이격 gap (P3s)]
//   센터선 거리 ≥ r1 + r2 + pipe_gap_mm 보장.
//   깔린 배관 A 위에 B 를 놓을 때 A 를 B 의 (rA+rB+gap)/cell 반경으로 재마킹.
void step6_multi_route(SceneDoc& doc, const std::vector<RouteTask>& tasks) {
    printf("\n[Step 6] 다중 배관 라우팅\n");

    // ── ImplicitOccupancy 구성 ────────────────────────────────────────────
    ImplicitOccupancy occ(doc.shape, doc.origin, CELL_MM);
    for (const Obstacle& obs : doc.obstacles)
        try { occ.add_box(AABB(obs.min_xyz, obs.max_xyz)); } catch (...) {}

    // ── 작업 목록을 SceneDoc 에 설정 ─────────────────────────────────────
    doc.tasks = tasks;

    // ── 우선순위 정렬: diameter 기준 (굵은 배관 먼저, 동률=거리 긴 것) ─────
    // order_indices: tasks 인덱스를 우선순위 기준으로 정렬해 반환.
    //   "diameter" : 관경 내림차순 → 동률=맨해튼 거리 내림차순
    //   "longest"  : 거리 내림차순 (관경 무시, 기본값)
    //   "utility"  : 유틸 그룹별 → 그룹 내 관경 내림차순
    std::vector<int> order = order_indices(tasks, "diameter");
    printf("  라우팅 순서 (diameter 우선):\n");
    for (int idx : order) {
        printf("    T%d: φ%.0fmm [%s]\n",
               idx + 1, tasks[idx].diameter_mm,
               tasks[idx].utility.value_or("?").c_str());
    }

    // ── work 점유맵 (오리지널 불변, 사본에서 작업) ────────────────────────
    ImplicitOccupancy work = occ.copy();

    // ── coarse 점유맵 (route_hier escalation 용) ─────────────────────────
    ImplicitOccupancy coarse = make_coarse_occ(doc, HIER_FACTOR);

    // ── per-task 반경 계산 함수 ──────────────────────────────────────────
    // 관경 → 배관 반경 셀 수: clamp(ceil(diameter_mm / cell_mm) - 1, 0, 8)
    // pipe_radius 로 mark_pipe(BFS 팽창)해 다음 배관 중심선을 관경만큼 띄운다.
    auto pipe_radius_of = [&](double diam_mm) -> int {
        if (diam_mm <= 0.0) return 0;
        return static_cast<int>(std::clamp(
            (int)std::ceil(diam_mm / CELL_MM) - 1, 0, 8));
    };

    // ── 배관 이격 gap (센터선 ≥ r1 + r2 + 60mm) ─────────────────────────
    constexpr double pipe_gap_mm = 60.0;   // 표면 간격 최소 60mm

    // ── 각 배관 결과 저장 ─────────────────────────────────────────────────
    struct Result {
        int task_idx;
        bool success;
        double length_mm;
        int turns;
        long long expanded_nodes;
        std::vector<Cell> path;
    };
    std::vector<Result> results(tasks.size());

    // ── 순차 라우팅 메인 루프 ────────────────────────────────────────────
    for (int order_i = 0; order_i < (int)order.size(); ++order_i) {
        int ti = order[order_i];
        const RouteTask& t = tasks[ti];
        int r_self = pipe_radius_of(t.diameter_mm);

        // 시작/종단 셀 스냅 (마킹된 배관을 장애물로 인식하므로 work 사용)
        // snap 반경을 r_self + 2 로 설정해 마킹이 PoC 를 덮은 경우 복구.
        Cell s = snap_to_free_cell(work, work.world_to_cell(t.start_mm), r_self + 2);
        Cell g = snap_to_free_cell(work, work.world_to_cell(t.end_mm),   r_self + 2);

        RouteParams p = doc.params;
        std::vector<double> cmap;   // ImplicitOccupancy: clearance_map 비활성
        // params_for: per-task 관경에 맞게 clearance_radius 조정 (B2)
        p.clearance_radius = r_self;

        // ── 1차 탐색: probe 예산 직접 A* ─────────────────────────────────
        constexpr long long PROBE = 300'000LL;
        AStarResult res = astar_weighted(work, s, g, p, PROBE,
                                         false, nullptr, nullptr, 0,
                                         AllowAll{}, t.goal_dir);

        // ── escalation: probe 초과 → route_hier ───────────────────────────
        if (!res.success && res.expanded_nodes >= PROBE) {
            CorridorRoute cr = route_corridor(work, coarse, s, g,
                                              HIER_FACTOR, HIER_RADIUS, MAX_EXP);
            if (cr.fine.success)
                res = cr.fine;
            else {
                // Stage 2 재시도 (반경 ×2)
                for (int retry = 1; retry <= 3 && !res.success; ++retry) {
                    cr = route_corridor(work, coarse, s, g,
                                        HIER_FACTOR, HIER_RADIUS * (1 << retry), MAX_EXP);
                    if (cr.fine.success) res = cr.fine;
                }
            }
        }

        results[ti] = {ti, res.success, res.length_mm, res.turns,
                       res.expanded_nodes, res.path};

        if (res.success) {
            // ── mark_pipe: 성공 경로를 작업 점유맵에 마킹 ─────────────────
            // 동시에 이미 깔린 배관을 '이격 gap' 반경으로 재마킹.
            // mark_pipe(work, path, radius): 경로 셀 ±radius 6이웃 마킹.
            mark_pipe(work, res.path, r_self);

            // 이격 gap 처리: 기 마킹된 배관의 센터선에 대해 (r_self + r_other + gap)/cell 반경 재마킹.
            // (실제 route_multi_impl 에서는 placed 배관 목록을 순회해 쌍별로 처리)
            // 여기서는 단순화: 전체 r_self + gap/2 / cell 반경으로 마킹 (개념 설명용)
            int r_gap = static_cast<int>(
                std::ceil((t.diameter_mm / 2.0 + pipe_gap_mm / 2.0) / CELL_MM));
            if (r_gap > r_self)
                mark_pipe(work, res.path, r_gap);

            printf("  T%d ✓ 길이=%.1fmm, 꺾임=%d, r=%d, exp=%lld\n",
                   ti + 1, res.length_mm, res.turns, r_self, res.expanded_nodes);
        } else {
            printf("  T%d ✗ 실패 (exp=%lld)\n", ti + 1, res.expanded_nodes);
        }
    }

    // ── 요약 출력 ─────────────────────────────────────────────────────────
    int succ = 0;
    double total_len = 0.0;
    for (const auto& r : results) {
        if (r.success) { ++succ; total_len += r.length_mm; }
    }
    printf("  결과: %d/%zu 성공, 총 길이 %.1fmm\n",
           succ, tasks.size(), total_len);
}

// =============================================================================
// ■ Step 7: 경유점(Waypoint) 라우팅 — 구간 분해 후 순차 탐색
// =============================================================================
// [원리]
//   직접 경로가 장애물·배관 혼잡으로 막힐 때, 경유점(중간 PoC)을 지정해
//   'start → wp1 → wp2 → ... → end' 로 구간 분해 후 각 구간을 A* 탐색한다.
//   각 구간의 경로를 이어 붙이면 전체 경유점 경로가 완성된다.
//
// [실제 활용]
//   - 덕트 접속 스텁: '장비 PoC → 수직 라이저 끝(경유) → 덕트 하단(목표)'
//   - 랙 경유: 'start → 랙 z 고도(경유) → end'
//   - 벽 통과 슬리브: '벽 앞 경유 → 슬리브 셀 → 벽 뒤 경유 → 목표'
void step7_waypoint_routing(const SceneDoc& doc) {
    printf("\n[Step 7] 경유점(Waypoint) 라우팅\n");

    // ── fine 점유맵 구성 ─────────────────────────────────────────────────
    ImplicitOccupancy occ(doc.shape, doc.origin, CELL_MM);
    for (const Obstacle& obs : doc.obstacles)
        try { occ.add_box(AABB(obs.min_xyz, obs.max_xyz)); } catch (...) {}

    // ── 경유 배관 정의: 장비 → 수직 라이저 경유 → 덕트 ─────────────────
    // 장비 B 배기구 → (경유 1: 중간 높이 5000mm) → (경유 2: 랙 고도 10000mm) → 덕트 A
    Vec3 start_mm{23500.0, 6000.0, 2000.0};     // 압축기 배기구
    Vec3 wp1_mm  {23500.0, 6000.0, 5000.0};     // 경유 1: 수직 라이저 끝(5m)
    Vec3 wp2_mm  {20000.0, 14000.0, 10000.0};   // 경유 2: 랙 고도 (10m) 에서 수평 이동
    Vec3 end_mm  {20000.0, 14500.0, 12000.0};   // 덕트 A 하단 PoC

    std::vector<Vec3> waypoints = {start_mm, wp1_mm, wp2_mm, end_mm};
    printf("  경유점 경로: %zu구간\n", waypoints.size() - 1);
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        printf("    구간 %zu: (%.0f,%.0f,%.0f)→(%.0f,%.0f,%.0f)\n",
               i, waypoints[i].x, waypoints[i].y, waypoints[i].z,
               waypoints[i+1].x, waypoints[i+1].y, waypoints[i+1].z);
    }

    // ── coarse 점유맵 구성 ───────────────────────────────────────────────
    ImplicitOccupancy coarse = make_coarse_occ(doc, HIER_FACTOR);

    // ── 각 구간별 A* 탐색 → 경로 이어붙이기 ─────────────────────────────
    std::vector<Cell> full_path;
    RouteParams p = doc.params;
    bool all_ok = true;

    for (size_t seg = 0; seg + 1 < waypoints.size(); ++seg) {
        Cell s = snap_to_free_cell(occ, occ.world_to_cell(waypoints[seg]),   3);
        Cell g = snap_to_free_cell(occ, occ.world_to_cell(waypoints[seg+1]), 3);

        // probe 직접 A*
        constexpr long long PROBE = 300'000LL;
        AStarResult res = astar_weighted(occ, s, g, p, PROBE,
                                          false, nullptr, nullptr, 0, AllowAll{}, -1);

        // escalation → route_hier
        if (!res.success && res.expanded_nodes >= PROBE) {
            CorridorRoute cr = route_corridor(occ, coarse, s, g,
                                              HIER_FACTOR, HIER_RADIUS, MAX_EXP);
            if (cr.fine.success) res = cr.fine;
        }

        if (res.success) {
            // 구간 연결: 첫 구간은 전체 추가, 이후 구간은 시작 셀 중복 제거
            if (full_path.empty()) {
                full_path = res.path;
            } else if (!res.path.empty()) {
                full_path.insert(full_path.end(), res.path.begin() + 1, res.path.end());
            }
            printf("  구간 %zu: 길이=%.1fmm, 꺾임=%d\n",
                   seg, res.length_mm, res.turns);
        } else {
            printf("  구간 %zu ✗ 실패\n", seg);
            all_ok = false;
            break;
        }
    }

    if (all_ok && !full_path.empty()) {
        // 전체 경로 길이·꺾임 집계
        double total_len = (full_path.size() - 1) * CELL_MM;
        int total_turns = count_turns(full_path);
        printf("  ✓ 전체 경유 경로: %zu셀, 길이=%.1fmm, 꺾임=%d\n",
               full_path.size(), total_len, total_turns);
    }
}

// =============================================================================
// ■ Step 8: C ABI (r3d_* 함수) 를 통한 동일 파이프라인
// =============================================================================
// routing3d_capi.dll / libRouting3d_capi.so 를 링크할 때 사용 가능한 외부 API.
// C ABI 이므로 C#(P/Invoke), Python(ctypes), C, C++ 등 모든 언어에서 호출 가능.
// 헤더 전용 컴파일 시 아래 함수들은 실행되지 않는다(링크 실패).
// 이 섹션은 개념 예시(pseudo-code with real API).
//
// [r3d_* 함수 목록]
//   r3d_create()            : R3dEngine* 핸들 생성
//   r3d_destroy(e)          : 핸들 해제
//   r3d_set_grid(e, grid)   : 격자 원점·크기·셀 크기 설정
//   r3d_set_params(e, p)    : RouteParams 설정
//   r3d_add_obstacle(e, ...) : AABB 장애물 추가
//   r3d_add_task(e, ...)    : 라우팅 작업 추가
//   r3d_set_task_diameter(e, ti, d): 관경 설정 (per-task 반경 B1)
//   r3d_set_task_goal_dir(e, ti, ax): 목표 진입축 제약
//   r3d_set_pipe_radius(e, r) : 전역 pipe_radius 설정
//   r3d_set_per_task_radius(e, 1): per-task 반경 활성 (B1)
//   r3d_set_pipe_gap(e, mm)  : 이격 gap 설정 (P3s)
//   r3d_set_cbs_depth(e, d)  : CBS-lite 깊이 설정 (C1)
//   r3d_set_min_straight_mm(e, mm): 코너 최소직선 (P3u)
//   r3d_route_multi(e, "diameter"): 다중 배관 라우팅 실행
//   r3d_get_result(e, ti, &res): 작업 ti 결과 조회
//   r3d_copy_path(e, ti, buf, len): 경로 셀(ijk 삼중항) 복사
//   r3d_enum_octree_leaves(e, buf, max, &cnt): 옥트리 리프 열거 (가시화)
//
// [링크 옵션]
//   MSVC: /link routing3d_capi.lib
//   GCC : -lRouting3d_capi

#ifdef ROUTING3D_CAPI_LINKED   // 실제 capi.dll 을 링크할 때만 컴파일

#include "routing3d_capi.h"   // r3d_* 함수 선언

void step8_capi_example(const std::vector<RouteTask>& tasks) {
    printf("\n[Step 8] C ABI (r3d_*) 파이프라인\n");

    // ── 1. 엔진 핸들 생성 ────────────────────────────────────────────────
    R3dEngine* e = r3d_create();
    if (!e) { printf("  r3d_create 실패\n"); return; }

    // ── 2. 격자 설정 ─────────────────────────────────────────────────────
    R3dGrid grid;
    grid.cell_mm = CELL_MM;
    grid.ox = ORIGIN.x; grid.oy = ORIGIN.y; grid.oz = ORIGIN.z;
    grid.nx = NX;       grid.ny = NY;       grid.nz = NZ;
    r3d_set_grid(e, &grid);

    // ── 3. 라우팅 파라미터 설정 ──────────────────────────────────────────
    R3dParams rp{};
    rp.cell_mm   = CELL_MM;
    rp.w_turn    = 200.0;
    rp.w_clear   = 0.0;
    rp.clearance_radius       = 0;
    rp.clearance_connectivity = 6;
    r3d_set_params(e, &rp);

    // ── 4. 런타임 옵션 (탐색 상한, 계층 라우팅 파라미터) ─────────────────
    R3dRuntimeOptions opts{};
    opts.large_grid_threshold = 5'000'000LL;    // 5M 셀 이상이면 ImplicitOccupancy + hier
    opts.max_expansions       = MAX_EXP;        // 48M
    opts.fallback_expansions  = MAX_EXP;
    opts.hier_factor          = HIER_FACTOR;    // 20
    opts.hier_radius          = HIER_RADIUS;    // 2
    opts.hier_probe           = 300'000LL;
    opts.ripup_enabled        = 1;              // 인라인 rip-up 활성
    r3d_set_runtime_options(e, &opts);

    // ── 5. 장애물 추가 ───────────────────────────────────────────────────
    // 외벽
    r3d_add_obstacle(e, -500.0, 0.0, 0.0,  0.0, 30000.0, 20000.0);   // X- 벽
    r3d_add_obstacle(e, 30000.0, 0.0, 0.0, 30500.0, 30000.0, 20000.0); // X+ 벽
    // ... 나머지 장애물 동일하게 추가

    // 덕트/레터럴은 passthrough=false(장애물), 관통구멍은 r3d_add_passthrough 로 추가.
    // r3d_add_passthrough(e, x0,y0,z0, x1,y1,z1): 이 AABB를 자유셀로 강제

    // ── 6. 라우팅 작업 추가 + 관경·목표진입축 설정 ───────────────────────
    for (int ti = 0; ti < (int)tasks.size(); ++ti) {
        const RouteTask& t = tasks[ti];
        // UTF-8 문자열로 변환 (한글 이름 안전)
        std::string util  = t.utility.value_or("");
        std::string group = t.utility_group.value_or("");
        r3d_add_task(e,
            t.start_mm.x, t.start_mm.y, t.start_mm.z,
            t.end_mm.x,   t.end_mm.y,   t.end_mm.z,
            reinterpret_cast<const uint8_t*>(util.c_str()),
            reinterpret_cast<const uint8_t*>(group.c_str()));
        r3d_set_task_diameter(e, ti, t.diameter_mm);  // 관경 설정 (diameter 우선순위용)
        r3d_set_task_goal_dir(e, ti, t.goal_dir);     // 목표 진입축 (-1=무제약)
    }

    // ── 7. 배관 물리 옵션 ────────────────────────────────────────────────
    r3d_set_per_task_radius(e, 1);     // per-task 반경 (B1): 관경별 물리 이격
    r3d_set_pipe_gap(e, 60.0);         // 이격 gap 60mm (P3s): 센터선 ≥ r1+r2+60mm
    r3d_set_cbs_depth(e, 2);           // CBS-lite 깊이 2 (C1): 혼잡 협상 재배치
    r3d_set_min_straight_mm(e, 100.0); // 코너 최소직선 100mm (P3u)

    // ── 8. 다중 배관 라우팅 실행 ─────────────────────────────────────────
    // r3d_route_multi: 우선순위 "diameter"(굵은 배관 먼저) 로 route_multi_impl 호출
    // 내부: order → probe A* → escalation(route_hier) → mark_pipe → rip-up → CBS → min-straight
    int rc = r3d_route_multi(e, reinterpret_cast<const uint8_t*>("diameter"));
    printf("  r3d_route_multi 반환: %d (0=OK)\n", rc);

    // ── 9. 결과 조회 + 경로 셀 읽기 ─────────────────────────────────────
    for (int ti = 0; ti < (int)tasks.size(); ++ti) {
        R3dResult res;
        r3d_get_result(e, ti, &res);
        printf("  T%d: %s 길이=%.1fmm, 꺾임=%d, 확장=%lld\n",
               ti + 1, res.success ? "✓" : "✗",
               res.length_mm, res.turns, res.expanded_nodes);
        if (res.success && res.path_len > 0) {
            // 경로 셀을 ijk 삼중항 배열로 복사: buf[0]=i0, buf[1]=j0, buf[2]=k0, ...
            std::vector<int> path_buf(res.path_len * 3);
            r3d_copy_path(e, ti, path_buf.data(), res.path_len);
            // 첫 셀 / 마지막 셀 월드 좌표로 변환
            Cell c0{path_buf[0], path_buf[1], path_buf[2]};
            Cell cN{path_buf[(res.path_len-1)*3], path_buf[(res.path_len-1)*3+1],
                    path_buf[(res.path_len-1)*3+2]};
            Vec3 p0 = cell_to_mm(c0);
            Vec3 pN = cell_to_mm(cN);
            printf("    경로 시작: (%.0f,%.0f,%.0f) / 끝: (%.0f,%.0f,%.0f)\n",
                   p0.x, p0.y, p0.z, pN.x, pN.y, pN.z);
        }
    }

    // ── 10. 옥트리 리프 열거 (3D 가시화용) ──────────────────────────────
    // r3d_enum_octree_leaves: 씬을 가변셀 옥트리로 분해해 리프 배열 반환.
    //   BLOCKED 리프 = 장애물 셀 클러스터 (크기 정보 포함)
    //   FREE 리프    = 자유공간 클러스터
    constexpr int MAX_LEAVES = 1'000'000;
    std::vector<R3dOctreeLeaf> leaves(MAX_LEAVES);
    int leaf_count = 0;
    r3d_enum_octree_leaves(e, leaves.data(), MAX_LEAVES, &leaf_count);
    printf("  옥트리 리프: %d개 (FREE+BLOCKED 합산)\n", leaf_count);
    // 가시화: BLOCKED 리프는 붉은 박스, FREE 리프는 민트색 반투명 박스로 렌더

    // ── 11. 엔진 핸들 해제 ───────────────────────────────────────────────
    r3d_destroy(e);
    printf("  Step 8 완료\n");
}
#endif   // ROUTING3D_CAPI_LINKED

// =============================================================================
// ■ main: 전체 파이프라인 순서대로 실행
// =============================================================================
int main() {
    printf("=================================================\n");
    printf(" Routing3D 샘플 — 30m×30m×20m, 10mm 셀\n");
    printf(" 격자: %d×%d×%d = %.1f억 셀\n",
           NX, NY, NZ, (double)NX * NY * NZ / 1e8);
    printf("=================================================\n\n");

    // Step 1: SceneDoc 구성
    SceneDoc doc = build_scene_doc();

    // Step 2: 장애물 추가
    add_obstacles(doc);

    // Step 3: 라우팅 작업 정의
    std::vector<RouteTask> tasks = build_tasks();
    doc.tasks = tasks;

    // Step 4: 직접 A* 단일 탐색 (T1 배관, probe 예산 내 시도)
    step4_direct_astar(doc, tasks[0]);

    // Step 5: 계층 corridor 라우팅 (T2 배관 — 덕트 접속, 장거리)
    step5_hier_corridor(doc, tasks[1]);

    // Step 6: 다중 배관 라우팅 (전체 5개 배관, per-task 반경 + gap)
    step6_multi_route(doc, tasks);

    // Step 7: 경유점 라우팅 (수직 라이저 + 수평 랙 경유)
    step7_waypoint_routing(doc);

    // Step 8: C ABI 예시 (capi.dll 링크 시에만 실행)
#ifdef ROUTING3D_CAPI_LINKED
    step8_capi_example(tasks);
#else
    printf("\n[Step 8] C ABI 예시는 routing3d_capi.dll/lib 링크 시 활성\n");
    printf("  빌드 옵션에 -DROUTING3D_CAPI_LINKED 를 추가하고\n");
    printf("  routing3d_capi.lib / libRouting3d_capi.a 를 링크하세요.\n");
#endif

    printf("\n=================================================\n");
    printf(" 샘플 완료\n");
    printf("=================================================\n");
    return 0;
}

// Routing3D 라이브러리 C ABI 헤더 (routing3d_capi) — Phase 3 (C#/HelixToolkit 인터롭)
// =============================================================================
// [이 파일이 하는 일]
//   C++ 라우팅 엔진(occupancy/astar/cost/multi_route/scene_io)을 C ABI(extern "C")로
//   노출해 C#(P/Invoke)·파이썬(ctypes) 등 어떤 호스트든 프로세스로 호출하게 한다.
//   설계: docs/csharp_helix_interop_design.md.
//
// [ABI 안전 규칙]
//   1) C++ 예외는 경계 밖으로 나가지 않는다 — 모든 반환은 R3dStatus(또는 정수/0)로 보고.
//   2) STL/C++ 객체를 노출하지 않는다 — 불투명 핸들(R3dEngine*) + POD 구조체 + 고정 배열.
//   3) 호출 규약 cdecl. 구조체는 blittable(고정 레이아웃).
//   4) 콜러 할당 문자열은 r3d_free_string 으로 해제. 경로 배열은 콜러 할당(2단계).
//   5) 문자열은 UTF-8(경로 이름) — 호스트는 UTF-8 로 마샬링한다.
//
// [빌드]  (프로젝트 루트에서; 코어와 링크 — 외부 의존성 없는 단일 DLL)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release --target routing3d_capi
//   # 출력물: cpp/build/Release/routing3d_capi.dll (+ .lib import)
//
// [검증]
//   ctest --test-dir cpp/build -C Release -R capi --output-on-failure
// =============================================================================
#ifndef ROUTING3D_CAPI_H
#define ROUTING3D_CAPI_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(ROUTING3D_CAPI_EXPORTS)
#    define R3D_API __declspec(dllexport)
#  else
#    define R3D_API __declspec(dllimport)
#  endif
#else
#  define R3D_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

// 반환 상태 코드.
typedef enum {
    R3D_OK = 0,
    R3D_ERR_ARG = 1,      // 잘못된 인자(null 포인터·음수·0 범위)
    R3D_ERR_PARSE = 2,    // scene 텍스트 파싱 실패
    R3D_ERR_RUNTIME = 3,  // 실행 중 예외
    R3D_ERR_RANGE = 4     // 인덱스/좌표 범위 오류 (pack20 한계 초과 등)
} R3dStatus;

// 정적 버전 문자열(해제 불필요).
R3D_API const char* r3d_version(void);
// 콜러 할당 문자열 해제(r3d_route_scene_text / r3d_dump_scene_text 출력).
R3D_API void r3d_free_string(char* s);

// ---------------------------------------------------------------- Level 1: 문자열 ABI
// 입력 scene 텍스트(UTF-8) 를 라우팅하고 결과 scene 텍스트(UTF-8) 반환. out_scene_text 는 해제 필요.
//   mode     : "multi"(순차, 충돌없음) | "single"(작업별 독립).
//   priority : "longest"|"shortest"|"utility"|"original" (mode=multi 전용). 기본="longest".
R3D_API R3dStatus r3d_route_scene_text(const char* scene_text, const char* mode,
                                       const char* priority, char** out_scene_text);

// ---------------------------------------------------------------- Level 2: 핸들 ABI
typedef struct R3dEngine R3dEngine;  // 불투명 핸들.

// blittable POD (C# StructLayout.Sequential 와 1:1).
// [입력 검증 — r3d_set_grid]
//   cell_mm > 0, nx/ny/nz > 0 : R3D_ERR_ARG
//   nx/ny/nz > 1,048,575 (pack20 20비트 상한) : R3D_ERR_RANGE
//   → corridor.hpp pack20 은 축당 20비트(최대 2^20-1 = 1,048,575 셀).
//     8,000m / 50mm = 160,000 으로 실제 플랜트 규모에선 충분하나,
//     4mm 이하 초미세격자에서는 초과 위험 — 즉시 R3D_ERR_RANGE 로 차단.
typedef struct {
    double cell_mm;
    double ox, oy, oz;  // origin (mm)
    int32_t nx, ny, nz;  // shape
} R3dGrid;

// [입력 검증 — r3d_set_params]
//   cell_mm > 0 : R3D_ERR_ARG
//   w_turn / w_clear / w_corridor / w_heur / w_heur_near < 0 : R3D_ERR_ARG
//   clearance_radius < 0 : R3D_ERR_ARG
//   clearance_connectivity ∉ {0, 6, 26} : R3D_ERR_ARG  (0 은 기본값으로 6 으로 처리)
typedef struct {
    double cell_mm, w_turn, w_clear;
    double w_corridor;               // 회랑 바이어스 가중치(mm). 0=비활성(기존 동작). >0=기존설계 회랑 번들링.
    double w_heur;                   // 휴리스틱 가중치(weighted A*). 0/1=표준 A*. >1=목표 지향 탐색, 시간 비최적.
    double w_heur_near;              // 적응 가중(목표근처 수렴). (0,w_heur] 범위 수렴 가중. 0=고정 w_heur(불변).
    int32_t clearance_radius, clearance_connectivity;  // connectivity: 0(기본=6) | 6 | 26
    int32_t corridor_radius;         // 회랑 팽창 반경(셀). 기본 1.
    int32_t rack_level_count;        // rack_levels 사용 개수(0~8).
    int32_t rack_levels[8];          // 선호 랙 z-셀 인덱스, 최대 8. 해당 레벨은 회랑 면제.
} R3dParams;

typedef struct {
    int64_t large_grid_threshold;   // 0=default 5,000,000 cells.
    int64_t max_expansions;         // 0=env R3D_MAX_EXP/default.
    int64_t fallback_expansions;    // 0=max_expansions/env R3D_FALLBACK_EXP.
    int32_t hier_factor;            // 0=default 8.
    int32_t hier_radius;            // 0=default 2.
    int64_t hier_probe;             // 0=default 300,000.
    int32_t ripup_enabled;          // -1=env/default, 0=off, 1=on.
} R3dRuntimeOptions;

typedef struct {
    int32_t success;        // 1/0
    double length_mm;       // 기하 길이
    double cost_mm;         // total cost including penalties
    int32_t turns;
    int64_t expanded_nodes;
    double elapsed_ms;
    int32_t path_len;       // 경로 셀 수 — r3d_copy_path 버퍼 크기 산출용.
    int32_t visited_len;    // 방문(탐색) 셀 수 — r3d_copy_visited 버퍼 크기 산출용. 비활성이면 0.
    // 실패 이유(A1) — success=0 일 때만 유효. 구조체 끝에 추가(기존 호출 안전).
    // 0=None·1=StartBlocked·2=GoalBlocked·3=CorridorMiss·4=ExpansionLimit·5=GoalDirBlocked·6=NoPath.
    int32_t fail_reason;
} R3dResult;

R3D_API R3dEngine* r3d_create(void);
R3D_API void r3d_destroy(R3dEngine* e);
R3D_API R3dStatus r3d_load_scene_text(R3dEngine* e, const char* scene_text);

R3D_API R3dStatus r3d_set_grid(R3dEngine* e, const R3dGrid* g);
R3D_API R3dStatus r3d_set_params(R3dEngine* e, const R3dParams* p);
R3D_API R3dStatus r3d_set_runtime_options(R3dEngine* e, const R3dRuntimeOptions* opt);
R3D_API R3dStatus r3d_add_obstacle(R3dEngine* e, double minx, double miny, double minz,
                                   double maxx, double maxy, double maxz);
// 통과(pass-through) 객체 추가: 점유맵 가시화용, 경로탐색 충돌 대상이 아님.
R3D_API R3dStatus r3d_add_passthrough(R3dEngine* e, double minx, double miny, double minz,
                                      double maxx, double maxy, double maxz);
// 작업 추가 — task index(>=0) 반환, 실패 시 음수. utility/utility_group 은 비용 분류에만 사용.
R3D_API int32_t r3d_add_task(R3dEngine* e, double sx, double sy, double sz,
                             double gx, double gy, double gz,
                             const char* utility, const char* utility_group);
// 작업 종단점 갱신(인터랙티브 재집).
R3D_API R3dStatus r3d_set_task_endpoints(R3dEngine* e, int32_t task,
                                         double sx, double sy, double sz,
                                         double gx, double gy, double gz);
// 작업 관경(mm) 설정 — 우선순위 "diameter"/"utility" 정렬에서 '굵은 배관 먼저' 효과를 낸다.
// 미설정(또는 0)이면 관경 무시(기존 거리 정렬과 동일). route_multi/route_corridor_multi/route_ripup 공통.
R3D_API R3dStatus r3d_set_task_diameter(R3dEngine* e, int32_t task, double diameter_mm);
// 작업 목표 진입축 제약(axis = NEIGHBORS_6 인덱스 0..5 = +x,-x,+y,-y,+z,-z).
// A* 가 목표(end_mm)에 그 방향으로 진입해야만 도달 인정. 덕트 종단 스텁 리드인 축을 주면 배관이 일직선 진입.
// 막히면 무제약 1회 폴백(연결 우선). axis 가 [0,5] 밖 또는 미설정이면 -1(무제약), 기존 동작·골든 불변.
R3D_API R3dStatus r3d_set_task_goal_dir(R3dEngine* e, int32_t task, int32_t axis);

// 라우팅
R3D_API R3dStatus r3d_route_multi(R3dEngine* e, const char* priority);  // 전체 순차(충돌없음)
R3D_API R3dStatus r3d_route_task(R3dEngine* e, int32_t task, R3dResult* out);  // 단일(장애물 전용)

// 학습된 회랑 셀(ijk 삼중값 배열, 길이 n)을 설정한다(L2b 소프트 바이어스). w_corridor>0(set_params)이면
// route_multi 가 이 셀들을 회랑 보상으로 삼아 배관을 그 곁으로 유도(기존설계 스텁/랙 상에 따라가기).
// n<=0 이나 ijk==NULL 이면 회랑을 비운다(기존 동작). 셀 좌표는 현재 격자(set_grid) 기준 (i,j,k).
R3D_API R3dStatus r3d_set_corridor_cells(R3dEngine* e, const int32_t* ijk, int32_t n);

// 라우팅 진행 콜백(cdecl). 뷰어 진행 다이얼로그용 — 처리 순서·전체/개별 진행도·성공/실패·지표·경로를
// 실시간에 알린다. ABI 안전: 라우팅과 같은 스레드에서만 호출. 콜백 예외는 경계를 넘기지 말 것.
//   user         : 호스트 컨텍스트 포인터(그대로 달고 다님).
//   phase        : 0=탐색 진행(처리상태 %), 1=배관 완료(결과 지표 + 경로).
//   order_index  : 처리 순서(0부터, priority 정렬 기준).
//   task_index   : 원본 작업 인덱스(get_result 와 동일 매핑).
//   success      : phase==1 에서만 유효(1/0).
//   length_mm/turns/expanded_nodes/elapsed_ms : phase==1 시 결과 지표(phase==0 은 expanded 만 유효).
//   done/total   : 진행률(done = 완료 배관 수, total = 전체).
//   progress01   : phase==0 시 탐색 진행도 0~1(휴리스틱 근접 기반). phase==1 이면 1.0.
//   path_ijk/path_len : phase==1 성공 시 경로 셀((i,j,k) 연속, path_len 개). 없으면 NULL/0.
//                       포인터는 콜백 호출 안에서만 유효 — 즉시 복사할 것.
//   반환값       : 0=계속, 0 아님=취소(abort). 탐색 중(phase 0, 약 5만 확장마다)·배관 완료(phase 1)
//                  마다 검사하므로 호스트가 취소를 요청하면 현재 배관 탐색을 즉시 중단하고
//                  앞서 배관은 처리하지 않은 채로 route_multi_progress 가 R3D_OK 를 여전히 반환한다
//                  (이미 완료된 배관 결과는 보존). 협력적(cooperative) 취소.
typedef int32_t(__cdecl* R3dProgressFn)(void* user, int32_t phase, int32_t order_index,
                                        int32_t task_index, int32_t success, double length_mm,
                                        int32_t turns, int64_t expanded_nodes, double elapsed_ms,
                                        int32_t done, int32_t total, double progress01,
                                        const int32_t* path_ijk, int32_t path_len);
// r3d_route_multi 와 동일(순차·충돌없음)이되 배관마다 cb 를 호출한다. cb 가 null이면 콜백 없이 동작.
R3D_API R3dStatus r3d_route_multi_progress(R3dEngine* e, const char* priority, R3dProgressFn cb,
                                           void* user);

// rip-up & reroute(Step 3.8): 순차 베이스라인 이후 막힌 배관을 '가로막은 기존 배관'을
// 재배치해 해소한다. 무손실 채택(성공 +1) 결정적 알고리즘. 결과는 원본 작업
// 인덱스별로 보존(get_result/copy_path 매핑 유지). 0=실패 작업 없음 외에는 상태코드.
//   max_rounds : 라운드 상한.  max_ripup : 한 번에 재배치할 배관 수 상한.
R3D_API R3dStatus r3d_route_ripup(R3dEngine* e, const char* priority, int32_t max_rounds,
                                  int32_t max_ripup);

// 계층 corridor 라우팅(단일, 개별). Sparse 점유 + coarse 가이드→fine tube(astar_hashed,
// 해시 기반) — 8,000m 급 초대형 격자에서 배열 할당 없이 동작. 작업별 독립(충돌 회피 없음).
//   factor : coarse/fine 셀 비율(기본 16).  radius : corridor 팽창 반경(coarse 셀).
// 비용함수(전환/클리어런스/러닝) 미적용 — 균일 비용. 결과는 작업 인덱스별로 보존.
R3D_API R3dStatus r3d_route_corridor(R3dEngine* e, int32_t factor, int32_t radius);

// 순차 계층 corridor 라우팅(대형 격자 + 배관 간 충돌 회피). r3d_route_corridor 와 동일 엔진.
// Sparse + astar_hashed(해시 기반, 전 셀 배열 미할당)이되, priority 순서로 배관을 라우팅하고
// 깔린 경로를 mark_pipe(pipe_radius) 로 점유 추가해 다음 배관이 그것을 피하도록 한다(충돌 0).
//   factor : coarse/fine 셀 비율.  radius : corridor 팽창 반경(coarse 셀).
//   priority : "longest"|"shortest"|"utility"|"original".  pipe_radius : 깔린 배관 팽창 반경(fine 셀).
// 비용함수(전환/클리어런스/러닝) 미적용 — 균일 비용. 결과는 작업 인덱스별로 보존.
R3D_API R3dStatus r3d_route_corridor_multi(R3dEngine* e, int32_t factor, int32_t radius,
                                           const char* priority, int32_t pipe_radius);

// 결과/경로 조회.
R3D_API R3dStatus r3d_get_result(const R3dEngine* e, int32_t task, R3dResult* out);
// 경로 셀을 buf(int32_t[3*buf_cells], (i,j,k) 연속)에 복사. 반환=실제 복사된 셀 수.
R3D_API int32_t r3d_copy_path(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells);
// 방문(탐색) 셀을 buf 에 복사(가시화 '방문맵'). 반환=실제 복사된 셀 수. 비활성/미집계면 0.
R3D_API int32_t r3d_copy_visited(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells);

// 방문 셀 집계 on/off (기본 on=1). off 이면 방문맵 없이 라우팅 — visited_len=0, copy_visited=0.
// 뷰어 방문맵/단계탐색 비활성 시 메모리 절약에 유용 — 미리 0 으로 설정 후 라우팅한다.
R3D_API R3dStatus r3d_set_collect_visited(R3dEngine* e, int32_t enabled);

// 배관 점유 팽창 반경(셀) 설정 — 배관-배관 충돌 회피(옵션1). route_multi(_progress) 가 깔린 배관의
// 경로 ±radius 6-이웃까지 점유로 막아 다음 배관 중심선을 그만큼 띄운다(실제 관경보다 더 벌리면 겹침 방지).
// 0=기존 동작(경로 셀만). env R3D_PIPE_RADIUS 로도 설정 가능. 음수는 0 으로 클램프.
R3D_API R3dStatus r3d_set_pipe_radius(R3dEngine* e, int32_t radius_cells);

// per-task 관경 반경(B1) 활성화. enabled!=0 이면 route_multi(_progress) 가 각 배관의 diameter_mm 와 cell_mm
// 로 마킹 반경을 자동 산출(radius = clamp(ceil(d/cell)-1, 0, 8)). 산출된 글로벌 pipe_radius 산출 책임을
// 제거하고, 가는 배관이 굵은 배관 반경으로 과패킹되던 문제를 해소한다. 관경 미상(0)·OFF이면 글로벌 pipe_radius
// 폴백(기존 동작·골든 불변). env R3D_PER_TASK_RADIUS 로도 줄 수 있다(헤드리스 A/B).
R3D_API R3dStatus r3d_set_per_task_radius(R3dEngine* e, int32_t enabled);

// C1 negotiated-congestion(CBS-lite, Phase C) 깊이 설정. 0=OFF(평면 rip-up만·기본 동작·골든 불변). >0 이면
// 평면 rip-up 으로도 해결되지 않는 실패 배관의 blocker 가 재배치를 못 하면 그 blocker 의 blocker 까지 해당 깊이만큼
// 파고들어 양보시켜 해소한다(conflict-based search 경량판). 무손실·결정적. [0,3] 클램프. env R3D_CBS 로 가능.
R3D_API R3dStatus r3d_set_cbs_depth(R3dEngine* e, int32_t depth);

// C2 코너 최소반경 배수(Phase C) 설정. 엘보 사이 직선(단관)이 (mult × 관경) 미만이면 제작 불가(짧은 단관) —
// 경로(셀) 단계에서 충돌검사 하에 인접 두 코너를 직교 합치기로 흡수한다(꺾임 비증가일 때만, 양 끝점 고정).
// 0=OFF(기존 동작·골든 불변). 권장 2.0(엘보 간 직선 ≥ 2×관경). env R3D_MIN_STRAIGHT 로 가능.
R3D_API R3dStatus r3d_set_min_straight(R3dEngine* e, double mult);

// 코너 최소직선(절대 mm, 하드 제약). >0 이면 A* 탐색이 '한 번 꺾인 뒤 이 길이만큼 직진하기 전엔 다시
// 꺾지 못하도록' 강제한다 → 엘보 간 모든 직선 구간(단관) ≥ 이 길이. r3d_set_min_straight(관경 배수·후처리
// 흡수)와 달리 탐색 단계의 하드 보장이며 관경 무관·전 배관 적용(목표 직전 마지막 접속 구간만 면제). 셀로는
// ceil(mm/cell) 로 환산. 0=OFF(기존 동작·골든 불변). env R3D_MIN_STRAIGHT_MM 으로 재정의. 권장 100mm.
R3D_API R3dStatus r3d_set_min_straight_mm(R3dEngine* e, double mm);

// 배관-배관 이격(mm) 설정 — 두 배관 센터선 거리 ≥ r1 + r2 + gap_mm 보장(표면 이격 최소 gap_mm 이상).
// 기존 마킹은 센터선을 ~관경(d)만큼만 띄워 표면이 맞닿았다(겹쳐 보임). gap>0 이면 route_multi 메인 루프가
// 깔린 배관을 'routing 배관 기준 팽창 반경 = ceil((r_a+r_b+gap)/cell)'으로 막아 정확히 r_a+r_b+gap 이격을
// 보장한다(per-pipe 구현). 0=OFF(기존 동작·골든 불변). 규격 권장 60mm. env R3D_PIPE_GAP.
R3D_API R3dStatus r3d_set_pipe_gap(R3dEngine* e, double gap_mm);

// 점유(블록된 셀) 인덱스를 buf 에 복사(가시화 '점유맵'). 현재 doc 의 obstacles 를 즉석 voxelize.
// 반환=실제 복사된 셀 수. buf_cells 가 부족하면 처음 buf_cells 개만 복사하고 그만둔다.
// 총 블록 셀 수를 미리 알려면 buf=NULL, buf_cells=0 으로 호출(총 수 반환).
// [주의] 대형 플랜트(수억 셀)에서 전체 복사는 메모리/시간 폭발 위험.
//        UI 미리보기에는 r3d_copy_blocked_sampled 사용 권장.
R3D_API int32_t r3d_copy_blocked(const R3dEngine* e, int32_t* buf, int32_t buf_cells);

// 대형 격자 시각화용 — blocked cell 을 최대 max_cells 개 균일 샘플링해 buf 에 복사.
// 전체 개수는 r3d_copy_blocked(e, NULL, 0) 로 조회.
// 반환=실제 복사 셀 수(≤max_cells). max_cells≤0 또는 buf=NULL 이면 0 반환.
// 대형 플랜트(수억 셀)에서 r3d_copy_blocked 전체 요청 대신 이 함수로 제한된 수를 받는다.
R3D_API int32_t r3d_copy_blocked_sampled(const R3dEngine* e, int32_t max_cells, int32_t* buf);

// 통과 객체 점유 셀 인덱스를 buf 에 복사(가시화 '통과 점유맵'). r3d_copy_blocked 와 동일 규약.
R3D_API int32_t r3d_copy_passthrough(const R3dEngine* e, int32_t* buf, int32_t buf_cells);

// 현재 상태를 scene 텍스트(UTF-8)로 덤프(Python 디버거 교차검증). out_text 는 해제 필요.
R3D_API R3dStatus r3d_dump_scene_text(const R3dEngine* e, char** out_text);

// ---- 가변셀 옥트리 라우팅 ----
// 장애물 AABB 를 옥트리로 색인하고 자유공간에서 대형 셀을 한 번에 점프하는
// 적응형 A* 로 단일 배관을 탐색한다. 10mm 이하 미세격자 최단경로에 유리.
// task: r3d_add_task 인덱스. max_exp: 탐색 상한(0=무제한).
// goal_dir: 진입축(-1=무제약, 0..5=NEIGHBORS_6). out: 결과 POD.
R3D_API R3dStatus r3d_route_task_octree(R3dEngine* e, int32_t task,
                                        int64_t max_exp, int32_t goal_dir,
                                        R3dResult* out);

// ---- 옥트리 리프 열거 (3D 가시화용) ----
// 엔진에 로드된 씬 문서로 옥트리를 빌드하고 모든 리프 노드를 buf 에 채운다.
// x0_mm/y0_mm/z0_mm: 리프 원점(mm). size_mm: 리프 한 변 크기(mm). state: 0=FREE, 1=BLOCKED.
// buf: 호출자 할당 배열(maxCount). *out_count: 실제 채운 개수.
// R3D_ERR_ARG: e=null / scene 미로드 / buf=null / maxCount<=0
typedef struct {
    float x0_mm, y0_mm, z0_mm;   // 리프 원점 (world mm)
    float size_mm;                 // 리프 한 변 크기 (mm)
    int32_t state;                 // 0=FREE, 1=BLOCKED
} R3dOctreeLeaf;

R3D_API R3dStatus r3d_enum_octree_leaves(R3dEngine* e,
                                          R3dOctreeLeaf* buf, int32_t maxCount,
                                          int32_t* out_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ROUTING3D_CAPI_H

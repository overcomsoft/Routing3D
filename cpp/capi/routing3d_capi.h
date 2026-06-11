// Routing3D 네이티브 C ABI (routing3d_capi) — Phase 3 (C#/HelixToolkit 인터롭)
// =============================================================================
// [이 파일이 하는 일]
//   C++ 라우팅 엔진(occupancy/astar/cost/multi_route/scene_io)을 C ABI(extern "C")로
//   노출해, C#(P/Invoke)·파이썬(ctypes) 등 어떤 호스트든 인프로세스로 호출하게 한다.
//   설계: docs/csharp_helix_interop_design.md.
//
// [ABI 안전 규칙]
//   1) C++ 예외를 경계 밖으로 던지지 않는다 → 모든 함수는 R3dStatus(또는 음수/0)로 보고.
//   2) STL/C++ 객체를 노출하지 않는다 → 불투명 핸들(R3dEngine*) + POD 구조체 + 원시 배열.
//   3) 호출 규약 cdecl. 구조체는 blittable(고정 레이아웃).
//   4) 콜리 할당 문자열은 r3d_free_string 으로 해제. 경로 배열은 콜러 할당(2단계).
//   5) 문자열은 UTF-8(한글 이름) — 호스트는 UTF-8 로 마샬링한다.
//
// [빌드]  (프로젝트 루트에서; 코어만 링크 → 외부 의존성 없는 단일 DLL)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release --target routing3d_capi
//   # 산출물: cpp/build/Release/routing3d_capi.dll (+ .lib import)
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
    R3D_ERR_ARG = 1,      // 잘못된 인자(널 등)
    R3D_ERR_PARSE = 2,    // scene.txt 파싱 실패
    R3D_ERR_RUNTIME = 3,  // 실행 중 예외
    R3D_ERR_RANGE = 4     // 인덱스/범위 오류
} R3dStatus;

// 정적 버전 문자열(해제 불필요).
R3D_API const char* r3d_version(void);
// 콜리 할당 문자열 해제(r3d_route_scene_text / r3d_dump_scene_text 의 출력).
R3D_API void r3d_free_string(char* s);

// ---------------------------------------------------------------- Level 1: 문자열 ABI
// 입력 scene.txt(UTF-8) → 라우팅 → 결과 scene.txt(UTF-8). out_scene_text 는 해제 필요.
//   mode     : "multi"(순차, 충돌없음) | "single"(작업별 독립).
//   priority : "longest"|"shortest"|"utility"|"original" (mode=multi 일 때). 널이면 "longest".
R3D_API R3dStatus r3d_route_scene_text(const char* scene_text, const char* mode,
                                       const char* priority, char** out_scene_text);

// ---------------------------------------------------------------- Level 2: 핸들 ABI
typedef struct R3dEngine R3dEngine;  // 불투명 핸들.

// blittable POD (C# StructLayout.Sequential 과 1:1).
typedef struct {
    double cell_mm;
    double ox, oy, oz;  // origin (mm)
    int32_t nx, ny, nz;  // shape
} R3dGrid;

typedef struct {
    double cell_mm, w_turn, w_clear;
    double w_corridor;               // 회랑 밖 셀 가산 mm. 0=비활성(기존 동작). >0=기존설계 유사 번들링.
    double w_heur;                   // 휴리스틱 가중(weighted A*). 0/1=표준. >1=목표 지향(확장 급감, 약간 비최적).
    double w_heur_near;              // 동적 가중 목표근처 값. (0,w_heur) 면 수렴 가중. 0=정적 w_heur(불변).
    int32_t clearance_radius, clearance_connectivity;  // connectivity 6 또는 26
    int32_t corridor_radius;         // 회랑 성장 반경(셀). 기본 1.
    int32_t rack_level_count;        // rack_levels 사용 개수(0~8).
    int32_t rack_levels[8];          // 선호 단(z셀 인덱스), 최대 8. 이 레벨은 회랑 면제.
} R3dParams;

typedef struct {
    int32_t success;        // 1/0
    double length_mm;       // 기하 길이
    double cost_mm;         // 페널티 포함 총비용
    int32_t turns;
    int64_t expanded_nodes;
    double elapsed_ms;
    int32_t path_len;       // 경로 셀 수(r3d_copy_path 버퍼 크기 산출용)
    int32_t visited_len;    // 방문(확장) 셀 수(r3d_copy_visited 버퍼 크기 산출용). 비활성 시 0.
    // 실패 사유(A1) — success=0 일 때만 의미. 0=None·1=StartBlocked·2=GoalBlocked·3=CorridorMiss·
    //   4=ExpansionLimit·5=GoalDirBlocked·6=NoPath. 구조체 끝에 추가(기존 호출 호환).
    int32_t fail_reason;
} R3dResult;

R3D_API R3dEngine* r3d_create(void);
R3D_API void r3d_destroy(R3dEngine* e);
R3D_API R3dStatus r3d_load_scene_text(R3dEngine* e, const char* scene_text);

R3D_API R3dStatus r3d_set_grid(R3dEngine* e, const R3dGrid* g);
R3D_API R3dStatus r3d_set_params(R3dEngine* e, const R3dParams* p);
R3D_API R3dStatus r3d_add_obstacle(R3dEngine* e, double minx, double miny, double minz,
                                   double maxx, double maxy, double maxz);
// 통과(pass-through) 객체 추가: 점유맵 가시화용, 경로탐색 충돌 대상 아님.
R3D_API R3dStatus r3d_add_passthrough(R3dEngine* e, double minx, double miny, double minz,
                                      double maxx, double maxy, double maxz);
// 작업 추가 → task index(>=0) 반환, 실패 시 음수. utility/utility_group 은 색 분류용(널 허용).
R3D_API int32_t r3d_add_task(R3dEngine* e, double sx, double sy, double sz,
                             double gx, double gy, double gz,
                             const char* utility, const char* utility_group);
// 작업 종단점 갱신(인터랙티브 편집).
R3D_API R3dStatus r3d_set_task_endpoints(R3dEngine* e, int32_t task,
                                         double sx, double sy, double sz,
                                         double gx, double gy, double gz);
// 작업 관경(mm) 설정 — 우선순위 "diameter"/"utility" 정렬에서 '굵은 배관 먼저' 키로 쓰인다.
// 미설정(또는 0)이면 관경 무시(기존 거리 정렬과 동일). route_multi/route_corridor_multi/route_ripup 공통.
R3D_API R3dStatus r3d_set_task_diameter(R3dEngine* e, int32_t task, double diameter_mm);
// 작업 목표 진입축 제약(axis = NEIGHBORS_6 인덱스 0..5 = +x,-x,+y,-y,+z,-z) — A* 가 목표(end_mm)에
// 그 방향으로 진입할 때만 도달 인정. 덕트 종단 스텁 리드인 축을 주면 배관이 일직선 진입(군더더기 꺾임 제거).
// 막히면 무제약 1회 폴백(연결 우선). axis 가 [0,5] 밖/미설정이면 -1(무제약, 기존 동작·골든 불변).
R3D_API R3dStatus r3d_set_task_goal_dir(R3dEngine* e, int32_t task, int32_t axis);

// 라우팅.
R3D_API R3dStatus r3d_route_multi(R3dEngine* e, const char* priority);  // 전체 순차(충돌없음)
R3D_API R3dStatus r3d_route_task(R3dEngine* e, int32_t task, R3dResult* out);  // 단일(원본 장애물)

// 학습된 회랑 셀(ijk 삼중항 배열, 길이 n)을 설정한다(L2b 소프트 바이어스). w_corridor>0(set_params)일 때
// route_multi 가 이 셀들을 회랑 시드로 삼아 배관을 그 곁으로 유도(기존설계 스텁/랙 형상 따라가기).
// n<=0 또는 ijk==NULL 이면 회랑을 비운다(기존 동작). 셀 좌표는 현재 격자(set_grid) 기준 (i,j,k).
R3D_API R3dStatus r3d_set_corridor_cells(R3dEngine* e, const int32_t* ijk, int32_t n);

// 라우팅 진행 콜백(cdecl). 뷰어 진행 다이얼로그용 — 처리 순서·단계별 진행율·성공/실패·지표·경로를
// 실시간 표시한다. ABI 안전: 라우팅과 같은 스레드에서 동기 호출. 콜백 예외는 경계를 넘기지 말 것.
//   user         : 호스트 컨텍스트 포인터(그대로 전달).
//   phase        : 0=탐색 진행(처리상태 %), 1=배관 완료(결과 지표 + 경로).
//   order_index  : 처리 순서(0부터, priority 정렬 기준).
//   task_index   : 원본 작업 인덱스(get_result 와 동일 매핑).
//   success      : phase==1 에서만 유효(1/0).
//   length_mm/turns/expanded_nodes/elapsed_ms : phase==1 의 결과 지표(phase==0 은 expanded 만 유효).
//   done/total   : 진행률(done = 완료 배관 수, total = 전체).
//   progress01   : phase==0 의 탐색 진행율(0~1, 휴리스틱 근접 기반). phase==1 은 1.0.
//   path_ijk/path_len : phase==1 성공 시 경로 셀((i,j,k) 연속, path_len 개). 그 외 NULL/0.
//                       포인터는 콜백 호출 동안만 유효 — 즉시 복사할 것.
//   반환값       : 0=계속, 0 아님=취소(abort). 탐색 중(phase 0, 약 5만 확장마다)·배관 완료(phase 1)
//                  마다 검사하므로, 호스트가 취소를 요청하면 현재 배관 탐색을 즉시 중단하고
//                  남은 배관은 처리하지 않은 채 route_multi_progress 가 R3D_OK 로 정상 반환한다
//                  (이미 완료된 배관 결과는 보존). 협력적(cooperative) 취소.
typedef int32_t(__cdecl* R3dProgressFn)(void* user, int32_t phase, int32_t order_index,
                                        int32_t task_index, int32_t success, double length_mm,
                                        int32_t turns, int64_t expanded_nodes, double elapsed_ms,
                                        int32_t done, int32_t total, double progress01,
                                        const int32_t* path_ijk, int32_t path_len);
// r3d_route_multi 와 동일(순차·충돌없음)하되 배관마다 cb 를 호출한다. cb 가 널이면 콜백 없이 동작.
R3D_API R3dStatus r3d_route_multi_progress(R3dEngine* e, const char* priority, R3dProgressFn cb,
                                           void* user);

// rip-up & reroute(Step 3.8): 순차 베이스라인 후, 막힌 배관을 '가로막는 기존 배관'을
// 뜯어내고 재배치해 해소한다. 무손실(채택 시 성공 +1) 결정적 알고리즘. 결과는 원본 작업
// 인덱스별로 저장(get_result/copy_path 매핑 보존). 0=실패 작업 없음 의미 아님(상태코드만).
//   max_rounds : 라운드 상한.  max_ripup : 한 번에 뜯어낼 배관 수 상한.
R3D_API R3dStatus r3d_route_ripup(R3dEngine* e, const char* priority, int32_t max_rounds,
                                  int32_t max_ripup);

// 계층 corridor 라우팅(대형 장면용). Sparse 점유 + coarse 가이드→fine tube(astar_hashed,
// 해시 기반 → 8,000m 등 초대형 격자도 배열 할당 없이 동작). 작업별 독립(충돌 회피 없음).
//   factor : coarse/fine 셀 비율(예 16).  radius : corridor 팽창 반경(coarse 셀).
// 비용함수(회전/클리어런스)는 미적용(균일 비용). 결과는 작업 인덱스별로 저장.
R3D_API R3dStatus r3d_route_corridor(R3dEngine* e, int32_t factor, int32_t radius);

// 순차 계층 corridor 라우팅(대형/정밀 격자 + 배관 간 충돌 회피). r3d_route_corridor 와 동일한
// Sparse + astar_hashed(해시 기반, 셀 수 배열 미할당)이되, priority 순서로 한 배관씩 라우팅하고
// 깔린 경로를 mark_pipe(pipe_radius) 로 점유 추가해 다음 배관이 피하도록 한다(충돌 0).
//   factor : coarse/fine 셀 비율.  radius : corridor 팽창 반경(coarse 셀).
//   priority : "longest"|"shortest"|"utility"|"original".  pipe_radius : 깔린 배관 팽창 반경(fine 셀).
// 비용함수(회전/클리어런스)는 미적용(균일 비용). 결과는 작업 인덱스별로 저장.
R3D_API R3dStatus r3d_route_corridor_multi(R3dEngine* e, int32_t factor, int32_t radius,
                                           const char* priority, int32_t pipe_radius);

// 결과/경로 조회.
R3D_API R3dStatus r3d_get_result(const R3dEngine* e, int32_t task, R3dResult* out);
// 경로 셀을 buf(int32_t[3*buf_cells], (i,j,k) 연속)에 복사. 반환=실제 복사한 셀 수.
R3D_API int32_t r3d_copy_path(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells);
// 방문(확장) 셀을 buf 에 복사(가시화 '방문맵'). 반환=실제 복사한 셀 수. 비활성/미집계면 0.
R3D_API int32_t r3d_copy_visited(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells);

// 방문 셀 수집 on/off (기본 on=1). off 면 라우팅 후 visited_len=0, copy_visited=0.
// 대형 장면에서 메모리 절약이 필요할 때 미리 0 으로 설정 후 라우팅한다.
R3D_API R3dStatus r3d_set_collect_visited(R3dEngine* e, int32_t enabled);

// 배관 점유 팽창 반경(셀) 설정 — 배관-배관 충돌 회피(옵션1). route_multi(_progress) 가 깔린 배관을
// 경로 ±radius 6-이웃까지 점유로 막아 다음 배관 중심선을 그만큼 띄운다(실제 관경 렌더 시 표면 겹침 방지).
// 0=기존 동작(경로 셀만). env R3D_PIPE_RADIUS 로도 재정의 가능. 음수는 0 으로 클램프.
R3D_API R3dStatus r3d_set_pipe_radius(R3dEngine* e, int32_t radius_cells);

// per-task 관경 반경(B1) 활성화 — enabled!=0 이면 route_multi(_progress) 가 각 배관의 diameter_mm 와 cell_mm
// 로 마킹 반경을 자동 산출(radius = clamp(ceil(d/cell)-1, 0, 8)). 호출자의 글로벌 pipe_radius 산출 책임을
// 제거하고, 가는 배관이 굵은 배관 반경으로 과패킹되던 문제를 해소한다. 관경 미상(0)·OFF면 글로벌 pipe_radius
// 폴백(기존 동작·골든 불변). env R3D_PER_TASK_RADIUS 로도 켤 수 있다(헤드리스 A/B).
R3D_API R3dStatus r3d_set_per_task_radius(R3dEngine* e, int32_t enabled);

// C1 negotiated-congestion(CBS-lite, Phase C) 깊이 설정. 0=OFF(평면 rip-up만·기존 동작·골든 불변). >0 이면
// 평면 rip-up 으로도 안 풀린 실패 배관을, blocker 가 재배치 못 하면 그 blocker 의 blocker 까지 이 깊이만큼
// 재귀적으로 양보시켜 해소한다(conflict-based search 경량판, 무손실·결정적). [0,3] 클램프. env R3D_CBS 도 가능.
R3D_API R3dStatus r3d_set_cbs_depth(R3dEngine* e, int32_t depth);

// C2 코너 최소반경 배수(Phase C) 설정. 엘보 사이 직선(런)이 (mult × 관경) 미만이면 제작 불가(짧은 단관) →
// 경로(셀) 단계에서 충돌검사 하에 그 단관을 양옆 코너 직교 연결로 흡수한다(꺾임 비증가일 때만, 양 끝점 고정).
// 0=OFF(기존 동작·골든 불변). 권장 2.0(엘보 간 직선 ≥ 2×관경). env R3D_MIN_STRAIGHT 도 가능.
R3D_API R3dStatus r3d_set_min_straight(R3dEngine* e, double mult);

// 배관-배관 이격(mm) 설정 — 두 배관 센터선 거리 ≥ r1 + r2 + gap_mm 보장(표면 사이 최소 gap_mm 띄움).
// 기존 마킹은 센터선을 ~관경(d)만큼만 띄워 표면이 맞닿았다(겹쳐 보임). gap>0 이면 route_multi 메인 루프가
// 깔린 배관을 'routing 배관 기준 쌍 반경 = ceil((r_a+r_b+gap)/cell)'으로 막아 정확히 r_a+r_b+gap 띄운다
// (per-pipe 재구성). 0=OFF(기존 동작·골든 불변). 규격 권장 60mm. env R3D_PIPE_GAP.
R3D_API R3dStatus r3d_set_pipe_gap(R3dEngine* e, double gap_mm);

// 점유맵(블록된 셀) 인덱스를 buf 에 복사(가시화 '점유맵'). 현재 doc 의 obstacles 로 즉석 voxelize.
// 반환=실제 복사한 셀 수. buf_cells 가 부족하면 처음 buf_cells 개만 복사하고 그만큼 반환.
// 총 블록 셀 수를 미리 알려면 buf=NULL, buf_cells=0 으로 호출(총 셀 수 반환).
R3D_API int32_t r3d_copy_blocked(const R3dEngine* e, int32_t* buf, int32_t buf_cells);

// 통과 객체 점유 셀 인덱스를 buf 에 복사(가시화 '통과 점유맵'). r3d_copy_blocked 와 동일 규약.
R3D_API int32_t r3d_copy_passthrough(const R3dEngine* e, int32_t* buf, int32_t buf_cells);

// 현재 상태를 scene.txt(UTF-8)로 덤프(저장/디버그/교차검증). out_text 는 해제 필요.
R3D_API R3dStatus r3d_dump_scene_text(const R3dEngine* e, char** out_text);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ROUTING3D_CAPI_H

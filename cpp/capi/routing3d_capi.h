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
    int32_t clearance_radius, clearance_connectivity;  // connectivity 6 또는 26
} R3dParams;

typedef struct {
    int32_t success;        // 1/0
    double length_mm;       // 기하 길이
    double cost_mm;         // 페널티 포함 총비용
    int32_t turns;
    int64_t expanded_nodes;
    double elapsed_ms;
    int32_t path_len;       // 경로 셀 수(r3d_copy_path 버퍼 크기 산출용)
} R3dResult;

R3D_API R3dEngine* r3d_create(void);
R3D_API void r3d_destroy(R3dEngine* e);
R3D_API R3dStatus r3d_load_scene_text(R3dEngine* e, const char* scene_text);

R3D_API R3dStatus r3d_set_grid(R3dEngine* e, const R3dGrid* g);
R3D_API R3dStatus r3d_set_params(R3dEngine* e, const R3dParams* p);
R3D_API R3dStatus r3d_add_obstacle(R3dEngine* e, double minx, double miny, double minz,
                                   double maxx, double maxy, double maxz);
// 작업 추가 → task index(>=0) 반환, 실패 시 음수. utility/utility_group 은 색 분류용(널 허용).
R3D_API int32_t r3d_add_task(R3dEngine* e, double sx, double sy, double sz,
                             double gx, double gy, double gz,
                             const char* utility, const char* utility_group);
// 작업 종단점 갱신(인터랙티브 편집).
R3D_API R3dStatus r3d_set_task_endpoints(R3dEngine* e, int32_t task,
                                         double sx, double sy, double sz,
                                         double gx, double gy, double gz);

// 라우팅.
R3D_API R3dStatus r3d_route_multi(R3dEngine* e, const char* priority);  // 전체 순차(충돌없음)
R3D_API R3dStatus r3d_route_task(R3dEngine* e, int32_t task, R3dResult* out);  // 단일(원본 장애물)

// 결과/경로 조회.
R3D_API R3dStatus r3d_get_result(const R3dEngine* e, int32_t task, R3dResult* out);
// 경로 셀을 buf(int32_t[3*buf_cells], (i,j,k) 연속)에 복사. 반환=실제 복사한 셀 수.
R3D_API int32_t r3d_copy_path(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells);

// 현재 상태를 scene.txt(UTF-8)로 덤프(저장/디버그/교차검증). out_text 는 해제 필요.
R3D_API R3dStatus r3d_dump_scene_text(const R3dEngine* e, char** out_text);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ROUTING3D_CAPI_H

# C# HelixToolkit WPF 가시화 — 네이티브 DLL 인터롭 설계서

> Routing3D Phase 3 보조 설계 · 버전 0.1 · 2026-05-29 · 단위 **mm**
> 대상: C++ 엔진(`cpp/`)을 네이티브 DLL 로 노출 → C#/.NET(WPF + HelixToolkit)에서 인프로세스 호출·가시화.
> 본 문서는 **설계**다. 구현은 단계적 로드맵(§8)을 따른다. 알고리즘/포맷 계약은 기존 명세
> ([algorithm_spec.md](spec/algorithm_spec.md), [scene_format_spec.md](spec/scene_format_spec.md))를 그대로 따른다.

---

## 0. 목표 · 비목표

**목표**

- C++ 라우팅 엔진을 **공유 라이브러리(`routing3d_capi.dll`)** 로 빌드해 C# 에서 직접 호출.
- C#/WPF + **HelixToolkit** 으로 장애물·경로를 3D 렌더(기존 Python `viz` 와 동일한 그림: 장애물 반투명 박스 + 유틸리티별 색 경로 튜브).
- **인터랙티브 재라우팅**: 종단점 편집 → 즉시 재탐색 → 화면 갱신(파일 왕복 없이 인프로세스).

**비목표(현 단계)**

- 기존 SpaceAI C# 소스 직접 포팅(읽기 권한 차단 — 개념만 참조).
- 별도 프로세스/네트워크 서비스화(필요 시 후속).
- 엔진 알고리즘 변경. 본 설계는 **순수 부가(additive)** — 기존 코어/CLI/바인딩을 건드리지 않는다.

---

## 1. 전체 아키텍처

```
  ┌──────────────────────────── C# / .NET (WPF, x64) ────────────────────────────┐
  │  HelixViewport3D 뷰어 (MVVM)                                                   │
  │   - 장애물 → BoxVisual3D / 머지 MeshGeometry3D (반투명)                        │
  │   - 경로   → TubeVisual3D (유틸리티별 색)   - start/end 구(球) 마커            │
  │        ▲ 렌더 모델                                                             │
  │        │                                                                       │
  │   SceneViewModel ── Engine(매니지드 래퍼) ── Native(P/Invoke) ── SafeHandle    │
  └──────────────────────────────────┬────────────────────────────────────────────┘
                                      │ C ABI (extern "C", cdecl)  ← 인터롭 경계
  ┌───────────────────────────────────▼─────────────────────────────────────────┐
  │  routing3d_capi.dll  (얇은 C 래퍼)                                            │
  │   r3d_create / load / set_grid / add_obstacle / add_task / route / get_path   │
  │        │ 호출                                                                  │
  │   routing3d_core (정적): occupancy / astar / cost / multi_route / scene_io     │
  │   (옵션: routing3d_vdb, routing3d_fcl)                                         │
  └───────────────────────────────────────────────────────────────────────────────┘

  공통 디버그/교환 계약: scene.txt (v1) — 파일 왕복은 항상 유효한 폴백·검증 경로로 유지.
```

핵심: **인터롭 경계는 C ABI 한 층**. 그 위(C#)·아래(C++ 코어)는 서로를 모른다. scene.txt 는 폴백 겸 디버그 채널로 계속 살린다.

---

## 2. 인터롭 경계 선택

| 방식 | 장점 | 단점 | 판정 |
|---|---|---|---|
| **C ABI(extern "C") + P/Invoke** | 가장 단순·이식성↑, .NET Core/5+/Framework 모두 동작, ABI 안정 | 수동 마샬링, OOP 손맛 적음 | **채택** |
| C++/CLI 래퍼(`ref class`) | C++ 객체를 .NET 객체처럼 | Windows 전용, MSVC 종속, /clr 빌드·혼합 어셈블리 복잡 | 보류 |

**결정: C ABI + P/Invoke**. 이유 — 엔진이 헤더 전용 템플릿이라 C++/CLI 로 감싸도 결국 구체 타입(DenseOccupancy)으로 고정해야 하고, C ABI 가 더 얇고 깨질 일이 적다. 향후 다른 호스트(파이썬 ctypes, 네이티브 앱)도 같은 ABI 재사용 가능.

**불변 규칙(ABI 안전)**

1. **C++ 예외를 경계 밖으로 던지지 않는다.** 모든 export 함수는 `try/catch(...)` 로 감싸 **상태 코드**를 반환.
2. 경계에서 STL 타입(`std::string`, `std::vector`)·C++ 객체를 노출하지 않는다 → **불투명 핸들 + POD 구조체 + 원시 배열**만.
3. 호출 규약 `cdecl` 고정. 구조체는 **blittable**(고정 레이아웃)만.
4. 메모리 소유권 규칙을 함수마다 명시(§3.3).

---

## 3. C ABI 설계 (`cpp/capi/routing3d_capi.h`)

두 레벨을 제공한다. **Level 1(문자열)** 은 즉시 뷰어를 띄우는 최소 표면, **Level 2(핸들)** 는 인터랙티브 재라우팅용.

```c
/* routing3d_capi.h — Routing3D 네이티브 C ABI (extern "C") */
#pragma once
#include <stddef.h>
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

typedef enum {
    R3D_OK = 0, R3D_ERR_ARG = 1, R3D_ERR_PARSE = 2,
    R3D_ERR_RUNTIME = 3, R3D_ERR_RANGE = 4
} R3dStatus;

R3D_API const char* r3d_version(void);          /* 정적 문자열(해제 불필요) */
R3D_API void        r3d_free_string(char* s);   /* 콜리 할당 문자열 해제 */
```

### 3.1 Level 1 — 문자열 ABI (scene.txt in/out)

기존 `scene_io` + `route_sequential`/`astar_weighted` 를 그대로 재사용. C# 은 **문자열만** 마샬링하면 된다.

```c
/* 입력 scene.txt(UTF-8) → 라우팅 → 결과 scene.txt(UTF-8).
   out_scene_text 는 r3d_free_string 으로 해제해야 한다. */
R3D_API R3dStatus r3d_route_scene_text(
    const char* scene_text,   /* 입력 scene.txt (UTF-8) */
    const char* mode,         /* "multi" | "single" */
    const char* priority,     /* "longest"|"shortest"|"utility"|"original" (multi) */
    char**      out_scene_text);
```

- 구현 = 현 CLI `cmd_route` 의 인메모리 버전: `loads_scene → occupancy_from_doc → route_sequential/astar_weighted → dumps_scene`.
- **장점**: 표면이 함수 1개. 엔진·scene_io 무수정 재사용. C# 뷰어는 어차피 scene.txt 파서가 필요하므로 추가 비용 0.
- **용도**: 배치 라우팅, 첫 화면, 회귀 검증.

### 3.2 Level 2 — 핸들 ABI (인터랙티브)

장면을 메모리에 유지하고 작업만 바꿔 재탐색(대형 scene 재파싱 회피).

```c
typedef struct R3dEngine R3dEngine;   /* 불투명 핸들 */

R3D_API R3dEngine* r3d_create(void);
R3D_API void       r3d_destroy(R3dEngine*);
R3D_API R3dStatus  r3d_load_scene_text(R3dEngine*, const char* scene_text); /* 선택: 한 번에 적재 */

/* --- blittable POD (C# StructLayout.Sequential 과 1:1) --- */
typedef struct { double cell_mm; double ox, oy, oz; int32_t nx, ny, nz; } R3dGrid;
typedef struct { double cell_mm, w_turn, w_clear;
                 int32_t clearance_radius, clearance_connectivity; } R3dParams;
typedef struct { int32_t success; double length_mm, cost_mm;
                 int32_t turns; int64_t expanded_nodes; double elapsed_ms;
                 int32_t path_len; } R3dResult;   /* path_len = 경로 셀 수 */

R3D_API R3dStatus r3d_set_grid  (R3dEngine*, const R3dGrid*);
R3D_API R3dStatus r3d_set_params(R3dEngine*, const R3dParams*);

/* 장애물 AABB 추가(이름 등 메타는 가시화에 불필요 → 생략, 필요 시 후속 확장). */
R3D_API R3dStatus r3d_add_obstacle(R3dEngine*,
    double minx, double miny, double minz, double maxx, double maxy, double maxz);

/* 작업 추가 → task index(>=0) 반환, 실패 시 음수(-R3dStatus). utility 는 색 분류용. */
R3D_API int32_t r3d_add_task(R3dEngine*,
    double sx, double sy, double sz, double gx, double gy, double gz,
    const char* utility, const char* utility_group);

/* 작업 종단점만 갱신(인터랙티브 편집). */
R3D_API R3dStatus r3d_set_task_endpoints(R3dEngine*, int32_t task,
    double sx, double sy, double sz, double gx, double gy, double gz);

/* 라우팅 */
R3D_API R3dStatus r3d_route_multi(R3dEngine*, const char* priority);     /* 전체 순차 */
R3D_API R3dStatus r3d_route_task (R3dEngine*, int32_t task, R3dResult*); /* 단일(원본 장애물) */

/* 결과/경로 조회 — 2단계 패턴: 먼저 R3dResult.path_len 확인 후 버퍼 제공 */
R3D_API R3dStatus r3d_get_result(const R3dEngine*, int32_t task, R3dResult*);
/* 경로 셀을 buf(int32_t[3*buf_cells], (i,j,k) 연속)로 복사. 반환=실제 복사한 셀 수. */
R3D_API int32_t   r3d_copy_path(const R3dEngine*, int32_t task,
                                int32_t* buf, int32_t buf_cells);

/* 현재 상태를 scene.txt 로 덤프(저장/디버그). out_text 는 r3d_free_string 해제. */
R3D_API R3dStatus r3d_dump_scene_text(const R3dEngine*, char** out_text);

#ifdef __cplusplus
}
#endif
```

내부 구현 메모: `R3dEngine` 는 `SceneDoc`(또는 grid/params/obstacles/tasks/결과 벡터) + 캐시한 `DenseOccupancy` 를 보유. `r3d_route_multi` = `route_sequential`; 결과를 task index 별로 저장(순서 보존). 좌표↔셀은 코어의 `to_world/to_cell` 사용.

### 3.3 메모리 소유권 · 마샬링 규칙

| 데이터 | 방향 | 규칙 |
|---|---|---|
| 입력 문자열(`const char*`) | C#→C++ | 호출 동안만 유효(콜러 소유). C++ 는 즉시 복사. |
| 출력 문자열(`char**`) | C++→C# | **콜리 할당**, C# 이 `r3d_free_string` 호출. |
| POD 구조체(`R3dGrid` 등) | 양방향 | blittable, 콜러 할당. |
| 경로 배열(`int32_t* buf`) | C++→C# | **콜러 할당**(2단계: `path_len` 먼저). C++ 는 채우기만. |
| 핸들(`R3dEngine*`) | — | `r3d_create`~`r3d_destroy` 까지 C++ 소유. |

- **인코딩**: scene.txt·이름은 **UTF-8**(한글 장애물/유틸명). P/Invoke 기본 ANSI 마샬링은 한글을 깨뜨리므로 **반드시 UTF-8 수동 마샬링**(§4).
- **스레드 안전**: 한 `R3dEngine` 핸들은 동시 호출 금지(핸들당 1스레드 또는 외부 락). 서로 다른 핸들은 독립.
- **에러**: 모든 함수 `try/catch(...)` → `R3dStatus`. 마지막 에러 메시지는 선택적으로 `r3d_last_error()`(후속) 로 노출.

---

## 4.
계층

```csharp
// Native.cs — 원시 P/Invoke (UTF-8 마샬링 주의)
using System;
using System.Runtime.InteropServices;

internal static class Native
{
    const string Dll = "routing3d_capi";   // routing3d_capi.dll (앱 출력 폴더)
    const CallingConvention Cdecl = CallingConvention.Cdecl;

    [StructLayout(LayoutKind.Sequential)]
    public struct R3dGrid { public double cell_mm, ox, oy, oz; public int nx, ny, nz; }
    [StructLayout(LayoutKind.Sequential)]
    public struct R3dParams { public double cell_mm, w_turn, w_clear;
                              public int clearance_radius, clearance_connectivity; }
    [StructLayout(LayoutKind.Sequential)]
    public struct R3dResult { public int success; public double length_mm, cost_mm;
                              public int turns; public long expanded_nodes;
                              public double elapsed_ms; public int path_len; }

    [DllImport(Dll, CallingConvention = Cdecl)] public static extern IntPtr r3d_create();
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern void   r3d_destroy(IntPtr e);
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern int    r3d_set_grid(IntPtr e, in R3dGrid g);
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern int    r3d_set_params(IntPtr e, in R3dParams p);
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern int    r3d_add_obstacle(
        IntPtr e, double minx, double miny, double minz, double maxx, double maxy, double maxz);
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern int    r3d_route_multi(IntPtr e, IntPtr priorityUtf8);
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern int    r3d_get_result(IntPtr e, int task, out R3dResult r);
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern int    r3d_copy_path(
        IntPtr e, int task, [Out] int[] buf, int bufCells);
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern void   r3d_free_string(IntPtr s);

    // add_task: utility 문자열은 UTF-8 바이트로 직접 전달(한글 안전).
    [DllImport(Dll, CallingConvention = Cdecl)] public static extern int    r3d_add_task(
        IntPtr e, double sx, double sy, double sz, double gx, double gy, double gz,
        byte[] utilityUtf8, byte[] utilityGroupUtf8);

    public static byte[] Utf8(string? s) =>
        System.Text.Encoding.UTF8.GetBytes((s ?? "") + "\0");   // null 종료
}
```

```csharp
// R3dEngineHandle.cs — SafeHandle 로 누수·이중해제 방지
using System;
using Microsoft.Win32.SafeHandles;

internal sealed class R3dEngineHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    public R3dEngineHandle() : base(true) { }
    public static R3dEngineHandle Create()
    {
        var h = new R3dEngineHandle();
        h.SetHandle(Native.r3d_create());
        return h;
    }
    protected override bool ReleaseHandle() { Native.r3d_destroy(handle); return true; }
}
```

```csharp
// Engine.cs — 매니지드 OOP 래퍼(앱이 실제로 쓰는 면)
public sealed class Engine : IDisposable
{
    readonly R3dEngineHandle _h = R3dEngineHandle.Create();

    public void SetGrid(double cell, (double x,double y,double z) origin, (int x,int y,int z) shape)
        => Check(Native.r3d_set_grid(_h.DangerousGetHandle(),
            new Native.R3dGrid { cell_mm=cell, ox=origin.x, oy=origin.y, oz=origin.z,
                                 nx=shape.x, ny=shape.y, nz=shape.z }));

    public int AddTask(Vec3 s, Vec3 g, string? utility, string? group)
        => Native.r3d_add_task(_h.DangerousGetHandle(), s.X,s.Y,s.Z, g.X,g.Y,g.Z,
                               Native.Utf8(utility), Native.Utf8(group));

    public void RouteMulti(string priority = "longest")
    {
        var p = Native.Utf8(priority);
        var pin = GCHandle.Alloc(p, GCHandleType.Pinned);
        try { Check(Native.r3d_route_multi(_h.DangerousGetHandle(), pin.AddrOfPinnedObject())); }
        finally { pin.Free(); }
    }

    public (Native.R3dResult res, (int i,int j,int k)[] path) GetRoute(int task)
    {
        Check(Native.r3d_get_result(_h.DangerousGetHandle(), task, out var r));
        var cells = new (int,int,int)[r.path_len];
        if (r.path_len > 0) {
            var buf = new int[r.path_len * 3];
            Native.r3d_copy_path(_h.DangerousGetHandle(), task, buf, r.path_len);
            for (int n = 0; n < r.path_len; n++) cells[n] = (buf[3*n], buf[3*n+1], buf[3*n+2]);
        }
        return (r, cells);
    }

    static void Check(int status) { if (status != 0) throw new InvalidOperationException($"r3d status {status}"); }
    public void Dispose() => _h.Dispose();
}
```

읽어온 출력 문자열은 `Marshal.PtrToStringUTF8(ptr)` 로 디코드 후 `Native.r3d_free_string(ptr)` 로 해제(Level 1 / dump 경로).

---

## 5. HelixToolkit WPF 뷰어

- **프로젝트**: WPF(.NET 8, `<PlatformTarget>x64</PlatformTarget>`) + NuGet `HelixToolkit.Wpf`.
- **패턴**: MVVM. `SceneViewModel` 이 모델 컬렉션 보유, `MainWindow` 의 `HelixViewport3D` 에 바인딩.

```
SceneViewModel
  ├─ Engine                       (네이티브 래퍼)
  ├─ ObservableCollection<Visual3D> Obstacles   ← 장애물 박스
  ├─ ObservableCollection<Visual3D> Paths       ← 유틸리티별 경로 튜브
  ├─ UtilityColor(label) → Color  (결정적 팔레트, Python utility_colors 와 동일 규약)
  └─ Commands: LoadScene / RouteMulti / RerouteSelected
```

**셀→월드**: `world = origin + (cell + 0.5) * cell_mm` (명세 §1과 동일). 경로 셀 배열을 `Point3D[]` 로 변환.

**렌더 매핑(Python viz ↔ HelixToolkit)**

| 요소 | Python(PyVista) | C#(HelixToolkit) |
|---|---|---|
| 장애물 | `occupancy_to_voxels` 복셀 머지, opacity 0.12 | `BoxVisual3D` 또는 `MeshBuilder.AddBox` 머지 1메시, 반투명 `DiffuseMaterial`(알파) |
| 경로 | `lines_from_points().tube(r)` | `TubeVisual3D{ Path=Point3D[], Diameter=cell*0.7 }` |
| 시작/끝 | `pv.Sphere` lime/red | `SphereVisual3D` |
| 유틸 색 | `utility_colors` 팔레트 | 동일 팔레트를 `Color[]` 로 |
| 축/범례 | `show_grid`/`add_legend` | `GridLinesVisual3D` + 커스텀 범례 패널 |

**성능**: 장애물이 많으면 박스별 Visual3D 는 느리다 → `MeshBuilder` 로 **모든 박스를 한 메시로 머지**(Python 의 복셀 머지에 해당). 경로도 유틸리티별로 한 메시에 합치면 draw call 감소.

**인터랙션(재라우팅)**: 종단 마커를 드래그/입력 → `Engine.SetTaskEndpoints` → `Engine.RouteTask` → 해당 튜브만 갱신. 무거운 호출은 §7 처럼 백그라운드.

---

## 6. 빌드 · 패키징

**CMake** — 코어를 공유 DLL 로 감싸는 타깃 추가(기존 타깃 무수정):

```cmake
# cpp/CMakeLists.txt 에 추가(안)
add_library(routing3d_capi SHARED capi/routing3d_capi.cpp)
target_link_libraries(routing3d_capi PRIVATE routing3d_core)   # 코어만 → 단일 DLL
target_compile_definitions(routing3d_capi PRIVATE ROUTING3D_CAPI_EXPORTS)
target_include_directories(routing3d_capi PUBLIC include capi)
# (옵션) USE_OPENVDB/USE_FCL 시 routing3d_vdb/fcl 링크 + 해당 런타임 DLL 동봉 필요
```

```powershell
cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
cmake --build cpp/build --config Release --target routing3d_capi
# 산출물: cpp/build/Release/routing3d_capi.dll
```

**비트/런타임**

- C# 앱은 **x64** 로 빌드(DLL 과 일치). AnyCPU + Prefer32bit 금지.
- 코어 전용 DLL 은 **외부 의존성 없음**(MSVC 런타임만) → 배포 간단.
- OpenVDB/FCL 활성화 시: `openvdb.dll`, `tbb*.dll`, `fcl.dll` 등(vcpkg applocal)을 **C# 앱 출력 폴더에 `routing3d_capi.dll` 과 함께** 배치.
- C# `.csproj` 에서 post-build 로 DLL 복사:

```xml
<ItemGroup>
  <None Include="..\..\cpp\build\Release\routing3d_capi.dll">
    <CopyToOutputDirectory>PreserveNewest</CopyToOutputDirectory>
  </None>
</ItemGroup>
```

---

## 7. 스레딩

- UI 스레드에서 라우팅 호출 금지(수백 배관·대형 장면은 수 초). `Task.Run(() => engine.RouteMulti())` 후 결과를 `Dispatcher` 로 모델에 반영.
- `R3dEngine` 핸들은 스레드 안전이 아니므로 **호출을 직렬화**(앱 측 단일 워커 큐 권장). 동시 탐색이 필요하면 핸들을 복수로.

---

## 8. 단계적 도입 로드맵

| 단계 | 내용 | 산출물 | 상태 |
|---|---|---|---|
| **P0** | `routing3d_capi`(코어 전용) + **Level 1·2 ABI**. ctest `capi` 로 골든 03(5/5·28050mm) DLL 재현 + 문자열 왕복. | DLL + 스모크 테스트 | **완료 2026-05-29** |
| **P1** | C# WPF + HelixToolkit 뷰어(`csharp/Routing3D.Viewer`). 내장 데모/scene.txt 로드 → 엔진(P/Invoke)으로 `route_multi` → 장애물 반투명 박스 + 유틸별 경로 튜브 렌더. | 뷰어 앱 | **완료 2026-05-29** |
| **P2** | **인터랙티브 재라우팅**: 작업 목록(ListBox)에서 배관 선택 → 종단점(시작/끝 XYZ) 편집 → 단일 재라우팅(`set_task_endpoints`+`route_task`) 또는 전체(`route_multi`) → 모델 갱신. | 인터랙티브 뷰어 | **완료 2026-05-29** |
| **P3a (뷰어)** | 충돌 셀 시각화(빨간 큐브), 표시 토글(장애물/경로/충돌), 3D 클릭 종단점 지정(`FindNearestPoint`→셀 스냅), 장애물/경로 메시 머지. | 뷰어 기능 | **완료 2026-05-29** |
| **P3b (엔진)** | 대형 장면 **계층 corridor 라우팅을 C ABI 로 노출**(`r3d_route_corridor`). **SparseOccupancy + astar_hashed** 라 배열 할당 없이 초대형 격자 동작 → **OpenVDB DLL 동봉 불필요**(DLL 코어 전용 유지). C# 뷰어 corridor 버튼. ctest `capi` 가 2000²×8 Sparse 장면 라우팅 검증. | 대형장면 라우팅 | **완료 2026-05-29** |
| P3b' (선택) | VDB 백엔드 capi(`USE_OPENVDB` 빌드 + openvdb/tbb DLL 동봉) — Sparse 로 목표 충족되어 보류. | (선택) | 미착수 |

각 단계는 독립 가치가 있고, **P0/P1 만으로도 "C++ 엔진 + C# 뷰어"가 성립**한다(scene.txt 폴백과 동치).

---

## 9. 리스크 · 미해결

- **UTF-8 마샬링**: 한글 이름. 모든 문자열을 UTF-8 바이트로 직접 전달/`PtrToStringUTF8` 로 수신(§4). ANSI 기본값 사용 금지.
- **비트 불일치**: x64 통일(런타임 `BadImageFormatException` 방지).
- **런타임 DLL 의존**: OpenVDB/FCL 켜면 동반 DLL 동봉 필수. 코어 전용으로 시작 권장.
- **예외 누수**: C 래퍼에서 `catch(...)` 누락 시 프로세스 크래시 → 모든 export 강제 가드.
- **대형 메시 성능**: HelixToolkit 은 Visual3D 수가 많으면 느림 → 박스/튜브 머지로 대응.
- **결과 정합 검증**: P0 에서 DLL 결과를 기존 CLI/Python 골든과 교차검증(동일 지표)으로 회귀 방지.

---

## 10. 관련 빌드/실행 명령(현행)

```powershell
# (참고) 현재 동작하는 폴백 경로: C++ CLI 로 라우팅 → Python 뷰어로 가시화
.\run.ps1 route --in scene.txt --out out/routed.scene.txt --mode multi
.\.venv\Scripts\python.exe -m routing3d_py.viz_scene --in out/routed.scene.txt --screenshot out/routed.png

# C# 뷰어(P1, 구현 완료): DLL 빌드 → 뷰어 빌드/실행
cmake --build cpp/build --config Release --target routing3d_capi   # routing3d_capi.dll 생성
dotnet build csharp/Routing3D.Viewer.sln -c Release                # DLL 자동 복사
dotnet run --project csharp/Routing3D.Viewer -c Release            # 창: 내장 데모 자동 표시
```

> 뷰어 구성: `csharp/Routing3D.Viewer/` — `Interop`(Native/SafeHandle/Engine) · `Model`(SceneData/SceneTextParser/UtilityColors) · `ViewModels`(SceneViewModel) · `MainWindow`. HelixToolkit.Wpf **2.24.0**(WPF 네이티브 MeshBuilder; 3.x 는 geometry 가 System.Numerics 로 분리). x64 고정, `routing3d_capi.dll` 은 빌드 출력으로 복사. 시작 시 내장 데모(골든03)를 엔진으로 라우팅해 표시한다.

# Routing3D C# 뷰어 — 3가지 경로탐색 방식 단계별 함수 호출 가이드

> 대상 파일: `csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs` · `Interop/Engine.cs` · `Interop/Native.cs` · `Model/StubExtractor.cs`
> 엔진: `routing3d_capi.dll` (C ABI, x64, UTF-8)
> 작성: 2026-07-02

---

## 1. 3가지 라우팅 모드 개요

```csharp
// SceneViewModel.cs
enum RoutingMode {
    Shortest,        // 순수 최단 A* — PoC→PoC 전구간
    PatternApplied,  // 스텁+번들+회랑 — 기본값
    FollowExisting,  // 기존설계 복제 + 국소 A* 수리
}
```

| 구분 | Shortest | PatternApplied | FollowExisting |
|------|----------|----------------|----------------|
| `_usePatterns` | false | **true** | true |
| `_useStubRouting` | false | **true** | true |
| `_useDesignReplicate` | false | false | **true** |
| `wCorr` (회랑 가중치) | 0.0 | **cell×0.5** | 0.0 |
| A* 범위 | PoC→PoC 전구간 | **스텁끝→스텁끝** | 스텁끝→스텁끝 |
| 복제 후처리 | 없음 | 없음 | **ReplicateMatchedPipes** |

---

## 2. 최상위 진입점

```
[툴바 자동설계 클릭]
  RunRouteAsync()                     SceneViewModel.cs
    └─ useGroupCorridor = (_routeMode == PatternApplied)
    └─ RouteRowsAsync(rows, corridor, showProgress)
```

```csharp
// RunRouteAsync 줄 1459–1497
private async Task RunRouteAsync()
{
    // 1. 범위(All / ByGroup / ByUtility)에 따라 행 인덱스 목록 결정
    var rows = GetCurrentRowPositions();

    // 2. PatternApplied만 그룹 회랑 ON
    bool useGroupCorridor = (_routingMode == RoutingMode.PatternApplied);

    // 3. 실제 라우팅
    await RouteRowsAsync(rows, label: "자동설계", corridor: useGroupCorridor, showProgress: true);
}
```

---

## 3. 공통 실행 뼈대: `RouteRowsAsync()`

```
RouteRowsAsync(rowPositions, label, corridor, showProgress)
  ├─ [PatternApplied, cellMm > 50, 다건] CellMm = 50, LoadFromDbAsync()
  ├─ ClearRouteResults(rowPositions)       → 이전 결과 제거
  ├─ BuildModel()                          → 빈 상태 3D 즉시 반영
  │
  ├─ ★ BuildEngineForRows(rowPositions, groupMode: corridor)
  │      → 엔진 초기화 + 장애물 + 작업 전부 설정
  │
  ├─ await Task.Run(() =>
  │      engine.RouteMultiProgress(priority, progressCallback)
  │      // 또는 RouteCorridorMulti(factor) — hier 대형 격자
  │  )
  │
  ├─ CacheResults(added)                   → r3d_get_result / r3d_copy_path
  │
  ├─ [FollowExisting만]
  │    replicated = await Task.Run(() => ReplicateMatchedPipes(added))
  │
  └─ BuildModel()                          → 최종 3D 렌더
```

---

## 4. 엔진 초기화: `BuildEngineForRows()`

### 4-1. 엔진 생성 및 격자 설정

```csharp
// 줄 1526–1528
_engine?.Dispose();
_engine = new Engine();                   // r3d_create()
_engine.SetGrid(cell, ox, oy, oz, nx, ny, nz);  // r3d_set_grid(H, grid)
```

### 4-2. A* 파라미터 계산

```csharp
// 줄 1539–1604
long totalCells = (long)g.Nx * g.Ny * g.Nz;
bool weighted   = totalCells > 300_000;       // 소형=정밀, 대형=가중 그리디

double wHeur = weighted ? 2.0 : 1.0;          // Shortest는 1.4 고정
double wHeurNear = weighted ? 1.0 : 0.0;      // 목표 근처 수렴 가중
double wCorr = corridor ? cell * 0.5 : 0.0;  // PatternApplied만 회랑 바이어스

int[] rackLevels = _useRackBundling
    ? BuildRackLevels(rowPositions)            // 기존설계에서 학습한 z-단 셀 인덱스
    : null;
```

### 4-3. SetParams 호출

```csharp
// 줄 1604
_engine.SetParams(
    cellMm:           cell,
    wTurn:            500.0,   // 꺾임 페널티
    wClearance:       10.0,    // 클리어런스 페널티
    clearanceRadius:  2,
    clearanceConnectivity: 6,
    wCorridor:        wCorr,
    corridorRadius:   2,
    rackLevels:       rackLevels,
    wHeur:            wHeur,
    wHeurNear:        wHeurNear
);
// → r3d_set_params(H, in RouteParamsC)
```

### 4-4. 알고리즘 옵션

```csharp
// 줄 1606–1642
_engine.SetSegmentAstar(useSegmentAstar, segmentMaxCells);  // r3d_set_segment_astar
_engine.SetOctreeGuide(useOctreeGuide, octreeCorrRadius);   // r3d_set_octree_guide
_engine.SetPerTaskRadius(true);                              // r3d_set_per_task_radius
_engine.SetPipeGap(60.0);                                    // r3d_set_pipe_gap (배관 이격 60mm)
_engine.SetMinStraight(2.0);                                 // r3d_set_min_straight (관경배수)
_engine.SetMinStraightMm(MinStraightMmForRouting());         // r3d_set_min_straight_mm (100mm 기본)
_engine.SetCbsDepth(_useCbs ? 2 : 0);                       // r3d_set_cbs_depth

// Shortest + 대형 격자만
if (_routingMode == Shortest && totalCells > 5_000_000)
    _engine.SetMaxExpansions(8_000_000L);                    // r3d_set_runtime_options
```

### 4-5. 장애물 등록

```csharp
// 줄 1643–1650
foreach (var o in _scene.Obstacles)
{
    if (o.IsPassThrough)
        _engine.AddPassthrough(o.X1, o.Y1, o.Z1, o.X2, o.Y2, o.Z2);
    else
        _engine.AddObstacle(o.X1, o.Y1, o.Z1, o.X2, o.Y2, o.Z2);
}
// AddFacilityObstacles: 설비 + 덕트/레터럴 + 완료배관 경로 AABB + 스텁 폴리라인
AddFacilityObstacles(_engine, currentRows);
// → 전부 r3d_add_obstacle(H, x1,y1,z1, x2,y2,z2)
```

---

## 5. 작업(Task) 등록 — 모드별 분기

### 공통 선행: 매칭 기존배관 탐색

```csharp
// 줄 1659
ExistingPipe? stubPipe = _useStubRouting
    ? FindMatchingExistingPipe(row)   // 씬 기존배관 중 양끝 PoC 거리 최소인 배관
    : null;
```

---

### 경로 A: 스텁 라우팅 (PatternApplied / FollowExisting)

매칭 기존배관이 있으면 스텁을 추출해 A* 범위를 줄인다.

```
StubExtractor.ForPipe(pipe)
  ① DirRuns(segs)        : 세그먼트 → 직교 (축, 거리mm) 런 목록
  ② MergeShort(runs)     : 250mm 미만 런 인접에 흡수 (지터 제거)
  ③ WalkStub(ordered)    : 수직런 + 첫 엘보(수직→수평 전환) + 800mm 리드인
                            → Point3D[] = 스텁 점열
  ④ WalkStub(reversed)   : 역방향 스텁

  반환: (startStub, endStub)
```

```csharp
// 줄 1666–1691
var (rawSrcStub, rawTgtStub) = StubExtractor.ForPipe(stubPipe);

// 방향 정합 (기존배관 방향 vs 작업 start/end)
bool fwd = Dist(ts, ps) + Dist(te, pe) <= Dist(ts, pe) + Dist(te, ps);
var startStub = fwd ? rawSrcStub : rawTgtStub;
var endStub   = fwd ? rawTgtStub : rawSrcStub;

// PoC 고정 — 스텁[0]을 실제 PoC로 덮어씀
startStub[0] = new Point3D(row.Sx, row.Sy, row.Sz);
endStub[0]   = new Point3D(row.Gx, row.Gy, row.Gz);
row.StartStub = startStub;
row.EndStub   = endStub;

// A* 시작/끝 = 스텁 끝점 (랙 위 자유 공간)
var se = startStub[^1];
var ee = endStub[^1];
(double sx, double sy, double sz) = SnapPocToFreeCell(se.X, se.Y, se.Z, null);
(double gx, double gy, double gz) = SnapPocToFreeCell(ee.X, ee.Y, ee.Z, null);

// 작업 등록 — A* 는 스텁끝→스텁끝 구간만 탐색
int tidx = _engine.AddTask(sx, sy, sz, gx, gy, gz, utility, group);
// → r3d_add_task(H, sx,sy,sz, gx,gy,gz, ...)
if (dia > 0) _engine.SetTaskDiameter(tidx, dia);   // r3d_set_task_diameter
if (_useBundlePattern)
    _engine.SetTaskGoalDir(tidx, AxisSnap(endStub));  // r3d_set_task_goal_dir
```

---

### 경로 B: 직접 PoC 라우팅 (Shortest / 매칭 기존배관 없음)

```csharp
// 줄 1697–1719
// 학습면 조회 (없으면 기하 규칙으로 폴백)
int startFace = LearnedFace("EQUIP", group, utility);   // PatternStore
int endFace   = LearnedDuctFace(row);

// 시작점 전처리
(sx, sy, sz) = DropStartBelowEquipment(row.Sx, row.Sy, row.Sz);
    // 설비 내부에 묻힌 PoC → 설비 바닥 ½셀 아래로
(sx, sy, sz) = LiftPocToSurface(sx, sy, sz, startFace);
    // 설비/덕트 박스 내부면 → 학습면(or 기하 최소면) 외부 ½셀 투영
(sx, sy, sz) = SnapPocToFreeCell(sx, sy, sz, startFace);
    // 학습면 방향 최대 16셀 행진 → 자유 셀 탐색

// 종단점 전처리 (동일)
(gx, gy, gz) = LiftPocToSurface(row.Gx, row.Gy, row.Gz, endFace);
(gx, gy, gz) = SnapPocToFreeCell(gx, gy, gz, endFace);

// 작업 등록 — A* 는 PoC→PoC 전구간 탐색
int tidx = _engine.AddTask(sx, sy, sz, gx, gy, gz, utility, group);
if (dia > 0) _engine.SetTaskDiameter(tidx, dia);
```

---

### 회랑 셀 주입 (PatternApplied/groupMode)

```csharp
// 줄 1723–1724
long[]? l2bCells = (_useDesignCorridor || groupMode)
    ? BuildDesignCorridorCells(rowPositions, radius: 2)
    : null;
// 기존배관 폴리라인을 cell/2 샘플 → ±2셀 팽창 → long[] IJ K

// 번들 회랑 + 특징 스파인 셀 합산
long[] allCorrCells = Combine(l2bCells, bundleCorr, featureSpineCorr);
_engine.SetCorridorCells(allCorrCells);
// → r3d_set_corridor_cells(H, ijk, n)
```

---

## 6. 엔진 실행

### A* 호출

```csharp
// RouteRowsAsync 줄 2807–2873
bool hier = totalCells > 5_000_000 && _useHierarchicalCorridor;

if (hier)
    engine.RouteCorridorMulti(factor, 2, priority, 0);
    // → r3d_route_corridor_multi(H, factor, probe_r, priority, ...)

else if (showProgress)
    engine.RouteMultiProgress(priority, progressCallback, shouldCancel);
    // → r3d_route_multi_progress(H, priority, cb, userPtr)

else
    engine.RouteMulti(priority);
    // → r3d_route_multi(H, priority)
```

`priority` = `"diameter"` (굵은 배관 우선) 또는 `"utility"` (유틸 묶음 후 관경)

### 내부 Escalation (route_multi_impl, 대형 격자)

```
배관 1건당:
  1. astar_weighted (max_exp=300k probe)    → 빠른 시도
  2. route_corridor (coarse factor≈200mm/cell,
                     fine corridor 실패 시 반경×2 최대 3회) → 계층 탐색
  3. astar_weighted (max_exp=fallback_exp)  → 마지막 시도
  → rip-up 라운드 (최대 10회, blocker ≤ 4개)
  → CBS-lite (depth ≤ 3, 연쇄 rip-up)
```

---

## 7. 결과 수집: `CacheResults()`

```csharp
// 줄 2510–2534
foreach (int e in added.Keys)
{
    var r = _engine.GetResult(e);
    // → r3d_get_result(H, e, out R3dResult)
    // → r3d_copy_path(H, e, buf, n)  — 셀 IJ K 배열

    rows[added[e]].Success   = r.Success;
    rows[added[e]].Path      = r.Path;      // Cell[]
    rows[added[e]].LastFail  = r.Fail;      // RouteFail enum
    rows[added[e]].ElapsedMs = r.ElapsedMs;
}
```

---

## 8. FollowExisting 전용: `ReplicateMatchedPipes()`

A* 결과를 버리고 기존설계 폴리라인을 셀로 복제한 뒤 막힌 구간만 국소 A*로 수리한다.

```csharp
// 줄 2542–2603
private int ReplicateMatchedPipes(IList<int> added)
{
    // 1. 수리 전용 엔진 생성 (장애물만, 배관 마크 없음)
    var rep = new Engine();
    rep.SetGrid(...);
    rep.SetParams(wHeur: 2.0);       // r3d_set_params
    foreach (var obs in obstacles)
        rep.AddObstacle(...);         // r3d_add_obstacle
    AddFacilityObstacles(rep, allRows);

    // 2. 각 행 처리
    foreach (int pos in added)
    {
        ExistingPipe? pipe = FindMatchingExistingPipe(rows[pos]);
        if (pipe == null) continue;   // 매칭 없음 → A* 결과 유지

        // 기존배관 폴리라인 → 셀 목록
        var cells = PolylineToCells(pipe.Points, g);
        //   각 구간 cell/2 샘플링
        //   비인접 점프 → AppendOrtho (z→x→y 순 6-연결 채움)

        // 셀 복제 + 막힌 구간 국소 수리
        var result = new List<Cell>();
        foreach (var (prev, cur) in Pairwise(cells))
        {
            if (!Blocked(cur))
                result.Add(cur);              // 자유 → 그대로 복제
            else
                result.AddRange(RepairAStar(prev, NextFreeCell(cur), rep));
                // RepairAStar:
                //   rep.AddTask(a, b)          r3d_add_task
                //   rep.RouteTask(task)         r3d_route_task
                //   r.Success ? r.Path : null
        }

        // 직선화 후 A* 결과 덮어씀
        result = StraightenOrtho(result);     // 직선 L이 자유이면 중간 제거
        result = DeJog(result);               // 짧은 지터 제거
        rows[pos].Path      = result;
        rows[pos].Success   = true;
        rows[pos].StartStub = null;
        rows[pos].EndStub   = null;
    }
    return replicated;
}
```

---

## 9. ProgressFn 콜백 — 진행/취소

```csharp
// Engine.cs 줄 222–239
engine.RouteMultiProgress(priority, p =>
{
    if (p.Phase == 0)
        onProgress(p);                  // 탐색 중: 진행률 갱신

    if (p.Phase == 1)
        onPipe(new RouteProgress(p));   // 배관 완료: 경로 수신 → 3D 즉시 추가

    // 취소: shouldCancel() == true → 1 반환
    return (shouldCancel != null && shouldCancel()) ? 1 : 0;
    // 비0 반환 시 C++ 엔진이 현재 배관 A* 즉시 중단
}, shouldCancel: () => _cancelRequested);
```

```csharp
// Native.cs 줄 132–136
[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
public delegate int R3dProgressFn(
    IntPtr user,
    int    phase,          // 0=탐색중, 1=배관완료
    int    orderIndex,     // 정렬 순서 (굵은→가는)
    int    taskIndex,
    int    success,
    double lengthMm,
    int    turns,
    long   expandedNodes,
    double elapsedMs,
    int    done, int total, double progress01,
    IntPtr pathIjk, int pathLen
);
// 반환: 0=계속, 비0=취소
```

---

## 10. 전체 Native 함수 호출 순서 요약

```
r3d_create()
r3d_set_grid(H, grid)
r3d_set_params(H, params)
r3d_set_segment_astar(H, ...)
r3d_set_octree_guide(H, ...)
r3d_set_per_task_radius(H, 1)
r3d_set_pipe_gap(H, 60.0)
r3d_set_min_straight(H, 2.0)
r3d_set_min_straight_mm(H, 100.0)
r3d_set_cbs_depth(H, depth)
[Shortest+대형] r3d_set_runtime_options(H, {max_exp=8M})

r3d_add_obstacle(H, x1,y1,z1, x2,y2,z2)  × N   장애물
r3d_add_passthrough(H, ...)               × P   통과허용 장애물
r3d_add_obstacle (설비+덕트+완료배관)      × M   AddFacilityObstacles

r3d_add_task(H, sx,sy,sz, gx,gy,gz, ...)  × K   작업 등록
r3d_set_task_diameter(H, tidx, dia)        × K
r3d_set_task_goal_dir(H, tidx, axis)       × K   패턴ON 시

r3d_set_corridor_cells(H, ijk[], n)              PatternApplied/groupMode 시

── 실행 ──
r3d_route_multi_progress(H, priority, cb, ptr)
  또는 r3d_route_multi(H, priority)
  또는 r3d_route_corridor_multi(H, factor, r, priority, ...)

── 결과 수집 ──
r3d_get_result(H, task, out R3dResult)     × K
r3d_copy_path(H, task, buf, n)             × K

── FollowExisting 수리 엔진 ──
r3d_create()  → r3d_set_grid/params/obstacles
r3d_add_task + r3d_route_task  (국소 A* 수리, 막힌 구간당)
r3d_destroy()

── 메인 엔진 ──
r3d_destroy(H)                                   Dispose()
```

---

## 11. 모드별 핵심 차이 한눈에 보기

| 단계 | Shortest | PatternApplied | FollowExisting |
|------|----------|----------------|----------------|
| wHeur | 1.4 | 2.0 | 2.0 |
| wCorr | 0 | **cell×0.5** | 0 |
| 스텁 추출 | ✗ | **✓** (WalkStub) | ✓ |
| A* 범위 | PoC→PoC | **스텁끝→스텁끝** | 스텁끝→스텁끝 |
| 회랑 주입 | ✗ | **✓** (기존+번들) | ✗ |
| rip-up | ✓ | ✓ | ✓ |
| CBS-lite | _useCbs | _useCbs | _useCbs |
| 복제 후처리 | ✗ | ✗ | **✓** (ReplicateMatchedPipes) |
| 국소 수리 | ✗ | ✗ | **✓** (RepairAStar) |
| 최종 경로 출처 | A* 직접 | A* + 스텁 합산 | **기존설계 복제** + A* 수리 |

---

---

# 장애물 등록 단계별 프로세스 및 예제 코드

## 1. 장애물 3종류

라우팅 엔진에 등록되는 장애물은 출처에 따라 3종류로 구분된다.

| 종류 | 출처 | 등록 함수 | 비고 |
|------|------|-----------|------|
| **정적 장애물** | `TB_BIM_OBSTACLE` (scene.txt) | `AddObstacle` / `AddPassthrough` | COLLISION_PASS=1이면 통과 처리 |
| **설비·덕트·레터럴** | `TB_EQUIPMENTS` · `TB_DUCT` · `TB_LATERAL_PIPE` | `AddBoxObstacle` | 두께 0 축 최소 1셀로 팽창 |
| **완료배관·스텁** | 이미 라우팅 성공한 `TaskRowVM.Path` · `StartStub` · `EndStub` | `AddPathObstacle` / `AddPolylineObstacle` | 새 배관이 기존 배관을 관통하지 않도록 |

---

## 2. 전체 등록 흐름 (BuildEngineForRows 내 순서)

```
BuildEngineForRows(rowPositions, groupMode)
  │
  ├─ [Step 1] 엔진 생성 + 격자/파라미터 설정
  │
  ├─ [Step 2] 정적 장애물 등록
  │    foreach (var o in scene.Obstacles)
  │      o.IsPassThrough → engine.AddPassthrough(...)   ← COLLISION_PASS=1
  │      else            → engine.AddObstacle(...)      ← 충돌 장애물
  │
  └─ [Step 3] 설비·덕트·완료배관 등록
       AddFacilityObstacles(engine, currentRows)
         ├─ foreach Equipment  → AddBoxObstacle(...)
         ├─ foreach DuctsLaterals → AddBoxObstacle(...)
         └─ foreach 완료된 Tasks (currentRows 제외)
              AddPathObstacle(engine, row.Path, grid, r)
              AddPolylineObstacle(engine, row.StartStub, r)
              AddPolylineObstacle(engine, row.EndStub, r)
```

---

## 3. Step 2 — 정적 장애물 (scene.txt / DB 로드 원본)

```csharp
// SceneViewModel.cs — BuildEngineForRows 줄 1643–1648
foreach (var o in _scene.Obstacles)
{
    if (o.IsPassThrough)
        // COLLISION_PASS = 1: 바닥 슬래브·격자보 등 — 점유맵에는 기록하되
        // A* 이웃 탐색에서 충돌 판정 제외. 배관이 관통 가능.
        _engine.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
    else
        // 일반 장애물 — A* 가 이 AABB 에 걸리는 셀을 blocked 처리.
        _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
}
```

**IsPassThrough 결정 (ObstacleDbLoader.cs):**

```csharp
// TB_BIM_OBSTACLE.COLLISION_PASS 컬럼
bool isPassthrough = reader.GetInt32(colCollisionPass) == 1;
new ObstacleBox
{
    MinX = reader.GetDouble(colMinX), ...
    IsPassThrough = isPassthrough,
};
```

---

## 4. 헬퍼 함수 4종

### 4-1. `AddBoxObstacle` — 두께 0 축 팽창

설비·덕트처럼 한 축이 매우 얇은 박스(벽판, 격자보)를 최소 1셀 두께로 강제 팽창한다.
팽창하지 않으면 셀 경계 사이에 끼어 A* 가 점유를 인식하지 못한다.

```csharp
// SceneViewModel.cs 줄 2443–2450
private static void AddBoxObstacle(
    Engine engine,
    double mnx, double mny, double mnz,
    double mxx, double mxy, double mxz,
    double minT)   // minT = CellMm (최소 1셀 두께)
{
    // 각 축의 두께가 minT 미만이면 중심 기준으로 양쪽 확장
    if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
    if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
    if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
    engine.AddObstacle(mnx, mny, mnz, mxx, mxy, mxz);
}

// 사용 예
double minT = _scene.Grid.CellMm;   // 예: 50mm
foreach (var e in scene.Equipment)
    AddBoxObstacle(engine, e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ, minT);
foreach (var d in scene.DuctsLaterals)
    AddBoxObstacle(engine, d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ, minT);
```

---

### 4-2. `AddPolylineObstacle` — 스텁 폴리라인 → 구간별 AABB

스텁(수직+엘보 리드인)은 Point3D 배열이다.
각 구간(선분)을 관경 반경 r로 팽창한 AABB로 등록한다.
셀 복셀화 없이 세그먼트당 박스 1개이므로 API 호출이 최소화된다.

```csharp
// SceneViewModel.cs 줄 2431–2440
private static void AddPolylineObstacle(
    Engine engine,
    IReadOnlyList<Pt3> poly,
    double r)   // r = 관경 반경 (DiameterMm / 2)
{
    for (int i = 1; i < poly.Count; i++)
    {
        var a = poly[i - 1];
        var b = poly[i];
        // 선분 AABB = min(a,b) - r ... max(a,b) + r
        engine.AddObstacle(
            Math.Min(a.X, b.X) - r, Math.Min(a.Y, b.Y) - r, Math.Min(a.Z, b.Z) - r,
            Math.Max(a.X, b.X) + r, Math.Max(a.Y, b.Y) + r, Math.Max(a.Z, b.Z) + r);
    }
}

// 사용 예 (완료 배관의 스텁)
double r = row.DiameterMm > 0 ? row.DiameterMm / 2.0 : cellMm;
if (row.StartStub != null) AddPolylineObstacle(engine, row.StartStub, r);
if (row.EndStub   != null) AddPolylineObstacle(engine, row.EndStub,   r);
```

---

### 4-3. `AddPathObstacle` — A* 경로(셀 목록) → 직선 구간별 AABB

A* 결과 `PathCell[]`은 셀 인덱스 배열이다.
방향이 바뀌는 꺾임 지점에서만 구간을 끊어 AABB로 등록한다.
배관 전 구간을 셀 하나씩 등록하면 API 호출이 수천 회이지만,
직선 구간 단위로 묶으면 호출이 꺾임 수+1 개로 줄어든다.

```csharp
// SceneViewModel.cs 줄 2453–2472
private static void AddPathObstacle(
    Engine engine,
    PathCell[] path,
    GridMeta g,
    double r)
{
    // 인접 두 셀의 이동 방향(부호 벡터) 계산
    (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
        (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));

    int n   = path.Length;
    int seg = 0;                              // 현재 구간의 시작 인덱스
    var cur = Dir(path[0], path[1]);          // 현재 진행 방향

    for (int i = 2; i <= n; i++)
    {
        // 방향이 바뀌거나 배열 끝이면 구간 확정
        var d = (i < n) ? Dir(path[i - 1], path[i]) : (int.MinValue, 0, 0);
        if (d != cur)
        {
            // 구간 시작 셀 ~ 끝 셀을 월드 좌표로 변환
            var pa = CellToWorld(g, path[seg]);
            var pb = CellToWorld(g, path[i - 1]);
            // 반경 r 팽창 AABB 등록
            engine.AddObstacle(
                Math.Min(pa.X, pb.X) - r, Math.Min(pa.Y, pb.Y) - r, Math.Min(pa.Z, pb.Z) - r,
                Math.Max(pa.X, pb.X) + r, Math.Max(pa.Y, pb.Y) + r, Math.Max(pa.Z, pb.Z) + r);
            seg = i - 1;
            if (i < n) cur = Dir(path[i - 1], path[i]);
        }
    }
}

// 사용 예
double r = row.DiameterMm > 0 ? row.DiameterMm / 2.0 : cellMm;
AddPathObstacle(engine, row.Path, scene.Grid, r);
```

---

### 4-4. `AddFacilityObstacles` — 설비+덕트+완료배관 일괄 등록

```csharp
// SceneViewModel.cs 줄 2374–2405
private void AddFacilityObstacles(
    Engine engine,
    HashSet<int> currentRows,         // 현재 라우팅 중인 행 — 자기 배관은 제외
    bool forceFacilities = false)
{
    if ((!_includeFacilities && !forceFacilities) || _scene == null) return;

    double cell = _scene.Grid.CellMm;
    double minT = cell;   // 두께 0 축 최소 1셀 팽창

    // ── 정적 설비 ──
    // 종단 PoC 가 포함된 덕트/설비도 솔리드 장애물로 추가.
    // (예전에는 PoC 포함 박스를 제외 → 배관이 덕트를 관통하는 버그)
    // 해결: LiftPocToSurface 가 PoC를 표면 바깥으로 투영.
    foreach (var e in _scene.Equipment)
        AddBoxObstacle(engine, e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ, minT);

    foreach (var d in _scene.DuctsLaterals)
        AddBoxObstacle(engine, d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ, minT);

    // ── 완료된 다른 배관 경로 + 고정 스텁 ──
    for (int i = 0; i < Tasks.Count; i++)
    {
        if (currentRows.Contains(i)) continue;   // 자기 배관 제외 (자기 스텁에 막히지 않게)

        var row = Tasks[i];
        if (!row.Success || row.Path.Length < 2) continue;

        // 실제 관경 반경. 미상이면 1셀 크기로 보수적 막음.
        double r = row.DiameterMm > 0 ? row.DiameterMm / 2.0 : cell;

        AddPathObstacle(engine, row.Path, _scene.Grid, r);       // A* 경로 구간
        if (row.StartStub != null) AddPolylineObstacle(engine, row.StartStub, r);  // 출발 스텁
        if (row.EndStub   != null) AddPolylineObstacle(engine, row.EndStub,   r);  // 종단 스텁
    }
}
```

---

## 5. C ABI 레이어 — C# → C++

### Engine.cs (래퍼)

```csharp
// Interop/Engine.cs 줄 175–181
public void AddObstacle(
    double minx, double miny, double minz,
    double maxx, double maxy, double maxz)
    => Check(Native.r3d_add_obstacle(H, minx, miny, minz, maxx, maxy, maxz), "add_obstacle");

public void AddPassthrough(
    double minx, double miny, double minz,
    double maxx, double maxy, double maxz)
    => Check(Native.r3d_add_passthrough(H, minx, miny, minz, maxx, maxy, maxz), "add_passthrough");
```

### Native.cs (P/Invoke)

```csharp
// Interop/Native.cs 줄 107–111
[DllImport("routing3d_capi.dll", CallingConvention = CallingConvention.Cdecl)]
public static extern int r3d_add_obstacle(
    IntPtr e,
    double minx, double miny, double minz,
    double maxx, double maxy, double maxz);

[DllImport("routing3d_capi.dll", CallingConvention = CallingConvention.Cdecl)]
public static extern int r3d_add_passthrough(
    IntPtr e,
    double minx, double miny, double minz,
    double maxx, double maxy, double maxz);
```

---

## 6. C++ 엔진 내부 — 등록 및 점유맵 변환

### 등록 단계 (routing3d_capi.cpp)

```cpp
// routing3d_capi.cpp 줄 1977–2003
extern "C" R3dStatus r3d_add_obstacle(R3dEngine* e,
    double minx, double miny, double minz,
    double maxx, double maxy, double maxz)
{
    Obstacle o;
    o.min_xyz = Vec3{minx, miny, minz};
    o.max_xyz = Vec3{maxx, maxy, maxz};
    e->doc.obstacles.push_back(std::move(o));  // ← 리스트에 추가만. 복셀화는 라우팅 시점.
    return R3D_OK;
}

extern "C" R3dStatus r3d_add_passthrough(R3dEngine* e, ...)
{
    Obstacle o;
    o.min_xyz = Vec3{minx, miny, minz};
    o.max_xyz = Vec3{maxx, maxy, maxz};
    e->doc.passthrough.push_back(std::move(o));  // ← 별도 리스트
    return R3D_OK;
}
```

**핵심:** `r3d_add_obstacle/passthrough` 호출 시점에는 AABB를 리스트에 추가만 한다.
실제 복셀화(점유맵 셀 마킹)는 `r3d_route_multi_progress` 호출 시 내부에서 이루어진다.

---

### 점유맵 변환 단계 (라우팅 시작 시)

라우팅 시작(`route_multi_progress` 내부)에서 총 셀 수에 따라 백엔드를 자동 선택하고
`doc.obstacles`를 순회해 `add_box()`로 셀을 마킹한다.

```cpp
// routing3d_capi.cpp 줄 1161 / 1315–1316
const long long large_threshold = 5'000'000LL;    // env R3D_LARGE_GRID_THRESHOLD
long long total_cells = (long long)doc.shape.i * doc.shape.j * doc.shape.k;
const bool large_grid = total_cells > large_threshold;

// ── 백엔드 자동 선택 ──
if (!large_grid)
    DenseOccupancy occ = occupancy_from_doc(doc);   // 배열 기반
else
    ImplicitOccupancy occ = implicit_from_doc(doc);  // AABB 인덱스 기반
```

```cpp
// 소격자 (< 5M셀): DenseOccupancy — 배열 순회로 복셀화
DenseOccupancy occupancy_from_doc(const SceneDoc& doc) {
    DenseOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles)
        occ.add_box(AABB(o.min_xyz, o.max_xyz));   // AABB → 셀 범위 순회 → data[lin] = 1
    return occ;
}

// 대격자 (≥ 5M셀): ImplicitOccupancy — AABB 인덱스 유지 (복셀화 없음)
ImplicitOccupancy implicit_from_doc(const SceneDoc& doc) {
    ImplicitOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles)
        occ.add_box(AABB(o.min_xyz, o.max_xyz));   // SpatialBoxIndex에 AABB 등록
    return occ;
}
```

### 백엔드별 `is_blocked()` 동작

```
DenseOccupancy:
  is_blocked(cell) → data[i*Nj*Nk + j*Nk + k] != 0   O(1) 배열 조회

ImplicitOccupancy:
  is_blocked(cell) → SpatialBoxIndex.query(cell_center)   O(장애물 수 / 버킷)
  → 유니폼 그리드 버킷으로 후보 필터 → AABB 포함 검사
  → 메모리 O(장애물수 + 마킹셀) — 셀 수 무관
```

### passthrough 처리

```cpp
// passthrough 는 별도로 관리:
// r3d_add_passthrough → doc.passthrough 리스트
// 라우팅 시 occupancy_from_doc 는 doc.obstacles 만 사용
//   → passthrough AABB 는 점유맵에 기록되지 않음 → A* 가 자유 셀로 통과
// r3d_copy_passthrough → 디버그/시각화용으로 별도 조회 가능
```

---

## 7. 단계별 전체 정리 (순서도)

```
[DB 로드 / scene.txt 파싱]
  ↓
  ObstacleBox.IsPassThrough = (COLLISION_PASS == 1)
  SceneData.Obstacles, Equipment, DuctsLaterals 채워짐

[BuildEngineForRows 시작]
  ↓
  Step 1: engine = new Engine()                     r3d_create()
          engine.SetGrid(cell, ox, oy, oz, nx, ny, nz)
          engine.SetParams(...)

  Step 2: foreach scene.Obstacles
    ├─ IsPassThrough=true  → engine.AddPassthrough(minX,minY,minZ, maxX,maxY,maxZ)
    │                                               → doc.passthrough.push_back(o)
    └─ IsPassThrough=false → engine.AddObstacle(minX,minY,minZ, maxX,maxY,maxZ)
                                                    → doc.obstacles.push_back(o)

  Step 3: AddFacilityObstacles(engine, currentRows)
    ├─ foreach Equipment
    │    AddBoxObstacle(engine, ..., minT=cell)     → engine.AddObstacle(팽창AABB)
    │
    ├─ foreach DuctsLaterals
    │    AddBoxObstacle(engine, ..., minT=cell)     → engine.AddObstacle(팽창AABB)
    │
    └─ foreach 완료 Tasks (currentRows 제외)
         r = DiameterMm/2  (미상이면 cell)
         AddPathObstacle(engine, row.Path, grid, r)
           → 직선 구간별 AABB(±r) → engine.AddObstacle(...)
         AddPolylineObstacle(engine, row.StartStub, r)
           → 구간별 AABB(±r) → engine.AddObstacle(...)
         AddPolylineObstacle(engine, row.EndStub, r)

[r3d_route_multi_progress 호출]
  ↓
  내부: total_cells = Nx × Ny × Nz 계산
  ├─ < 5M셀: occupancy_from_doc() → DenseOccupancy → add_box() 순회 복셀화
  └─ ≥ 5M셀: implicit_from_doc() → ImplicitOccupancy → SpatialBoxIndex에 등록

  A* 탐색: is_blocked(cell) 호출로 장애물 여부 판정
    ├─ Dense:   data[lin] != 0 → blocked
    └─ Implicit: SpatialBoxIndex.query(center) → AABB 포함 여부
```

---

## 8. 자주 나오는 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| 배관이 설비/덕트를 관통 | 종단 PoC 포함 박스를 제외하던 구 코드 | AddBoxObstacle 로 전부 솔리드 + LiftPocToSurface |
| PoC 시작점이 즉시 blocked (exp=0) | PoC 가 설비 솔리드에 묻힘 | SnapPocToFreeCell (학습면 방향 최대 16셀 탐색) |
| 두꺼운 배관이 가는 배관 경로에 겹침 | 완료배관 마킹 반경이 셀 1개(=cell*0.6) | AddPathObstacle 에 실관경 반경 r = DiameterMm/2 |
| 스텁끼리 교차/겹침 | 스텁이 장애물로 미등록 | AddPolylineObstacle(row.StartStub/EndStub, r) |
| 얇은 덕트 관통 | 두께 < CellMm → 셀 미점유 | AddBoxObstacle 에서 minT = CellMm 팽창 |
| 25mm 셀 메모리 폭발 | 5M셀 이상인데 Dense 선택 | ImplicitOccupancy 자동 전환 (5M 임계 확인) |

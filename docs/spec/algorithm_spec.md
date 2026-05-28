# 알고리즘 명세서 (Algorithm Specification) — Routing3D

> Phase 2 인터페이스 동결 · [phase2_plan.md](../phase2_plan.md) Step 2.1
> 입력: [phase2_input_notes.md](../phase2_input_notes.md) · 레퍼런스 구현: `python_experiments/routing3d_py/`
> 버전 1.0 · 2026-05-28 · 단위 **mm**, 기본 셀 50mm
> 본 명세는 **검증된 동작만** 기술한다. Phase 3 C++ 구현은 이 명세를 모호함 없이 따라야 하며, 회귀 골든셋([regression_set.md](regression_set.md)) 지표를 허용범위 내 재현해야 한다.

---

## 0. 표기

- `cell = (i, j, k)` 정수 셀 인덱스. `world = (x, y, z)` mm.
- `shape = (nx, ny, nz)`, `origin`(mm), `cell_mm`(float, mm).
- 의사코드는 0-기반 인덱스, `//`=정수 나눗셈, `floor`=내림.

---

## 1. 좌표 · 격자 규약 (동결)

```
to_world(cell)  = origin + (cell + 0.5) * cell_mm        # 셀 중심
to_cell(world)  = floor((world - origin) / cell_mm)      # 포함 셀
in_bounds(cell) = 0<=i<nx and 0<=j<ny and 0<=k<nz
world_bounds()  = (origin, origin + shape * cell_mm)
```

- **불변식 G1**: 격자 범위 밖 셀은 항상 점유(blocked)로 간주한다.
- **불변식 G2**: 길이·좌표·비용의 단위는 모두 mm. 각도/대각선 없음.

---

## 2. 점유맵 (Occupancy Map)

### 2.1 질의 인터페이스 (백엔드 무관 계약)

```
is_blocked(cell) -> bool      # not in_bounds(cell) 면 True, 아니면 점유비트
in_bounds(cell)  -> bool
bounds()         -> ((0,0,0), shape)
to_world / to_cell / world_bounds
block_cell(cell)              # in_bounds 일 때만 점유 설정
add_box(AABB) -> int          # 복셀화, 신규 점유 셀 수 반환
inflate(radius_cells, connectivity in {6,26}) -> OccupancyMap
copy() -> OccupancyMap
to_numpy() -> bool[nx,ny,nz]
```

- **계약 O1**: 3개 백엔드(Dense/BitPacked/Sparse)는 위 질의에 대해 **동일한 결과**를 반환한다(메모리 특성만 다름). A* 등 사용자 코드는 이 인터페이스에만 의존한다.

### 2.2 AABB 복셀화 (`add_box`)

```
add_box(box):
    cell_lo = max( floor((box.lo - origin)/cell_mm), 0 )
    cell_hi = min( ceil ((box.hi - origin)/cell_mm), shape )
    if any(cell_lo >= cell_hi): return 0
    채움: [cell_lo, cell_hi) 범위의 모든 셀을 점유로 설정
    return 신규로 점유된 셀 수
```

- AABB 는 `lo < hi`(모든 축) 를 만족해야 함(퇴화 박스는 호출측에서 스킵).

### 2.3 팽창 (`inflate`) — 하드 클리어런스

```
inflate(r, connectivity):
    offsets = NEIGHBORS_6 (면) 또는 NEIGHBORS_26 (면+모서리+꼭짓점)
    grid = 현재 점유
    repeat r 회: grid = dilate_once(grid, offsets)   # 이웃 방향 OR-시프트
    return 새 점유맵(grid)
```

---

## 3. 직교 A* (균일 비용) — `astar`

상태 = **셀**. 6방향(±X,±Y,±Z) 이동, 대각선 금지.

```
astar(occ, start, goal, step_cost = cell_mm):
    if is_blocked(start) or is_blocked(goal): return 실패
    if start == goal: return 경로 [start]

    h(c)      = manhattan(c, goal) * step_cost          # manhattan = |Δi|+|Δj|+|Δk|
    open      = min-heap of (f, counter, cell)          # counter: 단조증가, tie-break
    g[start]  = 0;  push (h(start), counter++, start)
    came_from = {};  closed = {}

    while open:
        (_, _, cur) = pop(open)
        if cur in closed: continue                       # 낡은 항목 무시
        closed.add(cur)
        if cur == goal: return reconstruct(came_from, cur)
        for d in NEIGHBORS_6:
            nb = cur + d
            if nb in closed or is_blocked(nb): continue
            t = g[cur] + step_cost
            if t < g.get(nb, +inf):
                g[nb] = t;  came_from[nb] = cur
                push (t + h(nb), counter++, nb)
    return 실패
```

- **계약 A1 (admissibility/consistency)**: 한 칸 비용 = `step_cost = cell_mm`, `h = manhattan×cell_mm` → admissible & consistent → 최적 경로 보장.
- **계약 A2 (결정성)**: 동일 입력은 동일 경로를 반환한다. tie-break 는 (f, 삽입순서 counter). 이웃 순서는 `NEIGHBORS_6 = [+X,−X,+Y,−Y,+Z,−Z]` 고정.
- `length_mm = (셀 수 − 1) × cell_mm`. `turns` = 인접 이동방향이 바뀐 횟수.

---

## 4. 비용함수 A* — `astar_weighted`

상태 = **(셀, 진입방향 dir_idx)**. turn penalty 때문에 같은 셀도 진입 방향별로 비용이 다르다. 시작 상태 `dir_idx = −1`(방향 없음).

### 4.1 비용 파라미터 (`RouteParams`, mm)

```
cell_mm, w_turn, w_clear, clearance_radius, clearance_connectivity, w_tier:{z->mm}
검증: 모든 값 >= 0 (음수/보너스 금지),  clearance_connectivity in {6,26}
```

### 4.2 클리어런스 거리맵 (bounded distance transform)

```
clearance_map(occ, R, connectivity):       # 각 셀 → 가장 가까운 장애물까지 거리(셀), 상한 R
    dist[*]      = R
    dist[장애물] = 0
    current = 장애물집합
    for d in 1..R:
        dilated = dilate_once(current, offsets)
        newly   = dilated AND NOT current
        dist[newly] = d
        current = dilated
    return dist                              # 장애물=0, 멀수록 큼(최대 R)
```

### 4.3 이동 비용 / 셀 페널티

```
cell_penalty(c):
    p = 0
    if clearance 사용 and dist[c] < clearance_radius:
        p += w_clear * (clearance_radius - dist[c])      # 가까울수록 큼, 인접(0)에서 최대
    if w_tier: p += w_tier.get(c.z, 0)
    return p

move_cost(to, prev_off, move_off):
    c = cell_mm
    if prev_off is not None and move_off != prev_off: c += w_turn   # 회전
    c += cell_penalty(to)
    return c

heuristic(c, goal) = manhattan(c, goal) * cell_mm
```

- **계약 C1 (admissibility 보존)**: `cell_penalty ≥ 0`, `w_turn ≥ 0` → 한 칸 이동 ≥ `cell_mm` → 맨해튼 휴리스틱 admissible & consistent. **이 불변식은 절대 위반 금지**(보너스/감산 도입 불가).

### 4.4 탐색 루프

```
astar_weighted(occ, start, goal, params):
    if is_blocked(start) or is_blocked(goal): return 실패
    if start == goal: return 경로 [start] (cost 0)
    model = CostModel(occ, params)                       # clearance 1회 사전계산
    open  = min-heap of (f, counter, cell, dir_idx)
    g[(start,-1)] = 0;  push (heuristic(start,goal), counter++, start, -1)
    came_from = {};  closed = {}

    while open:
        (_, _, cell, dir_idx) = pop(open)
        s = (cell, dir_idx)
        if s in closed: continue
        closed.add(s)
        if cell == goal: return reconstruct_states(came_from, s)   # 방향성분 제거
        prev_off = None if dir_idx<0 else NEIGHBORS_6[dir_idx]
        for nidx, d in enumerate(NEIGHBORS_6):
            nb = cell + d;  ns = (nb, nidx)
            if ns in closed or is_blocked(nb): continue
            t = g[s] + model.move_cost(nb, prev_off, d)
            if t < g.get(ns, +inf):
                g[ns] = t;  came_from[ns] = s
                push (t + model.heuristic(nb, goal), counter++, nb, nidx)
    return 실패
```

- 결과: `cost_mm = g[목표상태]`(페널티 포함), `length_mm = (셀 수−1)×cell_mm`(기하).
- **계약 W1 (결정성)**: `astar` 와 동일하게 (f, counter) tie-break + 고정 이웃 순서 → 재현 가능.

---

## 5. 다중 배관 순차 라우팅 — `route_sequential`

```
route_sequential(occ, tasks, params, priority, pipe_radius, snap_to_free):
    work    = occ.copy()                                 # 원본 보존
    ordered = order_tasks(occ, tasks, priority)
    results = []
    for task in ordered:
        s = snap(work, to_cell(task.start), snap_to_free) # 점유면 인접 빈 셀
        g = snap(work, to_cell(task.end),   snap_to_free)
        r = astar_weighted(work, s, g, params)
        results.append(r)
        if r.success: mark_pipe(work, r.path, pipe_radius) # 경로(+팽창)를 점유로 추가
    return MultiRouteResult(results, work, priority)

order_tasks(occ, tasks, priority):
    dist(t) = manhattan(to_cell(t.start), to_cell(t.end))
    longest : sort by dist desc       (기본)
    shortest: sort by dist asc
    utility : sort by (utility_label asc, dist desc)
    original: 입력 순서

mark_pipe(occ, path, radius):
    for c in path: block_cell(c)
    repeat radius 회: 경계 셀의 빈 6-이웃을 점유로 추가     # 배관 굵기/이격 근사
```

- **계약 M1 (충돌 0)**: 성공한 경로들은 쌍별로 셀을 공유하지 않는다(깔린 경로를 점유로 추가하므로).
- **계약 M2**: `route_sequential` 은 원본 `occ` 를 변경하지 않는다(사본 사용).
- 지표: `success_count / fail_count / success_rate / total_length_mm`(성공분 기하 길이 합).
- **범위**: 본 단계는 greedy sequential 베이스라인. rip-up & reroute / CBS 는 **Phase 3**.

---

## 6. 불변식 요약 (Phase 3 단위 테스트로 강제)

| ID | 불변식 |
|---|---|
| G1 | 격자 밖 셀 = 점유 |
| O1 | 3 백엔드 질의 결과 동일 |
| A1/C1 | 비용 가산항 ≥ 0 → 휴리스틱 admissible & consistent (최적성) |
| A2/W1 | 동일 입력 → 동일 경로 (f, counter tie-break, 고정 이웃 순서) |
| P1 | 경로의 모든 셀은 (원본) 비점유, 연속 셀은 6-이웃 |
| M1 | 다중 배관 성공 경로는 쌍별 셀 비공유(충돌 0) |
| M2 | route_sequential 은 입력 점유맵 불변 |

---

## 7. 상수 (동결)

```
NEIGHBORS_6  = [(+1,0,0),(-1,0,0),(0,+1,0),(0,-1,0),(0,0,+1),(0,0,-1)]   # 순서 고정
NEIGHBORS_26 = 3x3x3 큐브에서 중심 제외 26개
baseline RouteParams = {cell_mm:50, w_turn:500, w_clear:10,
                        clearance_radius:2, clearance_connectivity:6, w_tier:{}}
```

> **참고**: 본 명세의 의사코드는 레퍼런스(`astar.py`/`cost.py`/`multi_route.py`/`occupancy.py`)와 1:1 대응한다. Phase 3 C++ 매핑: `dict g/closed` → 선형 셀 인덱스(×6 방향) 배열, `heapq` → Boost.Heap(decrease-key), 점유맵 → OpenVDB. 자료구조 교체 시에도 위 계약/불변식은 동일하게 보존해야 한다.

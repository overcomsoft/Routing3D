# Phase 1 → Phase 2 입력 노트 — 알고리즘 · 제약 · 우선순위 정리

> 상위 문서: [development_plan.md](development_plan.md) · [phase1_plan.md](phase1_plan.md)
> 작성일: 2026-05-28 · 버전 1.0
> 목적: Phase 1(Python 실험)에서 **검증·동결된 결정사항**을 Phase 2 명세화와 Phase 3 C++ 엔진의 입력으로 정리한다.
> 단위 규약: 모든 길이·좌표는 **밀리미터(mm)**, 셀 크기 기본 50mm(`cell_mm`, 설정 가능).

---

## 0. 요약 (Phase 1 결과)

- 1.1~1.7 구현·테스트 완료. **pytest 203 통과**(DB 통합 6 포함).
- 점유맵(3 백엔드) → 직교 A* → 비용함수(turn/clearance/tier) → 다중 배관 순차 라우팅 → `scene.txt` I/O → 회귀 시나리오 3종 + 베이스라인 파라미터.
- 실데이터(SpaceAI project 6, 장애물 983·배관 208) **순차 라우팅 203/208 성공(98%)**, 충돌 0.
- DoD 5개 중 4개 충족, 마지막 항목(본 노트)으로 완결.

---

## 1. 동결 대상 — Phase 2/3 가 공유하는 계약(Interface)

이 절의 인터페이스는 **변경 시 Phase 3 C++ 와 회귀 데이터가 깨진다.** 신중히 다룬다.

### 1.1 좌표 · 단위 규약
- 월드 좌표·치수 단위 = **mm**. 라디안/각도 없음(직교만).
- 셀 인덱스 `(i, j, k) ∈ ℤ³`, 원점 `origin`(mm), 셀 크기 `cell_mm`(float).
- 셀↔월드: `world_mm = origin + (cell + 0.5) * cell_mm` (셀 중심), `cell = floor((world − origin)/cell_mm)`.
- 격자 범위 밖 셀은 항상 **점유(blocked)** 로 간주(탐색이 격자를 벗어나지 못함).

### 1.2 점유맵 질의 인터페이스 (`OccupancyMap`)
사용자 코드(A*, 비용함수, 시각화)는 **저장 백엔드와 무관하게** 아래에만 의존한다.
- `is_blocked(cell) -> bool` (점유 또는 격자 밖 = True)
- `in_bounds(cell) -> bool`, `bounds() -> ((0,0,0), shape)`
- `to_world(cell)` / `to_cell(world_mm)`, `world_bounds()`
- 변경: `block_cell`, `add_box(AABB)`, `inflate(radius, connectivity=6|26)`, `copy()`, `to_numpy()`
- 백엔드 3종(질의 결과 동일, 메모리 특성만 다름): **Dense**(1B/셀, 기본·최속), **BitPacked**(1bit/셀, ~8× ROI), **Sparse**(점유만 저장 — 점유 빽빽하면 부적합).

### 1.3 비용 파라미터 (`RouteParams`) — mm 단위
| 필드 | 베이스라인 | 의미 |
|---|---|---|
| `cell_mm` | 50.0 | 셀 1칸 이동 기본 비용 |
| `w_turn` | 500.0 | 방향 전환 1회 가산(= 셀 10칸). 회전 최소화 |
| `w_clear` | 10.0 | 장애물 1셀 근접당 가산(mm/셀). **가산 페널티**(감산 보너스 금지) |
| `clearance_radius` | 2 | 근접 페널티 적용 최대 거리(셀) |
| `clearance_connectivity` | 6 | 거리 측정 이웃(6 맨해튼 / 26 체비셰프) |
| `w_tier` | {} | 단(段) 분리 `{z셀: 가산 mm}` |

> 모든 가산항 ≥ 0 → 한 칸 이동 비용 ≥ `cell_mm` → 맨해튼×`cell_mm` 휴리스틱이 **admissible & consistent**(A* 최적성 보존). 이 불변식은 반드시 유지한다.

### 1.4 `scene.txt` 입출력 규약 (포맷 v1)
- 섹션 헤더 + TAB 구분 행. `[grid] [params] [obstacles] [tasks] [results]/[result]/[path]/[visited]`.
- 무손실: 실수는 `repr`, None 은 토큰 `\N`(빈 문자열과 구분). write→read→write 바이트 동일.
- 점유(obstacles)·경로(path)·방문(visited) 3개 레이어 표현. 상세는 [phase1_plan.md §2.5](phase1_plan.md) / `scene_io.py`.

---

## 2. 알고리즘 명세 (Phase 3 C++ 1:1 포팅 기준)

### 2.1 직교 A* (기본, 균일 비용)
- 이웃 6방향(±X,±Y,±Z), 대각선 금지. 상태 = **셀**.
- `f = g + h`, `h = manhattan(cell, goal) × cell_mm`.
- 우선순위 큐 `heapq`, 항목 `(f, counter, cell)` — `counter`(단조 증가)로 안정 tie-break.
- `closed: set`, `g: dict`, `came_from: dict`. 목표 pop 시 `came_from` 역추적으로 경로 복원.

### 2.2 비용함수 A* (`astar_weighted`)
- **상태 = (셀, 진입방향 dir_idx)** — turn penalty 때문에 같은 셀도 진입 방향별로 비용이 다름. 시작 상태 dir_idx = −1.
- 이동 비용 = `cell_mm + (방향 바뀌면 w_turn) + cell_penalty(목적지)`.
- `cell_penalty` = 클리어런스 근접 페널티(`w_clear×(radius−d)`, d<radius) + 단 분리(`w_tier[z]`).
- 클리어런스 거리맵 = **bounded distance transform**(반복 이진 팽창; 장애물=0, 멀수록 max_radius). CostModel 생성 시 1회 사전계산.

### 2.3 다중 배관 순차 라우팅 (`route_sequential`) — 베이스라인
- 우선순위로 정렬 → 장애물 점유맵 사본에 한 배관씩 라우팅 → **성공 경로 셀(+`pipe_radius` 팽창)을 점유로 추가** → 다음 배관이 회피.
- 성공 경로끼리 셀 비공유(충돌 0)가 **불변식**(회귀 테스트로 보장).
- start/end 가 점유면 `snap_to_free` 반경 내 빈 셀로 스냅.

---

## 3. 제약(Constraints) & 불변식(Invariants)

- 경로의 모든 셀은 (원본 장애물 기준) 점유되지 않는다. (`path_hits_obstacle == 0`)
- 경로의 연속 셀은 6-이웃 관계(단일 step).
- 시작·종료 셀은 격자 범위 내.
- 비용 가산항 ≥ 0 (admissibility 보호). 음수 비용/보너스 금지.
- 다중 배관: 성공 경로 집합은 쌍별로 셀 비공유.
- `scene.txt` round-trip 무손실.

---

## 4. 우선순위 규칙 (다중 배관)

| 값 | 정렬 기준 | 비고 |
|---|---|---|
| `longest` | 시작–끝 맨해튼 거리 긴 순(기본) | 어려운 것 먼저 |
| `shortest` | 짧은 순 | |
| `utility` | 유틸리티 라벨 그룹(이름순) → 그룹 내 긴 순 | 유틸별 묶음 |
| `original` | 입력 순서 | |

> 계획서의 **'직경 큰 순'은 직경 데이터 확보 시 추가**(현재 거리 기준). PoC/배관 직경을 DB에서 읽어오는 것이 Phase 2 데이터 모델 과제.

---

## 5. 검증된 베이스라인 + 회귀 기준

- 베이스라인 파라미터 1셋: `experiments/baseline_params.json` (cell 50 / w_turn 500 / w_clear 10 / clearance 2 / tier 없음). 3종 시나리오 모두 통과.
- 회귀 시나리오 지표(결정적):

| 시나리오 | 핵심 지표 |
|---|---|
| 01_single_empty (20³) | 길이 2850mm = 맨해튼(비율 1.000), 회전 2, 장애물 통과 0 |
| 02_single_obstacle (80³) | 길이 3950mm = 직선 3450mm × 1.145(±20% 내), 장애물 통과 0 |
| 03_multi_tier (120×120×60, 5배관) | 5/5 성공, 충돌 0, 총 28050mm |

- 실데이터: project 6 — 203/208(98%) 성공, 충돌 0. (실패 5건 = 혼잡 출발부 경합)

---

## 6. 알려진 한계 → Phase 3 이연

| 항목 | 현 상태(Phase 1) | Phase 3 계획 |
|---|---|---|
| 다중 배관 전역 최적화 | greedy sequential(후순위 경합 실패 측정만) | **rip-up & reroute / CBS** |
| 성능 | 순수 파이썬, weighted A* 가 120³에서 ~28s(1.33M 확장). 도메인 ≤120³ 권장 | **C++ + Boost.Heap, 평면배열 A***. Numba 는 단순 레퍼런스를 포크하므로 미채택 |
| 대규모 점유 압축 | Dense/BitPacked/Sparse(ROI 한정) | **OpenVDB**(옥트리/VDB) — 433m 스케일 |
| 충돌 검사 정밀도 | AABB 복셀화 | **메시 기반(FCL)** |
| 직경 기반 우선순위/이격 | 거리 기준 + `pipe_radius` 근사 | 실제 직경/이격 규칙 |
| weighted A* 과확장 | 정확하나 빈 공간서 동일비용 상태 폭증 | 휴리스틱 강화/jump-point 등 검토 |

---

## 7. Phase 2 명세화 / Phase 3 C++ 권고

1. **인터페이스 우선 동결**: §1 의 좌표 규약·`OccupancyMap` 질의 API·`RouteParams`·`scene.txt` v1 을 명세서에 그대로 옮긴다. 회귀 데이터(`tests/scenarios/`, `scene.txt`)를 C++ 결과 대조용 골든셋으로 재사용.
2. **1:1 포팅성 유지**: Python 레퍼런스의 자료구조(heap 항목, 상태=(셀,방향), 거리변환)를 단순·명시적으로 유지했으므로 C++ 매핑이 직접적. `dict g/closed` → 평면배열(선형 셀 인덱스×방향), `heapq` → Boost.Heap.
3. **admissibility 계약 보존**: 비용 가산항 ≥ 0 규칙을 C++ 에서도 단위 테스트로 강제.
4. **scene.txt 를 엔진 I/O 로 채택**: C++ 엔진이 같은 포맷을 읽고/쓰면 Python 실험과 결과를 직접 비교 가능(지표 허용범위 대조).
5. **검증 게이트**: Phase 3 C++ 가 3종 회귀 시나리오에서 Python 과 동일 지표(길이/회전/충돌/성공률)를 허용범위 내로 재현해야 한다.

---

## 8. 미해결 질문 (Open Questions)

- 배관 직경/이격 규칙의 데이터 출처(DB 필드?)와 우선순위 반영 방식.
- 단(段) 분리 정책: `w_tier` 수동 지정 vs 자동 단 할당.
- 혼잡 출발부(메인장비 PoC 밀집) 실패의 허용 기준 — Phase 3 rip-up 목표 성공률.
- 대규모(>120³) 시 ROI 분할/스트리밍 전략(OpenVDB 도입 전 임시안 필요 여부).
- `scene.txt` 에 배관 직경·재질·우선순위 등 메타 추가 시 v2 확장 규칙.

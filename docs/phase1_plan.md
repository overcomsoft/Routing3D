# Phase 1 — Python 알고리즘 실험 세부 개발계획서

> 상위 문서: [development_plan.md](development_plan.md) §3.1
> 작성일: 2026-05-27
> 버전 1.0
> 대상 디렉토리: [../python_experiments/](../python_experiments/)

## 목차

1. [목표와 비목표](#1-목표와-비목표)
2. [세부 작업 단계](#2-세부-작업-단계)
3. [모듈 / 디렉토리 구조](#3-모듈--디렉토리-구조)
4. [데이터 모델과 좌표 규약](#4-데이터-모델과-좌표-규약)
5. [회귀 시나리오와 베이스라인 파라미터](#5-회귀-시나리오와-베이스라인-파라미터)
6. [완료 기준 (Definition of Done)](#6-완료-기준-definition-of-done)
7. [작업 일정 (서브 마일스톤)](#7-작업-일정-서브-마일스톤)
8. [리스크와 대응](#8-리스크와-대응)

---

## 1. 목표와 비목표

### 1.1 목표
- 직교 A* 라우팅 알고리즘과 비용함수 파라미터를 **빠른 반복**으로 검증한다.
- Phase 2 명세화·Phase 3 C++ 엔진의 기준이 될 입출력 규약(`scene.txt`)을 **신규 설계·동결**한다.
- 대표 시나리오 3종에서 안정적으로 경로를 산출하는 베이스라인 파라미터 셋을 확보한다.
- 알고리즘·제약·우선순위 규칙을 Phase 2 명세화의 입력으로 정리한다.

> **단위 규약**: 본 계획서의 모든 길이·좌표는 **밀리미터(mm)** 기준이다. 셀 크기 기본값은 50mm이며 `RouteParams.cell_mm` 으로 설정 가능하다.

### 1.2 비목표 (이 단계에서 하지 않는 것)
- 1,000m 스케일 대규모 성능 최적화 → Phase 3 (OpenVDB / Boost.Heap / FCL).
- rip-up & reroute, CBS 등 다중 배관 전역 최적화 → Phase 3.
- 메시 기반 정밀 충돌검사(FCL) → Phase 3.
- C# 뷰어 연계 — 이 단계는 PyVista 즉석 렌더로 충분.

---

## 2. 세부 작업 단계

각 서브 단계는 **(a) 인터페이스 정의 → (b) 구현 → (c) 회귀 시나리오 통과** 순서로 진행한다. 인터페이스가 먼저 잡혀야 후속 단계가 병행 가능하다.

### 2.1 Step 1.1 — 점유맵 (Occupancy Map)

**산출물**: `occupancy.py`, 점유맵 생성·질의 단위 테스트.

| 항목 | 결정사항 |
|---|---|
| 표현 | 추상 `OccupancyMap` + 교체 가능한 **3개 백엔드**. A* 등 사용자 코드는 질의 인터페이스에만 의존 |
| 좌표 | `(i, j, k)` 정수 셀 인덱스, 원점 `origin`(mm), 셀 크기 `cell_mm`(mm, 기본 50) |
| 클리어런스 | `inflate(map, radius_cells, connectivity=6\|26)` dilation. 모든 백엔드 동일 결과 |
| 입력 (합성) | AABB 박스 리스트(mm) → 셀 voxelize. (메시 → voxelize 는 Phase 3) |
| 입력 (실데이터) | **PostgreSQL `TB_BIM_OBSTACLES`** 에서 MIN/MAX_X/Y/Z(mm) + OST_TYPE 로드 → 점유맵 구성 (`obstacle_db.py`) |
| 질의 | `is_blocked(cell) -> bool`, `in_bounds`, `bounds() -> (lo, hi)`, `to_world`/`to_cell` |

**저장 백엔드** (등간격 복셀 폭발 대응 — 동일 인터페이스, 다른 메모리 특성):

| 백엔드 | 셀당 | 질의 | 적합 |
|---|---|---|---|
| `DenseOccupancyMap` | 1 byte | O(1) 최속 | 작은 ROI, 기본값 |
| `BitPackedOccupancyMap` | 1 bit | 약간 느림 | 같은 메모리로 ~8배 큰 ROI |
| `SparseOccupancyMap` | set 엔트리 | 해시 | *점유 희박*할 때만 |

> **실측(점유율 25.6% 실데이터 영역, 50mm)**: Dense 625KB / BitPacked **78KB** / Sparse **23.6MB**.
> 바닥·기둥처럼 점유가 빽빽하면 Sparse 는 set 오버헤드로 Dense의 ~37배가 되므로 부적합.
> **권장**: ROI Dense(기본), 메모리 압박 시 BitPacked. 균일 블록의 진짜 압축(옥트리/VDB)은 Phase 3(OpenVDB).

**장애물 DB 로더** (`routing3d_py/obstacle_db.py`):

- `PgConnConfig` (localhost:5432 / AUTOROUTINGV7 / TB_BIM_OBSTACLES) → `load_obstacles(ost_types=, region=)` → `build_occupancy(cell_mm=, region=)`.
- 약 75,000행·~433×85×35m 규모이므로 **region(관심 영역)·OST_TYPE 필터 필수**. 전체 Dense 그리드화는 `max_cells` 한도로 차단.
- `OST_TYPE`(바닥/기둥/보 등)을 장애물에 보존하여 타입별 차등 클리어런스·충돌 검사에 활용.

**검증**: 단순 박스 점유 / 인플레이션 두께 / 경계 셀 처리 단위 테스트 + DB 로더(영역·타입 필터, 퇴화 박스 스킵, 셀 수 한도) 테스트(DB 연결 시).

**라우팅 씬 로더** (`routing3d_py/scene.py`) — SpaceAI 프로젝트 단위 데이터 환경:

- `space_project_map(project_id→source_file)` 로 프로젝트 식별, `SOURCE_FILE` 로 장애물·메인장비·종단객체 필터.
- **라우팅 작업(start PoC→end PoC)은 메인장비 `TB_BIM_EQUIPMENT.POC_LIST`(jsonb)** 에 들어 있음: 각 PoC = `{pocPosition(start), utility, utilityGroup, endPocs[].endPocPosition(end)}`.
- `load_scene(project_id)` → `RoutingScene`(장애물/장비/종단/작업/범위). `tasks_by_utility()` 로 유틸리티(`[그룹] 유틸`)별 그룹, `build_occupancy()` + `route_tasks()` 로 그룹 라우팅.
- ✅ 검증: project 6(CLEAN/WTNHJ03) = 장애물 983(원본 987·퇴화 4 제외), PoC 페어 **208**, 유틸 **21종** (SpaceAI 뷰어와 일치). `[Gas] PN2` 9개 A* 라우팅 9/9 성공 렌더 확인.

---

### 2.2 Step 1.2 — 직교 A* 탐색

**산출물**: `astar.py`, 단일 src→dst 경로 산출.

| 항목 | 결정사항 |
|---|---|
| 이웃 | 6방향(±X, ±Y, ±Z) 직교만. 대각선 금지 |
| 우선순위 큐 | `heapq` + `(f, counter, node)` 튜플. tie-break은 삽입 순서 |
| closed 집합 | `set[(i,j,k)]` (1차). 대용량에서 `dict`로 g-score 캐싱 |
| 휴리스틱 | 맨해튼 거리(셀 수) × `cell_mm`. 직교 라우팅에서 admissible & consistent |
| 종료 | dst 셀 pop 시점에 종료. 미발견은 `None` 반환 |
| 경로 복원 | `came_from` 역추적 → `list[cell]` |
| 방문맵 export | 옵션 플래그로 visited 셀 누적 (시각화·디버그용) |

**구현 원칙**: Phase 3 C++ 포팅이 1:1 대응되도록 함수 분리·자료구조 선택을 단순·명시적으로 유지한다. 컴프리헨션·제너레이터·메타프로그래밍 등 Python 고유 패턴은 회귀 통과 이후에만 도입한다.

**검증**: 빈 공간 직선 경로, L자 우회, 막다른 골목, 도달불가 케이스. ✅ 구현 완료 (`astar.py`, `AStarResult`: success/path/length_mm/turns/expanded_nodes/visited/elapsed_ms). 세 백엔드 동일 결과 + 실DB 영역 경로 렌더 검증.

---

### 2.3 Step 1.3 — 비용함수

**산출물**: `cost.py`, 파라미터 셋 dataclass `RouteParams`.

모든 비용은 **mm 단위**로 표현한다. 휴리스틱과 비교 가능해야 하므로 step_cost 와 동일 단위를 사용한다.

| 비용 요소 | 기호 | 적용 시점 | 비고 |
|---|---|---|---|
| 이동 비용 | `step_cost = cell_mm` | 모든 step | 셀 1칸 이동당 50mm (기본) |
| Turn penalty | `w_turn` (mm) | 이전 방향과 다를 때 가산 | 직각 회전 최소화 |
| 클리어런스 보너스 | `-w_clear * margin_cells` (mm) | 셀의 빈 여유 셀 수에 비례 감산 | 벽에서 멀어지도록 유도. 보너스 ≤ step_cost 유지(admissibility) |
| 단(段) 분리 가중치 | `w_tier(z)` (mm) | z 레벨별 가산 | 배관 단 강제 분리 |
| 기존 경로 회피 | `w_occupied_by_pipe(p)` (mm) | 이미 깔린 다른 배관 셀에 가산 | 다중 배관 단계에서 활용 |

**파라미터 셋 (초기 베이스라인 후보 — Step 1.3 에서 튜닝)**

```python
@dataclass
class RouteParams:
    cell_mm: float = 50.0         # 셀 1칸 = 50mm (설정 가능)
    w_turn: float = 500.0         # 회전당 500mm 상당 (= 셀 10칸)
    w_clear: float = 10.0         # 여유 셀당 -10mm (admissibility 보호)
    clearance_radius: int = 2     # 2셀 = 100mm 인플레이션
    w_tier: dict[int, float] = ... # 단별 가산값 (mm)
```

**검증 방식**: 동일 시나리오에서 파라미터를 grid-search 하며 (총 길이, 회전 수, 평균 클리어런스) 지표를 비교. 결과는 `experiments/<scenario>/results.csv` 로 누적.

✅ **구현 완료** (`cost.py`: `RouteParams`/`clearance_map`/`CostModel`, `astar.py`: `astar_weighted`).
- **클리어런스는 보너스(감산)가 아니라 근접 페널티(가산)로 구현** — 감산은 한 칸 비용을 cell_mm 미만으로 만들어 맨해튼 휴리스틱의 admissibility(A* 최적성)를 깨므로. 모든 항이 ≥0 이라 휴리스틱이 admissible & consistent 유지.
- **turn penalty 를 위해 탐색 상태를 (셀, 진입방향)으로 확장**. 같은 셀도 진입 방향에 따라 이후 비용이 다르기 때문.
- 검증: 합성 장면에서 기본 vs 클리어런스 경로 = 길이 1900→2300mm, 최소 클리어런스 1→5셀(장애물 회피 폭 증가) 확인.

---

### 2.4 Step 1.4 — 다중 배관 전략

**산출물**: `multi_route.py`, 순차 라우팅 + 우선순위 정책.

| 정책 | 설명 | 이 단계 채택 여부 |
|---|---|---|
| 순차 라우팅 (sequential) | 우선순위 순으로 한 배관씩 깔고, 깔린 셀을 점유로 추가 | **채택** (베이스라인) |
| 우선순위 규칙 | (1) 직경 큰 순 (2) 시작–끝 거리 긴 순 (3) 단(段) 낮은 순 | 비교 실험 |
| rip-up & reroute | 충돌 시 일부 배관을 뽑고 재라우팅 | **Phase 3 이연** |
| CBS (Conflict-Based Search) | 다중 에이전트 충돌 해소 | **Phase 3 이연** |

**검증**: 5~10개 배관 시나리오에서 (성공률, 총 길이 합, 실패한 배관 수) 지표 측정.

✅ **구현 완료** (`multi_route.py`: `route_sequential`/`order_tasks`/`MultiRouteResult`/`PipeResult`, `OccupancyMap.copy()`).

- **충돌 회피 메커니즘**: 장애물 점유맵의 작업용 사본을 만들고, 한 배관을 라우팅(astar_weighted)할 때마다 성공 경로 셀(+`pipe_radius` 팽창)을 사본에 점유로 추가 → 다음 배관이 피함. 성공 경로끼리 셀 비공유(단위 테스트로 보장).
- **우선순위**: longest(기본)/shortest/utility/original. (계획서의 '직경 큰 순'은 직경 데이터 확보 시 추가.)
- **혼잡 출발부 실패는 측정 대상**: 메인장비 면에 PoC 가 밀집해 후순위 배관이 막힐 수 있음. rip-up & reroute / CBS 로 해소하는 것은 계획대로 **Phase 3 이연**. 본 단계는 baseline sequential 의 성공률을 측정하는 것이 목적.
- 사용: `routing3d_py.scene` CLI `--multi --priority --pipe-radius`, 또는 `route_sequential(occ, scene.tasks, params)`.
- 검증: project 6(208 배관) 전체 순차 라우팅 — 성공률/유틸리티별 성공 측정 + 유틸리티 색 렌더.

---

### 2.5 Step 1.5 — I/O와 시각화

**산출물**: `scene_io.py`, `viz.py`, 사용 예제 노트북.

| 항목 | 결정사항 |
|---|---|
| `scene.txt` 포맷 | **신규 정의**. 점유·방문·경로 3개 레이어. 단위 mm, 셀 크기 헤더 명시 |
| 입력 | 박스 장애물 리스트(mm) + 배관 src/dst(mm) + 파라미터 JSON |
| 출력 | 경로 셀 리스트, 방문 셀 리스트, 성능 지표 (시간·확장 노드 수) |
| 시각화 | PyVista 1차 (회전·줌). Plotly 2차 (HTML 공유용) |
| 레이어 토글 | 점유 / 방문 / 경로 / 시작·종료 마커 |

**검증**: (1) `scene.txt` round-trip 무손실 (write → read → write 결과 동일). (2) PyVista 렌더에서 점유·경로·방문 레이어가 토글 가능. (3) 회귀 시나리오 3종 모두에서 입출력 파일 생성 성공.

---

### 2.6 Step 1.6 — 핫스팟 가속 (선택)

**산출물**: `astar_numba.py` (있을 시).

- 프로파일링 결과 A* 루프가 전체 시간의 60%를 초과할 때만 도입.
- Numba `@njit` 적용 대상: 휴리스틱 계산, 이웃 생성, closed 집합 lookup.
- 도입 효과가 2배 이하이면 채택하지 않는다 (가독성 손해 < 이득).

---

### 2.7 Step 1.7 — 회귀 시나리오 고정

**산출물**: `tests/scenarios/`, `pytest` 회귀 세트.

대표 시나리오 3종:
1. **단일 배관 / 빈 공간** — 알고리즘 sanity check.
2. **단일 배관 / 장애물 우회** — 비용함수 효과 확인.
3. **다중 배관 (5개) / 단 분리** — 다중 라우팅·우선순위 정책 검증.

각 시나리오는 `input.json` + `expected_metrics.json`(허용 범위 포함)으로 고정.

---

## 3. 모듈 / 디렉토리 구조

```
python_experiments/
├── README.md
├── pyproject.toml                 # 또는 src layout 미사용 시 생략
├── routing3d_py/
│   ├── __init__.py
│   ├── occupancy.py               # Step 1.1
│   ├── obstacle_db.py             # 장애물 DB 로더 (PostgreSQL → 점유맵)
│   ├── astar.py                   # Step 1.2 (구현 완료)
│   ├── cost.py                    # Step 1.3
│   ├── scene.py                   # SpaceAI 프로젝트 라우팅 씬 로더
│   ├── multi_route.py             # Step 1.4 (구현 완료)
│   ├── scene_io.py                # Step 1.5
│   ├── viz.py                     # 3D 점유맵 시각화 (PyVista, 구현 완료)
│   └── astar_numba.py             # Step 1.6 (선택)
├── experiments/
│   ├── 01_single_empty/
│   ├── 02_single_obstacle/
│   └── 03_multi_tier/
├── notebooks/
│   └── viz_demo.ipynb
└── tests/
    ├── test_occupancy.py
    ├── test_astar.py
    ├── test_cost.py
    ├── test_multi_route.py
    └── scenarios/
        ├── 01_single_empty/
        ├── 02_single_obstacle/
        └── 03_multi_tier/
```

---

## 4. 데이터 모델과 좌표 규약

| 항목 | 규약 |
|---|---|
| 단위 | **밀리미터(mm)**, 라디안 없음 (직교만) |
| 셀 크기 | `cell_mm` (float, mm). 기본값 `50.0`, `RouteParams.cell_mm` 으로 설정 |
| 셀 인덱스 | `(i, j, k)` ∈ ℤ³, 원점 `origin`(mm) 기준 |
| 셀↔월드 | `world_mm = origin + (cell + 0.5) * cell_mm` (셀 중심 기준) |
| 방향 | 6-이웃: `(±1,0,0), (0,±1,0), (0,0,±1)` |
| z축 의미 | 위로 갈수록 +z. 단(段) 분리는 z 레벨 정수로 표현 |
| 경로 표현 | `list[(i,j,k)]` — 인접 셀 간 단일 step 보장 |

**불변식 (invariants)**:
- 모든 경로 셀은 점유맵에서 `False` (장애 없음).
- 경로의 연속 셀은 6-이웃 관계.
- 시작·종료 셀은 점유맵 범위 내에 있다.

---

## 5. 회귀 시나리오와 베이스라인 파라미터

기본 셀 크기 50mm 기준. 실제 크기 = (셀 수) × 50mm.

| 시나리오 | 도메인 셀 수 | 실제 크기 (mm) | 배관 수 | 핵심 검증 |
|---|---|---|---|---|
| 01_single_empty | 50 / 50 / 50 | 2500 / 2500 / 2500 | 1 | 직선 경로 = 맨해튼 거리 |
| 02_single_obstacle | 80 / 80 / 80 | 4000 / 4000 / 4000 | 1 | 우회 후 길이가 직선 ±20% 이내 |
| 03_multi_tier | 120 / 120 / 60 | 6000 / 6000 / 3000 | 5 | 5개 모두 성공, 충돌 0 |

지표 정의 (`expected_metrics.json`):

- `length_mm`: 경로 총 길이 (mm)
- `turns`: 회전 횟수
- `expanded_nodes`: A* 확장 노드 수
- `elapsed_ms`: 실행 시간 (참고용, 회귀 통과 기준 아님)
- `success`: 성공 여부

---

## 6. 완료 기준 (Definition of Done)

Phase 1 완료 = 아래 항목 **전부** 충족.

- [ ] 3개 회귀 시나리오 모두 `pytest` 통과.
- [ ] `scene.txt` 입출력 규약 동결 (round-trip 무손실 + PyVista 렌더 확인).
- [ ] 베이스라인 `RouteParams` 1셋 확정 (`experiments/baseline_params.json`).
- [ ] 다중 배관 시나리오에서 성공률 ≥ 90%.
- [ ] Phase 2 명세화를 위한 알고리즘·제약·우선순위 정리 노트 1편.

---

## 7. 작업 일정 (서브 마일스톤)

자원에 따라 조정. 순서·의존성 기준의 권장 순.

| 순 | 작업 | 의존성 |
|---|---|---|
| M1 | Step 1.1 점유맵 + 단위 테스트 | — |
| M2 | Step 1.2 직교 A* + 단위 테스트 | M1 |
| M3 | Step 1.5 `scene.txt` 입출력 + PyVista 데모 | M1, M2 |
| M4 | Step 1.3 비용함수 + 파라미터 grid-search 도구 | M2 |
| M5 | 회귀 시나리오 1, 2 작성 및 통과 | M3, M4 |
| M6 | Step 1.4 다중 배관 순차 라우팅 | M4 |
| M7 | 회귀 시나리오 3 작성 및 통과 | M6 |
| M8 | (선택) Step 1.6 Numba 가속 | M5 |
| M9 | Phase 2 입력 노트 정리 (알고리즘·제약·우선순위) | M5, M7 |

---

## 8. 리스크와 대응

| 리스크 | 영향 | 대응 |
|---|---|---|
| 비용함수·휴리스틱 admissibility 깨짐 | A* 최적성 손실 | 클리어런스 보너스 ≤ step_cost 강제. 단위 테스트로 확인 |
| 비용함수 파라미터 과적합 | 신규 시나리오에서 실패 | grid-search 결과를 시나리오 교차검증 (한 셋이 3개 시나리오 모두 만족해야 채택) |
| 다중 배관 순차 라우팅 한계 | 일부 배관 실패 | Phase 1 에서는 우선순위 정책 비교까지만. rip-up 은 Phase 3 |
| Python 성능으로 대형 시나리오 실행 불가 | 검증 범위 축소 | 도메인을 120³ 셀 이하로 제한. 대형은 Phase 3 C++ 엔진에서 검증 |
| 시각화 라이브러리 의존 부담 | 환경 구축 마찰 | PyVista 1순위, 안 되면 Plotly fallback. Matplotlib 2D 단면도는 항상 가능 |
| Phase 3 C++ 포팅 시 결과 불일치 | 회귀 신뢰성 저하 | Phase 1 구현을 1:1 포팅 가능한 단순 자료구조로 유지. 동일 시나리오에서 지표 허용범위 비교 |

---

본 세부 계획서는 진행 상황에 따라 갱신한다. **다음 즉시 실행 항목** = Step 1.1 점유맵 구현 (`python_experiments/routing3d_py/occupancy.py`) + 단위 테스트.

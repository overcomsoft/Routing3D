# Phase 1 — Python 알고리즘 실험

목적: 성능 부담 없이 알고리즘과 파라미터를 빠르게 반복 검증한다. 핵심 지표는 **반복 속도**.

상세 계획: [../docs/phase1_plan.md](../docs/phase1_plan.md)

단위: **밀리미터(mm)**, 기본 셀 크기 50mm (`RouteParams.cell_mm`).

## 구조

```
python_experiments/
├── conftest.py            # pytest 루트 (routing3d_py import 가능하게 함)
├── routing3d_py/          # 알고리즘 패키지
│   ├── occupancy.py       # Step 1.1 — 점유맵 (구현 완료)
│   ├── obstacle_db.py     # 장애물 DB 로더 (PostgreSQL TB_BIM_OBSTACLES, 구현 완료)
│   ├── astar.py           # Step 1.2 — 직교 A* (구현 완료)
│   ├── cost.py            # Step 1.3 — 비용함수 (예정)
│   ├── multi_route.py     # Step 1.4 — 다중 배관 (예정)
│   ├── scene_io.py        # Step 1.5 — scene.txt I/O (예정)
│   └── viz.py             # 3D 점유맵 시각화 (PyVista, 구현 완료)
└── tests/                 # 회귀 테스트 (pytest)
```

## 의존성 설치

```powershell
# 프로젝트 루트(Routing3D)에서
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install "numpy>=1.26" "pytest>=8.0" "psycopg2-binary>=2.9.9"   # 핵심 (Step 1.1~1.4 + DB 로더)
pip install "pyvista>=0.43"   # 3D 시각화
```

## 테스트 실행

```powershell
# python_experiments/ 에서
..\.venv\Scripts\python.exe -m pytest -v
# DB 통합 테스트 제외(연결 불가 환경)
..\.venv\Scripts\python.exe -m pytest -v -m "not db"
```

## 점유맵 백엔드 (등간격 복셀 폭발 대응)

추상 `OccupancyMap` 뒤에 3개 백엔드를 두고 상황에 맞게 고른다. A* 등은 질의 인터페이스(`is_blocked`/`in_bounds`/`bounds`/`to_world`)에만 의존하므로 교체 자유롭다.

| 백엔드 | 셀당 | 적합 |
|---|---|---|
| `DenseOccupancyMap` (기본) | 1 byte | 작은 ROI, 최속 질의 |
| `BitPackedOccupancyMap` | 1 bit | 같은 메모리로 ~8배 큰 ROI |
| `SparseOccupancyMap` | set 엔트리 | *점유 희박*할 때만 |

실측(점유율 25.6% 실데이터, 50mm): Dense 625KB / **BitPacked 78KB** / Sparse 23.6MB.
바닥·기둥처럼 점유가 빽빽하면 Sparse 는 오히려 Dense의 ~37배 → 부적합. 대규모 균일 블록 압축(옥트리/VDB)은 Phase 3(OpenVDB).

```python
from routing3d_py import build_occupancy, BitPackedOccupancyMap
result = build_occupancy(obs, cell_mm=50, region=region, backend=BitPackedOccupancyMap)
```

## 3D 시각화 (PyVista)

점유맵을 OST_TYPE별 색상 복셀로 3D 렌더한다. 점유 셀만 머지 렌더하여 수십만 셀도 가볍다.

```powershell
# DB 영역을 OST_TYPE별 색으로 렌더 → 스크린샷 PNG (창 없이)
..\.venv\Scripts\python.exe -m routing3d_py.viz `
    --region 195000 8000 14000 205000 12000 16000 --cell-mm 50 --screenshot out/occupancy.png

# 인터랙티브 창으로 직접 회전/줌
..\.venv\Scripts\python.exe -m routing3d_py.viz --region 195000 8000 14000 205000 12000 16000 --show

# 브라우저용 인터랙티브 HTML
..\.venv\Scripts\python.exe -m routing3d_py.viz --region 195000 8000 14000 205000 12000 16000 --html out/occupancy.html
```

```python
from routing3d_py.viz import render_occupancy
render_occupancy(result.occupancy, screenshot="out/occ.png")          # 단일 맵
render_occupancy({"floors": occ_f, "columns": occ_c}, show=True)      # 레이어별 색상
```

## 장애물 DB 로더 (PostgreSQL → 점유맵)

플랜트 장애물(`TB_BIM_OBSTACLES`)을 읽어 점유맵을 구성한다. 단위 mm, OST_TYPE(바닥/기둥/보 등) 보존.

```powershell
# DB 통계 (행 수 / OST_TYPE 분포 / 전체 범위)
..\.venv\Scripts\python.exe -m routing3d_py.obstacle_db --stats

# 특정 타입·영역(mm)을 50mm 셀 점유맵으로 구성
..\.venv\Scripts\python.exe -m routing3d_py.obstacle_db `
    --types OST_Columns OST_Floors `
    --region 195000 8000 14000 205000 12000 16000 --cell-mm 50
```

```python
# 코드에서 사용
from routing3d_py import PgConnConfig, load_obstacles, build_occupancy
region = ((195000, 8000, 14000), (205000, 12000, 16000))   # mm
obs = load_obstacles(PgConnConfig(), region=region, ost_types=["OST_Columns", "OST_Floors"])
result = build_occupancy(obs, cell_mm=50, region=region)
occ = result.occupancy        # OccupancyMap — A* 탐색 입력
print(result.summary())
```

> 전체 영역은 ~433×85×35m, 약 75,000행. **반드시 region/타입으로 좁혀** 사용한다
> (전체 Dense 그리드화는 `max_cells` 한도로 차단됨).

## 직교 A* 경로 탐색 (Step 1.2) + 비용함수 (Step 1.3)

점유맵 위에서 6방향 직교 최단 경로를 찾는다. 휴리스틱은 맨해튼 거리×cell_mm.

- **기본**(`astar`): 균일 이동비용(셀=cell_mm).
- **비용함수**(`astar_weighted` + `RouteParams`): turn penalty(회전 최소화) / 클리어런스 근접 페널티(벽 회피) / 단(段) 분리. turn penalty 때문에 탐색 상태가 (셀, 진입방향)으로 확장된다.
  - 클리어런스는 admissibility 보호를 위해 **보너스가 아닌 가산 페널티**로 구현 → 휴리스틱이 admissible & consistent 유지(최적성 보존).

```powershell
# 기본 A* (CLI 에 --w-turn/--w-clear 없으면 균일 비용)
..\.venv\Scripts\python.exe -m routing3d_py.astar `
    --region 195000 8000 14000 205000 12000 16000 --cell-mm 50 `
    --start 195300 8300 14775 --goal 204700 11700 14775 --screenshot out/route.png

# 비용함수 A* (회전·클리어런스 페널티) — --inflate N 은 하드 클리어런스, --show 인터랙티브
..\.venv\Scripts\python.exe -m routing3d_py.astar `
    --region 195000 8000 14000 205000 12000 16000 --cell-mm 50 `
    --start 195300 8300 14775 --goal 204700 11700 14775 `
    --w-turn 800 --w-clear 40 --clearance 3 --screenshot out/route_cost.png
```

```python
from routing3d_py import astar, astar_world, astar_weighted, RouteParams
res = astar(occ, (6, 6, 15), (194, 74, 15))            # 기본(균일)
res = astar_world(occ, (195300, 8300, 14775), (204700, 11700, 14775))  # 월드 mm
res = astar_weighted(occ, start, goal,                 # 비용함수
                     RouteParams(w_turn=500, w_clear=10, clearance_radius=2))
print(res.summary())   # 성공/길이/비용/회전/확장노드/시간
path = res.path        # list[(i,j,k)] — viz render_occupancy(path=...) 로 렌더
```

## 라우팅 씬 데이터 환경 (SpaceAI 프로젝트 단위)

`space_project_map(project_id→source_file)` 로 프로젝트를 식별하고, `SOURCE_FILE` 필터로
장애물(`TB_BIM_OBSTACLES`)·메인장비(`TB_BIM_EQUIPMENT`)·종단객체(`TB_DUCT_LATERAL`)를 읽는다.
**라우팅 작업(start PoC→end PoC)은 메인장비의 `POC_LIST`(jsonb)** 안에 있고, 유틸리티
(`[utilityGroup] utility`, 예 `[Gas] PA`)별로 묶는다.

```powershell
# 프로젝트 목록 / 씬 요약(장애물·장비·종단·PoC페어·유틸리티·범위)
..\.venv\Scripts\python.exe -m routing3d_py.scene --list
..\.venv\Scripts\python.exe -m routing3d_py.scene --project 6

# start PoC→end PoC 를 유틸리티별 색으로 렌더 (직선 연결)
..\.venv\Scripts\python.exe -m routing3d_py.scene --project 6 --screenshot out/scene.png

# 한 유틸리티만 실제 A* 경로로 라우팅해서 렌더
..\.venv\Scripts\python.exe -m routing3d_py.scene --project 6 --route --utility "[Gas] PN2" --cell-mm 100 --show
```

```python
from routing3d_py import load_scene, route_tasks, RouteParams
scene = load_scene(project_id=6)            # RoutingScene
print(scene.summary())                       # 장애물/장비/종단/작업/유틸 요약
occ = scene.build_occupancy(cell_mm=100).occupancy
for util, tasks in scene.tasks_by_utility().items():   # 유틸리티별 그룹
    routed = route_tasks(occ, tasks, RouteParams(cell_mm=100, w_turn=300))
```

## 산출물 목표

- 검증된 알고리즘 파라미터 셋
- 실험 노트
- scene.txt 호환 입출력
- 비교 기준 (baseline) 데이터

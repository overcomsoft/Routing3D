"""라우팅 씬 데이터 환경 단위/통합 테스트 — scene
================================================================================

[실행 명령어]  (프로젝트 루트 또는 python_experiments/ 에서)
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_scene.py -v
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_scene.py -v -m "not db"

[검증 범위]
  - 순수 로직(DB 불필요): 유틸리티 라벨/색 배정, 작업 그룹핑, 작업→A* 라우팅, polylines.
  - DB 통합(@db): list_projects / load_scene(project 6) — 연결 불가 시 자동 skip.
================================================================================
"""

import pytest

from routing3d_py.cost import RouteParams
from routing3d_py.obstacle_db import PgConnConfig
from routing3d_py.occupancy import AABB, DenseOccupancyMap
from routing3d_py.scene import (
    EndPoc,
    MainEquipment,
    ProjectInfo,
    RouteTask,
    RoutingScene,
    StartPoc,
    list_projects,
    load_scene,
    route_tasks,
    scene_polylines,
    utility_colors,
    utility_label,
)


# ----------------------------------------------------------- 헬퍼/모델 (DB 불필요)

def test_utility_label():
    assert utility_label("Gas", "PA") == "[Gas] PA"
    assert utility_label(None, None) == "[?] ?"


def test_startpoc_and_task_label():
    poc = StartPoc("id", "POC1", (0, 0, 0), "PA", "Gas", True, ())
    assert poc.utility_label == "[Gas] PA"
    t = RouteTask((0, 0, 0), (1, 1, 1), "NW", "Water", "POC1", "END1", "g")
    assert t.utility_label == "[Water] NW"


def test_utility_colors_deterministic():
    labels = ["[Gas] PA", "[Water] NW", "[Gas] PN2"]
    c1 = utility_colors(labels)
    c2 = utility_colors(list(reversed(labels)))
    assert c1 == c2                      # 정렬 기반 → 순서 무관 동일
    assert len(set(c1.values())) == 3    # 서로 다른 색


def _make_scene(tasks):
    return RoutingScene(
        project=ProjectInfo(1, "f.json", "P", "E"),
        bounds_lo=(0, 0, 0), bounds_hi=(500, 500, 100),
        obstacles=[], main_equipment=[], terminals=[], tasks=tasks,
    )


def test_tasks_by_utility_and_counts():
    tasks = [
        RouteTask((0, 0, 0), (1, 0, 0), "PA", "Gas", "a", "b", "g"),
        RouteTask((0, 0, 0), (2, 0, 0), "PA", "Gas", "a", "c", "g"),
        RouteTask((0, 0, 0), (3, 0, 0), "NW", "Water", "a", "d", "g"),
    ]
    scene = _make_scene(tasks)
    groups = scene.tasks_by_utility()
    assert len(groups["[Gas] PA"]) == 2
    assert len(groups["[Water] NW"]) == 1
    counts = scene.utility_counts()
    assert list(counts.items())[0] == ("[Gas] PA", 2)   # 내림차순


def test_scene_build_occupancy():
    scene = RoutingScene(
        project=ProjectInfo(1, "f", "P", "E"),
        bounds_lo=(0, 0, 0), bounds_hi=(500, 500, 100),
        obstacles=[], main_equipment=[], terminals=[], tasks=[],
    )
    # 장애물 없는 빈 씬도 점유맵 구성 가능(점유 0).
    res = scene.build_occupancy(cell_mm=50)
    assert res.occupancy.shape == (10, 10, 2)
    assert res.occupancy.count_blocked() == 0


def test_route_tasks_on_synthetic():
    occ = DenseOccupancyMap(shape=(10, 10, 1), cell_mm=50)
    # 월드 (25,25,25)→셀(0,0,0), (475,25,25)→셀(9,0,0)
    tasks = [RouteTask((25, 25, 25), (475, 25, 25), "PA", "Gas", "a", "b", "g")]
    routed = route_tasks(occ, tasks, RouteParams(cell_mm=50, w_turn=0))
    assert len(routed) == 1
    assert routed[0].result.success
    assert routed[0].result.path[0] == (0, 0, 0)
    assert routed[0].result.path[-1] == (9, 0, 0)


def test_route_tasks_snaps_blocked_endpoint():
    occ = DenseOccupancyMap(shape=(10, 10, 1), cell_mm=50)
    occ.block_cell((0, 0, 0))   # 시작 셀 점유 → 이웃으로 스냅돼야 함
    tasks = [RouteTask((25, 25, 25), (475, 25, 25), "PA", "Gas", "a", "b", "g")]
    routed = route_tasks(occ, tasks, RouteParams(cell_mm=50, w_turn=0), snap_to_free=2)
    assert routed[0].result.success    # 스냅 덕분에 성공


def test_scene_polylines_straight():
    tasks = [RouteTask((25, 25, 25), (475, 25, 25), "PA", "Gas", "a", "b", "g")]
    scene = _make_scene(tasks)
    occ = scene.build_occupancy(cell_mm=50).occupancy
    lines = scene_polylines(scene, occ, routed=None)   # 직선
    assert len(lines) == 1
    pts, color, label = lines[0]
    assert pts == [(25, 25, 25), (475, 25, 25)]
    assert label == "[Gas] PA"


def test_scene_polylines_routed():
    occ = DenseOccupancyMap(shape=(10, 10, 1), cell_mm=50)
    tasks = [RouteTask((25, 25, 25), (475, 25, 25), "PA", "Gas", "a", "b", "g")]
    routed = route_tasks(occ, tasks, RouteParams(cell_mm=50, w_turn=0))
    scene = _make_scene(tasks)
    lines = scene_polylines(scene, occ, routed=routed)
    assert len(lines) == 1
    pts, color, label = lines[0]
    assert len(pts) == 10           # 셀 0..9 → 10점
    assert label == "[Gas] PA"


# ----------------------------------------------------------- DB 통합 (@db)

@pytest.fixture(scope="module")
def db_config():
    config = PgConnConfig.from_env()
    try:
        conn = config.connect()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"PostgreSQL 연결 불가 — skip ({e})")
    conn.close()
    return config


@pytest.mark.db
def test_db_list_projects(db_config):
    projects = list_projects(db_config)
    assert len(projects) > 0
    ids = [p.project_id for p in projects]
    assert 6 in ids


@pytest.mark.db
def test_db_load_scene_project6(db_config):
    scene = load_scene(db_config, 6)
    assert scene.project.project_id == 6
    assert "WTNHJ03" in (scene.project.equipment_code or "") or \
           "WTNHJ03" in scene.project.source_file
    # 뷰어 확인값: 장애물 987(원본) — load_scene 은 퇴화 박스 4개 제외 → 983.
    assert len(scene.obstacles) == 983
    # PoC 페어 208, 유틸 21종 (뷰어와 일치)
    assert len(scene.tasks) == 208
    assert len(scene.utility_counts()) == 21
    # 공간 범위(mm) 대략 일치
    assert scene.bounds_lo[0] == pytest.approx(183079.3, abs=1)
    assert scene.bounds_hi[2] == pytest.approx(17499.9, abs=1)


@pytest.mark.db
def test_db_scene_utility_grouping(db_config):
    scene = load_scene(db_config, 6)
    counts = scene.utility_counts()
    # 알려진 분포 일부 검증
    assert counts.get("[Gas] PA") == 18
    assert counts.get("[Exhaust] ACID") == 17
    assert counts.get("[UPW] UPW_S") == 42

"""scene.txt 입출력 단위 테스트 — Phase 1 Step 1.5
================================================================================

[실행 명령어]  (프로젝트 루트 또는 python_experiments/ 에서)
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_scene_io.py -v

[검증 범위]
  - round-trip 무손실: write → read → write 결과가 문자열로 동일.
  - 구조 동일성: grid/params/obstacles/tasks/results 가 역직렬화 후 원본과 == .
  - None(\\N) 과 빈 문자열 보존, 공백 포함 이름 보존.
  - occupancy_from_doc 가 점유 레이어(장애물)를 동일 점유 셀 수로 재구성.
  - w_tier(단 분리) 라운드트립, 결과 없는 씬, 버전 불일치 예외.
================================================================================
"""

import pytest

from routing3d_py import (
    AABB,
    DenseOccupancyMap,
    Obstacle,
    RouteParams,
    RouteTask,
    SceneDoc,
    astar_weighted,
    doc_from_scene,
    dumps_scene,
    loads_scene,
    occupancy_from_doc,
    read_scene,
    write_scene,
)


def _sample_doc(with_result=True, with_visited=True):
    """합성 SceneDoc 1개 생성(장애물 1, 작업 1, 선택적으로 결과 포함)."""
    cell_mm = 50.0
    shape = (10, 10, 3)
    origin = (0.0, 0.0, 0.0)
    params = RouteParams(cell_mm=cell_mm, w_turn=300.0, w_clear=10.0,
                         clearance_radius=2, w_tier={1: 200.0})

    obstacles = [
        Obstacle(object_id="G1", name="col 1", ost_type="OST_Columns",
                 ddworks_type=None,  # None → \N 로 보존되어야 함
                 min_xyz=(100.0, 100.0, 0.0), max_xyz=(200.0, 200.0, 150.0)),
        Obstacle(object_id="G2", name="", ost_type="OST_Floors",
                 ddworks_type="slab",  # 빈 문자열 name 과 None(ddworks)을 구분 검증
                 min_xyz=(0.0, 0.0, 0.0), max_xyz=(500.0, 500.0, 50.0)),
    ]
    tasks = [
        RouteTask(start_mm=(25.0, 275.0, 75.0), end_mm=(475.0, 275.0, 75.0),
                  utility="PA", utility_group="Gas",
                  start_name="POC 1", end_name=None, end_instance_guid="IG1"),
    ]

    results = [None]
    if with_result:
        occ = DenseOccupancyMap(shape, origin, cell_mm)
        occ.add_box(AABB(obstacles[0].min_xyz, obstacles[0].max_xyz))
        s = occ.to_cell(tasks[0].start_mm)
        g = occ.to_cell(tasks[0].end_mm)
        results = [astar_weighted(occ, s, g, params, collect_visited=with_visited)]

    return SceneDoc(cell_mm=cell_mm, origin=origin, shape=shape, params=params,
                    obstacles=obstacles, tasks=tasks, results=results)


# ------------------------------------------------------------ round-trip 무손실

def test_roundtrip_text_identical():
    doc = _sample_doc()
    t1 = dumps_scene(doc)
    doc2 = loads_scene(t1)
    t2 = dumps_scene(doc2)
    assert t1 == t2          # write → read → write 결과 동일(무손실)


def test_roundtrip_struct_equal():
    doc = _sample_doc()
    doc2 = loads_scene(dumps_scene(doc))
    assert doc2.cell_mm == doc.cell_mm
    assert doc2.origin == doc.origin
    assert doc2.shape == doc.shape
    assert doc2.params == doc.params
    assert doc2.obstacles == doc.obstacles
    assert doc2.tasks == doc.tasks
    assert doc2.results == doc.results


def test_file_write_read(tmp_path):
    doc = _sample_doc()
    p = tmp_path / "scene.txt"
    write_scene(str(p), doc)
    doc2 = read_scene(str(p))
    assert dumps_scene(doc2) == dumps_scene(doc)


# ------------------------------------------------------------ None / 공백 보존

def test_null_and_empty_preserved():
    doc = _sample_doc(with_result=False)
    doc2 = loads_scene(dumps_scene(doc))
    # None 은 None 으로, 빈 문자열은 빈 문자열로 구분되어 복원되어야 한다.
    assert doc2.obstacles[0].ddworks_type is None
    assert doc2.obstacles[1].name == ""
    assert doc2.obstacles[1].ddworks_type == "slab"
    assert doc2.tasks[0].end_name is None
    # 공백 포함 이름 보존.
    assert doc2.obstacles[0].name == "col 1"
    assert doc2.tasks[0].start_name == "POC 1"


# ------------------------------------------------------------ 결과 레이어

def test_result_path_and_visited_roundtrip():
    doc = _sample_doc(with_result=True, with_visited=True)
    doc2 = loads_scene(dumps_scene(doc))
    r0, r1 = doc.results[0], doc2.results[0]
    assert r1.success == r0.success
    assert r1.path == r0.path
    assert r1.visited == r0.visited
    assert r1.length_mm == r0.length_mm
    assert r1.turns == r0.turns
    assert r1.expanded_nodes == r0.expanded_nodes
    assert r1.cost_mm == r0.cost_mm


def test_no_results_section():
    doc = _sample_doc(with_result=False)
    text = dumps_scene(doc)
    assert "[results]" not in text
    doc2 = loads_scene(text)
    assert doc2.results == [None]


def test_visited_optional():
    doc = _sample_doc(with_result=True, with_visited=False)
    text = dumps_scene(doc)
    assert "[visited]" not in text
    assert "[path]" in text
    doc2 = loads_scene(text)
    assert doc2.results[0].visited is None
    assert doc2.results[0].path == doc.results[0].path


# ------------------------------------------------------------ 점유 레이어 재구성

def test_occupancy_reconstruction():
    doc = _sample_doc(with_result=False)
    occ = occupancy_from_doc(doc)
    assert occ.shape == doc.shape
    assert occ.cell_mm == doc.cell_mm
    # 동일 장애물로 직접 만든 점유맵과 점유 셀 수가 같아야 한다.
    ref = DenseOccupancyMap(doc.shape, doc.origin, doc.cell_mm)
    for o in doc.obstacles:
        ref.add_box(AABB(o.min_xyz, o.max_xyz))
    assert occ.count_blocked() == ref.count_blocked()
    assert occ.count_blocked() > 0


def test_degenerate_obstacle_skipped():
    # 두께 0(퇴화) 박스가 섞여 있어도 재구성이 죽지 않고 건너뛴다.
    doc = _sample_doc(with_result=False)
    doc.obstacles.append(
        Obstacle(object_id="D", name=None, ost_type=None, ddworks_type=None,
                 min_xyz=(300.0, 300.0, 100.0), max_xyz=(300.0, 350.0, 150.0))  # x 두께 0
    )
    occ = occupancy_from_doc(doc)   # ValueError 없이 통과해야 함
    assert occ.count_blocked() > 0


# ------------------------------------------------------------ params / 버전

def test_w_tier_roundtrip():
    doc = _sample_doc(with_result=False)
    doc2 = loads_scene(dumps_scene(doc))
    assert doc2.params.w_tier == {1: 200.0}


def test_empty_w_tier():
    doc = _sample_doc(with_result=False)
    doc.params.w_tier = {}
    doc2 = loads_scene(dumps_scene(doc))
    assert doc2.params.w_tier == {}


def test_doc_from_scene_helper():
    # RoutingScene 없이도 obstacles/tasks 를 들고 있는 간이 객체로 동작 확인.
    class _FakeScene:
        def __init__(self, obstacles, tasks):
            self.obstacles = obstacles
            self.tasks = tasks

    doc = _sample_doc(with_result=False)
    occ = occupancy_from_doc(doc)
    fake = _FakeScene(doc.obstacles, doc.tasks)
    built = doc_from_scene(fake, occ, params=doc.params)
    assert built.shape == doc.shape
    assert built.cell_mm == doc.cell_mm
    assert built.obstacles == doc.obstacles
    assert built.tasks == doc.tasks
    assert built.results == [None] * len(doc.tasks)


def test_version_mismatch_raises():
    doc = _sample_doc(with_result=False)
    text = dumps_scene(doc).replace("@version 1", "@version 2")
    with pytest.raises(ValueError):
        loads_scene(text)

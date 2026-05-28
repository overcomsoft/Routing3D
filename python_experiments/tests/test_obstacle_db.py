"""장애물 DB 로더 단위/통합 테스트 — obstacle_db
================================================================================

[실행 명령어]  (python_experiments/ 디렉토리에서)
  # 이 파일만 실행
  ..\\.venv\\Scripts\\python.exe -m pytest tests/test_obstacle_db.py -v
  # DB 통합 테스트 제외하고 순수 로직만
  ..\\.venv\\Scripts\\python.exe -m pytest tests/test_obstacle_db.py -v -m "not db"

[구성]
  - 순수 로직 테스트: DB 없이 동작(Obstacle/AABB 변환, 영역 겹침, build_occupancy 등).
  - DB 통합 테스트(@pytest.mark.db): 실제 PostgreSQL 에 접속. 연결 불가 시 자동 skip.
================================================================================
"""

import pytest

from routing3d_py.obstacle_db import (
    BuildResult,
    Obstacle,
    PgConnConfig,
    _aabb_overlaps,
    build_occupancy,
    distinct_ost_types,
    group_by_type,
    load_obstacles,
    obstacles_bounds,
)


# ----------------------------------------------------------- 순수 로직 (DB 불필요)

def _obs(min_xyz, max_xyz, ost_type="OST_Columns", oid="x"):
    """테스트용 Obstacle 생성 헬퍼."""
    return Obstacle(object_id=oid, name="n", ost_type=ost_type,
                    ddworks_type="d", min_xyz=min_xyz, max_xyz=max_xyz)


def test_obstacle_valid_box():
    assert _obs((0, 0, 0), (100, 100, 100)).is_valid_box()
    # z 축이 퇴화(두께 0)
    assert not _obs((0, 0, 0), (100, 100, 0)).is_valid_box()
    # x 축 역전
    assert not _obs((100, 0, 0), (0, 100, 100)).is_valid_box()


def test_obstacle_to_aabb():
    aabb = _obs((10, 20, 30), (110, 120, 130)).to_aabb()
    assert aabb.lo == (10, 20, 30)
    assert aabb.hi == (110, 120, 130)


def test_obstacle_to_aabb_degenerate_raises():
    with pytest.raises(ValueError):
        _obs((0, 0, 0), (100, 100, 0)).to_aabb()


def test_aabb_overlaps():
    # 겹침
    assert _aabb_overlaps((0, 0, 0), (10, 10, 10), (5, 5, 5), (20, 20, 20))
    # 면 접촉(경계 공유)도 겹침으로 본다
    assert _aabb_overlaps((0, 0, 0), (10, 10, 10), (10, 0, 0), (20, 10, 10))
    # x 축에서 분리
    assert not _aabb_overlaps((0, 0, 0), (10, 10, 10), (11, 0, 0), (20, 10, 10))


def test_obstacles_bounds():
    obs = [_obs((0, 0, 0), (10, 10, 10)), _obs((5, -5, 2), (20, 8, 30))]
    lo, hi = obstacles_bounds(obs)
    assert lo == pytest.approx((0, -5, 0))
    assert hi == pytest.approx((20, 10, 30))


def test_obstacles_bounds_empty_raises():
    with pytest.raises(ValueError):
        obstacles_bounds([])


def test_group_by_type():
    obs = [
        _obs((0, 0, 0), (1, 1, 1), ost_type="OST_Floors"),
        _obs((0, 0, 0), (1, 1, 1), ost_type="OST_Columns"),
        _obs((0, 0, 0), (1, 1, 1), ost_type="OST_Floors"),
        Obstacle("y", "n", None, "d", (0, 0, 0), (1, 1, 1)),  # None 타입 → ''
    ]
    groups = group_by_type(obs)
    assert len(groups["OST_Floors"]) == 2
    assert len(groups["OST_Columns"]) == 1
    assert len(groups[""]) == 1


def test_build_occupancy_auto_region():
    # 셀 50mm. 박스 0..100mm → 셀 [0,2) 2칸/축 = 8셀
    obs = [_obs((0, 0, 0), (100, 100, 100))]
    res = build_occupancy(obs, cell_mm=50)
    assert isinstance(res, BuildResult)
    assert res.n_obstacles == 1
    assert res.n_voxelized == 1
    assert res.occupancy.count_blocked() == 8
    assert res.occupancy.shape == (2, 2, 2)


def test_build_occupancy_skips_degenerate():
    obs = [
        _obs((0, 0, 0), (100, 100, 100)),       # 정상
        _obs((0, 0, 0), (100, 100, 0)),         # 퇴화(z 두께 0)
    ]
    res = build_occupancy(obs, cell_mm=50, region=((0, 0, 0), (100, 100, 100)))
    assert res.n_skipped_degenerate == 1
    assert res.n_voxelized == 1


def test_build_occupancy_skips_outside_region():
    obs = [
        _obs((0, 0, 0), (100, 100, 100)),           # 영역 안
        _obs((5000, 5000, 5000), (5100, 5100, 5100)),  # 영역 밖
    ]
    res = build_occupancy(obs, cell_mm=50, region=((0, 0, 0), (200, 200, 200)))
    assert res.n_voxelized == 1
    assert res.n_skipped_outside == 1


def test_build_occupancy_padding():
    obs = [_obs((0, 0, 0), (100, 100, 100))]
    res = build_occupancy(obs, cell_mm=50, padding_mm=50)
    # 영역이 -50..150mm 로 확장 → 4칸/축
    assert res.occupancy.shape == (4, 4, 4)
    assert res.region_lo == pytest.approx((-50, -50, -50))
    assert res.region_hi == pytest.approx((150, 150, 150))


def test_build_occupancy_max_cells_guard():
    obs = [_obs((0, 0, 0), (1_000_000, 1_000_000, 1_000_000))]
    with pytest.raises(ValueError, match="too large"):
        build_occupancy(obs, cell_mm=1, max_cells=1000)


def test_build_occupancy_empty_no_region_raises():
    with pytest.raises(ValueError):
        build_occupancy([], cell_mm=50)


# ----------------------------------------------------------- DB 통합 (연결 시에만)

@pytest.fixture(scope="module")
def db_conn():
    """실제 DB 연결. 불가하면 모듈 내 db 테스트 전체 skip."""
    config = PgConnConfig.from_env()
    try:
        conn = config.connect()
    except Exception as e:  # noqa: BLE001 — 연결 실패 사유 무관하게 skip
        pytest.skip(f"PostgreSQL 연결 불가 — DB 통합 테스트 skip ({e})")
    yield conn, config
    conn.close()


@pytest.mark.db
def test_db_distinct_ost_types(db_conn):
    conn, config = db_conn
    types = distinct_ost_types(conn=conn, config=config)
    assert isinstance(types, dict)
    assert len(types) > 0
    # 알려진 주요 타입이 존재해야 함
    assert "OST_Columns" in types or "OST_Floors" in types


@pytest.mark.db
def test_db_load_with_region_and_type(db_conn):
    conn, config = db_conn
    # 샘플로 확인된 기둥이 있는 영역 부근
    region = ((199000, 9000, 14000), (201000, 11000, 16000))
    obs = load_obstacles(conn=conn, config=config,
                         ost_types=["OST_Columns"], region=region, limit=50)
    assert all(o.ost_type == "OST_Columns" for o in obs)
    assert all(o.is_valid_box() for o in obs)  # skip_degenerate 기본 True


@pytest.mark.db
def test_db_build_occupancy_small_region(db_conn):
    conn, config = db_conn
    region = ((199000, 9000, 14000), (201000, 11000, 16000))
    obs = load_obstacles(conn=conn, config=config, region=region, limit=500)
    res = build_occupancy(obs, cell_mm=50, region=region)
    assert res.occupancy.shape[0] > 0
    # 영역 내 장애물이 있으면 점유 셀이 생겨야 함
    if res.n_voxelized > 0:
        assert res.occupancy.count_blocked() > 0

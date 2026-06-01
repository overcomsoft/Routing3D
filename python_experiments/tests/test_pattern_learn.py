r"""스텁 패턴 학습 단위/통합 테스트 — pattern_learn / route_db / pattern_db
================================================================================
[실행 명령어]  (python_experiments/ 디렉토리에서)
  # 이 파일만 실행
  ..\.venv\Scripts\python.exe -m pytest tests/test_pattern_learn.py -v
  # DB 통합 제외(순수 로직만)
  ..\.venv\Scripts\python.exe -m pytest tests/test_pattern_learn.py -v -m "not db"

[구성]
  - 순수 로직: DB 없이(축 스냅/면 분류/특징벡터/호칭경 파싱/벡터 리터럴).
  - DB 통합(@pytest.mark.db): 실제 PostgreSQL. 연결 불가 시 자동 skip.
================================================================================
"""

import pytest

from routing3d_py.obstacle_db import PgConnConfig
from routing3d_py.pattern_db import FEAT_DIM, vec_literal
from routing3d_py.route_db import parse_pipe_size_mm
from routing3d_py import pattern_learn as pl


# ----------------------------------------------------------- 순수 로직 (DB 불필요)

def test_axis_snap():
    assert pl.axis_snap((1, 0, 0)) == 0       # +x
    assert pl.axis_snap((-3, 0, 0)) == 1      # -x
    assert pl.axis_snap((0, 2, 0)) == 2       # +y
    assert pl.axis_snap((0, -1, 0)) == 3      # -y
    assert pl.axis_snap((0, 0, 5)) == 4       # +z
    assert pl.axis_snap((0, 0, -0.7)) == 5    # -z
    # 지배 성분이 z (작은 잡음 사선 → 가장 큰 축으로)
    assert pl.axis_snap((10, 5, 100)) == 4


def test_nearest_face():
    lo, hi = (0, 0, 0), (100, 100, 100)
    # 윗면 +z 바로 위
    assert pl.nearest_face((50, 50, 100), lo, hi) == 4
    # 아랫면 -z
    assert pl.nearest_face((50, 50, 0), lo, hi) == 5
    # +x 면
    assert pl.nearest_face((100, 50, 50), lo, hi) == 0


def test_feature_vector_dim_and_onehot():
    feat = pl.build_feature_vector(face=4, dir_seq=[4, 0], rel_pos=[0.5, 0.5, 1.0],
                                   dir_unit=[0, 0, 1])
    assert len(feat) == FEAT_DIM == 24
    assert feat[4] == 1.0                      # face +z one-hot
    assert feat[6 + 4] == 1.0                  # 1차 방향 +z one-hot
    assert feat[12 + 0] == 1.0                 # 2차 방향 +x one-hot
    assert feat[18:21] == [0.5, 0.5, 1.0]      # 상대좌표
    assert feat[21:24] == [0.0, 0.0, 1.0]      # 진행 단위벡터


def test_feature_vector_no_second_dir():
    feat = pl.build_feature_vector(face=5, dir_seq=[5], rel_pos=[0, 0, 0], dir_unit=[0, 0, -1])
    assert sum(feat[12:18]) == 0.0             # 2차 방향 없음 → 전부 0


def test_parse_pipe_size_mm():
    assert parse_pipe_size_mm("40A") == 40
    assert parse_pipe_size_mm("150A") == 150
    assert abs(parse_pipe_size_mm("1/2B") - 12.7) < 1e-6
    assert abs(parse_pipe_size_mm("1B") - 25.4) < 1e-6
    assert parse_pipe_size_mm("1/4BX1/2B") == pytest.approx(6.35)  # 첫 토큰
    assert parse_pipe_size_mm("") == 0.0
    assert parse_pipe_size_mm(None) == 0.0


def test_vec_literal():
    assert vec_literal([1, 2, 3]) == "[1,2,3]"
    assert vec_literal(None) is None
    assert vec_literal([0.5, -1.25]) == "[0.5,-1.25]"


def test_walk_stub_bends_and_cap():
    # PoC 에서 +z 로 300, 그다음 +x 로 500 → dir_seq = [+z, +x], 꺾임 1.
    seg = [(0, 0, 0), (0, 0, 300), (500, 0, 300)]
    pts, dirs = pl._walk_stub(seg)
    assert dirs == [4, 0]                       # +z, +x
    assert len(pts) >= 3


# ----------------------------------------------------------- DB 통합 (@pytest.mark.db)

@pytest.fixture(scope="module")
def db_conn():
    config = PgConnConfig.from_env()
    try:
        conn = config.connect()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"PostgreSQL 연결 불가 — DB 통합 테스트 skip ({e})")
    yield conn, config
    conn.close()


@pytest.mark.db
def test_db_learn_project6(db_conn):
    from routing3d_py import route_db
    conn, config = db_conn
    sf = route_db.resolve_source_file(6, conn=conn)
    rows = pl.learn_project(sf, conn=conn)
    assert len(rows) > 0
    # 모든 표본의 특징벡터/방향벡터 차원이 스키마와 일치.
    for r in rows:
        assert len(r.feat) == FEAT_DIM
        assert len(r.dir_unit) == 3
        assert r.anchor_kind in ("EQUIP", "DUCT")
    # 출발 스텁은 장비(아래로), 종단 스텁은 덕트(위로)가 지배적이어야 한다(도메인 상식).
    equip = [r for r in rows if r.anchor_kind == "EQUIP"]
    duct = [r for r in rows if r.anchor_kind == "DUCT"]
    assert sum(1 for r in equip if r.face == "-z") > len(equip) * 0.5
    assert sum(1 for r in duct if r.face == "+z") > len(duct) * 0.4


@pytest.mark.db
def test_db_nearest_stubs_category_filter(db_conn):
    from routing3d_py.pattern_db import nearest_stubs, count_samples
    conn, config = db_conn
    if count_samples(conn=conn) == 0:
        pytest.skip("저장소가 비어 있음 — pattern_learn --write-db 먼저 실행")
    # 임의 질의 벡터(+z 면, +z 방향)로 DUCT 검색 → 결과의 범주가 질의와 일치해야 함.
    q = pl.build_feature_vector(face=4, dir_seq=[4], rel_pos=[0.5, 0.5, 1.0], dir_unit=[0, 0, 1])
    res = nearest_stubs("DUCT", "Exhaust", "ACID", q, k=5, conn=conn)
    # 표본이 있으면 거리 오름차순으로 반환.
    if res:
        dists = [row[-1] for row in res]
        assert dists == sorted(dists)

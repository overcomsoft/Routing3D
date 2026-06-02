r"""그룹(번들) 배관 탐지 단위 테스트 — bundle_detect
================================================================================
[실행 명령어]  (python_experiments/ 디렉토리에서)
  ..\.venv\Scripts\python.exe -m pytest tests/test_bundle_detect.py -v

[구성]
  순수 로직(DB 불필요): 합성 폴리라인으로 특징 추출·복합 유사도·번들 탐지 검증.
    - 평행 다발(동일 pitch, 2엘보) → 1 번들.
    - 직선/1엘보 → 번들 아님(꺾임 게이트 탈락).
    - 불규칙 간격 평행 → 번들 아님(pitch CV 탈락).
================================================================================
"""

from routing3d_py.route_db import ExistingPipe
from routing3d_py import bundle_detect as bd


# ----------------------------------------------------------- 합성 배관 헬퍼
def _pipe(guid, points, owner="EQ1", util="ACID"):
    return ExistingPipe(route_path_guid=guid, owner_name=owner, utility=util, points=points)


def _bundle_member(guid, y_offset, owner="EQ1", util="ACID"):
    """동일 형태(올라가서 +x 로 길게 가다 다시 내려옴, 2엘보)의 평행 배관 — y 만 offset."""
    return _pipe(guid, [
        (0.0, y_offset, 0.0),       # 바닥
        (0.0, y_offset, 3000.0),    # ↑ 수직(R)
        (8000.0, y_offset, 3000.0),  # → 수평(H)  ← 엘보 1
        (8000.0, y_offset, 0.0),    # ↓ 수직(R)  ← 엘보 2
    ], owner, util)


# ----------------------------------------------------------- Phase 1 특징
def test_arrow_code_RHR():
    p = _bundle_member("g", 0.0)
    assert bd.arrow_code(p.points) == "RHR"


def test_count_ortho_bends():
    p = _bundle_member("g", 0.0)
    # R(z) → H(x) → R(z) : 축 전환 2회.
    assert bd.count_ortho_bends(p.points) == 2


def test_classify_seg_diagonal():
    assert bd._classify_seg((0, 0, 0), (0, 0, 100)) == "R"      # 수직
    assert bd._classify_seg((0, 0, 0), (100, 0, 0)) == "H"      # 수평
    assert bd._classify_seg((0, 0, 0), (100, 0, 100)) == "D"    # 45° 경사


def test_resample_and_units():
    p = _bundle_member("g", 0.0)
    rs = bd.resample_polyline(p.points, 13)
    assert len(rs) == 13
    units = bd._seg_units(rs)
    assert len(units) == 12


# ----------------------------------------------------------- Phase 2 유사도
def test_levenshtein():
    assert bd.levenshtein("RHR", "RHR") == 0
    assert bd.levenshtein("RHR", "RHRH") == 1
    assert bd.levenshtein("", "RH") == 2


def test_identical_pipes_similarity_high():
    fa = bd.extract_feature(_bundle_member("a", 0.0))
    fb = bd.extract_feature(_bundle_member("b", 500.0))   # 같은 형태, y 평행 이동.
    s = bd.composite_similarity(fa, fb)
    assert s > 0.95          # 평행 동일 형태 → 매우 높음.


def test_different_shape_similarity_low():
    fa = bd.extract_feature(_bundle_member("a", 0.0))
    fb = bd.extract_feature(_pipe("b", [(0, 0, 0), (5000, 0, 0)]))   # 직선.
    assert bd.composite_similarity(fa, fb) < 0.7


# ----------------------------------------------------------- Phase 3 번들 탐지
def test_detect_parallel_bundle():
    """동일 pitch(500) 평행 3배관(2엘보) → 1 번들, 멤버 3."""
    pipes = [_bundle_member(f"g{i}", i * 500.0) for i in range(3)]
    groups = bd.detect_bundles(pipes)
    assert len(groups) == 1
    g = groups[0]
    assert g.n_members == 3
    assert g.n_ortho_bends == 2
    assert abs(g.pitch_mm - 500.0) < 1.0
    assert abs(g.trunk_xy_spread - 1000.0) < 1.0     # 0..1000.
    assert abs(g.trunk_z - 3000.0) < 1.0             # 수평 런 고도.
    assert g.owner_name == "EQ1" and g.utility == "ACID"


def test_straight_pipes_not_bundle():
    """직선 평행 배관 → 꺾임 0 이므로 번들 아님."""
    pipes = [_pipe(f"s{i}", [(0, i * 500.0, 0), (6000, i * 500.0, 0)]) for i in range(3)]
    assert bd.detect_bundles(pipes) == []


def test_single_elbow_not_bundle():
    """1엘보 평행 배관 → 꺾임 1 < 2 이므로 번들 아님."""
    pipes = [_pipe(f"e{i}", [(0, i * 500.0, 0), (0, i * 500.0, 3000), (5000, i * 500.0, 3000)])
             for i in range(3)]
    assert bd.detect_bundles(pipes) == []


def test_irregular_pitch_not_bundle():
    """불규칙 간격(0, 500, 3000) 평행 → pitch CV 큼 → 번들 아님."""
    offs = [0.0, 500.0, 3000.0]
    pipes = [_bundle_member(f"g{i}", offs[i]) for i in range(3)]
    assert bd.detect_bundles(pipes, pitch_cv_max=0.30) == []


def test_different_utility_not_grouped():
    """같은 형태·간격이라도 유틸리티가 다르면 같은 그룹에 묶이지 않는다."""
    pipes = [_bundle_member("a", 0.0, util="ACID"),
             _bundle_member("b", 500.0, util="CAUSTIC")]
    assert bd.detect_bundles(pipes) == []


def test_two_member_bundle():
    """2개 평행(2엘보) → 번들(2개면 pitch CV=0, 자명한 등간격)."""
    pipes = [_bundle_member("a", 0.0), _bundle_member("b", 500.0)]
    groups = bd.detect_bundles(pipes)
    assert len(groups) == 1 and groups[0].n_members == 2


# ----------------------------------------------------------- 템플릿(신규설계 활용)
def test_aggregate_templates():
    """같은 (owner,util) 두 번들이 한 템플릿으로 집계되고 트렁크 고도가 합쳐진다."""
    pipes = [_bundle_member(f"g{i}", i * 500.0) for i in range(3)]
    groups = bd.detect_bundles(pipes)
    tpls = bd.aggregate_templates(groups)
    assert len(tpls) == 1
    t = tpls[0]
    assert t.owner_name == "EQ1" and t.utility == "ACID"
    assert any(abs(z - 3000.0) < 1.0 for z in t.trunk_zs)
    assert t.n_members == 3


def test_suggest_bundle_exact_and_fallback():
    pipes = [_bundle_member(f"g{i}", i * 500.0) for i in range(3)]
    tpls = bd.aggregate_templates(bd.detect_bundles(pipes))
    # 정확 키.
    s = bd.suggest_bundle(tpls, "EQ1", "ACID")
    assert s is not None and any(abs(z - 3000.0) < 1.0 for z in s.trunk_zs)
    # 유틸 폴백(다른 장비라도 같은 유틸 트렁크 고도 활용).
    f = bd.suggest_bundle(tpls, "EQ_NEW", "ACID")
    assert f is not None and any(abs(z - 3000.0) < 1.0 for z in f.trunk_zs)
    # 미스.
    assert bd.suggest_bundle(tpls, "EQ1", "UNKNOWN") is None

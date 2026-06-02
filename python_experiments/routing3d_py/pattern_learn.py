r"""기존설계 스텁 패턴 학습 — 추출·정규화·특징벡터 (패턴 학습 L1)

================================================================================
[실행 명령어]  (모두 프로젝트 루트에서)
  # ① 학습 + 검수 리포트(콘솔, DB 미적재)
  .\.venv\Scripts\python.exe -m routing3d_py.pattern_learn --project 6 --report

  # ② 학습 + pgvector 저장소 적재(기존 표본은 먼저 정리)
  .\.venv\Scripts\python.exe -m routing3d_py.pattern_learn --project 6 --write-db

  # ③ 적재 결과(대표 템플릿) 확인
  .\.venv\Scripts\python.exe -m routing3d_py.pattern_db --stats
================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
사람이 설계한 기존배관(route_db.ExistingPipe)의 '양 끝 스텁'을 추출해 정규화하고,
(anchor_kind, utility_group, utility) 키 + 기하 특징벡터(feat 24차원) + 진행 단위벡터
(dir_unit 3차원)로 표현한다. 결과는 StubSampleRow 로 pattern_db 에 적재된다.

  출발 스텁 : SOURCE PoC 쪽. 앵커 = 그 PoC 를 포함/최근접하는 장비 AABB(EQUIP).
  종단 스텁 : TARGET PoC 쪽. 앵커 = 그 PoC 를 포함/최근접하는 덕트·레터럴 AABB(DUCT).

[정규화(로컬 프레임)]  좌표를 앵커 AABB 기준으로 본다 — 위치·크기가 달라도 같은 패턴으로 정렬.
  face     : PoC 가 가장 가까운 앵커 면(+x..-z).
  dir_seq  : 스텁 폴리라인을 직교 축으로 스냅한 방향 시퀀스(연속 동일 방향 병합). 꺾임 = len-1.
  rise_mm  : 면 법선축으로 스텁이 이동한 최대 거리(덕트 상부로 뜬 높이 등).
  offset_mm: PoC 와 면 평면 사이 간극(표면 바깥 여유).

[특징벡터 feat(24)]  (pattern_db.FEAT_DIM 와 일치)
  [face 1hot 6][1차방향 1hot 6][2차방향 1hot 6][앵커내 PoC 상대좌표 3][시작→종단 단위 3]
  one-hot/상대좌표/단위벡터라 성분 스케일이 균형 → 그룹 내 L2/코사인 검색이 의미를 가진다.
================================================================================
"""

from __future__ import annotations

import math

from .obstacle_db import PgConnConfig
from .pattern_db import FEAT_DIM, StubSampleRow, apply_schema, clear_source, insert_samples
from . import route_db
from .route_db import DuctLateral, EquipmentBox, ExistingPipe, Vec3

# 6직교 축: 인덱스 0..5 = +x,-x,+y,-y,+z,-z. 각 단위벡터.
AXIS_NAMES = ["+x", "-x", "+y", "-y", "+z", "-z"]
AXIS_VECS: list[Vec3] = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]

# 스텁으로 인정할 PoC 끝에서의 최대 누적 길이(mm)·최대 꺾임 수.
STUB_MAX_MM = 4000.0
STUB_MAX_BENDS = 3
# 방향 런(run)이 이 길이 미만이면 잡음(설계 지터)으로 보고 인접 런에 흡수한다 — 미세 옵셋 지터가
# '엘보'로 오인돼 꺾임 예산을 소진하고 정작 수직→수평 전환(진짜 엘보)을 놓치는 문제를 막는다.
STUB_MIN_DIR_RUN_MM = 250.0
# 스텁은 '수직배관 + 첫 엘보(수직→수평 전환)'까지 본다. 엘보 이후 수평 리드인을 이만큼만 담고 종료
# (전체 랙 런이 아니라 엘보 방향을 기록할 정도). 이로써 스텁 = 출발/진입면 + 수직 + 엘보.
STUB_LEADIN_MM = 800.0
# 앵커(장비/덕트) 매칭 허용 반경(mm) — AABB 밖이어도 이 거리 내 중심이면 매칭.
ANCHOR_MAX_MM = 3000.0


# ------------------------------------------------------------------ 벡터 소도구

def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def axis_snap(d: Vec3) -> int:
    """3-벡터를 가장 가까운 6직교 축 인덱스(0..5)로 스냅한다(최대 절대성분 축의 부호)."""
    ax = max(range(3), key=lambda i: abs(d[i]))
    return ax * 2 + (0 if d[ax] >= 0 else 1)


def _onehot6(idx: int | None) -> list[float]:
    v = [0.0] * 6
    if idx is not None and 0 <= idx < 6:
        v[idx] = 1.0
    return v


def _unit(d: Vec3) -> list[float]:
    n = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    if n < 1e-9:
        return [0.0, 0.0, 0.0]
    return [d[0] / n, d[1] / n, d[2] / n]


# ------------------------------------------------------------------ 앵커 매칭

def _box_center(lo: Vec3, hi: Vec3) -> Vec3:
    return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)


def find_equipment(poc: Vec3, equipment: list[EquipmentBox]) -> EquipmentBox | None:
    """PoC 를 포함하는 장비, 없으면 ANCHOR_MAX_MM 내 가장 가까운 장비. 없으면 None."""
    for e in equipment:
        if e.contains(poc):
            return e
    best, bestd = None, ANCHOR_MAX_MM
    for e in equipment:
        d = _dist(poc, _box_center(e.lo, e.hi))
        if d < bestd:
            best, bestd = e, d
    return best


def find_duct(poc: Vec3, ducts: list[DuctLateral]) -> DuctLateral | None:
    """PoC 를 포함하는 덕트, 없으면 ANCHOR_MAX_MM 내 가장 가까운 덕트. 없으면 None."""
    for d in ducts:
        if d.contains(poc):
            return d
    best, bestd = None, ANCHOR_MAX_MM
    for d in ducts:
        c = _box_center(d.lo, d.hi)
        dd = _dist(poc, c)
        if dd < bestd:
            best, bestd = d, dd
    return best


# ------------------------------------------------------------------ 스텁 형상

def nearest_face(poc: Vec3, lo: Vec3, hi: Vec3) -> int:
    """PoC 가 가장 가까운 앵커 AABB 면 인덱스(0..5 = +x,-x,+y,-y,+z,-z)."""
    cand = [
        (hi[0] - poc[0], 0), (poc[0] - lo[0], 1),
        (hi[1] - poc[1], 2), (poc[1] - lo[1], 3),
        (hi[2] - poc[2], 4), (poc[2] - lo[2], 5),
    ]
    return min(cand, key=lambda c: abs(c[0]))[1]


def _dir_runs(seg: list[Vec3]) -> list[list]:
    """폴리라인을 방향 런 [[축d, 누적길이], …] 으로 압축한다(연속 동일 방향 병합)."""
    runs: list[list] = []
    for i in range(1, len(seg)):
        a, b = seg[i - 1], seg[i]
        seglen = _dist(a, b)
        if seglen < 1e-6:
            continue
        d = axis_snap(_sub(b, a))
        if runs and runs[-1][0] == d:
            runs[-1][1] += seglen
        else:
            runs.append([d, seglen])
    return runs


def _merge_short_runs(runs: list[list]) -> list[list]:
    """STUB_MIN_DIR_RUN_MM 미만 방향 런을 인접 런에 흡수한다(설계 지터 제거).

    가장 짧은 미달 런을 골라: 양 이웃이 같은 방향이면 셋을 병합, 아니면 더 긴 이웃에 길이를
    흡수시키고 제거한다. 모든 런이 임계 이상이거나 1개만 남을 때까지 반복.
    """
    runs = [r[:] for r in runs]
    while len(runs) > 1:
        idx = min(range(len(runs)), key=lambda i: runs[i][1])
        if runs[idx][1] >= STUB_MIN_DIR_RUN_MM:
            break
        if idx == 0:
            runs[1][1] += runs[0][1]
            runs.pop(0)
        elif idx == len(runs) - 1:
            runs[-2][1] += runs[-1][1]
            runs.pop()
        elif runs[idx - 1][0] == runs[idx + 1][0]:
            runs[idx - 1][1] += runs[idx][1] + runs[idx + 1][1]
            del runs[idx:idx + 2]
        elif runs[idx - 1][1] >= runs[idx + 1][1]:
            runs[idx - 1][1] += runs[idx][1]
            runs.pop(idx)
        else:
            runs[idx + 1][1] += runs[idx][1]
            runs.pop(idx)
    return runs


def _points_until(seg: list[Vec3], length: float) -> list[Vec3]:
    """PoC(seg[0])에서 누적 길이 length 까지의 점열(마지막 세그먼트는 잘라서 끝점 추가)."""
    out: list[Vec3] = [seg[0]]
    total = 0.0
    for i in range(1, len(seg)):
        a, b = seg[i - 1], seg[i]
        seglen = _dist(a, b)
        if seglen < 1e-6:
            continue
        if total + seglen >= length:
            t = (length - total) / seglen if seglen > 0 else 1.0
            out.append(_lerp(a, b, max(0.0, min(1.0, t))))
            break
        out.append(b)
        total += seglen
    return out


def _walk_stub(seg: list[Vec3]) -> tuple[list[Vec3], list[int]]:
    """PoC(=seg[0])에서 시작해 스텁 구간(출발/진입면 + 수직배관 + 첫 엘보)을 잘라낸다.

    [개선] 단순히 '수직배관까지'가 아니라 '수직 → 첫 엘보(수직축→수평축 전환)'까지를 스텁으로 본다.
      ① 방향 런 압축 후 STUB_MIN_DIR_RUN_MM 미만 런(설계 지터)을 흡수해 가짜 꺾임을 없앤다.
      ② 첫 방향(런[0])의 축을 '수직축'으로 보고, 축이 다른 첫 런 = 엘보. 엘보 직후 STUB_LEADIN_MM
         수평 리드인까지만 담아 스텁을 종료한다(엘보 방향을 기록할 정도, 전체 랙 런은 제외).
      ③ 엘보가 없으면 STUB_MAX_MM·STUB_MAX_BENDS 한도까지.
    반환: (스텁 점열, 방향 시퀀스 인덱스 리스트[엘보 포함]).
    """
    runs = _merge_short_runs(_dir_runs(seg))
    if not runs:
        return [seg[0]], []

    vert_axis = runs[0][0] // 2          # 첫 방향(수직배관)의 축.
    elbow = None                          # 첫 엘보(축이 다른 첫 런)의 인덱스.
    for i in range(1, len(runs)):
        if runs[i][0] // 2 != vert_axis:
            elbow = i
            break

    if elbow is None:
        dir_seq = [r[0] for r in runs[:STUB_MAX_BENDS + 1]]
        length = min(STUB_MAX_MM, sum(r[1] for r in runs[:STUB_MAX_BENDS + 1]))
    else:
        keep = runs[:elbow + 1][:STUB_MAX_BENDS + 1]   # 수직(들) + 엘보, 꺾임 한도.
        dir_seq = [r[0] for r in keep]
        pre = sum(r[1] for r in runs[:elbow])           # 엘보 이전(수직) 누적.
        length = min(STUB_MAX_MM, pre + min(runs[elbow][1], STUB_LEADIN_MM))

    return _points_until(seg, length), dir_seq


def _rel_pos(poc: Vec3, lo: Vec3, hi: Vec3) -> list[float]:
    """앵커 AABB 내 PoC 상대좌표(축별 [0,1], 퇴화축은 0.5)."""
    out = []
    for i in range(3):
        span = hi[i] - lo[i]
        out.append(0.5 if span <= 1e-6 else min(1.0, max(0.0, (poc[i] - lo[i]) / span)))
    return out


def build_feature_vector(face: int, dir_seq: list[int], rel_pos: list[float],
                         dir_unit: list[float]) -> list[float]:
    """24차원 특징벡터: [face 6][1차방향 6][2차방향 6][상대좌표 3][진행단위 3]."""
    feat = (_onehot6(face)
            + _onehot6(dir_seq[0] if len(dir_seq) >= 1 else None)
            + _onehot6(dir_seq[1] if len(dir_seq) >= 2 else None)
            + list(rel_pos)
            + list(dir_unit))
    assert len(feat) == FEAT_DIM, f"feat dim {len(feat)} != {FEAT_DIM}"
    return feat


# ------------------------------------------------------------------ 한 스텁 → 표본

def _make_sample(source_file: str, pipe: ExistingPipe, anchor_kind: str,
                 anchor_name: str | None, lo: Vec3, hi: Vec3, poc: Vec3,
                 seg: list[Vec3], dir_unit: list[float]) -> StubSampleRow | None:
    """앵커·PoC·스텁 점열로 StubSampleRow 하나를 만든다(스텁 길이 0 이면 None)."""
    stub_pts, dir_seq = _walk_stub(seg)
    if not dir_seq:
        return None
    face = nearest_face(poc, lo, hi)
    axis = face // 2
    rise = max((abs(p[axis] - poc[axis]) for p in stub_pts), default=0.0)
    face_plane = hi[axis] if face % 2 == 0 else lo[axis]
    offset = abs(face_plane - poc[axis])
    rel = _rel_pos(poc, lo, hi)
    feat = build_feature_vector(face, dir_seq, rel, dir_unit)
    return StubSampleRow(
        source_file=source_file, anchor_kind=anchor_kind, poc_pos=poc,
        utility_group=pipe.group, utility=pipe.utility,
        route_path_guid=pipe.route_path_guid, anchor_name=anchor_name,
        anchor_min=lo, anchor_max=hi,
        face=AXIS_NAMES[face],
        dir_seq=",".join(AXIS_NAMES[d] for d in dir_seq),
        n_bends=max(0, len(dir_seq) - 1),
        rise_mm=rise, offset_mm=offset, diameter_mm=pipe.diameter_mm or None,
        dir_unit=dir_unit, feat=feat,
    )


def _oriented(points: list[Vec3], src: Vec3) -> list[Vec3]:
    """폴리라인을 SOURCE 가 앞이 되도록 정렬(앞/뒤 끝 중 src 에 가까운 쪽을 앞으로)."""
    from .route_db import _dist2  # noqa
    if _dist2(points[0], src) <= _dist2(points[-1], src):
        return points
    return list(reversed(points))


def _nearest_index(points: list[Vec3], p: Vec3) -> int:
    return min(range(len(points)), key=lambda i: _dist(points[i], p))


def learn_pipe(source_file: str, pipe: ExistingPipe, equipment: list[EquipmentBox],
               ducts: list[DuctLateral]) -> list[StubSampleRow]:
    """기존배관 1개에서 출발(EQUIP)·종단(DUCT) 스텁 표본을 추출한다(최대 2건)."""
    if len(pipe.points) < 2 or pipe.source_pos is None or pipe.target_pos is None:
        return []
    src, tgt = pipe.source_pos, pipe.target_pos
    pts = _oriented(pipe.points, src)
    i_src = _nearest_index(pts, src)
    i_tgt = _nearest_index(pts, tgt)
    if i_tgt <= i_src:
        return []
    dir_unit = _unit(_sub(tgt, src))
    rows: list[StubSampleRow] = []

    # 출발 스텁(EQUIP): i_src 에서 i_tgt 방향으로.
    eq = find_equipment(src, equipment)
    if eq is not None:
        seg = pts[i_src:i_tgt + 1]
        if len(seg) >= 2:
            r = _make_sample(source_file, pipe, "EQUIP", eq.name, eq.lo, eq.hi,
                             src, [src] + seg[1:], dir_unit)
            if r:
                rows.append(r)

    # 종단 스텁(DUCT): i_tgt 에서 i_src 방향으로(역순).
    du = find_duct(tgt, ducts)
    if du is not None:
        seg = list(reversed(pts[i_src:i_tgt + 1]))
        if len(seg) >= 2:
            # 종단은 시작→종단의 반대 방향으로 진행하므로 dir_unit 부호 반전.
            r = _make_sample(source_file, pipe, "DUCT", du.name, du.lo, du.hi,
                             tgt, [tgt] + seg[1:], [-c for c in dir_unit])
            if r:
                rows.append(r)
    return rows


def learn_project(source_file: str, config: PgConnConfig | None = None,
                  conn=None) -> list[StubSampleRow]:
    """한 프로젝트의 모든 기존배관에서 스텁 표본을 추출한다."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        equipment = route_db.load_equipment(source_file, conn=conn)
        ducts = route_db.load_ducts(source_file, conn=conn)
        bbox = route_db.project_xy_bbox(source_file, conn=conn)
        pipes = route_db.load_existing_pipes(source_file, conn=conn, xy_bbox=bbox)
        rows: list[StubSampleRow] = []
        for p in pipes:
            rows.extend(learn_pipe(source_file, p, equipment, ducts))
        return rows
    finally:
        if own:
            conn.close()


# ------------------------------------------------------------------ 랙 레벨 학습 (L3a)
#
# 사람이 설계한 기존배관은 긴 '수평 런'을 특정 z-높이(파이프 랙)에 모아 깐다. 같은 유틸그룹의
# 수평 세그먼트 길이를 z-버킷으로 누적하면 그 그룹의 주 랙 높이가 드러난다. 자동라우팅에서
# 이 z-레벨을 엔진 rack_levels(= w_corridor 면제 z-셀)로 주면 같은 그룹 배관이 공용 랙에 뭉친다.

RACK_HORIZ_MIN_MM = 800.0   # 랙으로 인정할 최소 수평 런 길이(짧은 연결/엘보 제외).
RACK_BIN_MM = 100.0         # z 버킷 크기(mm) — 같은 랙 높이로 묶을 해상도.


def _is_horizontal(a: Vec3, b: Vec3, tol: float = 0.34) -> bool:
    """세그먼트가 수평(주 이동이 xy 평면)인가 — |dz| <= tol × 수평거리."""
    horiz = math.hypot(b[0] - a[0], b[1] - a[1])
    return horiz > 1e-6 and abs(b[2] - a[2]) <= tol * horiz


def learn_rack_levels(pipes: list[ExistingPipe], *, bin_mm: float = RACK_BIN_MM,
                      min_run_mm: float = RACK_HORIZ_MIN_MM,
                      ) -> dict[str | None, list[tuple[float, float, int]]]:
    """기존배관 수평 런의 z-레벨별 누적 길이(유틸그룹별 랙 높이)를 학습한다.

    [알고리즘]
      각 폴리라인 세그먼트에서 수평(_is_horizontal)이고 min_run_mm 이상인 것만 채택,
      중점 z 를 bin_mm 버킷으로 양자화해 그룹별로 (누적 런 길이, 세그먼트 수)를 누적한다.

    반환값:
        dict[utility_group] -> [(z_mm, run_mm, n_seg), …]  (run_mm 내림차순).
    """
    from collections import defaultdict
    acc: dict[str | None, dict[float, list]] = defaultdict(lambda: defaultdict(lambda: [0.0, 0]))
    for p in pipes:
        for i in range(1, len(p.points)):
            a, b = p.points[i - 1], p.points[i]
            seglen = _dist(a, b)
            if seglen < min_run_mm or not _is_horizontal(a, b):
                continue
            zbin = round(((a[2] + b[2]) / 2) / bin_mm) * bin_mm
            slot = acc[p.group][zbin]
            slot[0] += seglen
            slot[1] += 1
    out: dict[str | None, list[tuple[float, float, int]]] = {}
    for g, zmap in acc.items():
        out[g] = sorted(((z, rn, n) for z, (rn, n) in zmap.items()), key=lambda t: -t[1])
    return out


def rack_report(levels: dict[str | None, list[tuple[float, float, int]]], top: int = 5) -> str:
    """랙 레벨 학습 결과를 유틸그룹별 상위 z-레벨 표로 요약한다(검수용)."""
    lines = [f"학습 랙 레벨: {len(levels)} 그룹", ""]
    lines.append(f"  {'group':16} {'z_mm':>9} {'run_mm':>10} {'n':>4}  {'share':>6}")
    for g in sorted(levels, key=lambda k: -sum(r for _, r, _ in levels[k])):
        rows = levels[g]
        tot = sum(r for _, r, _ in rows) or 1.0
        for z, run, n in rows[:top]:
            lines.append(f"  {(g or '')[:16]:16} {z:9.0f} {run:10.0f} {n:4d}  {run / tot * 100:5.1f}%")
        lines.append("")
    return "\n".join(lines).rstrip()


# ------------------------------------------------------------------ 리포트

def report(rows: list[StubSampleRow]) -> str:
    """추출 표본을 (anchor_kind, group, utility) 키로 묶어 검수용 요약 문자열을 만든다."""
    from collections import defaultdict
    groups: dict[tuple, list[StubSampleRow]] = defaultdict(list)
    for r in rows:
        groups[(r.anchor_kind, r.utility_group, r.utility)].append(r)

    lines = [f"추출 스텁 표본: {len(rows)} (키 {len(groups)}종)", ""]
    lines.append(f"  {'kind':5} {'group':12} {'util':10} {'n':>4} {'face*':6} "
                 f"{'dir_seq*':12} {'rise~':>7} {'bend~':>5}")
    from statistics import median, mode

    def _mode(xs):
        try:
            return mode(xs)
        except Exception:
            return xs[0] if xs else ""

    for key in sorted(groups, key=lambda k: -len(groups[k])):
        kind, grp, util = key
        g = groups[key]
        face = _mode([r.face for r in g])
        ds = _mode([r.dir_seq for r in g])
        rise = median([r.rise_mm or 0 for r in g])
        bend = median([r.n_bends or 0 for r in g])
        lines.append(f"  {kind:5} {(grp or '')[:12]:12} {(util or '')[:10]:10} {len(g):4d} "
                     f"{face:6} {(ds or '')[:12]:12} {rise:7.0f} {bend:5.1f}")
    return "\n".join(lines)


# ------------------------------------------------------------------ CLI

def _main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(description="기존설계 스텁 패턴 학습(추출·정규화·특징벡터)")
    ap.add_argument("--project", type=int, default=None, help="project_id")
    ap.add_argument("--source", default=None, help="SOURCE_FILE 직접 지정")
    ap.add_argument("--report", action="store_true", help="검수 리포트 출력")
    ap.add_argument("--rack-report", action="store_true",
                    help="유틸그룹별 랙 레벨(수평 런 z-높이) 학습 리포트(L3a)")
    ap.add_argument("--write-db", action="store_true",
                    help="pgvector 저장소 적재(기존 표본 정리 후). --apply-schema 자동")
    ap.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = ap.parse_args(argv)

    overrides = {}
    if args.dbname:
        overrides["dbname"] = args.dbname
    config = PgConnConfig.from_env(**overrides)

    conn = config.connect()
    try:
        if args.source:
            sf = args.source
        elif args.project is not None:
            sf = route_db.resolve_source_file(args.project, conn=conn)
        else:
            ap.error("--project 또는 --source 가 필요합니다.")
            return 2
        print(f"source_file = {sf}")

        if args.rack_report:   # 랙 레벨(L3a) 만 — 스텁 학습과 독립.
            bbox = route_db.project_xy_bbox(sf, conn=conn)
            pipes = route_db.load_existing_pipes(sf, conn=conn, xy_bbox=bbox)
            print(f"기존배관 {len(pipes)}개")
            print()
            print(rack_report(learn_rack_levels(pipes)))
            return 0

        rows = learn_project(sf, conn=conn)
        print(f"추출 스텁 표본 {len(rows)}건")

        if args.report or not args.write_db:
            print()
            print(report(rows))

        if args.write_db:
            apply_schema(conn=conn)
            n_del = clear_source(sf, conn=conn)
            n_ins = insert_samples(rows, conn=conn)
            print(f"\nDB 적재: 기존 {n_del}건 삭제 → {n_ins}건 적재 (route_stub_pattern)")
    finally:
        conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

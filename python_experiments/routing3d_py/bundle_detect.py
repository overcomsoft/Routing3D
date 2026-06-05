r"""그룹(번들) 배관 탐지 — 기하 유사도 분석으로 평행 다발 추출 (패턴 학습 L4)

================================================================================
[실행 명령어]  (모두 프로젝트 루트에서)
  # 탐지 + 콘솔 리포트(DB 미적재)
  .\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --report

  # 임계/피치 조정
  .\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --threshold 0.75 --pitch-cv 0.25 --report

  # 결과 저장(route_bundle_group, 스키마 자동 적용)
  .\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --write-db

  # DB 전체(모든 프로젝트) 탐지 + 저장 — space_project_map 의 모든 source_file 순회
  .\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --all --write-db

  # DB 전체(모든 프로젝트) 탐지 + 저장 — space_project_map 의 모든 source_file 순회
  .\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --all --write-db --threshold 0.75 --pitch-cv 0.25

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
사람이 설계한 기존배관(route_db.ExistingPipe)을 장비명(owner_name)·유틸리티(utility)별로 묶어,
경로 '형태 패턴'의 유사도를 분석해 함께 다발(Bundle)로 깔린 배관 그룹을 자동 추출한다.

  번들(그룹) 배관 정의(v3) = 여러 배관이 '동일 이격간격'으로 나란히(평행) '함께 진행'하는 묶음.
    · 진행축은 수평(x·y) 뿐 아니라 수직(z, 입상/리저)도 포함한다.
    · 각 배관은 진행하는 동안 자기 '레인'(진행축과 직교하는 고정 좌표)을 ±10mm 안에서 유지한다(레인 유지).
    · 꺾임(밴딩)이 있으면, 같은 각도(±2°)로 함께 꺾이는 배관은 한 그룹으로 이어서 포함한다(코너 추종).
    · 꺾임 자체는 필수가 아니다 — 직선 평행 다발도 번들이다(옛 '≥2꺾임 필수' 게이트 폐기).

[3단계 파이프라인]  (개발계획: docs/routing3d_bundle_detection_plan.md)
  Phase 1  개별 경로 특징 추출 : 방향 런 압축 → Arrow Coding(R/H/D) · 꺾임 수 · 방향벡터 · 길이 · 규모(유사도 참고).
  Phase 2  복합 유사도(4대 지표): 형태 30%(Levenshtein) + 방향 30%(코사인) + 길이 20% + 규모 20% (참고 메트릭).
  Phase 3  공간 동시진행 검출   : (장비,유틸) 키 → 배관을 '축정렬 직선 런'으로 분해(레인 유지 ±10mm)
                                 → 같은 축·겹치는 진행구간·공유 직교좌표(±10mm)로 동시진행 묶음
                                 → perp 등간격 분할(outlier 제외) → 코너(멤버≥2 공유) 병합 → trunk_axis·z·pitch 산출.

[단위]  좌표·치수 모두 mm. 라우트=BIM 동일 월드 프레임. 직교(맨해튼) 형상 가정.
================================================================================
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field

from .obstacle_db import PgConnConfig
from . import route_db
from .route_db import ExistingPipe

Vec3 = tuple[float, float, float]

# ------------------------------------------------------------------ 파라미터(기본값)
SIM_THRESHOLD = 0.70    # (참고) 유사도 메트릭 임계 — v3 검출 게이트엔 미사용(CLI 호환).
W_SHAPE, W_DIR, W_LEN, W_SCALE = 0.30, 0.30, 0.20, 0.20   # 4대 지표 가중(합=1, 참고 메트릭).
MIN_BENDS = 0           # 번들 게이트: 최소 수직·수평 꺾임 수(v3 기본 0 — 직선 평행 다발도 번들).
PITCH_CV_MAX = 0.30     # (참고) 동일 이격간격 허용 변동계수 — v3 미사용(CLI 호환).
RESAMPLE_N = 24         # 방향 유사도 리샘플 세그먼트 수(점 개수).
DIAG_TOL = 0.34         # R/H 판정 tol — 비우세 성분이 우세 성분의 이 비율 이하라야 직교(아니면 D).
Z_BIN_MM = 100.0        # 트렁크 z 최빈 버킷(mm).

# ── v3 공간 동시진행(co-travel) 검출 파라미터 ──
LANE_TOL_MM = 10.0       # 레인 유지: 진행 중 직교(perp) 좌표 변동 허용(±). 사용자 정의 핵심값.
BEND_ANGLE_TOL_DEG = 2.0  # 밴딩 각도 동일 허용(±). 직교 BIM 데이터는 축 분류로 흡수(명시 검사 불필요).
MIN_RUN_MM = 800.0       # 동시진행 '런'으로 인정할 최소 진행길이(짧은 지터·스텁 제외).
MIN_OVERLAP_MM = 300.0   # 두 런이 '함께 진행'한다고 볼 최소 진행방향 겹침.
PITCH_GAP_FACTOR = 2.5   # 등간격 분할 — 인접 간격이 기준 런의 이 배수보다 크면 다른 다발로 끊음.
MIN_RACK_MEMBERS = 2     # 한 번들로 인정할 최소 멤버(배관) 수.
Z_MERGE_MM = 400.0       # (참고) 트렁크 z 근접 병합 임계.

# 6직교 축: 0..5 = +x,-x,+y,-y,+z,-z.
AXIS_NAMES = ["+x", "-x", "+y", "-y", "+z", "-z"]


# ================================================================== 벡터 소도구
def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _norm(d: Vec3) -> float:
    return math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def axis_snap(d: Vec3) -> int:
    """3-벡터를 가장 가까운 6직교 축 인덱스(0..5)로 스냅(최대 절대성분 축의 부호)."""
    ax = max(range(3), key=lambda i: abs(d[i]))
    return ax * 2 + (0 if d[ax] >= 0 else 1)


# ================================================================== Phase 1 — 특징
def dir_runs(points: list[Vec3]) -> list[tuple[int, float]]:
    """폴리라인을 방향 런 [(축d 0..5, 누적길이), …] 으로 압축(연속 동일 방향 병합)."""
    runs: list[list] = []
    for i in range(1, len(points)):
        L = _dist(points[i - 1], points[i])
        if L < 1e-6:
            continue
        d = axis_snap(_sub(points[i], points[i - 1]))
        if runs and runs[-1][0] == d:
            runs[-1][1] += L
        else:
            runs.append([d, L])
    return [(d, L) for d, L in runs]


def _classify_seg(a: Vec3, b: Vec3) -> str | None:
    """세그먼트를 R(수직 z) · H(수평 xy) · D(경사) 로 분류. 길이 0 이면 None."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    horiz = math.hypot(dx, dy)
    vert = abs(dz)
    if horiz < 1e-6 and vert < 1e-6:
        return None
    if vert >= horiz:
        return "R" if horiz <= DIAG_TOL * vert else "D"
    return "H" if vert <= DIAG_TOL * horiz else "D"


def arrow_code(points: list[Vec3]) -> str:
    """배관 형태를 R/H/D 시퀀스 문자열로 부호화(연속 동일 코드 압축). Levenshtein 입력."""
    out: list[str] = []
    for i in range(1, len(points)):
        c = _classify_seg(points[i - 1], points[i])
        if c is None:
            continue
        if not out or out[-1] != c:
            out.append(c)
    return "".join(out)


def count_ortho_bends(points: list[Vec3]) -> int:
    """수직·수평 꺾임(90° 엘보) 수 = 방향 런의 축(dir//2) 전환 횟수."""
    runs = dir_runs(points)
    axes = [d // 2 for d, _ in runs]
    return sum(1 for i in range(1, len(axes)) if axes[i] != axes[i - 1])


def resample_polyline(points: list[Vec3], n: int) -> list[Vec3]:
    """폴리라인을 호 길이 등간격 n 점으로 리샘플(방향 유사도 정렬용)."""
    if len(points) == 1 or n <= 1:
        return [points[0]] * max(1, n)
    cum = [0.0]
    for i in range(1, len(points)):
        cum.append(cum[-1] + _dist(points[i - 1], points[i]))
    total = cum[-1]
    if total < 1e-9:
        return [points[0]] * n
    out: list[Vec3] = []
    j = 0
    for k in range(n):
        target = total * k / (n - 1)
        while j < len(cum) - 2 and cum[j + 1] < target:
            j += 1
        seg = cum[j + 1] - cum[j]
        t = 0.0 if seg < 1e-9 else (target - cum[j]) / seg
        out.append(_lerp(points[j], points[j + 1], max(0.0, min(1.0, t))))
    return out


def _seg_units(resampled: list[Vec3]) -> list[Vec3]:
    """리샘플 점열 → 단위 방향벡터 열(길이 n-1)."""
    units: list[Vec3] = []
    for i in range(1, len(resampled)):
        d = _sub(resampled[i], resampled[i - 1])
        nrm = _norm(d)
        units.append((0.0, 0.0, 0.0) if nrm < 1e-9 else (d[0] / nrm, d[1] / nrm, d[2] / nrm))
    return units


def _trunk_axis(runs: list[tuple[int, float]]) -> int:
    """주 수평축(0=x, 1=y) = 가장 긴 수평 런의 축. 수평 런 없으면 0."""
    best_axis, best_len = 0, -1.0
    for d, L in runs:
        ax = d // 2
        if ax in (0, 1) and L > best_len:
            best_axis, best_len = ax, L
    return best_axis


@dataclass
class PipeFeature:
    """배관 1개의 형태 특징(Phase 1 산출). 유사도·번들 게이트 입력."""

    pipe: ExistingPipe
    code: str                       # Arrow Coding(R/H/D)
    n_bends: int                    # 수직·수평 꺾임 수
    units: list[Vec3]               # 리샘플 단위 방향벡터(길이 RESAMPLE_N-1)
    units_rev: list[Vec3]           # 역방향(부호 반전·역순) — 배관 양방향 정합용
    total_len: float                # 폴리라인 누적 길이(mm)
    extent: Vec3                    # bbox (dx, dy, dz)
    centroid: Vec3                  # 중심점
    trunk_axis: int                 # 주 수평축(0=x, 1=y)
    runs: list[tuple[int, float]] = field(default_factory=list)


def extract_feature(pipe: ExistingPipe, *, resample_n: int = RESAMPLE_N) -> PipeFeature:
    """ExistingPipe → PipeFeature(Phase 1 전체 특징 추출)."""
    pts = pipe.points
    runs = dir_runs(pts)
    rs = resample_polyline(pts, resample_n)
    units = _seg_units(rs)
    units_rev = [(-u[0], -u[1], -u[2]) for u in reversed(units)]
    total = sum(_dist(pts[i - 1], pts[i]) for i in range(1, len(pts)))
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
    extent = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    centroid = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
    return PipeFeature(
        pipe=pipe, code=arrow_code(pts), n_bends=count_ortho_bends(pts),
        units=units, units_rev=units_rev, total_len=total, extent=extent,
        centroid=centroid, trunk_axis=_trunk_axis(runs), runs=runs,
    )


# ================================================================== Phase 2 — 유사도
def levenshtein(a: str, b: str) -> int:
    """두 문자열의 Levenshtein 편집거리(형태 일치도용)."""
    if a == b:
        return 0
    if not a:
        return len(b)
    if not b:
        return len(a)
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def shape_similarity(fa: PipeFeature, fb: PipeFeature) -> float:
    """형태 일치도(30%) = 1 − Levenshtein / max(len). 둘 다 빈 코드면 1."""
    m = max(len(fa.code), len(fb.code))
    if m == 0:
        return 1.0
    return 1.0 - levenshtein(fa.code, fb.code) / m


def _mean_cos(ua: list[Vec3], ub: list[Vec3]) -> float:
    """정렬된 단위벡터 열의 평균 코사인(영벡터 쌍 제외)."""
    n = min(len(ua), len(ub))
    s, cnt = 0.0, 0
    for i in range(n):
        if _norm(ua[i]) < 1e-9 or _norm(ub[i]) < 1e-9:
            continue
        s += _dot(ua[i], ub[i])      # 이미 단위벡터.
        cnt += 1
    return s / cnt if cnt else 0.0


def direction_similarity(fa: PipeFeature, fb: PipeFeature) -> float:
    """방향성 일치도(30%) = 평균 코사인(배관 양방향 중 큰 값), [0,1] 클램프."""
    c = max(_mean_cos(fa.units, fb.units), _mean_cos(fa.units, fb.units_rev))
    return max(0.0, min(1.0, c))


def length_similarity(fa: PipeFeature, fb: PipeFeature) -> float:
    """길이 일치도(20%) = 1 − |Lₐ−L_b| / max."""
    m = max(fa.total_len, fb.total_len)
    if m < 1e-9:
        return 1.0
    return 1.0 - abs(fa.total_len - fb.total_len) / m


def scale_similarity(fa: PipeFeature, fb: PipeFeature) -> float:
    """물리적 규모 일치도(20%) = 축별 min/max extent 평균."""
    vals = []
    for i in range(3):
        ea, eb = fa.extent[i], fb.extent[i]
        m = max(ea, eb)
        vals.append(1.0 if m < 1e-9 else min(ea, eb) / m)
    return sum(vals) / 3.0


def composite_similarity(fa: PipeFeature, fb: PipeFeature) -> float:
    """4대 지표 가중합 복합 유사도 ∈ [0,1]."""
    return (W_SHAPE * shape_similarity(fa, fb)
            + W_DIR * direction_similarity(fa, fb)
            + W_LEN * length_similarity(fa, fb)
            + W_SCALE * scale_similarity(fa, fb))


# ================================================================== Phase 3 — 그룹화
class UnionFind:
    """경로 클러스터링용 Union-Find(경로 압축 + 랭크)."""

    def __init__(self, n: int):
        self.parent = list(range(n))
        self.rank = [0] * n

    def find(self, x: int) -> int:
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return
        if self.rank[ra] < self.rank[rb]:
            ra, rb = rb, ra
        self.parent[rb] = ra
        if self.rank[ra] == self.rank[rb]:
            self.rank[ra] += 1


@dataclass
class BundleGroup:
    """탐지된 번들 그룹 1개(최종 리포트 행)."""

    group_id: int
    owner_name: str | None
    utility: str | None
    member_guids: list[str]
    n_members: int
    avg_similarity: float
    trunk_z: float
    trunk_xy_spread: float
    pitch_mm: float
    n_ortho_bends: int
    arrow_code: str
    trunk_axis: int = 0          # 주 진행축 0=x · 1=y · 2=z(수직/입상). 2 면 trunk_z 는 랙고도 아님(중앙값).


def _trunk_z(feats: list[PipeFeature]) -> float:
    """멤버들의 수평 런 중점 z 를 길이가중·버킷 최빈으로 → 공용 랙 고도."""
    acc: dict[float, float] = {}
    for f in feats:
        pts = f.pipe.points
        for i in range(1, len(pts)):
            a, b = pts[i - 1], pts[i]
            if abs(b[2] - a[2]) <= DIAG_TOL * math.hypot(b[0] - a[0], b[1] - a[1]):
                L = _dist(a, b)
                if L < 1e-6:
                    continue
                zb = round(((a[2] + b[2]) / 2) / Z_BIN_MM) * Z_BIN_MM
                acc[zb] = acc.get(zb, 0.0) + L
    if not acc:
        return statistics.fmean(f.centroid[2] for f in feats)
    return max(acc.items(), key=lambda kv: kv[1])[0]


def _avg_pair_similarity(mfeats: list["PipeFeature"], max_pairs: int = 200) -> float:
    """번들 멤버 쌍 평균 형태유사도(참고 메트릭). 멤버가 많으면 앞쪽 쌍을 표본화(O(n²) 회피)."""
    n = len(mfeats)
    if n < 2:
        return 1.0
    sims: list[float] = []
    for i in range(n):
        for j in range(i + 1, n):
            sims.append(composite_similarity(mfeats[i], mfeats[j]))
            if len(sims) >= max_pairs:
                return statistics.fmean(sims)
    return statistics.fmean(sims) if sims else 1.0


# ================================================================== v3 — 공간 동시진행 검출
@dataclass
class _Run:
    """배관 1개의 '축정렬 직선 런' 1개 — 레인 유지(±LANE_TOL) 구간.

    pipe_idx : 소속 배관 인덱스(멤버 식별).
    axis     : 진행축 0=x · 1=y · 2=z.
    t0,t1    : 진행축 좌표 구간(t0<t1, mm).
    perp     : 진행축과 직교하는 두 축의 고정 좌표(perp_axes 순서, mm).
    perp_axes: 직교 두 축 인덱스 (a,b)  (a<b).
    length   : 진행 길이(t1-t0).
    """

    pipe_idx: int
    axis: int
    t0: float
    t1: float
    perp: tuple[float, float]
    perp_axes: tuple[int, int]
    length: float


def _extract_runs(pipe_idx: int, pts: list[Vec3]) -> list[_Run]:
    """폴리라인을 '축정렬 직선 런'으로 분해한다(레인 유지 ±LANE_TOL).

    각 세그먼트의 우세축으로 분류하고, 같은 축이 이어지는 동안 직교(perp) 좌표 밴드가 ±LANE_TOL 을
    넘지 않으면 한 런으로 병합한다. 밴드를 넘으면(=레인 이탈/꺾임) 런을 끊는다. 사선(perp 드리프트가
    LANE_TOL 초과) 세그먼트는 깨끗한 런을 못 이루므로 자연히 제외된다(밴드 게이트). 90° 밴딩은 우세축
    전환으로 검출되어 ±2° 오차를 흡수한다(직교 BIM 가정). MIN_RUN_MM 미만 런은 버린다(지터·스텁).
    """
    band = 2.0 * LANE_TOL_MM             # perp 변동 허용 폭(±LANE_TOL → 폭 2×).
    cur: dict | None = None
    raw: list[dict] = []

    def close():
        if cur is not None:
            raw.append(cur)

    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        d = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        if _norm(d) < 1e-6:
            continue
        ax = max(range(3), key=lambda k: abs(d[k]))
        pax = tuple(k for k in range(3) if k != ax)     # (a,b) a<b.
        pmin = [min(a[pax[0]], b[pax[0]]), min(a[pax[1]], b[pax[1]])]
        pmax = [max(a[pax[0]], b[pax[0]]), max(a[pax[1]], b[pax[1]])]
        tlo, thi = (a[ax], b[ax]) if a[ax] <= b[ax] else (b[ax], a[ax])
        if cur is not None and cur["axis"] == ax:
            nmin = [min(cur["pmin"][j], pmin[j]) for j in range(2)]
            nmax = [max(cur["pmax"][j], pmax[j]) for j in range(2)]
            if all(nmax[j] - nmin[j] <= band for j in range(2)):
                cur["tlo"] = min(cur["tlo"], tlo); cur["thi"] = max(cur["thi"], thi)
                cur["pmin"], cur["pmax"] = nmin, nmax
                continue
        close()
        cur = {"axis": ax, "tlo": tlo, "thi": thi, "pmin": pmin, "pmax": pmax, "pax": pax}
    close()

    out: list[_Run] = []
    for r in raw:
        length = r["thi"] - r["tlo"]
        if length < MIN_RUN_MM:
            continue
        # 깨끗한 레인만(perp 밴드 ≤ band) — 사선/드리프트 런 제외.
        if any(r["pmax"][j] - r["pmin"][j] > band for j in range(2)):
            continue
        perp = ((r["pmin"][0] + r["pmax"][0]) / 2.0, (r["pmin"][1] + r["pmax"][1]) / 2.0)
        out.append(_Run(pipe_idx, r["axis"], r["tlo"], r["thi"], perp, r["pax"], length))
    return out


def _overlap(a: _Run, b: _Run) -> float:
    """두 런의 진행방향 겹침 길이(없으면 0 이하)."""
    return min(a.t1, b.t1) - max(a.t0, b.t0)


def _cotravel_components(runs: list[_Run]) -> list[list[_Run]]:
    """같은 sh-좌표 클러스터 내 런들을 '진행 겹침'(≥MIN_OVERLAP)으로 연결된 성분으로 묶는다."""
    n = len(runs)
    uf = UnionFind(n)
    for i in range(n):
        for j in range(i + 1, n):
            if _overlap(runs[i], runs[j]) >= MIN_OVERLAP_MM:
                uf.union(i, j)
    comp: dict[int, list[_Run]] = {}
    for i, r in enumerate(runs):
        comp.setdefault(uf.find(i), []).append(r)
    return list(comp.values())


def _split_equal_spacing(runs: list[_Run], sp_idx: int) -> list[list[_Run]]:
    """동시진행 런들을 spread 축 좌표로 정렬해 '등간격 다발'로 분할(outlier 분리).

    인접 간격이 기준 런 간격(비0 간격의 작은 절반 중앙값)의 PITCH_GAP_FACTOR 배보다 크면 끊는다.
    각 다발은 서로 다른 배관 ≥MIN_RACK_MEMBERS 개여야 채택.
    """
    runs = sorted(runs, key=lambda r: r.perp[sp_idx])
    offs = [r.perp[sp_idx] for r in runs]
    gaps = [offs[i] - offs[i - 1] for i in range(1, len(offs))]
    nz = sorted(g for g in gaps if g > LANE_TOL_MM)
    ref = statistics.median(nz[:max(1, len(nz) // 2)]) if nz else 0.0

    out: list[list[_Run]] = []
    cur = [runs[0]]
    for i in range(1, len(runs)):
        if ref > 1e-6 and gaps[i - 1] > PITCH_GAP_FACTOR * ref:
            if len({r.pipe_idx for r in cur}) >= MIN_RACK_MEMBERS:
                out.append(cur)
            cur = [runs[i]]
        else:
            cur.append(runs[i])
    if len({r.pipe_idx for r in cur}) >= MIN_RACK_MEMBERS:
        out.append(cur)
    return out


def _axis_bundles(runs: list[_Run]) -> list[list[_Run]]:
    """한 진행축의 런들에서 평행 동시진행 다발(런 묶음)을 찾는다.

    직교 두 축 중 한 축을 '공유(sh)', 다른 축을 '간격(sp)'으로 보고:
      sh 좌표 ±LANE_TOL 로 클러스터 → 진행 겹침 성분 → sp 등간격 분할.
    두 (sh,sp) 선택을 모두 시도하되, 동일 멤버집합 다발은 중복 제거한다(평면 랙은 한쪽만 채택됨).
    """
    out: list[list[_Run]] = []
    seen: set[frozenset] = set()
    for sh_idx in (0, 1):
        sp_idx = 1 - sh_idx
        rs = sorted(runs, key=lambda r: r.perp[sh_idx])
        # sh 근접 클러스터.
        cluster: list[_Run] = [rs[0]]
        clusters: list[list[_Run]] = []
        for r in rs[1:]:
            if r.perp[sh_idx] - cluster[-1].perp[sh_idx] <= LANE_TOL_MM:
                cluster.append(r)
            else:
                clusters.append(cluster); cluster = [r]
        clusters.append(cluster)
        for shc in clusters:
            if len(shc) < MIN_RACK_MEMBERS:
                continue
            for comp in _cotravel_components(shc):
                if len(comp) < MIN_RACK_MEMBERS:
                    continue
                for bundle in _split_equal_spacing(comp, sp_idx):
                    key = frozenset(r.pipe_idx for r in bundle)
                    if key in seen:
                        continue
                    seen.add(key)
                    out.append(bundle)
    return out


def _bundle_stats(bundle: list[_Run], feat_by_idx: dict[int, PipeFeature],
                  axis: int) -> tuple[float, float, float]:
    """런 묶음 → (pitch 중앙값, 다발 폭, 트렁크 대표 z). 멤버별 sp 좌표로 pitch 산출."""
    # spread 축 = 두 perp 축 중 분산 큰 축(평면 랙은 한쪽이 펼쳐짐).
    spreads = []
    for j in (0, 1):
        vals = sorted({round(r.perp[j], 3) for r in bundle})
        spreads.append((max(vals) - min(vals)) if len(vals) > 1 else 0.0)
    sp_j = 0 if spreads[0] >= spreads[1] else 1
    # 멤버(배관)별 대표 sp 좌표.
    per_pipe: dict[int, float] = {}
    for r in bundle:
        per_pipe.setdefault(r.pipe_idx, r.perp[sp_j])
    offs = sorted(per_pipe.values())
    spread = offs[-1] - offs[0] if len(offs) > 1 else 0.0
    gaps = [offs[i] - offs[i - 1] for i in range(1, len(offs))]
    nz = [g for g in gaps if g > LANE_TOL_MM]
    pitch = statistics.median(nz) if nz else 0.0
    # 트렁크 z — 수평축이면 멤버 features 의 수평 런 최빈 z, 수직축(z 진행)이면 런 중앙 z.
    idxs = list(per_pipe.keys())
    if axis == 2:
        trunk_z = statistics.fmean((r.t0 + r.t1) / 2.0 for r in bundle)
    else:
        trunk_z = _trunk_z([feat_by_idx[i] for i in idxs])
    return pitch, spread, trunk_z


def detect_bundles(
    pipes: list[ExistingPipe],
    *,
    threshold: float = SIM_THRESHOLD,
    min_bends: int = MIN_BENDS,
    pitch_cv_max: float = PITCH_CV_MAX,
) -> list[BundleGroup]:
    """기존배관 리스트에서 번들 그룹을 탐지한다(v3 — 공간 동시진행 검출).

    [흐름]
      ① 특징 추출(유사도 메트릭용) + 런 분해(_extract_runs, 레인 유지 ±LANE_TOL)
      → ② (owner_name, utility) 키 pre-filter
      → ③ 축별 평행 동시진행 다발 검출(_axis_bundles: sh 공유 ±LANE_TOL · 진행 겹침 · sp 등간격)
      → ④ 코너 병합(멤버 ≥2 공유 다발을 한 그룹으로 — 같은 배관들이 꺾여 이어지는 부분)
      → ⑤ trunk_axis·z·pitch·spread·꺾임·대표코드 산출.

    ※ v3 — 수평(x·y) 뿐 아니라 수직(z, 입상/리저) 다발도 검출하고, 꺾임은 필수가 아니다(직선 평행도
       번들). '레인 유지 ±10mm'(진행 중 직교좌표 일정)로 깨끗한 평행 런만 묶는다. threshold·pitch_cv_max
       인자는 무시(CLI 호환). min_bends 기본 0(>0 이면 꺾임 게이트로 직선 다발 제외).
    """
    feats = [extract_feature(p) for p in pipes if len(p.points) >= 2]
    # 런 분해 — feats 순서(=배관 인덱스) 기준.
    feat_by_idx = {i: f for i, f in enumerate(feats)}
    runs_by_idx: dict[int, list[_Run]] = {
        i: _extract_runs(i, f.pipe.points) for i, f in enumerate(feats)
    }

    # ② (장비, 유틸) 키로 pre-filter.
    by_key: dict[tuple, list[int]] = {}
    for i, f in enumerate(feats):
        by_key.setdefault((f.pipe.owner_name, f.pipe.utility), []).append(i)

    groups: list[BundleGroup] = []
    gid = 0
    for (owner, util), idxs in by_key.items():
        if len(idxs) < MIN_RACK_MEMBERS:
            continue
        # ③ 키 내 모든 런을 축별로 모아 평행 다발 검출.
        by_axis: dict[int, list[_Run]] = {0: [], 1: [], 2: []}
        for i in idxs:
            for r in runs_by_idx[i]:
                by_axis[r.axis].append(r)

        raw_bundles: list[tuple[int, set[int], list[_Run]]] = []  # (axis, member set, runs)
        for ax in (0, 1, 2):
            arr = by_axis[ax]
            if len(arr) < MIN_RACK_MEMBERS:
                continue
            for bundle in _axis_bundles(arr):
                members = {r.pipe_idx for r in bundle}
                if len(members) >= MIN_RACK_MEMBERS:
                    raw_bundles.append((ax, members, bundle))

        if not raw_bundles:
            continue

        # ④ 코너 병합 — 멤버를 ≥2 공유하는 다발은 같은 물리 번들(꺾여 이어짐)로 union.
        nb = len(raw_bundles)
        uf = UnionFind(nb)
        for a in range(nb):
            for b in range(a + 1, nb):
                if len(raw_bundles[a][1] & raw_bundles[b][1]) >= 2:
                    uf.union(a, b)
        merged: dict[int, list[int]] = {}
        for a in range(nb):
            merged.setdefault(uf.find(a), []).append(a)

        for comp in merged.values():
            members: set[int] = set()
            all_runs: list[_Run] = []
            for a in comp:
                members |= raw_bundles[a][1]
                all_runs.extend(raw_bundles[a][2])
            if len(members) < MIN_RACK_MEMBERS:
                continue
            # ⑤ 주 진행축 = 총 런 길이가 가장 큰 축.
            len_by_axis = {0: 0.0, 1: 0.0, 2: 0.0}
            for r in all_runs:
                len_by_axis[r.axis] += r.length
            trunk_axis = max((0, 1, 2), key=lambda k: len_by_axis[k])
            trunk_runs = [r for r in all_runs if r.axis == trunk_axis]
            mfeats = [feat_by_idx[i] for i in members]
            med_bends = int(statistics.median(f.n_bends for f in mfeats))
            if med_bends < min_bends:
                continue
            pitch, spread, trunk_z = _bundle_stats(trunk_runs, feat_by_idx, trunk_axis)
            codes = [f.code for f in mfeats]
            rep_code = statistics.mode(codes) if codes else ""
            groups.append(BundleGroup(
                group_id=gid, owner_name=owner, utility=util,
                member_guids=[feat_by_idx[i].pipe.route_path_guid for i in members
                              if feat_by_idx[i].pipe.route_path_guid],
                n_members=len(members), avg_similarity=_avg_pair_similarity(mfeats),
                trunk_z=trunk_z, trunk_xy_spread=spread, pitch_mm=pitch,
                n_ortho_bends=med_bends, arrow_code=rep_code, trunk_axis=trunk_axis,
            ))
            gid += 1

    groups.sort(key=lambda g: (-g.n_members, -g.avg_similarity))
    for new_id, g in enumerate(groups):
        g.group_id = new_id
    return groups


# ================================================================== 리포트·적재
def report(groups: list[BundleGroup]) -> str:
    """탐지 번들을 표로 요약(검수용)."""
    axis_n = {0: "x", 1: "y", 2: "z"}
    n_vert = sum(1 for g in groups if g.trunk_axis == 2)
    lines = [f"탐지 번들 그룹: {len(groups)}  (수직/입상 {n_vert})", ""]
    lines.append(f"  {'gid':>3} {'owner':18} {'util':10} {'n':>3} {'ax':>2} {'avg~':>5} "
                 f"{'bends':>5} {'pitch':>8} {'spread':>8} {'trunk_z':>9}  code")
    for g in groups:
        lines.append(
            f"  {g.group_id:3d} {(g.owner_name or '')[:18]:18} {(g.utility or '')[:10]:10} "
            f"{g.n_members:3d} {axis_n.get(g.trunk_axis, '?'):>2} {g.avg_similarity:5.2f} "
            f"{g.n_ortho_bends:5d} {g.pitch_mm:8.0f} {g.trunk_xy_spread:8.0f} "
            f"{g.trunk_z:9.0f}  {g.arrow_code[:20]}")
    return "\n".join(lines)


_SCHEMA_PATH = "db/schema/route_bundle_group.sql"


def apply_schema(conn) -> None:
    """route_bundle_group 스키마를 적용한다(파일 실행)."""
    import os
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    path = os.path.join(here, *_SCHEMA_PATH.split("/"))
    with open(path, "r", encoding="utf-8") as fh:
        sql = fh.read()
    cur = conn.cursor()
    cur.execute(sql)
    conn.commit()


def write_db(source_file: str, groups: list[BundleGroup], conn) -> int:
    """탐지 결과를 route_bundle_group 에 적재(기존 source_file 행 정리 후)."""
    cur = conn.cursor()
    cur.execute('DELETE FROM route_bundle_group WHERE source_file=%s', (source_file,))
    for g in groups:
        cur.execute(
            'INSERT INTO route_bundle_group '
            '(source_file, group_id, owner_name, utility, n_members, avg_similarity, '
            ' trunk_z, trunk_xy_spread, pitch_mm, n_ortho_bends, arrow_code, trunk_axis, member_guids) '
            'VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)',
            (source_file, g.group_id, g.owner_name, g.utility, g.n_members,
             g.avg_similarity, g.trunk_z, g.trunk_xy_spread, g.pitch_mm,
             g.n_ortho_bends, g.arrow_code, g.trunk_axis, g.member_guids))
    conn.commit()
    return len(groups)


@dataclass
class BundleTemplate:
    """(owner, utility) 키별 대표 번들 패턴 — 신규 배관설계 활용용(route_bundle_template 미러)."""

    owner_name: str | None
    utility: str | None
    trunk_zs: list[float]       # 공용 트렁크 고도 후보(여러 번들이면 모두, mm)
    pitch_mm: float             # 대표 이격간격(중앙값, mm)
    trunk_xy_spread: float      # 대표 다발 폭(중앙값, mm)
    n_members: int              # 키 내 총 멤버 수
    arrow_code: str             # 대표 형태 코드
    n_groups: int               # 키 내 번들 수


def aggregate_templates(groups: list[BundleGroup]) -> list[BundleTemplate]:
    """탐지 번들들을 (owner, utility) 키로 집계해 대표 템플릿을 만든다(route_bundle_template 와 동일 규약).

    한 키에 번들이 여럿이면 트렁크 고도를 모두 모으고(공용 랙 후보), pitch/spread 는 중앙값,
    형태코드는 최빈을 대표로 한다. 신규 배관설계 시 키로 조회해 트렁크 z·pitch 를 활용한다.
    """
    by_key: dict[tuple, list[BundleGroup]] = {}
    for g in groups:
        by_key.setdefault((g.owner_name, g.utility), []).append(g)
    out: list[BundleTemplate] = []
    for (owner, util), gs in by_key.items():
        # trunk_zs = '수평 트렁크 랙 고도'만(수직 입상 번들의 trunk_z 는 중앙값이라 랙고도 아님 → 제외).
        zs = sorted({round(g.trunk_z) * 1.0 for g in gs if g.trunk_axis != 2})
        codes = [g.arrow_code for g in gs]
        out.append(BundleTemplate(
            owner_name=owner, utility=util, trunk_zs=zs,
            pitch_mm=statistics.median(g.pitch_mm for g in gs),
            trunk_xy_spread=statistics.median(g.trunk_xy_spread for g in gs),
            n_members=sum(g.n_members for g in gs),
            arrow_code=statistics.mode(codes) if codes else "",
            n_groups=len(gs),
        ))
    out.sort(key=lambda t: -t.n_members)
    return out


def suggest_bundle(templates: list[BundleTemplate], owner: str | None,
                   utility: str | None) -> BundleTemplate | None:
    """신규 배관(owner, utility)에 적용할 번들 템플릿을 조회. (owner,util)→(util) 폴백, 미스면 None."""
    exact = [t for t in templates if t.owner_name == owner and t.utility == utility]
    if exact:
        return exact[0]
    # 유틸리티 단위 폴백 — 같은 유틸 번들들의 트렁크 고도 합집합.
    same_util = [t for t in templates if t.utility == utility]
    if not same_util:
        return None
    zs = sorted({z for t in same_util for z in t.trunk_zs})
    return BundleTemplate(
        owner_name=owner, utility=utility, trunk_zs=zs,
        pitch_mm=statistics.median([t.pitch_mm for t in same_util]),
        trunk_xy_spread=statistics.median([t.trunk_xy_spread for t in same_util]),
        n_members=sum(t.n_members for t in same_util),
        arrow_code=statistics.mode([t.arrow_code for t in same_util]),
        n_groups=sum(t.n_groups for t in same_util),
    )


def load_templates(source_file: str, conn) -> list[BundleTemplate]:
    """route_bundle_template 뷰에서 한 프로젝트의 대표 번들 템플릿을 읽는다(신규설계 활용 조회)."""
    cur = conn.cursor()
    cur.execute(
        'SELECT owner_name, utility, trunk_zs, pitch_mm, trunk_xy_spread, '
        'n_members, arrow_code, n_groups FROM route_bundle_template WHERE source_file=%s',
        (source_file,))
    out: list[BundleTemplate] = []
    for owner, util, zs, pitch, spread, nmem, code, ngrp in cur.fetchall():
        out.append(BundleTemplate(
            owner_name=owner, utility=util,
            trunk_zs=[float(z) for z in (zs or [])],
            pitch_mm=float(pitch or 0.0), trunk_xy_spread=float(spread or 0.0),
            n_members=int(nmem or 0), arrow_code=code or "", n_groups=int(ngrp or 0)))
    return out


def detect_project(source_file: str, config: PgConnConfig | None = None, conn=None,
                   **kwargs) -> list[BundleGroup]:
    """한 프로젝트의 기존배관을 로드해 번들을 탐지한다."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        bbox = route_db.project_xy_bbox(source_file, conn=conn)
        pipes = route_db.load_existing_pipes(source_file, conn=conn, xy_bbox=bbox)
        return detect_bundles(pipes, **kwargs)
    finally:
        if own:
            conn.close()


def list_source_files(conn) -> list[str]:
    """space_project_map 의 모든 프로젝트 source_file(중복·NULL 제거, 정렬). DB 전체 처리용."""
    cur = conn.cursor()
    cur.execute(
        "SELECT DISTINCT source_file FROM space_project_map "
        "WHERE source_file IS NOT NULL ORDER BY source_file")
    return [str(r[0]) for r in cur.fetchall()]


# ================================================================== CLI
def _main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(description="그룹(번들) 배관 탐지 — 기하 유사도 분석")
    ap.add_argument("--project", type=int, default=None, help="project_id (space_project_map)")
    ap.add_argument("--source", default=None, help="SOURCE_FILE 직접 지정")
    ap.add_argument("--all", action="store_true",
                    help="DB 전체 — space_project_map 의 모든 프로젝트를 순회 처리")
    ap.add_argument("--threshold", type=float, default=SIM_THRESHOLD, help="유사도 임계(기본 0.70)")
    ap.add_argument("--min-bends", type=int, default=MIN_BENDS, help="번들 최소 꺾임(기본 2)")
    ap.add_argument("--pitch-cv", type=float, default=PITCH_CV_MAX, help="동일 이격 허용 CV(기본 0.30)")
    ap.add_argument("--report", action="store_true", help="콘솔 리포트 출력")
    ap.add_argument("--templates", action="store_true",
                    help="(owner,util) 대표 번들 템플릿 출력(신규설계 활용용)")
    ap.add_argument("--write-db", action="store_true", help="route_bundle_group 적재(스키마 자동)")
    ap.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = ap.parse_args(argv)

    overrides = {}
    if args.dbname:
        overrides["dbname"] = args.dbname
    config = PgConnConfig.from_env(**overrides)

    def process_one(conn, sf: str, *, indent: str = "") -> int:
        """source_file 하나를 탐지·리포트·적재한다. 반환: 탐지 그룹 수."""
        groups = detect_project(
            sf, conn=conn, threshold=args.threshold,
            min_bends=args.min_bends, pitch_cv_max=args.pitch_cv)
        print(f"{indent}탐지 번들 그룹 {len(groups)}개 "
              f"(임계 {args.threshold} · 최소꺾임 {args.min_bends} · pitchCV≤{args.pitch_cv})")

        if args.report or not (args.write_db or args.templates):
            print()
            print(report(groups))

        if args.templates:
            tpls = aggregate_templates(groups)
            print(f"\n{indent}대표 번들 템플릿(신규설계 활용): {len(tpls)} 키")
            print(f"{indent}  {'owner':18} {'util':10} {'grp':>3} {'mem':>3} {'pitch':>7}  trunk_zs(mm)")
            for t in tpls:
                zs = ",".join(f"{z:.0f}" for z in t.trunk_zs[:6])
                print(f"{indent}  {(t.owner_name or '')[:18]:18} {(t.utility or '')[:10]:10} "
                      f"{t.n_groups:3d} {t.n_members:3d} {t.pitch_mm:7.0f}  {zs}")

        if args.write_db:
            n = write_db(sf, groups, conn)
            print(f"{indent}DB 적재: {n}개 그룹 (route_bundle_group)")
        return len(groups)

    conn = config.connect()
    try:
        if args.write_db:
            apply_schema(conn)   # 전체/단일 공통으로 스키마는 1회만 적용.

        # ── DB 전체(--all): 모든 프로젝트 source_file 순회 ──
        if args.all:
            sources = list_source_files(conn)
            print(f"DB 전체 처리: 프로젝트 {len(sources)}개\n")
            total_groups = 0
            for i, sf in enumerate(sources, 1):
                print(f"[{i}/{len(sources)}] source_file = {sf}")
                try:
                    total_groups += process_one(conn, sf, indent="  ")
                except Exception as e:   # 한 프로젝트 실패가 전체를 멈추지 않게.
                    print(f"  ! 실패: {e}")
                print()
            print(f"=== 완료: {len(sources)}개 프로젝트 · 총 {total_groups}개 번들 그룹 ===")
            if args.write_db:
                print("    (route_bundle_group + 템플릿 뷰 route_bundle_template 갱신)")
            return 0

        # ── 단일 프로젝트(--project / --source) ──
        if args.source:
            sf = args.source
        elif args.project is not None:
            sf = route_db.resolve_source_file(args.project, conn=conn)
        else:
            ap.error("--project · --source · --all 중 하나가 필요합니다.")
            return 2
        print(f"source_file = {sf}")
        process_one(conn, sf)
        if args.write_db:
            print("    + 템플릿 뷰(route_bundle_template)")
    finally:
        conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

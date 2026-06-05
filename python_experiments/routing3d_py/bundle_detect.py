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

  번들(그룹) 배관 정의 = 여러 배관이 '동일 이격간격'으로 나란히 가면서, 각 배관이 '2번 이상의
  수직·수평 꺾임'을 공유하는 묶음(파이프 랙 다발). 두 조건을 모두 만족해야 번들로 인정한다.

[3단계 파이프라인]  (개발계획: docs/routing3d_bundle_detection_plan.md)
  Phase 1  개별 경로 특징 추출 : 방향 런 압축 → Arrow Coding(R/H/D) · 꺾임 수 · 방향벡터 · 길이 · 규모.
  Phase 2  복합 유사도(4대 지표): 형태 30%(Levenshtein) + 방향 30%(코사인) + 길이 20% + 규모 20%.
  Phase 3  그룹화·트렁크 탐지   : (장비,유틸) 키 내 Union-Find(임계 0.70) → 번들 게이트(≥2꺾임 + 동일 pitch)
                                 → 트렁크 z·다발 폭·이격간격 산출.

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
SIM_THRESHOLD = 0.70    # Union-Find union 임계(인포그래픽 70%).
W_SHAPE, W_DIR, W_LEN, W_SCALE = 0.30, 0.30, 0.20, 0.20   # 4대 지표 가중(합=1).
MIN_BENDS = 2           # 번들 게이트: 최소 수직·수평 꺾임 수.
PITCH_CV_MAX = 0.30     # 동일 이격간격 허용 변동계수(std/mean).
RESAMPLE_N = 24         # 방향 유사도 리샘플 세그먼트 수(점 개수).
DIAG_TOL = 0.34         # R/H 판정 tol — 비우세 성분이 우세 성분의 이 비율 이하라야 직교(아니면 D).
Z_BIN_MM = 100.0        # 트렁크 z 최빈 버킷(mm).

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


def _pitch_stats(feats: list[PipeFeature], trunk_axis: int) -> tuple[float, float, float]:
    """멤버 중심선을 주축의 수직 수평축에 투영 → (pitch 중앙값, pitch 변동계수, 다발 폭).

    trunk_axis(0=x,1=y) 와 직교하는 수평축(perp)으로 멤버 centroid offset 을 정렬해
    인접 간격(pitch)을 본다. 멤버 1개면 (0,0,0). 2개면 CV=0(자명한 등간격).
    """
    perp = 1 - trunk_axis               # 0↔1 (x↔y).
    offs = sorted(f.centroid[perp] for f in feats)
    spread = offs[-1] - offs[0]
    if len(offs) < 2:
        return 0.0, 0.0, 0.0
    # 인접 간격 중 '비0'(=실제 평행 레인 간격)만으로 피치·CV 산출. 다수가 공통 트렁크를 공유(간격 0)해도
    #   레인 간격이 의미를 갖도록. 전부 0(완전 공유 트렁크)이면 pitch=0.
    pitches = [offs[i] - offs[i - 1] for i in range(1, len(offs))]
    nz = [p for p in pitches if p > 1e-6]
    if not nz:
        return 0.0, 0.0, spread
    med = statistics.median(nz)
    if len(nz) == 1:
        return med, 0.0, spread
    mean = statistics.fmean(nz)
    cv = (statistics.pstdev(nz) / mean) if mean > 1e-9 else 0.0
    return med, cv, spread


def _pipe_trunk_z(f: "PipeFeature") -> float:
    """배관 1개의 '수평 트렁크 런' 최빈 z(랙 높이). _trunk_z 단일 적용."""
    return _trunk_z([f])


# 큰 갭(다른 랙)으로 분할할 임계 — 인접 간격이 런 중앙값의 이 배수보다 크면 끊는다.
PITCH_GAP_FACTOR = 2.5
# 한 랙(번들)으로 인정할 최소 멤버 수.
MIN_RACK_MEMBERS = 2
# 같은 랙 높이로 볼 z-근접 임계(mm) — 연속 trunk_z 간격이 이하면 한 랙(고정 버킷 경계 분할 방지).
Z_MERGE_MM = 400.0


def _split_into_racks(
    mfeats: list["PipeFeature"], trunk_axis: int
) -> list[list["PipeFeature"]]:
    """형태 유사 클러스터(공간상 흩어졌을 수 있음)를 '물리적 서브랙'으로 분할한다.

    ① 트렁크 z-높이를 '근접 군집'(연속 간격 ≤ Z_MERGE_MM)으로 묶어 같은 랙 높이끼리 모으고
       (고정 버킷의 경계 분할 방지: 14900/15000/15100 이 한 랙으로),
    ② 같은 높이 안에서 perp(주축의 직교 수평축) 오프셋으로 정렬한 뒤 인접 간격이 런 중앙값의
       PITCH_GAP_FACTOR 배보다 크면 끊어(다른 랙) 균일 피치 런으로 분할.
    → 거대 클러스터가 실제 평행 다발(등간격) 단위로 나뉜다. 각 런(≥MIN_RACK_MEMBERS)이 번들.
    """
    perp = 1 - trunk_axis
    tz = {id(f): _pipe_trunk_z(f) for f in mfeats}     # 배관별 랙 높이(1회 산출).

    # ① z-근접 군집.
    zsorted = sorted(mfeats, key=lambda f: tz[id(f)])
    z_groups: list[list[PipeFeature]] = []
    cur_z = [zsorted[0]]
    for f in zsorted[1:]:
        if tz[id(f)] - tz[id(cur_z[-1])] <= Z_MERGE_MM:
            cur_z.append(f)
        else:
            z_groups.append(cur_z)
            cur_z = [f]
    z_groups.append(cur_z)

    racks: list[list[PipeFeature]] = []
    for group in z_groups:
        if len(group) < MIN_RACK_MEMBERS:
            continue
        # ② perp 오프셋 정렬 후 피치 갭으로 분할.
        group.sort(key=lambda f: f.centroid[perp])
        offs = [f.centroid[perp] for f in group]
        gaps = [offs[i] - offs[i - 1] for i in range(1, len(offs))]
        # 강건 '레인 간격' 기준 = 비0 간격의 작은 절반 중앙값(소수의 큰 랙-경계 갭에 휘둘리지 않음).
        #   인접 간격이 이 기준의 PITCH_GAP_FACTOR 배보다 크면 다른 랙으로 끊는다(outlier 분리).
        nz = sorted(g for g in gaps if g > 1e-6)
        ref = statistics.median(nz[:max(1, len(nz) // 2)]) if nz else 0.0
        cur = [group[0]]
        for i in range(1, len(group)):
            if ref > 1e-6 and gaps[i - 1] > PITCH_GAP_FACTOR * ref:
                if len(cur) >= MIN_RACK_MEMBERS:
                    racks.append(cur)
                cur = [group[i]]
            else:
                cur.append(group[i])
        if len(cur) >= MIN_RACK_MEMBERS:
            racks.append(cur)
    return racks


def _avg_pair_similarity(mfeats: list["PipeFeature"], max_pairs: int = 200) -> float:
    """랙 멤버 쌍 평균 형태유사도(참고 메트릭). 멤버가 많으면 앞쪽 쌍을 표본화(O(n²) 회피)."""
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


def detect_bundles(
    pipes: list[ExistingPipe],
    *,
    threshold: float = SIM_THRESHOLD,
    min_bends: int = MIN_BENDS,
    pitch_cv_max: float = PITCH_CV_MAX,
) -> list[BundleGroup]:
    """기존배관 리스트에서 번들 그룹을 탐지한다(Phase 1~3 전체).

    [흐름]
      ① 특징 추출 → ② (owner_name, utility) 키 pre-filter
      → ③ 공간 랙 검출(트렁크 축별 → z-근접·등간격 _split_into_racks)
      → ④ 번들 게이트(≥min_bends 꺾임 + pitch CV≤pitch_cv_max) → ⑤ 트렁크 z·다발 폭·이격간격 산출.

    ※ v2 — 형태 유사도 Union-Find 폐기. 랙(평행 다발)은 '형태'가 아니라 '공간(같은 트렁크 축·높이에
       등간격 평행)'으로 정의된다. 같은 랙 배관들은 분기 지점이 달라 길이가 제각각이라, 유사도의 길이·규모
       항이 composite 를 임계 아래로 끌어내려 클러스터를 잘게 부쉈다(실측: 4번들·각 2멤버, ALKA 전멸).
       공간 직접 검출로 큰 랙(UPW_S 60·AKWW 50·ALKA 11×다수 …)을 잡는다. threshold 인자는 무시(호환 유지).
    """
    feats = [extract_feature(p) for p in pipes if len(p.points) >= 2]

    # ② (장비, 유틸) 키로 pre-filter.
    by_key: dict[tuple, list[PipeFeature]] = {}
    for f in feats:
        by_key.setdefault((f.pipe.owner_name, f.pipe.utility), []).append(f)

    groups: list[BundleGroup] = []
    gid = 0
    for (owner, util), kfeats in by_key.items():
        if len(kfeats) < MIN_RACK_MEMBERS:
            continue
        # ③ 공간 랙 검출 — 트렁크 주축(0=x,1=y)별로 나눠 z-근접·등간격 랙으로 분할.
        by_axis: dict[int, list[PipeFeature]] = {}
        for f in kfeats:
            by_axis.setdefault(f.trunk_axis, []).append(f)
        for ax, afeats in by_axis.items():
            for mfeats in _split_into_racks(afeats, ax):
                # ④ 번들 게이트 — 꺾임.
                med_bends = int(statistics.median(f.n_bends for f in mfeats))
                if med_bends < min_bends:
                    continue
                pitch, cv, spread = _pitch_stats(mfeats, ax)
                # ④ pitch CV 게이트는 제거 — 실제 플랜트 랙은 공통 트렁크 공유(간격 0)·불규칙 피치라 CV 가
                #   크다(실측 CV 4↑). 균일 피치를 요구하면 진짜 트렁크가 전부 기각된다. cv 는 참고 지표로만.
                #   pitch_cv_max 인자는 호환 유지(미사용). 공간(같은 축·z-근접·perp-런) + 꺾임으로 충분.
                # ⑤ 지표 — 랙 멤버 쌍 평균 형태유사도(참고용 메트릭). 멤버 많으면 비용 위해 표본화.
                avg_sim = _avg_pair_similarity(mfeats)
                codes = [f.code for f in mfeats]
                rep_code = statistics.mode(codes) if codes else ""
                groups.append(BundleGroup(
                    group_id=gid, owner_name=owner, utility=util,
                    member_guids=[f.pipe.route_path_guid for f in mfeats
                                  if f.pipe.route_path_guid],
                    n_members=len(mfeats), avg_similarity=avg_sim,
                    trunk_z=_trunk_z(mfeats), trunk_xy_spread=spread,
                    pitch_mm=pitch, n_ortho_bends=med_bends, arrow_code=rep_code,
                ))
                gid += 1
    groups.sort(key=lambda g: (-g.n_members, -g.avg_similarity))
    # group_id 를 정렬 후 재부여(안정적 번호).
    for new_id, g in enumerate(groups):
        g.group_id = new_id
    return groups


# ================================================================== 리포트·적재
def report(groups: list[BundleGroup]) -> str:
    """탐지 번들을 표로 요약(검수용)."""
    lines = [f"탐지 번들 그룹: {len(groups)}", ""]
    lines.append(f"  {'gid':>3} {'owner':18} {'util':10} {'n':>3} {'avg~':>5} "
                 f"{'bends':>5} {'pitch':>8} {'spread':>8} {'trunk_z':>9}  code")
    for g in groups:
        lines.append(
            f"  {g.group_id:3d} {(g.owner_name or '')[:18]:18} {(g.utility or '')[:10]:10} "
            f"{g.n_members:3d} {g.avg_similarity:5.2f} {g.n_ortho_bends:5d} "
            f"{g.pitch_mm:8.0f} {g.trunk_xy_spread:8.0f} {g.trunk_z:9.0f}  {g.arrow_code[:20]}")
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
            ' trunk_z, trunk_xy_spread, pitch_mm, n_ortho_bends, arrow_code, member_guids) '
            'VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)',
            (source_file, g.group_id, g.owner_name, g.utility, g.n_members,
             g.avg_similarity, g.trunk_z, g.trunk_xy_spread, g.pitch_mm,
             g.n_ortho_bends, g.arrow_code, g.member_guids))
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
        zs = sorted({round(g.trunk_z) * 1.0 for g in gs})
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

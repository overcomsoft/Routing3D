r"""기존 설계배관 · 장비 · 덕트 DB 로더 (PostgreSQL) — 패턴 학습 L1

================================================================================
[실행 명령어]  (모두 프로젝트 루트에서)
  # 한 프로젝트의 기존배관/장비/덕트 적재 통계
  .\.venv\Scripts\python.exe -m routing3d_py.route_db --project 6

  # SOURCE_FILE 직접 지정
  .\.venv\Scripts\python.exe -m routing3d_py.route_db --source "CLEAN_..._total.json"
================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
C# ObstacleDbLoader 의 기존배관/장비/덕트 로딩을 Python 으로 1:1 포팅한다(패턴 학습이
Python 쪽에서 기존설계 폴리라인과 장비·덕트 AABB 에 접근하기 위함). 좌표 단위 mm,
라우트=BIM 동일 월드 프레임.

  load_equipment(...)      → TB_BIM_EQUIPMENT     : 장비 박스(AABB, IS_MAIN, NAME)
  load_ducts(...)          → TB_DUCT_LATERAL      : 덕트·레터럴 박스(AABB, UTILITY, CATEGORY)
  load_existing_pipes(...) → TB_ROUTE_PATH 3-join : PoC→종단 폴리라인 + 양 끝 PoC 좌표·유틸·경

[기존배관 3-테이블 조인]
  TB_ROUTE_SEGMENT_DETAIL sd (FROM/TO XYZ)
    JOIN TB_ROUTE_SEGMENTS  s  ON s.SEGMENT_GUID  = sd.SEGMENT_GUID
    JOIN TB_ROUTE_PATH      rp ON rp.ROUTE_PATH_GUID = s.SEGMENT_GUID 의 PATH
    JOIN TB_BIM_EQUIPMENT   eq ON eq.NAME = rp.SOURCE_OWNER_NAME AND IS_MAIN AND SOURCE_FILE
  ORDER BY ROUTE_PATH_GUID, s.ORDER, sd.ORDER 로 폴리라인을 순서대로 잇는다.
  같은 공정의 다른 tool 배관을 거르려고 SOURCE_OWNER_POS(장비 위치)를 장애물 XY bbox 로 필터한다.
================================================================================
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .obstacle_db import PgConnConfig, load_obstacles, obstacles_bounds

Vec3 = tuple[float, float, float]


# ------------------------------------------------------------------ 자료구조

@dataclass(frozen=True)
class EquipmentBox:
    """장비 1건(TB_BIM_EQUIPMENT). AABB + IS_MAIN."""

    name: str | None
    is_main: bool
    lo: Vec3
    hi: Vec3

    def contains(self, p: Vec3, eps: float = 1.0) -> bool:
        return all(self.lo[i] - eps <= p[i] <= self.hi[i] + eps for i in range(3))


@dataclass(frozen=True)
class DuctLateral:
    """덕트/레터럴 1건(TB_DUCT_LATERAL). AABB + UTILITY/CATEGORY."""

    name: str | None
    category: str | None
    utility: str | None
    lo: Vec3
    hi: Vec3

    def contains(self, p: Vec3, eps: float = 1.0) -> bool:
        return all(self.lo[i] - eps <= p[i] <= self.hi[i] + eps for i in range(3))


@dataclass
class ExistingPipe:
    """기존 설계배관 1개(TB_ROUTE_PATH). PoC→종단 폴리라인 + 메타.

    필드:
        points        : 월드 mm 폴리라인(순서대로, 연속 중복점 제거).
        source_pos    : 출발 PoC(SOURCE_POS) 좌표. None 가능.
        target_pos    : 종단 PoC(TARGET_POS) 좌표. None 가능.
        utility/group : SOURCE_UTILITY / UTILITY_GROUP.
        diameter_mm   : SOURCE_SIZE 파싱 외경(mm). 미상 0.
        route_path_guid : 원본 경로 GUID(출처).
    """

    route_path_guid: str | None = None
    utility: str | None = None
    group: str | None = None
    owner_name: str | None = None        # SOURCE_OWNER_NAME(출발 장비명) — 그룹배관 탐지 키.
    diameter_mm: float = 0.0
    source_pos: Vec3 | None = None
    target_pos: Vec3 | None = None
    points: list[Vec3] = field(default_factory=list)


# ------------------------------------------------------------------ source_file 해석

def resolve_source_file(project_id: int, config: PgConnConfig | None = None, conn=None) -> str:
    """space_project_map 에서 project_id → source_file 을 조회한다."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute("SELECT source_file FROM space_project_map WHERE project_id=%s", (project_id,))
        row = cur.fetchone()
        if not row or row[0] is None:
            raise ValueError(f"project_id={project_id} 가 space_project_map 에 없습니다.")
        return str(row[0])
    finally:
        if own:
            conn.close()


# ------------------------------------------------------------------ 장비 / 덕트

def load_equipment(source_file: str, config: PgConnConfig | None = None, conn=None,
                   main_only: bool = False) -> list[EquipmentBox]:
    """TB_BIM_EQUIPMENT 에서 장비 박스를 로드한다(퇴화 박스 스킵)."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        sql = ('SELECT "NAME","IS_MAIN","MIN_X","MIN_Y","MIN_Z","MAX_X","MAX_Y","MAX_Z" '
               'FROM "TB_BIM_EQUIPMENT" WHERE "SOURCE_FILE"=%s')
        if main_only:
            sql += ' AND "IS_MAIN"=true'
        cur.execute(sql, (source_file,))
        out: list[EquipmentBox] = []
        for name, is_main, mnx, mny, mnz, mxx, mxy, mxz in cur.fetchall():
            if mxx <= mnx or mxy <= mny or mxz <= mnz:
                continue
            out.append(EquipmentBox(
                name=name, is_main=bool(is_main),
                lo=(float(mnx), float(mny), float(mnz)),
                hi=(float(mxx), float(mxy), float(mxz)),
            ))
        return out
    finally:
        if own:
            conn.close()


def load_ducts(source_file: str, config: PgConnConfig | None = None, conn=None) -> list[DuctLateral]:
    """TB_DUCT_LATERAL 에서 덕트·레터럴 박스를 로드한다."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute(
            'SELECT "NAME","CATEGORY","UTILITY","MIN_X","MIN_Y","MIN_Z","MAX_X","MAX_Y","MAX_Z" '
            'FROM "TB_DUCT_LATERAL" WHERE "SOURCE_FILE"=%s', (source_file,))
        out: list[DuctLateral] = []
        for name, cat, util, mnx, mny, mnz, mxx, mxy, mxz in cur.fetchall():
            lo = (float(mnx), float(mny), float(mnz))
            hi = (float(mxx), float(mxy), float(mxz))
            out.append(DuctLateral(name=name, category=cat, utility=util, lo=lo, hi=hi))
        return out
    finally:
        if own:
            conn.close()


# ------------------------------------------------------------------ 기존배관

def parse_pipe_size_mm(size: str | None) -> float:
    """배관 호칭경 문자열 → 외경 근사(mm). C# ParsePipeSizeMm 포팅.

    예: '40A'→40, '150A'→150(A=DN mm), '1/2B'→12.7, '1B'→25.4(B=inch×25.4).
        레듀서('1/4BX1/2B')는 첫 토큰. 미상/실패 0.
    """
    if not size:
        return 0.0
    tok = re.split(r"[xX*]", size.strip())[0].strip()
    m = re.match(r"^\s*(\d+(?:/\d+)?(?:\.\d+)?)\s*([ABab]?)", tok)
    if not m:
        return 0.0
    num, unit = m.group(1), m.group(2).upper()
    if "/" in num:
        a, b = num.split("/")
        try:
            val = float(a) / float(b)
        except (ValueError, ZeroDivisionError):
            return 0.0
    else:
        try:
            val = float(num)
        except ValueError:
            return 0.0
    return val * 25.4 if unit == "B" else val


def _dist2(a: Vec3, b: Vec3) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


def load_existing_pipes(
    source_file: str,
    config: PgConnConfig | None = None,
    conn=None,
    xy_bbox: tuple[float, float, float, float] | None = None,
) -> list[ExistingPipe]:
    """TB_ROUTE_PATH(3-join) 에서 기존 설계배관 폴리라인을 로드한다(C# LoadExistingPipes 포팅).

    [알고리즘]
      ROUTE_PATH_GUID 별로 SEGMENT.ORDER → SEGMENT_DETAIL.ORDER 순서로 FROM/TO 좌표를
      이어 폴리라인 1개를 만든다(연속 중복점 1mm² 이내 생략 → 퇴화 튜브 방지).
      xy_bbox 가 주어지면 SOURCE_OWNER_POS(장비 위치)를 그 XY 범위로 필터(같은 공정 다른 tool 제외).

    매개변수:
        source_file : 프로젝트 SOURCE_FILE.
        xy_bbox     : (minx, maxx, miny, maxy) 장애물 XY 범위(+마진). None=필터 없음.
    반환값:
        list[ExistingPipe] (points>=2 인 것만).
    """
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        params: list[object] = [source_file]
        bbox_clause = ""
        if xy_bbox is not None:
            minx, maxx, miny, maxy = xy_bbox
            bbox_clause = (' AND rp."SOURCE_OWNER_POSX" BETWEEN %s AND %s'
                           ' AND rp."SOURCE_OWNER_POSY" BETWEEN %s AND %s')
            params += [minx, maxx, miny, maxy]
        sql = (
            'SELECT s."ROUTE_PATH_GUID", rp."UTILITY_GROUP", rp."SOURCE_UTILITY", '
            'sd."FROM_POSX", sd."FROM_POSY", sd."FROM_POSZ", '
            'sd."TO_POSX", sd."TO_POSY", sd."TO_POSZ", '
            'rp."SOURCE_POSX", rp."SOURCE_POSY", rp."SOURCE_POSZ", '
            'rp."TARGET_POSX", rp."TARGET_POSY", rp."TARGET_POSZ", '
            'rp."SOURCE_SIZE", rp."SOURCE_OWNER_NAME" '
            'FROM "TB_ROUTE_SEGMENT_DETAIL" sd '
            'JOIN "TB_ROUTE_SEGMENTS" s ON s."SEGMENT_GUID" = sd."SEGMENT_GUID" '
            'JOIN "TB_ROUTE_PATH" rp ON rp."ROUTE_PATH_GUID" = s."ROUTE_PATH_GUID" '
            'JOIN "TB_BIM_EQUIPMENT" eq ON eq."NAME" = rp."SOURCE_OWNER_NAME" '
            'AND eq."IS_MAIN" = true AND eq."SOURCE_FILE" = %s' + bbox_clause +
            ' ORDER BY s."ROUTE_PATH_GUID", s."ORDER", sd."ORDER"'
        )
        cur.execute(sql, params)

        pipes: list[ExistingPipe] = []
        cur_guid: str | None = None
        cur_pipe: ExistingPipe | None = None

        def add_pt(p: Vec3):
            pts = cur_pipe.points
            if not pts or _dist2(pts[-1], p) > 1.0:
                pts.append(p)

        for row in cur.fetchall():
            g = row[0]
            if g != cur_guid:
                if cur_pipe is not None and len(cur_pipe.points) >= 2:
                    pipes.append(cur_pipe)
                cur_guid = g
                sp = (None if row[9] is None or row[10] is None or row[11] is None
                      else (float(row[9]), float(row[10]), float(row[11])))
                tp = (None if row[12] is None or row[13] is None or row[14] is None
                      else (float(row[12]), float(row[13]), float(row[14])))
                cur_pipe = ExistingPipe(
                    route_path_guid=g,
                    group=row[1], utility=row[2],
                    owner_name=row[16],
                    diameter_mm=parse_pipe_size_mm(row[15]),
                    source_pos=sp, target_pos=tp,
                )
            add_pt((float(row[3]), float(row[4]), float(row[5])))
            add_pt((float(row[6]), float(row[7]), float(row[8])))
        if cur_pipe is not None and len(cur_pipe.points) >= 2:
            pipes.append(cur_pipe)
        return pipes
    finally:
        if own:
            conn.close()


def project_xy_bbox(source_file: str, config: PgConnConfig | None = None, conn=None,
                    margin: float = 1000.0) -> tuple[float, float, float, float] | None:
    """프로젝트 장애물의 XY bbox(+margin) — 기존배관 tool 필터용. 장애물 없으면 None."""
    obstacles = load_obstacles(config or PgConnConfig.from_env(), source_file=source_file, conn=conn)
    if not obstacles:
        return None
    (lx, ly, _), (hx, hy, _) = obstacles_bounds(obstacles)
    return (lx - margin, hx + margin, ly - margin, hy + margin)


# ------------------------------------------------------------------ CLI

def _main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(description="기존배관/장비/덕트 DB 로더")
    ap.add_argument("--project", type=int, default=None, help="project_id (space_project_map)")
    ap.add_argument("--source", default=None, help="SOURCE_FILE 직접 지정")
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
            sf = resolve_source_file(args.project, conn=conn)
        else:
            ap.error("--project 또는 --source 중 하나가 필요합니다.")
            return 2
        print(f"source_file = {sf}")

        eq = load_equipment(sf, conn=conn)
        du = load_ducts(sf, conn=conn)
        bbox = project_xy_bbox(sf, conn=conn)
        pipes = load_existing_pipes(sf, conn=conn, xy_bbox=bbox)

        n_main = sum(1 for e in eq if e.is_main)
        print(f"장비 {len(eq)} (IS_MAIN {n_main}) · 덕트/레터럴 {len(du)} · 기존배관 {len(pipes)}")
        with_pts = sum(len(p.points) for p in pipes)
        with_sp = sum(1 for p in pipes if p.source_pos and p.target_pos)
        print(f"  폴리라인 총 점 {with_pts} · 양끝 PoC 보유 {with_sp}/{len(pipes)}")
        if pipes:
            p0 = pipes[0]
            print(f"  sample: util={p0.utility} grp={p0.group} dia={p0.diameter_mm:.0f} "
                  f"pts={len(p0.points)} src={p0.source_pos} tgt={p0.target_pos}")
        # 유틸리티별 기존배관 분포(상위).
        by_u: dict[str, int] = {}
        for p in pipes:
            by_u[p.utility or "?"] = by_u.get(p.utility or "?", 0) + 1
        top = sorted(by_u.items(), key=lambda kv: -kv[1])[:12]
        print("  유틸리티별 기존배관:", ", ".join(f"{u}:{n}" for u, n in top))
    finally:
        conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

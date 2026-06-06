r"""기존 설계배관 · 장비 · 덕트 DB 로더 (PostgreSQL, DDW_AI_DB) — 패턴 학습 L1

================================================================================
[실행 명령어]  (모두 프로젝트 루트에서)
  # 그룹(툴) 목록
  .\.venv\Scripts\python.exe -m routing3d_py.route_db --list

  # 한 그룹(=프로젝트 순번)의 기존배관/장비/덕트 적재 통계
  .\.venv\Scripts\python.exe -m routing3d_py.route_db --project 1
================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
C# ObstacleDbLoader 의 기존배관/장비/덕트 로딩을 Python 으로 1:1 포팅한다(패턴 학습이
Python 쪽에서 기존설계 폴리라인과 장비·덕트 AABB 에 접근하기 위함). 좌표 단위 mm,
라우트=BIM 동일 월드 프레임. DB 는 DDW_AI_DB(구 AUTOROUTINGV7 폐기).

  list_groups()           → TB_SPACE_GROUP_INFO   : 그룹(툴) 목록(=프로젝트, 1-based 순번)
  load_equipment(bbox)    → TB_EQUIPMENTS         : 장비 박스(AABB, MAIN_SUB_TYPE, INSTANCE_NAME)
  load_ducts(bbox)        → TB_LATERAL_PIPE+TB_DUCT: 덕트·레터럴 박스(AABB, UTILITY, CATEGORY)
  load_existing_pipes(bbox) → TB_ROUTE_PATH 3-join : 출발→종단 폴리라인 + 양 끝 PoC 좌표·유틸·경

[구 DB(AUTOROUTINGV7) → 신 DB(DDW_AI_DB) 매핑]
  · 프로젝트 키 SOURCE_FILE/space_project_map → TB_SPACE_GROUP_INFO(툴 단위) + 그룹 AABB 공간교차.
  · 좌표 MIN_*/MAX_* → AABB_MINX/.../AABB_MAXZ.
  · 장비 TB_BIM_EQUIPMENT(IS_MAIN) → TB_EQUIPMENTS(MAIN_SUB_TYPE='MainTool'=메인).
  · 덕트 TB_DUCT_LATERAL → TB_LATERAL_PIPE + TB_DUCT(분리, CATEGORY 는 테이블명에서 부여).
  · 기존배관 owner_name: 구 SOURCE_OWNER_NAME 폐지 → rp.EQUIPMENT_NAME(소유 장비명, 번들 키).
  · 스코프: 구 장비 조인 → rp.SOURCE_POSX/Y 가 그룹박스 안인 경로만(C# LoadRoutesAndTasks 동일).

[기존배관 3-테이블 조인]
  TB_ROUTE_SEGMENT_DETAIL sd (FROM/TO XYZ)
    JOIN TB_ROUTE_SEGMENTS  s  ON s.SEGMENT_GUID  = sd.SEGMENT_GUID
    JOIN TB_ROUTE_PATH      rp ON rp.ROUTE_PATH_GUID = s.ROUTE_PATH_GUID
  WHERE rp.SOURCE_POSX/Y BETWEEN 그룹박스
  ORDER BY ROUTE_PATH_GUID, s.ORDER, sd.ORDER 로 폴리라인을 순서대로 잇는다.
================================================================================
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .obstacle_db import PgConnConfig

Vec3 = tuple[float, float, float]

# 그룹 AABB 공간교차 시 경계 여유(mm) — C# ObstacleDbLoader.ScopeMarginMm 와 동일.
SCOPE_MARGIN_MM = 500.0


# ------------------------------------------------------------------ 자료구조

@dataclass(frozen=True)
class GroupInfo:
    """그룹(툴) 1건(TB_SPACE_GROUP_INFO) = 본 프로젝트의 '프로젝트' 단위.

    필드:
        project_id : 1-based 순번(콤보/--project 호환). PROCESS·TAG 이름 정렬 순.
        group_id   : TAG_GROUP_ID(문자).
        group_name : TAG_GROUP_NM(예: 'WTNHJ02').
        bay/process: BAY_GROUP_NM / PROCESS_GROUP_NM.
        lo/hi      : 그룹 AABB(mm). 객체 공간 스코프 박스.
    """

    project_id: int
    group_id: str
    group_name: str
    bay: str | None
    process: str | None
    lo: Vec3
    hi: Vec3

    def xy_bbox(self, margin: float = SCOPE_MARGIN_MM) -> tuple[float, float, float, float]:
        """공간 스코프용 XY 박스 (minx, maxx, miny, maxy) (+여유)."""
        return (self.lo[0] - margin, self.hi[0] + margin,
                self.lo[1] - margin, self.hi[1] + margin)

    def __str__(self) -> str:
        return f"[{self.project_id}] {self.group_name} / {self.bay or '?'} / {self.process or '?'}"


@dataclass(frozen=True)
class EquipmentBox:
    """장비 1건(TB_EQUIPMENTS). AABB + is_main(MAIN_SUB_TYPE='MainTool')."""

    name: str | None
    is_main: bool
    lo: Vec3
    hi: Vec3

    def contains(self, p: Vec3, eps: float = 1.0) -> bool:
        return all(self.lo[i] - eps <= p[i] <= self.hi[i] + eps for i in range(3))


@dataclass(frozen=True)
class DuctLateral:
    """덕트/레터럴 1건(TB_DUCT / TB_LATERAL_PIPE). AABB + UTILITY/CATEGORY."""

    name: str | None
    category: str | None       # 'DUCT' / 'LATERAL'(테이블에서 부여).
    utility: str | None
    lo: Vec3
    hi: Vec3

    def contains(self, p: Vec3, eps: float = 1.0) -> bool:
        return all(self.lo[i] - eps <= p[i] <= self.hi[i] + eps for i in range(3))


@dataclass
class ExistingPipe:
    """기존 설계배관 1개(TB_ROUTE_PATH). 출발→종단 폴리라인 + 메타.

    필드:
        points        : 월드 mm 폴리라인(순서대로, 연속 중복점 제거).
        source_pos    : 출발 PoC(SOURCE_POS) 좌표. None 가능.
        target_pos    : 종단 PoC(TARGET_POS) 좌표. None 가능.
        utility/group : SOURCE_UTILITY / UTILITY_GROUP.
        owner_name    : EQUIPMENT_NAME(출발 장비명) — 그룹배관 탐지 키.
        diameter_mm   : SOURCE_SIZE 파싱 외경(mm). 미상 0.
        route_path_guid : 원본 경로 GUID(출처).
    """

    route_path_guid: str | None = None
    utility: str | None = None
    group: str | None = None
    owner_name: str | None = None
    diameter_mm: float = 0.0
    source_pos: Vec3 | None = None
    target_pos: Vec3 | None = None
    points: list[Vec3] = field(default_factory=list)


# ------------------------------------------------------------------ 그룹(프로젝트) 해석

def list_groups(config: PgConnConfig | None = None, conn=None) -> list[GroupInfo]:
    """TB_SPACE_GROUP_INFO 의 모든 그룹(툴)을 GroupInfo 로 반환(공정·이름 순, 1-based 순번).

    C# ObstacleDbLoader.ListProjects 와 동일 정렬/순번 규약.
    """
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute(
            'SELECT "TAG_GROUP_ID","TAG_GROUP_NM","BAY_GROUP_NM","PROCESS_GROUP_NM",'
            '"AABB_MINX","AABB_MINY","AABB_MINZ","AABB_MAXX","AABB_MAXY","AABB_MAXZ" '
            'FROM "TB_SPACE_GROUP_INFO" '
            'ORDER BY "PROCESS_GROUP_NM","TAG_GROUP_NM"')
        out: list[GroupInfo] = []
        seq = 1
        for gid, gnm, bay, proc, mnx, mny, mnz, mxx, mxy, mxz in cur.fetchall():
            out.append(GroupInfo(
                project_id=seq, group_id=str(gid) if gid is not None else "",
                group_name=str(gnm) if gnm is not None else "",
                bay=bay, process=proc,
                lo=(float(mnx or 0), float(mny or 0), float(mnz or 0)),
                hi=(float(mxx or 0), float(mxy or 0), float(mxz or 0)),
            ))
            seq += 1
        return out
    finally:
        if own:
            conn.close()


def resolve_group(project_id: int, config: PgConnConfig | None = None, conn=None) -> GroupInfo:
    """프로젝트 순번(1-based) → GroupInfo. 없으면 ValueError."""
    groups = list_groups(config, conn=conn)
    for g in groups:
        if g.project_id == project_id:
            return g
    raise ValueError(f"project_id={project_id} 가 TB_SPACE_GROUP_INFO 에 없습니다(총 {len(groups)}개).")


def resolve_source_file(project_id: int, config: PgConnConfig | None = None, conn=None) -> str:
    """구 API 호환: 프로젝트 순번 → 그룹명(=구 source_file 자리). 패턴 적재 태깅용."""
    return resolve_group(project_id, config, conn=conn).group_name


# ------------------------------------------------------------------ 장비 / 덕트

# AABB(객체) ∩ 그룹박스(XY) 교차 술어(파라미터 4개: minx,maxx,miny,maxy 순).
_ISECT_XY = ('"AABB_MAXX">=%s AND "AABB_MINX"<=%s '
             'AND "AABB_MAXY">=%s AND "AABB_MINY"<=%s')


def _bbox_params(xy_bbox: tuple[float, float, float, float]) -> list[object]:
    """_ISECT_XY 술어에 바인딩할 (minx,maxx,miny,maxy) 순서 파라미터."""
    minx, maxx, miny, maxy = xy_bbox
    return [minx, maxx, miny, maxy]


def load_equipment(xy_bbox: tuple[float, float, float, float],
                   config: PgConnConfig | None = None, conn=None,
                   main_only: bool = False) -> list[EquipmentBox]:
    """TB_EQUIPMENTS 에서 그룹박스(XY)와 교차하는 장비 박스를 로드한다(퇴화 박스 스킵)."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        sql = ('SELECT "INSTANCE_NAME","MAIN_SUB_TYPE",'
               '"AABB_MINX","AABB_MINY","AABB_MINZ","AABB_MAXX","AABB_MAXY","AABB_MAXZ" '
               'FROM "TB_EQUIPMENTS" WHERE ' + _ISECT_XY)
        if main_only:
            sql += " AND \"MAIN_SUB_TYPE\"='MainTool'"
        cur.execute(sql, _bbox_params(xy_bbox))
        out: list[EquipmentBox] = []
        for name, mst, mnx, mny, mnz, mxx, mxy, mxz in cur.fetchall():
            if mxx <= mnx or mxy <= mny or mxz <= mnz:
                continue
            out.append(EquipmentBox(
                name=name,
                is_main=(mst is not None and str(mst).lower() == "maintool"),
                lo=(float(mnx), float(mny), float(mnz)),
                hi=(float(mxx), float(mxy), float(mxz)),
            ))
        return out
    finally:
        if own:
            conn.close()


def load_ducts(xy_bbox: tuple[float, float, float, float],
               config: PgConnConfig | None = None, conn=None) -> list[DuctLateral]:
    """TB_LATERAL_PIPE + TB_DUCT 에서 그룹박스(XY)와 교차하는 종단객체를 로드한다."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        out: list[DuctLateral] = []
        cur = conn.cursor()
        for table, category in (("TB_LATERAL_PIPE", "LATERAL"), ("TB_DUCT", "DUCT")):
            cur.execute(
                f'SELECT "INSTANCE_NAME","UTILITY",'
                '"AABB_MINX","AABB_MINY","AABB_MINZ","AABB_MAXX","AABB_MAXY","AABB_MAXZ" '
                f'FROM "{table}" WHERE ' + _ISECT_XY, _bbox_params(xy_bbox))
            for name, util, mnx, mny, mnz, mxx, mxy, mxz in cur.fetchall():
                if mxx <= mnx or mxy <= mny or mxz <= mnz:
                    continue
                out.append(DuctLateral(
                    name=name, category=category, utility=util,
                    lo=(float(mnx), float(mny), float(mnz)),
                    hi=(float(mxx), float(mxy), float(mxz)),
                ))
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
    xy_bbox: tuple[float, float, float, float] | None,
    config: PgConnConfig | None = None,
    conn=None,
) -> list[ExistingPipe]:
    """TB_ROUTE_PATH(3-join) 에서 기존 설계배관 폴리라인을 로드한다(C# LoadRoutesAndTasks 포팅).

    [알고리즘]
      ROUTE_PATH_GUID 별로 SEGMENT.ORDER → SEGMENT_DETAIL.ORDER 순서로 FROM/TO 좌표를
      이어 폴리라인 1개를 만든다(연속 중복점 1mm² 이내 생략 → 퇴화 튜브 방지).
      xy_bbox 가 주어지면 SOURCE_POSX/Y(출발 PoC 위치)를 그 XY 범위로 필터(다른 툴 제외).

    매개변수:
        xy_bbox : (minx, maxx, miny, maxy) 그룹박스(+여유). None=필터 없음.
    반환값:
        list[ExistingPipe] (points>=2 인 것만).
    """
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        params: list[object] = []
        bbox_clause = ""
        if xy_bbox is not None:
            minx, maxx, miny, maxy = xy_bbox
            bbox_clause = (' WHERE rp."SOURCE_POSX" BETWEEN %s AND %s'
                           ' AND rp."SOURCE_POSY" BETWEEN %s AND %s')
            params += [minx, maxx, miny, maxy]
        sql = (
            'SELECT s."ROUTE_PATH_GUID", rp."UTILITY_GROUP", rp."SOURCE_UTILITY", '
            'sd."FROM_POSX", sd."FROM_POSY", sd."FROM_POSZ", '
            'sd."TO_POSX", sd."TO_POSY", sd."TO_POSZ", '
            'rp."SOURCE_POSX", rp."SOURCE_POSY", rp."SOURCE_POSZ", '
            'rp."TARGET_POSX", rp."TARGET_POSY", rp."TARGET_POSZ", '
            'rp."SOURCE_SIZE", rp."EQUIPMENT_NAME" '
            'FROM "TB_ROUTE_SEGMENT_DETAIL" sd '
            'JOIN "TB_ROUTE_SEGMENTS" s ON s."SEGMENT_GUID" = sd."SEGMENT_GUID" '
            'JOIN "TB_ROUTE_PATH" rp ON rp."ROUTE_PATH_GUID" = s."ROUTE_PATH_GUID"' +
            bbox_clause +
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


# ------------------------------------------------------------------ CLI

def _main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(description="기존배관/장비/덕트 DB 로더 (DDW_AI_DB)")
    ap.add_argument("--list", action="store_true", help="그룹(툴) 목록 출력")
    ap.add_argument("--project", type=int, default=None, help="그룹 순번(1-based)")
    ap.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = ap.parse_args(argv)

    overrides = {}
    if args.dbname:
        overrides["dbname"] = args.dbname
    config = PgConnConfig.from_env(**overrides)

    conn = config.connect()
    try:
        if args.list or args.project is None:
            for g in list_groups(config, conn=conn):
                print(g)
            return 0

        grp = resolve_group(args.project, conn=conn)
        print(f"group = {grp}")
        bbox = grp.xy_bbox()

        eq = load_equipment(bbox, conn=conn)
        du = load_ducts(bbox, conn=conn)
        pipes = load_existing_pipes(bbox, conn=conn)

        n_main = sum(1 for e in eq if e.is_main)
        print(f"장비 {len(eq)} (MainTool {n_main}) · 덕트/레터럴 {len(du)} · 기존배관 {len(pipes)}")
        with_pts = sum(len(p.points) for p in pipes)
        with_sp = sum(1 for p in pipes if p.source_pos and p.target_pos)
        print(f"  폴리라인 총 점 {with_pts} · 양끝 PoC 보유 {with_sp}/{len(pipes)}")
        if pipes:
            p0 = pipes[0]
            print(f"  sample: util={p0.utility} grp={p0.group} owner={p0.owner_name} "
                  f"dia={p0.diameter_mm:.0f} pts={len(p0.points)} src={p0.source_pos} tgt={p0.target_pos}")
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

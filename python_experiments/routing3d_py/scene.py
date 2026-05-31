"""라우팅 씬(Scene) 데이터 환경 — SpaceAI 프로젝트 단위 로딩
================================================================================

[실행 명령어]  (editable 설치 후 프로젝트 루트에서)
  # ① 프로젝트 목록
  .\\.venv\\Scripts\\python.exe -m routing3d_py.scene --list

  # ② 프로젝트 씬 요약(장애물/메인장비/종단/PoC페어/유틸리티/범위)
  .\\.venv\\Scripts\\python.exe -m routing3d_py.scene --project 6

  # ③ 씬 렌더 — start PoC→end PoC 를 유틸리티별 색으로 (직선 연결)
  .\\.venv\\Scripts\\python.exe -m routing3d_py.scene --project 6 --screenshot python_experiments/out/scene.png

  # ④ 특정 유틸리티만 실제 A* 경로로 라우팅해서 렌더
  .\\.venv\\Scripts\\python.exe -m routing3d_py.scene --project 6 --route --utility "[Gas] PN2" --cell-mm 100 --show

  # ⑤ 단위 테스트
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_scene.py -v

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
SpaceAI 의 PostgreSQL 데이터를 '프로젝트 단위'로 읽어, 직교 A* 라우팅에 필요한 씬을
구성한다. 핵심 산출물은 "메인장비 start PoC → 종단객체 end PoC" 라우팅 작업 목록이며,
이를 유틸리티(예: [Gas] PA, [Chemical] H3PO4)별로 묶어 그룹 라우팅에 쓴다.

[데이터 모델 — DB 역설계로 확인 (AUTOROUTINGV7, 단위 mm)]
--------------------------------------------------------------------------------
  space_project_map(project_id → source_file)   # 프로젝트 식별
        │  source_file 로 아래 테이블들을 필터
        ├── TB_BIM_OBSTACLES   : 장애물 AABB (점유맵 입력)
        ├── TB_BIM_EQUIPMENT(IS_MAIN=true) : 메인장비 박스 + POC_LIST(jsonb)
        │       └ POC_LIST[i] = {pocPosition(=start), utility, utilityGroup,
        │                        isConnected, endPocs:[{endPocPosition(=end),
        │                                              endName, endInstanceGuid}]}
        ├── TB_DUCT_LATERAL    : 종단객체(LATERAL PIPE) 박스 (시각화용)
        └── TB_BIM_SPACE_INFO  : 공간(층) 범위

  → 라우팅 작업(RouteTask) = (start=pocPosition, end=endPocPosition, utility)
    한 PoC 가 여러 endPocs 를 가지면 각각이 별도 작업. (프로젝트6: 115 PoC → 208 작업)
  → 유틸리티 라벨 = "[utilityGroup] utility" (예: "[Gas] PA"). 프로젝트6: 21종.

[전체 흐름]
--------------------------------------------------------------------------------
  list_projects()                         # 프로젝트 목록
  load_scene(project_id) → RoutingScene   # 장애물/장비/종단/작업/범위
        │
        ├── scene.tasks_by_utility()      # 유틸리티별 작업 묶음
        ├── scene.build_occupancy(cell_mm)# 장애물 → 점유맵(A* 입력)
        └── route_tasks(occ, tasks, params)# 작업별 A* 경로
              → viz.render_occupancy(polylines=...)  # 유틸리티별 색 렌더
================================================================================
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

from .astar import AStarResult, astar_weighted
from .cost import RouteParams
from .obstacle_db import (
    DEFAULT_MAX_CELLS,
    Obstacle,
    PgConnConfig,
    build_occupancy,
    load_obstacles,
    obstacles_bounds,
)
from .occupancy import AABB, DenseOccupancyMap, OccupancyMap

Vec3 = tuple[float, float, float]


# ------------------------------------------------------------------ 데이터 모델

@dataclass(frozen=True)
class ProjectInfo:
    """프로젝트 식별 정보 (space_project_map)."""

    project_id: int
    source_file: str
    process: str | None
    equipment_code: str | None

    def __str__(self) -> str:
        return f"[{self.project_id}] {self.process}/{self.equipment_code} — {self.source_file}"


@dataclass(frozen=True)
class EndPoc:
    """종단 PoC(종단객체 쪽 연결점)."""

    name: str | None          # endName, 예: "END_00002_LATERAL PIPE"
    end_type: str | None      # endType, 예: "lateral pipe"
    guid: str | None          # endPocGuid
    instance_guid: str | None # endInstanceGuid → TB_DUCT_LATERAL 인스턴스 연결
    position: Vec3            # endPocPosition (mm)


@dataclass(frozen=True)
class StartPoc:
    """메인장비 쪽 시작 PoC."""

    poc_id: str | None
    name: str | None          # 예: "POC04995"
    position: Vec3            # pocPosition (mm)
    utility: str | None       # 예: "NW"
    utility_group: str | None # 예: "Water"
    is_connected: bool
    end_pocs: tuple[EndPoc, ...]

    @property
    def utility_label(self) -> str:
        """유틸리티 라벨 '[그룹] 유틸' (예: '[Water] NW')."""
        return utility_label(self.utility_group, self.utility)


@dataclass(frozen=True)
class RouteTask:
    """하나의 라우팅 작업: 메인장비 start PoC → 종단 end PoC.

    필드:
        start_mm/end_mm   : 시작/끝 월드좌표(mm).
        utility/utility_group : 유틸리티 이름/그룹.
        start_name/end_name   : PoC 이름.
        end_instance_guid     : 종단객체(LATERAL) 인스턴스 GUID.
    """

    start_mm: Vec3
    end_mm: Vec3
    utility: str | None
    utility_group: str | None
    start_name: str | None
    end_name: str | None
    end_instance_guid: str | None
    size: str | None = None        # 호칭경 원문(예 '40A','1/2B'). POC_LIST[i].size.
    diameter_mm: float = 0.0       # size → mm 파싱 외경 근사(0=미상). parse_pipe_size_mm.

    @property
    def utility_label(self) -> str:
        return utility_label(self.utility_group, self.utility)


def parse_pipe_size_mm(size: str | None) -> float:
    """배관 호칭경 문자열 → 외경 근사(mm). C# ObstacleDbLoader.ParsePipeSizeMm 와 동일 규약.

    규칙:
        '40A','150A'  → A 호칭 = DN(mm) 그대로(40, 150).
        '1/2B','1B'   → B 호칭 = 인치 × 25.4 ('1/2'→12.7, '1'→25.4).
        '1-1/4B'      → 혼합수 인치(1.25 × 25.4).
        '1/4BX1/2B'   → 레듀서는 시작측(첫 토큰) 사용.
        '65','32'     → 단위 문자 없는 숫자는 DN(mm) 로 간주.
        빈값/파싱실패 → 0.0.
    """
    if not size:
        return 0.0
    tok = size.strip().split("X")[0].split("x")[0].strip()
    if not tok:
        return 0.0
    unit = tok[-1].upper()
    if unit == "A":
        try:
            return float(tok[:-1].strip())
        except ValueError:
            return 0.0
    if unit == "B":
        inch = _parse_inch(tok[:-1].strip())
        return inch * 25.4 if inch > 0 else 0.0
    try:
        return float(tok)
    except ValueError:
        return 0.0


def _parse_inch(s: str) -> float:
    """'1/2','3/8','1','1-1/4','1 1/4' 같은 인치 표기를 double 인치로."""
    s = s.strip().replace("-", " ")
    if not s:
        return 0.0
    total, any_ = 0.0, False
    for part in s.split():
        if "/" in part:
            a, _, b = part.partition("/")
            try:
                na, nb = float(a), float(b)
                if nb != 0:
                    total += na / nb
                    any_ = True
            except ValueError:
                pass
        else:
            try:
                total += float(part)
                any_ = True
            except ValueError:
                pass
    return total if any_ else 0.0


@dataclass(frozen=True)
class MainEquipment:
    """메인장비(IS_MAIN) — 박스 + 시작 PoC 목록."""

    eq_id: str | None
    name: str | None
    aabb: AABB | None
    pocs: tuple[StartPoc, ...]


@dataclass(frozen=True)
class TerminalObject:
    """종단객체(TB_DUCT_LATERAL) — 시각화용 박스."""

    object_id: str | None
    name: str | None
    utility: str | None
    aabb: AABB | None


@dataclass
class RoutingScene:
    """프로젝트 하나의 라우팅 씬(데이터 환경).

    필드:
        project          : ProjectInfo.
        bounds_lo/bounds_hi : 씬 공간 범위(mm) — 장애물 전체 AABB 기준.
        obstacles        : 장애물 리스트(점유맵 입력).
        main_equipment   : 메인장비 리스트.
        terminals        : 종단객체 리스트(시각화용).
        tasks            : 라우팅 작업(start→end) 리스트.
    """

    project: ProjectInfo
    bounds_lo: Vec3
    bounds_hi: Vec3
    obstacles: list[Obstacle]
    main_equipment: list[MainEquipment]
    terminals: list[TerminalObject]
    tasks: list[RouteTask]

    # ---- 유틸리티 그룹 ----

    def tasks_by_utility(self) -> dict[str, list[RouteTask]]:
        """작업을 유틸리티 라벨('[그룹] 유틸')별로 묶는다."""
        groups: dict[str, list[RouteTask]] = {}
        for t in self.tasks:
            groups.setdefault(t.utility_label, []).append(t)
        return groups

    def utility_counts(self) -> dict[str, int]:
        """유틸리티별 작업(페어) 수 (내림차순)."""
        counts = {k: len(v) for k, v in self.tasks_by_utility().items()}
        return dict(sorted(counts.items(), key=lambda x: -x[1]))

    # ---- 점유맵 ----

    def build_occupancy(
        self,
        cell_mm: float = 50.0,
        *,
        padding_mm: float = 0.0,
        backend: type[OccupancyMap] = DenseOccupancyMap,
        max_cells: int = DEFAULT_MAX_CELLS,
    ):
        """씬의 장애물을 점유맵으로 구성한다(범위=씬 공간 범위).

        반환값: obstacle_db.BuildResult (.occupancy = OccupancyMap).
        """
        region = (self.bounds_lo, self.bounds_hi)
        return build_occupancy(self.obstacles, cell_mm=cell_mm, region=region,
                               padding_mm=padding_mm, backend=backend,
                               max_cells=max_cells)

    def summary(self) -> str:
        """씬 요약 문자열(여러 줄)."""
        lo, hi = self.bounds_lo, self.bounds_hi
        size = tuple(round(hi[i] - lo[i], 1) for i in range(3))
        lines = [
            f"프로젝트 {self.project}",
            f"  공간 범위 mm: {tuple(round(v,1) for v in lo)} → {tuple(round(v,1) for v in hi)}  (크기 {size})",
            f"  장애물 {len(self.obstacles):,} | 메인장비 {len(self.main_equipment)} | "
            f"종단객체 {len(self.terminals)} | 라우팅 작업(PoC 페어) {len(self.tasks)}",
            f"  유틸리티 {len(self.utility_counts())}종:",
        ]
        for label, n in self.utility_counts().items():
            lines.append(f"    {label}: {n}")
        return "\n".join(lines)


# ------------------------------------------------------------------ 헬퍼

def utility_label(group: str | None, util: str | None) -> str:
    """'[그룹] 유틸' 라벨 생성. None 은 '?' 로."""
    return f"[{group or '?'}] {util or '?'}"


# 유틸리티 색 팔레트(결정적 배정용). 21+종을 구분 가능한 색.
_PALETTE: tuple[str, ...] = (
    "red", "blue", "green", "orange", "purple", "deeppink", "teal", "gold",
    "saddlebrown", "cyan", "magenta", "limegreen", "navy", "crimson", "darkorange",
    "mediumspringgreen", "slateblue", "tomato", "seagreen", "royalblue", "violet",
    "olive", "indianred", "turquoise",
)


def utility_colors(labels: list[str]) -> dict[str, str]:
    """유틸리티 라벨 목록에 결정적으로 색을 배정한다(정렬 순 → 팔레트 순환)."""
    return {lab: _PALETTE[i % len(_PALETTE)] for i, lab in enumerate(sorted(labels))}


# ------------------------------------------------------------------ 로딩

def list_projects(config: PgConnConfig | None = None, conn=None) -> list[ProjectInfo]:
    """space_project_map 에서 프로젝트 목록을 읽는다(project_id 오름차순)."""
    config = config or PgConnConfig()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute(
            "SELECT project_id, source_file, process, equipment_code "
            "FROM space_project_map ORDER BY project_id"
        )
        return [ProjectInfo(*row) for row in cur.fetchall()]
    finally:
        if own:
            conn.close()


def _as_list(jsonb_val) -> list:
    """psycopg2 가 jsonb 를 dict/list 로 주지만, str 로 올 경우도 방어적으로 파싱."""
    if jsonb_val is None:
        return []
    if isinstance(jsonb_val, str):
        return json.loads(jsonb_val)
    return jsonb_val


def _aabb_or_none(mn: Vec3, mx: Vec3) -> AABB | None:
    """세 축 모두 max>min 이면 AABB, 아니면 None(퇴화)."""
    if all(mx[i] > mn[i] for i in range(3)):
        return AABB(lo=tuple(mn), hi=tuple(mx))
    return None


def load_scene(
    config: PgConnConfig | None = None,
    project_id: int = 1,
    *,
    connected_only: bool = True,
    conn=None,
) -> RoutingScene:
    """프로젝트 하나의 라우팅 씬을 DB 에서 로드한다.

    [알고리즘]
      1) space_project_map 에서 project_id → source_file.
      2) source_file 로 장애물(TB_BIM_OBSTACLES) 로드.
      3) 메인장비(TB_BIM_EQUIPMENT, IS_MAIN) 의 POC_LIST 를 파싱:
         각 PoC 의 pocPosition(start)와 endPocs[].endPocPosition(end) 로 RouteTask 생성.
         connected_only=True 면 isConnected PoC 만.
      4) 종단객체(TB_DUCT_LATERAL) 박스 로드(시각화용).
      5) 공간 범위 = 장애물 전체 AABB (뷰어와 동일 기준).

    매개변수:
        config         : 접속 설정. conn 우선.
        project_id     : space_project_map 의 project_id.
        connected_only : True(기본)면 isConnected PoC 의 작업만 포함.
        conn           : 재사용 연결(선택).
    반환값:
        RoutingScene.
    예외:
        ValueError : project_id 없음 / 장애물 없음(범위 산출 불가).
    """
    config = config or PgConnConfig()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute(
            "SELECT source_file, process, equipment_code "
            "FROM space_project_map WHERE project_id=%s", (project_id,))
        row = cur.fetchone()
        if not row:
            raise ValueError(f"project_id {project_id} not found in space_project_map")
        source_file, process, equipment_code = row
        project = ProjectInfo(project_id, source_file, process, equipment_code)

        obstacles = load_obstacles(config, source_file=source_file, conn=conn)

        # 메인장비 + POC_LIST → 작업
        cur.execute(
            'SELECT "EQ_ID","NAME","MIN_X","MIN_Y","MIN_Z","MAX_X","MAX_Y","MAX_Z","POC_LIST" '
            'FROM "TB_BIM_EQUIPMENT" WHERE "SOURCE_FILE"=%s AND "IS_MAIN"=true', (source_file,))
        main_equipment: list[MainEquipment] = []
        tasks: list[RouteTask] = []
        for eq_id, name, mnx, mny, mnz, mxx, mxy, mxz, poc_list in cur.fetchall():
            aabb = _aabb_or_none((mnx, mny, mnz), (mxx, mxy, mxz))
            pocs: list[StartPoc] = []
            for p in _as_list(poc_list):
                start = tuple(p.get("pocPosition") or (0.0, 0.0, 0.0))
                util = p.get("utility")
                grp = p.get("utilityGroup")
                size = p.get("size")
                dia = parse_pipe_size_mm(size)
                connected = bool(p.get("isConnected"))
                ends: list[EndPoc] = []
                for e in (p.get("endPocs") or []):
                    ep = EndPoc(
                        name=e.get("endName"), end_type=e.get("endType"),
                        guid=e.get("endPocGuid"), instance_guid=e.get("endInstanceGuid"),
                        position=tuple(e.get("endPocPosition") or (0.0, 0.0, 0.0)),
                    )
                    ends.append(ep)
                    if connected or not connected_only:
                        tasks.append(RouteTask(
                            start_mm=start, end_mm=ep.position,
                            utility=util, utility_group=grp,
                            start_name=p.get("name"), end_name=ep.name,
                            end_instance_guid=ep.instance_guid,
                            size=size, diameter_mm=dia,
                        ))
                pocs.append(StartPoc(
                    poc_id=p.get("id"), name=p.get("name"), position=start,
                    utility=util, utility_group=grp, is_connected=connected,
                    end_pocs=tuple(ends),
                ))
            main_equipment.append(MainEquipment(eq_id, name, aabb, tuple(pocs)))

        # 종단객체(시각화용)
        cur.execute(
            'SELECT "OBJECT_ID","NAME","UTILITY","MIN_X","MIN_Y","MIN_Z","MAX_X","MAX_Y","MAX_Z" '
            'FROM "TB_DUCT_LATERAL" WHERE "SOURCE_FILE"=%s', (source_file,))
        terminals = [
            TerminalObject(oid, nm, util, _aabb_or_none((mnx, mny, mnz), (mxx, mxy, mxz)))
            for oid, nm, util, mnx, mny, mnz, mxx, mxy, mxz in cur.fetchall()
        ]

        # 공간 범위 = 장애물 전체 AABB
        if not obstacles:
            raise ValueError(f"project {project_id} has no obstacles; cannot derive bounds")
        lo, hi = obstacles_bounds(obstacles)

        return RoutingScene(
            project=project, bounds_lo=lo, bounds_hi=hi,
            obstacles=obstacles, main_equipment=main_equipment,
            terminals=terminals, tasks=tasks,
        )
    finally:
        if own:
            conn.close()


# ------------------------------------------------------------------ 라우팅

@dataclass
class RoutedTask:
    """라우팅된 작업: 작업 + A* 결과."""

    task: RouteTask
    result: AStarResult


def route_tasks(
    occ: OccupancyMap,
    tasks: list[RouteTask],
    params: RouteParams | None = None,
    *,
    snap_to_free: int = 2,
) -> list[RoutedTask]:
    """작업 목록을 점유맵 위에서 각각 A* 로 라우팅한다.

    start/end 가 점유 셀이면(장비·종단 표면이 점유로 복셀화된 경우) 가까운 빈 셀로
    스냅을 시도한다(snap_to_free 반경). 실패한 작업도 결과(success=False)로 포함한다.

    매개변수:
        occ         : 점유맵.
        tasks       : RouteTask 리스트.
        params      : RouteParams. None 이면 기본값.
        snap_to_free: start/end 점유 시 빈 셀 탐색 반경(셀). 0 이면 스냅 안 함.
    반환값:
        list[RoutedTask].
    """
    params = params or RouteParams(cell_mm=occ.cell_mm)
    out: list[RoutedTask] = []
    for t in tasks:
        s = _snap(occ, occ.to_cell(t.start_mm), snap_to_free)
        g = _snap(occ, occ.to_cell(t.end_mm), snap_to_free)
        res = astar_weighted(occ, s, g, params)
        out.append(RoutedTask(t, res))
    return out


def _snap(occ: OccupancyMap, cell, radius: int):
    """cell 이 점유면 반경 radius 내 가장 가까운 빈 셀을 찾아 반환(없으면 원래 cell)."""
    if not occ.is_blocked(cell):
        return cell
    if radius <= 0:
        return cell
    ci, cj, ck = cell
    best = None
    best_d = None
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            for dk in range(-radius, radius + 1):
                c = (ci + di, cj + dj, ck + dk)
                if not occ.is_blocked(c):
                    d = abs(di) + abs(dj) + abs(dk)
                    if best_d is None or d < best_d:
                        best_d, best = d, c
    return best if best is not None else cell


def scene_polylines(
    scene: RoutingScene,
    occ: OccupancyMap,
    routed: list[RoutedTask] | None = None,
    *,
    colors: dict[str, str] | None = None,
) -> list[tuple[list[Vec3], str, str]]:
    """씬을 viz.render_occupancy(polylines=...) 용 선분 목록으로 변환한다.

    routed 가 주어지면 실제 A* 경로(셀→월드)를, 없으면 start→end 직선을 그린다.
    각 선분은 (점목록 mm, 색, 유틸리티 라벨).

    매개변수:
        scene  : RoutingScene.
        occ    : 셀→월드 변환에 쓸 점유맵.
        routed : route_tasks 결과(선택). None 이면 직선 연결.
        colors : 유틸리티→색 매핑(선택). None 이면 자동 배정.
    반환값:
        list[(points_mm, color, label)].
    """
    colors = colors or utility_colors(list(scene.utility_counts().keys()))
    lines: list[tuple[list[Vec3], str, str]] = []
    if routed is not None:
        for rt in routed:
            if not rt.result.success or not rt.result.path:
                continue
            pts = [occ.to_world(c) for c in rt.result.path]
            label = rt.task.utility_label
            lines.append((pts, colors.get(label, "gray"), label))
    else:
        for t in scene.tasks:
            label = t.utility_label
            lines.append(([t.start_mm, t.end_mm], colors.get(label, "gray"), label))
    return lines


# ------------------------------------------------------------------ CLI 진입점

def _main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(description="SpaceAI 프로젝트 라우팅 씬 로더")
    parser.add_argument("--list", action="store_true", help="프로젝트 목록 출력")
    parser.add_argument("--project", type=int, default=None, help="프로젝트 id")
    parser.add_argument("--cell-mm", type=float, default=50.0, help="셀 크기 mm")
    parser.add_argument("--route", action="store_true", help="작업을 독립 A* 로 라우팅(충돌 허용)")
    parser.add_argument("--multi", action="store_true",
                        help="다중 배관 순차 라우팅(충돌 없이, 깔린 경로를 점유로 추가)")
    parser.add_argument("--priority", default="longest",
                        choices=["longest", "shortest", "utility", "original"],
                        help="--multi 우선순위 규칙(기본 longest)")
    parser.add_argument("--pipe-radius", type=int, default=0,
                        help="--multi 시 깔린 배관 점유 팽창 반경(셀). 기본 0")
    parser.add_argument("--utility", default=None, help="이 유틸리티 라벨만 (예: \"[Gas] PN2\")")
    parser.add_argument("--max-tasks", type=int, default=None, help="라우팅할 최대 작업 수")
    parser.add_argument("--w-turn", type=float, default=500.0, help="회전 비용 mm")
    parser.add_argument("--w-clear", type=float, default=0.0, help="클리어런스 페널티 mm/셀")
    parser.add_argument("--clearance", type=int, default=2, help="클리어런스 반경 셀")
    parser.add_argument("--screenshot", default=None, help="PNG 경로")
    parser.add_argument("--html", default=None, help="인터랙티브 HTML 경로")
    parser.add_argument("--show", action="store_true", help="인터랙티브 창")
    parser.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = parser.parse_args(argv)

    overrides = {"dbname": args.dbname} if args.dbname else {}
    config = PgConnConfig.from_env(**overrides)

    if args.list or args.project is None:
        for p in list_projects(config):
            print(p)
        return 0

    scene = load_scene(config, args.project)
    print(scene.summary())

    if not (args.screenshot or args.html or args.show):
        return 0

    # 점유맵 구성
    occ = scene.build_occupancy(cell_mm=args.cell_mm).occupancy
    print(f"점유맵: 셀 {occ.shape}, 점유 {occ.count_blocked():,}")

    # 라우팅 대상 작업 선택
    tasks = scene.tasks
    if args.utility:
        tasks = [t for t in tasks if t.utility_label == args.utility]
        print(f"유틸리티 '{args.utility}' 작업 {len(tasks)}개")
    if args.max_tasks:
        tasks = tasks[: args.max_tasks]

    routed = None
    if args.route or args.multi:
        params = RouteParams(cell_mm=args.cell_mm, w_turn=args.w_turn,
                             w_clear=args.w_clear, clearance_radius=args.clearance)
        if args.multi:
            from .multi_route import route_sequential
            mr = route_sequential(occ, tasks, params, priority=args.priority,
                                  pipe_radius=args.pipe_radius)
            print(mr.summary())
            print("유틸리티별 성공/전체:")
            for util, plist in sorted(mr.by_utility().items(),
                                      key=lambda x: -len(x[1])):
                ok = sum(1 for p in plist if p.result.success)
                print(f"  {util}: {ok}/{len(plist)}")
            routed = mr.pipes   # PipeResult 는 .task/.result 보유 → scene_polylines 호환
        else:
            routed = route_tasks(occ, tasks, params)
            ok = sum(1 for r in routed if r.result.success)
            print(f"라우팅(독립): {ok}/{len(routed)} 성공")
    scene = RoutingScene(scene.project, scene.bounds_lo, scene.bounds_hi,
                         scene.obstacles, scene.main_equipment, scene.terminals, tasks)

    from .viz import render_occupancy
    polylines = scene_polylines(scene, occ, routed)
    render_occupancy(
        {"obstacles": occ}, opacity=0.12,
        polylines=polylines,
        show=args.show, screenshot=args.screenshot, html=args.html,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

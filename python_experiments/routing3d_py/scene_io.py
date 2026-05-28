"""씬 입출력 (scene.txt I/O) — Phase 1 Step 1.5
================================================================================

[실행 명령어]  (editable 설치 후 프로젝트 루트에서)
  # ① DB 프로젝트 씬을 scene.txt 로 내보내기(경로 결과 포함, 다중 배관)
  .\\.venv\\Scripts\\python.exe -m routing3d_py.scene_io ^
      --project 6 --cell-mm 100 --multi --out python_experiments/out/project6.scene.txt

  # ② scene.txt 읽어 요약 출력 + write→read→write 무손실 자기검증
  .\\.venv\\Scripts\\python.exe -m routing3d_py.scene_io ^
      --in python_experiments/out/project6.scene.txt --roundtrip

  # ③ 단위 테스트
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_scene_io.py -v

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
라우팅 씬(입력)과 탐색 결과(출력)를 사람이 읽을 수 있는 텍스트 파일 `scene.txt`
하나로 직렬화/역직렬화한다. 이 포맷은 Phase 2 명세화·Phase 3 C++ 엔진이 공유할
**입출력 규약(계약)**이며, 본 단계에서 신규 정의·동결한다.

[설계 원칙]
--------------------------------------------------------------------------------
  1) 무손실(round-trip): write → read → write 결과가 바이트 단위로 동일.
     - 실수는 repr() 로 적어 부동소수점 값을 정확히 보존.
     - 선택 문자열의 None 과 빈 문자열("")을 구분: None = 토큰 ``\\N`` (PostgreSQL COPY 관례).
  2) C++ 포팅 친화: 섹션 헤더 + TAB 구분 행. 한 줄 = 한 레코드(또는 한 키-값).
     파서는 단순 상태기계(현재 섹션에 따라 행 해석)면 충분.
  3) 사람 판독: 주석(#)·헤더(@)·섹션([])으로 구조가 한눈에 보인다.
  4) 단위 mm. 셀 크기·원점·격자 크기를 [grid] 헤더에 명시.

[파일 구조 — 3개 레이어(점유/방문/경로)를 포함]
--------------------------------------------------------------------------------
  # Routing3D scene file — units: mm
  @format routing3d-scene
  @version 1

  [grid]                         # 격자 메타 (점유 레이어의 좌표계)
  cell_mm   <float>
  origin    <x> <y> <z>
  shape     <nx> <ny> <nz>

  [params]                       # 비용함수 파라미터 (RouteParams)
  cell_mm <f> / w_turn <f> / w_clear <f> / clearance_radius <i>
  clearance_connectivity <i> / w_tier  z:mm  z:mm ...

  [obstacles] count=N            # 점유 레이어 입력: AABB 박스 목록
  # minx miny minz maxx maxy maxz ost_type name object_id ddworks_type   (TAB, \\N=null)
  ...

  [tasks] count=M                # 라우팅 작업: start→end PoC + 유틸리티
  # sx sy sz gx gy gz utility utility_group start_name end_name end_instance_guid
  ...

  [results] count=K              # (선택) 작업별 탐색 결과 묶음 시작
  [result] task=<idx>            #   지표
  success/length_mm/cost_mm/turns/expanded_nodes/elapsed_ms ...
  [path] task=<idx> count=<n>    #   경로 레이어: i j k (셀)
  ...
  [visited] task=<idx> count=<n> #   방문 레이어: i j k (셀, 선택)
  ...

[자료구조]
  SceneDoc : 위 파일 전체를 담는 메모리 표현(grid/params/obstacles/tasks/results).
             results 는 tasks 와 평행한 list[AStarResult | None].
================================================================================
"""

from __future__ import annotations

import io
from dataclasses import dataclass, field

from .astar import AStarResult
from .cost import RouteParams
from .obstacle_db import Obstacle
from .occupancy import AABB, Cell, DenseOccupancyMap, OccupancyMap
from .scene import RouteTask

Vec3 = tuple[float, float, float]

# 파일 포맷 식별자/버전 — 헤더에 적고 읽을 때 검증한다.
FORMAT_TAG = "routing3d-scene"
FORMAT_VERSION = 1

# None(널)을 빈 문자열과 구분하기 위한 토큰. PostgreSQL COPY 의 NULL 표기와 동일.
NULL_TOKEN = "\\N"


# ==============================================================================
# 메모리 표현
# ==============================================================================
@dataclass
class SceneDoc:
    """scene.txt 파일 한 개에 대응하는 메모리 표현.

    필드:
        cell_mm    : 셀 한 변 길이(mm).
        origin     : 격자 원점 월드좌표(mm) (x, y, z).
        shape      : 격자 셀 개수 (nx, ny, nz).
        params     : 비용함수 파라미터(RouteParams).
        obstacles  : 장애물 AABB 목록(Obstacle). 점유 레이어 입력.
        tasks      : 라우팅 작업 목록(RouteTask).
        results    : tasks 와 평행한 list[AStarResult | None]. 결과 없으면 None.
    """

    cell_mm: float
    origin: Vec3
    shape: tuple[int, int, int]
    params: RouteParams
    obstacles: list[Obstacle]
    tasks: list[RouteTask]
    results: list[AStarResult | None] = field(default_factory=list)

    def summary(self) -> str:
        n_res = sum(1 for r in self.results if r is not None)
        n_ok = sum(1 for r in self.results if r is not None and r.success)
        return (
            f"[scene] 격자 {self.shape} cell={self.cell_mm:g}mm origin={tuple(round(v,1) for v in self.origin)} "
            f"| 장애물 {len(self.obstacles)} | 작업 {len(self.tasks)} | 결과 {n_ok}/{n_res} 성공"
        )


# ------------------------------------------------------------------ 직렬화 헬퍼

def _ff(x: float) -> str:
    """실수를 무손실로 적기 위한 포맷(repr). float 로 캐스팅 후 repr."""
    return repr(float(x))


def _opt(s: str | None) -> str:
    """선택 문자열 → 필드 문자열. None 은 NULL_TOKEN, 그 외엔 원문 그대로."""
    return NULL_TOKEN if s is None else str(s)


def _popt(tok: str) -> str | None:
    """필드 문자열 → 선택 문자열. NULL_TOKEN 은 None 으로 복원."""
    return None if tok == NULL_TOKEN else tok


# ==============================================================================
# 쓰기 (SceneDoc → scene.txt)
# ==============================================================================
def write_scene(path: str, doc: SceneDoc) -> None:
    """SceneDoc 을 scene.txt 파일로 직렬화해 저장한다(UTF-8, LF 개행)."""
    text = dumps_scene(doc)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def dumps_scene(doc: SceneDoc) -> str:
    """SceneDoc 을 scene.txt 문자열로 직렬화한다(파일에 쓰지 않고 문자열 반환)."""
    out = io.StringIO()
    w = out.write

    w("# Routing3D scene file — units: mm\n")
    w(f"@format {FORMAT_TAG}\n")
    w(f"@version {FORMAT_VERSION}\n\n")

    # ---- [grid]
    w("[grid]\n")
    w(f"cell_mm\t{_ff(doc.cell_mm)}\n")
    w("origin\t" + "\t".join(_ff(v) for v in doc.origin) + "\n")
    w("shape\t" + "\t".join(str(int(v)) for v in doc.shape) + "\n\n")

    # ---- [params]
    p = doc.params
    w("[params]\n")
    w(f"cell_mm\t{_ff(p.cell_mm)}\n")
    w(f"w_turn\t{_ff(p.w_turn)}\n")
    w(f"w_clear\t{_ff(p.w_clear)}\n")
    w(f"clearance_radius\t{int(p.clearance_radius)}\n")
    w(f"clearance_connectivity\t{int(p.clearance_connectivity)}\n")
    tier = "\t".join(f"{int(z)}:{_ff(v)}" for z, v in sorted(p.w_tier.items()))
    w("w_tier" + (("\t" + tier) if tier else "") + "\n\n")

    # ---- [obstacles]
    w(f"[obstacles]\tcount={len(doc.obstacles)}\n")
    w("# minx\tminy\tminz\tmaxx\tmaxy\tmaxz\tost_type\tname\tobject_id\tddworks_type\n")
    for o in doc.obstacles:
        row = [
            _ff(o.min_xyz[0]), _ff(o.min_xyz[1]), _ff(o.min_xyz[2]),
            _ff(o.max_xyz[0]), _ff(o.max_xyz[1]), _ff(o.max_xyz[2]),
            _opt(o.ost_type), _opt(o.name), _opt(o.object_id), _opt(o.ddworks_type),
        ]
        w("\t".join(row) + "\n")
    w("\n")

    # ---- [tasks]
    w(f"[tasks]\tcount={len(doc.tasks)}\n")
    w("# sx\tsy\tsz\tgx\tgy\tgz\tutility\tutility_group\tstart_name\tend_name\tend_instance_guid\n")
    for t in doc.tasks:
        row = [
            _ff(t.start_mm[0]), _ff(t.start_mm[1]), _ff(t.start_mm[2]),
            _ff(t.end_mm[0]), _ff(t.end_mm[1]), _ff(t.end_mm[2]),
            _opt(t.utility), _opt(t.utility_group),
            _opt(t.start_name), _opt(t.end_name), _opt(t.end_instance_guid),
        ]
        w("\t".join(row) + "\n")
    w("\n")

    # ---- [results] (선택)
    res = doc.results or []
    n_res = sum(1 for r in res if r is not None)
    if n_res:
        w(f"[results]\tcount={n_res}\n")
        for idx, r in enumerate(res):
            if r is None:
                continue
            w(f"[result]\ttask={idx}\n")
            w(f"success\t{1 if r.success else 0}\n")
            w(f"length_mm\t{_ff(r.length_mm)}\n")
            w(f"cost_mm\t{_ff(r.cost_mm)}\n")
            w(f"turns\t{int(r.turns)}\n")
            w(f"expanded_nodes\t{int(r.expanded_nodes)}\n")
            w(f"elapsed_ms\t{_ff(r.elapsed_ms)}\n")
            if r.path is not None:
                w(f"[path]\ttask={idx}\tcount={len(r.path)}\n")
                for (i, j, k) in r.path:
                    w(f"{int(i)}\t{int(j)}\t{int(k)}\n")
            if r.visited is not None:
                w(f"[visited]\ttask={idx}\tcount={len(r.visited)}\n")
                for (i, j, k) in r.visited:
                    w(f"{int(i)}\t{int(j)}\t{int(k)}\n")
        w("\n")

    return out.getvalue()


# ==============================================================================
# 읽기 (scene.txt → SceneDoc)
# ==============================================================================
def read_scene(path: str) -> SceneDoc:
    """scene.txt 파일을 읽어 SceneDoc 으로 역직렬화한다."""
    with open(path, "r", encoding="utf-8") as f:
        return loads_scene(f.read())


def loads_scene(text: str) -> SceneDoc:
    """scene.txt 문자열을 SceneDoc 으로 역직렬화한다.

    [파서] 단순 상태기계. '[' 로 시작하면 섹션 전환, 그 외 행은 현재 섹션 규칙으로 해석.
      - grid/params : "key<TAB>값..." 키-값.
      - obstacles/tasks : TAB 구분 레코드 행.
      - result : 지표 키-값. path/visited : "i j k" 셀 행.
    """
    cell_mm: float | None = None
    origin: Vec3 = (0.0, 0.0, 0.0)
    shape: tuple[int, int, int] = (1, 1, 1)
    params_kv: dict[str, list[str]] = {}
    obstacles: list[Obstacle] = []
    tasks: list[RouteTask] = []
    result_kv: dict[int, dict[str, str]] = {}
    path_by_task: dict[int, list[Cell]] = {}
    visited_by_task: dict[int, list[Cell]] = {}

    section = None
    cur_task = None

    for raw in text.splitlines():
        line = raw.rstrip("\r")
        if not line.strip() or line.startswith("#"):
            continue
        if line.startswith("@"):
            _check_header(line)
            continue
        if line.startswith("["):
            parts = line.split("\t")
            section = parts[0][1:].split("]")[0]  # "[result]" → "result"
            attrs = dict(kv.split("=", 1) for kv in parts[1:] if "=" in kv)
            if section in ("result", "path", "visited"):
                cur_task = int(attrs["task"])
                if section == "result":
                    result_kv.setdefault(cur_task, {})
                elif section == "path":
                    path_by_task[cur_task] = []
                elif section == "visited":
                    visited_by_task[cur_task] = []
            continue

        cols = line.split("\t")
        if section == "grid":
            key = cols[0]
            if key == "cell_mm":
                cell_mm = float(cols[1])
            elif key == "origin":
                origin = (float(cols[1]), float(cols[2]), float(cols[3]))
            elif key == "shape":
                shape = (int(cols[1]), int(cols[2]), int(cols[3]))
        elif section == "params":
            params_kv[cols[0]] = cols[1:]
        elif section == "obstacles":
            obstacles.append(_parse_obstacle(cols))
        elif section == "tasks":
            tasks.append(_parse_task(cols))
        elif section == "result":
            result_kv[cur_task][cols[0]] = cols[1] if len(cols) > 1 else ""
        elif section == "path":
            path_by_task[cur_task].append((int(cols[0]), int(cols[1]), int(cols[2])))
        elif section == "visited":
            visited_by_task[cur_task].append((int(cols[0]), int(cols[1]), int(cols[2])))

    params = _build_params(params_kv, cell_mm if cell_mm is not None else 50.0)

    results: list[AStarResult | None] = [None] * len(tasks)
    for idx, kv in result_kv.items():
        results[idx] = AStarResult(
            success=kv.get("success") == "1",
            path=path_by_task.get(idx),
            length_mm=float(kv["length_mm"]),
            turns=int(kv["turns"]),
            expanded_nodes=int(kv["expanded_nodes"]),
            visited=visited_by_task.get(idx),
            elapsed_ms=float(kv["elapsed_ms"]),
            cost_mm=float(kv.get("cost_mm", "0.0")),
        )

    if cell_mm is None:
        raise ValueError("scene.txt 에 [grid] cell_mm 이 없습니다.")
    return SceneDoc(
        cell_mm=cell_mm, origin=origin, shape=shape, params=params,
        obstacles=obstacles, tasks=tasks, results=results,
    )


def _check_header(line: str) -> None:
    """@format / @version 헤더 검증(불일치 시 경고성 ValueError)."""
    if line.startswith("@version"):
        ver = int(line.split()[1])
        if ver != FORMAT_VERSION:
            raise ValueError(f"지원하지 않는 scene 버전: {ver} (지원 {FORMAT_VERSION})")


def _build_params(kv: dict[str, list[str]], default_cell_mm: float) -> RouteParams:
    """params 키-값에서 RouteParams 를 복원한다."""
    def f(key: str, default: float) -> float:
        return float(kv[key][0]) if kv.get(key) else default

    def i(key: str, default: int) -> int:
        return int(kv[key][0]) if kv.get(key) else default

    tier: dict[int, float] = {}
    for tok in kv.get("w_tier", []):
        z, m = tok.split(":")
        tier[int(z)] = float(m)

    return RouteParams(
        cell_mm=f("cell_mm", default_cell_mm),
        w_turn=f("w_turn", 500.0),
        w_clear=f("w_clear", 10.0),
        clearance_radius=i("clearance_radius", 2),
        clearance_connectivity=i("clearance_connectivity", 6),
        w_tier=tier,
    )


def _parse_obstacle(c: list[str]) -> Obstacle:
    return Obstacle(
        object_id=_popt(c[8]),
        name=_popt(c[7]),
        ost_type=_popt(c[6]),
        ddworks_type=_popt(c[9]) if len(c) > 9 else None,
        min_xyz=(float(c[0]), float(c[1]), float(c[2])),
        max_xyz=(float(c[3]), float(c[4]), float(c[5])),
    )


def _parse_task(c: list[str]) -> RouteTask:
    return RouteTask(
        start_mm=(float(c[0]), float(c[1]), float(c[2])),
        end_mm=(float(c[3]), float(c[4]), float(c[5])),
        utility=_popt(c[6]),
        utility_group=_popt(c[7]),
        start_name=_popt(c[8]),
        end_name=_popt(c[9]),
        end_instance_guid=_popt(c[10]) if len(c) > 10 else None,
    )


# ==============================================================================
# 점유맵 재구성 / 씬→SceneDoc 변환
# ==============================================================================
def occupancy_from_doc(
    doc: SceneDoc, *, backend: type[OccupancyMap] = DenseOccupancyMap
) -> OccupancyMap:
    """SceneDoc 의 grid 메타 + obstacles 로 점유맵을 재구성한다(점유 레이어 복원).

    퇴화(두께 0) 박스는 건너뛴다(AABB 생성 시 ValueError → 무시).
    """
    occ = backend(shape=doc.shape, origin=tuple(doc.origin), cell_mm=doc.cell_mm)
    for o in doc.obstacles:
        try:
            occ.add_box(AABB(o.min_xyz, o.max_xyz))
        except ValueError:
            continue
    return occ


def doc_from_scene(
    scene,
    occ: OccupancyMap,
    *,
    params: RouteParams | None = None,
    results: list[AStarResult | None] | None = None,
) -> SceneDoc:
    """RoutingScene(+이미 만든 점유맵)과 결과로 SceneDoc 을 구성한다.

    매개변수:
        scene   : RoutingScene (obstacles/tasks 제공).
        occ     : 격자 메타(cell_mm/origin/shape) 출처가 되는 점유맵.
        params  : RouteParams (없으면 cell_mm 만 맞춘 기본값).
        results : tasks 평행 결과 리스트(없으면 빈 결과).
    """
    return SceneDoc(
        cell_mm=float(occ.cell_mm),
        origin=tuple(float(v) for v in occ.origin),
        shape=tuple(int(v) for v in occ.shape),
        params=params or RouteParams(cell_mm=float(occ.cell_mm)),
        obstacles=list(scene.obstacles),
        tasks=list(scene.tasks),
        results=results or [None] * len(scene.tasks),
    )


# ------------------------------------------------------------------ CLI 진입점

def _main(argv: list[str] | None = None) -> int:
    """커맨드라인 진입점. DB 씬 내보내기 / scene.txt 읽기·무손실 자기검증."""
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(description="scene.txt 입출력 (Step 1.5)")
    parser.add_argument("--in", dest="in_path", default=None, help="읽을 scene.txt 경로")
    parser.add_argument("--out", dest="out_path", default=None, help="내보낼 scene.txt 경로")
    parser.add_argument("--project", type=int, default=None, help="DB 프로젝트 id(내보내기)")
    parser.add_argument("--cell-mm", type=float, default=100.0, help="셀 크기 mm (기본 100)")
    parser.add_argument("--route", action="store_true", help="작업을 독립 A* 로 라우팅해 결과 포함")
    parser.add_argument("--multi", action="store_true", help="다중 배관 순차 라우팅 결과 포함")
    parser.add_argument("--priority", default="longest", help="다중 배관 우선순위")
    parser.add_argument("--roundtrip", action="store_true",
                        help="--in 후 read→write→read 무손실 자기검증")
    parser.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = parser.parse_args(argv)

    # (A) 읽기 모드
    if args.in_path:
        doc = read_scene(args.in_path)
        print(doc.summary())
        if args.roundtrip:
            again = loads_scene(dumps_scene(doc))
            ok = dumps_scene(doc) == dumps_scene(again)
            print(f"[round-trip] write→read→write 동일: {'OK' if ok else 'FAIL'}")
            return 0 if ok else 2
        return 0

    # (B) 내보내기 모드
    if args.project is not None and args.out_path:
        from .obstacle_db import PgConnConfig
        from .scene import load_scene, route_tasks

        overrides = {"dbname": args.dbname} if args.dbname else {}
        config = PgConnConfig.from_env(**overrides)
        scene = load_scene(config, project_id=args.project)
        occ = scene.build_occupancy(cell_mm=args.cell_mm).occupancy
        params = RouteParams(cell_mm=args.cell_mm)

        results: list[AStarResult | None] = [None] * len(scene.tasks)
        if args.multi:
            from .multi_route import route_sequential
            mr = route_sequential(occ, scene.tasks, params, priority=args.priority)
            res_by_id = {id(p.task): p.result for p in mr.pipes}
            results = [res_by_id.get(id(t)) for t in scene.tasks]
        elif args.route:
            routed = route_tasks(occ, scene.tasks, params)
            res_by_id = {id(rt.task): rt.result for rt in routed}
            results = [res_by_id.get(id(t)) for t in scene.tasks]

        doc = doc_from_scene(scene, occ, params=params, results=results)
        write_scene(args.out_path, doc)
        print(doc.summary())
        print(f"[저장] {args.out_path}")
        return 0

    parser.error("--in 또는 (--project 와 --out) 중 하나가 필요합니다.")
    return 1


if __name__ == "__main__":
    raise SystemExit(_main())

"""회귀 시나리오 로더/실행기 — Phase 1 Step 1.7
================================================================================

[실행 명령어]  (이 모듈은 직접 실행하지 않고 test_scenarios.py 가 import 한다)
  # 회귀 시나리오 테스트
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_scenarios.py -v
  # 기대 지표 재생성(알고리즘 의도 변경 시)
  .\\.venv\\Scripts\\python.exe python_experiments/out/_gen_expected.py

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
`tests/scenarios/<name>/input.json` 한 개를 읽어 점유맵·작업·파라미터를 구성하고,
단일(single) 또는 다중(multi) 라우팅을 실행해 **결정적 지표(metrics)**를 산출한다.
A* 는 (f, 삽입순서 counter) tie-break 로 입력이 같으면 항상 같은 경로를 내므로,
길이/회전/확장노드/총길이 같은 지표가 재현 가능하다 → 회귀 테스트의 기준값으로 고정.

[input.json 스키마]
  {
    "name": str, "description": str,
    "cell_mm": float, "origin": [x,y,z], "shape": [nx,ny,nz],
    "obstacles": [ {"min":[x,y,z], "max":[x,y,z], "ost_type": str?}, ... ],
    "params": {"cell_mm","w_turn","w_clear","clearance_radius","clearance_connectivity"},
    "mode": "single" | "multi",
    "priority": "longest" (multi 일 때),
    "tasks": [ {"start":[x,y,z], "end":[x,y,z], "utility":str?, "group":str?}, ... ]
  }

[산출 지표(run_scenario 반환)]
  single : success, length_mm, turns, expanded_nodes, path_hits_obstacle(=0 이어야 함)
  multi  : success_count, fail_count, success_rate, total_length_mm, collisions(=0 이어야 함)
================================================================================
"""

from __future__ import annotations

import json
import os

from routing3d_py import (
    AABB,
    DenseOccupancyMap,
    RouteParams,
    RouteTask,
    astar_weighted,
    route_sequential,
)


def load_input(path: str) -> dict:
    """input.json 을 읽어 dict 로 반환한다."""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def build_occupancy(spec: dict) -> DenseOccupancyMap:
    """spec 의 grid 메타 + obstacles 로 점유맵을 구성한다."""
    occ = DenseOccupancyMap(
        tuple(spec["shape"]), tuple(spec["origin"]), float(spec["cell_mm"])
    )
    for o in spec.get("obstacles", []):
        occ.add_box(AABB(tuple(o["min"]), tuple(o["max"])))
    return occ


def build_params(spec: dict) -> RouteParams:
    """spec 의 params 로 RouteParams 를 구성한다(누락 키는 기본값)."""
    p = spec.get("params", {})
    return RouteParams(
        cell_mm=float(p.get("cell_mm", spec["cell_mm"])),
        w_turn=float(p.get("w_turn", 500.0)),
        w_clear=float(p.get("w_clear", 10.0)),
        clearance_radius=int(p.get("clearance_radius", 2)),
        clearance_connectivity=int(p.get("clearance_connectivity", 6)),
        w_tier={int(k): float(v) for k, v in p.get("w_tier", {}).items()},
    )


def build_tasks(spec: dict) -> list[RouteTask]:
    """spec 의 tasks 로 RouteTask 리스트를 구성한다."""
    out = []
    for t in spec["tasks"]:
        out.append(RouteTask(
            start_mm=tuple(t["start"]), end_mm=tuple(t["end"]),
            utility=t.get("utility"), utility_group=t.get("group"),
            start_name=None, end_name=None, end_instance_guid=None,
        ))
    return out


def run_scenario(spec: dict) -> dict:
    """시나리오를 실행해 결정적 지표 dict 를 반환한다."""
    occ = build_occupancy(spec)
    params = build_params(spec)
    tasks = build_tasks(spec)
    mode = spec.get("mode", "single")

    if mode == "single":
        t = tasks[0]
        res = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm), params)
        hits = 0
        if res.path:
            # 경로 셀이 '원본 장애물' 점유맵에서 막혀 있으면 안 된다(불변식).
            hits = sum(1 for c in res.path if occ.is_blocked(c))
        return {
            "mode": "single",
            "success": res.success,
            "length_mm": res.length_mm,
            "turns": res.turns,
            "expanded_nodes": res.expanded_nodes,
            "path_hits_obstacle": hits,
        }

    # multi
    priority = spec.get("priority", "longest")
    mr = route_sequential(occ, tasks, params, priority=priority)
    succ_paths = [set(p.result.path) for p in mr.pipes if p.result.success]
    collisions = 0
    for i in range(len(succ_paths)):
        for j in range(i + 1, len(succ_paths)):
            if not succ_paths[i].isdisjoint(succ_paths[j]):
                collisions += 1
    return {
        "mode": "multi",
        "success_count": mr.success_count,
        "fail_count": mr.fail_count,
        "success_rate": mr.success_rate,
        "total_length_mm": mr.total_length_mm,
        "collisions": collisions,
    }


def _main() -> int:
    """모든 시나리오를 실행해 결정적 지표를 출력한다(기대치 갱신 참고용).

      .\\.venv\\Scripts\\python.exe python_experiments/tests/scenario_runner.py
    출력된 값으로 각 scenarios/<name>/expected_metrics.json 의 checks 를 갱신한다.
    """
    import sys

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass
    scen_dir = os.path.join(os.path.dirname(__file__), "scenarios")
    for name in sorted(os.listdir(scen_dir)):
        inp = os.path.join(scen_dir, name, "input.json")
        if not os.path.isfile(inp):
            continue
        metrics = run_scenario(load_input(inp))
        print(f"\n=== {name} ===")
        print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

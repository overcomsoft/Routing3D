"""pybind11 바인딩 교차검증 — Routing3D C++ 엔진 (Phase 3, Step 3.10)
================================================================================

[실행 명령어]  (프로젝트 루트에서; 먼저 -DBUILD_PYTHON_BINDINGS=ON 으로 빌드)
  .\\.venv\\Scripts\\python.exe cpp/bindings/test_bindings.py
  # 또는 ctest (BUILD_PYTHON_BINDINGS=ON 일 때 자동 등록):
  ctest --test-dir cpp/build -C Release -R bindings --output-on-failure

[이 스크립트가 하는 일]
--------------------------------------------------------------------------------
빌드된 C++ 확장 모듈 routing3d_cpp 를 import 하여, **Python 레퍼런스(routing3d_py)와
동일 입력에 대해 동일 결과**를 내는지 교차검증한다. 바인딩이 엔진을 올바르게 노출하고
결정성(동일 tie-break)이 언어 경계를 넘어 보존됨을 확인한다.

  1) 골든 01/02 (단일 비용함수 A*): 길이/회전/확장노드 + **경로 셀 완전 일치**.
  2) 골든 03 (다중 순차): 성공수/실패수/총길이/충돌수 일치.
  3) scene.txt 왕복: C++ read_scene → dumps_scene 가 Python 픽스처와 바이트 동일.

레퍼런스 지표는 routing3d_py(scenario_runner)로 즉석 계산해 하드코딩을 피한다.
================================================================================
"""

from __future__ import annotations

import glob
import importlib.util
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SCEN_DIR = os.path.join(ROOT, "python_experiments", "tests", "scenarios")
TESTS_DIR = os.path.join(ROOT, "python_experiments", "tests")
FIXTURE = os.path.join(ROOT, "cpp", "tests", "fixtures", "roundtrip.scene.txt")

_failures = 0


def check(cond: bool, msg: str) -> None:
    global _failures
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
    if not cond:
        _failures += 1


def load_cpp_module():
    """빌드 산출물에서 routing3d_cpp .pyd 를 찾아 import 한다."""
    # 후보 디렉토리: ctest 가 넘긴 인자 > 환경변수 > 기본 빌드 경로(Release/Debug).
    candidates = []
    if len(sys.argv) > 1:
        candidates.append(sys.argv[1])
    if os.environ.get("ROUTING3D_CPP_DIR"):
        candidates.append(os.environ["ROUTING3D_CPP_DIR"])
    candidates += [
        os.path.join(ROOT, "cpp", "build", "Release"),
        os.path.join(ROOT, "cpp", "build", "Debug"),
        os.path.join(ROOT, "cpp", "build"),
    ]
    for d in candidates:
        hits = glob.glob(os.path.join(d, "routing3d_cpp*.pyd")) + \
            glob.glob(os.path.join(d, "routing3d_cpp*.so"))
        if hits:
            spec = importlib.util.spec_from_file_location("routing3d_cpp", hits[0])
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            print(f"[모듈] {hits[0]}")
            return mod
    raise SystemExit(f"routing3d_cpp 모듈을 찾을 수 없습니다. 후보: {candidates}")


def cpp_build_params(m, spec):
    p = spec.get("params", {})
    return m.RouteParams(
        cell_mm=float(p.get("cell_mm", spec["cell_mm"])),
        w_turn=float(p.get("w_turn", 500.0)),
        w_clear=float(p.get("w_clear", 10.0)),
        clearance_radius=int(p.get("clearance_radius", 2)),
        clearance_connectivity=int(p.get("clearance_connectivity", 6)),
        w_tier={int(k): float(v) for k, v in p.get("w_tier", {}).items()},
    )


def cpp_build_occ(m, spec):
    occ = m.DenseOccupancy(m.Cell(*spec["shape"]), m.Vec3(*spec["origin"]), float(spec["cell_mm"]))
    for o in spec.get("obstacles", []):
        occ.add_box(m.AABB(m.Vec3(*o["min"]), m.Vec3(*o["max"])))
    return occ


def cpp_run_scenario(m, spec):
    """scenario_runner.run_scenario 의 C++ 바인딩 버전(동일 지표 산출)."""
    occ = cpp_build_occ(m, spec)
    params = cpp_build_params(m, spec)
    mode = spec.get("mode", "single")

    if mode == "single":
        t = spec["tasks"][0]
        s = occ.to_cell(m.Vec3(*t["start"]))
        g = occ.to_cell(m.Vec3(*t["end"]))
        res = m.astar_weighted(occ, s, g, params)
        hits = sum(1 for c in res.path if occ.is_blocked(c))
        return {
            "mode": "single", "success": res.success, "length_mm": res.length_mm,
            "turns": res.turns, "expanded_nodes": res.expanded_nodes,
            "path_hits_obstacle": hits, "_path": [c.as_tuple() for c in res.path],
        }

    tasks = [m.RouteTask(m.Vec3(*t["start"]), m.Vec3(*t["end"]), t.get("utility"), t.get("group"))
             for t in spec["tasks"]]
    mr = m.route_sequential(occ, tasks, params, priority=spec.get("priority", "longest"))
    succ = [set(c.as_tuple() for c in pr.result.path) for pr in mr.pipes if pr.result.success]
    collisions = sum(1 for i in range(len(succ)) for j in range(i + 1, len(succ))
                     if succ[i] & succ[j])
    return {
        "mode": "multi", "success_count": mr.success_count, "fail_count": mr.fail_count,
        "success_rate": mr.success_rate, "total_length_mm": mr.total_length_mm,
        "collisions": collisions,
    }


def py_path_for_single(spec):
    """레퍼런스(routing3d_py)로 단일 시나리오의 경로 셀을 구한다(경로 일치 검증용)."""
    from routing3d_py import AABB, DenseOccupancyMap, astar_weighted
    from scenario_runner import build_params

    occ = DenseOccupancyMap(tuple(spec["shape"]), tuple(spec["origin"]), float(spec["cell_mm"]))
    for o in spec.get("obstacles", []):
        occ.add_box(AABB(tuple(o["min"]), tuple(o["max"])))
    params = build_params(spec)
    t = spec["tasks"][0]
    res = astar_weighted(occ, occ.to_cell(t["start"]), occ.to_cell(t["end"]), params)
    return [tuple(c) for c in res.path]


def main() -> int:
    sys.path.insert(0, TESTS_DIR)  # scenario_runner.
    m = load_cpp_module()
    from scenario_runner import load_input, run_scenario  # 레퍼런스.

    # ---- (1)(2) 골든 시나리오 교차검증 ----
    for name in sorted(os.listdir(SCEN_DIR)):
        inp = os.path.join(SCEN_DIR, name, "input.json")
        if not os.path.isfile(inp):
            continue
        spec = load_input(inp)
        print(f"=== {name} ({spec.get('mode', 'single')}) ===")
        ref = run_scenario(spec)
        got = cpp_run_scenario(m, spec)
        for key, want in ref.items():
            check(got.get(key) == want, f"{key} == {want!r} (C++ {got.get(key)!r})")
        # 단일은 경로 셀까지 완전 일치(결정성).
        if spec.get("mode", "single") == "single":
            check(got["_path"] == py_path_for_single(spec), "경로 셀 완전 일치(결정성)")

    # ---- (3) scene.txt 왕복(바인딩 경유) ----
    print("=== scene.txt round-trip via binding ===")
    with open(FIXTURE, "rb") as f:
        original = f.read()
    doc = m.read_scene(FIXTURE)
    out = m.dumps_scene(doc).encode("utf-8")
    check(out == original, "C++ read_scene→dumps_scene 가 Python 픽스처와 바이트 동일")
    check(len(doc.obstacles) == 4 and len(doc.tasks) == 2, "파싱 구조(장애물4/작업2)")

    print(f"\n{'ALL PASS' if _failures == 0 else 'FAILED'} (failures={_failures})")
    return 0 if _failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

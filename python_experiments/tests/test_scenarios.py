"""회귀 시나리오 테스트 — Phase 1 Step 1.7
================================================================================

[실행 명령어]  (프로젝트 루트 또는 python_experiments/ 에서)
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_scenarios.py -v

[검증 범위]
  tests/scenarios/<name>/input.json 을 실행한 결정적 지표를 같은 폴더의
  expected_metrics.json 의 checks 와 비교한다. 대표 시나리오 3종:
    01_single_empty   — 빈 공간 직선 = 맨해튼 거리(알고리즘 sanity).
    02_single_obstacle— 장애물 우회 길이가 직선의 ±20% 이내 + 장애물 비통과.
    03_multi_tier     — 다중 배관 5개 모두 성공 + 충돌(셀 공유) 0.

[expected_metrics.json checks 키 해석]
  success / turns / path_hits_obstacle / collisions / fail_count / success_count : 정확히 일치.
  length_mm / total_length_mm / success_rate : 근사 일치(pytest.approx).
  detour_ratio_max : length_mm <= 맨해튼거리 * 값 (우회 비율 상한).
  expanded_nodes_max : expanded_nodes <= 값 (탐색량 회귀 상한).
================================================================================
"""

import json
import os

import pytest

from scenario_runner import build_params, load_input, run_scenario  # noqa: F401

SCEN_DIR = os.path.join(os.path.dirname(__file__), "scenarios")


def _scenario_names():
    if not os.path.isdir(SCEN_DIR):
        return []
    return sorted(
        n for n in os.listdir(SCEN_DIR)
        if os.path.isfile(os.path.join(SCEN_DIR, n, "input.json"))
    )


def _manhattan_mm(spec):
    cm = spec["cell_mm"]
    o = spec["origin"]
    t = spec["tasks"][0]
    def cell(w, i):
        return int((w - o[i]) // cm)
    return sum(abs(cell(t["end"][i], i) - cell(t["start"][i], i)) for i in range(3)) * cm


@pytest.mark.parametrize("name", _scenario_names())
def test_scenario(name):
    d = os.path.join(SCEN_DIR, name)
    spec = load_input(os.path.join(d, "input.json"))
    with open(os.path.join(d, "expected_metrics.json"), "r", encoding="utf-8") as f:
        checks = json.load(f)["checks"]

    m = run_scenario(spec)

    for key, want in checks.items():
        if key in ("length_mm", "total_length_mm", "success_rate"):
            assert m[key] == pytest.approx(want), f"{name}.{key}: {m[key]} != {want}"
        elif key == "detour_ratio_max":
            man = _manhattan_mm(spec)
            assert m["length_mm"] <= man * want + 1e-6, \
                f"{name}: 우회 {m['length_mm']}mm > 맨해튼 {man}mm × {want}"
        elif key == "expanded_nodes_max":
            assert m["expanded_nodes"] <= want, \
                f"{name}: expanded {m['expanded_nodes']} > {want}"
        else:  # 정확히 일치(success/turns/collisions/counts/path_hits_obstacle)
            assert m[key] == want, f"{name}.{key}: {m[key]} != {want}"


def test_baseline_params_loads():
    """experiments/baseline_params.json 이 유효한 RouteParams 로 로드된다."""
    root = os.path.dirname(os.path.dirname(__file__))
    path = os.path.join(root, "experiments", "baseline_params.json")
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    params = build_params({"cell_mm": data["params"]["cell_mm"], "params": data["params"]})
    assert params.cell_mm == 50.0
    assert params.w_turn >= 0 and params.w_clear >= 0

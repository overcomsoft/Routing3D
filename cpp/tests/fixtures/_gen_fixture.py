"""scene.txt 교차검증 픽스처 생성기 — Routing3D C++ Phase 3 Step 3.9
================================================================================

[실행 명령어]  (editable 설치 후 프로젝트 루트에서)
  .\\.venv\\Scripts\\python.exe cpp/tests/fixtures/_gen_fixture.py

[이 스크립트가 하는 일]
  Python 레퍼런스(routing3d_py.scene_io)로 대표 + 까다로운 부동소수점 값이 담긴
  SceneDoc 을 만들어 `roundtrip.scene.txt` 로 직렬화한다. 이 파일은 C++ scene_io 가
  '읽고 → 다시 써서' Python 출력과 **바이트 단위로 동일**한지(계약 F2/F4) 검증하는
  골든 픽스처다. 따라서 알고리즘 의도 변경 시에만 재생성한다.

[픽스처에 담긴 검증 포인트]
  - 실수 표기(F4): 0.1 / 0.0001 / 1e-05 / 1e+16 / 1e15 / -0.0 / 지수·고정 경계값.
  - \\N(None) vs ""(빈 문자열) 구분(F3): object_id/name/utility 등에 혼재.
  - 이름 필드의 공백·"//"·유니코드(한글) 보존.
  - 결과 레이어: 성공(경로+방문 있음) / 실패(경로·방문 없음) 혼재.
================================================================================
"""

import os

from routing3d_py.astar import AStarResult
from routing3d_py.cost import RouteParams
from routing3d_py.obstacle_db import Obstacle
from routing3d_py.scene import RouteTask
from routing3d_py.scene_io import SceneDoc, write_scene

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "roundtrip.scene.txt")


def build_doc() -> SceneDoc:
    obstacles = [
        Obstacle(object_id="G1", name="Floor // slab 1", ost_type="OST_Floors",
                 ddworks_type=None, min_xyz=(0.0, 0.0, 0.0), max_xyz=(6000.0, 6000.0, 250.0)),
        Obstacle(object_id=None, name="", ost_type="OST_Columns",
                 ddworks_type="FLOOR_ARCHITECTURE",
                 min_xyz=(1900.0, 0.0, 0.0), max_xyz=(2150.5, 2250.0, 4000.0)),
        # 까다로운 실수: 작은/큰 지수, 고정/지수 경계.
        Obstacle(object_id="", name="한글 이름 / unicode", ost_type=None, ddworks_type=None,
                 min_xyz=(0.1, 0.0001, 1e-05), max_xyz=(1e16, 1e15, 3.141592653589793)),
        Obstacle(object_id="neg", name="neg coords", ost_type="OST_Walls", ddworks_type=None,
                 min_xyz=(-0.0, -2150.5, 2.5e-10),
                 max_xyz=(100000.0, 1234.5, 9999999999999998.0)),
    ]

    tasks = [
        RouteTask(start_mm=(275.0, 3025.0, 1525.0), end_mm=(5725.0, 3025.0, 1525.0),
                  utility="PA", utility_group="Gas",
                  start_name="POC 1", end_name=None, end_instance_guid="IG1"),
        RouteTask(start_mm=(25.5, 275.25, 75.125), end_mm=(475.0, 275.0, 75.0),
                  utility=None, utility_group="",
                  start_name="", end_name="end // x", end_instance_guid=None),
    ]

    results = [
        AStarResult(success=True, path=[(0, 5, 1), (1, 5, 1), (2, 5, 1)],
                    length_mm=450.0, turns=2, expanded_nodes=1234,
                    visited=[(0, 5, 1), (0, 6, 1), (1, 5, 1)],
                    elapsed_ms=0.123, cost_mm=470.5),
        AStarResult(success=False, path=None, length_mm=0.0, turns=0,
                    expanded_nodes=42, visited=None, elapsed_ms=0.05, cost_mm=0.0),
    ]

    params = RouteParams(cell_mm=50.0, w_turn=500.0, w_clear=10.0,
                         clearance_radius=2, clearance_connectivity=6,
                         w_tier={1: 200.0, 3: 50.5})

    return SceneDoc(cell_mm=50.0, origin=(0.0, 0.0, 0.0), shape=(120, 120, 60),
                    params=params, obstacles=obstacles, tasks=tasks, results=results)


def main() -> int:
    doc = build_doc()
    write_scene(OUT, doc)
    print(doc.summary())
    print(f"[저장] {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

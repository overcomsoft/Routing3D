"""Routing3D Phase 1 — Python 알고리즘 실험 패키지
================================================================================

[실행 명령어]  (모두 python_experiments/ 디렉토리에서)
  # 전체 테스트
  ..\\.venv\\Scripts\\python.exe -m pytest -v
  # 패키지 import 확인
  ..\\.venv\\Scripts\\python.exe -c "import routing3d_py; print(routing3d_py.__all__)"

[패키지 개요]
  플랜트 배관 3D 직교 자동 라우팅 엔진의 Python 실험 코드.
  좌표·치수 단위는 밀리미터(mm), 기본 셀 크기 50mm.
  세부 계획: docs/phase1_plan.md

[구성 모듈] (단계별로 추가됨)
  occupancy   : Step 1.1 점유맵 — 추상 OccupancyMap + Dense/Sparse/BitPacked 백엔드 (완료)
  obstacle_db : 장애물 DB 로더 (PostgreSQL TB_BIM_OBSTACLES → 점유맵, 구현 완료)
  astar       : Step 1.2 직교 A* 탐색 + Step 1.3 비용함수 적용 A*(astar_weighted)
  cost        : Step 1.3 비용함수 — RouteParams / clearance_map / CostModel (구현 완료)
  scene       : SpaceAI 프로젝트 라우팅 씬 로더 (project→장애물/메인장비/종단/PoC페어/유틸)
  multi_route : Step 1.4 다중 배관 전략 (예정)
  scene_io    : Step 1.5 scene.txt 입출력 (예정)
  viz         : Step 1.5 PyVista/Plotly 시각화 (예정)
================================================================================
"""

from .astar import (
    AStarResult,
    astar,
    astar_weighted,
    astar_world,
    count_turns,
    manhattan,
)
from .cost import CostModel, RouteParams, clearance_map
from .scene import (
    EndPoc,
    MainEquipment,
    ProjectInfo,
    RoutedTask,
    RouteTask,
    RoutingScene,
    StartPoc,
    TerminalObject,
    list_projects,
    load_scene,
    route_tasks,
    scene_polylines,
    utility_colors,
)
from .occupancy import (
    AABB,
    BitPackedOccupancyMap,
    DenseOccupancyMap,
    OccupancyMap,
    SparseOccupancyMap,
)
from .obstacle_db import (
    BuildResult,
    Obstacle,
    PgConnConfig,
    build_occupancy,
    distinct_ost_types,
    group_by_type,
    load_obstacles,
    obstacles_bounds,
)

# 패키지에서 외부로 공개하는 이름 목록.
__all__ = [
    # Step 1.1 점유맵 (추상 인터페이스 + 3개 백엔드)
    "AABB",
    "OccupancyMap",
    "DenseOccupancyMap",
    "SparseOccupancyMap",
    "BitPackedOccupancyMap",
    # 장애물 DB 로더
    "PgConnConfig",
    "Obstacle",
    "BuildResult",
    "load_obstacles",
    "distinct_ost_types",
    "group_by_type",
    "obstacles_bounds",
    "build_occupancy",
    # Step 1.2 직교 A*
    "astar",
    "astar_world",
    "AStarResult",
    "manhattan",
    "count_turns",
    # Step 1.3 비용함수
    "astar_weighted",
    "RouteParams",
    "CostModel",
    "clearance_map",
    # 라우팅 씬 (SpaceAI 프로젝트 데이터 환경)
    "ProjectInfo",
    "EndPoc",
    "StartPoc",
    "RouteTask",
    "MainEquipment",
    "TerminalObject",
    "RoutingScene",
    "RoutedTask",
    "list_projects",
    "load_scene",
    "route_tasks",
    "scene_polylines",
    "utility_colors",
]

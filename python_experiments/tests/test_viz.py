"""점유맵 시각화 단위 테스트 — viz
================================================================================

[실행 명령어]  (python_experiments/ 디렉토리에서)
  ..\\.venv\\Scripts\\python.exe -m pytest tests/test_viz.py -v

[검증 범위]
  렌더링 창 없이(off-screen 불필요) 검증 가능한 부분만 테스트한다:
  복셀 메시 추출(occupancy_to_voxels)이 점유 셀 수·기하와 일치하는지.
  실제 화면 렌더(스크린샷/HTML/창)는 환경 의존이라 자동 테스트에서 제외한다.
================================================================================
"""

import pytest

from routing3d_py.occupancy import (
    AABB,
    BitPackedOccupancyMap,
    DenseOccupancyMap,
    SparseOccupancyMap,
)

pv = pytest.importorskip("pyvista", reason="pyvista 미설치 시 시각화 테스트 skip")

from routing3d_py.viz import occupancy_to_voxels  # noqa: E402

ALL_BACKENDS = [DenseOccupancyMap, SparseOccupancyMap, BitPackedOccupancyMap]


@pytest.mark.parametrize("backend", ALL_BACKENDS, ids=lambda c: c.__name__)
def test_voxels_cell_count_matches_blocked(backend):
    """추출된 복셀 메시의 셀 수가 점유 셀 수와 같아야 한다(모든 백엔드 동일)."""
    m = backend(shape=(6, 6, 6), cell_mm=50)
    m.add_box(AABB((50, 50, 50), (250, 150, 200)))  # 4x2x3 = 24 셀
    voxels = occupancy_to_voxels(m)
    assert m.count_blocked() == 24
    assert voxels.n_cells == 24


def test_voxels_empty_when_no_occupancy():
    m = DenseOccupancyMap(shape=(4, 4, 4))
    voxels = occupancy_to_voxels(m)
    assert voxels.n_cells == 0


def test_voxels_world_position():
    """복셀 메시가 월드 좌표(mm)·셀 크기에 맞게 배치되는지 경계로 확인."""
    m = DenseOccupancyMap(shape=(4, 4, 4), origin=(1000, 2000, 3000), cell_mm=50)
    m.block_cell((0, 0, 0))  # 월드 [1000,1050] x [2000,2050] x [3000,3050]
    voxels = occupancy_to_voxels(m)
    xmin, xmax, ymin, ymax, zmin, zmax = voxels.bounds
    assert (xmin, ymin, zmin) == pytest.approx((1000, 2000, 3000))
    assert (xmax, ymax, zmax) == pytest.approx((1050, 2050, 3050))

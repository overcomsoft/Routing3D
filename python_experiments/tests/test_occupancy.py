"""점유맵 단위 테스트 — Phase 1 Step 1.1
================================================================================

[실행 명령어]  (python_experiments/ 디렉토리에서)
  # 이 파일만 실행
  ..\\.venv\\Scripts\\python.exe -m pytest tests/test_occupancy.py -v
  # 전체 테스트 실행
  ..\\.venv\\Scripts\\python.exe -m pytest -v

[테스트 대상]
  routing3d_py.occupancy 의 AABB + 3개 백엔드
    DenseOccupancyMap / SparseOccupancyMap / BitPackedOccupancyMap

[전략]
  - 공통 인터페이스 동작은 ALL_BACKENDS 로 매개변수화하여 세 백엔드가 *동일하게*
    동작함을 검증한다(생성/좌표변환/경계/박스복셀화/팽창).
  - Dense 전용(.grid), 백엔드 메모리 특성은 별도 테스트로 검증한다.
================================================================================
"""

import numpy as np
import pytest

from routing3d_py.occupancy import (
    AABB,
    BitPackedOccupancyMap,
    DenseOccupancyMap,
    OccupancyMap,
    SparseOccupancyMap,
)

# 공통 인터페이스 검증에 사용할 백엔드 목록.
ALL_BACKENDS = [DenseOccupancyMap, SparseOccupancyMap, BitPackedOccupancyMap]


@pytest.fixture(params=ALL_BACKENDS, ids=lambda c: c.__name__)
def backend(request):
    """세 백엔드 클래스를 차례로 주입하는 픽스처."""
    return request.param


# ====================================================== 공통: 생성 / 검증

def test_construct_default(backend):
    m = backend(shape=(4, 4, 4))
    assert m.shape == (4, 4, 4)
    assert m.cell_mm == 50.0
    assert m.count_blocked() == 0
    assert isinstance(m, OccupancyMap)


@pytest.mark.parametrize("bad_shape", [(0, 4, 4), (-1, 4, 4), (4, 4)])
def test_construct_bad_shape(backend, bad_shape):
    with pytest.raises(ValueError):
        backend(shape=bad_shape)


def test_construct_bad_cell_mm(backend):
    with pytest.raises(ValueError):
        backend(shape=(4, 4, 4), cell_mm=0)


def test_from_world_bounds_rounds_up(backend):
    # 0..210mm 범위, 셀 50mm → 천장(210/50)=5 셀
    m = backend.from_world_bounds((0, 0, 0), (210, 100, 100), cell_mm=50)
    assert m.shape == (5, 2, 2)
    assert m.cell_mm == 50
    assert isinstance(m, backend)


def test_from_world_bounds_invalid(backend):
    with pytest.raises(ValueError):
        backend.from_world_bounds((0, 0, 0), (0, 100, 100))


# ====================================================== 공통: 좌표 변환

def test_to_world_cell_center(backend):
    m = backend(shape=(4, 4, 4), origin=(0, 0, 0), cell_mm=50)
    assert m.to_world((1, 2, 3)) == pytest.approx((75.0, 125.0, 175.0))


def test_to_world_with_origin(backend):
    m = backend(shape=(4, 4, 4), origin=(100, 200, 300), cell_mm=50)
    assert m.to_world((0, 0, 0)) == pytest.approx((125.0, 225.0, 325.0))


def test_to_cell_roundtrip(backend):
    m = backend(shape=(10, 10, 10), origin=(7, -3, 11), cell_mm=50)
    for cell in [(0, 0, 0), (1, 2, 3), (9, 9, 9)]:
        assert m.to_cell(m.to_world(cell)) == cell


def test_to_cell_floor(backend):
    m = backend(shape=(4, 4, 4), cell_mm=50)
    assert m.to_cell((49, 49, 49)) == (0, 0, 0)
    assert m.to_cell((50, 50, 50)) == (1, 1, 1)
    assert m.to_cell((99, 99, 99)) == (1, 1, 1)


# ====================================================== 공통: 경계 / 질의

def test_in_bounds(backend):
    m = backend(shape=(4, 4, 4))
    assert m.in_bounds((0, 0, 0))
    assert m.in_bounds((3, 3, 3))
    assert not m.in_bounds((4, 0, 0))
    assert not m.in_bounds((-1, 0, 0))


def test_is_blocked_out_of_bounds_is_true(backend):
    m = backend(shape=(4, 4, 4))
    assert m.is_blocked((-1, 0, 0)) is True
    assert m.is_blocked((4, 0, 0)) is True
    assert m.is_blocked((0, 0, 0)) is False


def test_world_bounds(backend):
    m = backend(shape=(4, 2, 1), origin=(10, 20, 30), cell_mm=50)
    lo, hi = m.world_bounds()
    assert lo == pytest.approx((10, 20, 30))
    assert hi == pytest.approx((10 + 200, 20 + 100, 30 + 50))


def test_block_cell(backend):
    m = backend(shape=(4, 4, 4))
    m.block_cell((2, 2, 2))
    assert m.is_blocked((2, 2, 2))
    assert m.count_blocked() == 1
    m.block_cell((5, 5, 5))  # 격자 밖은 무시
    assert m.count_blocked() == 1


# ====================================================== 공통: 박스 복셀화

def test_add_box_interior(backend):
    m = backend(shape=(4, 4, 4), cell_mm=50)
    n = m.add_box(AABB(lo=(50, 50, 50), hi=(150, 150, 150)))
    assert n == 8
    assert m.count_blocked() == 8
    assert m.is_blocked((1, 1, 1))
    assert m.is_blocked((2, 2, 2))
    assert not m.is_blocked((0, 0, 0))
    assert not m.is_blocked((3, 3, 3))


def test_add_box_clips_to_grid(backend):
    m = backend(shape=(4, 4, 4), cell_mm=50)
    n = m.add_box(AABB(lo=(-1000, -1000, -1000), hi=(1000, 1000, 1000)))
    assert n == 4 * 4 * 4
    assert m.count_blocked() == 64


def test_add_box_fully_outside(backend):
    m = backend(shape=(4, 4, 4), cell_mm=50)
    n = m.add_box(AABB(lo=(1000, 1000, 1000), hi=(2000, 2000, 2000)))
    assert n == 0
    assert m.count_blocked() == 0


def test_add_box_overlap_counts_only_new(backend):
    m = backend(shape=(4, 4, 4), cell_mm=50)
    first = m.add_box(AABB(lo=(50, 50, 50), hi=(150, 150, 150)))
    second = m.add_box(AABB(lo=(50, 50, 50), hi=(150, 150, 150)))
    assert first == 8
    assert second == 0


def test_aabb_invalid():
    with pytest.raises(ValueError):
        AABB(lo=(0, 0, 0), hi=(0, 10, 10))


# ====================================================== 공통: 팽창(inflate)

def test_inflate_zero_is_copy(backend):
    m = backend(shape=(5, 5, 5), cell_mm=50)
    m.block_cell((2, 2, 2))
    out = m.inflate(0)
    assert out.count_blocked() == 1
    assert isinstance(out, backend)
    out.block_cell((0, 0, 0))  # 독립 복사본
    assert m.count_blocked() == 1


def test_inflate_radius1_6conn(backend):
    m = backend(shape=(5, 5, 5), cell_mm=50)
    m.block_cell((2, 2, 2))
    out = m.inflate(1, connectivity=6)
    assert out.count_blocked() == 7  # 중심 1 + 면인접 6
    for off in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]:
        assert out.is_blocked((2 + off[0], 2 + off[1], 2 + off[2]))
    assert not out.is_blocked((3, 3, 2))  # 대각선은 6-conn 비점유


def test_inflate_radius1_26conn(backend):
    m = backend(shape=(5, 5, 5), cell_mm=50)
    m.block_cell((2, 2, 2))
    out = m.inflate(1, connectivity=26)
    assert out.count_blocked() == 27  # 3x3x3
    assert out.is_blocked((3, 3, 3))


def test_inflate_radius2_6conn_manhattan_ball(backend):
    m = backend(shape=(9, 9, 9), cell_mm=50)
    m.block_cell((4, 4, 4))
    out = m.inflate(2, connectivity=6)
    assert out.count_blocked() == 25  # 3D 맨해튼 볼 r=2
    assert out.is_blocked((6, 4, 4))
    assert not out.is_blocked((7, 4, 4))


def test_inflate_does_not_mutate_source(backend):
    m = backend(shape=(5, 5, 5), cell_mm=50)
    m.block_cell((2, 2, 2))
    _ = m.inflate(1)
    assert m.count_blocked() == 1


def test_inflate_clips_at_edge(backend):
    m = backend(shape=(3, 3, 3), cell_mm=50)
    m.block_cell((0, 0, 0))
    out = m.inflate(1, connectivity=6)
    assert out.count_blocked() == 4  # 모서리: 안쪽 면인접 3 + 중심
    assert out.is_blocked((1, 0, 0))
    assert out.is_blocked((0, 1, 0))
    assert out.is_blocked((0, 0, 1))


def test_inflate_negative_radius(backend):
    m = backend(shape=(4, 4, 4))
    with pytest.raises(ValueError):
        m.inflate(-1)


def test_inflate_bad_connectivity(backend):
    m = backend(shape=(4, 4, 4))
    with pytest.raises(ValueError):
        m.inflate(1, connectivity=18)


# ====================================================== 백엔드 간 동등성(parity)

def test_backends_agree_on_voxelization():
    """동일 박스를 세 백엔드에 넣으면 점유 셀 집합이 완전히 동일해야 한다."""
    boxes = [
        AABB((50, 50, 50), (250, 150, 350)),
        AABB((400, 100, 100), (500, 500, 200)),
    ]
    maps = {cls.__name__: cls(shape=(12, 12, 12), cell_mm=50) for cls in ALL_BACKENDS}
    for m in maps.values():
        m.add_boxes(boxes)

    counts = {name: m.count_blocked() for name, m in maps.items()}
    assert len(set(counts.values())) == 1, f"count mismatch: {counts}"

    # 셀 단위로도 완전 일치 확인
    nx, ny, nz = (12, 12, 12)
    ref = maps["DenseOccupancyMap"]
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                c = (i, j, k)
                want = ref.is_blocked(c)
                for name, m in maps.items():
                    assert m.is_blocked(c) is want, f"{name} disagrees at {c}"


def test_backends_agree_after_inflate():
    """팽창 후에도 세 백엔드의 점유 셀 집합이 동일해야 한다."""
    maps = {cls.__name__: cls(shape=(9, 9, 9), cell_mm=50) for cls in ALL_BACKENDS}
    for m in maps.values():
        m.block_cell((4, 4, 4))
    inflated = {name: m.inflate(2, connectivity=26) for name, m in maps.items()}
    counts = {name: m.count_blocked() for name, m in inflated.items()}
    assert len(set(counts.values())) == 1, f"count mismatch: {counts}"


# ====================================================== 백엔드 전용

def test_dense_grid_is_bool():
    m = DenseOccupancyMap(shape=(4, 4, 4))
    assert m.grid.dtype == bool
    assert m.grid.shape == (4, 4, 4)


def test_bitpacked_uses_less_memory_than_dense():
    shape = (64, 64, 64)
    dense = DenseOccupancyMap(shape=shape)
    packed = BitPackedOccupancyMap(shape=shape)
    # 비트팩은 Dense 의 약 1/8 (셀당 1비트 vs 1바이트)
    assert packed.approx_bytes() < dense.approx_bytes()
    assert packed.approx_bytes() <= dense.approx_bytes() // 4


def test_sparse_memory_scales_with_occupancy():
    m = SparseOccupancyMap(shape=(100, 100, 100))
    empty_bytes = m.approx_bytes()
    m.add_box(AABB((0, 0, 0), (1000, 1000, 1000)))  # 일부 점유
    assert m.approx_bytes() > empty_bytes
    assert m.count_blocked() > 0


def test_bitpacked_nz_not_multiple_of_8():
    # z=5 (8의 배수 아님) 에서도 정확해야 함
    m = BitPackedOccupancyMap(shape=(3, 3, 5), cell_mm=50)
    m.block_cell((1, 1, 4))  # 마지막 z
    assert m.is_blocked((1, 1, 4))
    assert m.count_blocked() == 1
    assert not m.is_blocked((1, 1, 3))

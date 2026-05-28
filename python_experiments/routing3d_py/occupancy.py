"""점유맵 (Occupancy Map) — Phase 1 Step 1.1
================================================================================

[실행 명령어]
--------------------------------------------------------------------------------
  # 단위 테스트 실행 (python_experiments/ 디렉토리에서)
  ..\\.venv\\Scripts\\python.exe -m pytest tests/test_occupancy.py -v

  # 백엔드 메모리 비교 (python_experiments/ 디렉토리에서)
  ..\\.venv\\Scripts\\python.exe -c "from routing3d_py import DenseOccupancyMap, SparseOccupancyMap, BitPackedOccupancyMap, AABB; \
[ (m.__class__.__name__, m.add_box(AABB((100,100,100),(900,900,200))), m.approx_bytes()) for m in (DenseOccupancyMap((40,40,40)), SparseOccupancyMap((40,40,40)), BitPackedOccupancyMap((40,40,40))) ] and print('ok')"

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
플랜트 공간을 일정 크기(기본 50mm)의 정육면체 셀로 나눈 3차원 격자로 표현하고,
각 셀이 "장애물로 점유되었는지"를 True/False 로 관리한다. 이후 직교 A* 탐색이
이 점유맵에 "이 셀로 지나갈 수 있는가?"를 질의한다.

[저장 백엔드 — 동일 인터페이스, 다른 메모리 특성]
--------------------------------------------------------------------------------
등간격 복셀은 고해상도/대영역에서 셀 수가 폭발한다. 그래서 같은 질의 인터페이스
(OccupancyMap 추상 클래스) 뒤에 세 가지 저장 방식을 두고 상황에 맞게 고른다.
A* 등 사용자 코드는 인터페이스(is_blocked/in_bounds/bounds/to_world/to_cell)에만
의존하므로 백엔드를 자유롭게 교체할 수 있다.

  ┌─ DenseOccupancyMap   : NumPy bool 배열. 셀당 1바이트. 질의 O(1)로 가장 빠르고
  │                        구현이 단순. 빈 공간도 1바이트라 ROI(관심영역)가 작으면 최선.
  ├─ BitPackedOccupancyMap: 비트팩(셀당 1비트, Dense 대비 1/8 메모리). 질의 약간 느림.
  │                        같은 메모리로 ~8배 큰 ROI 를 Python 에서 다룰 수 있음.
  └─ SparseOccupancyMap  : 점유 셀만 set 에 저장. *점유가 희박*하면 메모리 절약.
                           단, 바닥·기둥처럼 점유가 빽빽하면 set 오버헤드로 오히려
                           Dense 보다 클 수 있음(주의).

  (참고) 바닥 슬래브 같은 '균일한 큰 덩어리'의 진짜 압축은 옥트리/VDB 의 몫이며,
         이는 433m 스케일 성능이 목표인 Phase 3(OpenVDB)에서 도입한다.

[단위 규약] (docs/phase1_plan.md §4)
  - 모든 월드 좌표·치수의 단위는 밀리미터(mm).
  - 셀 인덱스는 정수 3-튜플 (i, j, k). 셀 (0,0,0) 은 origin 에서 시작.
  - 셀 중심 월드좌표 = origin + (cell + 0.5) * cell_mm
  - shape = (nx, ny, nz) 는 각 축의 셀 개수.
  - 격자 범위 밖의 셀은 항상 "점유(True)"로 간주(A* 가 격자 밖으로 못 나가게).

[설계 의도]
--------------------------------------------------------------------------------
  Phase 3 C++ 포팅이 1:1 대응되도록 자료구조·함수를 단순하고 명시적으로 유지한다.
================================================================================
"""

from __future__ import annotations

import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

# 셀 인덱스 타입 별칭: (i, j, k) 정수 3-튜플
Cell = tuple[int, int, int]

# ------------------------------------------------------------------------------
# 이웃 오프셋 상수
#   - NEIGHBORS_6  : 면(face)으로 맞닿은 6개 이웃 (±x, ±y, ±z). 직교 A* 이동 방향이자
#                    6-연결 팽창(inflation)에 사용. 두 셀 사이 맨해튼 거리 1.
#   - NEIGHBORS_26 : 면+모서리(edge)+꼭짓점(corner)으로 맞닿은 26개 이웃.
#                    26-연결 팽창에 사용. 3x3x3 큐브에서 중심을 뺀 나머지.
# ------------------------------------------------------------------------------
NEIGHBORS_6: tuple[Cell, ...] = (
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
)


def _offsets_26() -> tuple[Cell, ...]:
    """26-연결 이웃 오프셋을 생성한다.

    -1,0,1 을 세 축에 대해 모두 조합(3^3=27)하고, 자기 자신(0,0,0)만 제외한다.
    반환값: 길이 26 의 오프셋 튜플(각 원소는 (di, dj, dk)).
    """
    offsets: list[Cell] = []
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            for dk in (-1, 0, 1):
                if di == 0 and dj == 0 and dk == 0:
                    continue  # 중심 셀 자신은 이웃이 아님
                offsets.append((di, dj, dk))
    return tuple(offsets)


NEIGHBORS_26: tuple[Cell, ...] = _offsets_26()


@dataclass(frozen=True)
class AABB:
    """축 정렬 바운딩 박스(Axis-Aligned Bounding Box), 단위 mm.

    장애물을 가장 단순한 직육면체로 표현한 것. 생성 시 lo(최소 모서리) < hi(최대
    모서리) 를 반드시 만족해야 한다.

    필드:
        lo : (x_min, y_min, z_min) — 박스의 최소 좌표 (mm)
        hi : (x_max, y_max, z_max) — 박스의 최대 좌표 (mm)
    """

    lo: tuple[float, float, float]
    hi: tuple[float, float, float]

    def __post_init__(self) -> None:
        for a, b in zip(self.lo, self.hi):
            if b <= a:
                raise ValueError(f"AABB hi must be > lo, got lo={self.lo} hi={self.hi}")


# ==============================================================================
# 추상 베이스: OccupancyMap (질의 인터페이스 + 백엔드 공통 로직)
# ==============================================================================
class OccupancyMap(ABC):
    """점유맵 추상 베이스 클래스.

    좌표 변환·박스 복셀화 등 '저장 방식과 무관한' 로직은 여기서 공통 구현하고,
    실제 점유 비트의 저장/조회만 백엔드별로 구현한다(_get/_set/count_blocked/inflate).

    [A* 가 의존하는 공개 인터페이스 (백엔드 무관 계약)]
        is_blocked(cell) -> bool   : 점유 또는 격자 밖이면 True
        in_bounds(cell)  -> bool   : 격자 범위 안인가
        bounds()         -> (lo, hi): 셀 인덱스 범위(hi 제외)
        to_world / to_cell         : 셀 ↔ 월드좌표(mm) 변환
        world_bounds()             : 격자가 덮는 월드 AABB

    속성:
        shape   : (nx, ny, nz) 각 축 셀 개수
        origin  : 격자 원점 월드좌표(mm), numpy float64 길이 3
        cell_mm : 셀 한 변 길이(mm)
    """

    def __init__(
        self,
        shape: tuple[int, int, int],
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        cell_mm: float = 50.0,
    ) -> None:
        """공통 생성자: shape/origin/cell_mm 검증·저장 후 백엔드 저장소 초기화.

        매개변수:
            shape   : (nx, ny, nz) 셀 개수. 세 값 모두 양의 정수.
            origin  : 격자 원점 월드좌표(mm). 기본 (0,0,0).
            cell_mm : 셀 한 변 길이(mm). 기본 50.0. 양수.
        예외:
            ValueError : shape 가 3개 양의 정수가 아니거나 cell_mm <= 0.
        """
        shape = tuple(int(s) for s in shape)
        if len(shape) != 3 or any(s <= 0 for s in shape):
            raise ValueError(f"shape must be 3 positive ints, got {shape}")
        if cell_mm <= 0:
            raise ValueError(f"cell_mm must be positive, got {cell_mm}")

        self.shape: tuple[int, int, int] = shape
        self.origin: np.ndarray = np.asarray(origin, dtype=np.float64)
        self.cell_mm: float = float(cell_mm)
        self._init_storage()

    @classmethod
    def from_world_bounds(
        cls,
        world_lo: tuple[float, float, float],
        world_hi: tuple[float, float, float],
        cell_mm: float = 50.0,
    ) -> "OccupancyMap":
        """월드 좌표 범위 [lo, hi] (mm) 를 덮는 빈 점유맵 생성(셀 단위 올림).

        호출한 구체 클래스(cls)의 인스턴스를 만든다. 즉
        DenseOccupancyMap.from_world_bounds(...) 는 Dense 를 만든다.

        매개변수:
            world_lo : 영역 최소 좌표(mm). 각 축에서 world_hi 보다 작아야 함.
            world_hi : 영역 최대 좌표(mm).
            cell_mm  : 셀 한 변 길이(mm). 기본 50.0.
        예외:
            ValueError : 어느 축에서든 world_hi <= world_lo.
        """
        lo = np.asarray(world_lo, dtype=np.float64)
        hi = np.asarray(world_hi, dtype=np.float64)
        if np.any(hi <= lo):
            raise ValueError(f"world_hi must be > world_lo, got lo={lo} hi={hi}")
        extent = hi - lo
        shape = tuple(int(np.ceil(e / cell_mm)) for e in extent)
        return cls(shape=shape, origin=tuple(lo), cell_mm=cell_mm)

    # ------------------------------------------------------- 좌표/경계 (공통)

    def in_bounds(self, cell: Cell) -> bool:
        """셀 인덱스가 격자 범위 안인지 검사."""
        i, j, k = cell
        nx, ny, nz = self.shape
        return 0 <= i < nx and 0 <= j < ny and 0 <= k < nz

    def bounds(self) -> tuple[Cell, tuple[int, int, int]]:
        """셀 인덱스 범위 (lo 포함, hi 제외) = ((0,0,0), shape)."""
        return (0, 0, 0), self.shape

    def world_bounds(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """격자가 덮는 월드 AABB (lo_mm, hi_mm)."""
        lo = tuple(self.origin)
        hi = tuple(self.origin + np.asarray(self.shape, dtype=np.float64) * self.cell_mm)
        return lo, hi

    def to_world(self, cell: Cell) -> tuple[float, float, float]:
        """셀 인덱스 → 셀 중심 월드좌표(mm). world = origin + (cell+0.5)*cell_mm."""
        ijk = np.asarray(cell, dtype=np.float64)
        return tuple(self.origin + (ijk + 0.5) * self.cell_mm)

    def to_cell(self, world_mm: tuple[float, float, float]) -> Cell:
        """월드좌표(mm) → 포함 셀 인덱스. cell = floor((world-origin)/cell_mm)."""
        rel = (np.asarray(world_mm, dtype=np.float64) - self.origin) / self.cell_mm
        return tuple(int(np.floor(v)) for v in rel)

    # ------------------------------------------------- 질의/변경 (공통 wrapper)

    def is_blocked(self, cell: Cell) -> bool:
        """점유 여부. 격자 밖은 항상 True. (실제 비트 조회는 백엔드 _get)."""
        if not self.in_bounds(cell):
            return True
        return self._get(cell)

    def block_cell(self, cell: Cell) -> None:
        """단일 셀을 점유로 표시(격자 밖은 무시). 실제 set 은 백엔드 _set."""
        if self.in_bounds(cell):
            self._set(cell)

    def add_box(self, box: AABB) -> int:
        """AABB 박스를 복셀화하여 점유로 표시. 새로 점유된 셀 수를 반환.

        [알고리즘]
          1) lo/hi 월드좌표 → 셀 범위 (시작=floor, 끝=ceil 제외경계).
          2) 격자 [0,shape) 로 클리핑.
          3) 비어 있으면 0, 아니면 백엔드 _fill_box 로 채우고 신규 점유 수 반환.
        """
        lo = np.asarray(box.lo, dtype=np.float64)
        hi = np.asarray(box.hi, dtype=np.float64)
        cell_lo = np.floor((lo - self.origin) / self.cell_mm).astype(int)
        cell_hi = np.ceil((hi - self.origin) / self.cell_mm).astype(int)
        cell_lo = np.maximum(cell_lo, 0)
        cell_hi = np.minimum(cell_hi, np.asarray(self.shape))
        if np.any(cell_lo >= cell_hi):
            return 0
        return self._fill_box(
            (int(cell_lo[0]), int(cell_lo[1]), int(cell_lo[2])),
            (int(cell_hi[0]), int(cell_hi[1]), int(cell_hi[2])),
        )

    def add_boxes(self, boxes: list[AABB]) -> int:
        """여러 AABB 박스를 일괄 등록. 새로 점유된 셀 수의 합을 반환."""
        return sum(self.add_box(b) for b in boxes)

    def _fill_box(self, cell_lo: Cell, cell_hi: Cell) -> int:
        """[기본 구현] 셀 범위 [lo, hi) 를 점유로 채우고 신규 점유 수 반환.

        백엔드 무관한 안전한 기본 구현(셀 단위 루프). Dense 는 NumPy 로 오버라이드해
        가속한다. Sparse/BitPacked 는 이 기본 구현을 사용한다.
        """
        newly = 0
        for i in range(cell_lo[0], cell_hi[0]):
            for j in range(cell_lo[1], cell_hi[1]):
                for k in range(cell_lo[2], cell_hi[2]):
                    c = (i, j, k)
                    if not self._get(c):
                        self._set(c)
                        newly += 1
        return newly

    # ------------------------------------------------------ 백엔드 구현 필수

    @abstractmethod
    def _init_storage(self) -> None:
        """백엔드 저장소 초기화(생성자에서 호출). shape/origin/cell_mm 은 이미 설정됨."""

    @abstractmethod
    def _get(self, cell: Cell) -> bool:
        """격자 안(in-bounds 보장) 셀의 점유 비트 조회."""

    @abstractmethod
    def _set(self, cell: Cell) -> None:
        """격자 안(in-bounds 보장) 셀을 점유로 설정."""

    @abstractmethod
    def count_blocked(self) -> int:
        """점유된 셀의 총 개수."""

    @abstractmethod
    def inflate(self, radius_cells: int, *, connectivity: int = 6) -> "OccupancyMap":
        """장애물을 radius_cells 만큼 팽창시킨 같은 백엔드의 새 점유맵 반환.

        connectivity: 6(면 인접)→맨해튼 볼, 26(면+모서리+꼭짓점)→체비셰프 볼.
        """

    @abstractmethod
    def approx_bytes(self) -> int:
        """저장소가 대략 차지하는 바이트 수(백엔드 비교용 추정치)."""

    @abstractmethod
    def to_numpy(self) -> np.ndarray:
        """점유 상태를 (nx, ny, nz) bool NumPy 배열로 펼쳐 반환한다.

        백엔드와 무관하게 동일한 Dense bool 배열을 돌려준다. 시각화·내보내기
        (scene.txt) 등 격자 전체를 한 번에 다뤄야 할 때 사용한다.
        반환값은 내부 저장소와 독립적인 복사본이어야 한다.
        """


# ==============================================================================
# 백엔드 1: Dense (NumPy bool 배열)
# ==============================================================================
class DenseOccupancyMap(OccupancyMap):
    """NumPy bool 3D 배열 백엔드. 셀당 1바이트, 질의 O(1)로 가장 빠르고 단순.

    속성:
        grid : numpy.ndarray(bool), shape=(nx,ny,nz). True=점유.
    """

    def _init_storage(self) -> None:
        self.grid: np.ndarray = np.zeros(self.shape, dtype=bool)

    def _get(self, cell: Cell) -> bool:
        return bool(self.grid[cell])

    def _set(self, cell: Cell) -> None:
        self.grid[cell] = True

    def count_blocked(self) -> int:
        return int(self.grid.sum())

    def _fill_box(self, cell_lo: Cell, cell_hi: Cell) -> int:
        """NumPy 슬라이스로 가속한 박스 채우기(기본 루프 구현 오버라이드)."""
        slc = (
            slice(cell_lo[0], cell_hi[0]),
            slice(cell_lo[1], cell_hi[1]),
            slice(cell_lo[2], cell_hi[2]),
        )
        before = int(self.grid[slc].sum())
        self.grid[slc] = True
        after = int(self.grid[slc].sum())
        return after - before

    def inflate(self, radius_cells: int, *, connectivity: int = 6) -> "DenseOccupancyMap":
        if radius_cells < 0:
            raise ValueError("radius_cells must be >= 0")
        if connectivity not in (6, 26):
            raise ValueError("connectivity must be 6 or 26")
        out = DenseOccupancyMap(self.shape, tuple(self.origin), self.cell_mm)
        out.grid = self.grid.copy()
        if radius_cells == 0:
            return out
        offsets = NEIGHBORS_6 if connectivity == 6 else NEIGHBORS_26
        current = out.grid
        for _ in range(radius_cells):
            current = _dilate_once(current, offsets)
        out.grid = current
        return out

    def approx_bytes(self) -> int:
        return int(self.grid.nbytes)

    def to_numpy(self) -> np.ndarray:
        return self.grid.copy()


# ==============================================================================
# 백엔드 2: Sparse (점유 셀만 set 에 저장)
# ==============================================================================
class SparseOccupancyMap(OccupancyMap):
    """점유 셀 좌표만 set 에 저장하는 백엔드.

    점유가 *희박*할 때 메모리 절약. 단 set 엔트리당 오버헤드가 커서(수십 바이트),
    바닥·기둥처럼 점유가 빽빽하면 Dense 보다 메모리가 커질 수 있다.

    속성:
        blocked : set[Cell] — 점유된 셀 인덱스 집합.
    """

    def _init_storage(self) -> None:
        self.blocked: set[Cell] = set()

    def _get(self, cell: Cell) -> bool:
        return cell in self.blocked

    def _set(self, cell: Cell) -> None:
        self.blocked.add(cell)

    def count_blocked(self) -> int:
        return len(self.blocked)

    def inflate(self, radius_cells: int, *, connectivity: int = 6) -> "SparseOccupancyMap":
        if radius_cells < 0:
            raise ValueError("radius_cells must be >= 0")
        if connectivity not in (6, 26):
            raise ValueError("connectivity must be 6 or 26")
        out = SparseOccupancyMap(self.shape, tuple(self.origin), self.cell_mm)
        out.blocked = set(self.blocked)
        if radius_cells == 0:
            return out
        offsets = NEIGHBORS_6 if connectivity == 6 else NEIGHBORS_26
        nx, ny, nz = self.shape
        current = out.blocked
        for _ in range(radius_cells):
            nxt = set(current)
            for (i, j, k) in current:
                for (di, dj, dk) in offsets:
                    ni, nj, nk = i + di, j + dj, k + dk
                    if 0 <= ni < nx and 0 <= nj < ny and 0 <= nk < nz:
                        nxt.add((ni, nj, nk))
            current = nxt
        out.blocked = current
        return out

    def approx_bytes(self) -> int:
        # set 컨테이너 + 엔트리(작은 int 튜플) 추정치. 정밀하지 않은 비교용 값.
        per_entry = sys.getsizeof((0, 0, 0)) + 32  # 튜플 객체 + set 슬롯 대략
        return sys.getsizeof(self.blocked) + len(self.blocked) * per_entry

    def to_numpy(self) -> np.ndarray:
        grid = np.zeros(self.shape, dtype=bool)
        for (i, j, k) in self.blocked:
            grid[i, j, k] = True
        return grid


# ==============================================================================
# 백엔드 3: BitPacked (셀당 1비트, Dense 대비 1/8 메모리)
# ==============================================================================
class BitPackedOccupancyMap(OccupancyMap):
    """비트팩 백엔드. z 축 8셀을 1바이트에 담아 Dense 대비 메모리 1/8.

    같은 메모리로 약 8배 큰 ROI 를 Python 에서 다룰 수 있다. 비트 연산이 끼어
    질의는 Dense 보다 약간 느리다.

    속성:
        packed : numpy.ndarray(uint8), shape=(nx, ny, ceil(nz/8)).
                 비트 순서는 'little'(셀 k 는 byte 의 (k%8) 번째 비트).
    """

    def _init_storage(self) -> None:
        nx, ny, nz = self.shape
        self._nbytes_z = (nz + 7) // 8
        self.packed: np.ndarray = np.zeros((nx, ny, self._nbytes_z), dtype=np.uint8)

    def _get(self, cell: Cell) -> bool:
        i, j, k = cell
        byte = self.packed[i, j, k >> 3]
        return bool((byte >> (k & 7)) & 1)

    def _set(self, cell: Cell) -> None:
        i, j, k = cell
        self.packed[i, j, k >> 3] |= np.uint8(1 << (k & 7))

    def count_blocked(self) -> int:
        # 패딩 비트는 항상 0(유효 k 만 set)이므로 전체 set 비트 수 = 점유 셀 수.
        return int(np.unpackbits(self.packed).sum())

    def _to_dense_bool(self) -> np.ndarray:
        """내부 비트팩을 (nx,ny,nz) bool 배열로 펼친다(inflate 등에 사용)."""
        nz = self.shape[2]
        bits = np.unpackbits(self.packed, axis=2, bitorder="little")
        return bits[:, :, :nz].astype(bool)

    def _from_dense_bool(self, dense: np.ndarray) -> None:
        """(nx,ny,nz) bool 배열을 비트팩으로 저장한다."""
        self.packed = np.packbits(dense, axis=2, bitorder="little")
        self._nbytes_z = self.packed.shape[2]

    def inflate(self, radius_cells: int, *, connectivity: int = 6) -> "BitPackedOccupancyMap":
        if radius_cells < 0:
            raise ValueError("radius_cells must be >= 0")
        if connectivity not in (6, 26):
            raise ValueError("connectivity must be 6 or 26")
        # 팽창은 오프라인 전처리이므로, 비트팩을 잠시 Dense 로 펼쳐 계산 후 다시 팩.
        dense = self._to_dense_bool()
        if radius_cells > 0:
            offsets = NEIGHBORS_6 if connectivity == 6 else NEIGHBORS_26
            for _ in range(radius_cells):
                dense = _dilate_once(dense, offsets)
        out = BitPackedOccupancyMap(self.shape, tuple(self.origin), self.cell_mm)
        out._from_dense_bool(dense)
        return out

    def approx_bytes(self) -> int:
        return int(self.packed.nbytes)

    def to_numpy(self) -> np.ndarray:
        return self._to_dense_bool()


# ------------------------------------------------------------------ 팽창 헬퍼

def _shift(grid: np.ndarray, di: int, dj: int, dk: int) -> np.ndarray:
    """그리드를 (di, dj, dk) 만큼 평행이동(범위 밖은 0). 팽창의 이웃 시프트용.

    지역 변수 src_* / dst_* : 원본에서 복사할 구간 / 결과에서 채울 구간.
    """
    out = np.zeros_like(grid)
    nx, ny, nz = grid.shape
    src_i = slice(max(0, -di), nx - max(0, di))
    dst_i = slice(max(0, di), nx - max(0, -di))
    src_j = slice(max(0, -dj), ny - max(0, dj))
    dst_j = slice(max(0, dj), ny - max(0, -dj))
    src_k = slice(max(0, -dk), nz - max(0, dk))
    dst_k = slice(max(0, dk), nz - max(0, -dk))
    out[dst_i, dst_j, dst_k] = grid[src_i, src_j, src_k]
    return out


def _dilate_once(grid: np.ndarray, offsets: tuple[Cell, ...]) -> np.ndarray:
    """이웃 방향 OR 시프트로 1단계 이진 팽창. (Dense/BitPacked 공용)"""
    result = grid.copy()
    for di, dj, dk in offsets:
        np.logical_or(result, _shift(grid, di, dj, dk), out=result)
    return result

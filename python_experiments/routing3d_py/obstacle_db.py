"""장애물 DB 로더 (PostgreSQL → 점유맵) — Phase 1
================================================================================

[실행 명령어]  (모두 python_experiments/ 디렉토리에서)
  # ① DB 통계 출력 (행 수 / OST_TYPE 분포 / 전체 범위)
  ..\\.venv\\Scripts\\python.exe -m routing3d_py.obstacle_db --stats

  # ② 특정 타입·영역의 장애물을 점유맵으로 구성해 요약 출력
  #    --region 은 mm 단위 (min_x min_y min_z max_x max_y max_z)
  ..\\.venv\\Scripts\\python.exe -m routing3d_py.obstacle_db ^
      --types OST_Columns OST_Floors ^
      --region 199000 9000 14000 201000 11000 16000 ^
      --cell-mm 50

  # ③ 연결 단위 테스트 (DB 연결 안 되면 자동 skip)
  ..\\.venv\\Scripts\\python.exe -m pytest tests/test_obstacle_db.py -v

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
플랜트 BIM 장애물 데이터를 PostgreSQL 의 "TB_BIM_OBSTACLES" 테이블에서 읽어와
점유맵(OccupancyMap)으로 구성한다. 각 장애물의 축 정렬 바운딩 박스(AABB)는
MIN_X/Y/Z ~ MAX_X/Y/Z 필드로 표현되며, OST_TYPE(바닥/기둥/보 등)을 함께 읽어
점유 구성과 충돌 검사 시 타입별로 활용할 수 있게 한다.

[DB 정보] (인접 프로젝트 SpaceAI / RoutingAI 와 동일 환경)
  host=localhost  port=5432  dbname=AUTOROUTINGV7  user=postgres  password=dinno
  table=TB_BIM_OBSTACLES
  좌표 단위: 밀리미터(mm)  ← 본 프로젝트 규약과 동일
  주요 OST_TYPE: OST_Floors(바닥), OST_Columns(기둥), OST_BeamStartSegment(보),
                 OST_StructuralColumns, OST_StructuralFraming, OST_Ceilings, OST_Walls

[전체 흐름도]
--------------------------------------------------------------------------------
  PgConnConfig (접속 설정)
       │  connect()
       ▼
  load_obstacles(config, ost_types=…, region=…)   # SQL 로 타입·영역 필터
       │   → list[Obstacle]  (각 Obstacle 은 min/max + OST_TYPE 등 메타 보유)
       ▼
  build_occupancy(obstacles, cell_mm=50, region=…) # AABB 들을 복셀화
       │   → BuildResult(.occupancy = OccupancyMap, + 통계)
       ▼
  (이후) A* 탐색이 result.occupancy.is_blocked() 로 통과 가능 여부 질의

[주요 자료구조]
--------------------------------------------------------------------------------
  PgConnConfig : 접속 파라미터 (host/port/dbname/user/password/table).
  Obstacle     : 장애물 1건. object_id, name, ost_type, ddworks_type,
                 min_xyz(=AABB 최소), max_xyz(=AABB 최대).
  BuildResult  : build_occupancy 결과. .occupancy(점유맵) + 복셀화/스킵 통계.

[안전장치 / 주의]
--------------------------------------------------------------------------------
  - 테이블 전체는 약 75,000행, 전체 범위는 ~433m×85m×35m 로 매우 크다.
    50mm 셀로 전체를 Dense 그리드화하면 수십억 셀이 되어 메모리가 폭발한다.
    → 반드시 region(관심 영역) 또는 큰 cell_mm 으로 범위를 제한할 것.
    → build_occupancy 는 max_cells 한도를 초과하면 ValueError 로 막는다.
  - 일부 행은 어떤 축에서 max<=min 인 '퇴화 박스'(약 467건)다. 복셀화에서 스킵하고
    BuildResult 에 개수를 기록한다.
  - password 는 dev 기본값('dinno')을 두되 환경변수 PGPASSWORD 또는 인자로 덮어쓸 수
    있다. 운영 환경에서는 소스에 비밀번호를 두지 말 것.
================================================================================
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import psycopg2

from .occupancy import AABB, DenseOccupancyMap, OccupancyMap

# 점유맵 구성 시 허용하는 최대 셀 수(안전 한도). 약 8천만 셀 ≈ 80MB(bool).
# 이를 넘으면 region 을 좁히거나 cell_mm 을 키우라고 안내하며 막는다.
DEFAULT_MAX_CELLS = 80_000_000

# 윈도우 + 한글 PostgreSQL 환경에서 서버 메시지가 cp949 로 와서 psycopg2 가 UTF-8
# 디코딩에 실패하는 것을 피하기 위해, 접속 시 서버 메시지를 영문(C)으로 강제한다.
_CONNECT_OPTIONS = "-c client_encoding=UTF8 -c lc_messages=C"


@dataclass(frozen=True)
class PgConnConfig:
    """PostgreSQL 접속 설정.

    필드:
        host     : DB 호스트. 기본 'localhost'.
        port     : 포트. 기본 5432.
        dbname   : 데이터베이스 이름. 기본 'AUTOROUTINGV7'.
        user     : 사용자. 기본 'postgres'.
        password : 비밀번호. 기본 'dinno'(dev). 환경변수 PGPASSWORD 로 덮어쓰기 권장.
        table    : 장애물 테이블 이름. 기본 'TB_BIM_OBSTACLES'.
        connect_timeout : 접속 타임아웃(초). 기본 5.
    """

    host: str = "localhost"
    port: int = 5432
    dbname: str = "AUTOROUTINGV7"
    user: str = "postgres"
    password: str = "dinno"
    table: str = "TB_BIM_OBSTACLES"
    connect_timeout: int = 5

    @classmethod
    def from_env(cls, **overrides) -> "PgConnConfig":
        """환경변수(PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD)에서 설정을 읽어 생성한다.

        환경변수가 없으면 클래스 기본값을 사용한다. overrides 로 개별 값을 직접 덮어쓸
        수 있다(가장 우선).

        예: PgConnConfig.from_env(dbname="OTHER_DB")
        """
        base = dict(
            host=os.environ.get("PGHOST", cls.host),
            port=int(os.environ.get("PGPORT", cls.port)),
            dbname=os.environ.get("PGDATABASE", cls.dbname),
            user=os.environ.get("PGUSER", cls.user),
            password=os.environ.get("PGPASSWORD", cls.password),
        )
        base.update(overrides)
        return cls(**base)

    def connect(self):
        """psycopg2 연결을 생성해 반환한다(client_encoding=UTF8 강제).

        반환값:
            psycopg2 connection 객체. 호출자가 close() 책임을 진다.
        예외:
            psycopg2.OperationalError 등 — DB 미가동/권한 오류 시.
        """
        conn = psycopg2.connect(
            host=self.host,
            port=self.port,
            dbname=self.dbname,
            user=self.user,
            password=self.password,
            connect_timeout=self.connect_timeout,
            options=_CONNECT_OPTIONS,
        )
        conn.set_client_encoding("UTF8")
        return conn


@dataclass(frozen=True)
class Obstacle:
    """장애물 1건. AABB(min/max) + 의미 메타데이터.

    필드:
        object_id    : BIM 객체 ID (OBJECT_ID).
        name         : 객체 이름 (NAME).
        ost_type     : 장애물 타입 (OST_TYPE). 예: 'OST_Floors', 'OST_Columns'.
        ddworks_type : DDWorks 분류 (DDWORKS_TYPE). 예: 'FLOOR_ARCHITECTURE'.
        min_xyz      : AABB 최소 좌표 (mm) = (MIN_X, MIN_Y, MIN_Z).
        max_xyz      : AABB 최대 좌표 (mm) = (MAX_X, MAX_Y, MAX_Z).
    """

    object_id: str | None
    name: str | None
    ost_type: str | None
    ddworks_type: str | None
    min_xyz: tuple[float, float, float]
    max_xyz: tuple[float, float, float]

    def is_valid_box(self) -> bool:
        """세 축 모두 max > min 인 정상 박스인지 검사한다(부피가 양수).

        반환값:
            정상이면 True, 퇴화(어느 축에서 max<=min)면 False.
        """
        return all(b > a for a, b in zip(self.min_xyz, self.max_xyz))

    def to_aabb(self) -> AABB:
        """이 장애물을 AABB 로 변환한다.

        반환값:
            AABB(lo=min_xyz, hi=max_xyz).
        예외:
            ValueError : 퇴화 박스일 때(AABB 가 lo<hi 를 강제하므로).
        """
        return AABB(lo=self.min_xyz, hi=self.max_xyz)


@dataclass
class BuildResult:
    """build_occupancy 의 결과 묶음.

    필드:
        occupancy            : 구성된 OccupancyMap.
        n_obstacles          : 입력 장애물 총 개수.
        n_voxelized          : 실제로 복셀화(점유 표시)된 장애물 수.
        n_skipped_degenerate : 퇴화 박스라 스킵된 수.
        n_skipped_outside    : 영역(region) 밖이라 스킵된 수.
        region_lo / region_hi: 점유맵이 덮는 영역(mm).
    """

    occupancy: OccupancyMap
    n_obstacles: int
    n_voxelized: int
    n_skipped_degenerate: int
    n_skipped_outside: int
    region_lo: tuple[float, float, float]
    region_hi: tuple[float, float, float]

    def summary(self) -> str:
        """사람이 읽기 좋은 한 줄 요약 문자열."""
        nx, ny, nz = self.occupancy.shape
        return (
            f"[점유맵] 셀 {nx}x{ny}x{nz} (cell={self.occupancy.cell_mm:g}mm) | "
            f"장애물 {self.n_obstacles}건 중 복셀화 {self.n_voxelized}, "
            f"퇴화 스킵 {self.n_skipped_degenerate}, 영역밖 스킵 {self.n_skipped_outside} | "
            f"점유셀 {self.occupancy.count_blocked()}"
        )


# ------------------------------------------------------------------ 내부 헬퍼

def _aabb_overlaps(
    lo1: tuple[float, float, float],
    hi1: tuple[float, float, float],
    lo2: tuple[float, float, float],
    hi2: tuple[float, float, float],
) -> bool:
    """두 AABB 가 겹치는지(분리축 없음) 검사한다.

    매개변수:
        lo1, hi1 : 박스 A 의 최소/최대.
        lo2, hi2 : 박스 B 의 최소/최대.
    반환값:
        세 축 모두에서 구간이 겹치면 True.
    """
    return all(hi1[i] >= lo2[i] and lo1[i] <= hi2[i] for i in range(3))


# ------------------------------------------------------------------ 로딩

def distinct_ost_types(config: PgConnConfig | None = None, conn=None) -> dict[str, int]:
    """OST_TYPE 별 행 개수를 조회해 {타입: 개수} 로 반환한다.

    매개변수:
        config : 접속 설정. conn 이 주어지면 무시.
        conn   : 재사용할 기존 연결(선택). 없으면 config 로 새로 열고 닫는다.
    반환값:
        {ost_type: count} 딕셔너리(개수 내림차순).
    """
    config = config or PgConnConfig()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute(
            f'SELECT "OST_TYPE", COUNT(*) FROM "{config.table}" '
            f'GROUP BY "OST_TYPE" ORDER BY COUNT(*) DESC'
        )
        return {row[0]: row[1] for row in cur.fetchall()}
    finally:
        if own:
            conn.close()


def load_obstacles(
    config: PgConnConfig | None = None,
    *,
    source_file: str | None = None,
    ost_types: list[str] | None = None,
    region: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None,
    exclude_empty_type: bool = False,
    skip_degenerate: bool = True,
    limit: int | None = None,
    conn=None,
) -> list[Obstacle]:
    """장애물을 DB 에서 읽어 Obstacle 리스트로 반환한다.

    [알고리즘]
      1) WHERE 절을 동적으로 구성한다.
         - source_file      → "SOURCE_FILE" = %s  (프로젝트 단위 필터)
         - ost_types        → "OST_TYPE" IN (...)
         - region(AABB)      → 세 축 겹침 조건 (MAX>=region_lo AND MIN<=region_hi)
         - exclude_empty_type→ OST_TYPE 가 빈 문자열/NULL 인 행 제외
         - skip_degenerate   → max<=min 인 퇴화 박스를 SQL 단계에서 제외
      2) 파라미터 바인딩(%s)으로 안전하게 질의한다(SQL 인젝션 방지).
      3) 각 행을 Obstacle 로 변환해 리스트로 반환한다.

    매개변수:
        config             : 접속 설정. conn 이 주어지면 무시.
        source_file        : 프로젝트의 SOURCE_FILE(예: 'CLEAN_WTNHJ03_..._total.json').
                             이 프로젝트의 장애물만. None=전체 프로젝트.
        ost_types          : 가져올 타입 목록(예: ['OST_Columns','OST_Floors']). None=전부.
        region             : ((min_x,min_y,min_z),(max_x,max_y,max_z)) mm. 이 영역과
                             겹치는 장애물만. None=전체 영역.
        exclude_empty_type : True 면 OST_TYPE 가 비었거나 NULL 인 행 제외.
        skip_degenerate    : True(기본)면 퇴화 박스를 SQL 에서 제외.
        limit              : 최대 행 수(디버그용). None=제한 없음.
        conn               : 재사용할 기존 연결(선택).
    반환값:
        list[Obstacle].

    지역 변수:
        where  : WHERE 조건 문자열 조각 리스트.
        params : %s 자리에 바인딩할 값 리스트.
    """
    config = config or PgConnConfig()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        where: list[str] = []
        params: list[object] = []

        if source_file is not None:
            where.append('"SOURCE_FILE" = %s')
            params.append(source_file)

        if ost_types:
            placeholders = ", ".join(["%s"] * len(ost_types))
            where.append(f'"OST_TYPE" IN ({placeholders})')
            params.extend(ost_types)

        if exclude_empty_type:
            where.append('"OST_TYPE" IS NOT NULL')
            where.append('"OST_TYPE" <> \'\'')

        if region is not None:
            (rlx, rly, rlz), (rhx, rhy, rhz) = region
            # AABB 겹침: 각 축에서 (박스 MAX >= 영역 lo) AND (박스 MIN <= 영역 hi)
            where.append('"MAX_X" >= %s AND "MIN_X" <= %s')
            params.extend([rlx, rhx])
            where.append('"MAX_Y" >= %s AND "MIN_Y" <= %s')
            params.extend([rly, rhy])
            where.append('"MAX_Z" >= %s AND "MIN_Z" <= %s')
            params.extend([rlz, rhz])

        if skip_degenerate:
            where.append('"MAX_X" > "MIN_X" AND "MAX_Y" > "MIN_Y" AND "MAX_Z" > "MIN_Z"')

        sql = (
            'SELECT "OBJECT_ID", "NAME", "OST_TYPE", "DDWORKS_TYPE", '
            '"MIN_X", "MIN_Y", "MIN_Z", "MAX_X", "MAX_Y", "MAX_Z" '
            f'FROM "{config.table}"'
        )
        if where:
            sql += " WHERE " + " AND ".join(where)
        if limit is not None:
            sql += " LIMIT %s"
            params.append(limit)

        cur = conn.cursor()
        cur.execute(sql, params)

        obstacles: list[Obstacle] = []
        for oid, name, ost, ddw, mnx, mny, mnz, mxx, mxy, mxz in cur.fetchall():
            obstacles.append(
                Obstacle(
                    object_id=oid,
                    name=name,
                    ost_type=ost,
                    ddworks_type=ddw,
                    min_xyz=(float(mnx), float(mny), float(mnz)),
                    max_xyz=(float(mxx), float(mxy), float(mxz)),
                )
            )
        return obstacles
    finally:
        if own:
            conn.close()


def obstacles_bounds(
    obstacles: list[Obstacle],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """장애물 리스트 전체를 감싸는 AABB(lo, hi) 를 mm 로 계산한다.

    매개변수:
        obstacles : Obstacle 리스트(비어 있으면 ValueError).
    반환값:
        ((min_x,min_y,min_z),(max_x,max_y,max_z)).
    예외:
        ValueError : 리스트가 비었을 때.
    """
    if not obstacles:
        raise ValueError("obstacles is empty; cannot compute bounds")
    mins = np.array([o.min_xyz for o in obstacles], dtype=np.float64)
    maxs = np.array([o.max_xyz for o in obstacles], dtype=np.float64)
    lo = tuple(float(v) for v in mins.min(axis=0))
    hi = tuple(float(v) for v in maxs.max(axis=0))
    return lo, hi


def group_by_type(obstacles: list[Obstacle]) -> dict[str, list[Obstacle]]:
    """장애물을 OST_TYPE 별로 묶는다(타입별 차등 클리어런스/충돌 검사에 활용).

    매개변수:
        obstacles : Obstacle 리스트.
    반환값:
        {ost_type: [Obstacle, ...]}. None 타입은 키 '' 로 묶는다.
    """
    groups: dict[str, list[Obstacle]] = {}
    for o in obstacles:
        groups.setdefault(o.ost_type or "", []).append(o)
    return groups


# ------------------------------------------------------------------ 점유맵 구성

def build_occupancy(
    obstacles: list[Obstacle],
    cell_mm: float = 50.0,
    *,
    region: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None,
    padding_mm: float = 0.0,
    max_cells: int = DEFAULT_MAX_CELLS,
    backend: type[OccupancyMap] = DenseOccupancyMap,
) -> BuildResult:
    """장애물 리스트를 복셀화하여 OccupancyMap 을 구성한다.

    [알고리즘]
      1) 점유맵이 덮을 영역(region)을 정한다.
         - region 인자가 있으면 그대로 사용.
         - 없으면 모든 장애물을 감싸는 AABB 를 자동 계산.
         - padding_mm 만큼 사방으로 여유를 준다.
      2) 영역 셀 수가 max_cells 를 넘으면 ValueError(메모리 폭발 방지).
      3) backend.from_world_bounds 로 빈 점유맵 생성(백엔드 교체 가능).
      4) 각 장애물을 검사하며 복셀화:
         - 퇴화 박스 → 스킵(카운트).
         - 영역과 안 겹치면 → 스킵(카운트).
         - 그 외 → add_box 로 점유 표시(카운트).

    매개변수:
        obstacles  : Obstacle 리스트.
        cell_mm    : 셀 크기(mm). 기본 50.
        region     : 점유맵이 덮을 영역((lo),(hi)) mm. None=장애물 전체 범위 자동.
        padding_mm : 영역 사방 여유(mm). 기본 0.
        max_cells  : 허용 최대 셀 수(안전 한도).
        backend    : 점유맵 백엔드 클래스. 기본 DenseOccupancyMap.
                     점유가 희박하면 SparseOccupancyMap, 메모리를 아끼려면
                     BitPackedOccupancyMap 으로 교체 가능(질의 인터페이스 동일).
    반환값:
        BuildResult (.occupancy + 통계).
    예외:
        ValueError : obstacles 가 비었거나(region 도 없음), 셀 수가 한도 초과일 때.

    지역 변수:
        lo, hi   : 점유맵이 덮을 영역(패딩 적용 후).
        shape    : 축별 셀 개수.
        n_cells  : 총 셀 수(한도 검사용).
    """
    if region is not None:
        lo = np.asarray(region[0], dtype=np.float64)
        hi = np.asarray(region[1], dtype=np.float64)
    else:
        b_lo, b_hi = obstacles_bounds(obstacles)  # 비었으면 여기서 ValueError
        lo = np.asarray(b_lo, dtype=np.float64)
        hi = np.asarray(b_hi, dtype=np.float64)

    lo = lo - padding_mm
    hi = hi + padding_mm

    shape = tuple(int(np.ceil((hi[i] - lo[i]) / cell_mm)) for i in range(3))
    if any(s <= 0 for s in shape):
        raise ValueError(f"invalid region produces non-positive shape: {shape}")

    n_cells = shape[0] * shape[1] * shape[2]
    if n_cells > max_cells:
        raise ValueError(
            f"grid too large: {shape} = {n_cells:,} cells > max_cells({max_cells:,}). "
            f"region 을 좁히거나 cell_mm 을 키우세요. "
            f"(현재 cell_mm={cell_mm}, 영역 크기 mm={tuple((hi - lo).round(1))})"
        )

    occ = backend.from_world_bounds(tuple(lo), tuple(hi), cell_mm=cell_mm)

    region_lo = tuple(lo)
    region_hi = tuple(hi)
    n_vox = 0
    n_degen = 0
    n_outside = 0
    for o in obstacles:
        if not o.is_valid_box():
            n_degen += 1
            continue
        if not _aabb_overlaps(o.min_xyz, o.max_xyz, region_lo, region_hi):
            n_outside += 1
            continue
        occ.add_box(o.to_aabb())
        n_vox += 1

    return BuildResult(
        occupancy=occ,
        n_obstacles=len(obstacles),
        n_voxelized=n_vox,
        n_skipped_degenerate=n_degen,
        n_skipped_outside=n_outside,
        region_lo=region_lo,
        region_hi=region_hi,
    )


# ------------------------------------------------------------------ CLI 진입점

def _main(argv: list[str] | None = None) -> int:
    """커맨드라인 진입점. 상단 [실행 명령어] 참고.

    --stats              : DB 통계(행수/타입분포/전체범위) 출력.
    --types T [T ...]    : 가져올 OST_TYPE 목록.
    --region 6값         : min_x min_y min_z max_x max_y max_z (mm).
    --cell-mm F          : 셀 크기(mm). 기본 50.
    --limit N            : 최대 행 수.
    """
    import argparse
    import sys

    # 윈도우 콘솔(cp949)에서도 한글이 깨지지 않도록 출력 인코딩을 UTF-8 로 맞춘다.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(description="TB_BIM_OBSTACLES 장애물 로더/점유맵 구성")
    parser.add_argument("--stats", action="store_true", help="DB 통계만 출력")
    parser.add_argument("--types", nargs="+", default=None, help="가져올 OST_TYPE 목록")
    parser.add_argument("--region", nargs=6, type=float, default=None,
                        metavar=("MINX", "MINY", "MINZ", "MAXX", "MAXY", "MAXZ"),
                        help="관심 영역 mm (min xyz, max xyz)")
    parser.add_argument("--cell-mm", type=float, default=50.0, help="셀 크기 mm (기본 50)")
    parser.add_argument("--limit", type=int, default=None, help="최대 행 수")
    parser.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = parser.parse_args(argv)

    overrides = {}
    if args.dbname:
        overrides["dbname"] = args.dbname
    config = PgConnConfig.from_env(**overrides)

    if args.stats:
        conn = config.connect()
        try:
            cur = conn.cursor()
            cur.execute(f'SELECT COUNT(*) FROM "{config.table}"')
            print(f"테이블 {config.table}: {cur.fetchone()[0]:,} 행")
            print("OST_TYPE 분포:")
            for t, n in distinct_ost_types(conn=conn, config=config).items():
                print(f"  {t!r}: {n:,}")
            cur.execute(
                f'SELECT MIN("MIN_X"),MAX("MAX_X"),MIN("MIN_Y"),MAX("MAX_Y"),'
                f'MIN("MIN_Z"),MAX("MAX_Z") FROM "{config.table}"'
            )
            mnx, mxx, mny, mxy, mnz, mxz = cur.fetchone()
            print(f"전체 범위 mm: X[{mnx:.0f},{mxx:.0f}] Y[{mny:.0f},{mxy:.0f}] Z[{mnz:.0f},{mxz:.0f}]")
        finally:
            conn.close()
        return 0

    region = None
    if args.region is not None:
        region = (tuple(args.region[0:3]), tuple(args.region[3:6]))

    obstacles = load_obstacles(
        config, ost_types=args.types, region=region, limit=args.limit
    )
    print(f"로드된 장애물: {len(obstacles):,}건")
    if not obstacles:
        print("조건에 맞는 장애물이 없습니다.")
        return 0

    result = build_occupancy(obstacles, cell_mm=args.cell_mm, region=region)
    print(result.summary())
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

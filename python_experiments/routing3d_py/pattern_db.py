r"""스텁 패턴 벡터 저장소 DB 레이어 (PostgreSQL + pgvector) — 패턴 학습 L1

================================================================================
[실행 명령어]  (모두 프로젝트 루트에서)
  # ① 스키마 적용(테이블/뷰/인덱스 생성, 멱등)
  .\.venv\Scripts\python.exe -m routing3d_py.pattern_db --apply-schema

  # ② 저장소 통계(표본 수 / 키별 대표 템플릿)
  .\.venv\Scripts\python.exe -m routing3d_py.pattern_db --stats

  # ③ 특정 프로젝트의 적재 표본 삭제(재학습 전 정리)
  .\.venv\Scripts\python.exe -m routing3d_py.pattern_db --clear-source "CLEAN_..._total.json"
================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
route_stub_pattern(스텁 표본) 테이블의 생성·적재·조회를 담당하는 얇은 DB 레이어다.
실제 '학습'(스텁 추출·정규화·특징벡터)은 pattern_learn.py 가 하고, 이 모듈은 그 결과를
pgvector 컬럼(dir_unit vector(3), feat vector(24))로 영속화/질의한다.

[스키마] db/schema/route_stub_pattern.sql (단일 출처). apply_schema() 가 그 파일을 실행한다.

[pgvector 적재 규약]
  psycopg2 는 vector 타입을 모르므로, 벡터를 '[a,b,c]' 텍스트 리터럴로 바인딩한다
  (pgvector 가 텍스트 표현을 허용). vec_literal() 이 그 포맷을 만든다. None → SQL NULL.

[멱등 재학습]
  같은 프로젝트를 다시 학습할 때는 clear_source(source_file) 로 기존 표본을 지운 뒤
  insert_samples() 로 새로 넣는다(중복 누적 방지).
================================================================================
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

from .obstacle_db import PgConnConfig

# 특징벡터 차원(스키마 vector(24) 와 반드시 일치). pattern_learn.build_feature_vector 와 공유.
FEAT_DIM = 24

# db/schema/route_stub_pattern.sql 절대경로(이 파일 → routing3d_py → python_experiments → 루트).
_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
SCHEMA_SQL = os.path.join(_REPO_ROOT, "db", "schema", "route_stub_pattern.sql")


@dataclass
class StubSampleRow:
    """route_stub_pattern 한 행에 적재할 값(특징벡터까지 계산 완료된 상태).

    필드(스키마 컬럼과 1:1):
        source_file/route_path_guid/anchor_kind/anchor_name : 출처.
        utility_group/utility                               : 범주 키.
        poc_pos/anchor_min/anchor_max                       : 원자료 좌표(mm) 3-튜플.
        face/dir_seq/n_bends/rise_mm/offset_mm/diameter_mm  : 정규화 스텁 형상.
        dir_unit (len 3) / feat (len FEAT_DIM)              : pgvector 값(리스트).
    """

    source_file: str
    anchor_kind: str                       # 'EQUIP' | 'DUCT'
    poc_pos: tuple[float, float, float]
    utility_group: str | None = None
    utility: str | None = None
    route_path_guid: str | None = None
    anchor_name: str | None = None
    anchor_min: tuple[float, float, float] | None = None
    anchor_max: tuple[float, float, float] | None = None
    face: str | None = None
    dir_seq: str | None = None
    n_bends: int | None = None
    rise_mm: float | None = None
    offset_mm: float | None = None
    diameter_mm: float | None = None
    dir_unit: list[float] | None = None
    feat: list[float] | None = None


# ------------------------------------------------------------------ 포맷 헬퍼

def vec_literal(values: list[float] | tuple[float, ...] | None) -> str | None:
    """파이썬 실수 리스트를 pgvector 텍스트 리터럴 '[a,b,c]' 로 만든다.

    매개변수:
        values : 실수 시퀀스 또는 None.
    반환값:
        '[1,2,3]' 형태 문자열. values 가 None 이면 None(→ SQL NULL).
    """
    if values is None:
        return None
    return "[" + ",".join(f"{float(v):.6g}" for v in values) + "]"


def _arr(values: tuple[float, float, float] | None) -> list[float] | None:
    """3-튜플을 double precision[] 바인딩용 리스트로(또는 None)."""
    return None if values is None else [float(v) for v in values]


# ------------------------------------------------------------------ 스키마

def apply_schema(config: PgConnConfig | None = None, conn=None) -> None:
    """db/schema/route_stub_pattern.sql 을 실행해 테이블/뷰/인덱스를 생성한다(멱등).

    매개변수:
        config : 접속 설정. conn 이 주어지면 무시.
        conn   : 재사용할 기존 연결(선택). 없으면 config 로 열고 닫는다.
    예외:
        FileNotFoundError : 스키마 파일이 없을 때.
        psycopg2.Error    : 권한/문법 오류.
    """
    if not os.path.isfile(SCHEMA_SQL):
        raise FileNotFoundError(f"schema not found: {SCHEMA_SQL}")
    with open(SCHEMA_SQL, "r", encoding="utf-8") as f:
        sql = f.read()
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute(sql)
        conn.commit()
    finally:
        if own:
            conn.close()


# ------------------------------------------------------------------ 적재/정리

def clear_source(source_file: str, config: PgConnConfig | None = None, conn=None) -> int:
    """한 프로젝트(source_file)의 기존 표본을 모두 삭제한다(재학습 전 정리).

    반환값:
        삭제된 행 수.
    """
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute('DELETE FROM route_stub_pattern WHERE source_file = %s', (source_file,))
        n = cur.rowcount
        conn.commit()
        return n
    finally:
        if own:
            conn.close()


def insert_samples(rows: list[StubSampleRow], config: PgConnConfig | None = None,
                   conn=None, batch: int = 500) -> int:
    """StubSampleRow 들을 route_stub_pattern 에 적재한다.

    [알고리즘]
      행마다 17개 컬럼을 바인딩한다. double[] 컬럼은 파이썬 리스트로, vector 컬럼은
      vec_literal() 의 '[...]' 텍스트로 바인딩(::vector 캐스트). batch 단위로 executemany.

    매개변수:
        rows   : 적재할 표본.
        config : 접속 설정. conn 우선.
        conn   : 재사용 연결(선택).
        batch  : executemany 묶음 크기.
    반환값:
        적재된 행 수.
    """
    if not rows:
        return 0
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        sql = (
            "INSERT INTO route_stub_pattern "
            "(source_file, route_path_guid, anchor_kind, anchor_name, utility_group, utility, "
            " poc_pos, anchor_min, anchor_max, face, dir_seq, n_bends, rise_mm, offset_mm, "
            " diameter_mm, dir_unit, feat) "
            "VALUES (%s,%s,%s,%s,%s,%s, %s,%s,%s, %s,%s,%s,%s,%s, %s, %s::vector, %s::vector)"
        )
        params = [
            (
                r.source_file, r.route_path_guid, r.anchor_kind, r.anchor_name,
                r.utility_group, r.utility,
                _arr(r.poc_pos), _arr(r.anchor_min), _arr(r.anchor_max),
                r.face, r.dir_seq, r.n_bends, r.rise_mm, r.offset_mm,
                r.diameter_mm, vec_literal(r.dir_unit), vec_literal(r.feat),
            )
            for r in rows
        ]
        for i in range(0, len(params), batch):
            cur.executemany(sql, params[i:i + batch])
        conn.commit()
        return len(params)
    finally:
        if own:
            conn.close()


# ------------------------------------------------------------------ 조회

def count_samples(config: PgConnConfig | None = None, conn=None) -> int:
    """route_stub_pattern 전체 행 수."""
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute("SELECT count(*) FROM route_stub_pattern")
        return int(cur.fetchone()[0])
    finally:
        if own:
            conn.close()


def fetch_templates(config: PgConnConfig | None = None, conn=None) -> list[tuple]:
    """집계 뷰 route_stub_template 전체를 표본 수 내림차순으로 반환한다.

    반환값:
        (anchor_kind, utility_group, utility, face, dir_seq, rise_mm, offset_mm,
         diameter_mm, avg_bends, n) 튜플 리스트.
    """
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        cur.execute(
            "SELECT anchor_kind, utility_group, utility, face, dir_seq, rise_mm, offset_mm, "
            "diameter_mm, avg_bends, n FROM route_stub_template ORDER BY n DESC"
        )
        return cur.fetchall()
    finally:
        if own:
            conn.close()


def nearest_stubs(anchor_kind: str, utility_group: str | None, utility: str | None,
                  feat: list[float], k: int = 5, *, strict_category: bool = True,
                  config: PgConnConfig | None = None, conn=None) -> list[tuple]:
    """추론 프리미티브 — 범주로 pre-filter 한 뒤 feat L2 거리로 최근접 스텁 K개를 검색한다.

    [원칙] 범주(anchor_kind/utility_group/utility)는 절대 제약(유틸 위반 금지)이므로 WHERE 로
           먼저 거르고, 그 안에서만 벡터 ANN(feat <-> query) 으로 형상이 비슷한 스텁을 고른다.
           strict_category=False 면 결과가 비었을 때 utility 무시 → group 만으로 폴백한다.

    매개변수:
        anchor_kind     : 'EQUIP' | 'DUCT'.
        utility_group   : 범주 그룹(예 'Exhaust'). None 허용(NULL 매칭).
        utility         : 범주 유틸(예 'ACID'). None 허용.
        feat            : 질의 특징벡터(길이 FEAT_DIM).
        k               : 최근접 개수.
        strict_category : True 면 (group, utility) 모두 일치. False 면 빈 결과 시 group 만으로 재시도.
    반환값:
        (id, face, dir_seq, rise_mm, offset_mm, poc_pos, anchor_min, anchor_max, dist) 리스트(가까운 순).
    """
    config = config or PgConnConfig.from_env()
    own = conn is None
    if own:
        conn = config.connect()
    try:
        cur = conn.cursor()
        q = vec_literal(feat)
        cols = ("id, face, dir_seq, rise_mm, offset_mm, poc_pos, anchor_min, anchor_max, "
                "feat <-> %s::vector AS dist")

        def run(use_util: bool) -> list[tuple]:
            where = ['anchor_kind = %s', 'utility_group IS NOT DISTINCT FROM %s']
            params: list[object] = [q, anchor_kind, utility_group]
            if use_util:
                where.append('utility IS NOT DISTINCT FROM %s')
                params.append(utility)
            params += [q, k]
            cur.execute(
                f"SELECT {cols} FROM route_stub_pattern WHERE {' AND '.join(where)} "
                f"ORDER BY feat <-> %s::vector LIMIT %s", params)
            return cur.fetchall()

        rows = run(use_util=True)
        if not rows and not strict_category:
            rows = run(use_util=False)
        return rows
    finally:
        if own:
            conn.close()


# ------------------------------------------------------------------ CLI

def _main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(description="스텁 패턴 벡터 저장소(route_stub_pattern) DB 레이어")
    ap.add_argument("--apply-schema", action="store_true", help="스키마(테이블/뷰/인덱스) 생성")
    ap.add_argument("--stats", action="store_true", help="표본 수 + 키별 대표 템플릿 출력")
    ap.add_argument("--clear-source", metavar="SOURCE_FILE", default=None,
                    help="해당 프로젝트의 표본 전체 삭제")
    ap.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = ap.parse_args(argv)

    overrides = {}
    if args.dbname:
        overrides["dbname"] = args.dbname
    config = PgConnConfig.from_env(**overrides)

    did = False
    if args.apply_schema:
        apply_schema(config)
        print(f"스키마 적용 완료: {SCHEMA_SQL}")
        did = True
    if args.clear_source:
        n = clear_source(args.clear_source, config)
        print(f"삭제: {n} 행 (source_file={args.clear_source})")
        did = True
    if args.stats or not did:
        total = count_samples(config)
        print(f"route_stub_pattern: {total:,} 표본")
        templates = fetch_templates(config)
        if templates:
            print(f"대표 템플릿 {len(templates)}종 (표본수 내림차순):")
            print(f"  {'kind':5} {'group':12} {'util':12} {'face':4} {'dir_seq':10} "
                  f"{'rise':>7} {'off':>6} {'n':>5}")
            for k, g, u, face, ds, rise, off, dia, ab, n in templates:
                print(f"  {k or '':5} {(g or '')[:12]:12} {(u or '')[:12]:12} "
                      f"{face or '':4} {(ds or '')[:10]:10} "
                      f"{(rise or 0):7.0f} {(off or 0):6.0f} {n:5d}")
        else:
            print("(아직 적재된 표본 없음 — pattern_learn.py 로 학습/적재하세요)")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

-- =============================================================================
-- route_bundle_group.sql — 그룹(번들) 배관 탐지 결과 저장소
-- =============================================================================
-- [목적]
--   기존 설계배관(TB_ROUTE_PATH)을 장비명·유틸리티별로 묶어 경로 형태 유사도로 탐지한
--   '번들 그룹'(동일 이격간격으로 2회 이상 수직·수평 꺾임을 공유하는 평행 다발)을 1행=1그룹으로
--   적재한다. routing3d_py.bundle_detect 가 생성한다.
--
-- [적용]  (프로젝트 루트, .venv 에 psycopg2 존재)
--   .\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --write-db
--   또는 psql:  psql "host=localhost dbname=AUTOROUTINGV7 user=postgres" -f db/schema/route_bundle_group.sql
--
-- [단위]  좌표·치수 모두 mm. 라우트=BIM 동일 월드 프레임.
-- =============================================================================

-- 1행 = 탐지된 번들 그룹 1개.
CREATE TABLE IF NOT EXISTS route_bundle_group (
    id              bigserial PRIMARY KEY,

    -- ── 출처 ──
    source_file     text        NOT NULL,            -- 프로젝트(SOURCE_FILE)
    group_id        int         NOT NULL,            -- 프로젝트 내 그룹 번호(0..)

    -- ── 그룹 키 ──
    owner_name      text,                            -- 장비명(SOURCE_OWNER_NAME)
    utility         text,                            -- 유틸리티(SOURCE_UTILITY)

    -- ── 지표 ──
    n_members       int         NOT NULL,            -- 그룹 내 배관 수(>=2)
    avg_similarity  double precision,                -- 그룹 내 경로쌍 평균 유사도(0..1)
    trunk_z         double precision,                -- 주경로 공용 고도(mm)
    trunk_xy_spread double precision,                -- 주경로 내 배관 간 최대 수평 벌어짐(다발 폭, mm)
    pitch_mm        double precision,                -- 인접 배관 대표 이격간격(중앙값, mm)
    n_ortho_bends   int,                             -- 대표 수직·수평 꺾임 수(>=2)
    arrow_code      text,                            -- 대표 형태 부호(R/H/D 시퀀스)

    -- ── 멤버 ──
    member_guids    text[],                          -- 멤버 ROUTE_PATH_GUID 목록

    created_at      timestamptz NOT NULL DEFAULT now(),
    UNIQUE (source_file, group_id)
);

CREATE INDEX IF NOT EXISTS ix_rbg_src ON route_bundle_group (source_file);
CREATE INDEX IF NOT EXISTS ix_rbg_key ON route_bundle_group (source_file, owner_name, utility);

-- =============================================================================
-- route_bundle_template — (source_file, owner_name, utility) 키별 '대표 번들 패턴'.
--   신규 배관설계 시 조회용. 한 키에 번들이 여러 개면 트렁크 고도를 모두 모으고(공용 랙 후보),
--   pitch 는 중앙값, 형태코드는 최빈을 대표로 쓴다. 유틸리티 단위 폴백은 뷰어가 집계한다.
-- =============================================================================
CREATE OR REPLACE VIEW route_bundle_template AS
SELECT
    source_file,
    owner_name,
    utility,
    array_agg(DISTINCT round(trunk_z::numeric, 0)::double precision)       AS trunk_zs,
    percentile_cont(0.5) WITHIN GROUP (ORDER BY pitch_mm)                  AS pitch_mm,
    percentile_cont(0.5) WITHIN GROUP (ORDER BY trunk_xy_spread)           AS trunk_xy_spread,
    sum(n_members)::int                                                    AS n_members,
    mode() WITHIN GROUP (ORDER BY arrow_code)                             AS arrow_code,
    count(*)::int                                                          AS n_groups
FROM route_bundle_group
GROUP BY source_file, owner_name, utility;

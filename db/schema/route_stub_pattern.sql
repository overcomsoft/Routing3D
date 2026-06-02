-- =============================================================================
-- route_stub_pattern.sql — 기존설계 스텁 패턴 벡터 저장소 (pgvector)
-- =============================================================================
-- [목적]
--   사람이 설계한 기존배관(TB_ROUTE_PATH)의 '양 끝 스텁'(장비 출발 / 덕트·레터럴 진입)
--   형상을 정규화해 1행=1스텁 표본으로 적재한다. 자동라우팅은 (anchor_kind, utility_group,
--   utility) 범주로 pre-filter 한 뒤 feat/dir_unit 벡터로 유사 스텁을 검색해 회랑·waypoint 로 활용한다.
--
-- [적용]  (프로젝트 루트, .venv 에 psycopg2 존재)
--   .\.venv\Scripts\python.exe -m routing3d_py.pattern_db --apply-schema
--   또는 psql:  psql "host=localhost dbname=AUTOROUTINGV7 user=postgres" -f db/schema/route_stub_pattern.sql
--
-- [전제]  pgvector / cube / postgis 설치됨(확인 완료, 2026-06-01).
-- [단위]  좌표·치수 모두 mm. 라우트=BIM 동일 월드 프레임.
-- =============================================================================

CREATE EXTENSION IF NOT EXISTS vector;

-- 1행 = 기존배관 한 끝에서 추출한 스텁 표본 1개.
CREATE TABLE IF NOT EXISTS route_stub_pattern (
    id              bigserial PRIMARY KEY,

    -- ── 출처(provenance) ──
    source_file     text        NOT NULL,           -- 프로젝트(SOURCE_FILE)
    route_path_guid text,                            -- 원본 경로 GUID
    anchor_kind     text        NOT NULL,            -- 'EQUIP'(출발 스텁) | 'DUCT'(종단 스텁)
    anchor_name     text,                            -- 앵커(장비/덕트) 이름

    -- ── 범주 키(추론 시 pre-filter — 유틸 위반 금지) ──
    utility_group   text,
    utility         text,

    -- ── 원자료(역변환·검수용, mm) ──
    poc_pos         double precision[3] NOT NULL,    -- 스텁의 PoC 끝점(월드)
    anchor_min      double precision[3],             -- 앵커 AABB 최소
    anchor_max      double precision[3],             -- 앵커 AABB 최대

    -- ── 정규화 스텁 형상(로컬 프레임) ──
    face            text,                            -- 진출/진입 면: '+x','-x','+y','-y','+z','-z'
    dir_seq         text,                            -- 정준 방향 시퀀스(예 '+z,+x'). 꺾임 수 = 토큰-1
    n_bends         int,
    rise_mm         double precision,                -- 면 법선 방향 첫 이동 거리
    offset_mm       double precision,                -- 면 바깥 진입/이탈 여유
    diameter_mm     double precision,                -- 배관 외경(SOURCE_SIZE 파싱)

    -- ── 벡터(pgvector) ──
    dir_unit        vector(3),                       -- 시작→종단 진행 단위벡터(코사인 검색)
    feat            vector(24),                      -- 정규화 기하 특징벡터(그룹내 L2 검색)
                                                     --   [face 1hot 6][1차방향 1hot 6][2차방향 1hot 6]
                                                     --   [앵커내 PoC 상대좌표 3][시작→종단 단위 3]
    created_at      timestamptz NOT NULL DEFAULT now()
);

-- 범주 pre-filter(가장 무거운 일은 여전히 범주 키가 한다).
CREATE INDEX IF NOT EXISTS ix_rsp_key
    ON route_stub_pattern (anchor_kind, utility_group, utility);
CREATE INDEX IF NOT EXISTS ix_rsp_src
    ON route_stub_pattern (source_file);

-- 벡터 ANN(그룹 내 유사 스텁 검색). 표본이 적으면 brute force 도 충분하나, 인덱스는 무해.
CREATE INDEX IF NOT EXISTS ix_rsp_feat_hnsw
    ON route_stub_pattern USING hnsw (feat vector_l2_ops);
CREATE INDEX IF NOT EXISTS ix_rsp_dir_hnsw
    ON route_stub_pattern USING hnsw (dir_unit vector_cosine_ops);

-- =============================================================================
-- 집계 템플릿 — (anchor_kind, utility_group, utility) 키별 '대표 스텁'.
--   방향성 데이터라 평균이 아니라 최빈(face/dir_seq) + 중앙값(rise/offset)을 쓴다.
--   추론의 폴백 1순위(ANN 미스 시) 및 검수 리포트의 요약으로 사용.
-- =============================================================================
CREATE OR REPLACE VIEW route_stub_template AS
SELECT
    anchor_kind,
    utility_group,
    utility,
    mode() WITHIN GROUP (ORDER BY face)     AS face,
    mode() WITHIN GROUP (ORDER BY dir_seq)  AS dir_seq,
    percentile_cont(0.5) WITHIN GROUP (ORDER BY rise_mm)     AS rise_mm,
    percentile_cont(0.5) WITHIN GROUP (ORDER BY offset_mm)   AS offset_mm,
    percentile_cont(0.5) WITHIN GROUP (ORDER BY diameter_mm) AS diameter_mm,
    avg(n_bends)::double precision           AS avg_bends,
    count(*)                                 AS n
FROM route_stub_pattern
GROUP BY anchor_kind, utility_group, utility;

# AUTOROUTINGV7 → DDW_AI_DB 전환 — 스키마·필드 차이 분석

> 작성: 2026-06-06 · 단위 mm · 대상: 본 프로젝트(Routing3D)가 사용하는 테이블/필드 한정
> 조사 방법: 코드 내 SQL 참조 수집 + 양 DB `information_schema` 실측(localhost/5432)

---

## 0. 핵심 결론 (먼저 읽기)

1. **`SOURCE_FILE`(프로젝트 키)가 완전히 사라졌다.** AUTOROUTINGV7은 모든 BIM 테이블을 `SOURCE_FILE`(=BIM 파일=툴 1개)로 스코프했지만, DDW_AI_DB에는 그 컬럼이 없다. → 프로젝트 분리 기준을 **`TB_SPACE_GROUP_INFO`(65행=툴 단위)** 로 재설계해야 한다. 이게 `space_project_map`의 대체다.
2. **좌표 표기가 `MIN_*/MAX_*` → `AABB_MIN*/AABB_MAX*`(+ OBB 8코너)** 로 바뀌었다. 단순 rename 이지만 전 테이블에 영향. OBB(회전 박스)가 추가로 제공된다(현재는 AABB만 써도 됨).
3. **장비 PoC가 `POC_LIST`(jsonb) → 병렬 텍스트 배열(`POC_ID_LIST`/`POC_POSITIONS_LIST`/…) + 전용 테이블 `TB_POCINSTANCES`(11.4만행)** 로 분해됐다. 파싱 로직 전면 재작성 필요. 대신 PoC 정보가 훨씬 풍부(관경·재질·연결 PoC).
4. **장애물 통과여부가 휴리스틱 → `COLLISION_PASS`(0/1) 컬럼으로 명시**됐다(우리 `IsPassThrough` 추정 로직을 DB 값으로 대체 가능).
5. **DDW_AI_DB는 이미 'AI 설계 학습' 인프라를 1급 테이블로 보유**한다 — `TB_ROUTE_FEATURE_VECTOR`(7,276)·`TB_ROUTE_DESIGN_GROUP`(168)·`TB_ROUTE_SEGMENT_TEMPLATE`(20,021)·`TB_ROUTE_NODES/EDGES`·`TB_ROUTE_AUTO_DESIGN`. **우리가 pgvector로 만든 `route_stub_pattern`·`route_bundle_group`은 이걸로 대체**될 수 있다(중복 학습 폐기 후보).

---

## 1. 연결·스코프 차이

| 항목 | AUTOROUTINGV7 | DDW_AI_DB |
|---|---|---|
| DB 이름 | `AUTOROUTINGV7` | `DDW_AI_DB` |
| 프로젝트 목록 | **`space_project_map`** (project_id, source_file, process, equipment_code) | **없음 → `TB_SPACE_GROUP_INFO`** (TAG_GROUP_NM·BAY_GROUP_NM·PROCESS_GROUP_NM + AABB + EQUIPMENT_TAG_LIST·DUCT_ID_LIST·LATERAL_PIPE_ID_LIST) |
| 객체↔프로젝트 연결 | 모든 테이블의 **`SOURCE_FILE`** 컬럼 | **`SOURCE_FILE` 없음.** 그룹(TAG_GROUP)의 AABB·ID 리스트 또는 **공간 bbox 교차**로 객체를 묶어야 함 |
| 스코프 단위 | BIM 파일 1개 = 툴 1개 | `TB_SPACE_GROUP_INFO` 1행 = 툴 1개 (예: `WTNHJ04 / BAY004 / CLEAN`) |

**영향이 가장 큰 지점.** 장애물(`TB_BIM_OBSTACLE`)에는 그룹/프로젝트 키가 없고 `GROUP_NAME_LIST·LEVEL·BAY`만 있다. 따라서 "한 프로젝트의 장애물"을 얻으려면 (a) 선택 그룹의 장비 AABB로 공간 bbox를 구하고 (b) 그 bbox와 교차하는 장애물을 공간 필터링하는 방식으로 바꿔야 한다(현재 `LoadExistingPipes`가 이미 bbox 필터를 쓰는 패턴과 동일).

---

## 2. 테이블 매핑 (old → new)

| 용도 | AUTOROUTINGV7 | DDW_AI_DB | 비고 |
|---|---|---|---|
| 프로젝트 목록 | `space_project_map` | **`TB_SPACE_GROUP_INFO`** | 65 그룹 |
| 장애물 | `TB_BIM_OBSTACLES` | **`TB_BIM_OBSTACLE`** (단수) | 303,776행 · COLLISION_PASS 추가 |
| 메인 장비+PoC | `TB_BIM_EQUIPMENT` | **`TB_EQUIPMENTS`** | 476행 · IS_MAIN→MAIN_SUB_TYPE |
| PoC(개별) | (POC_LIST jsonb 내부) | **`TB_POCINSTANCES`** (신규 전용 테이블) | 113,804행 |
| 종단(덕트/레터럴) | `TB_DUCT_LATERAL` (CATEGORY로 구분) | **`TB_LATERAL_PIPE`(665) + `TB_DUCT`(214)** | 2개로 분리 |
| 공간 영역 | `TB_BIM_SPACE_INFO` | **`TB_SPACE_INFO`(6) + `TB_SPACE_GROUP_INFO`(65)** | LEVEL_NAME→SPACE_NAME/LEVEL |
| 기존 설계배관 | `TB_ROUTE_PATH` | `TB_ROUTE_PATH` (컬럼 변경) | 7,052행 |
| 배관 세그먼트 | `TB_ROUTE_SEGMENTS` / `_SEGMENT_DETAIL` | 동일(컬럼 확장) | 폴리라인 빌더 거의 호환 |
| **(우리 생성)** 스텁 학습 | `route_stub_pattern`/`_template` | **`TB_ROUTE_FEATURE_VECTOR`·`TB_ROUTE_SEGMENT_TEMPLATE`** | DB가 이미 보유 |
| **(우리 생성)** 번들 그룹 | `route_bundle_group`/`_template` | **`TB_ROUTE_DESIGN_GROUP`** | DB가 이미 보유 |
| (신규) 배관 그래프 | — | `TB_ROUTE_NODES`(피팅) / `TB_ROUTE_EDGES`(스풀) | 배관자재(피팅) 정보 |
| (신규) 자동설계 결과 | — | `TB_ROUTE_AUTO_DESIGN` + `VW_*` 뷰 | 결과 저장소 |

---

## 3. 공통 좌표 패턴 변화 (전 테이블)

| AUTOROUTINGV7 | DDW_AI_DB |
|---|---|
| `MIN_X, MIN_Y, MIN_Z` | `AABB_MINX, AABB_MINY, AABB_MINZ` |
| `MAX_X, MAX_Y, MAX_Z` | `AABB_MAXX, AABB_MAXY, AABB_MAXZ` |
| (없음) | `POS_X/Y/Z`(원점) + `ANGLE_X/Y/Z`·`RADIAN`(회전) |
| (없음) | `OBB_*` 8코너 24컬럼(정밀 회전 박스 — 향후 FCL 정밀충돌에 활용 가능) |

→ 우리 엔진은 AABB만 쓰므로 **`MIN_X→AABB_MINX` 식 rename 매핑**이면 충분. OBB는 선택적 고도화.

---

## 4. 테이블별 필드 매핑 (사용 컬럼 한정)

### 4.1 장애물 — `TB_BIM_OBSTACLES` → `TB_BIM_OBSTACLE`
| 용도 | old | new |
|---|---|---|
| 프로젝트 키 | `SOURCE_FILE` | **없음**(공간 bbox/그룹으로 대체) |
| 박스 | `MIN_*`/`MAX_*` | `AABB_MIN*`/`AABB_MAX*` |
| 이름 | `NAME` | `INSTANCE_NAME` |
| 타입 | `OST_TYPE`,`DDWORKS_TYPE` | `OST_TYPE`,`DDWORKS_TYPE`,`OBS_TYPE`,`INSTANCE_TYPE` |
| **통과여부** | (휴리스틱 추정) | **`COLLISION_PASS`(0/1)** ← 직접 사용 |
| 식별자 | `OBJECT_ID` | `INSTANCE_ID` |

- `DDWORKS_TYPE` 상위: COLUMN_ARCHITECTURE(14만)·FLOOR_ARCHITECTURE(13.8만)·BEAM_*·P_PIPE·P_STANDARD_DUCT.
- `COLLISION_PASS=1`(13.8만) ≈ FLOOR_ARCHITECTURE → **바닥이 통과객체**임이 DB에 명시(우리 휴리스틱과 일치, 이제 컬럼으로 대체).

### 4.2 장비 — `TB_BIM_EQUIPMENT` → `TB_EQUIPMENTS`
| 용도 | old | new |
|---|---|---|
| 메인구분 | `IS_MAIN`(bool) | **`MAIN_SUB_TYPE`** ('MainTool' 55 / 'SubTool' 411 / '' 10) |
| 이름 | `NAME` | `INSTANCE_NAME` (+ 별도 태그는 ROUTE_PATH.EQUIPMENT_TAG) |
| 박스 | `MIN_*`/`MAX_*` | `AABB_*` |
| **PoC** | **`POC_LIST`(jsonb)** = `{id,name,pocPosition,utility,utilityGroup,isConnected,endPocs[…]}` | **병렬 텍스트 배열**: `POC_ID_LIST`·`POC_POSITIONS_LIST`·`POC_SIZES_LIST`·`POC_COUNT` + **종단**: `POC_TARGET_OWNER_ID_LIST`·`POC_TARGET_OWNER_TYPE_LIST` |
| PROCESS | (space_project_map) | `PROCESS_NAME`,`LEVEL`,`BAY` |

- `POC_POSITIONS_LIST` = `[[x,y,z],[x,y,z],…]` JSON 문자열, `POC_ID_LIST` = `["guid",…]` 와 **인덱스 정렬**.
- 종단 타입(`POC_TARGET_OWNER_TYPE_LIST`) 예: `NOZZLE`/`LATERAL PIPE`/`DUCT`/`PIPE`/`ENDCAP`. → **작업(start→end) 생성 = 장비 PoC → POC_TARGET_OWNER(LATERAL PIPE/DUCT)**. `connectedOnly`는 종단 타입이 실연결(LATERAL/DUCT/PIPE)인 PoC만 채택으로 재해석.

### 4.3 PoC 전용 테이블 — `TB_POCINSTANCES` (신규)
`INSTANCE_ID, OWNER_INSTANCE_ID, OWNER_INSTANCE_TYPE, UTILITY_GROUP_NM, UTILITY_NM, MATERIAL_NM, PIPESTD_NM, PIPESIZE_NM, POSX/Y/Z, DIAMETER, CONNECTED_POC_ID, CONNECTION_ORDER, FLOWDIRECTION, …`
- **장비/레터럴/덕트의 PoC를 한 테이블로 통합** + `CONNECTED_POC_ID`로 양끝 연결을 직접 표현 → 작업 생성을 jsonb 파싱 없이 **조인**으로 가능(더 견고). 관경(`DIAMETER`)·재질도 PoC 단위로 제공.

### 4.4 종단 객체 — `TB_DUCT_LATERAL` → `TB_LATERAL_PIPE` + `TB_DUCT`
| old | new |
|---|---|
| 단일 테이블 + `CATEGORY`(LATERAL/DUCT) | **두 테이블로 분리** |
| `NAME`,`UTILITY`,`MIN_*`/`MAX_*` | `INSTANCE_NAME`,`UTILITY_GROUP`,`UTILITY`,`AABB_*` + `POC_*`/`TAKEOFF_POC_*` 배열 + `CONNECTED_OWNER/POC_GUID_LIST` |

### 4.5 공간 — `TB_BIM_SPACE_INFO` → `TB_SPACE_INFO`
| old | new |
|---|---|
| `LEVEL_NAME`,`MIN_*`/`MAX_*` | `SPACE_NAME`,`SPACE_TYPE`,`PARENT_SPACE_ID`,`LEVEL`,`BAY`,`AABB_*` |

### 4.6 기존 설계배관 — `TB_ROUTE_PATH` (양 DB 존재, 컬럼 변경)
| 용도 | old | new |
|---|---|---|
| 소유 장비 | `SOURCE_OWNER_NAME` + `SOURCE_OWNER_POS*` | **`EQUIPMENT_NAME`,`EQUIPMENT_TAG`,`EQUIPMENT_POS*`** |
| 출발 | `SOURCE_UTILITY,SOURCE_SIZE,SOURCE_POS*` | 동일 + **`SOURCE_GUID`** |
| 종단 | `TARGET_OWNER_NAME,TARGET_POS*` | 동일 + **`TARGET_GUID`** |
| 지표 | `PR_BRANCH_COUNT,PR_BEND_COUNT,PR_PATH_EFFICIENCY,PR_TOTAL_LENGTH` | **`BRANCH_COUNT,BEND_COUNT,TOTAL_LENGTH`**(efficiency 없음) + `BRANCH_GUID_LIST,BRANCH_TYPE_LIST` |
| 프로젝트 조인 | `eq.SOURCE_FILE` | **EQUIPMENT_NAME/TAG ↔ TB_EQUIPMENTS.INSTANCE_NAME** (source_file 조인 폐지) |

- `TB_ROUTE_SEGMENTS`/`TB_ROUTE_SEGMENT_DETAIL`: `ROUTE_PATH_GUID`·`ORDER`·`FROM_POS*`/`TO_POS*` 구조 **그대로** → 폴리라인 빌더 거의 무수정. SEGMENT_DETAIL에 `TYPE,SIZE,INSTANCE_ID,CONNECTED_*_LIST,AABB/OBB` 추가(자재/피팅 식별 가능).

---

## 5. 신규 AI 자산 (우리 pgvector 학습 대체 후보)

DDW_AI_DB는 우리가 직접 만든 학습 저장소를 **공식 1급 테이블**로 이미 보유한다:

| DDW_AI_DB 테이블 | 행수 | 내용 | 우리 대응물 |
|---|---|---|---|
| `TB_ROUTE_FEATURE_VECTOR` | 7,276 | 경로별 특징벡터(DIRECTION_PATTERN·TOTAL_LENGTH_MM·STEP_COUNT·`FEATURE_VECTOR` pgvector·RAW_FEATURES_JSON) | `route_stub_pattern`(특징) |
| `TB_ROUTE_CONTEXT_VECTOR` | — | 컨텍스트 벡터(start/end meta) | L3b ANN 컨텍스트 |
| `TB_ROUTE_DESIGN_GROUP` | 168 | 설계 그룹(클러스터): MEMBER_ROUTE_GUIDS·REPRESENTATIVE_VECTOR·A/B/C_LEN_MEAN/STD·START/END_BBOX_JSON·SUB_CLUSTER_ID | **`route_bundle_group`**(번들) |
| `TB_ROUTE_SEGMENT_TEMPLATE` | 20,021 | 세그먼트 템플릿: SEGMENT_ROLE(start/end/mid)·LOCAL_POINTS_JSON·MATERIAL_SEQ_JSON·FEATURE_VECTOR | **스텁 템플릿**(L1′) |
| `TB_ROUTE_NODES` | — | 배관 그래프 **노드=피팅**(TYPE·POS·BENDING_RADIUS·BENDING_ANGLE·SIZE·MATERIAL) | (배관자재 큐브의 정식 소스!) |
| `TB_ROUTE_EDGES` | — | 배관 그래프 **엣지=스풀**(PIPETYPE·POS·SIZE·MATERIAL) | — |
| `TB_ROUTE_AUTO_DESIGN` | — | 자동설계 결과 저장(INPUT/ASSIGNMENTS/ROUTES_JSON·COLLISION_COUNT·LENGTH_DEVIATION_PCT·STATUS·리뷰) | (우리 결과 저장소) |
| `VW_ROUTE_SEARCH`,`VW_AUTO_DESIGN_SUMMARY` | — | 검색/요약 뷰 | — |

**시사점**: 전환 시 `db/schema/route_stub_pattern.sql`·`route_bundle_group.sql`과 `pattern_learn.py`·`bundle_detect.py`의 **학습/적재 단계를 폐기/대체**하고, 위 공식 테이블을 **읽어서** 패턴/그룹/스텁을 활용하도록 바꾸는 게 정석. (직전 요청한 "배관자재 cube"도 `TB_ROUTE_NODES`(피팅 노드)를 직접 쓰면 정밀해짐 — 현재는 폴리라인 정점 추정.)

---

## 6. 코드 영향 범위

| 파일 | 변경 강도 | 사유 |
|---|---|---|
| `csharp/.../Model/ObstacleDbLoader.cs` | **전면 재작성** | 테이블/컬럼/스코프 전부 변경(source_file→그룹·bbox, MIN/MAX→AABB, POC_LIST jsonb→배열/조인, duct/lateral 분리) |
| `csharp/.../Model/BundleStore.cs` | **대체** | `route_bundle_*` → `TB_ROUTE_DESIGN_GROUP` 조회로 |
| `csharp/.../Model/PatternStore.cs` | **대체** | `route_stub_*` → `TB_ROUTE_FEATURE_VECTOR`/`SEGMENT_TEMPLATE` |
| `csharp/.../Diagnostics/DbRouteDiag.cs` | 중간 | 로더/스토어 의존부 따라 수정 |
| `csharp/.../MainWindow.xaml(.cs)` | 소 | 프로젝트 콤보 소스: `space_project_map`→`TB_SPACE_GROUP_INFO` |
| `python_experiments/routing3d_py/obstacle_db.py` | 전면 | 동일 |
| `python_experiments/routing3d_py/scene.py` | 전면 | `list_projects`/`load_scene` 재작성(그룹 기준) |
| `python_experiments/routing3d_py/route_db.py` | 중간 | `TB_ROUTE_PATH` 컬럼 rename(SOURCE_OWNER_*→EQUIPMENT_*) |
| `pattern_learn.py`·`pattern_db.py`·`bundle_detect.py` | **폐기/대체 검토** | 학습을 공식 테이블로 대체 |
| `db/schema/*.sql` | 폐기 검토 | 공식 테이블 존재 |

---

## 7. 결정이 필요한 사항 (전환 설계 전 합의)

1. **프로젝트 단위**: `TB_SPACE_GROUP_INFO`의 (TAG_GROUP_NM=장비태그) 단위가 맞는지? 콤보 표시는 `TAG_GROUP_NM / BAY / PROCESS`.
2. **장애물 스코프**: 그룹 AABB와 **공간 교차**로 필터(권장) vs 다른 키? 장애물에 그룹 키가 없어 공간 필터가 유력.
3. **작업(start→end) 생성**: `TB_EQUIPMENTS` POC 배열 파싱 vs **`TB_POCINSTANCES` 조인(권장)** 중 무엇으로? 후자가 견고(관경·연결·재질 포함).
4. **학습 자산**: 우리 pgvector(`route_stub_pattern`/`route_bundle_group`)를 버리고 **DDW_AI_DB 공식 테이블**(`TB_ROUTE_DESIGN_GROUP`/`FEATURE_VECTOR`/`SEGMENT_TEMPLATE`)을 소비할지?
5. **통과객체**: `COLLISION_PASS` 컬럼을 그대로 신뢰할지(현행 휴리스틱 대체).
6. **병행 기간**: AUTOROUTINGV7 로더를 남겨두고 런타임 토글할지, 완전 교체할지.

---

## 8. 데이터 적재 현황 (DDW_AI_DB 실측)

```
obstacle      303,776   equip          476 (Main 55 / Sub 411)
poc           113,804   lateral        665   duct        214
route_path      7,052   feature_vec  7,276   seg_template 20,021
design_group      168   space            6   space_group   65
```

전 카테고리 데이터가 풍부하게 적재돼 있어, 로더만 재작성하면 즉시 검증 가능하다.

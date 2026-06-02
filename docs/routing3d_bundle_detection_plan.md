# 그룹(Bundle) 배관 탐지 — 개발계획

> 기하학적 유사도 분석을 통한 지능형 그룹 배관 탐지
> 단위 mm · 입력 = PostgreSQL(AUTOROUTINGV7) 기존 설계배관(TB_ROUTE_PATH) · 작성 2026-06-02

---

## 1. 목표·정의

사람이 설계한 **기존배관 경로**를 **장비명(SOURCE_OWNER_NAME)·유틸리티(SOURCE_UTILITY)별**로 묶어,
경로 **형태 패턴의 유사도**를 분석해 함께 다발(Bundle)로 깔린 배관 그룹을 자동 추출한다.

**그룹(번들) 배관의 정의** (도메인 규칙, 필수 게이트):

> 여러 배관이 **동일 이격간격(equal pitch)** 으로 나란히 가면서, 각 배관이 **2번 이상의 수직·수평 꺾임(orthogonal bend)** 을 공유하는 묶음.

즉 두 조건을 모두 만족해야 번들로 인정한다.

1. **꺾임 ≥ 2**: 대표 경로가 수직↔수평 축 전환을 2회 이상 한다(단순 직선·1엘보 제외).
2. **동일 이격간격**: 묶인 배관들이 트렁크 구간에서 거의 평행하고, 인접 배관 사이 간격(pitch)이 일정하다(파이프 랙 다발의 특징).

활용: 추출된 번들은 자동라우팅에서 **공용 트렁크(주경로)** 로 묶어 라우팅(랙 번들 L3a 의 상위 개념)하거나,
기존설계 추종(L2b 회랑)의 **다발 단위 가이드**로 쓴다. 본 계획은 **탐지(추출)까지**를 범위로 한다.

---

## 2. 입력·전제

- **입력**: `route_db.load_existing_pipes(source_file)` → `list[ExistingPipe]`
  (폴리라인 `points`, `source_pos/target_pos`, `utility`, `group`, `owner_name`, `diameter_mm`).
  - 본 계획에서 `ExistingPipe.owner_name`(=SOURCE_OWNER_NAME) 필드를 **신규 추가**(완료) — 장비명 그룹핑 키.
- **장비/덕트 AABB**: 트렁크 z-레벨 검증 보조(선택).
- **좌표**: 월드 mm, BIM 동일 프레임. 직교(맨해튼) 형상 가정.
- **전제**: 폴리라인은 순서대로 정렬돼 있고 중복점은 제거됨(로더가 보장).

---

## 3. 알고리즘 — 3단계 파이프라인

인포그래픽의 Phase 1~3 을 본 데이터(이미 폴리라인이 있는 기존배관)에 맞춰 구체화한다.
※ 인포그래픽의 'BFS + 부모 포인터'는 *우리 라우팅 엔진의* 경로 생성용이고, **기존배관은 이미 폴리라인이
존재**하므로 Phase 1 은 'BFS 탐색'이 아니라 **형태 특징 추출**로 대체한다.

### Phase 1 — 개별 경로 특징 추출 (`PipeFeature`)

각 배관 폴리라인에서:

| 특징 | 정의 | 비고 |
|---|---|---|
| `dir_runs` | 세그먼트를 6직교 축으로 스냅 → 연속 동일방향 병합한 [(축,길이)] | 스텁 추출과 동일 규약 |
| `arrow_code` | 각 런을 **R(수직 z) · H(수평 xy) · D(경사)** 로 부호화한 문자열 (예 `RHRH`) | Levenshtein 입력 |
| `n_ortho_bends` | 인접 런의 **축 전환 횟수**(수직↔수평/수평↔수평 축변경) | 번들 게이트(≥2) |
| `seg_units` | 길이 N 으로 리샘플한 단위 방향벡터 열 | 방향 유사도 입력 |
| `total_len` | 폴리라인 누적 길이(mm) | 길이 유사도 |
| `extent` | bbox (dx, dy, dz) | 물리적 규모 유사도 |
| `centroid` · `trunk_axis` | 중심점 + 주 수평축(최장 수평런 방향) | 이격간격·트렁크 계산 |

**Arrow Coding(R/H/D)**: 런 방향벡터에서 |dz| 우세 → `R`(수직), xy 우세 → `H`(수평),
어느 축도 tol 이상 우세하지 않으면 → `D`(경사). 연속 동일 코드는 압축.

### Phase 2 — 복합 유사도 계산 (4대 지표 가중합)

두 배관 a,b 의 유사도 `sim(a,b) ∈ [0,1]`:

| 지표 | 가중 | 계산 |
|---|---|---|
| **형태 일치도** | 30% | `1 − Levenshtein(arrow_a, arrow_b) / max(len)` |
| **방향성 일치도** | 30% | 리샘플 단위벡터 열의 평균 코사인 유사도(배관 양방향 중 큰 값) |
| **길이 일치도** | 20% | `1 − |Lₐ−L_b| / max(Lₐ,L_b)` |
| **물리적 규모 일치도** | 20% | 축별 `min(eₐ,e_b)/max(eₐ,e_b)` 평균 (X·Y·Z extent) |

`sim = 0.3·shape + 0.3·dir + 0.2·len + 0.2·scale`.
단순 거리 비교가 아니라 형태·방향·규모·길이를 함께 보아 **평행 다발**을 잡아낸다.

### Phase 3 — 그룹화 및 트렁크 구간 탐지

1. **Pre-filter**: `(owner_name, utility)` 키로 묶어 **같은 키 안에서만** 쌍 비교(유틸 위반 금지, 비교량 축소).
2. **Union-Find 클러스터링**: 키 내 모든 쌍 중 `sim ≥ THRESHOLD(기본 0.70)` 인 쌍을 union → 후보 그룹.
3. **번들 게이트(도메인 규칙 검증)** — 후보 그룹이 다음을 모두 만족해야 번들로 채택:
   - 멤버 수 ≥ 2,
   - 대표 `n_ortho_bends ≥ 2`(멤버 중앙값),
   - **동일 이격간격**: 멤버 중심선을 트렁크 주축의 수직평면에 투영 → offset 정렬 → 인접 pitch 의
     변동계수(CV=std/mean) ≤ `PITCH_CV_MAX(기본 0.30)`.
4. **트렁크(주경로) 탐지**: 멤버들의 **수평 런 z-높이** 최빈값 = `trunk_z`(공용 랙 고도).
   트렁크 구간 멤버 중심선 간 최대 수평 벌어짐 = `trunk_xy_spread`(다발 폭). 양 끝 = Fan-in/out.
5. **출력**: 그룹별 리포트(아래 §4).

```
load_existing_pipes ─▶ Phase1 PipeFeature[]
                         │  (owner_name, utility) pre-filter
                         ▼
              Phase2 pairwise sim ≥ 0.70 ─▶ Phase3 Union-Find
                         │
                 번들 게이트(≥2 꺾임 + 동일 pitch)
                         ▼
              BundleGroup[]  (group_id, avg_sim, trunk_z, trunk_xy_spread, pitch, n_bends, members)
```

---

## 4. 최종 데이터 리포트 구조

| 항목 | 설명 |
|---|---|
| `group_id` | 탐지된 번들 그룹 고유 번호 |
| `owner_name` · `utility` | 그룹 키(장비명·유틸리티) |
| `n_members` | 그룹 내 배관 수 |
| `avg_similarity` | 그룹 내 경로쌍 평균 유사도(1.0 에 가까울수록 강결합) |
| `trunk_z` | 주경로가 형성된 공용 고도(mm) |
| `trunk_xy_spread` | 주경로 내 배관들 간 최대 수평 벌어짐(다발 폭, mm) |
| `pitch_mm` | 인접 배관 간 대표 이격간격(중앙값, mm) — 동일 이격 확인 |
| `n_ortho_bends` | 대표 수직·수평 꺾임 수(≥2) |
| `member_guids` | 멤버 ROUTE_PATH_GUID 목록 |

---

## 5. 산출물·파일

| 파일 | 내용 | 상태 |
|---|---|---|
| `routing3d_py/route_db.py` | `ExistingPipe.owner_name` 필드 + SELECT 추가 | **수정(완료)** |
| `routing3d_py/bundle_detect.py` | **신규** — Phase1~3 전체 + CLI(`--report`/`--write-db`) | **신규** |
| `db/schema/route_bundle_group.sql` | **신규** — `route_bundle_group` 결과 저장 테이블 | **신규** |
| `tests/test_bundle_detect.py` | **신규** — 합성 평행 다발/비다발 단위 테스트 | **신규** |

### CLI

```powershell
# 탐지 + 콘솔 리포트(DB 미적재)
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --report

# 임계/피치 조정
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --threshold 0.75 --pitch-cv 0.25 --report

# 결과 저장(route_bundle_group, 스키마 자동 적용)
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --write-db
```

---

## 6. 파라미터(초기값)

| 상수 | 기본값 | 의미 |
|---|---|---|
| `SIM_THRESHOLD` | 0.70 | Union-Find union 임계(인포그래픽 70%) |
| `W_SHAPE / W_DIR / W_LEN / W_SCALE` | 0.30 / 0.30 / 0.20 / 0.20 | 4대 지표 가중 |
| `MIN_BENDS` | 2 | 번들 게이트 최소 수직·수평 꺾임 |
| `PITCH_CV_MAX` | 0.30 | 동일 이격간격 허용 변동계수 |
| `RESAMPLE_N` | 24 | 방향 유사도 리샘플 세그먼트 수 |
| `DIAG_TOL` | 0.34 | R/H 판정 tol(이하 우세 없으면 D=경사) |
| `Z_BIN_MM` | 100 | 트렁크 z 최빈 버킷 |

---

## 7. 검증 계획

1. **단위 테스트**(DB 불필요, 합성 폴리라인):
   - 평행 3배관(동일 pitch, 2엘보) → 1 번들, `n_members=3`, pitch CV 작음.
   - 직선/1엘보 배관 → 번들 아님(꺾임 게이트 탈락).
   - 불규칙 간격 평행 배관 → 번들 아님(pitch CV 탈락).
   - arrow_code·Levenshtein·코사인·복합유사도 각 함수 경계값.
2. **실데이터**(project6): `--report` 로 그룹 수·avg_similarity·trunk_z·pitch 출력, 육안 검수.
3. **회귀 무해성**: 신규 모듈·신규 테이블만 추가 — 기존 엔진/라우팅/스텁 학습 불변(pytest 기존 통과 유지).

---

## 8. 후속(범위 밖, 다음 단계 후보)

- 탐지된 번들을 자동라우팅 트렁크(공용 회랑/랙)로 주입 → 다발 단위 라우팅(L3a 확장).
- C# 뷰어에서 번들 그룹 하이라이트 레이어(색=group_id, 트렁크 z 평면 표시).
- pgvector 번들 형태 임베딩으로 프로젝트 간 유사 번들 검색.

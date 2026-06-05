# 그룹배관(번들) 생성 알고리즘 — 설계·구현 상세

> 모듈: `python_experiments/routing3d_py/bundle_detect.py` · 단위 mm · 직교(맨해튼) 형상 가정.
> 사람이 설계한 기존배관(TB_ROUTE_PATH)에서 '평행 다발(번들)'을 자동 탐지해, 신규 자동라우팅이
> 기존설계처럼 공용 트렁크 랙에 다발로 모이도록 학습 데이터(route_bundle_group/template)를 만든다.

---

## 1. 개요

### 1.1 그룹배관(번들)이란

플랜트에서 같은 유틸리티 배관들은 **공용 파이프 랙을 따라 나란히** 깔린다. 수평 랙(트렁크 고도)뿐
아니라 **수직 입상(riser) 다발**로도 묶인다. 이렇게 **동일 이격간격으로 평행하게 함께 진행하는 배관
다발**을 '번들(bundle)'이라 부른다. 자동 라우팅이 이를 따르면 결과가 사람 설계처럼 정돈된다(L4 학습).

### 1.2 핵심 정의 (v3)

> **번들 = 같은 (장비 owner · 유틸리티) 의 배관 중, 동일 이격간격으로 평행하게 '함께 진행'하는
> ≥2개 배관의 묶음.** 진행축은 수평(x·y)·수직(z, 입상) 모두 포함한다.

세 가지 기준:

1. **레인 유지 (±10mm)** — 각 배관은 진행하는 동안 자기 '레인'(진행축과 직교하는 두 고정 좌표)을
   **±10mm 안에서 일정하게** 유지한다. 이 밴드를 벗어나면 레인 이탈(=다른 런/꺾임)로 본다.
2. **동시진행 + 동일간격** — 같은 진행축에서 진행구간이 겹치고(co-travel), 직교 한 축은 공유(±10mm),
   다른 축으로 **등간격** 떨어진 평행 런들을 한 다발로 묶는다.
3. **밴딩 각도 동일 (±2°)** — 꺾임이 있으면 **같은 각도로 함께 꺾이는** 배관은 코너를 넘어 한 그룹으로
   이어 포함한다. 단, 꺾임 자체는 **필수가 아니다** — 직선 평행 다발도 번들이다.

> v1(형태 유사 + 균일 피치) → v2(공간: 수평 트렁크·z-근접) → **v3(세그먼트 동시진행: 수평·수직 + 레인
> 유지 + 코너 추종)** 로 재설계했다. 배경은 §6.

### 1.3 입력·출력

```
입력:  TB_ROUTE_PATH (route_db.load_existing_pipes)
        = 기존 설계배관 폴리라인 list[ExistingPipe]
        (route_path_guid, owner_name, utility, points[월드 mm], source/target_pos)

처리:  Phase 1 특징 추출(참고) + 런 분해 → Phase 3 공간 동시진행 검출 → 템플릿 집계

출력:  route_bundle_group   (번들 그룹: group_id, member_guids, trunk_axis, trunk_z, pitch …)
        route_bundle_template (집계 뷰: (owner,util)별 trunk_zs[], pitch, n_members, n_vert)
```

신규 라우팅(C# 뷰어)은 `route_bundle_template`/`route_bundle_group` 를 키로 조회해 **트렁크 고도
(rack_levels)** 와 **수평·수직 레인 회랑**을 라우팅에 주입한다.

---

## 2. 3단계 파이프라인

```
Phase 1  개별 경로 특징 추출 (참고 메트릭 + 런 분해 입력)
   방향 런 압축 → Arrow Coding(R/H/D) · 꺾임 수 · 리샘플 방향벡터 · 길이 · 규모 · centroid
        │
Phase 2  복합 유사도 (참고 지표 avg_similarity — 그룹화 게이트 아님)
   형태 30% + 방향 30% + 길이 20% + 규모 20%
        │
Phase 3  공간 동시진행(co-travel) 검출  ★ 핵심
   배관을 축정렬 직선 런으로 분해(레인 유지 ±10mm) →
   (owner,util) 키 → 축별 평행 동시진행 다발(sh 공유·진행 겹침·sp 등간격) →
   코너 병합(멤버 ≥2 공유) → trunk_axis·z·pitch·spread 산출
```

---

## 3. Phase 1 — 개별 경로 특징 추출 (`extract_feature`)

배관 1개 폴리라인에서 형태·공간 특징을 뽑는다(유사도 참고 지표 + 꺾임 수). 검출 본체는 §5의 '런'
분해를 별도로 쓴다.

### 3.1 방향 런 압축 (`dir_runs`)

각 세그먼트를 6직교 축(±x,±y,±z) 중 **가장 가까운 축으로 스냅**(`axis_snap`)하고, 연속 동일 방향을
하나의 런으로 병합한다. → `[(축d, 누적길이), …]`.

### 3.2 Arrow Coding (`arrow_code`, `_classify_seg`)

세그먼트를 **R(수직 z) · H(수평 xy) · D(경사)** 로 분류해 연속 압축한 문자열. 분류 기준:

```
vert = |dz|,  horiz = √(dx²+dy²)
vert ≥ horiz : horiz ≤ DIAG_TOL·vert  →  R, 아니면 D
vert < horiz : vert  ≤ DIAG_TOL·horiz →  H, 아니면 D   (DIAG_TOL = 0.34)
```

> **주의 — 엘보 챔퍼**: TB_ROUTE_PATH 의 90° 엘보는 짧은 45° 사선 세그먼트로 표현돼 R↔H 전환마다
> `D` 가 끼어 `RDHDRDHD…` 형태가 된다. 형태 코드 기반 유사도가 이 `D` 에 오염되므로 그룹화 게이트에서
> 빼고 공간 기준으로 검출한다(§6).

### 3.3 꺾임 수 (`count_ortho_bends`)

`dir_runs` 의 런 축(d//2) 전환 횟수 = 90° 엘보 수. `avg_similarity` 참고·선택적 꺾임 게이트(`min_bends`)에
쓴다. v3 기본 `MIN_BENDS=0`(직선 평행도 번들).

### 3.4 기타 특징

| 특징 | 산출 | 용도 |
|---|---|---|
| `units` / `units_rev` | 호 길이 등간격 `RESAMPLE_N`(24)점 리샘플의 단위 방향벡터(+역방향) | 방향 유사도(양방향 정합) |
| `total_len` | 폴리라인 누적 길이 | 길이 유사도 |
| `extent` | bbox (dx,dy,dz) | 규모 유사도 |
| `centroid` | 중심점 | 참고 메트릭 |

---

## 4. Phase 2 — 복합 유사도 (`composite_similarity`)

네 지표의 가중합 ∈ [0,1] — v3 에서는 **참고 지표 `avg_similarity`** 로만 쓰고 그룹화 게이트가 아니다.

```
composite = 0.30·형태(shape)  + 0.30·방향(direction)
          + 0.20·길이(length) + 0.20·규모(scale)
```

| 지표 | 정의 |
|---|---|
| 형태 `shape_similarity` | 1 − Levenshtein(arrow_code) / max(len) |
| 방향 `direction_similarity` | 정렬 단위벡터 평균 코사인(양방향 중 큰 값) |
| 길이 `length_similarity` | 1 − \|Lₐ−L_b\| / max |
| 규모 `scale_similarity` | 축별 min/max extent 평균 |

> **왜 게이트가 아닌가**: 같은 랙 배관은 분기 지점이 달라 길이가 제각각이라 길이(20%)+규모(20%) 항이
> composite 를 끌어내린다. 형태/길이로 묶으면 진짜 랙이 쪼개진다(§6).

---

## 5. Phase 3 — 공간 동시진행 검출 ★ (`detect_bundles`)

핵심 알고리즘. 형태가 아니라 **공간(평행 동시진행)** 으로 다발을 찾는다.

### 5.1 런 분해 (`_extract_runs`) — 레인 유지 ±10mm

각 배관 폴리라인을 **축정렬 직선 런**으로 쪼갠다. 세그먼트의 우세축으로 분류하고, 같은 축이 이어지는
동안 직교(perp) 두 좌표의 밴드가 **±`LANE_TOL_MM`(10)** 을 넘지 않으면 한 런으로 병합한다.

```
런 = (pipe_idx, axis 0/1/2, [t0,t1] 진행구간, perp 두 고정좌표, length)
  · 밴드 초과 → 레인 이탈/꺾임으로 런을 끊는다.
  · 사선(perp 드리프트 > 밴드) 세그먼트는 깨끗한 런을 못 이뤄 자연 제외.
  · 90° 밴딩은 우세축 전환으로 검출돼 ±2° 오차를 흡수(직교 BIM 가정).
  · length < MIN_RUN_MM(800) 런은 버린다(지터·짧은 스텁).
```

### 5.2 축별 평행 동시진행 다발 (`_axis_bundles`)

한 진행축의 런들에서 직교 두 축 중 하나를 **공유(sh)**, 다른 하나를 **간격(sp)** 으로 보고:

```
① sh 좌표 ±LANE_TOL 로 클러스터(같은 레인면 공유)
② 클러스터 안에서 진행 겹침 ≥ MIN_OVERLAP_MM(300) 으로 연결된 성분(_cotravel_components)
③ 성분을 sp 좌표로 정렬 → 등간격 분할(_split_equal_spacing): 인접 간격이 '강건 기준'의
   PITCH_GAP_FACTOR(2.5)배보다 크면 끊는다(outlier 분리). 각 다발 ≥MIN_RACK_MEMBERS(2).
두 (sh,sp) 선택을 모두 시도하되 동일 멤버집합 다발은 중복 제거(평면 랙은 한쪽만 채택).
```

이 절차는 진행축이 **x·y(수평)·z(수직 입상)** 모두에 동일하게 적용된다 → 수직 다발도 검출된다.

> **outlier 분리 예**: sp 오프셋 [0, 500, 3000] → 간격 [500, 2500]. 강건 기준 ≈ 500, 2500 > 2.5·500 →
> 분리. 밀집 쌍 [0,500] 만 번들(3000 은 outlier 제외).

### 5.3 코너 병합 — 밴딩 각도 동일(±2°) 이어붙이기

축별로 검출한 다발들 중 **멤버를 ≥2개 공유**하는 다발은 같은 물리 번들(같은 배관들이 꺾여 이어짐)로
Union-Find 병합한다. 이로써 ㄷ/ㄴ 자로 꺾이는 랙이 축마다 따로가 아니라 **한 그룹**으로 묶인다. 같은
각도(우세축 전환)로 함께 꺾이는 배관만 멤버를 공유하므로, 다르게 분기하는 배관은 자연히 갈라진다.

### 5.4 지표 산출 (`_bundle_stats`)

| 지표 | 의미 |
|---|---|
| `trunk_axis` | 병합 그룹에서 **총 런 길이가 가장 큰 축**(0=x·1=y·2=z) |
| `trunk_z` | 수평 트렁크면 멤버 수평 런 최빈 z(공용 랙 고도). **수직(axis=2)이면 런 중앙 z**(랙고도 아님) |
| `pitch_mm` | 멤버별 대표 sp 좌표 인접 간격의 중앙값(비0) = 레인 간격 |
| `trunk_xy_spread` | sp 오프셋 폭(다발 폭) |
| `n_ortho_bends` | 멤버 꺾임 수 중앙값(참고) |
| `avg_similarity` | 멤버 쌍 평균 복합 유사도(참고, 표본화) |

---

## 6. 재설계 배경 (v1 → v2 → v3)

### 6.1 v1 의 실패 — 형태 유사 + 균일 피치

형태 유사도 Union-Find(≥0.70) + pitch CV ≤0.30 게이트는 실제 랙을 대부분 놓쳤다(실측 CMP_KSCTA08:
4 번들·각 2멤버, 큰 유틸 전멸). 원인 ① 같은 랙은 분기로 길이가 달라 유사도가 임계 아래로 깨짐,
② 실제 랙은 공통 트렁크 공유·불규칙 피치라 CV 가 커서(4↑) 통째 탈락.

### 6.2 v2 — 공간(수평 트렁크) 검출

랙을 '형태'가 아닌 '공간'(같은 트렁크 축 x/y · z-근접 · perp 등간격)으로 정의해 큰 수평 랙을 잡았다.
그러나 **수평 트렁크(centroid·_trunk_axis 가 x/y 만)** 기준이라 두 한계가 남았다:

| 한계 | 증상(사용자 실측 이미지) |
|---|---|
| **수직(입상) 다발 미검출** | 입상으로 나란히 올라가는 다발을 **전혀** 못 찾음(trunk_axis 가 수평만) |
| **수평 다발 일부 누락** | centroid 기반이라 분기로 centroid 가 흩어진 평행 런을 놓침 |

### 6.3 v3 — 세그먼트 동시진행(수평·수직 + 레인 유지 + 코너)

배관을 **축정렬 런**으로 분해해 **진행축(x·y·z)별로** 동시진행·등간격 다발을 직접 찾는다. centroid 대신
**런의 진행구간 겹침 + 직교좌표 공유(±10mm)** 로 평행을 판정하므로 분기·길이 차이에 강건하고, z 진행축을
대등하게 다뤄 수직 입상 다발을 잡는다. 꺾임 게이트는 선택(기본 0)으로 완화해 직선 평행 다발도 포함한다.

### 6.4 결과 (project6 = CLEAN_WTNHJ03)

| 지표 | v2 | v3 |
|---|---|---|
| project6 번들 | (수평만) | **21**(수직/입상 **2** 포함) |
| 큰 수평 랙 | 일부 | UPW_S 42 · HOT DI_S 36 · NFW 23 … |
| 전체 70프로젝트 번들 | 353 | **1,215** (수평 x 312 · y 262 · **수직 z 641**) |
| 수직 다발 보유 키 | 0 | **301** |
| pytest | 15/15 | **17/17** (수직·레인유지·동시진행 테스트 추가) |

> 전체 DB 에서 **수직 다발이 641개로 최다** — 이전 버전이 수직을 통째로 놓치고 있었음을 정량 확인.

---

## 7. 템플릿 집계 (`aggregate_templates`)

탐지 번들들을 (owner, utility) 키로 묶어 **대표 템플릿**을 만든다(신규설계 조회용).

- `trunk_zs` = 그 키의 **수평 트렁크(trunk_axis<2)** 번들 trunk_z 합집합 = 공용 랙 후보 고도들.
  수직(axis=2) 번들의 trunk_z(중앙값)는 랙 고도가 아니므로 **제외**(rack_levels 오염 방지).
- `pitch_mm`·`trunk_xy_spread` = 중앙값, `arrow_code` = 최빈, `n_members` = 합, `n_vert` = 수직 다발 수.

---

## 8. 저장 스키마

`db/schema/route_bundle_group.sql` (`--write-db` 시 자동 적용 + 기존 설치 `ALTER TABLE … trunk_axis` 마이그레이션).

| 객체 | 키 컬럼 |
|---|---|
| `route_bundle_group` (테이블) | group_id, source_file, owner_name, utility, member_guids[], n_members, **trunk_axis**, trunk_z, pitch_mm, trunk_xy_spread, n_ortho_bends, arrow_code |
| `route_bundle_template` (집계 뷰) | owner_name, utility, trunk_zs[]`(수평만)`, pitch_mm, trunk_xy_spread, n_members, n_groups, **n_vert** |

---

## 9. 신규 자동설계에서의 활용 (C# 뷰어)

`Model/BundleStore.cs` 가 `route_bundle_template` 를 읽어 (owner,util) 키별 템플릿을, `route_bundle_group`
에서 멤버 GUID→group_id 를 메모리에 올린다. `SceneViewModel`·`Diagnostics/DbRouteDiag` 가 활용한다.

| 활용 | 방법 |
|---|---|
| **트렁크 고도(z)** | 템플릿 `trunk_zs`(수평) → 엔진 `rack_levels`(회랑 페널티 면제 z-셀) → 새 배관이 공용 랙 높이에 모임 |
| **수평 레인 회랑** | 트렁크 고도 ±1셀 수평 런을 타이트 회랑 셀로 주입 → 충돌회피가 인접 레인에 분산(등간격 다발) |
| **수직 레인 회랑(v3)** | **번들 멤버(`GroupIdOf≥0`)의 수직 입상**을 회랑 셀로 주입(`BuildBundleCorridorCells(includeVertical:true)`) → 수직 입상 다발도 라우팅이 따라감 |
| **패턴 표시** | 학습 트렁크 레인 + 입상을 보라 반투명 큐브로(메인/미니 3D) |

미적재/키 미스 시 기하 규칙으로 자동 폴백(무해).

**실측(project6 ALL c100, 스텁 ON)**: 번들 OFF 208/208·rackZ 35.5% → 번들 ON(수직 레인) 207/208·
totalLen↓·**rackZ 38.5%**·corridor 46,483셀. 수직 레인이 설계추종(짧은 경로·높은 랙집중)을 강화한다.

---

## 10. 파라미터·CLI

### 10.1 주요 상수

| 상수 | 기본 | 의미 |
|---|---|---|
| `LANE_TOL_MM` | 10 | **레인 유지** — 진행 중 직교좌표 변동 허용(±). 핵심값 |
| `BEND_ANGLE_TOL_DEG` | 2 | 밴딩 각도 동일 허용(±) — 직교 분류로 흡수 |
| `MIN_RUN_MM` | 800 | 동시진행 런 최소 진행길이 |
| `MIN_OVERLAP_MM` | 300 | 함께 진행으로 볼 최소 진행 겹침 |
| `PITCH_GAP_FACTOR` | 2.5 | 등간격 분할 갭 배수 |
| `MIN_RACK_MEMBERS` | 2 | 다발 최소 멤버 |
| `MIN_BENDS` | 0 | 선택적 꺾임 게이트(0=직선 평행도 번들) |
| `DIAG_TOL` | 0.34 | R/H/D 직교 판정 tol |
| `Z_BIN_MM` | 100 | trunk_z 최빈 버킷 |

> `SIM_THRESHOLD`(0.70)·`PITCH_CV_MAX`(0.30) 는 v3 에서 **그룹화 게이트로 미사용**(CLI 인자 호환 유지).

### 10.2 실행 (프로젝트 루트)

```powershell
# 탐지 + 콘솔 리포트(DB 미적재) — 'ax' 열에 진행축(x/y/z), 헤더에 수직 다발 수
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --report

# 템플릿(신규설계 조회 형태) 출력
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --templates

# 결과 저장(route_bundle_group + 템플릿 뷰, 스키마/마이그레이션 자동)
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 6 --write-db

# DB 전체(모든 프로젝트) 탐지 + 저장
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --all --write-db
```

---

## 11. 결론

그룹배관 생성은 **기존 설계배관에서 평행 다발(랙)을 공간 기준으로 탐지**해, 신규 자동 라우팅이 사람
설계처럼 다발로 모이도록 학습 데이터를 만든다. v3 재설계로 **수평·수직(입상)을 대등하게** 다루고,
**레인 유지(±10mm)·동시진행·코너 추종(±2°)** 으로 분기·길이 차이에 강건하게 진짜 다발을 잡는다.

**핵심 교훈**: 평행 다발은 '형태'가 아니라 '공간(같은 축으로 함께 진행 + 동일간격 + 레인 유지)'으로
정의해야 한다. 그리고 진행축은 수평만이 아니라 **수직도 대등하게** 다뤄야 입상 다발을 놓치지 않는다.

---

*문서 끝.*

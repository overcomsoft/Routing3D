# 그룹배관(번들) 생성 알고리즘 — 설계·구현 상세

> 모듈: `python_experiments/routing3d_py/bundle_detect.py` · 단위 mm · 직교(맨해튼) 형상 가정.
> 사람이 설계한 기존배관(TB_ROUTE_PATH)에서 '평행 다발(번들)'을 자동 탐지해, 신규 자동라우팅이
> 기존설계처럼 공용 트렁크 랙에 다발로 모이도록 학습 데이터(route_bundle_group/template)를 만든다.

---

## 1. 개요

### 1.1 그룹배관(번들)이란

플랜트에서 같은 유틸리티 배관들은 **공용 파이프 랙(트렁크 고도)을 따라 나란히** 깔린다. 이렇게
**같은 트렁크 축·같은 높이에 평행하게 모인 배관 다발**을 '번들(bundle)'이라 부른다. 자동 라우팅이
이를 따르면 결과가 사람 설계처럼 정돈된다(L4 패턴 학습).

### 1.2 핵심 정의 (v2)

> **번들 = 같은 (장비 owner · 유틸리티) 의 배관 중, 같은 트렁크 주축(x 또는 y)·근접한 트렁크
> 고도(z)에서 평행하게 달리는 ≥2개 배관의 묶음.**

초기 버전(v1)은 번들을 '형태 유사 + 균일 이격간격'으로 정의했으나, 실데이터 검증 결과 이 정의가
실제 랙을 대부분 놓쳐(§6) **'공간(트렁크 축·높이·평행)'** 정의로 재설계했다.

### 1.3 입력·출력

```
입력:  TB_ROUTE_PATH (route_db.load_existing_pipes)
        = 기존 설계배관 폴리라인 list[ExistingPipe]
        (route_path_guid, owner_name, utility, points[월드 mm], source/target_pos)

처리:  Phase 1 특징 추출 → Phase 3 공간 랙 검출 → 템플릿 집계

출력:  route_bundle_group   (번들 그룹: group_id, member_guids, trunk_z, pitch …)
        route_bundle_template (집계 뷰: (owner,util)별 trunk_zs[], pitch, n_members)
```

신규 라우팅(C# 뷰어)은 `route_bundle_template` 를 (owner,util) 키로 조회해 **트렁크 고도(rack_levels)**
와 **레인 회랑**을 라우팅에 주입한다.

---

## 2. 3단계 파이프라인

```
Phase 1  개별 경로 특징 추출
   방향 런 압축 → Arrow Coding(R/H/D) · 꺾임 수 · 트렁크 주축 ·
   리샘플 방향벡터 · 길이 · 규모(extent) · centroid
        │
Phase 2  복합 유사도(참고 지표로 강등 — v2에서 그룹화 게이트 아님)
   형태 30% + 방향 30% + 길이 20% + 규모 20%
        │
Phase 3  공간 랙 검출  ★ 핵심
   (owner,util) pre-filter → trunk_axis 분리 → z-근접 군집 →
   perp 등간격 런 분할(outlier 분리) → 꺾임 게이트 → trunk_z·pitch·spread
```

---

## 3. Phase 1 — 개별 경로 특징 추출 (`extract_feature`)

배관 1개 폴리라인에서 형태·공간 특징을 뽑는다.

### 3.1 방향 런 압축 (`dir_runs`)

각 세그먼트를 6직교 축(±x,±y,±z) 중 **가장 가까운 축으로 스냅**(`axis_snap` = 최대 절대성분 축의 부호)
하고, 연속 동일 방향을 하나의 런으로 병합한다. → `[(축d, 누적길이), …]`.

### 3.2 Arrow Coding (`arrow_code`, `_classify_seg`)

세그먼트를 **R(수직 z) · H(수평 xy) · D(경사)** 로 분류해 연속 압축한 문자열. 분류 기준:

```
vert = |dz|,  horiz = √(dx²+dy²)
vert ≥ horiz : horiz ≤ DIAG_TOL·vert  →  R, 아니면 D
vert < horiz : vert  ≤ DIAG_TOL·horiz →  H, 아니면 D   (DIAG_TOL = 0.34)
```

> **주의 — 엘보 챔퍼**: TB_ROUTE_PATH 의 90° 엘보는 짧은 45° 사선 세그먼트로 표현돼 R↔H 전환마다
> `D` 가 끼어 `RDHDRDHD…` 형태가 된다. 형태 코드 기반 유사도가 이 `D` 에 오염되므로, v2 는 형태
> 유사도를 그룹화 게이트에서 빼고 공간 기준으로 전환했다(§6).

### 3.3 꺾임 수 (`count_ortho_bends`)

`dir_runs` 의 런 축(d//2) 전환 횟수 = 90° 엘보 수. 번들 게이트(≥`MIN_BENDS`=2)에 쓴다. `axis_snap`
기반이라 챔퍼 `D` 에 비교적 강건(우세 축으로 스냅).

### 3.4 트렁크 주축 (`_trunk_axis`)

가장 긴 **수평** 런의 축(0=x, 1=y) = 이 배관이 따르는 트렁크 방향. 평행 다발은 같은 주축을 공유한다.

### 3.5 기타 특징

| 특징 | 산출 | 용도 |
|---|---|---|
| `units` / `units_rev` | 호 길이 등간격 `RESAMPLE_N`(24)점 리샘플의 단위 방향벡터(+역방향) | 방향 유사도(양방향 정합) |
| `total_len` | 폴리라인 누적 길이 | 길이 유사도 |
| `extent` | bbox (dx,dy,dz) | 규모 유사도 |
| `centroid` | 중심점 | **공간 랙의 perp 오프셋** |

---

## 4. Phase 2 — 복합 유사도 (`composite_similarity`)

네 지표의 가중합 ∈ [0,1]:

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

> **v2 에서의 역할 변경**: v1 은 이 유사도(≥0.70)로 Union-Find 군집을 만들어 번들을 정의했다.
> 그러나 **같은 랙 배관은 분기 지점이 달라 길이가 제각각**이라 길이(20%)+규모(20%) 항이 composite 를
> 임계 아래로 끌어내려 군집이 잘게 부서졌다(§6). v2 는 이 유사도를 **참고 메트릭(avg_similarity)** 으로만
> 쓰고, 그룹화는 공간 기준으로 한다.

---

## 5. Phase 3 — 공간 랙 검출 ★ (`detect_bundles` · `_split_into_racks`)

핵심 알고리즘. 형태가 아니라 **공간**으로 평행 다발을 찾는다.

### 5.1 절차

```
for each (owner_name, utility) 키:                       # ② pre-filter (≥2개)
    for each trunk_axis(0=x,1=y) 그룹:                    # 같은 트렁크 방향끼리
        racks = _split_into_racks(그 축 배관들, axis)     # ③ 공간 분할
        for each rack:
            if median(꺾임) < MIN_BENDS: 건너뜀            # ④ 꺾임 게이트
            pitch, cv, spread = _pitch_stats(rack, axis)  # ⑤ 지표 산출
            emit BundleGroup(trunk_z=_trunk_z(rack), …)
```

### 5.2 `_split_into_racks` — z-근접 군집 + perp 등간격 런 분할

```
① z-근접 군집:  배관별 트렁크 고도(_pipe_trunk_z = 수평 런 최빈 z)로 정렬 후,
                연속 간격 ≤ Z_MERGE_MM(400) 이면 한 랙 높이로 묶는다.
                (고정 버킷의 경계 분할 방지 — 14900/15000/15100 이 한 랙)

② perp 등간격 런 분할:  같은 높이 안에서 perp(주축의 직교 수평축) 오프셋(centroid)으로 정렬.
                인접 간격이 '강건 레인 간격 기준'의 PITCH_GAP_FACTOR(2.5)배보다 크면 끊는다(다른 랙).
                강건 기준 = 비0 간격의 '작은 절반' 중앙값 → 소수의 큰 랙-경계 갭·outlier 에 안 휘둘림.
                각 런(≥MIN_RACK_MEMBERS=2)이 하나의 번들.
```

> **outlier 분리 예**: perp 오프셋 [0, 500, 3000] → 간격 [500, 2500]. 강건 기준 ≈ 500,
> 2500 > 2.5·500 → 분리. → 밀집 쌍 [0,500] 만 번들(3000 은 outlier 로 제외).

### 5.3 지표 산출

| 지표 | 함수 | 의미 |
|---|---|---|
| `trunk_z` | `_trunk_z` | 멤버 수평 런 중점 z 의 길이가중 최빈(Z_BIN_MM=100 버킷) = 공용 랙 고도 |
| `pitch_mm` | `_pitch_stats` | perp 인접 간격 중 **비0** 값의 중앙값 = 실제 레인 간격 |
| `trunk_xy_spread` | `_pitch_stats` | perp 오프셋 폭(다발 폭) |
| `avg_similarity` | `_avg_pair_similarity` | 멤버 쌍 평균 복합 유사도(참고용, 표본화) |

> **pitch CV 게이트 폐기**: 실제 플랜트 랙은 **공통 트렁크를 공유**(perp 간격 0)·불규칙 피치라 CV 가
> 크다(실측 4↑). 균일 피치를 요구하면 진짜 트렁크가 전부 기각되므로, v2 는 CV 게이트를 제거하고
> CV 는 참고 지표로만 둔다(§6).

---

## 6. v1 → v2 재설계 배경 (왜 바꿨나)

실데이터(CMP_KSCTA08, 기존배관 1,016개) 검증에서 v1 의 치명적 한계가 드러났다.

### 6.1 v1 의 실패 (실측)

| 확인 | 결과 |
|---|---|
| v1 검출 | **4 번들 · 각 2멤버** (ALKA·UPW_S·AKWW 등 큰 유틸 전멸) |
| ALKA(140개) | **0 번들** — 템플릿 자체가 없어 패턴 표시도 안 됨 |
| ALKA 쌍 유사도 | 평균 0.85 · 300/300쌍 ≥ 0.70 (유사도는 충분) |

### 6.2 두 가지 근본 원인

1. **형태 유사도 Union-Find 가 같은 랙을 쪼갬** — 한 랙 배관들은 분기 지점이 달라 길이가 제각각.
   유사도의 길이(20%)+규모(20%) 항이 composite 를 0.70 밑으로 끌어내려 군집이 잘게 부서졌다.
2. **pitch CV ≤ 0.30 게이트가 진짜 트렁크를 기각** — 실제 랙은 공통 트렁크 공유·불규칙 피치라
   CV 가 커서(실측 4↑) 통째 탈락. 우연히 2개만 뭉친 작은 군집(CV=0)만 살아남았다.

### 6.3 v2 의 해법

랙(평행 다발)을 **형태가 아니라 공간**(같은 트렁크 축 · z-근접 · perp 등간격 평행)으로 정의해
재구현. 형태유사 Union-Find·pitch CV 게이트를 폐기하고 꺾임 게이트만 유지.

### 6.4 결과 (CMP_KSCTA08)

| 지표 | v1 | v2 |
|---|---|---|
| 번들 그룹 | 4 | ~246(전수) / 36(프로젝트 bbox) |
| 검출 유틸 | 4개 | **12개 전부** |
| ALKA trunk_zs | 없음 | **[14700, 14900, 21700]** |
| 멤버 커버리지 | 극소 | ~89%(908/1016) |
| pytest | — | 15/15 (+ 옛 테스트 1개 새 동작에 맞게 갱신) |

전체 70개 프로젝트 재생성: **총 1,805개 번들 그룹**(69/70 검출).

---

## 7. 템플릿 집계 (`aggregate_templates`)

탐지 번들들을 (owner, utility) 키로 묶어 **대표 템플릿**을 만든다(신규설계 조회용).

- `trunk_zs` = 그 키의 모든 번들 trunk_z 합집합(반올림·중복 제거) = **공용 랙 후보 고도들**
- `pitch_mm`·`trunk_xy_spread` = 중앙값, `arrow_code` = 최빈, `n_members` = 합

> 한 유틸이 여러 랙 높이를 가지면 `trunk_zs` 에 모두 모인다(예 ALKA [14700,14900,21700]).
> 번들이 다소 잘게 나뉘어도 템플릿 `trunk_zs` 는 안정적(집계가 흡수).

---

## 8. 저장 스키마

`db/schema/route_bundle_group.sql` (`--write-db` 시 자동 적용).

| 객체 | 키 컬럼 |
|---|---|
| `route_bundle_group` (테이블) | group_id, source_file, owner_name, utility, member_guids[], n_members, trunk_z, pitch_mm, trunk_xy_spread, n_ortho_bends, arrow_code |
| `route_bundle_template` (집계 뷰) | owner_name, utility, trunk_zs[], pitch_mm, trunk_xy_spread, n_members, n_groups |

---

## 9. 신규 자동설계에서의 활용 (C# 뷰어)

`Model/BundleStore.cs` 가 `route_bundle_template` 를 source_file 로 읽어 (owner,util) 키별 템플릿을
메모리에 올린다. `SceneViewModel` 이 라우팅·표시에 활용한다.

| 활용 | 방법 |
|---|---|
| **트렁크 고도(z)** | 템플릿 `trunk_zs` → 엔진 `rack_levels`(회랑 페널티 면제 z-셀) → 같은 유틸 새 배관이 공용 랙 높이에 모임 |
| **레인 회랑(xy)** | 트렁크 고도 ±1셀 수평 런을 타이트 회랑 셀로 주입 → 충돌회피가 인접 레인에 분산(등간격 다발) |
| **패턴 표시** | 학습 트렁크 레인 + 입상(트렁크 밴드 통과 리저)을 보라 반투명 큐브로(메인/미니 3D) |

미적재/키 미스 시 기하 규칙으로 자동 폴백(무해).

---

## 10. 파라미터·CLI

### 10.1 주요 상수

| 상수 | 기본 | 의미 |
|---|---|---|
| `MIN_BENDS` | 2 | 번들 최소 90° 꺾임 |
| `DIAG_TOL` | 0.34 | R/H/D 직교 판정 tol |
| `RESAMPLE_N` | 24 | 방향 유사도 리샘플 점수 |
| `Z_BIN_MM` | 100 | trunk_z 최빈 버킷 |
| `Z_MERGE_MM` | 400 | 같은 랙으로 볼 z-근접 임계 |
| `PITCH_GAP_FACTOR` | 2.5 | 랙 경계 갭 분할 배수 |
| `MIN_RACK_MEMBERS` | 2 | 랙 최소 멤버 |
| `W_SHAPE/DIR/LEN/SCALE` | .3/.3/.2/.2 | 복합 유사도 가중(참고 지표) |

> `SIM_THRESHOLD`(0.70)·`PITCH_CV_MAX`(0.30) 는 v2 에서 **그룹화 게이트로 미사용**(CLI 인자 호환 유지).

### 10.2 실행 (프로젝트 루트)

```powershell
# 탐지 + 콘솔 리포트(DB 미적재)
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 1 --report

# 템플릿(신규설계 조회 형태) 출력
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 1 --templates

# 결과 저장(route_bundle_group + 템플릿 뷰, 스키마 자동 적용)
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --project 1 --write-db

# DB 전체(모든 프로젝트) 탐지 + 저장
.\.venv\Scripts\python.exe -m routing3d_py.bundle_detect --all --write-db
```

---

## 11. 결론

그룹배관 생성은 **기존 설계배관에서 공용 트렁크 랙(평행 다발)을 공간 기준으로 탐지**해, 신규 자동
라우팅이 사람 설계처럼 다발로 모이도록 학습 데이터를 만든다. v2 재설계로 형태유사·균일피치라는
비현실적 가정을 버리고, 실제 플랜트 랙(공통 트렁크 공유·불규칙 피치)을 충실히 잡는다.

**핵심 교훈**: 평행 다발(랙)은 '형태'가 아니라 '공간(같은 축·높이·평행)'으로 정의해야 한다. 형태/길이
유사도는 같은 랙의 분기 다양성을 과소평가해 검출을 망친다.

---

*문서 끝.*

# scene.txt 데이터 포맷 규격서 (v1, 동결) — Routing3D

> Phase 2 인터페이스 동결 · [phase2_plan.md](../phase2_plan.md) Step 2.2
> 레퍼런스 구현: `python_experiments/routing3d_py/scene_io.py`
> 버전 1.0 · 2026-05-28 · 단위 **mm**
> 이 규격은 Phase 2/3(Python 실험 ↔ C++ 엔진)이 공유하는 **중간 데이터 포맷의 정식 규격**이다. v1 은 동결되며, 변경은 §8 버전 정책을 따른다.

---

## 1. 개요 · 설계 목표

라우팅 씬의 **입력**(격자·파라미터·장애물·작업)과 **출력**(경로·방문·지표)을 사람이 읽을 수 있는 텍스트 파일 하나로 직렬화한다.

- **무손실(round-trip)**: write → read → write 결과가 **바이트 단위로 동일**.
- **C++ 포팅 친화**: 섹션 헤더 + TAB 구분 행. 한 줄 = 한 레코드(또는 한 키-값). 파서는 단순 상태기계.
- **3개 레이어**: 점유(`[obstacles]`) · 경로(`[path]`) · 방문(`[visited]`).

---

## 2. 어휘 · 인코딩 규칙

| 규칙 | 내용 |
|---|---|
| 인코딩 | UTF-8 |
| 개행 | LF (`\n`) |
| 필드 구분자 | **TAB** (`\t`). 필드 값은 TAB·개행을 포함하지 않는다고 가정 |
| 주석 | `#` 로 시작하는 줄 (무시) |
| 헤더 지시자 | `@` 로 시작하는 줄 (`@format`, `@version`) |
| 섹션 | `[name]` 로 시작하는 줄. 뒤에 TAB 구분 속성 `key=value` 가능 (예: `[obstacles]\tcount=983`) |
| 빈 줄 | 무시 |
| **실수** | Python `repr(float)` 로 기록(무손실). 파싱은 표준 float 파서로 가능 |
| **정수** | 10진 정수 문자열 |
| **null** | 토큰 `\N` (백슬래시+대문자 N). **빈 문자열("")과 구분**된다 |

> **무손실 핵심**: (1) 실수 `repr` 보존, (2) `\N`=None vs ""=빈문자열 구분. 이 두 규칙으로 모든 필드가 정확히 왕복된다.

---

## 3. 파일 구조 (순서 고정)

```
# Routing3D scene file — units: mm
@format routing3d-scene
@version 1

[grid]            ← 격자 메타 (필수)
[params]          ← 비용 파라미터 (필수)
[obstacles] count=N   ← 점유 레이어 입력 (필수, N=0 가능)
[tasks] count=M       ← 라우팅 작업 (필수, M=0 가능)
[results] count=K     ← (선택) 결과 묶음 시작. 없으면 결과 없음
  [result] task=<idx> ← 작업 idx 의 지표
  [path]   task=<idx> count=<n>   ← (선택) 경로 셀
  [visited]task=<idx> count=<n>   ← (선택) 방문 셀
  ... (성공/결과 있는 작업마다 반복)
```

---

## 4. 섹션별 필드 정의

### 4.1 `[grid]` (키-값)

| 키 | 형식 | 의미 |
|---|---|---|
| `cell_mm` | float | 셀 한 변 길이(mm) |
| `origin` | float×3 | 격자 원점 월드좌표(mm) |
| `shape` | int×3 | 격자 셀 개수 (nx, ny, nz) |

### 4.2 `[params]` (키-값, `RouteParams`)

| 키 | 형식 | 비고 |
|---|---|---|
| `cell_mm` | float | |
| `w_turn` | float | 회전 가산 |
| `w_clear` | float | 클리어런스 계수 |
| `clearance_radius` | int | |
| `clearance_connectivity` | int | 6 또는 26 |
| `w_tier` | `z:mm` 토큰 0..n개 | 비어 있으면 키만(`w_tier`) |

### 4.3 `[obstacles]` (레코드 행, 헤더 `count=N`)

행 = TAB 구분 10필드:

```
minx  miny  minz  maxx  maxy  maxz  ost_type  name  object_id  ddworks_type
float float float float float float  str|\N    str|\N str|\N    str|\N
```

### 4.4 `[tasks]` (레코드 행, 헤더 `count=M`)

행 = TAB 구분 11필드:

```
sx  sy  sz  gx  gy  gz  utility  utility_group  start_name  end_name  end_instance_guid
float×3      float×3     str|\N   str|\N         str|\N      str|\N    str|\N
```

### 4.5 `[results]` / `[result]` / `[path]` / `[visited]`

- `[results]\tcount=K` — 결과 있는 작업 수 K. (작업별 블록 시작 표지)
- `[result]\ttask=<idx>` 다음 키-값:

| 키 | 형식 |
|---|---|
| `success` | `1`/`0` |
| `length_mm` | float (기하 길이) |
| `cost_mm` | float (페널티 포함 총비용) |
| `turns` | int |
| `expanded_nodes` | int |
| `elapsed_ms` | float (참고용) |

- `[path]\ttask=<idx>\tcount=<n>` 다음 n줄: `i\tj\tk` (경로 셀).
- `[visited]\ttask=<idx>\tcount=<n>` 다음 n줄: `i\tj\tk` (방문 셀, 선택).

---

## 5. 불변식

| ID | 불변식 |
|---|---|
| F1 | `[grid] cell_mm` 필수(없으면 오류) |
| F2 | round-trip 무손실: write→read→write 바이트 동일 |
| F3 | `\N`=None, `""`=빈 문자열 (구분 보존) |
| F4 | 실수는 `repr` 보존 → 재파싱 시 동일 float |
| F5 | `count=` 는 뒤따르는 레코드/셀 행 수와 일치 |
| F6 | `[result]/[path]/[visited]` 의 `task` idx 는 `[tasks]` 인덱스(0-기반)와 대응 |

---

## 6. 최소 예시

```
# Routing3D scene file — units: mm
@format routing3d-scene
@version 1

[grid]
cell_mm	50.0
origin	0.0	0.0	0.0
shape	10	10	3

[params]
cell_mm	50.0
w_turn	300.0
w_clear	10.0
clearance_radius	2
clearance_connectivity	6
w_tier	1:200.0

[obstacles]	count=1
# minx	miny	minz	maxx	maxy	maxz	ost_type	name	object_id	ddworks_type
100.0	100.0	0.0	200.0	200.0	150.0	OST_Columns	col 1	G1	\N

[tasks]	count=1
# sx	sy	sz	gx	gy	gz	utility	utility_group	start_name	end_name	end_instance_guid
25.0	275.0	75.0	475.0	275.0	75.0	PA	Gas	POC 1	\N	IG1

[results]	count=1
[result]	task=0
success	1
length_mm	450.0
cost_mm	450.0
turns	0
expanded_nodes	10
elapsed_ms	0.2
[path]	task=0	count=10
0	5	1
1	5	1
...
```

> 실데이터 발췌: `python_experiments/out/project6.scene.txt` (장애물 983 · 작업 208). 이름 필드에 `//`·공백·특수문자가 있어도 TAB·`\N` 규칙으로 무손실 왕복된다.

---

## 7. C++ 파서 구현 주의점

1. 줄 단위 읽기 → `\r` 제거 → 빈 줄/`#` 스킵 → `@`(헤더 검증) / `[`(섹션 전환) / 그 외(현재 섹션 규칙으로 파싱).
2. 섹션 전환 시 `[name]\tk=v` 의 속성 파싱(`task=`, `count=`).
3. 필드 분리는 TAB 단일 분리(공백으로 split 금지 — 이름에 공백 포함).
4. `\N` → null/optional 없음, 그 외 → 문자열 그대로(빈 문자열 허용).
5. 실수 파싱은 로캘 무관(`.` 소수점) 파서 사용.

---

## 8. 버전 정책 (v1 동결 / v2 확장 규칙)

- `@version 1` 불일치 시 거부(현재 레퍼런스 동작).
- **하위호환 확장**(새 선택 섹션/키 추가)은 `@version` 유지 가능하되, 신규 필드는 끝에 추가하고 구파서가 무시 가능하도록 한다.
- **비호환 변경**(필드 순서/의미 변경, 필수 섹션 추가)은 `@version 2` 로 올리고 회귀 골든셋을 재생성한다.
- 향후 확장 후보(v2): 배관 직경/재질/우선순위, 그룹 메타, 좌표계 식별자.

> 본 v1 규격은 동결 대상이다. 변경은 [freeze_signoff.md](freeze_signoff.md) 의 변경관리 절차를 따른다.

# Phase 2 — 인터페이스 동결 세부 개발계획서

> 상위 문서: [development_plan.md](development_plan.md) §3.2
> 입력: [phase2_input_notes.md](phase2_input_notes.md) (Phase 1 결과 정리), [phase1_plan.md](phase1_plan.md)
> 작성일: 2026-05-28 · 버전 1.0
> 단위 규약: 모든 길이·좌표 **밀리미터(mm)**, 기본 셀 50mm(`cell_mm`).

## 목차

1. [목표와 비목표](#1-목표와-비목표)
2. [입력 자산 (Phase 1 산출물)](#2-입력-자산-phase-1-산출물)
3. [세부 작업 단계](#3-세부-작업-단계)
4. [산출물 / 디렉토리 구조](#4-산출물--디렉토리-구조)
5. [완료 기준 (Definition of Done)](#5-완료-기준-definition-of-done)
6. [리스크와 대응](#6-리스크와-대응)
7. [Phase 3 연결](#7-phase-3-연결)

---

## 1. 목표와 비목표

### 1.1 목표

실험으로 확정된 알고리즘·제약·데이터 포맷을 **명세로 고정**하여 Phase 3 C++ 포팅의 설계 변경 리스크를 제거한다.

- 비용함수·제약·우선순위 규칙을 **모호함 없이** 구현 가능한 수준으로 문서화·확정한다.
- 중간 데이터 포맷(`scene.txt`)을 정식 규격(문법/필드/불변식)으로 **동결**한다.
- 대표 시나리오를 입력→기대지표 회귀 케이스로 **고정**(골든셋)한다.
- 셀 규모별 **성능 목표치**(시간·메모리)를 수치로 정의한다.

### 1.2 비목표 (이 단계에서 하지 않는 것)

- C++ 구현 자체 → Phase 3.
- 새 알고리즘 추가/튜닝 → Phase 1 으로 회귀해서 진행(명세는 '확정된 것'만 담는다).
- rip-up·CBS·OpenVDB·FCL 설계 상세 → Phase 3(목표치·인터페이스 자리만 명세에 예약).

> **동결 원칙**: 명세는 *이미 검증된 것*만 기술한다. 미검증 아이디어는 §open-questions 로 분리하고 동결 대상에서 제외한다.

---

## 2. 입력 자산 (Phase 1 산출물)

| 자산 | 위치 | Phase 2 에서의 역할 |
|---|---|---|
| Phase 1→2 입력 노트 | [phase2_input_notes.md](phase2_input_notes.md) | 명세서 초안의 1차 소스 |
| 알고리즘 레퍼런스 구현 | `python_experiments/routing3d_py/` | 명세 ↔ 코드 대조(정답) |
| 상세 설계 문서 | `docs/spec/step1_*.docx` | 알고리즘 서술 재사용 |
| 회귀 시나리오 3종 | `python_experiments/tests/scenarios/` | 골든셋 동결 대상 |
| 베이스라인 파라미터 | `python_experiments/experiments/baseline_params.json` | 명세 기본값 |
| scene.txt 예시 | `scene_io.py` + project6 실측 | 포맷 규격 예시 |

---

## 3. 세부 작업 단계

각 단계 산출물은 **(a) 코드/노트에서 사실 추출 → (b) 규격 문서화 → (c) 레퍼런스와 교차검증** 순.

### 3.1 Step 2.1 — 알고리즘 명세서

**산출물**: `docs/spec/algorithm_spec.md` (한글 + 의사코드).

| 항목 | 내용 |
|---|---|
| 좌표·단위·격자 규약 | mm, `cell_mm`, 셀↔월드, 격자 밖=점유 |
| 점유맵 질의 인터페이스 | `is_blocked`/`in_bounds`/`to_world`/`to_cell`/`inflate` 계약 + 3 백엔드 동등성 |
| 직교 A* | 상태=셀, `f=g+h`, `h=manhattan×cell_mm`, tie-break, 경로 복원 (의사코드) |
| 비용함수 A* | 상태=(셀,진입방향), `move_cost`, `cell_penalty`, clearance distance transform (의사코드) |
| 다중 배관 순차 | 우선순위 정렬 + 깔린 경로 점유 + 스냅 (의사코드) |
| 불변식·계약 | admissibility(가산항≥0), 경로 비점유, 충돌 0, 결정성(tie-break) |

**검증**: 명세 의사코드가 레퍼런스 동작과 일치(시나리오 지표 재현)함을 명시.

### 3.2 Step 2.2 — 데이터 포맷 규격서 (scene.txt v1 동결)

**산출물**: `docs/spec/scene_format_spec.md`.

- 파일 문법(헤더 `@format/@version`, 섹션 `[grid] [params] [obstacles] [tasks] [results]/[result]/[path]/[visited]`).
- 필드 정의·자료형·순서·단위, `\N`(null) 규칙, 실수 `repr` 보존 규칙.
- 무손실 불변식(write→read→write 바이트 동일), 버전 정책(v2 확장 규칙).
- 최소 예시 + project6 발췌. C++ 파서 구현 주의점(TAB 분리, 상태기계).

**검증**: 규격대로 `scene_io.py` 가 동작함을 round-trip 테스트로 보증(이미 통과).

### 3.3 Step 2.3 — 회귀 테스트 세트 동결 (골든셋)

**산출물**: `docs/spec/regression_set.md` + 기존 `tests/scenarios/` 동결 표기.

- 3종 시나리오의 입력·기대지표·허용범위를 표로 고정(입력 JSON 해시/요약 포함).
- Phase 3 C++ 가 **동일 지표를 허용범위 내 재현**해야 함을 합격 기준으로 명문화.
- (선택) scene.txt 골든 파일 1~2개를 `tests/golden/` 에 동결.

### 3.4 Step 2.4 — 성능 목표 정의

**산출물**: `docs/spec/performance_targets.md`.

- 셀 규모(ROI)별 **목표 탐색 시간 / 메모리** 표 (예: 10⁶ / 10⁷ / 10⁸ 셀).
- 단일/다중 배관, Python(현재 측정치) vs Phase 3 C++(목표) 대비.
- 8,000m 스케일 대응 전략(계층 corridor + OpenVDB)의 수용 기준.
- ⚠️ **수치는 사용자 입력 필요** — 대상 플랜트 최대 규모, 1배관 응답 시간 예산, 전체 배관 수, 메모리 상한.

### 3.5 Step 2.5 — 동결 합의 체크리스트 (sign-off)

**산출물**: `docs/spec/freeze_signoff.md`.

- 위 산출물별 검토 항목 체크리스트 + 동결 합의 표기란.
- 변경 관리 규칙: 동결 후 변경은 버전업(+회귀 재실행) 절차로만.

---

## 4. 산출물 / 디렉토리 구조

```
docs/spec/
├── step1_1_occupancy.docx ...        # Phase 1 설계 문서(기존)
├── algorithm_spec.md                 # Step 2.1
├── scene_format_spec.md              # Step 2.2
├── regression_set.md                 # Step 2.3
├── performance_targets.md            # Step 2.4
└── freeze_signoff.md                 # Step 2.5
python_experiments/tests/
├── scenarios/                        # 동결 대상(기존)
└── golden/                           # (선택) scene.txt 골든 파일
```

---

## 5. 완료 기준 (Definition of Done)

- [x] 알고리즘 명세서 — 의사코드 수준, 레퍼런스와 일치 확인. ([spec/algorithm_spec.md](spec/algorithm_spec.md))
- [x] scene.txt v1 규격서 — 문법·필드·불변식·버전 정책 동결. ([spec/scene_format_spec.md](spec/scene_format_spec.md))
- [x] 회귀 골든셋 문서화 + Phase 3 합격 기준 명문화. ([spec/regression_set.md](spec/regression_set.md))
- [x] 성능 목표치 수치 확정(사용자 입력 반영: 8000m / <1초 / <1분 / <32GB). ([spec/performance_targets.md](spec/performance_targets.md))
- [~] 동결 합의 체크리스트 작성 완료, **sign-off 는 검토자 확인 대기**. ([spec/freeze_signoff.md](spec/freeze_signoff.md))

> 산출물 4종 + 체크리스트 **작성 완료**. 검토자 sign-off(freeze_signoff.md §4) 후 Phase 2 동결 확정 → Phase 3 착수.

---

## 6. 리스크와 대응

| 리스크 | 영향 | 대응 |
|---|---|---|
| 명세–코드 불일치 | C++ 포팅 오류 | 명세 작성 시 레퍼런스 함수/시나리오 지표를 직접 인용·교차검증 |
| 성능 목표 비현실 | Phase 3 재설계 | 현재 Python 측정치 + C++ 일반 배수로 보수적 산정, 조기 PoC 로 검증 |
| 동결 후 알고리즘 변경 욕구 | 규격 흔들림 | 변경은 버전업 절차로만, 미검증 아이디어는 open-questions 로 분리 |
| 포맷 확장 필요(직경·재질 등) | v1 부족 | v2 확장 규칙을 규격서에 미리 명시 |

---

## 7. Phase 3 연결

본 단계 산출물(명세서·포맷 규격·골든셋·성능 목표)이 Phase 3 C++ 엔진의 입력이 된다.
Phase 3 합격 = **3종 골든셋 지표를 허용범위 내 재현 + 성능 목표 달성**.

> **다음 즉시 실행 항목** = Step 2.1 알고리즘 명세서 작성. 단, Step 2.4 성능 목표는 사용자 입력(대상 규모·시간 예산·메모리 상한) 확인 후 수치화한다.

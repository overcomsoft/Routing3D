# 동결 합의 체크리스트 (Freeze Sign-off) — Routing3D Phase 2

> [phase2_plan.md](../phase2_plan.md) Step 2.5 · 버전 1.0 · 2026-05-28
> 목적: Phase 2 산출물을 **동결**하고, 이후 변경을 버전 관리 절차로만 허용한다.

---

## 1. 산출물 검토 체크리스트

| 산출물 | 문서 | 검토 항목 | 상태 |
|---|---|---|---|
| 알고리즘 명세서 | [algorithm_spec.md](algorithm_spec.md) | 의사코드가 레퍼런스와 일치 / 불변식 명시 / 상수 고정 | ☐ |
| 데이터 포맷 규격서 | [scene_format_spec.md](scene_format_spec.md) | 문법·필드·`\N`·무손실·버전정책 | ☐ |
| 회귀 골든셋 | [regression_set.md](regression_set.md) | 3종 입력/기대지표 + Phase 3 합격 기준 | ☐ |
| 성능 목표 정의 | [performance_targets.md](performance_targets.md) | 규모별 시간·메모리 수치(사용자 입력 반영) | ☐ |

> 각 항목을 검토 후 ☑ 로 표시하고, 아래 동결 합의란에 날짜·검토자를 기록한다.

---

## 2. 동결 대상 인터페이스 요약 (변경 시 영향 큼)

- 좌표·단위 규약(mm, `cell_mm`, 셀↔월드, 격자밖=점유).
- `OccupancyMap` 질의 인터페이스 + 3 백엔드 동등성.
- `RouteParams` 필드/기본값 + admissibility 계약(가산항 ≥ 0).
- A* / weighted A* / 순차 라우팅 의사코드 및 결정성(tie-break).
- `scene.txt` v1 문법·필드·불변식.
- 회귀 골든셋 3종(입력·기대지표).

---

## 3. 변경 관리 규칙 (동결 후)

1. 동결된 인터페이스/포맷 변경은 **버전업**으로만 한다(`scene.txt @version`, 명세서 버전).
2. 알고리즘 변경은 **Phase 1 레퍼런스에서 먼저 검증** → 회귀 골든셋 재생성 → 명세 갱신 순.
3. 미검증 아이디어는 명세에 넣지 않고 **open-questions**([phase2_input_notes.md](../phase2_input_notes.md) §8)로 분리.
4. 모든 변경은 회귀 테스트(`pytest`) 재통과를 전제로 한다.

---

## 4. 동결 합의란

| 산출물 | 검토자 | 동결일 | 비고 |
|---|---|---|---|
| algorithm_spec.md | | | |
| scene_format_spec.md | | | |
| regression_set.md | | | |
| performance_targets.md | | | |

> 4개 산출물 모두 동결되면 Phase 2 완료. 다음은 Phase 3(C++ 엔진화 + pybind11).

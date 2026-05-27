# Phase 2 — 인터페이스 동결

목적: Phase 1 실험으로 확정된 알고리즘·제약·데이터 포맷을 명세로 고정해 C++ 포팅의 설계 변경 리스크를 제거한다.

상세 계획: [../docs/development_plan.md §3.2](../docs/development_plan.md#32-2단계--인터페이스-동결)

## 산출물 목록 (동결 시점에 작성)

| 문서 | 내용 |
|---|---|
| `algorithm_spec.md` | 비용함수·제약·우선순위 규칙 |
| `data_format_spec.md` | scene.txt 등 중간 데이터 포맷 |
| `regression_tests/` | 대표 시나리오 회귀 테스트 (입력 → 기대 경로/지표) |
| `performance_targets.md` | 셀 규모별 목표 시간·메모리 |

## 통과 기준

- 알고리즘 명세 ↔ Python 구현 일치 확인
- 회귀 테스트 세트가 Python 구현에서 100% 통과
- 성능 목표치가 수치로 정의됨 (C++ 트랙의 KPI)

# 회귀 테스트 골든셋 (Regression Golden Set, 동결) — Routing3D

> Phase 2 인터페이스 동결 · [phase2_plan.md](../phase2_plan.md) Step 2.3
> 레퍼런스: `python_experiments/tests/scenarios/`, 실행기 `tests/scenario_runner.py`, 하니스 `tests/test_scenarios.py`
> 버전 1.0 · 2026-05-28 · 단위 **mm**, 셀 50mm

---

## 1. 목적

대표 시나리오를 **입력 → 기대지표(허용범위)** 로 고정한다. A* 가 결정적(동일 입력→동일 경로)이므로 길이/회전/충돌/총길이를 기준값으로 회귀를 검출한다. **Phase 3 C++ 엔진은 동일 시나리오에서 이 지표를 허용범위 내 재현**해야 한다(합격 기준).

---

## 2. 골든 시나리오 3종

각 시나리오 = `scenarios/<name>/input.json` + `expected_metrics.json`. 모든 시나리오는 베이스라인 파라미터(`cell_mm=50, w_turn=500, w_clear=10, clearance_radius=2, connectivity=6, w_tier={}`)를 사용한다.

### 2.1 `01_single_empty` — 빈 공간 단일 배관

| 항목 | 값 |
|---|---|
| 격자 | 20×20×20 (1000×1000×1000 mm), 장애물 없음 |
| 작업 | start (25,25,25) → end (975,975,975) mm (cell (0,0,0)→(19,19,19)) |
| 검증 의도 | 직교 최단 길이 = 맨해튼 거리, 회전 최소 |

기대지표(`checks`):
- `success = true`
- `length_mm = 2850.0` (정확) — 맨해튼 57셀×50mm, 비율 1.000
- `turns = 2` (정확)
- `path_hits_obstacle = 0` (정확)
- `expanded_nodes_max = 30000` (상한; 측정 22,856)

### 2.2 `02_single_obstacle` — 장애물 우회

| 항목 | 값 |
|---|---|
| 격자 | 80×80×80 (4000³ mm) |
| 장애물 | 벽 `[1900,0,0]~[2150,2250,4000]` (x 38~42셀 전높이, y 0~44셀) |
| 작업 | start (275,2025,2025) → end (3725,2025,2025) mm |
| 검증 의도 | 우회 길이가 직선의 ±20% 이내, 장애물 비통과 |

기대지표:
- `success = true`
- `length_mm = 3950.0` (정확) — 직선 3450mm × **1.145 (±20% 이내)**
- `turns = 2`, `path_hits_obstacle = 0`
- `detour_ratio_max = 1.2` (length ≤ 맨해튼×1.2)
- `expanded_nodes_max = 12000` (측정 9,036)

### 2.3 `03_multi_tier` — 다중 배관(5개) 충돌 회피

| 항목 | 값 |
|---|---|
| 격자 | 120×120×60 (6000×6000×3000 mm) |
| 장애물 | 바닥 슬래브 `[0,0,0]~[6000,6000,250]` (z 0~4셀) |
| 작업 | 동일 통로(y=60,z=30) 5개 배관, priority=longest |
| 검증 의도 | 모두 성공 + 성공 경로끼리 셀 공유 0(충돌 없음) |

기대지표:
- `success_count = 5`, `fail_count = 0`, `success_rate = 1.0`
- `collisions = 0` (성공 경로 쌍별 셀 공유 수)
- `total_length_mm = 28050.0` (정확)

---

## 3. `checks` 키 의미 (하니스 해석)

| 키 | 비교 방식 |
|---|---|
| success / turns / path_hits_obstacle / collisions / *_count | 정확히 일치 |
| length_mm / total_length_mm / success_rate | 근사 일치(부동소수점) |
| detour_ratio_max | `length_mm ≤ 맨해튼거리 × 값` |
| expanded_nodes_max | `expanded_nodes ≤ 값` (탐색량 회귀 상한) |

기대지표 재생성: `python python_experiments/tests/scenario_runner.py` (전 시나리오 지표 출력).

---

## 4. 실데이터 참조 (비골든, 참고)

- SpaceAI **project 6** (장애물 983 · 작업 208, cell 100mm, priority longest): **203/208 성공(98%)**, 충돌 0, 총 3,677,400 mm. 혼잡 출발부 경합으로 5건 실패(rip-up/CBS 는 Phase 3).
- 내보내기/왕복: `python -m routing3d_py.scene_io --project 6 --cell-mm 100 --multi --out <path>` → `--in <path> --roundtrip` = OK.

---

## 5. Phase 3 합격 기준 (C++ 교차검증)

1. C++ 엔진이 3종 시나리오를 동일 입력으로 실행해 **위 기대지표를 허용범위 내 재현**한다.
   - 정확 일치 항목(success/turns/collisions/counts): 완전 일치.
   - 길이/총길이: 부동소수점 허용오차 내 일치.
   - expanded_nodes: 동일 tie-break 구현 시 일치, 자료구조 차이가 있으면 상한 이내.
2. `scene.txt` 입출력이 v1 규격([scene_format_spec.md](scene_format_spec.md))과 일치하고 무손실.
3. 불변식([algorithm_spec.md](algorithm_spec.md) §6) 단위 테스트 통과.

---

## 6. 동결 표기

- 입력/기대 파일은 `python_experiments/tests/scenarios/` 에 동결. 변경 시 [freeze_signoff.md](freeze_signoff.md) 절차.
- (선택) scene.txt 골든 파일을 `python_experiments/tests/golden/` 에 추가해 포맷 회귀까지 고정 가능.

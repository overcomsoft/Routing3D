# Phase 1 — Python 알고리즘 실험

목적: 성능 부담 없이 알고리즘과 파라미터를 빠르게 반복 검증한다. 핵심 지표는 **반복 속도**.

상세 계획: [../docs/development_plan.md §3.1](../docs/development_plan.md#31-1단계--python-알고리즘-실험)

## 권장 구조 (구현 시)

```
python_experiments/
├── routing/           # 알고리즘 패키지 (occupancy / astar / cost / multi-pipe)
├── scenes/            # scene.txt 픽스처 (입력 시나리오)
├── notebooks/         # 실험 노트북 (파라미터 튜닝)
├── viz/               # PyVista / Plotly 시각화 헬퍼
└── tests/             # 알고리즘 회귀 테스트 (pytest)
```

## 의존성 설치

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r ../requirements.txt
```

## 산출물 목표

- 검증된 알고리즘 파라미터 셋
- 실험 노트
- scene.txt 호환 입출력
- 비교 기준 (baseline) 데이터

# Routing3D

플랜트 배관 3D 직교 자동 라우팅 엔진 — Python 알고리즘 실험 → C++ 엔진화 (pybind11 연계).

상세 계획은 [docs/development_plan.md](docs/development_plan.md) 참조 (원본 PDF: `docs/development_plan.pdf`).

## 개발 트랙

| 단계 | 위치 | 목적 |
|---|---|---|
| **Phase 1** — Python 실험 | [python_experiments/](python_experiments/) | NumPy + heapq 기반 직교 A* 빠른 반복 검증 |
| **Phase 2** — 인터페이스 동결 | [interface_spec/](interface_spec/) | 알고리즘 명세 + 데이터 포맷 + 회귀 테스트 동결 |
| **Phase 3** — C++ 엔진화 | [cpp_engine/](cpp_engine/) + [bindings/](bindings/) | OpenVDB / Boost.Heap / FCL + pybind11 바인딩 |

## 폴더 구조

```
Routing3D/
├── README.md
├── .gitignore
├── requirements.txt              # Phase 1 Python 의존성
├── docs/
│   ├── development_plan.md       # 마스터 개발 계획서
│   └── development_plan.pdf      # 원본 PDF
├── python_experiments/           # Phase 1
├── interface_spec/               # Phase 2
├── cpp_engine/                   # Phase 3
└── bindings/                     # Phase 3 — pybind11
```

## 현재 상태

- 2026-05-27: 프로젝트 스켈레톤 생성. 다음 즉시 실행 항목 = Phase 1 Python 실험 환경 구축 (C++ A* 로직 미러링 + PyVista 시각화).

## 인접 프로젝트

- `..\SpaceAI\` — 선행 프로젝트. C# 자동 설계 엔진 + WPF 뷰어 + Hybrid v2 라우팅. Routing3D 는 알고리즘 측면에서 SpaceAI 의 직교 A* 와 동일 계열이나, Python 실험 → C++ 엔진화 트랙으로 새로 시작.

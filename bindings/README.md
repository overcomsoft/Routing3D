# Phase 3 — Python 바인딩 (pybind11)

목적: [../cpp_engine/](../cpp_engine/) 의 C++ 엔진을 Python 에서 호출할 수 있도록 pybind11 바인딩을 제공한다.

상세 계획: [../docs/development_plan.md §3.3](../docs/development_plan.md#33-3단계--c-엔진화--pybind11)

## 역할

- Python 실험 환경 ([../python_experiments/](../python_experiments/)) 이 C++ 엔진을 회귀 테스트·튜닝 도구로 영구 활용
- 동일 시나리오 → Python 구현 vs C++ 구현 결과 교차 검증
- 비용함수 파라미터 튜닝은 Python 에서, 실행은 C++ 에서

## 권장 구조 (구현 시)

```
bindings/
├── CMakeLists.txt
├── src/                # pybind11 모듈 소스
├── routing3d/          # Python 패키지 (.pyi stubs 포함)
└── tests/              # 바인딩 호출 검증
```

## 통과 기준

- Python 에서 `import routing3d` 로 엔진 호출 가능
- 회귀 테스트가 Python 구현 / C++ 엔진 양쪽에서 동일 결과

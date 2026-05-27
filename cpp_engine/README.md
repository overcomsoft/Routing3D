# Phase 3 — C++ 엔진

목적: Phase 2 에서 동결된 알고리즘을 프로덕션 수준의 C++ 엔진으로 구현한다.

상세 계획: [../docs/development_plan.md §3.3](../docs/development_plan.md#33-3단계--c-엔진화--pybind11)

## 권장 구조 (구현 시)

```
cpp_engine/
├── CMakeLists.txt
├── include/routing3d/     # public 헤더
├── src/                   # 엔진 구현
├── tests/                 # C++ 단위 테스트 (gtest 등)
└── benchmark/             # 성능 벤치마크
```

## 핵심 라이브러리

| 영역 | 라이브러리 |
|---|---|
| 점유맵 | OpenVDB |
| 탐색 자료구조 | Boost.Heap |
| 방문/closed 맵 | abseil flat_hash_map |
| 충돌검사 | FCL |
| 다중 배관 조정 | 직접 구현 (rip-up & reroute / CBS) |

Python 바인딩은 [../bindings/](../bindings/) 참조.

## 통과 기준

- Phase 2 에서 정의된 회귀 테스트 세트가 C++ 구현에서 통과
- 성능 목표치 (8,000m 스케일) 달성
- Python 구현 결과와의 교차 검증 통과

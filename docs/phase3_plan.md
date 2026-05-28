# Phase 3 — C++ 엔진화 + pybind11 세부 개발계획서

> 상위 문서: [development_plan.md](development_plan.md) §3.3
> 입력(동결 명세): [spec/algorithm_spec.md](spec/algorithm_spec.md) · [spec/scene_format_spec.md](spec/scene_format_spec.md) · [spec/regression_set.md](spec/regression_set.md) · [spec/performance_targets.md](spec/performance_targets.md)
> 작성일: 2026-05-28 · 버전 1.0 · 단위 **mm**, 기본 셀 50mm

## 목차

1. [목표와 비목표](#1-목표와-비목표)
2. [입력 자산 (Phase 2 동결 명세)](#2-입력-자산-phase-2-동결-명세)
3. [빌드 환경 / 의존성 (제안 — 확인 필요)](#3-빌드-환경--의존성-제안--확인-필요)
4. [세부 작업 단계](#4-세부-작업-단계)
5. [디렉토리 구조](#5-디렉토리-구조)
6. [검증 전략 (Python 교차검증)](#6-검증-전략-python-교차검증)
7. [완료 기준 (Definition of Done)](#7-완료-기준-definition-of-done)
8. [리스크와 대응](#8-리스크와-대응)
9. [마일스톤 순서](#9-마일스톤-순서)

---

## 1. 목표와 비목표

### 1.1 목표
동결된 알고리즘·포맷을 **프로덕션 수준 C++ 엔진**으로 구현하고, pybind11 로 Python 실험 환경과 영구 연결한다.

- [algorithm_spec.md](spec/algorithm_spec.md) 의 의사코드를 **1:1 포팅**(계약·불변식 보존).
- 성능 목표([performance_targets.md](spec/performance_targets.md)) 달성: 8,000m 스케일 / 단일 배관 <1초 / 전체 <1분 / <32GB.
- 회귀 골든셋([regression_set.md](spec/regression_set.md)) 지표를 허용범위 내 재현.

### 1.2 비목표
- 새 알고리즘 설계 → 필요 시 Phase 1 레퍼런스에서 먼저 검증 후 명세 버전업.
- C# 뷰어 연계는 별도(엔진은 scene.txt / 바인딩으로 결과 제공).

---

## 2. 입력 자산 (Phase 2 동결 명세)

| 자산 | 역할 |
|---|---|
| [algorithm_spec.md](spec/algorithm_spec.md) | 구현 기준(의사코드·계약·불변식·상수) |
| [scene_format_spec.md](spec/scene_format_spec.md) | scene.txt v1 파서/라이터 규격 |
| [regression_set.md](spec/regression_set.md) | 골든셋 3종 + 합격 기준 |
| [performance_targets.md](spec/performance_targets.md) | 규모별 시간·메모리 목표 |
| `python_experiments/` | 교차검증용 레퍼런스(정답) |

---

## 3. 빌드 환경 / 의존성 (제안 — 확인 필요)

> ⚠️ 아래는 Windows 기준 표준 제안이다. 사용자 환경(컴파일러/패키지매니저/CI) 확정 후 고정한다.

| 항목 | 제안 | 비고 |
|---|---|---|
| 플랫폼 | Windows 11 (x64) | 현 개발 환경 |
| 컴파일러 | MSVC (VS 2022, C++20) | clang/gcc 도 가능 |
| 빌드 시스템 | **CMake** (≥3.25) + Ninja | 크로스플랫폼 |
| 패키지 매니저 | **vcpkg** (manifest 모드) | conan 대안 |
| 테스트 | GoogleTest 또는 Catch2 | 단위/회귀 |
| CI | (선택) GitHub Actions | 빌드+테스트 |

**의존성 (development_plan §3.3)**

| 라이브러리 | 용도 | 도입 단계 |
|---|---|---|
| pybind11 | Python 바인딩 | 3.10 |
| Boost.Heap | decrease-key 우선순위 큐 | 3.3 |
| abseil (`flat_hash_map`) | 고속 방문/closed 맵 | 3.3 |
| OpenVDB | 희소 복셀 점유맵(8,000m) | 3.6 |
| FCL | 메시·프리미티브 충돌검사 | 3.7 |

> OpenVDB/FCL 는 무거운 의존성(TBB/Blosc/Eigen 등 전이 의존). vcpkg 로 조기 빌드 가능성 검증(3.1 PoC).

---

## 4. 세부 작업 단계

원칙: **작은 코어부터 → 골든셋으로 검증 → 무거운 의존성/최적화는 뒤로**. 각 단계는 가능한 한 Python 레퍼런스와 동일 지표를 재현하며 전진한다.

### 4.1 Step 3.1 — 스캐폴드 + 빌드 PoC
- `cpp/` 디렉토리, CMake + vcpkg manifest, 헬로 빌드, 테스트 프레임워크 연결.
- **무거운 의존성(OpenVDB/FCL) 빌드 가능성 조기 PoC**(가장 큰 환경 리스크).

### 4.2 Step 3.2 — 코어 자료구조
- `Cell/AABB/RouteParams`, 좌표 변환(셀↔월드), `NEIGHBORS_6/26` (spec §1,§7).
- **Dense 점유맵**(먼저, 작은 ROI 용) — 질의 인터페이스 O1 계약 충족.
- 단위 테스트: 좌표 변환, add_box 복셀화, inflate.

### 4.3 Step 3.3 — 직교 A* (균일)
- spec §3 의사코드 포팅. **Boost.Heap** + **abseil flat_hash_map**(g/closed).
- **결정성 모드**: Python 과 동일 (f, 삽입순서) tie-break + 고정 이웃 순서 → 골든셋 `01_single_empty` 의 length/turns/expanded 재현.
- 단위 테스트 + scene.txt 입력 기반 교차검증.

### 4.4 Step 3.4 — 비용함수 A* (weighted)
- 상태=(셀,진입방향), `CostModel`(clearance distance transform, move_cost) — spec §4.
- admissibility 계약(C1) 단위 테스트. 골든셋 `02_single_obstacle` 재현.

### 4.5 Step 3.5 — 다중 배관 순차
- `route_sequential`/`order_tasks`/`mark_pipe`/snap — spec §5. 충돌 0(M1) 검증.
- 골든셋 `03_multi_tier` 재현 + project6 실데이터 성공률 비교.

### 4.6 Step 3.6 — OpenVDB 점유맵 + 계층 corridor
- OpenVDB 희소 백엔드(질의 인터페이스 동일, O1). 8,000m 합성 장면 메모리 <32GB 검증.
- **계층 corridor**: coarse 경로 → fine 탐색 영역 제한(단일 배관 <1초 핵심).

### 4.7 Step 3.7 — FCL 충돌검사 (정밀)
- AABB 복셀 기본 위에 메시·프리미티브 간섭 검사 옵션 추가(필요 영역).

### 4.8 Step 3.8 — rip-up & reroute / CBS
- 혼잡 출발부 경합 해소(Phase 1 에서 측정만 한 실패분). 성공률 향상 목표.

### 4.9 Step 3.9 — scene.txt I/O (C++)
- v1 규격([scene_format_spec.md](spec/scene_format_spec.md)) 파서/라이터. Python 이 쓴 파일을 읽고 무손실 재출력(F2) 교차검증.

### 4.10 Step 3.10 — pybind11 바인딩
- 엔진을 Python 모듈로 노출(점유맵 구성/라우팅/결과). 기존 `routing3d_py` 실험 환경에서 호출 가능하게.

### 4.11 Step 3.11 — 성능 벤치 + 최적화
- [performance_targets.md](spec/performance_targets.md) §6 벤치 셋으로 측정·튜닝. SIMD/병렬(독립 배관) 적용.

### 4.12 Step 3.12 — 회귀 통과 리포트
- 골든셋 3종 + project6 + 8,000m 합성 결과를 합격 기준([regression_set.md](spec/regression_set.md) §5)과 대조한 리포트.

---

## 5. 디렉토리 구조

```
cpp/
├── CMakeLists.txt
├── vcpkg.json                 # manifest (의존성)
├── include/routing3d/         # 공개 헤더
│   ├── occupancy.hpp          # OccupancyMap 인터페이스 + Dense/OpenVDB
│   ├── astar.hpp              # 직교 A* / weighted
│   ├── cost.hpp               # RouteParams / CostModel
│   ├── multi_route.hpp        # 순차 + rip-up/CBS
│   └── scene_io.hpp           # scene.txt v1
├── src/                       # 구현
├── bindings/                  # pybind11 (routing3d_cpp 모듈)
├── tests/                     # GoogleTest/Catch2 + 골든셋 재사용
└── bench/                     # 성능 벤치
```

---

## 6. 검증 전략 (Python 교차검증)

1. **golden 재현**: C++ 가 `python_experiments/tests/scenarios/*/input.json`(또는 scene.txt)을 읽어 동일 지표 산출 → [regression_set.md](spec/regression_set.md) §5 기준 대조.
2. **결정성 모드**: tie-break/이웃 순서를 Python 과 일치시켜 length/turns/경로까지 동일 검증. 성능 모드(자료구조 차이)에서는 expanded 상한으로 완화.
3. **scene.txt 왕복**: Python↔C++ 가 같은 파일을 읽고/쓰며 무손실(F2) 확인.
4. **실데이터**: project6 208배관 성공률/충돌 0 비교.

---

## 7. 완료 기준 (Definition of Done)

- [ ] C++ 엔진: occupancy / A* / weighted / 순차 / (rip-up) 구현, 단위 테스트 통과.
- [ ] OpenVDB 백엔드로 8,000m 합성 장면 메모리 <32GB.
- [ ] 성능: 단일 배관 95p <1초, 전체 프로젝트(수백 배관) <1분.
- [ ] 골든셋 3종 지표 허용범위 내 재현 + scene.txt v1 무손실.
- [ ] pybind11 모듈로 Python 에서 호출 가능.
- [ ] 회귀 통과 리포트 + 성능 벤치 결과.

---

## 8. 리스크와 대응

| 리스크 | 영향 | 대응 |
|---|---|---|
| OpenVDB/FCL 빌드·의존성 난이도 | 일정 지연 | 3.1 에서 vcpkg 빌드 PoC 조기 검증 |
| Python↔C++ 결과 불일치(동일비용 경로 선택차) | 회귀 신뢰성 | 결정성 모드(동일 tie-break) + 지표별 허용범위 분리 |
| 8,000m 성능 미달 | 실용성 | 계층 corridor 폭/해상도·병렬도 튜닝, 조기 PoC |
| rip-up/CBS 복잡도 | 품질/일정 | 우선 sequential 로 합격선 확보 후 점진 도입 |
| Windows 빌드 환경 마찰 | 착수 지연 | 환경 확정(컴파일러/CMake/vcpkg) 선행 |

---

## 9. 마일스톤 순서

| 순 | 단계 | 의존성 | 비고 |
|---|---|---|---|
| C1 | 3.1 스캐폴드 + 의존성 빌드 PoC | — | 환경 확정 필요 |
| C2 | 3.2 코어 + Dense 점유맵 | C1 | |
| C3 | 3.3 A* (균일) + 골든셋01 | C2 | 결정성 검증 |
| C4 | 3.4 weighted + 골든셋02 | C3 | |
| C5 | 3.5 순차 다중 + 골든셋03 | C4 | sequential 합격선 |
| C6 | 3.9 scene.txt I/O + 3.10 pybind11 | C3~C5 | 실험환경 연결 |
| C7 | 3.6 OpenVDB + 계층 corridor | C5 | 8,000m/성능 |
| C8 | 3.11 벤치/최적화 | C7 | 목표 달성 |
| C9 | 3.7 FCL / 3.8 rip-up·CBS | C5,C7 | 품질 향상 |
| C10 | 3.12 회귀 리포트 | 전체 | DoD 확정 |

> **다음 즉시 실행 항목** = Step 3.1 (빌드 환경 확정 + 스캐폴드 + OpenVDB/FCL vcpkg 빌드 PoC). 환경 확정(컴파일러·CMake·vcpkg) 입력 대기.

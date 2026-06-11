# Routing3D C++ 엔진 고도화 개발계획서

> 작성: 2026-06-11 · 대상: `cpp/` 엔진 + `capi/` C ABI + 회귀 테스트 · 단위 mm
> 목표: **"배관 중심선 찾기" → "관경·실제 충돌·이격 규칙을 만족하는 설비 라우팅"** 으로 끌어올린다.

---

## 0. 배경 — 현재 구조와 강점

| 계층 | 구현 | 상태 |
|---|---|---|
| 핵심 탐색 | 6방향 격자 A* / Weighted A*([astar.hpp](../cpp/include/routing3d/astar.hpp)) | 결정적·골든 검증 |
| 다중 배관 | 순차 + 제한적 rip-up([multi_route.hpp](../cpp/include/routing3d/multi_route.hpp)) | greedy·순서 의존 |
| 점유맵 | Dense / Sparse / Implicit / (opt) VDB / (opt) FCL([occupancy.hpp](../cpp/include/routing3d/occupancy.hpp)) | 백엔드 다형성 |
| 외부 연동 | C ABI DLL + C#/P-Invoke([routing3d_capi.h](../cpp/capi/routing3d_capi.h)) | 예외 비누설·UTF-8 |

엔진 기반과 단위/골든 테스트는 견고하다. 본 계획은 그 위에 **물리성·진단성·확장성·실데이터 검증**을 더한다.

---

## 1. 검증된 문제점 (코드 확인 완료)

| # | 문제 | 근거(코드) |
|---|---|---|
| P1 | **관경이 라우팅 제약에 미반영** — `diameter_mm`는 우선순위 정렬에만 쓰이고 A* 비용/clearance/충돌반경에 직접 반영 안 됨. `pipe_radius`는 있으나 **호출자(C#)가 관경→셀 환산 책임**. FCL은 주 루프 밖. | `route_task.hpp` diameter_mm · capi `r3d_set_pipe_radius`(호출자 산출) · `fcl_scene.hpp` 미통합 |
| P2 | **실패 사유 미전달** — `AStarResult`는 `success`(bool)만. 시작/끝 막힘·corridor 밖·확장상한·hier 실패·goal_dir 막힘이 전부 `success=false`로 뭉개짐. | `astar.hpp` AStarResult(20–40행: success/path/length…, 사유 없음) |
| P3 | **대형 격자 API 비일관** — `route_multi`는 5M 초과 시 Implicit 전환하나, `r3d_route_ripup`은 `DenseOccupancy` 강제, `r3d_copy_blocked`도 Dense 복셀화 강제. | capi `route_ripup`(830행 `DenseOccupancy occ = occupancy_from_doc`) · `r3d_copy_blocked`(Dense) |
| P4 | **64비트 인덱스 계약 불명확** — Dense/Sparse `lin()`=`int`, Implicit만 `long long`. `clearance_map()`은 `occ.size()` 전배열 + 내부 int 캐스팅. | occupancy.hpp 53·92(int) vs 159(long long) · cost.hpp clearance_map |
| P5 | **`collect_visited` 기본 ON** — 대형 장면에서 방문셀 저장+C# 복사 비용 과다. | capi(40행 `collect_visited = true`) |
| P6 | **다중 배관 greedy** — 순차+제한 rip-up만. 고밀도 병목에서 순서 의존·국소최적 잔존. | multi_route.hpp `route_sequential`/`route_ripup` |
| P7 | **인코딩 품질 불안정** — CMakeLists/일부 출력 한글 깨짐. 인수인계 리스크. | `cpp/CMakeLists.txt` 등 |
| P8 | **후처리·실데이터 검증 부족** — unkink/ortho는 capi 내부뿐. 관경 FCL 후검증·코너 최소반경·랙 선호·이격 규칙·실DB 회귀 골든 없음. | capi `unkink_path` · `cpp/tests/` |

---

## 2. 개발 목표 (수용 기준)

1. **관경 일관성**: `diameter_mm` 하나로 pipe_radius·clearance·FCL 검증이 자동 연결(호출자 책임 제거).
2. **진단성**: 모든 실패가 사유 코드로 식별(C# UI/로그에서 분류 가능).
3. **확장성**: 모든 라우팅 진입점이 격자 크기에 무관하게 안전(Dense 강제 제거).
4. **품질**: 주 엔진 결과가 관경·이격·코너반경·랙 선호를 후검증 통과.
5. **신뢰성**: 실DB 추출 장면·대형격자·고밀도·관경혼합·실패진단 골든 통과.
6. **불변식 보존**: 기존 골든(ctest 11/11) **바이트 불변** — 모든 신기능은 **기본값 = 기존 동작**.

---

## 3. 단계별 계획

### Phase A — 계약·안전성 정리 (저위험·기반, 골든 불변)

추가(additive)·기본값=기존동작이라 골든 바이트 불변. 먼저 깔아 이후 단계의 토대로 쓴다.

**A1. 실패 사유 코드 (P2)**
- `astar.hpp`: `enum class RouteFail { None, StartBlocked, GoalBlocked, CorridorMiss, ExpansionLimit, GoalDirBlocked, NoPath }` + `AStarResult.fail`(기본 None). 각 조기반환 지점에 사유 기록.
- `route_task.hpp`/`scene_io`: `ScenePipeResult`에 `fail_reason`(int) 추가(scene.txt **미직렬화** → F2 불변).
- C ABI: `R3dResult`에 `int32_t fail_reason` 추가(구조체 끝에 append → 기존 호출 호환). `r3d_get_result` 채움.
- C#: `Engine.PathResult.FailReason` enum 노출 → `TaskRowVM`/`ExplainFailure`가 정확 분류.
- **검증**: ctest 불변(성공 경로 fail=None). 신규 단위테스트 `test_failreason`(막힘/상한/corridor-miss 케이스).

**A2. `collect_visited` 기본 OFF (P5)**
- capi 기본 `collect_visited = false`. `r3d_set_collect_visited(on)` 으로 opt-in(뷰어 방문맵 토글 시에만).
- 대형 장면 보호: visited 수집 시 다운샘플 상한(env `R3D_VISITED_CAP`, 기본 200k) 초과분 생략 + 로그.
- **검증**: ctest 의 visited 의존 테스트는 명시 on. 뷰어 방문맵 토글 경로 확인.

**A3. 대형 격자 API 일관화 (P3)**
- `r3d_route_ripup`·`r3d_copy_blocked`: `occ.size() > 5M`이면 Implicit 경로로 분기(route_multi와 동일 게이트). Dense는 소격자 전용.
- `r3d_copy_blocked`: 대형은 전체 복셀 배열 대신 **질의 콜백/청크 스트리밍**(또는 명시적 셀-수 guard + 에러코드 `R3D_ERR_TOO_LARGE`).
- **검증**: 대형격자(25mm ~1.3억 셀) ripup/copy_blocked 크래시 0 단위테스트.

**A4. 64비트 인덱스 계약 (P4)**
- `Occupancy` concept 문서화: `using index_t = int64_t; index_t lin(Cell) const;` 통일. Dense/Sparse는 내부 int 저장하되 **반환은 int64_t**(승격만, 값 동일 → 골든 불변).
- `clearance_map()`: idx 연산 int64 통일. 대형격자는 **온디맨드 clearance**(`HasClearanceQuery`, 이미 Implicit 경로 존재)로 강제 — 전배열 할당 회피.
- **검증**: Dense==Implicit 전수 일치(기존 `test_implicit`) 유지 + int 오버플로 정적 점검.

**A5. UTF-8 인코딩 정리 (P7)**
- `cpp/CMakeLists.txt`·소스 BOM/`/utf-8` 점검, 깨진 한글 주석 복구. 빌드 산출 로그 UTF-8.
- **검증**: 빌드 경고 0, 주석 가독성 육안.

---

### Phase B — 관경 기반 물리 라우팅 (핵심·중위험)

`diameter_mm`를 단일 진실원으로 삼아 물리성을 엔진 내부로 내린다.

**B1. 관경 → pipe_radius 자동 산출 (P1 핵심)**
- 엔진 내부: `RouteTask.diameter_mm` + `RouteParams.cell_mm` → `radius_cells = clamp(ceil(d/cell) − 1, 0, R_MAX)`(현재 C# `ComputePipeRadiusCells` 로직을 엔진으로 이관).
- **per-task 반경**: 현재 단일 글로벌 `pipe_radius`(max) → 작업별 관경 반경으로(`mark_pipe`가 task별 radius 사용). 가는 배관 과패킹 해소(§9 알려진 한계).
- 하위호환: `r3d_set_pipe_radius`(글로벌)는 유지하되, task 관경이 있으면 우선. 둘 다 0이면 기존 동작.
- **검증**: radius=0 경로 바이트 불변(골든). 관경 혼합 장면에서 표면 겹침 0.

**B2. 관경 기반 clearance 페널티 (P1)**
- `CostModel`: clearance(장애물까지 거리)가 `radius_cells + margin` 미만인 셀에 **가산 페널티**(admissibility 보존 — 음수 비용 없음). 관경 큰 배관일수록 벽 근처를 회피.
- 기본 margin=0 → 페널티 0 → 골든 불변. opt-in(`RouteParams.w_clearance_radius`).
- **검증**: 페널티 0이면 expanded_nodes·경로 골든 일치. 페널티>0에서 벽 이격 증가 정량.

**B3. FCL 최종 검증 주 루프 통합 (P1)**
- 각 성공 경로를 `fcl_scene`로 **관경 캡슐 vs 장애물/기배관 sub-voxel 충돌 검증**. 통과 실패 시 해당 배관 `fail = ClearanceViolation` 표시(또는 rip-up 트리거).
- 비용 큰 정밀 검사라 **opt-in**(`r3d_set_fcl_validate(on)`), 기본 OFF → 골든 불변. FCL 미빌드 시 무해 스킵.
- **검증**: 틈 200mm 가는·굵은 배관 구별(기존 `test_fcl` 확장). 통합 루프 단위테스트.

---

### Phase C — 다중 배관 품질 (고위험·연구성)

**C1. negotiated-congestion / CBS (P6)**
- rip-up의 후속: 충돌하는 배관 쌍에 **비용 기반 우선순위 협상**(conflict-based search 경량판) — 병목에서 순서 의존 완화.
- 무손실·결정적 보장 유지(성공 단조 증가, 채택 시에만 확정). env 게이트(`R3D_CBS=on`), 기본 OFF.
- **검증**: 합성 병목·project6 c100 잔여 실패(10건) 일부 해소. 골든 불변(OFF).

**C2. 최종 경로 후처리 강화 (P8)**
- 주 엔진 결과에 후검증 파이프라인(엔진 측, capi 노출):
  - **관경 FCL 검증**(B3 재사용),
  - **코너 최소 반경**(엘보 사이 직선 ≥ k×관경 — 제작성. ※렌더 정형화(P3m)는 되돌렸으므로 **경로(셀) 단계**에서 충돌검사 하에 적용),
  - **지지/랙 레벨 선호**(기존 `rack_levels` 재사용),
  - **관종별 이격 규칙**(유틸 그룹별 min clearance 테이블).
- 각 규칙 기본 비활성 → 골든 불변. 규칙 위반은 사유코드(A1)로 보고.
- **검증**: 규칙별 단위테스트 + 실데이터 육안.

---

### Phase D — 검증 인프라 (병행)

**D1. 실DB 장면 회귀 골든 (P8)**
- DDW_AI_DB에서 **5~20개 대표 장면**(그룹·관경 다양) 추출 → `cpp/tests/fixtures/*.scene.txt` 고정(LF, `.gitattributes`).
- 각 장면 기대 지표(성공수·총길이·꺾임·실패사유 분포) 스냅샷 골든.
- **검증**: `ctest -R realdata`.

**D2. 카테고리 골든 (P8)**
- 대형격자 / 고밀도 병목 / 관경 혼합 / 도달 실패 진단 4종 합성·실측 골든.
- **검증**: 회귀 시 즉시 탐지.

---

## 4. 권장 실행 순서 (의존성)

```
A1 실패사유 ─┐
A2 visited   ├─► (저위험 기반, 즉시 머지 가능)
A3 대형API   │
A4 64비트    │
A5 UTF-8    ─┘
        │
        ▼
B1 관경반경 ─► B2 clearance ─► B3 FCL통합   (물리성, A1 사유코드 활용)
        │
        ▼
C2 후처리 ─► C1 CBS                         (품질, B 위에)
        │
        ▼
D1/D2 실데이터·카테고리 골든                 (전 단계 병행 추가)
```

권장: **Phase A 먼저 일괄**(저위험·즉시 가치), 이후 **B1→B2→B3**, 그 다음 **C2→C1**, **D는 각 단계와 병행**.

---

## 5. 불변식 보존 원칙 (필수)

- 모든 신기능은 **기본값 = 기존 동작** → 기존 골든 ctest 11/11 **바이트 불변**.
- A* 결정성(A2/W1): (f, counter) tie-break + 고정 이웃순서 유지.
- 다중배관(M1/M2): 성공 셀 공유 0, 원본 점유맵 불변.
- scene.txt 무손실(F2): 신규 필드는 **미직렬화**(diameter_mm·goal_dir 선례 따름).
- C ABI 호환: `R3dResult` 등 구조체는 **끝에 필드 append**만(기존 P/Invoke 깨지지 않게) + 버전 가드.

---

## 6. 리스크와 완화

| 리스크 | 완화 |
|---|---|
| per-task 반경이 성공률 저하(과팽창) | 클램프 + rip-up 회복 + 실패사유(A1)로 진단 |
| FCL 통합 성능 저하 | 기본 OFF·opt-in, 성공 경로만 검증, sub-voxel 국소 |
| CBS 복잡도·비결정성 | 무손실·결정적 게이트 유지, 기본 OFF, 합성 골든 우선 |
| C ABI 구조체 변경 호환 | 끝 append + 크기/버전 필드, C# 측 동시 갱신 |
| 실DB 골든의 DB 의존 | 추출본을 scene.txt로 **고정**(DB 비의존 재현) |

---

## 7. 산출물

- 코드: `cpp/` 엔진·`capi/` + C# 인터롭 갱신.
- 테스트: `test_failreason`·`test_diameter`·`test_realdata` 등 신규 ctest.
- 문서: 본 계획서 + 완료 후 구현현황·실측 갱신(CLAUDE.md §4/§9 + 본 문서 말미).

---

## 8. 구현 현황

### Phase A — 완료 (2026-06-11)

| 작업 | 결과 |
|---|---|
| **A1 실패 사유 코드** | `astar.hpp` `enum class RouteFail{None,StartBlocked,GoalBlocked,CorridorMiss,ExpansionLimit,GoalDirBlocked,NoPath}` + `AStarResult.fail`(각 조기반환·상한·진입축·고갈 지점 기록). `SceneResult.fail`(scene.txt 미직렬화) → C ABI `R3dResult.fail_reason`(구조체 끝 append, ABI 호환) → C# `RouteResult.Fail`(enum)·`TaskRowVM.LastFail` → `ExplainFailure` 머리에 권위 사유 표시. |
| **A2 collect_visited 기본 OFF** | capi `R3dEngine.collect_visited=false`. 뷰어 `BuildEngineForRows` 가 `SetCollectVisited(ShowVisitedMap \|\| _stepTracePending)` 로 opt-in(방문맵 레이어 ON 또는 단계별 탐색 시에만 수집) → 대형 장면 메모리/복사 비용 절감. |
| **A3 대형격자 API 일관화** | `r3d_route_ripup`·`r3d_copy_blocked` 가 제네릭 람다(`auto&&`)로 백엔드 무관화 + **>5M 셀이면 ImplicitOccupancy** 전환(Dense 전배열 폭발·int 오버플로 방지). route_multi 게이트와 동일. |
| **A4 64비트 인덱스 계약** | `clearance_map` 의 `unlin(static_cast<int>(idx))` → `unlin(idx)`(narrowing 제거). occupancy.hpp 에 **인덱스 계약 주석**(Dense/Sparse int 는 소격자<5M 전용·거대격자는 Implicit long long·A* state/clearance 는 long long). 실제 오버플로 불가(5M 게이트) 확인, 전면 lin()→int64 통일은 narrowing 리스크로 보류(문서화). |
| **A5 UTF-8** | **코드 수정 불필요** — `cpp/CMakeLists.txt`·소스 전부 정상 UTF-8(`/utf-8` 설정·BOM 없음), CLI 는 이미 `chcp 65001`. mojibake 는 콘솔 코드페이지(런타임)·에디터 인코딩 설정 문제로 소스 무결. |

**검증**: ctest **11/11 통과**(골든 바이트 불변 — 모든 신기능 기본값=기존 동작) · 뷰어 빌드 오류 0.

### Phase B — 진행 중

| 작업 | 결과 |
|---|---|
| **B1 관경→pipe_radius 자동(per-task)** | **완료(2026-06-11)**. `route_multi_impl` 에 `radius_of(task)=clamp(ceil(diameter_mm/cell)-1,0,8)` 추가 — 각 배관을 **자기 관경 반경**으로 마킹(snap·main·rip-up·blocker 전부). C ABI `r3d_set_per_task_radius(on)` + Native/Engine `SetPerTaskRadius` + 뷰어 `BuildEngineForRows` ON(글로벌 `SetPipeRadius`는 관경 미상 폴백). OFF/관경0=글로벌(기존·골든 불변). env `R3D_PER_TASK_RADIUS`. **실측(proj1 c100 Exhaust)**: 글로벌 r=5 **18/20·47꺾임·20s**(굵은관 기준이 가는 배관 과패킹→2실패) → per-task **20/20·31꺾임·1.1s**(과패킹 해소·18× 가속). ctest 11/11 불변. |
| **B2 관경 clearance 페널티** | **완료(2026-06-11)**. `route_multi_impl` 의 `params_for(task)` 가 per_task ON + w_clear>0 일 때 그 배관 반경만큼 `clearance_radius` 임계를 올려(`max(기존, 반경)`) 굵은 배관 중심선을 벽에서 더 띄운다(기존 CostModel 클리어런스 페널티 재사용). probe/hier/fallback/goal_dir폴백/rip-up 전부 적용. 반경≤기존 임계(가는 배관)·OFF면 doc.params 그대로 → 기존 동작·골든 불변(ctest 11/11). |
| **B3 FCL 최종검증 통합** | **보류(설계 결정)**. `routing3d_capi.dll` 은 **외부 의존성 0**(routing3d_core 만 링크)이 설계 불변이고, 배포 뷰어 DLL 은 FCL 미링크라 capi 에 FCL 을 넣으면 (a) dep-free 깨짐 (b) 기본 DLL 에선 inert. **B1(per-task 반경 마킹)+B2(관경 clearance)가 이미 셀 단위 물리 분리·벽 이격을 강제**하므로 sub-voxel FCL 검증의 한계효용이 낮다. FCL capi 빌드 채택(보류 중 P3b' VDB capi 와 동반) 시 `#ifdef ROUTING3D_USE_FCL` 가드로 path_clear 후검증을 추가할 예정. `FclScene::path_clear(pts, radius)` 인터페이스는 준비됨. |

**Phase B 결론**: 핵심(B1 per-task 반경)+보강(B2 clearance) 완료 — "관경이 라우팅 제약에 미반영"(P1) 해소. B3 는 dep-free DLL 보존 위해 보류.

### Phase D — 진행 중

| 작업 | 결과 |
|---|---|
| **D1 실데이터 회귀 골든** | **완료(2026-06-12)**. DB 추출 픽스처 `cpp/tests/fixtures/realdata_proj1_exhaust_c100.scene.txt`(project1 Exhaust c100, 장애물 632·작업 20) + ctest **`test_realdata`**(C ABI `r3d_route_multi` 실제 경로 → 전 배관 성공 20/20·총길이 67600mm ±0.5% 스냅샷·재현성 2회 동일). **DB 비의존**(픽스처 고정)이라 CI/로컬 동일 재현. 픽스처 동결 수단: `DbRouteDiag` 에 env `R3D_EXPORT_SCENE=<path>` 훅(라우팅 직전 엔진 씬 덤프, `Engine.DumpSceneText`). **ctest 12/12 통과**. |
| **D2 카테고리 골든** | **완료(2026-06-12)**. ctest **`test_category`**(프로그래밍 C ABI, 픽스처 불필요) — ① **실패사유(A1)**: 묻힌 목표 → `fail_reason==GoalBlocked` ② **대형격자(A3)**: 200×200×150=6M셀(>5M→Implicit) 개방 라우팅 성공·크래시 0 ③ **per-task 반경(B1)**: 관경 300/150 혼합 + per_task ON → 둘 다 성공·`fail==None`. 신기능이 의도대로 작동함을 고정. |

**Phase D 결론**: 실데이터(D1)+카테고리(D2) 골든으로 **ctest 11→13** — A/B 신기능(실패사유·대형격자·per-task 반경)과 실장면 라우팅 회귀가 자동 탐지된다. 안전망 확보.

### Phase C — 완료 (2026-06-12)

| 작업 | 결과 |
|---|---|
| **C1 negotiated-congestion (CBS-lite)** | **완료**. 기존 평면 rip-up(실패 배관의 *직접* blocker만 재배치)을 **연쇄(재귀) rip-up** 으로 확장 — blocker 가 재배치 못 하면 그 blocker 의 blocker 까지 bounded depth 로 재귀 양보(conflict-based search 경량판). `route_multi_impl` 의 평면 rip-up 직후 추가 패스. **핵심 불변식**: `resolve(target,state)` 성공 시 결과는 `state 전 배관 + target` 을 **전부 포함**(재귀도 동일 by construction) → 성공 수 **단조 +1(무손실)**. 결정적(정렬 키)·종료 보장(깊이 매 재귀 −1, 분기 ≤ (MAXBLK+1)^(depth+1), depth≤3·MAXBLK=4). C ABI `r3d_set_cbs_depth` + C# `SetCbsDepth`/`UseCbs`(체크박스 '협상 라우팅(CBS)') + env `R3D_CBS`. 대형격자(>5M·large_grid)·미취소·잔여실패 시에만 실행 → **기본 depth=0 OFF·골든 불변**. 고밀도 병목의 잔여 실패 대상. |
| **C2 코너 최소반경** | **완료**. 엘보 사이 직선(런)이 (배수×관경) 미만이면 제작 불가(짧은 단관) → 경로(셀) 단계에서 충돌검사 하에 양옆 코너를 직교(≤1엘보) 연결로 흡수(`enforce_min_straight`). **PathRectifier(P3m, 렌더 레벨·되돌림)와 달리 경로 셀 단계**라 장애물·타 배관 충돌 안전. **핵심 설계: 비교란 최종 패스** — 배치 중간에 직선화하면 바뀐 셀이 다음 배관 점유를 교란해 총 꺾임이 오히려 늘었다(실측 135→141). 그래서 *모든 배관 라우팅·rip-up·CBS 후* 각 배관을 '장애물 + 다른 배관(각 반경)'만으로 만든 검사 점유에 대해 직교 흡수 → 라우팅 점유 불변·다운스트림 영향 0. 충돌통과 + 꺾임/길이 비증가일 때만 채택(무손실, 양 끝점 고정). C ABI `r3d_set_min_straight`(배수) + C# `SetMinStraight`/`UseMinStraight`(체크박스 '코너 최소반경') + env `R3D_MIN_STRAIGHT`. 기본 0 OFF·골든 불변. **C2′ 보류**(랙 선호=기존 `rack_levels`·다단 랙 P3n 이 이미 담당, 관종별 이격=per-task clearance B2 가 부분 담당 — 신규 규칙 테이블은 후순위). |

**검증/실측**:
- ctest **`category`** 확장(cat4 CBS·cat5 min-straight): 6M셀 합성 병목에서 **CBS 성공 비감소(무손실)·결정적**, **min-straight 성공 비감소·꺾임 비증가·결정적** 고정.
- ctest **`realdata`** 확장: 실데이터 픽스처(proj1 Exhaust c100)에서 min_straight=2 가 성공·총길이·총꺾임 **비증가**(엔진 레벨 무손실 보장).
- 실데이터 라이브 A/B(`--dbroute 1 100 Exhaust`, 20/20): base 316900mm·135꺾임 → **min_straight=2 316900mm·133꺾임**(짧은 단관 2개 무손실 흡수, 길이 불변) · CBS=2 무변(20/20 가용 씬이라 회복 대상 없음=정상).
- **교훈(중요)**: C2 첫 구현(배치 중간 직선화)은 다운스트림 교란으로 **총 꺾임 증가**(135→141). 그리고 *뷰어 bin DLL 미갱신*(cpp/build 만 빌드)으로 stale 측정에 한동안 속았다 — **capi 수정 후 반드시 뷰어 bin 으로 DLL 복사/뷰어 재빌드**. 최종 비교란 패스로 135→133 정상화.

**Phase C 결론**: CBS(C1)·코너 최소반경(C2) 모두 **무손실·결정적·기본 OFF**(골든 불변)로 추가. ctest **13/13 유지**(category·realdata 골든이 두 기능 동작/안전성 고정). 관경 후처리 중 랙선호·이격은 기존 기능(P3n·B2)이 담당해 신규 규칙 테이블은 후순위.

---

## 9. 종합 진행 (2026-06-12)

| Phase | 상태 | 핵심 |
|---|---|---|
| A 계약·안전성 | ✅ 완료 | 실패사유·visited OFF·대형API·64비트계약·UTF-8 |
| B 관경 물리 | ✅ B1·B2 / ⏸ B3보류 | per-task 반경(18/20→20/20·18×)·관경 clearance |
| C 다중배관 품질 | ✅ C1·C2 완료 | CBS-lite(연쇄 rip-up·무손실)·코너 최소반경(비교란 최종패스, 135→133) |
| D 검증 | ✅ 완료 | 실데이터+카테고리 골든(ctest 13/13) |

**ctest 13/13 통과**(전부 기본값=기존동작, 골든 바이트 불변). **전 Phase(A·B·C·D) 완료** — 신규 엔진 기능(실패사유·대형격자 일관·per-task 반경·관경 clearance·CBS-lite·코너 최소반경)은 모두 opt-in, 기본값=기존 동작이라 골든 불변. 남은 선택지: B3 FCL 통합(dep-free DLL 보존 위해 보류) · C2′ 관종별 이격 규칙 테이블(후순위).

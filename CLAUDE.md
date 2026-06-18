# Routing3D — 프로젝트 컨텍스트

이 파일은 다른 PC / 새 세션에서 프로젝트를 재개할 때 필요한 핵심 정보를 한곳에 모은 것이다.
세부 사항은 git 이력과 `docs/`, `cpp/`, `csharp/`, `python_experiments/` 가 정답.

> 마지막 갱신: 2026-06-10 · 단위 mm · 기본 셀 25mm · DB=DDW_AI_DB (배관-배관 충돌 회피[옵션1 pipe_radius] + 탐색상한 env 상향 포함)

---

## 1. 프로젝트 개요

**Routing3D** = 플랜트 배관 3D 직교 자동 라우팅 엔진. PostgreSQL(AUTOROUTINGV7) 의 실제 BIM 장애물·메인장비 PoC를 입력으로 받아 충돌 없는 배관 경로를 산출한다.

전략:

- **Phase 1**: Python 알고리즘 실험 (`python_experiments/routing3d_py/`)
- **Phase 2**: 인터페이스·계약 동결 (`docs/spec/*.md`)
- **Phase 3**: C++ 엔진 + pybind11 (`cpp/`) + **C ABI DLL** + **C# WPF 뷰어** (`csharp/Routing3D.Viewer/`)

신규 개발. 인접 `..\SpaceAI\`(C# 직교 A* + 동일 DB)는 데이터·UI 스타일 참조용(소스 읽기 가능, 직접 포팅 안 함).

---

## 2. 단위·환경·실행

- **단위**: 모든 좌표·치수 **mm**. 기본 셀 50mm (`RouteParams.cell_mm`).
- **Python 환경**: 루트 `.venv`. `python_experiments` editable 설치 (`pip install -e "python_experiments[viz]"`) → 어디서든 `python -m routing3d_py.<module>` 실행.
- **C++ 빌드**: MSVC VS2022 + CMake, **C++20**, **`/utf-8` 필수**(한글 주석). x64 고정.
- **C# 빌드**: .NET 9, `net9.0-windows`, **x64 고정**(네이티브 DLL 비트 일치).
- **DB**: localhost / 5432 / **DDW_AI_DB** / postgres / dinno (로컬 dev, 2026-06-06 AUTOROUTINGV7 폐기). 운영 환경에서는 PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD 환경변수 우선 — **소스에 비밀번호 두지 말 것**. 스키마 차이·매핑은 §8.

### 표준 명령

```powershell
# === Python (Phase 1) ===
.\.venv\Scripts\python.exe -m pytest python_experiments        # 전체 pytest
.\.venv\Scripts\python.exe -m routing3d_py.scene --project 6   # DB 씬 로드
.\.venv\Scripts\python.exe -m routing3d_py.scene_io --project 6 --cell-mm 100 --multi --out out/project6.scene.txt

# === C++ 엔진 (Phase 3) ===
cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
cmake --build cpp/build --config Release            # routing3d_cli + routing3d_capi 동시 빌드
ctest --test-dir cpp/build -C Release               # 9/9 통과 기대
.\run.ps1                                            # 내장 데모(자동 빌드+실행)
.\run.ps1 route --in scene.txt --out routed.scene.txt --mode multi
.\run.ps1 route --in scene.txt --mode ripup          # Step 3.8

# === C# 뷰어 ===
cmake --build cpp/build --config Release --target routing3d_capi   # DLL 먼저
dotnet build csharp/Routing3D.Viewer.sln -c Release                # routing3d_capi.dll 자동 복사
dotnet run --project csharp/Routing3D.Viewer -c Release            # DB 자동 로드(P3f)
csharp/Routing3D.Viewer/bin/x64/Release/net9.0-windows/Routing3D.Viewer.exe scene.txt          # 특정 scene
csharp/Routing3D.Viewer/bin/x64/Release/net9.0-windows/Routing3D.Viewer.exe --selftest scene.txt out.txt  # 헤드리스

# === 문서 생성 ===
python python_experiments/out/_gen_dev_report.py                                            # docx
python python_experiments/out/_gen_regression_report.py                                      # docx (project6 4케이스 라우팅, ~5분)
powershell -ExecutionPolicy Bypass -File python_experiments/out/_docx_to_pdf.ps1 -in docs/X.docx -out docs/X.pdf
```

---

## 3. 아키텍처 (3-tier)

```
┌──────────────────────────────────────────────────────────────────┐
│ Python 레퍼런스 (Phase 1)                                         │
│  routing3d_py/{occupancy, astar, cost, multi_route, scene_io,    │
│                 obstacle_db, scene, viz, viz_scene}.py            │
└────────────────────────────┬─────────────────────────────────────┘
                             │ (씬·골든 셋 1:1 미러)
┌────────────────────────────▼─────────────────────────────────────┐
│ C++ 엔진 (Phase 3) — cpp/                                         │
│  include/routing3d/{geometry, occupancy(Dense/Sparse/Vdb),        │
│                     cost, astar, multi_route, corridor,           │
│                     scene_io, fcl_scene, route_task}.hpp          │
│  cli/routing3d_cli.cpp           → routing3d_cli.exe              │
│  bindings/bindings.cpp           → routing3d_cpp.pyd (pybind11)   │
│  capi/routing3d_capi.{h,cpp}     → routing3d_capi.dll (C ABI)     │
│  tests/                          → ctest 9/9                      │
└────────────────────────────┬─────────────────────────────────────┘
                             │ (P/Invoke + UTF-8)
┌────────────────────────────▼─────────────────────────────────────┐
│ C# 뷰어 — csharp/Routing3D.Viewer/                                │
│  Interop/{Native, R3dEngineHandle, Engine}.cs                     │
│  Model/{SceneData, SceneTextParser, UtilityColors,                │
│         CollisionFinder, ObstacleDbLoader}.cs                     │
│  ViewModels/{SceneViewModel, TaskRowVM, UtilityFilterVM,          │
│              ObservableObject, RelayCommand}.cs                   │
│  MainWindow.{xaml,.xaml.cs} + App.{xaml,.xaml.cs}                 │
└──────────────────────────────────────────────────────────────────┘
```

**불변식(C1·M1·M2·O1·F2~F4·A2/W1)**: `docs/spec/*.md`. A* 결정성, 다중배관 충돌 0, 원본 점유맵 불변, 백엔드 무관 동일 결과, scene.txt 바이트 무손실 왕복, format_repr_double = Python `repr(float)`.

---

## 4. 현재 상태 (모든 마일스톤)

### Phase 1 — Python 알고리즘 (완료, 2026-05-28)

- occupancy: Dense/Sparse/BitPacked 백엔드, A* 백엔드 무관, `copy()` 지원
- astar: 균일 + `astar_weighted`(상태=(셀,진입방향))
- cost: `RouteParams` + `clearance_map`(BFS 거리변환) + `CostModel` (가산 페널티만 → admissibility 보존)
- multi_route: `route_sequential` + `route_ripup` (Step 3.8) + `order_tasks`/`order_indices`
- scene_io: scene.txt v1, `dumps/loads/read/write_scene`, `occupancy_from_doc`
- obstacle_db: `PgConnConfig` + `load_obstacles`
- scene: `list_projects`, `load_scene(project_id)`
- viz: PyVista 3D (점유/복셀격자/A* 경로/유틸별 다중경로/방문 레이어), `--visited` CLI
- 회귀 시나리오 3종(`tests/scenarios/01_single_empty`, `02_single_obstacle`, `03_multi_tier`) + `baseline_params.json`
- **pytest 203 통과**

### Phase 2 — 인터페이스 동결 (완료, 2026-05-28)

- `docs/phase2_plan.md` + `docs/spec/{algorithm_spec, scene_format_spec, regression_set, performance_targets, freeze_signoff}.md`
- **성능 목표**: 8,000m 도메인, 단일 배관 <1초, 전체(수백) <1분, 메모리 <32GB

### Phase 3 — C++ 엔진 (완료, 2026-05-29~30)

| Step | 내용 | 검증 |
|---|---|---|
| 3.1~3.4 | geometry / occupancy / cost / astar — 헤더 전용 템플릿(점유 백엔드 무관, 컴파일타임 다형성) | 골든 01/02 expanded_nodes 까지 Python 정확 일치 |
| 3.5 | multi_route — `route_sequential`/`order_tasks`/`mark_pipe`/`snap_to_free_cell` | 골든 03: 5/5·충돌 0·총 28050mm |
| 3.6 | OpenVDB 백엔드 + **계층 corridor**(`astar_hashed` 해시 기반, 초대형 격자) | 8,000m³ 로컬 배관 ~75ms |
| 3.7 | FCL 정밀 충돌(`fcl_scene`) — sub-voxel 캡슐 검사 | 틈 200mm 가는·굵은 파이프 구별 |
| **3.8** | **rip-up & reroute** — 무손실 결정적, 합성 1/2→2/2, project6 cell=200 +3 실측 | ctest `ripup` + pytest |
| 3.9 | scene.txt I/O (Python 픽스처 바이트 동일 재출력) | F2 무손실 왕복 |
| 3.10 | pybind11 바인딩 → `routing3d_cpp.pyd` | 골든 01/02/03 + scene.txt 왕복 일치 |
| **3.11** | **sparse 확장(정밀 셀 대응)** — `ImplicitOccupancy`(복셀화 폐기, 장애물 AABB를 `SpatialBoxIndex` 유니폼그리드로 색인) + A* **64비트 키**(`state_of`/`unlin`) + **온디맨드 클리어런스**(전역 거리변환 폐기, `CostModel` `HasClearanceQuery`). 격자>5M셀이면 capi 가 자동 전환(이하 Dense=골든 바이트 불변) | ctest `implicit`(Dense==Implicit 전수 일치) + 25mm 14/14·10mm 크래시 0(20.3억 셀) |
| **3.12** | **회귀 리포트** — 표준 벤치 자동 측정·기대치 비교 → `docs/routing3d_regression_report.{docx,pdf}` | 골든 3/3 PASS |

- **CLI**: `routing3d_cli` (코어만, demo/route/summary 명령, `--mode multi\|single\|ripup`)
- **DLL**: `routing3d_capi.dll` (외부 의존성 0)
- **점유 백엔드 3종**: `DenseOccupancy`(소격자·골든) / `SparseOccupancy`(corridor) / **`ImplicitOccupancy`**(정밀·거대격자 — 메모리 O(장애물+깔린셀), 셀 크기 무관)
- **ctest 10/10**: golden · scene_io · occupancy · corridor · **implicit** · ripup · attract · capi · vdb · fcl
  (pybind `bindings` 는 commit 2ce3eb8 에서 `astar_weighted` 에 corridor 인자 추가 후 미갱신 — `bindings.cpp:134` 에 `py::arg("corridor")` 누락. 파이썬 모듈 빌드 시에만 영향, capi/뷰어 무관)

### C# 인터롭 (완료, 2026-05-29~30)

| 단계 | 내용 |
|---|---|
| **P0** | `routing3d_capi` DLL + Level 1·2 C ABI |
| **P1** | WPF + HelixToolkit 2.24.0 뷰어, `route_multi`·3D 렌더 |
| **P2** | 인터랙티브 재라우팅 — 종단점 편집 + 단일/전체 |
| **P3a** | 충돌 시각화 + 표시 토글 + 3D 클릭 종단점 지정 |
| **P3b** | corridor 라우팅 C ABI(`r3d_route_corridor`, Sparse, OpenVDB 불필요) |
| **P3c** | scene.txt CLI 인자 로드 + `--selftest` 헤드리스 검증 |
| **P3d** | SpaceAI 다크 3-컬럼 UI + 🔍 검색 + 유틸리티 체크박스 필터 + ↺ 전체보기 |
| **P3e** | 3D 신규 레이어 3종 — **복셀 전체맵 / 점유맵 / 방문맵(유틸리티 색)** |
| **P3f** | **PostgreSQL 자동 로드** — 실행 시 AUTOROUTINGV7 접속 → 프로젝트 콤보 → 라우팅 → 전체보기 |
| **P3g** | **워크플로 재설계** — 창 즉시 표시 + DB 비동기 로드(UI 비차단) · 프로젝트 선택 시 장애물만 표시 · 탐색 범위(모두/유틸그룹별/유틸별) 선택 실행 · 좌측 드릴다운(그룹→유틸→PoC) |
| **P3h** | **DB 레이어 확장** — 장비(TB_BIM_EQUIPMENT) · 레터럴/덕트(TB_DUCT_LATERAL, CATEGORY별 토글) · 공간영역(TB_BIM_SPACE_INFO LEVEL_NAME, 와이어+라벨) · PoC 이름 로드 · 점유맵 원본/샘플 토글 · 객체 중앙 정렬 |
| **P3i** | **탐색 시각화** — 선택 배관 단계별 A* 애니메이션(방문셀 확장순서) + 경로 꺾임 마커 + 우측 구간 단계 리스트(클릭 시 카메라 이동) |
| **P3j** | **기존설계 패턴 학습(pgvector)** — 학습면 PoC 투영 + 접근불가 PoC 스냅 + 기존설계 회랑(L2b). 아래 별도 절 참조 |
| **P3k** | **자동설계 진행/진단 패널** — 우측 '자동설계 결과 경로'에 전체 진행바(완료/성공/실패·%) + 행별 라이브 상태(대기/탐색 N%/성공/실패) · 하단 분석에 **꺾임 발생 이유**(라이저/장애물 회피/랙 전환/정렬 분류) + **실패 원인 상세 진단**(PoC 매몰·격자 밖·국소 차단·혼잡·탐색 상한, 최근접 자유셀 거리). `TaskRowVM.RunState`·`SceneViewModel.{RouteProgress*,ExplainBends,ExplainFailure}`. **취소 버튼**(진행바 옆 ✖, 진행 중에만): 협력적 취소 — 콜백 ABI `R3dProgressFn` 반환값(0=계속·0아님=중단)을 `astar_weighted`(약 5만 확장마다)·배관 완료마다 검사해 현재 배관에서 즉시 중단, 완료분 보존·나머지 미라우팅. `SceneViewModel.CancelRoutingCommand`/`_cancelRequested`·`Engine.RouteMultiProgress(…, shouldCancel)`. ctest `capi` 에 진행/취소 검증 추가 |
| **P3l** | **자동설계 결과 리포트(2026-06-11)** — 직전 자동설계 배치를 별도 모드리스 창([Views/RouteReportWindow](csharp/Routing3D.Viewer/Views/RouteReportWindow.xaml))으로 띄우고 **Markdown(.md) 자동 저장**(`내 문서\Routing3D\autodesign_result_<ts>.md`, 창에서 '다른 이름으로 저장'·복사·폴더열기). 내용: ① **설계 순서**(우선순위 정렬=굵은 배관 먼저, 진행 콜백 `OrderIndex`로 기록 → `TaskRowVM.RouteOrder`) ② **꺾임 지점 선정 방법**(가중 A* 회전비용 + 4범주 분류 라이저/회피/랙/정렬, 전체 집계) ③ **연결 성공/실패 + 실패 원인**(`ShortFailReason`). 툴바 '📄 결과 리포트' 버튼(`RouteResultReportCommand`, 배치 완료 후 활성). `SceneViewModel.{BuildRouteResultReportMarkdown,ClassifyBends(추출),ShowRouteReportRequested}` · `RouteRowsAsync` 가 배치 끝에 `_lastReport*` 저장. 화면에 이미 그린 결과를 설명(헤드리스 `AutoDesignReport` 비교 리포트와 별개) |
| ~~**P3m**~~ | **(되돌림 2026-06-11) 단관 정형화 — 배관 렌더링 문제로 제거**. `PathRectifier.cs`·`TaskRowVM.RenderPolyline/RenderSig`·`SceneViewModel.{BuildRawRoutedPolyline,SnapToCellCenter,RasterizePolylineCells,SegmentClear,RectifiedRenderPolyline,UpdatePipeOwner}`·env `R3D_MIN_SPOOL` 전부 삭제. 자동 경로 렌더는 다시 원시 합성 `BuildRoutedPolyline`([스텁 실좌표]+[A* 셀중심]+[reverse 종단스텁])으로 그린다(셀 중심 스냅·정형화 없음). 굵은배관 우선(§9)·결과 리포트(P3l)·다단 랙(P3n)은 유지. **(원래 시도 내용·교훈 보존)** 자동 경로 폴리라인에 짧은 계단 꺾임(직각 한 번이면 될 곳에 2단). 실제 배관은 엘보 사이 직선(단관)이 **관경의 2배보다 짧으면 제작 불가**. **핵심 원인=스텁(실좌표) vs A*(셀 중심)의 ½셀 격자 불일치 → 접속부 3축 미세 지터**. 옛 정형화기만으로는 3축 지터를 직교화하다 꺾임이 오히려 늘었다(4→5). **수정=① `BuildRawRoutedPolyline` 이 스텁 점을 A* 와 같은 셀 중심 격자로 스냅(`SnapToCellCenter`) → 스텁 끝이 A* 시작과 같은 셀(=같은 점)에 떨어져 접속 지터 소멸 ② [Model/PathRectifier.cs](csharp/Routing3D.Viewer/Model/PathRectifier.cs)(직교화→압축→직선화/계단합치기/코너플립)가 남은 셀정렬 단을 `minSeg=2×관경` 미만이면 직선화** — **충돌검사(`SegmentClear`=장애물 `CellBlocked` + 타 배관 셀 `pipeCellOwner`) 통과 시에만**, 양 끝점(PoC·스텁) 고정. 실측 합성 접속지터 케이스 **4~5꺾임 → 2꺾임**(리저-수평-리저). `BuildModel` 이 `RectifiedRenderPolyline`(서명 캐시 = 토글마다 재계산 안 함)로 `TaskRowVM.RenderPolyline` 채움 → 렌더·클릭검출·`DescribeRoutedPipe`(길이/꺾임)·스텁 셸 공유. **증분 처리**(`UpdatePipeOwner` = 정형화된 배관 셀로 색인 갱신 → 다발에서 뒤 배관이 앞 배관의 깨끗해진 결과 기준으로 직선화). env `R3D_MIN_SPOOL`(배수, 기본 2.0·0=끔). **한계: 다발(번들) 혼잡 구간은 깨끗한 엘보가 인접 배관과 충돌해 계단 잔존 — 근본 해법은 z-레벨 분리(사람 설계처럼 다단 랙). cell>pitch(100>~56)면 인접 레인이 같은 셀로 뭉개짐(§L4 한계)** |
| **P3n** | **다단 랙 z-레벨 분리(2026-06-11)** — 자동 라우팅이 모든 그룹을 ~한 평면으로 수렴시켜 혼잡·계단·우회가 생김(사람 설계는 그룹마다 다른 높이=랙 단). 툴바 '🪜 다단 랙 (z-분리)'(`RouteTieredCommand`) = 현재 범위를 유틸그룹마다 전용 z-단에 배정해 **단별 순차 라우팅**(앞 단을 다음 단의 장애물로 누적 → 충돌 0). **핵심 검증/교훈**: **소프트 랙 바이어스(`rack_levels`+`w_corridor`)는 배관을 수직으로 못 움직인다** — 실측 자연고도(Exhaust z=44, 83% 추종) 외 z=46 강제 시 **0% 추종**(배관은 스텁 끝 높이를 따라가지 랙 바이어스를 안 따름). 따라서 `RouteTieredAsync`는 **A* 끝점(스텁 끝) 자체를 단 높이로 강제**(`BuildEngineForRows` 의 `_forcedRackZ` 분기 = 각 PoC→tierZ 수직 라이저 스텁 + A* 는 tierZ 평면 탐색) → 끝점이 단 높이라 그 평면 traverse 가 보장됨. `BuildGroupTierZ`(그룹별 기존설계 지배 랙 z 자연고도 → 충돌 시 관경반경+2셀 간격으로 위로 분리). 한 그룹=한 단이라 **그룹 간** 분리엔 효과적, **그룹 내** 혼잡은 여전(cell≤pitch 필요). **개선(2026-06-11): 학습된 실단(real tier) 배정** — 임의 push-up 대신 기존설계의 **전역 랙 단을 추출**(`BuildLearnedTiers` = 전 기존배관 수평 런 z 를 런길이 가중 히스토그램 → 인접 250mm 병합 클러스터 → 3% 미만 노이즈 단 제거, 헤드리스 `--racktiers <proj> <cell>` = `DbRouteDiag.RunRackTiers` 와 동일 추출)해 그룹을 **자연 친화도 강한 순으로 가장 가까운 빈 실단**에 배정(실단 소진 시에만 최상단 위 합성). 사람설계와 같은 높이엔 실제 수평 런이 있어 traverse 자연스럽고 결과가 기존설계에 근접. 학습 단 없으면 옛 휴리스틱 폴백. **실측(proj1 c100): 8그룹이 5개 실단(z≈11922~14951, 약 3m)에 분산 = 깨끗한 다단 입증**(Waste계=최상단·Water=최하단·Gas/Exhaust/Chem 중간). **기본 통합(2026-06-11): '자동설계'(`RunRouteAsync`)가 범위에 그룹 2개 이상이면 자동으로 다단 z-분리(`RouteTieredAsync`) 적용** — 토글 `UseTieredRack`(라우팅 옵션 '🪜 다단 랙(z-분리)' 체크박스, 기본 ON) · 단일 그룹/유틸은 분리 대상 없어 일반 충돌회피 라우팅(라이브 진행 유지) · 다단 배치도 `_lastReport*` 채워 '📄 결과 리포트' 연동. 🪜 버튼은 그룹 수 무관 강제 실행용으로 유지 |
| **P3o** | **덕트 스텁 일직선 진입(목표 진입축 제약, 2026-06-11)** — 자동배관이 덕트 종단 스텁 끝(A* 목표)에 임의 방향으로 진입해 접속부에 군더더기 꺾임·옆 배관 간섭이 생겼다(사람설계[자주색]는 스텁에 일직선 진입). **엔진 A* 상태가 이미 진입방향(`dir`)을 추적**하는 점을 이용해 **목표 진입축 제약** 추가: `astar_weighted(..., int goal_dir=-1)` — 목표 수락 조건 `cur.cell==goal && (goal_dir<0 || cur.dir==goal_dir)`. **기본 -1=무제약 → 골든 바이트 불변(ctest 11/11)**. `RouteTask.goal_dir`(scene.txt 미직렬화) → `multi_route`(route_sequential/ripup)·capi `route_multi_impl`/인라인 ripup 전파. C ABI `r3d_set_task_goal_dir(task, axis)`(axis=NEIGHBORS_6 0..5=+x,-x,+y,-y,+z,-z) + Native/Engine 래퍼. **C#** `BuildEngineForRows` 스텁 분기가 종단 스텁 **리드인 축**(`StubExtractor.AxisSnap(endStub[^2]−endStub[^1])`, 수평 x/y 일 때만)을 `goal_dir`로 설정 → A* 가 리드인과 일직선으로 진입. **막히면(장애물로 그 축 진입 불가) 무제약 1회 폴백**(연결 우선, 성공률 보존). `DbRouteDiag` env `R3D_GOALDIR=off`(A/B). **실측(proj1 c100 Exhaust 20/20)**: off turns 26 → on 32. **STUBDUMP 렌더 꺾임 분해(배관당 ~8회)로 goal_dir 무효 판명** — 종단접속 0.8→0.9·총 7.9→8.3 악화. 꺾임 주원인은 진입'방향'이 아니라 **렌더 배관=[스텁]+[A* 연결부]+[스텁] 3조각 구조**(스텁 자체 4.2 + A* 연결부 2.5 + ½셀 접속 kink 1.7)였고, 자주색(기존)은 끊김 없는 한 줄. → **goal_dir GUI/헤드리스 기본 OFF**(엔진 기능·C ABI 는 골든 안전하게 보존, env `R3D_GOALDIR=on` 실험용). 진단: `R3D_STUBDUMP=on`(렌더 폴리라인 구간별 꺾임). **해법은 P3p(기존설계 추종 복제)** |
| **P3p** | **분홍 배관을 자주색처럼 — 기존설계 추종 복제 기본화(2026-06-11)** — P3o 분석 결론: A* 가 매칭 기존설계의 **중간을 버리고 새로 탐색**해 연결부(2.5)+접속 kink(1.7)가 자주색엔 없는 꺾임을 더한다. 해법=이미 성숙한 `ReplicateMatchedPipes`/`ReplicateCellPath`(기존 폴리라인 전체를 셀로 복제 + 현재 점유에서 **막힌 구간만** 국소 A* 수리 + `StraightenOrtho`/`DeJog` 양자화 정리)를 **기본 경로 방식으로** 승격. `Mode` 기본값 `GroupPattern`→**`FollowExisting`**(`_useDesignReplicate=true`·`_routeWaypoints=false`). 매칭 배관은 자주색 형상 그대로(셀 중심 ½셀 오프셋만), 미매칭은 A* 폴백. **다단 랙(z-분리)은 FollowExisting 에서 자동 OFF**(`RunRouteAsync` 가 `_routeMode!=FollowExisting` 일 때만 tiered) — 복제가 사람설계 z-레벨을 그대로 따르므로 단 강제와 상충. **실측(proj1 c100 Exhaust)**: `replicate matched 20 ok 20 fail 0 repairs 19`(전 배관 복제 성공, 평균 ~1회 국소 수리). 라디오 '기존설계 추종'·`R3D_REPLICATE` |
| **P3q** | **엔진 고도화 Phase A+B(2026-06-11)** — 개발계획서 [docs/routing3d_engine_improvement_plan.md](docs/routing3d_engine_improvement_plan.md). **A(계약·안전성, 골든 불변)**: A1 실패사유 `RouteFail` enum(astar→`SceneResult.fail`→C ABI `R3dResult.fail_reason`→C# `RouteResult.Fail`·`TaskRowVM.LastFail`→`ExplainFailure` 권위 사유) · A2 `collect_visited` 기본 OFF(뷰어가 방문맵/단계탐색 시 `SetCollectVisited` opt-in) · A3 `r3d_route_ripup`·`r3d_copy_blocked` >5M셀 ImplicitOccupancy 전환(Dense 폭발 방지) · A4 `clearance_map` narrowing 제거+인덱스 계약 문서화 · A5 UTF-8 점검(소스 무결, 수정 불필요). **B(관경 물리 라우팅)**: B1 **per-task 반경**(`route_multi_impl.radius_of`=clamp(ceil(d/cell)-1,0,8), C ABI `r3d_set_per_task_radius`, 뷰어 ON) — 실측 글로벌r=5 18/20·47꺾임·20s → per-task **20/20·31꺾임·1.1s**(가는관 과패킹 해소·18×) · B2 관경 clearance(`params_for` 가 반경만큼 `clearance_radius` 임계↑) · B3 FCL통합=보류(capi dep-free 보존). **ctest 11/11 불변**(전부 기본값=기존동작). env `R3D_PER_TASK_RADIUS`·`R3D_GOALDIR`·`R3D_STUBDUMP` |
| **P3r** | **엔진 고도화 Phase C+D(2026-06-12)** — 개발계획서 [docs/routing3d_engine_improvement_plan.md](docs/routing3d_engine_improvement_plan.md) §8·§9. **C1 negotiated-congestion(CBS-lite)**: 평면 rip-up(실패 배관의 *직접* blocker만 재배치)을 **연쇄(재귀) rip-up**으로 확장 — blocker 가 재배치 못 하면 그 blocker 의 blocker 까지 bounded depth 재귀 양보(conflict-based search 경량판). `route_multi_impl` 평면 rip-up 직후 패스. 불변식 `resolve(target,state)` 성공 시 결과⊇`state 전 배관 ∪ {target}`(재귀도 동일 by construction)→성공 **단조 +1(무손실)**·결정적·종료(깊이 −1, 분기 ≤(MAXBLK4+1)^(depth≤3+1)). 대형격자·잔여실패 시에만. C ABI `r3d_set_cbs_depth`·C# `SetCbsDepth`/`UseCbs`(체크박스 '협상 라우팅(CBS)')·env `R3D_CBS`. **C2 코너 최소반경**: 엘보 간 직선 < (배수×관경)인 짧은 단관을 **경로(셀) 단계**에서 충돌검사 하에 양옆 코너 직교 흡수(`enforce_min_straight`) — PathRectifier(P3m·렌더 레벨·되돌림)와 달리 충돌 안전. **핵심: 비교란 최종 패스** — 배치 중간 직선화는 다운스트림 점유 교란으로 총꺾임 ↑(실측 135→141)였다 → *모든 라우팅·rip-up·CBS 후* 각 배관을 '장애물+다른 배관'만으로 만든 검사점유에 흡수(라우팅 점유 불변·꺾임/길이 비증가만 채택, 양 끝점 고정). C ABI `r3d_set_min_straight`·C# `SetMinStraight`/`UseMinStraight`(체크박스 '코너 최소반경')·env `R3D_MIN_STRAIGHT`. **D 검증**: ctest `category` 확장(cat4 CBS·cat5 min-straight 무손실·결정적) + `realdata` 확장(픽스처 min_straight 비증가). **실측(--dbroute 1 100 Exhaust 20/20)**: base 316900mm·135꺾임 → min_straight=2 **316900mm·133꺾임**(단관 2개 무손실 흡수·길이불변) · CBS=2 무변(20/20 가용씬=회복대상 없음). **둘 다 기본 OFF·골든 불변(ctest 13/13)**. **교훈: capi 수정 후 뷰어 bin DLL 복사 필수**(cpp/build 만 빌드 시 stale 측정). 전 Phase(A·B·C·D) 완료 |
| **P3s** | **배관 이격 규격(센터선 ≥ r1+r2+60mm) + 자동경로 가시성(2026-06-12)** — 사용자 지적 3건. **① 이격 규격**: 기존 per-task 마킹(`radius_of≈ceil(d/cell)-1`)은 센터선을 ~관경(d)만큼만 띄워 **두 배관 표면이 맞닿아 겹쳐 보였다**(gap=0). 규격=센터선 거리 ≥ r1+r2+60mm. `route_multi_impl` 메인 루프에 **쌍(pairwise) 마킹**: gap>0 이면 깔린 배관을 routing 배관 기준 `ceil((r_a+r_b+gap)/cell)` 반경으로 막아(per-pipe 재구성) 센터선을 정확히 r_a+r_b+gap 띄운다. 종단 스냅 반경도 쌍 자기반경으로 키워 근접 PoC 가 묻히지 않게. C ABI `r3d_set_pipe_gap`·C# `SetPipeGap`/`UseClearanceGap`(체크박스 '배관 이격(60mm)', 기본 ON)·env `R3D_PIPE_GAP`. **실측(--dbroute 1 100 Exhaust)**: gap0 20/20·135꺾임·번들밀집 5.6% → **gap60 20/20·180꺾임·번들밀집 3.1%**(배관 분리=겹침 해소 입증). **트레이드오프(중요)**: 평면에서 이격하면 배관이 서로 우회해 꺾임 ↑(135→180). 겹침↔꺾임은 본질 상충 — 둘 다 줄이는 길은 z-레벨 분리(다단 랙 P3n). gap+min_straight=2 면 180→176(C2 가 단배관 일부 흡수). **② 자동경로 가시성 버그**: 라우팅 중 라이브 오버레이가 `disp.BeginInvoke` 로 갱신되는데 일부가 최종 `BuildModel` 이후 실행돼 경로를 가려 '체크박스 토글해야 보이던' 현상 → `RouteRowsAsync` finally 에서 디스패처 유휴(Background) 시점에 확정 렌더 1회 추가. **③** 기본 0(gap)=기존 동작·골든 불변. ctest `category` cat6(평행 2배관 센터선 분리 OFF 4셀→ON 7셀·둘 다 라우팅) 추가, **13/13 유지**. **주의**: FollowExisting(기본 복제) 모드에선 매칭 배관이 기존설계 복제라 gap 은 미매칭(A*) 배관에만 적용 — 매칭 배관 겹침은 기존설계 자체 이격을 따름 |
| **P3t** | **자동경로 렌더 3대 오류 수정(계단·충돌·관경, 2026-06-12)** — 사용자 스크린샷 분석(3유형). 모두 **C# 뷰어(FollowExisting 기본 복제) 렌더/후처리** 문제, 엔진 무변경(ctest/골든 불변). **① 관경이상**: 미매칭(A*) 작업이 `FindMatchingExistingPipe` 실패 시 `DiameterMm=0` → 렌더 폴백(`cell×0.7` 가는 튜브)으로 이웃 실관경과 굵기 불일치. **수정=작업 본인의 `TB_ROUTE_PATH.SOURCE_SIZE` 를 모든 작업에 직접 적재**(`TaskInfo.DiameterMm`·`ObstacleDbLoader`(idx3)·`BuildTaskRows`→`TaskRowVM.DiameterMm`) → 매칭 탐색 무관하게 실관경 렌더. + **스텁 셸 1.35× 제거**(`stubDia=routeDia`) — 본관 튜브(`BuildRoutedPolyline`)가 이미 스텁 구간을 같은 관경으로 그리는데 1.35× 강조 셸이 접합부 굵기 단차를 만들던 것. **② 배관충돌**: 고정 스텁(라이저)이 엔진/복제/직관화 어디서도 점유 마킹 안 돼 종단 근처 스텁끼리·본관과 겹침 + 종단 `margin=rr+1` 마킹 제외로 PoC 근처 본관 겹침. **수정=`RectifyRoutedPaths` 가 전 행의 출발/종단 스텁을 셀 표본화(cell/2)해 선마킹(`MarkStub`) + 마킹 `margin=1`(공유 PoC 셀만 제외, 종단 직전까지 관경 반경 마킹)**. **③ 계단현상**: `StraightenOrtho` 가 직선 L 이 옆 배관/장애물에 막히면 원본 계단 유지(혼잡 다발에 잔존). **수정=`StraightenOuter`(신설) — 직선 L 막히면 국소(≤24셀) 외곽 U자 우회(`OuterConnect`, 4방향×K, 최소 K 채택, 시도예산 4000)로 길이 늘려서라도 직선 런으로 편다**(사용자 요구 "길이 늘어도 외각으로"). 우회도 막히면 원경로 유지(충돌 불변). **dotnet build 0 errors**. 측정 미검증(GUI 렌더 — 헤드리스 `--dbroute` 는 route_multi 직접이라 복제/직관화 경로 안 탐) → 사용자 스크린샷 확인 필요. **근본 한계 유지**: 평면 내 겹침↔꺾임 상충, cell>pitch 다발 밀집 → z-레벨 분리(P3n) 필요 |
| **P3u** | **엔진 코너 최소직선(절대 100mm, A* 하드 제약, 2026-06-16)** — 사용자 요구: "한번 꺾였으면 최소 100mm 직진 후 다음 꺾임". 기존 `min_straight`(P3r·C2)는 **관경 배수·후처리 흡수**(best-effort, 관경 0이면 미적용)라 절대 보장이 아니었다. **신규=A* 탐색 상태에 진행 셀 수(run) 추가** → `run < min_run`이면 꺾기 금지 → 엘보 간 모든 내부 직선 런 ≥ min_run 셀(하드 보장, 관경 무관·전 배관, 목표 직전 마지막 접속 구간만 면제). `RouteParams.min_straight_cells`(=ceil(mm/cell))·[astar.hpp](cpp/include/routing3d/astar.hpp) 상태키 `(lin*7+(dir+1))*RS+run`(RS=min_run+1, **min_run≤1이면 RS=1·run=0 → 기존 상태키와 바이트 동일=골든 불변**). C ABI `r3d_set_min_straight_mm`(별도, 기존 `r3d_set_min_straight` 배수와 공존)·env `R3D_MIN_STRAIGHT_MM`·C# `Engine.SetMinStraightMm`·`BuildEngineForRows` 가 **100mm 기본 적용**(`apply_min_straight_cells` 가 라우팅 직전 cell로 환산, main·rip-up·CBS·corridor 전 경로 전파). **cell=100이면 min_run=1=비활성**(1셀이 이미 최소해상도), **cell=25/50이면 4/2셀=활성**(사용자 작업 셀). **실측(--dbroute 1 100 Exhaust, R3D_MIN_STRAIGHT_MM=300=3셀)**: OFF 141꺾임·318,000mm → ON **135꺾임·321,200mm**(계단 단관 제거, 길이 +1%, 20/20 유지, 시간 23s→55s=상태 ×min_run). **ctest 13/13 불변** + cat7 신설(내부 런 ≥ min_run 불변식·결정성). 복제(FollowExisting) 매칭 배관은 A* 미탐색이라 영향 없음(사람설계가 이미 만족), A*/스텁 구간에만 적용 |
| **P3v** | **엔진 적응형 해상도 라우팅 Tier-3 Stage 1·2(2026-06-16)** — 사용자 지적: cell=25 미세격자에서 경로탐색 느림 + 메모리 한계로 실패. **진단**: 병목은 OpenVDB 점유맵이 **아니라** A* 탐색 상태(`std::unordered_map` g/came/`unordered_set` closed 3개, 노드기반 → 배관당 ~1GB 피크). 거대격자(>5M셀)는 `route_hier`(coarse 골격→fine corridor) 적응형 경로를 escalation 으로 쓰는데 **coarse 골격을 greedy(w_heur=2.0)로 풀어** 골격이 우회 + fine 이 좁은 튜브에 갇혀 3~4× 우회·느림. **Stage 1(품질·속도)**: `route_hier` coarse 골격을 **최적(w_heur=1.0·동적가중 OFF·min_straight OFF)** 으로 푼다 — coarse 격자는 fine 의 1/factor³(예 8³=512배 작음 ~수십만 셀)이라 최적해도 저렴, 좋은 골격이 fine corridor 길잡이. **실측(--dbroute 1 25 Exhaust, cap 8M, 20/20)**: 199초→**127초(1.57×)**·최대확장 2.69M→2.13M·길이 동일. **Stage 2(메모리 robustness)**: fine A*가 corridor(coarse 골격 ±r 튜브)에서 실패하면 **반경 r 을 ×2 로 키워 최대 3회 재시도** → 좁은 튜브 미스로 '전체격자 fall-through(메모리 폭발·상한 실패)' 대신 넓힌 튜브 재탐색(메모리 한계 실패 직접 감소). 탐색은 항상 corridor 바운드라 최종 실패해도 메모리 안전. 성공 경로는 첫 시도=기존과 동일(무회귀). **둘 다 거대격자(hier 활성)에서만 동작 → 소형 골든격자 무영향, ctest 13/13 불변**(golden·capi·category·realdata 재검증). **남은 단계**: Stage 3=hier 활성 임계(5M) 하향 + 평면 해시맵(flat hash) 교체(메모리 2~3×↓·속도 2~5×↑, Tier-1 후보) · 진짜 옥트리 가변셀 단일그래프(per-pipe 마킹 재세분 문제로 대규모). **핵심 교훈**: cell=100(1.8M셀<5M)은 large_grid=false라 hier 자체가 비활성 → cell=100 우회는 hier 아닌 weighted A*(w=2.0) greedy 탓. 적응형 효과는 cell≤50(>5M셀)에서만 |
| **P3w** | **엔진 적응형 해상도 Tier-3 Stage 3a·3b(2026-06-16)** — 10mm 초미세격자 검토 + 메모리/속도 추가 개선. **Stage 3a 평면 해시맵**: A* 탐색 상태 3개(노드기반 `unordered_map` g/came + `unordered_set` closed, 엔트리당 ~3×·캐시미스)를 **오픈어드레싱 평면 해시맵 1개**(`detail::StateMap`, state→{g,parent,closed} 슬롯, splitmix64·선형탐사·load 0.7)로 합침([astar.hpp](cpp/include/routing3d/astar.hpp)). 메모리 ~3×↓·캐시친화. **결정성은 PQ(f,counter)가 보장 → 맵 교체는 결과/expanded_nodes 무영향(골든 바이트 불변, ctest 13/13)**. 주의: emplace 가 grow(재할당)하므로 Slot 참조를 emplace 너머로 보관 금지(g_st 값 복사). **실측(--dbroute 1 25 Exhaust)**: Stage1 127초→**86.6초(1.46×↑)**·golden 0.93→0.43s·category 17→10s. **Stage 3b 적응형 coarse factor**: route_hier 의 coarse factor 를 미설정 시 fine 셀 크기에 맞춰 **coarse 셀 ~200mm 고정**(factor=round(200/cell), [4,32])([routing3d_capi.cpp](cpp/capi/routing3d_capi.cpp)) → coarse 격자 셀 수가 해상도 무관하게 일정 → 10mm 초미세격자에서도 coarse 골격 솔브 비용 안 터짐. cell=25→factor8(기존 동일·측정 불변)·cell=10→20·cell=50→4. **10mm 실측(1351×1458×903 ≈ 17.8억 셀)**: **스텁모드 20/20·10.8초·67,570mm·랙정렬82%·메모리실패 0**(VDB 점유 + 평면맵 + 스텁라우팅이 17.8억 셀을 무난히 처리 — 현실 워크플로[FollowExisting/PatternApplied]는 10mm 완전 해결). **누적 효과(Stage1+2+3a, cell=25 shortest)**: 199→86.6초 = **2.3×↑** + 메모리 한계 실패 직접 감소(corridor 바운드 + 확장 재시도). **남은 일**: 진짜 가변셀 옥트리 단일그래프(shortest 모드 초미세격자 전용, per-pipe 마킹 재세분 문제·골든위험 큼 → 현실모드 이미 해결돼 ROI 낮음, 보류). **교훈**: 메모리 병목은 OpenVDB 점유맵이 아니라 A* 탐색 상태 컨테이너였음(평면맵으로 직접 해결) |
| P3b' | OpenVDB capi (선택, 보류) |
| (보류) | DDW_AI_DB 전환 — 스키마 전면 재설계(로더 재작성 필요)로 보류, 현재 AUTOROUTINGV7 |

### 기존설계 패턴 학습 (pgvector, 2026-06-02)

사람이 설계한 기존배관(TB_ROUTE_PATH)의 양 끝 '스텁'(장비 출발·덕트/레터럴 진입) 형상을 학습해 자동라우팅에 활용한다. 커밋 `e1adeb3`(L1/L2a/전처리) + `5eae5a8`(L2b) + **속도튜닝**(`weighted A* w_heur 1.5→2.0`). 엔진 골든 불변(회랑은 `w_corridor>0` 일 때만 동작).

| 단계 | 내용 | 산출물 | 효과(project6 c100 ALL 208작업, **w_heur=2.0**) |
|---|---|---|---|
| **L1** | pgvector 학습 저장소 + Python 학습 파이프라인 | `db/schema/route_stub_pattern.sql`(feat `vector(24)`·dir_unit `vector(3)`·HNSW + 집계뷰 `route_stub_template`) · `routing3d_py/{route_db,pattern_learn,pattern_db}.py` | 405표본/38키, 도메인규칙 입증(EQUIP면=−z·DUCT면=+z), pytest 9/9 |
| **L1′ 엘보** | 스텁을 '수직배관까지'가 아니라 '수직 + 첫 엘보(수직→수평 전환)'까지로 — 방향 런 압축 + 짧은 런(<250mm) 지터 흡수 + 엘보 포함(`_dir_runs`·`_merge_short_runs`, `STUB_LEADIN_MM=800`) | `pattern_learn._walk_stub` 재작성 | 예) EQUIP `-z,-y,-z`→`-z,+x` · DUCT `+z,-z,+z`→`+z,-x`(엘보 방향이 dir2 에 인코딩). 라우팅 199 불변, pytest +2 |
| **L2a** | 학습면 PoC 투영(C#만, 엔진변경 0) | `Model/PatternStore.cs` + `SceneViewModel.LiftPocToSurface(preferFace)` | 기하 187 · 38s→23s(헛탐색 0) |
| **전처리** | 접근불가(파묻힌) PoC→최근접 자유셀 스냅 | `SceneViewModel.SnapPocToFreeCell`(학습면 행진+반경확장) | 187→**199** · 23s→6.3s |
| **L2b** | 기존설계 회랑 소프트바이어스(옵트인) | C ABI `r3d_set_corridor_cells` + corridor 키 `long long` 확장 + `SceneViewModel.{UseDesignCorridor,BuildDesignCorridorCells}` | 199(동일성공)·**설계추종**(totalLen↓)·6.4s |
| **속도튜닝** | weighted A* `w_heur` 1.5→2.0 + 임계 5M→300k셀 | `SceneViewModel.BuildEngineForRows` · `DbRouteDiag`(env `R3D_WCORR/WHEUR/CORRRAD`) | **회랑 ON 47s→6.2s(7.6×)** · 198→199 · 기하 baseline 도 194→199 |
| **L3a 랙번들** | 유틸그룹 수평 런 z-높이(랙) 학습 → 엔진 `rack_levels`(면제 z-셀)로 공용 랙에 번들. 랙 페널티 cell×0.2(회랑보다 부드럽게) | Python `pattern_learn.learn_rack_levels`(+`--rack-report`) · C# `SceneViewModel.BuildRackLevels`(런타임, env `R3D_RACK`) · UI '랙 번들(L3a)' | **199→200**(랙이 혼잡 배관에 구조 제공) · 랙 z-집중도 17.3%→21.4% · totalLen↓ · 6.4s |
| **L3b 검색증강** | 다중면(bimodal) DUCT 키에서 PoC 컨텍스트(앵커 내 rel·접근 dir) 최근접 표본으로 진입면 분기. **자기검증 LOO 게이트**(ANN 이 집계 다수결을 +10pp 이기는 키만 적용) | Python `pattern_db.{suggest_stub,StubSuggestion}` · C# `PatternStore.{LoadAnnSamples,TryGetFaceAnn}`+`SceneViewModel.LearnedDuctFace` · 진단 env `R3D_ANN` | **무분별 ANN 은 해로움**(199→192, UPW_S/HOT DI_S 에서 rel/dir 이 면과 반상관) → LOO 게이트로 **NFW 1키만 채택**(ANN 97% vs 집계 59%). 회귀 0(199 유지), 라우팅 지표 중립·면 품질↑ |
| **스텁 라우팅** | 매칭 기존배관 스텁(수직+엘보)을 **고정 설계 구간**으로 깔고 A* 는 스텁 끝(랙)~끝만 탐색. 표시=출발스텁+A*중간+종단스텁. 기존엔 PoC 에서 A* 가 스텁 무시·재탐색(=시각화 스텁과 라우팅 불일치) | C# `Model/StubExtractor.cs`(Python `_walk_stub` 포팅) + `SceneViewModel.BuildEngineForRows`(스텁 끝점 시작) + `TaskRowVM.{StartStub,EndStub}` · 진단 env `R3D_STUB` · UI '스텁 라우팅'(기본 ON) | **199→206**(스텁 208/208 매칭, 랙↔랙 자유공간 탐색이 쉬움) · 랙집중 17%→37% · 잔여 2=중간 혼잡(→ **동적 가중 A* 로 208/208 완전**, §9 참조) |

- **학습/적재 CLI**: `python -m routing3d_py.pattern_learn --project N {--report|--rack-report|--write-db}` · 통계 `python -m routing3d_py.pattern_db --stats` · 스키마 `--apply-schema`. (`--rack-report`=유틸그룹 랙 z-높이 학습, L3a)
- **검색증강(L3b)**: `pattern_db.suggest_stub(kind, group, util, feat, k)` → `StubSuggestion`(거리가중 K-NN 투표 면/평균 rise·offset·신뢰도). pgvector HNSW 본격 활용.
- **추론(C#)**: PoC가 속한 (장비·유틸)·(덕트·유틸) 키로 학습 면(EQUIP=−z·DUCT=+z 등) 조회 → 표면 투영 방향 결정. L2b는 매칭 기존배관(`FindMatchingExistingPipe`) 폴리라인을 회랑 셀로 주입.
- **UI 토글**: '기존설계 패턴'(학습면, 기본 ON) · '기존설계 회랑(L2b)'(기본 OFF). 미적재/키 미스 시 기존 기하 규칙으로 자동 폴백(무해, 회귀 0).
- **헤드리스 A/B**: `Routing3D.Viewer.exe --dbroute <proj> <cell> ALL <out>` + env `R3D_PATTERNS`/`R3D_SNAP`/`R3D_CORRIDOR`=off 로 각 단계 비교.
- **남은 실패**(project6 c100 10건)는 expanded>0(경로 없음=혼잡/막힘)으로 rip-up/CBS 영역 — 패턴 범위 밖.
- **개발계획 문서**: `docs/routing3d_pattern_learning_plan.{docx,pdf}`(생성기 `python_experiments/out/_gen_pattern_learning_plan.py`).
- **스텁 추출 알고리즘 상세문서**: `docs/routing3d_stub_extraction.{docx,pdf}`(런압축→지터흡수→엘보탐지→점열절단 16장, Python↔C# 1:1, 생성기 `_gen_stub_extraction_algorithm.py`).

### AI 자동설계 비교 리포트 (2026-06-07, P1 MVP)

메인장비별·(장비,유틸리티그룹) '케이스'마다 자동설계 2전략(**최단**=순수 A* · **Stub+그룹패턴**=스텁+번들/랙)을 수행하고 **기존설계**와 길이·꺾임·**그룹핑 Factor**를 비교하는 헤드리스 리포트.

- **CLI**: `Routing3D.Viewer.exe --autodesign-report <projectId> <cellMm> <outDir> [maxCases]` → `<group>_autodesign_report.{csv,txt,html}` + `img/`(스냅샷) + `_run.log`. maxCases=스모크 제한. env `R3D_ADR_NOIMG=1`=스냅샷/HTML 생략(빠른 지표만).
- **GUI 버튼**: 메인 툴바 '📊 자동설계 리포트'(`SceneViewModel.AutoDesignReportCommand`) — 선택 프로젝트/셀로 출력폴더 선택 후 실행, 완료 시 HTML 자동 오픈. 오프스크린 렌더가 STA(UI 스레드)에서만 동작하므로 **UI 스레드 동기 실행**(상태 1회 갱신 후 → 생성 중 화면 잠깐 멈춤, 케이스 많으면 수 분).
- **그룹핑 Factor**(0~1, 4성분 완성) = **0.35×랙집중도 + 0.30×번들밀집도 + 0.20×pitch일관성 + 0.15×레인정렬도**(가중 계획서값, N/A 성분은 가중 재정규화). pitch=같은 묶음(한 축·z, 평행배관 ≥3) 간격 CV→1−min(1,CV) 평균(미해상=N/A, cell>피치/2 면 흔함) · lane=각 배관 주(major) 수평 z-레벨을 공유하는 배관 비율. 랙집중·번들밀집은 DbRouteDiag 정의 재사용. CSV/TXT/HTML 에 4성분 개별 표기.
- **구현**: `Diagnostics/AutoDesignReport.cs`(케이스빌더·전략실행·지표·CSV/TXT/HTML·스냅샷). DbRouteDiag 순수 헬퍼(`MatchPipe`/`BuildRackLevels`/`MergeBundle`/`BuildBundleCorridor`/`D`)를 `internal` 재사용. 스텁 전략은 고정 스텁(라이저+엘보) 길이·꺾임을 엔진 결과에 가산해 공정 비교.
- **P4 3D 스냅샷(완료)**: `Diagnostics/OffscreenRenderer.cs` — 창 없이 `Viewport3D`+`RenderTargetBitmap`(소프트웨어 폴백) 오프스크린 렌더(STA=App.OnStartup). HelixToolkit `MeshBuilder`(박스=`AddBox`·배관=`AddTube`+`AddSphere`), Z-업 아이소 `PerspectiveCamera`. 케이스별 기존/최단/Stub+그룹 **3장을 같은 bounds(=동일 카메라)** 로 렌더해 다발화 차이를 그대로 비교. 색=`UtilityColors`(유틸 라벨 결정적), 맥락 박스=bounds 교차 장비/덕트만(혼잡 억제). 결과 폴리라인 캡처: 엔진 셀경로(`CW`) + 스텁 전략은 출발/종단 스텁(`StubExtractor`)을 앞뒤로 이어 '전체 설계' 표시. **자체 포함 HTML**(전체집계·케이스별 지표표·`img/` 3-up, 브라우저 인쇄=PDF).
- **실측(WTNHJ02 cell=200, 2성분 시절)**: 다발화 재현 검증(스냅샷에서도 육안 확인: 최단=산개·Stub+그룹=랙 레인 다발).
- **실측(WTNHJ02 cell=100 전체, 4성분, 8케이스·스냅샷 24장, ~3.5분)**: 그룹핑F 집계 **기존 0.563 > Stub+그룹 0.446 > 최단 0.248**(설계의도 정량 재현). 총길이 기존 900k < Stub+그룹 1.30M < 최단 1.43M(번들링이 길이도 단축). 성공 기존 151 · 최단 150/151 · Stub+그룹 148/151.
- **개발계획 문서**: `docs/routing3d_autodesign_report_plan.{docx,pdf}`(생성기 `_gen_autodesign_report_plan.py`).
- **공식 문서(docx/pdf/xlsx) 생성기**: `python_experiments/out/_gen_autodesign_report_doc.py --in <outDir> [--out <docx>] [--pdf]` — C# 출력폴더(CSV+img)를 읽어 `docs/routing3d_autodesign_report.{docx,xlsx}`(+`--pdf` 시 pdf) 생성. docx=전체집계표+케이스별 4성분 지표표+3D 스냅샷 3-up(이미지 임베드), xlsx=케이스별 시트(그룹핑F 최고 전략 초록 강조)+전체집계 시트. 서식 헬퍼 `_gen_spec_docs.py` 재사용, PDF=`_docx_to_pdf.ps1`(Word COM). 의존성 python-docx·openpyxl.
- **그룹핑 가중치 튜닝**: env `R3D_ADR_W="0.35,0.30,0.20,0.15"`(랙/밀집/pitch/lane, 합 무관·가용성분 재정규화) 로 재정의, 리포트 헤더에 사용 가중 로깅. 예: `R3D_ADR_W=1,0,0,0` → GF=랙집중만.
- **완료(2026-06-07)**: 4성분 그룹핑 Factor · GUI 버튼 · 전체 cell=100 배치 · docx/pdf/xlsx 생성기 · 가중치 env 튜닝. **남은 일**: 가중치 기본값 튜닝(데이터 누적 후).
- **빌드 주의**: 반드시 `dotnet build csharp/Routing3D.Viewer.sln -c Release`(솔루션=x64 → `bin/x64/Release`). csproj 직접 빌드는 `bin/Release`(다른 경로)라 실행 exe 와 불일치.

### 그룹(번들) 배관 탐지·활용 L4 (pgvector, 2026-06-02)

장비명·유틸리티별 기존배관 경로 **형태 유사도**로 '번들'(동일 이격간격으로 2회+ 수직/수평 꺾임 공유 평행 다발)을 탐지·저장하고, 신규 라우팅에 활용한다.

| 단계 | 내용 | 산출물 |
|---|---|---|
| **탐지** | 3단계: ①특징(방향런 압축·Arrow R/H/D·꺾임수·리샘플 방향벡터·extent) ②복합유사도(형태30% Levenshtein + 방향30% 코사인 + 길이20% + 규모20%) ③(owner,util) pre-filter→Union-Find(임계0.70)→**번들게이트(꺾임≥2 + pitch CV≤0.30)**→트렁크z·다발폭·pitch | `routing3d_py/bundle_detect.py` · `db/schema/route_bundle_group.sql`(테이블+집계뷰 `route_bundle_template`) · `route_db.ExistingPipe.owner_name` 추가 |
| **저장** | CLI `--project N\|--all --write-db\|--templates`. `--all`=DB 전체 순회 | **70프로젝트·353그룹·템플릿 275키 적재** |
| **신규설계 활용(C#)** | `Model/BundleStore.cs`(템플릿+멤버 guid→group_id) + `UseBundlePattern` 토글. **MergeBundleLevels**(유틸 trunk_z→rack_levels) + **BuildBundleCorridorCells 레인모드**(트렁크고도 ±1셀 수평런만 타이트 회랑 → 충돌회피가 인접레인 분산) | `SceneData.SourceFile` · DbRouteDiag env `R3D_BUNDLE` |
| **그룹배관 강조 표시** | 기존배관 중 번들 멤버를 **그룹별 고유색**(황금비 `BundleGroupColor`), 비멤버 흐리게. `ExistingPipe.RoutePathGuid`(member_guids 매칭) + `ShowBundleGroups` 토글 | UI 체크박스 '그룹배관 강조' |

- **실측(project6 ALL c100)**: 전부 OFF 199·rackZ 17.3% → **스텁+번들 레인 207·rackZ 39.7%**(corridor 28k셀, 옵션1 broad 118k 대비 4배 타이트).
- **핵심 한계**: cell(100) > pitch(~56mm) 면 인접 레인이 같은 셀로 뭉개져 물리적 패킹 불가 → **cell ≤ pitch/2(25~50mm) 필요**(cell=50 LPS rackZ 92%). route_multi(capi)는 이미 w_corridor>0 시 동적 자기번들링 + seed corridor 지원.
- **개발계획 문서**: `docs/routing3d_bundle_detection_plan.md`.

### 실데이터 교차검증 (Python = C++ = C#)

| 씬 | 결과 |
|---|---|
| `project6_c100.scene.txt` (cell=100, 장애물 983·작업 208) | **194/208 · 3,400,800mm — 3자 완전 일치** |
| `project6.scene.txt` (cell=200) | multi 77 / **ripup 80(+3)** — rip-up 실데이터 개선 실측 |
| 합성 혼잡 (9×9 벽+틈 2개) | seq 1/2 → ripup 2/2 (LONG 1300·SHORT 900) — C++/Python 동일 |

---

## 5. 테스트·CI

```powershell
ctest --test-dir cpp/build -C Release                                # C++ 9/9
.\.venv\Scripts\python.exe -m pytest python_experiments              # Python 203 + 11(multi_route)
.\Routing3D.Viewer.exe --selftest scene.txt out.txt                  # C# 헤드리스
```

회귀 시 가장 빨리 확인:
1. `ctest -R "golden|ripup|capi"` (수초)
2. `pytest python_experiments/tests/test_scenarios.py -v` (수초)
3. C# `--selftest` 로 project6 cell=100 → 194/208 그대로 확인

---

## 6. 문서·산출물

| 파일 | 내용 |
|---|---|
| `docs/development_plan.md` · `docs/phase{1,2,3}_plan.md` | 마스터·단계별 계획 |
| `docs/spec/algorithm_spec.md` + 4종 | Phase 2 동결 명세(불변식 포함) |
| `docs/routing3d_dev_report.{docx,pdf}` | 전체 + 단계별 개발보고서 (Phase 1~3 + 인터롭 5장 + 결론 6장) |
| `docs/routing3d_regression_report.{docx,pdf}` | Step 3.12 회귀 리포트 (실측+기대치 비교) |
| `docs/routing3d_cpp_engine_spec.{docx,pdf}` | **C++ 엔진 상세 개발문서** — 전체프로세스·핵심알고리즘·주요함수·변수(13장, 생성기 `_gen_cpp_engine_spec.py`) |
| `docs/routing3d_pattern_learning_plan.{docx,pdf}` | **기존설계 패턴 학습 개발계획 + 구현현황·실측**(P3j) |
| `docs/routing3d_stub_pattern.{docx,pdf}` | **스텁 패턴 기술 레퍼런스** — 출발/종단 스텁 프로세스·추출/특징 알고리즘·pgvector 데이터생성·자동설계 활용(L2a/L2b/L3a/L3b) |
| `docs/routing3d_stub_example_exhaust.{docx,pdf}` | **실측 워크드 예시** — Clean 장비 WTNHJ02_ Exhaust(ACID) 배관 1개(GUID 2014e40a, 150mm)로 출발(EQUIP −z)/종단(DUCT +z) 스텁 추출·특징벡터·집계대표·활용을 실제 좌표로 |
| `docs/csharp_helix_interop_design.md` | C ABI/뷰어 설계 + 로드맵 P0~P3j |
| `docs/phase2_input_notes.md` | Phase 2 동결 입력 노트 |
| 생성기 (gitignore 예외 추적) | `python_experiments/out/_gen_dev_report.py` · `_gen_regression_report.py` · `_gen_spec_docs.py` · `_gen_pattern_learning_plan.py` · `_gen_stub_pattern_doc.py` · `_gen_stub_example_exhaust.py` · `_gen_route_analysis_xlsx.py`(DDW 전수 분석 엑셀) · `_md_to_pdf.py` · `_docx_to_pdf.ps1` |

---

## 7. 핵심 규약

- **코드 문서화**: 한글 상세 주석 + 모든 모듈 상단에 실행명령어 블록 (기본 "주석 최소화" 규칙을 덮어씀)
- **인터롭 안전 규칙**: 예외는 C ABI 경계를 절대 넘지 않는다(try/catch → R3dStatus). 모든 문자열 UTF-8. POD blittable 구조체. cdecl. x64.
- **scene.txt 무손실**: `format_repr_double` 가 Python `repr(float)` 와 동일 표기. 선택 문자열은 `optional<string>` 으로 None(`\N`) vs `""` 구분(F3).
- **A* 결정성(A2/W1)**: (f, 삽입순서 counter) tie-break + 고정 이웃 순서 → 동일 입력 → 동일 경로/확장수.
- **다중배관(M1·M2)**: 성공 경로 셀 공유 0, 원본 점유맵 불변(`copy()` 사본 사용).
- **gitignore 예외**: `python_experiments/out/` 은 `*.py`/`*.ps1` 만 추적(생성기 소스). `cpp/tests/fixtures/*.scene.txt` 도 추적(LF 고정 `.gitattributes`).

---

## 8. 외부 시스템 참조

### PostgreSQL — DDW_AI_DB (2026-06-06 완전교체, 구 AUTOROUTINGV7 폐기)

기본: localhost / 5432 / postgres / dinno (로컬 dev). PGHOST 등 env 우선. **DB명 기본값 `DDW_AI_DB`**(`DbConfig.FromEnv`).

DDW_AI_DB 는 `SOURCE_FILE` 가 없고 좌표는 `AABB_MIN/MAX_*`(+OBB), PoC 는 jsonb 대신 별도 테이블, IS_MAIN→`MAIN_SUB_TYPE`. 작업·기존배관은 `TB_ROUTE_PATH` 가 정본(`SOURCE_GUID = TB_POCINSTANCES.INSTANCE_ID` 조인). 학습자산은 공식 AI 테이블 사용.

| 테이블 | 용도 | 키 컬럼 |
|---|---|---|
| `TB_SPACE_GROUP_INFO` | **프로젝트=툴그룹 목록** | TAG_GROUP_ID, TAG_GROUP_NM, BAY_GROUP_NM, PROCESS_GROUP_NM, AABB_MIN/MAX_* |
| `TB_BIM_OBSTACLE` | 장애물 AABB | AABB_MIN/MAX_*, INSTANCE_NAME, OST_TYPE, DDWORKS_TYPE, **COLLISION_PASS**(0/1 통과플래그) |
| `TB_EQUIPMENTS` | 장비(+PoC는 별도) | **MAIN_SUB_TYPE**('MainTool'/'SubTool'), AABB_MIN/MAX_*, EQUIPMENT_NAME |
| `TB_POCINSTANCES` | PoC 인스턴스 | **INSTANCE_ID**(=ROUTE_PATH.SOURCE_GUID), POC 좌표 평행배열, UTILITY/UTILITY_GROUP |
| `TB_LATERAL_PIPE` · `TB_DUCT` | 종단 객체(구 TB_DUCT_LATERAL 분리) | AABB_MIN/MAX_*, NAME, UTILITY, CATEGORY |
| `TB_SPACE_INFO` | 공간영역(와이어+라벨) | LEVEL_NAME, AABB_MIN/MAX_* |
| `TB_ROUTE_PATH` (+`_SEGMENTS`/`_SEGMENT_DETAIL`) | **작업·기존배관 정본** | ROUTE_PATH_GUID, **SOURCE_GUID**(→POC), UTILITY_GROUP, SOURCE/TARGET_POS, SOURCE/TARGET_OWNER_NAME, SOURCE_SIZE |
| `TB_ROUTE_DESIGN_GROUP` | **공식 번들그룹**(L4 대체) | GROUP_ID, EQUIPMENT_NAME, UTILITY_GROUP, UTILITY, MEMBER_COUNT, **MEMBER_ROUTE_GUIDS (text[])** |
| `TB_ROUTE_FEATURE_VECTOR`·`TB_ROUTE_SEGMENT_TEMPLATE`·`TB_ROUTE_NODES`·`TB_ROUTE_EDGES`·`TB_AUTO_DESIGN` | 공식 학습/자동설계 자산(우리 `route_stub_pattern` 대체 후보) | — |

확장: **pgvector / cube / postgis 설치됨**.

**작업 생성(완전교체)**: `TB_ROUTE_PATH`(SOURCE_POS→TARGET_POS, SOURCE_OWNER_NAME→PoC명, TARGET_OWNER_NAME→종단명)를 그룹 AABB 공간교차로 스코프해 작업화. PoC 메타는 `SOURCE_GUID = TB_POCINSTANCES.INSTANCE_ID` 조인.
**스코프 필터**: 선택 그룹(`TB_SPACE_GROUP_INFO`)의 AABB ± `ScopeMarginMm`(500) 공간교차로 장애물·장비·덕트·작업 한정.
**격자(중요)**: origin·shape 를 **그룹 AABB 박스(3축 전부)** 로 클램프(`ComputeGrid`) — 거대 공유 슬래브(건물 전체 X 0~443m·Z 48m)가 공간필터로 새어 28억 셀 폭발하던 것을 차단. 실측 그룹 WTNHJ02: 136×149×91 ≈ 184만 셀(Dense).

**뷰어 표시 보강(2026-06-06, PR #25~#31)**:
- **그룹 AABB 클리핑**: 공간영역(`TB_SPACE_INFO` 층, 건물 전체 443m)·장애물(바닥/천장 슬래브)을 그룹 AABB 3축으로 클리핑(작업영역 크기로 제한). **공간영역=와이어프레임**(cube box 아님), 장애물 클리핑은 **라우팅 불변**(엔진이 이미 장애물∩격자만 복셀화, 146/151·totalLen 동일, 1177→1139=작업 Z 밖 38개 제외).
- **배관 자재**: 흰색 cube 를 휴리스틱(폴리라인 정점)→**실제 부속**으로. `TB_ROUTE_SEGMENT_DETAIL.TYPE` 이 PIPE/POC/BENDING 이 아닌 것(ELBOW/TEE/VALVE/FLANGE/REDUCER/UNION/GLAND/GASKET/FILTER/TAKEOFF…)을 FROM/TO 중점에 타입별 색 cube 로, 독립 토글 `ShowFittings`(UI '배관자재'). 모델 `SceneData.PipeFitting`·로더 `LoadFittings`.
- **3D 클릭 정보 확장**: `SelectObjectAt` 에 배관 자재(작은 박스)·기존배관(중심선 거리 `DistPointToSeg`, 큰 장애물보다 우선·부속이 더 구체적)·PoC 마커(시작/종단 구)·**자동생성(라우팅) 배관**(2026-06-10 — `BuildRoutedPolyline`=렌더와 동일 [스텁]+[Path]+[종단스텁] 합성, 기존배관과 같은 중심선 거리 경쟁 → 가까운 쪽 선택, `DescribeRoutedPipe`=유틸·관경·길이·꺾임·경로셀수·시작/종단) 추가. 객체별 `Describe*` + 모달리스 `ObjectInfoWindow`. 좌측 그룹/유틸 필터·ShowPaths 동일 적용.
- **세그먼트 상세 탭(2026-06-10)**: 우측 하단 '분석결과'를 **TabControl 2탭**으로 — ① 분석결과(기존) ② **세그먼트 상세**. 결과 리스트에서 배관 선택 시 그 기존배관(`TaskRowVM.RoutePathGuid`=`TB_ROUTE_PATH.ROUTE_PATH_GUID`)의 `TB_ROUTE_SEGMENT_DETAIL`을 `(s.ORDER, sd.ORDER)` 순으로 조회(`ObstacleDbLoader.LoadSegmentDetail`)해 **#·종류(TYPE)·관경·Owner(소유 객체)** 를 DataGrid 로 표시(좌표 컬럼은 제거, 줌용 수치좌표만 내부 보유). Owner=세그먼트 INSTANCE_ID→`TB_POCINSTANCES.OWNER_INSTANCE_TYPE`(시작 POC=MAIN_EQUIPMENT·종단 POC=MODEL/DUCT…·중간 PIPE/ELBOW=`-`), 시작/종단 POC 는 라우트 헤더 실제 이름 보강(예 종단 `Damper-FMPVC-150A-Duct [MODEL]`). `SceneViewModel.{SelectedRouteSegments,SegmentDetailStatus,UpdateSegmentDetail}`(SelectedTask 변경 시 갱신). scene.txt 로드(GUID 없음)·DB예외 시 안내문구. **세그먼트 행 클릭 → 해당 객체 강조+위치 줌**(`SelectedSegment`→`FocusSegment`: FROM~TO AABB+여유로 `ShowHighlight`[노란 박스]+`ZoomToBoxRequested`. SegmentDetailRow 에 수치좌표 Fx..Tz+유효플래그. FROM/TO 는 기존배관 폴리라인과 동일 좌표라 강조가 배관에 정확히 안착). **검증**: 종단=Damper 인 배관은 SEGMENT_DETAIL 도 마지막 POC=댐퍼 PoC(=TARGET_GUID, 분포 동일 DUCT371·MODEL145·OTHER80) — 즉 TARGET_GUID 는 SEGMENT_DETAIL 종단 POC 의 비정규화 복사본.
- **종단 PoC 데이터 주의**: 종단명=`TB_ROUTE_PATH.TARGET_OWNER_NAME`, 위치=`TARGET_POS`, 실체=`TARGET_GUID`→`TB_POCINSTANCES`. **종단이 Duct 가 아니라 Damper/Elbow/Takeoff 로 보이는 건 정상**(설계상 배기 배관이 덕트 부속[댐퍼 등]에 접속). 실측 Exhaust 종단: DUCT 669·Damper 122·PIPE 88·… 댐퍼 소유자는 `OWNER_INSTANCE_TYPE='MODEL'`(TB_DUCT/OBSTACLE 에 없어 화면엔 PoC 만).
- **전수 분석 엑셀**: `python python_experiments/out/_gen_route_analysis_xlsx.py` → `out/ddw_route_analysis.xlsx`(전체 7,052 경로 + '종단소유자 분석' 교차표). 의존성 openpyxl. `TB_ROUTE_PATH` + `TB_POCINSTANCES` 조인(PoC 소유자타입·관경 보강).

### SpaceAI (인접 프로젝트, `..\SpaceAI\`)

C# 직교 A* + 동일 DB. UI 스타일·DB 흐름 참조용(직접 포팅 안 함).
다크 팔레트 `#1e2230 / #252b3d / #2b3548 / #404a64`, 강조 `#385b85` — Routing3D 뷰어가 동일 팔레트 채택(P3d).

---

## 9. 다음 작업 후보

- ~~**배관-배관 시각/물리 충돌 회피(옵션1)**~~: **완료(2026-06-10)**. 엔진 충돌-회피는 '셀 공유 0'만 보장했고 깔린 배관을 `mark_pipe(…,0)`(중심선)으로만 점유시켜, 실제 관경으로 렌더하면 인접 1셀(=cell_mm) 간격 배관 표면이 겹쳤다(특히 cell≥pitch). 수정: ① 엔진에 `pipe_radius` 상태 + `r3d_set_pipe_radius` C ABI → `route_multi(_progress)`/`RouteCorridorMulti` 가 깔린 배관을 ±radius 6-이웃까지 막아 다음 배관 중심선을 관경만큼 띄움. ② 뷰어 `ComputePipeRadiusCells`(대상 유틸 기존배관 관경 P90 → `R≥d/cell−1`, [0,6] 클램프, env `R3D_PIPE_RADIUS` 재정의)로 산출해 `BuildEngineForRows`/hier 호출에 전달. ③ 팽창이 인접 종단을 묻어 `exp=0` 즉시실패가 나던 것을 종단 스냅 반경 `2+pipe_radius` 로 보강. ④ **기본 셀 50→25mm**(팽창을 물리적으로 실현하려면 셀 ≤ 관경 이격). ⑤ **거대격자 탐색상한 12M → env `R3D_MAX_EXP` 상향 가능**(32GB+ 서버: 25mm 어려운 배관 커버리지; 메모리는 확장 노드 해시맵에 비례). **실측(--dbroute 1 25 Exhaust, 20작업)**: radius=0 20/20 → radius=3 19/20(#12 묻힘) → **스냅 보강 후 20/20**(번들밀집 4.5%→0.4%=배관 띄워짐). 골든/capi/implicit/ripup ctest 불변(radius=0 경로 무변경).
  - **혼잡 마지막배관 실패 회복(2026-06-10 후속)**: GUI 그룹 모드 실측에서 마지막 배관(#146 ALKA 150A, 직선 2,277mm·91셀인데 12M 상한 도달=경로없음)이 실패 — 원인=① pipe_radius 팽창이 25mm 다발의 마지막 배관 진입로를 좁힘 ② 12M 상한 ③ **그룹 모드(`w_corridor>0`)에서 `use_hier`가 꺼져** 계층 corridor escalation 미적용 ④ bounded greedy A* 함정. **수정 4종**: ⓐ `large_grid_cap()` 기본 **12M→48M**(GUI도 적용; env `R3D_MAX_EXP`), ⓑ **`use_hier = large_grid`**(회랑 모드도 ON — probe 는 회랑 바이어스 유지[쉬운 배관 번들 보존], 예산 초과 어려운 배관만 hier escalate[연결 우선]), ⓒ **C# 반경 = 대표 관경 max 기준, 클램프 [0,6]**(처음엔 #146 회피로 [0,3]까지 낮췄으나 그러면 150mm 관경이 100mm 간격으로 과소이격돼 **다시 겹쳤다**[사용자 재지적] — rip-up 이 생긴 지금은 관경에 맞춰 R=5[150mm 간격]를 줘도 성공 유지), ⓓ **route_multi_impl 내 인라인 rip-up 회복**(main 패스 후 실패 배관의 장애물-only 이상경로를 막는 placed blocker[≤4]를 뜯어 재배치, 무손실·결정적·`build_work`로 M1 보존, env `R3D_RIPUP=off`). **실측(--dbroute 1 25 Exhaust, R3D_CORRIDOR=on)**: R3D_PIPE_RADIUS=3 20/20이나 150mm 겹침 → **R3D_PIPE_RADIUS=5 20/20·번들밀집 0.0%(완전 분리)·170s**. ctest 6/6 불변(작은 골든격자는 large_grid=false → rip-up/hier 미적용). **남은 한계**: ① 단일 글로벌 반경(max)이라 가는 배관 그룹은 과패킹 가능 → 진짜 해법은 per-배관 관경 반경(RouteTask 에 관경 추가) ② **고정 스텁(라이저)은 엔진이 mark 안 함**(stub 끝~끝만 탐색·마킹) → 종단 근처 스텁끼리 겹칠 수 있음(엔진은 스텁 미인지) ③ 자동은 공용 랙 1평면에 모으는데 사람설계는 z-레인 분리(수직 스택)라 근본적으로 다름.
  - **남은 일**: ALL 전체 25mm 실측(작업당 ~2M 확장·느림)으로 혼잡 그룹 성공률·과패킹 확인 후 P90/클램프(3)·`R3D_MAX_EXP`(48M) 기본값 튜닝. rip-up 회복이 느릴 수 있음(실패당 48M×blocker 재라우팅).
  - **굵은 배관 우선 라우팅(2026-06-11)**: 굵은 배관이 마지막에 깔려(거리순 "longest" 정렬에서 수직하강=최단=뒤) 작은 배관 다발 사이를 우회·충돌하던 문제(사용자 지적: 우측 사각영역 큰배관↔작은배관 충돌, 큰배관은 수직하강이 최단인데 둘아감). 수정: `RouteTask.diameter_mm` 필드 추가 + C ABI `r3d_set_task_diameter` + 우선순위 **"diameter"**(굵은 배관 먼저, 동률은 거리 긴 것) 신설 + **"utility"** 2차키에 관경 추가(유틸 묶음 안에서도 굵은 배관 먼저). 굵은 배관이 직선 경로를 선점하고 가는 배관이 그 곁을 피한다. 정렬은 `route_multi_impl`(capi 인라인)·`order_indices`(multi_route, ripup/corridor_multi) **둘 다** 갱신. 뷰어 `_priority` "longest"→**"diameter"**, `BuildEngineForRows` 가 `ResolveTaskDiameter`(행 캐시 or 매칭 기존배관 SOURCE_SIZE)로 각 작업 관경을 라우팅 전 확정해 `SetTaskDiameter` 전달. `DbRouteDiag` 도 동일(기본 "diameter", env `R3D_PRIORITY` A/B). **관경 미상(0)이면 전 작업 동률 → "longest"와 동일**(골든/capi/ripup ctest 11/11 불변). **실측(--dbroute 1 100 Exhaust, 20/20)**: longest=totalLen 321,100·turns 140·비싼성공 11 → **diameter=316,900·135·10**(굵은관 직선화로 길이·꺾임·우회 ↓). per-배관 관경 반경(다음 후보)의 선행 토대(`RouteTask` 에 관경 확보).
- ~~**패턴학습 L2b 속도 튜닝**~~: **완료** — 회랑 페널티 비용장을 휴리스틱이 과소평가하던 게 병목. weighted A* `w_heur` 1.5→2.0 + weighted 임계 5M→300k셀로 **회랑 ON 47s→6.2s(7.6× 단축)**, 성공 198→199, 회랑 OFF baseline 도 194→199. 엔진 무변경(뷰어 파라미터). 잔여 9실패는 혼잡/막힘(아래 CBS).
- ~~**패턴학습 L3**~~: **완료**. L3a 랙 번들링(유틸그룹 수평 런 z-높이→`rack_levels`, 199→200·랙 집중도 17→21%) + L3b 검색증강(다중면 DUCT 키 PoC별 ANN 면 분기 + **자기검증 LOO 게이트** = ANN 이 집계를 이기는 키만 적용, 무분별 ANN 회귀 199→192 방지, NFW 1키 채택). **교훈**: 한 키 내 진입면 분기는 [rel,dir] 로 항상 예측되진 않음(UPW_S/HOT DI_S 반상관) → 게이트 필수. 추가 특징(접근 세그먼트 방향열·인접 장애물)로 분기 정밀도 향상이 다음 후보.
- ~~**정밀 셀 탐색량 최적화**~~: **대부분 해결(2026-06-02)**. 실측으로 옛 "10mm ~110s/배관"은 **스텁 라우팅+ImplicitOccupancy+weighted A* 이전 수치**임이 확인됨 — 현재 project6 ALL 208: **cell=50 0.14s·cell=25 0.5s·cell=10 7.7s(스텁ON, 208/208 매칭)**. 스텁이 랙↔랙 짧은 구간만 탐색해 미세격자 탐색량을 근본 해소. 추가로 **계층 corridor(coarse 가이드→fine 튜브 하드 제한)를 escalation 게이트로 구현**(`astar_weighted` 에 `in_corridor` 술어[기본 AllowAll → 골든 불변] + capi `route_hier`/`coarse_implicit_from_doc`). 게이트: 저예산(300k) 직접 A* 먼저 → 초과하는 '어려운 배관'만 hier. **쉬운 배관(개방 랙 직선)은 직접 A* 그대로**라 cell≤25 회귀 0·품질 바이트 동일, **최악 cell=10 스텁OFF 41s→28s(1.47×)**. ctest 11/11 불변.
- ~~**가중(ε)A* 추가 튜닝**~~: **완료(2026-06-02) — 동적(수렴) 가중 A***. 정적 `w_heur=2.0`(공격적 그리디)은 목표/PoC 근처 혼잡·막다른길에서 함정에 빠져 마지막 접근 경로를 놓쳤다. `RouteParams.w_heur_near` 추가 → 휴리스틱 가중을 목표까지 거리비로 보간(`w_eff = w_heur_near + (w_heur−w_heur_near)·h/h_start`): **먼 곳=2.0(빠름)·목표 근처=1.0(표준 A*, 정확)**. 준최적 상한은 여전히 `w_heur`. 핵심 게이트: **무제한 탐색(`max_expansions≤0`)에서만 적용** — 예산-게이트(probe 300k·hier)에서 목표근처 가중을 낮추면 예산 초과로 escalation 폭주(실측 cell=25 1.3s→33s)하므로 거대격자(항상 상한)에선 자동 비활성. **실측 project6 c100: 스텁ON 206→208(완전!)·스텁OFF 199→203, 시간 동일 / cell=25·10 무영향(자동 off) / 골든 불변(`w_heur=1` 이면 dyn off, ctest 10/10)**. 뷰어 기본 ON(weighted 일 때 `w_heur_near=1.0`), env `R3D_WHEUR_NEAR`(0=끔)로 재정의.
- ~~**독립 배관 병렬화**~~: **시도·기각(2026-06-02)**. optimistic 병렬(Phase A: 모든 배관을 마크 없는 점유에 동시 라우팅 → Phase B: 우선순위 순 커밋, 충돌 시 마크된 점유에 재라우팅)을 구현. **순차와 바이트 동일**(정확성 검증됨)했으나 **project6 c100/c25/c10 전부 wall-clock 이득 0~음수**(c10 26.1s↔26.8s). 원인: 미세격자 A* 는 거대 해시맵(g/came/closed)을 스트리밍하는 **메모리대역 바운드**라 스레드들이 대역을 경합(스케일 안 됨) + Phase A 가 '마크 없는' 더 큰 탐색을 중복 수행해 병렬 이득 상쇄. → 코드 제거(순차 유지). **교훈: 미세격자 라우팅 가속은 스레드 병렬이 아니라 탐색량 자체 축소(스텁·동적 가중·hier)가 정답.**
- ~~**접근불가 PoC 전처리**~~: **완료(P3j 전처리)** — 파묻힌 PoC를 학습면 행진+반경확장으로 최근접 자유셀 스냅(project6 c100 +7 복구). 남은 실패는 혼잡/막힘(아래 CBS 대상).
- **negotiated-congestion / CBS**: 비용기반 충돌 회피 — rip-up 의 더 강력한 후속. project6 c100 잔여 10실패(expanded>0=경로 없음)가 이 대상.
- **P3b' OpenVDB capi**: VDB 백엔드를 C ABI 로 노출 + 런타임 DLL 동봉 (Sparse로 목표 충족돼 보류 중)
- ~~**DDW_AI_DB 전환**~~: **뷰어 완전교체 완료(2026-06-06)**. `ObstacleDbLoader`(C#) 전면 재작성 — `TB_SPACE_GROUP_INFO`(프로젝트=툴그룹)·`TB_BIM_OBSTACLE`(COLLISION_PASS 통과플래그·damper 제외)·`TB_EQUIPMENTS`(MAIN_SUB_TYPE)·`TB_POCINSTANCES` 조인·`TB_LATERAL_PIPE`+`TB_DUCT`·`TB_ROUTE_PATH`(작업·기존배관 정본, SOURCE_GUID=INSTANCE_ID). `BundleStore`→`TB_ROUTE_DESIGN_GROUP`. **격자 3축 그룹 AABB 클램프**로 셀 폭발 차단(28억→184만, 313s→139s, 146/151). 검증: `--dbroute 1 100 ALL R3D_STUB=on R3D_BUNDLE=on` → 번들 149키·스텁 151/151. **남은 일**: `PatternStore`→공식 `TB_ROUTE_SEGMENT_TEMPLATE`(현재 null 폴백, 스텁라우팅은 `StubExtractor` 직접 사용이라 무해) · Python 레퍼런스(`obstacle_db`/`scene`/`route_db`/`pattern_learn`/`bundle_detect`) 재작성(뷰어가 실행 주체라 후순위).

---

## 10. 디렉토리 구조

```
Routing3D/
├── CLAUDE.md                         # ← 이 파일(프로젝트 컨텍스트)
├── run.ps1                           # C++ 빌드+CLI 래퍼
├── README.md
├── .venv/                            # Python 환경
├── db/schema/route_stub_pattern.sql       # ← pgvector 학습 저장소 스키마(P3j)
├── docs/
│   ├── development_plan.md  phase{1,2,3}_plan.md  phase2_input_notes.md
│   ├── spec/{algorithm,scene_format,regression_set,performance_targets,freeze_signoff}.md
│   ├── csharp_helix_interop_design.md
│   └── routing3d_{dev,regression,pattern_learning_plan}.{docx,pdf}
├── python_experiments/
│   ├── routing3d_py/{occupancy,astar,cost,multi_route,scene_io,obstacle_db,scene,viz,viz_scene}.py
│   ├── routing3d_py/{route_db,pattern_learn,pattern_db}.py    # ← 패턴 학습(P3j)
│   ├── tests/{test_*.py, test_pattern_learn.py, scenarios/, scenario_runner.py}
│   ├── experiments/baseline_params.json
│   └── out/{_gen_dev_report.py, _gen_regression_report.py, _gen_spec_docs.py, _gen_pattern_learning_plan.py, _docx_to_pdf.ps1}
├── cpp/
│   ├── CMakeLists.txt
│   ├── include/routing3d/{geometry,occupancy,box_index,cost,astar,multi_route,corridor,scene_io,fcl_scene,route_task}.hpp
│   ├── cli/routing3d_cli.cpp                  → routing3d_cli.exe
│   ├── bindings/bindings.cpp                  → routing3d_cpp.pyd
│   ├── capi/routing3d_capi.{h,cpp}            → routing3d_capi.dll  (r3d_set_corridor_cells = L2b)
│   ├── tests/{test_golden, test_scene_io, test_occupancy, test_corridor, test_implicit, test_ripup, test_attract, test_capi, test_vdb, test_fcl}
│   └── build/                                 # gitignored
└── csharp/Routing3D.Viewer/
    ├── Routing3D.Viewer.csproj                # net9.0-windows, x64, HelixToolkit.Wpf 2.24.0, Npgsql 8.0.4
    ├── App.{xaml,.xaml.cs}                    # OnStartup: --selftest / --dbroute / scene.txt 인자 / DB 자동
    ├── MainWindow.{xaml,.xaml.cs}             # SpaceAI 다크 3-컬럼 (패턴/회랑 토글)
    ├── Interop/{Native, R3dEngineHandle, Engine}.cs
    ├── Model/{SceneData, SceneTextParser, UtilityColors, CollisionFinder, ObstacleDbLoader, PatternStore}.cs
    ├── Diagnostics/DbRouteDiag.cs             # 헤드리스 라우팅 진단(--dbroute, A/B env)
    └── ViewModels/{SceneViewModel, TaskRowVM, UtilityFilterVM, ObservableObject, RelayCommand}.cs
```

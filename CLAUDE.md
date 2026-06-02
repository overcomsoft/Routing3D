# Routing3D — 프로젝트 컨텍스트

이 파일은 다른 PC / 새 세션에서 프로젝트를 재개할 때 필요한 핵심 정보를 한곳에 모은 것이다.
세부 사항은 git 이력과 `docs/`, `cpp/`, `csharp/`, `python_experiments/` 가 정답.

> 마지막 갱신: 2026-06-02 · 단위 mm · 기본 셀 50mm

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
- **DB**: localhost / 5432 / AUTOROUTINGV7 / postgres / dinno (로컬 dev). 운영 환경에서는 PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD 환경변수 우선 — **소스에 비밀번호 두지 말 것**.

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
| **스텁 라우팅** | 매칭 기존배관 스텁(수직+엘보)을 **고정 설계 구간**으로 깔고 A* 는 스텁 끝(랙)~끝만 탐색. 표시=출발스텁+A*중간+종단스텁. 기존엔 PoC 에서 A* 가 스텁 무시·재탐색(=시각화 스텁과 라우팅 불일치) | C# `Model/StubExtractor.cs`(Python `_walk_stub` 포팅) + `SceneViewModel.BuildEngineForRows`(스텁 끝점 시작) + `TaskRowVM.{StartStub,EndStub}` · 진단 env `R3D_STUB` · UI '스텁 라우팅'(기본 ON) | **199→206**(스텁 208/208 매칭, 랙↔랙 자유공간 탐색이 쉬움) · 랙집중 17%→37% · 잔여 2=중간 혼잡 |

- **학습/적재 CLI**: `python -m routing3d_py.pattern_learn --project N {--report|--rack-report|--write-db}` · 통계 `python -m routing3d_py.pattern_db --stats` · 스키마 `--apply-schema`. (`--rack-report`=유틸그룹 랙 z-높이 학습, L3a)
- **검색증강(L3b)**: `pattern_db.suggest_stub(kind, group, util, feat, k)` → `StubSuggestion`(거리가중 K-NN 투표 면/평균 rise·offset·신뢰도). pgvector HNSW 본격 활용.
- **추론(C#)**: PoC가 속한 (장비·유틸)·(덕트·유틸) 키로 학습 면(EQUIP=−z·DUCT=+z 등) 조회 → 표면 투영 방향 결정. L2b는 매칭 기존배관(`FindMatchingExistingPipe`) 폴리라인을 회랑 셀로 주입.
- **UI 토글**: '기존설계 패턴'(학습면, 기본 ON) · '기존설계 회랑(L2b)'(기본 OFF). 미적재/키 미스 시 기존 기하 규칙으로 자동 폴백(무해, 회귀 0).
- **헤드리스 A/B**: `Routing3D.Viewer.exe --dbroute <proj> <cell> ALL <out>` + env `R3D_PATTERNS`/`R3D_SNAP`/`R3D_CORRIDOR`=off 로 각 단계 비교.
- **남은 실패**(project6 c100 10건)는 expanded>0(경로 없음=혼잡/막힘)으로 rip-up/CBS 영역 — 패턴 범위 밖.
- **개발계획 문서**: `docs/routing3d_pattern_learning_plan.{docx,pdf}`(생성기 `python_experiments/out/_gen_pattern_learning_plan.py`).
- **스텁 추출 알고리즘 상세문서**: `docs/routing3d_stub_extraction.{docx,pdf}`(런압축→지터흡수→엘보탐지→점열절단 16장, Python↔C# 1:1, 생성기 `_gen_stub_extraction_algorithm.py`).

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
| `docs/routing3d_pattern_learning_plan.{docx,pdf}` | **기존설계 패턴 학습 개발계획 + 구현현황·실측**(P3j) |
| `docs/routing3d_stub_pattern.{docx,pdf}` | **스텁 패턴 기술 레퍼런스** — 출발/종단 스텁 프로세스·추출/특징 알고리즘·pgvector 데이터생성·자동설계 활용(L2a/L2b/L3a/L3b) |
| `docs/routing3d_stub_example_exhaust.{docx,pdf}` | **실측 워크드 예시** — Clean 장비 WTNHJ02_ Exhaust(ACID) 배관 1개(GUID 2014e40a, 150mm)로 출발(EQUIP −z)/종단(DUCT +z) 스텁 추출·특징벡터·집계대표·활용을 실제 좌표로 |
| `docs/csharp_helix_interop_design.md` | C ABI/뷰어 설계 + 로드맵 P0~P3j |
| `docs/phase2_input_notes.md` | Phase 2 동결 입력 노트 |
| 생성기 (gitignore 예외 추적) | `python_experiments/out/_gen_dev_report.py` · `_gen_regression_report.py` · `_gen_spec_docs.py` · `_gen_pattern_learning_plan.py` · `_gen_stub_pattern_doc.py` · `_gen_stub_example_exhaust.py` · `_docx_to_pdf.ps1` |

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

### PostgreSQL — AUTOROUTINGV7

기본: localhost / 5432 / postgres / dinno (로컬 dev). PGHOST 등 env 우선.

| 테이블 | 용도 | 키 컬럼 |
|---|---|---|
| `space_project_map` | 프로젝트 목록 | project_id, source_file, process, equipment_code |
| `TB_BIM_OBSTACLES` | 장애물 AABB | SOURCE_FILE, MIN/MAX_X/Y/Z (mm), OST_TYPE, NAME, OBJECT_ID, DDWORKS_TYPE |
| `TB_BIM_EQUIPMENT` | 메인 장비 + PoC | SOURCE_FILE, IS_MAIN, EQ_ID, NAME, MIN/MAX_X/Y/Z, **POC_LIST (jsonb)** |
| `TB_DUCT_LATERAL` | 종단 객체(시각화) | SOURCE_FILE, OBJECT_ID, NAME, UTILITY, MIN/MAX_X/Y/Z |
| `TB_ROUTE_PATH` (+`_SEGMENTS`/`_SEGMENT_DETAIL`) | 기존 설계배관 폴리라인(패턴 학습 입력) | ROUTE_PATH_GUID, UTILITY_GROUP, SOURCE_UTILITY, SOURCE/TARGET_POS, SOURCE_OWNER_NAME/POS, SOURCE_SIZE |
| `route_stub_pattern` (+뷰 `route_stub_template`) | **학습 스텁 패턴 저장소(pgvector, 우리가 생성)** | anchor_kind/utility/face/dir_seq/rise · `feat vector(24)`·`dir_unit vector(3)`·HNSW |

확장: **pgvector / cube / postgis 설치됨**(2026-06-02 확인). 스키마 적용 `python -m routing3d_py.pattern_db --apply-schema`(소스 `db/schema/route_stub_pattern.sql`).

**POC_LIST jsonb 구조**: 각 PoC = `{id, name, pocPosition:[x,y,z], utility, utilityGroup, isConnected, endPocs:[{endName, endType, endPocGuid, endInstanceGuid, endPocPosition:[x,y,z]}, …]}`.
`connectedOnly=true`(기본) 면 `isConnected=true` 만 작업으로 만든다.
**격자**: `origin = lo`, `shape = ceil((hi-lo)/cell)` (장애물 BBOX).

### SpaceAI (인접 프로젝트, `..\SpaceAI\`)

C# 직교 A* + 동일 DB. UI 스타일·DB 흐름 참조용(직접 포팅 안 함).
다크 팔레트 `#1e2230 / #252b3d / #2b3548 / #404a64`, 강조 `#385b85` — Routing3D 뷰어가 동일 팔레트 채택(P3d).

---

## 9. 다음 작업 후보

- ~~**패턴학습 L2b 속도 튜닝**~~: **완료** — 회랑 페널티 비용장을 휴리스틱이 과소평가하던 게 병목. weighted A* `w_heur` 1.5→2.0 + weighted 임계 5M→300k셀로 **회랑 ON 47s→6.2s(7.6× 단축)**, 성공 198→199, 회랑 OFF baseline 도 194→199. 엔진 무변경(뷰어 파라미터). 잔여 9실패는 혼잡/막힘(아래 CBS).
- ~~**패턴학습 L3**~~: **완료**. L3a 랙 번들링(유틸그룹 수평 런 z-높이→`rack_levels`, 199→200·랙 집중도 17→21%) + L3b 검색증강(다중면 DUCT 키 PoC별 ANN 면 분기 + **자기검증 LOO 게이트** = ANN 이 집계를 이기는 키만 적용, 무분별 ANN 회귀 199→192 방지, NFW 1키 채택). **교훈**: 한 키 내 진입면 분기는 [rel,dir] 로 항상 예측되진 않음(UPW_S/HOT DI_S 반상관) → 게이트 필수. 추가 특징(접근 세그먼트 방향열·인접 장애물)로 분기 정밀도 향상이 다음 후보.
- ~~**정밀 셀 탐색량 최적화**~~: **대부분 해결(2026-06-02)**. 실측으로 옛 "10mm ~110s/배관"은 **스텁 라우팅+ImplicitOccupancy+weighted A* 이전 수치**임이 확인됨 — 현재 project6 ALL 208: **cell=50 0.14s·cell=25 0.5s·cell=10 7.7s(스텁ON, 208/208 매칭)**. 스텁이 랙↔랙 짧은 구간만 탐색해 미세격자 탐색량을 근본 해소. 추가로 **계층 corridor(coarse 가이드→fine 튜브 하드 제한)를 escalation 게이트로 구현**(`astar_weighted` 에 `in_corridor` 술어[기본 AllowAll → 골든 불변] + capi `route_hier`/`coarse_implicit_from_doc`). 게이트: 저예산(300k) 직접 A* 먼저 → 초과하는 '어려운 배관'만 hier. **쉬운 배관(개방 랙 직선)은 직접 A* 그대로**라 cell≤25 회귀 0·품질 바이트 동일, **최악 cell=10 스텁OFF 41s→28s(1.47×)**. ctest 11/11 불변. 잔여 = 가중(ε)A*/독립 배관 병렬화(미착수).
- ~~**접근불가 PoC 전처리**~~: **완료(P3j 전처리)** — 파묻힌 PoC를 학습면 행진+반경확장으로 최근접 자유셀 스냅(project6 c100 +7 복구). 남은 실패는 혼잡/막힘(아래 CBS 대상).
- **negotiated-congestion / CBS**: 비용기반 충돌 회피 — rip-up 의 더 강력한 후속. project6 c100 잔여 10실패(expanded>0=경로 없음)가 이 대상.
- **P3b' OpenVDB capi**: VDB 백엔드를 C ABI 로 노출 + 런타임 DLL 동봉 (Sparse로 목표 충족돼 보류 중)
- **DDW_AI_DB 전환(보류)**: 새 DB는 스키마 전면 재설계(`TB_BIM_OBSTACLE`/`TB_EQUIPMENTS`/`TB_LATERAL_PIPE`/`TB_DUCT`, source_file 없음, POC 평행 텍스트 배열, 작업 생성이 장비 PoC↔lateral/duct 3-테이블 매칭). `ObstacleDbLoader`(C#)·Python `obstacle_db`/`scene` 재작성 필요. 현재는 AUTOROUTINGV7 사용.

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

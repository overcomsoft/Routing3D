# Routing3D — 프로젝트 컨텍스트

이 파일은 다른 PC / 새 세션에서 프로젝트를 재개할 때 필요한 핵심 정보를 한곳에 모은 것이다.
세부 사항은 git 이력과 `docs/`, `cpp/`, `csharp/`, `python_experiments/` 가 정답.

> 마지막 갱신: 2026-05-30 · 단위 mm · 기본 셀 50mm

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
| 3.11 | 벤치 · 최적화 — **미착수** | — |
| **3.12** | **회귀 리포트** — 표준 벤치 자동 측정·기대치 비교 → `docs/routing3d_regression_report.{docx,pdf}` | 골든 3/3 PASS |

- **CLI**: `routing3d_cli` (코어만, demo/route/summary 명령, `--mode multi\|single\|ripup`)
- **DLL**: `routing3d_capi.dll` (외부 의존성 0, 261KB)
- **ctest 9/9**: golden · scene_io · occupancy · corridor · **ripup** · capi · vdb · fcl · bindings

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
| P3b' | OpenVDB capi (선택, 보류) |

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
| `docs/csharp_helix_interop_design.md` | C ABI/뷰어 설계 + 로드맵 P0~P3f |
| `docs/phase2_input_notes.md` | Phase 2 동결 입력 노트 |
| 생성기 (gitignore 예외 추적) | `python_experiments/out/_gen_dev_report.py` · `_gen_regression_report.py` · `_gen_spec_docs.py` · `_docx_to_pdf.ps1` |

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

**POC_LIST jsonb 구조**: 각 PoC = `{id, name, pocPosition:[x,y,z], utility, utilityGroup, isConnected, endPocs:[{endName, endType, endPocGuid, endInstanceGuid, endPocPosition:[x,y,z]}, …]}`.
`connectedOnly=true`(기본) 면 `isConnected=true` 만 작업으로 만든다.
**격자**: `origin = lo`, `shape = ceil((hi-lo)/cell)` (장애물 BBOX).

### SpaceAI (인접 프로젝트, `..\SpaceAI\`)

C# 직교 A* + 동일 DB. UI 스타일·DB 흐름 참조용(직접 포팅 안 함).
다크 팔레트 `#1e2230 / #252b3d / #2b3548 / #404a64`, 강조 `#385b85` — Routing3D 뷰어가 동일 팔레트 채택(P3d).

---

## 9. 다음 작업 후보

- **3.11 벤치 · 최적화**: corridor 폭/해상도 튜닝, 클리어런스 로컬화, 라우팅에 FCL 통합, 독립 배관 병렬화
- **접근불가 PoC 전처리**: 종단 PoC가 장애물에 파묻혔을 때 스냅 반경 확장 / 표면 투사(rip-up으로는 구조상 해소 불가)
- **negotiated-congestion / CBS**: 비용기반 충돌 회피 — rip-up 의 더 강력한 후속
- **P3b' OpenVDB capi**: VDB 백엔드를 C ABI 로 노출 + 런타임 DLL 동봉 (Sparse로 목표 충족돼 보류 중)

---

## 10. 디렉토리 구조

```
Routing3D/
├── CLAUDE.md                         # ← 이 파일(프로젝트 컨텍스트)
├── run.ps1                           # C++ 빌드+CLI 래퍼
├── README.md
├── .venv/                            # Python 환경
├── docs/
│   ├── development_plan.md  phase{1,2,3}_plan.md  phase2_input_notes.md
│   ├── spec/{algorithm,scene_format,regression_set,performance_targets,freeze_signoff}.md
│   ├── csharp_helix_interop_design.md
│   └── routing3d_{dev,regression}_report.{docx,pdf}
├── python_experiments/
│   ├── routing3d_py/{occupancy,astar,cost,multi_route,scene_io,obstacle_db,scene,viz,viz_scene}.py
│   ├── tests/{test_*.py, scenarios/, scenario_runner.py}
│   ├── experiments/baseline_params.json
│   └── out/{_gen_dev_report.py, _gen_regression_report.py, _gen_spec_docs.py, _docx_to_pdf.ps1}
├── cpp/
│   ├── CMakeLists.txt
│   ├── include/routing3d/{geometry,occupancy,cost,astar,multi_route,corridor,scene_io,fcl_scene,route_task}.hpp
│   ├── cli/routing3d_cli.cpp                  → routing3d_cli.exe
│   ├── bindings/bindings.cpp                  → routing3d_cpp.pyd
│   ├── capi/routing3d_capi.{h,cpp}            → routing3d_capi.dll
│   ├── tests/{test_golden, test_scene_io, test_occupancy, test_corridor, test_ripup, test_capi, test_vdb, test_fcl, test_bindings.py}
│   └── build/                                 # gitignored
└── csharp/Routing3D.Viewer/
    ├── Routing3D.Viewer.csproj                # net9.0-windows, x64, HelixToolkit.Wpf 2.24.0, Npgsql 8.0.4
    ├── App.{xaml,.xaml.cs}                    # OnStartup: --selftest / scene.txt 인자 / DB 자동
    ├── MainWindow.{xaml,.xaml.cs}             # SpaceAI 다크 3-컬럼
    ├── Interop/{Native, R3dEngineHandle, Engine}.cs
    ├── Model/{SceneData, SceneTextParser, UtilityColors, CollisionFinder, ObstacleDbLoader}.cs
    └── ViewModels/{SceneViewModel, TaskRowVM, UtilityFilterVM, ObservableObject, RelayCommand}.cs
```

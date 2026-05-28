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

## 실행 (프로젝트 루트 = `Routing3D/` 에서)

패키지를 한 번 **editable 설치**하면, 작업 디렉토리·`PYTHONPATH` 와 무관하게
`python -m routing3d_py.*` 가 바로 동작한다.

```powershell
# 최초 1회만: 가상환경 + editable 설치
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e python_experiments        # 핵심 + DB 로더
.\.venv\Scripts\python.exe -m pip install -e "python_experiments[viz]" # 3D 시각화(PyVista)
```

```powershell
# 점유맵 3D 시각화 → 스크린샷 PNG (창 없이)
.\.venv\Scripts\python.exe -m routing3d_py.viz `
    --region 195000 8000 14000 205000 12000 16000 --cell-mm 50 `
    --screenshot python_experiments\out\occupancy.png

# 전체 복셀 격자(빈 셀 포함) + 점유셀을 함께 보기 (작은 영역 권장)
.\.venv\Scripts\python.exe -m routing3d_py.viz `
    --region 199800 9000 14000 200400 10200 15500 --cell-mm 50 `
    --all-voxels --edges --opacity 0.55 --show

# 인터랙티브 창으로 보기
.\.venv\Scripts\python.exe -m routing3d_py.viz --region 195000 8000 14000 205000 12000 16000 --show

# 장애물 DB 통계
.\.venv\Scripts\python.exe -m routing3d_py.obstacle_db --stats

# 전체 테스트
.\.venv\Scripts\python.exe -m pytest python_experiments -v
```

> editable 설치를 하지 않았다면 `python_experiments/` 디렉토리 안에서 실행하거나
> `$env:PYTHONPATH="python_experiments"` 를 설정하면 동일하게 동작한다.



## 현재 상태

- 2026-05-27: 프로젝트 스켈레톤 생성.
- 2026-05-27: Phase 1 세부 개발계획서 — [docs/phase1_plan.md](docs/phase1_plan.md).
- 2026-05-27: Step 1.1 점유맵(Dense/Sparse/BitPacked 백엔드) + 장애물 DB 로더(PostgreSQL `TB_BIM_OBSTACLES`) + 3D 시각화(PyVista) 구현·검증.
- 2026-05-27: Step 1.2 직교 A*(6방향, heapq, 맨해튼 휴리스틱) 구현·검증 — 실DB 영역 경로 렌더 확인.
- 2026-05-27: Step 1.3 비용함수(turn penalty / 클리어런스 근접 페널티 / 단 분리, `astar_weighted`) 구현·검증.
- 2026-05-27: 라우팅 씬 데이터 환경(`scene.py`) — SpaceAI 프로젝트 단위(`space_project_map`)로 장애물·메인장비·종단·PoC페어 로드, 유틸리티별 그룹. project 6에서 208 페어/21 유틸 확인, A* 라우팅·렌더 검증. 다음 = Step 1.4 다중 배관.

## 단위 규약

좌표·치수는 **밀리미터(mm)** 기준. 기본 셀 크기 50mm (`RouteParams.cell_mm` 으로 설정 가능). 자세한 좌표 규약은 [docs/phase1_plan.md §4](docs/phase1_plan.md#4-데이터-모델과-좌표-규약) 참조.

## 인접 프로젝트

- `..\SpaceAI\` — 선행 프로젝트. C# 자동 설계 엔진 + WPF 뷰어 + Hybrid v2 라우팅. 본 Routing3D 는 알고리즘 측면에서 SpaceAI 의 직교 A* 와 같은 계열이나, **신규 개발**로 Python 실험 → C++ 엔진화 트랙으로 새로 시작한다. SpaceAI 코드를 직접 포팅하지 않는다.

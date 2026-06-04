// 뷰모델 — C++ 엔진(routing3d_capi) 라우팅 + HelixToolkit 3D + 인터랙티브 재라우팅(P2) + 충돌/토글/3D피킹(P3)
// =============================================================================
// [이 파일이 하는 일]
//   scene.txt(또는 내장 데모)를 읽어 격자/장애물/작업을 파싱하고, 같은 장면을 C++ 엔진에
//   적재해 라우팅한 뒤(엔진=C++, 뷰어=C#), 결과 경로를 받아 HelixToolkit Model3DGroup
//   (장애물 반투명 박스 + 유틸리티별 경로 튜브 + 충돌 셀 큐브)으로 만든다.
//   P2: 작업 선택 → 종단점 편집 → 단일/전체 재라우팅.
//   P3: 표시 토글(장애물/경로/충돌), 충돌 셀 시각화, 3D 클릭으로 종단점 지정.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;
using System.Windows.Threading;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using Routing3D.Viewer.Interop;
using Routing3D.Viewer.Model;
using Routing3D.Viewer.Views;

namespace Routing3D.Viewer.ViewModels
{
    /// <summary>3D 클릭 피킹 모드.</summary>
    public enum PickMode { None, Start, End }

    /// <summary>경로 탐색 범위 — 전체 한 번에 / 유틸리티그룹 1개 / 유틸리티 1개.</summary>
    public enum RouteScope { All, ByGroup, ByUtility }

    /// <summary>범위 콤보 항목(라벨 + enum).</summary>
    public sealed class RouteScopeOption
    {
        public RouteScope Scope { get; init; }
        public string Label { get; init; } = string.Empty;
        public override string ToString() => Label;
    }

    /// <summary>범례 항목(색 견본 + 라벨).</summary>
    public sealed class LegendItem
    {
        public Brush Swatch { get; init; } = Brushes.Gray;
        public string Label { get; init; } = string.Empty;
    }

    /// <summary>3D 공간 영역 텍스트 라벨(코드비하인드가 BillboardText 로 렌더). 위치는 월드 mm.</summary>
    public sealed class SpaceLabel
    {
        public string Text { get; init; } = string.Empty;
        public Point3D Position { get; init; }
        public Color Color { get; init; } = Colors.White;
    }

    /// <summary>선택 경로의 한 직선 구간(단계) — 방향/길이 라벨 + 시작 월드좌표(클릭 시 이동 대상).</summary>
    public sealed class PathStep
    {
        public string Label { get; init; } = string.Empty;
        public Point3D Position { get; init; }
        public Point3D A { get; init; }   // 직선 구간 시작 월드좌표(구간 강조용).
        public Point3D B { get; init; }   // 직선 구간 끝 월드좌표.
        public override string ToString() => Label;
    }

    public sealed class SceneViewModel : ObservableObject
    {
        private Engine? _engine;
        private SceneData? _scene;
        private readonly string _priority = "longest";

        // ── 3D 객체 클릭 → 속성 정보 표시 ───────────────────────────────
        // 씬은 원본 mm 좌표로 렌더되므로(ApplyPick 이 픽 점을 그대로 mm 로 사용),
        // 클릭 지점을 포함하는 객체를 AABB 포함 검사로 찾는다. 표시 중(레이어 켜짐)인
        // 객체만 후보로 삼고, 겹치면 부피가 가장 작은(가장 구체적인) 객체를 고른다.
        private string? _selectedObjectInfo;
        public string? SelectedObjectInfo
        {
            get => _selectedObjectInfo;
            private set => Set(ref _selectedObjectInfo, value);
        }

        public void SelectObjectAt(Point3D p)
        {
            var s = _scene;
            if (s is null) { SelectedObjectInfo = null; return; }

            string? best = null; double bestVol = double.MaxValue;
            Point3D blo = default, bhi = default;
            void Consider(string text, double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            {
                const double eps = 1.0;
                if (p.X < mnx - eps || p.X > mxx + eps) return;
                if (p.Y < mny - eps || p.Y > mxy + eps) return;
                if (p.Z < mnz - eps || p.Z > mxz + eps) return;
                double vol = Math.Max(1, mxx - mnx) * Math.Max(1, mxy - mny) * Math.Max(1, mxz - mnz);
                if (vol < bestVol) { bestVol = vol; best = text; blo = new Point3D(mnx, mny, mnz); bhi = new Point3D(mxx, mxy, mxz); }
            }

            if (ShowEquipment)
                foreach (var e in s.Equipment)
                    Consider(DescribeEquipment(e), e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);

            foreach (var d in s.DuctsLaterals)
            {
                if (d.IsLateral ? !ShowLaterals : !ShowDucts) continue;
                Consider(DescribeDuct(d), d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
            }

            if (ShowObstacles)
                for (int i = 0; i < s.Obstacles.Count; i++)
                {
                    var o = s.Obstacles[i];
                    Consider(DescribeObstacle(i, o), o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                }

            // 공간영역(A/F·CSF·CR 등)은 씬 전체를 덮는 거대 AABB 라 클릭 선택 대상에서 제외한다.
            // (와이어프레임·라벨 렌더는 ShowSpaces 로 그대로 유지 — 표시는 하되 클릭으로는 잡히지 않음.)

            SelectedObjectInfo = best;
            if (best is null) { HighlightModel = null; Status = "선택된 객체 없음(빈 공간 클릭)"; }
            else { ShowHighlight(blo, bhi); Status = "객체를 선택했습니다."; }
        }

        // 선택한 객체 AABB 를 밝은 노란 와이어프레임 + 반투명 박스로 강조한다.
        private void ShowHighlight(Point3D lo, Point3D hi)
        {
            var grp = new Model3DGroup();
            double dx = hi.X - lo.X, dy = hi.Y - lo.Y, dz = hi.Z - lo.Z;
            double diag = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            double r = Math.Max(6.0, diag * 0.004);   // 선택선 굵기(얇게).
            AddBoxFrame(grp, lo, hi, Colors.Yellow, r, 255);
            var fill = new MeshBuilder(false, false);
            fill.AddBox(new Point3D((lo.X + hi.X) / 2, (lo.Y + hi.Y) / 2, (lo.Z + hi.Z) / 2), dx, dy, dz);
            grp.Children.Add(Geometry(fill, Colors.Yellow, 55));
            HighlightModel = grp;
        }

        private static string F(double v) => v.ToString("F0", System.Globalization.CultureInfo.InvariantCulture);

        private static string Dims(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            => $"크기(mm): {F(mxx - mnx)} × {F(mxy - mny)} × {F(mxz - mnz)}\n"
             + $"중심(mm): ({F((mnx + mxx) / 2)}, {F((mny + mxy) / 2)}, {F((mnz + mxz) / 2)})";

        private static string DescribeEquipment(EquipmentBox e)
            => "[장비]\n"
             + $"이름: {(string.IsNullOrEmpty(e.Name) ? "(이름없음)" : e.Name)}\n"
             + $"메인 장비: {(e.IsMain ? "예" : "아니오")}\n"
             + Dims(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);

        private static string DescribeDuct(DuctLateral d)
            => $"[{(d.IsLateral ? "레터럴" : "덕트")}]\n"
             + $"이름: {(string.IsNullOrEmpty(d.Name) ? "(이름없음)" : d.Name)}\n"
             + $"CATEGORY: {d.Category}\n"
             + $"UTILITY: {(string.IsNullOrEmpty(d.Utility) ? "N/A" : d.Utility)}\n"
             + Dims(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);

        private static string DescribeObstacle(int i, ObstacleBox o)
            => $"[장애물 #{i}]\n"
             + $"이름: {(string.IsNullOrEmpty(o.Name) ? "(이름없음)" : o.Name)}\n"
             + $"OST_TYPE: {(string.IsNullOrEmpty(o.OstType) ? "N/A" : o.OstType)}\n"
             + $"DDWORKS_TYPE: {(string.IsNullOrEmpty(o.DdworksType) ? "N/A" : o.DdworksType)}\n"
             + $"통과 객체: {(o.IsPassThrough ? "예 (경로탐색 통과)" : "아니오")}\n"
             + Dims(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ) + "\n"
             + $"AABB(mm): ({F(o.MinX)}, {F(o.MinY)}, {F(o.MinZ)})\n"
             + $"        ~ ({F(o.MaxX)}, {F(o.MaxY)}, {F(o.MaxZ)})";

        private static string DescribeSpace(SpaceArea sp)
            => "[공간영역]\n"
             + $"이름: {sp.Name}\n"
             + Dims(sp.MinX, sp.MinY, sp.MinZ, sp.MaxX, sp.MaxY, sp.MaxZ);

        private Model3D? _sceneModel;
        private string _status = string.Empty;
        private TaskRowVM? _selectedTask;
        private PickMode _pickMode = PickMode.None;
        private bool _showObstacles = true;
        private bool _showPaths = true;
        private bool _showCollisions = true;
        private bool _showGridFrame = false;        // 복셀 전체맵(격자 BBOX 와이어).
        private bool _showOccupancyVoxels = false;  // 점유맵(복셀화된 장애물 셀).
        private bool _occupancyFullRes = false;     // 점유맵 해상도: true=원본(전체 셀), false=다운샘플(상한).
        private bool _showVisitedMap = false;       // 방문맵(A* 확장 셀, 유틸리티 색).
        private bool _showSpaces = true;            // 공간 영역(CR/A/F/CSF) 와이어프레임 + 텍스트.
        private bool _showEquipment = true;         // 장비(TB_BIM_EQUIPMENT) 큐브 박스.
        private bool _showLaterals = true;          // 레터럴(TB_DUCT_LATERAL, CATEGORY=LATERAL) 박스.
        private bool _showDucts = true;             // 덕트(TB_DUCT_LATERAL, CATEGORY=DUCT) 박스.
        private bool _showExistingPipes = true;     // 기존 설계배관(TB_ROUTE_PATH) 폴리라인(유틸리티 색).
        private bool _showPocMarkers = true;        // 모든 작업의 시작 PoC(빨강)·종단 PoC(파랑) 마커(초기 표시).
        private bool _showStubs = true;             // 기존설계 배관의 출발(빨강)·종단(파랑) 스텁(수직+엘보) 강조.
        private bool _showBundleGroups;             // 그룹배관 강조 — 탐지 번들(route_bundle_group) 멤버를 그룹별 색으로.
        private readonly bool _includeFacilities = true;  // 충돌확장: 설비·덕트·레터럴 + 기설계 배관을 장애물로. 항상 ON 고정(readonly).
        // 기존설계 패턴(pgvector) — 학습된 진출/진입 면으로 시작/종단 PoC 를 투영(L2a). null=미적재(기하 폴백).
        private PatternStore? _patterns;
        private bool _patternsTried;                    // DB 1회만 조회(미스도 캐시).
        private bool _usePatterns = true;               // 기존설계 패턴 활용 토글(기본 ON, 미스 시 자동 기하 폴백).
        // 기존설계 회랑(L2b) — 매칭되는 기존 설계배관(TB_ROUTE_PATH) 폴리라인을 회랑 시드로 주입하고
        // w_corridor 로 회랑 밖을 가산 → 새 경로가 사람 설계를 부드럽게 따라간다(충돌은 여전히 회피).
        // 경로 '모양'을 바꾸므로 기본 OFF(옵트인). ON 시 BuildEngineForRows 가 회랑 셀 주입 + w_corridor>0.
        private bool _useDesignCorridor;
        // 유틸그룹 랙 번들링(L3a) — 기존배관의 수평 런이 모이는 z-높이(랙)를 그룹별로 학습해 엔진
        // rack_levels(= w_corridor 면제 z-셀)로 준다. 같은 그룹 새 배관이 공용 랙 높이에 뭉친다(사람 설계다움).
        // rack_levels 는 w_corridor>0 일 때만 효력(랙 밖 가산) → ON 시 BuildEngineForRows 가 가벼운 w_corridor 부여.
        private bool _useRackBundling;
        // 그룹배관 패턴(L4) — Python bundle_detect 가 DB(route_bundle_template)에 저장한 '대표 그룹배관 패턴'
        // (같은 유틸 배관들이 동일 이격간격·2회+ 꺾임으로 공유하는 공용 트렁크 고도)을 읽어, 신규 라우팅 시
        // 같은 유틸 새 배관을 그 트렁크 고도(rack_levels)에 뭉치게 한다. 미적재/키 미스면 자동 폴백(무해).
        // 기본 ON(항상 적용) — 학습 패턴이 없으면 폴백이라 무해하고, 있으면 사람 설계처럼 공용 랙에 다발링.
        private BundleStore? _bundles;
        private bool _bundlesTried;
        private bool _useBundlePattern = true;
        // 스텁 라우팅 — 매칭 기존배관의 출발/종단 스텁(수직+엘보)을 '고정 설계 구간'으로 깔고, A* 는 스텁 끝~끝
        // (랙 위 자유공간)만 탐색한다. 표시 경로 = [출발 스텁] + [A* 중간] + [종단 스텁]. 매칭 배관 없으면 PoC
        // 직접 라우팅으로 폴백. 학습 스텁과 자동설계를 일치시킨다(기존엔 PoC 에서 A* 가 스텁을 무시하고 재탐색).
        private bool _useStubRouting = true;
        private bool _useHierarchicalCorridor = false;  // false=route_multi(가중 A*, 고품질). 엔진 astar_weighted 의 closed 가 해시 기반이 되어 대형 격자(25mm 1.3억 셀)에서도 OOM 없이 동작. true=계층 corridor(이 장면에선 대부분 실패해 비권장).
        private string _searchText = string.Empty;
        private bool _suppressFilterRebuild;   // BuildTaskRows 중 IsVisible 이벤트 폭주 방지.

        // DB 접속 설정(환경변수 우선) + 선택된 프로젝트 / 격자 셀 크기.
        private readonly DbConfig _dbConfig = DbConfig.FromEnv();
        private ProjectInfo? _selectedProject;
        private double _cellMm = 25.0;   // 라우팅 격자 셀 크기(mm). 작을수록 정밀하지만 셀 수가 (비율)³ 으로 폭증.
        private bool _suppressProjectAutoLoad;

        // 경로 탐색 범위(모두/그룹별/유틸별) + 선택 대상(그룹/유틸 1개).
        private RouteScopeOption _selectedRouteScope;
        private string? _selectedRouteTarget;

        // 좌측 드릴다운(그룹 → 유틸리티 → 개별 PoC) 선택 상태.
        private string? _selectedGroup;
        private string? _selectedUtility;
        private Model3D? _selectionModel;   // 선택 PoC 강조(시작/끝 마커) 오버레이.
        private Model3D? _searchModel;      // A* 단계별 탐색(방문 셀 점진 표시) 오버레이.
        private Model3D? _highlightModel;   // 3D 클릭으로 선택한 객체 강조(노란 박스) 오버레이.

        // ── 기존설계 비교(선택 배관) ────────────────────────────────────
        // 선택 배관에 매칭된 기존 설계경로(주황)와 개발 경로(시안)를 동시에 그려 정량 비교한다.
        private Model3D? _compareModel;        // 비교 오버레이(기존=주황 / 개발=시안 굵은 튜브).
        private string? _comparisonReport;     // 우측 '기존설계 비교 분석' 패널 텍스트.
        private bool _compareMode;             // 비교 포커스: true 면 나머지 기존배관 레이어 숨김.
        private ExistingPipe? _comparePipe;    // 현재 선택에 매칭된 기존 경로(없으면 null).
        private bool _hidePathsForAnim;     // 단계별 탐색 중 최종 경로를 숨겼다가 끝에 드러내기.
        private bool _animating;            // 단계별 탐색 진행 중(중복 실행 방지).

        public SceneViewModel(string? initialScene = null)
        {
            OpenCommand = new RelayCommand(Open);
            DemoCommand = new RelayCommand(LoadDemo);
            RunRouteCommand = new RelayCommand(() => _ = RunRouteAsync(), () => _scene != null);
            RerouteCorridorCommand = new RelayCommand(
                () => _ = RouteRowsAsync(AllRows(), "기존설계 유사(그룹배관)", corridor: true),
                () => _scene != null);
            // 단건 라우팅도 그룹배관 모드(corridor:true) — 학습 공용 트렁크 회랑(L4)+매칭 기존배관(L2b)을 따라
            //   1개 배관도 그룹 트렁크에 정렬된다. (단건은 셀 재적재 없이 = 다른 누적 경로 보존, 셀 가드는 다건만.)
            RerouteSelectedCommand = new RelayCommand(
                () => { if (_selectedTask != null) _ = RouteRowsAsync(new List<int> { _selectedTask.Index }, $"선택 #{_selectedTask.Index}", corridor: true); },
                () => _selectedTask != null);
            CompareSelectedCommand = new RelayCommand(
                () => _ = CompareSelectedAsync(),
                () => _selectedTask != null && _scene != null);
            AnimateSelectedCommand = new RelayCommand(
                () => _ = AnimateSelectedAsync(),
                () => _selectedTask != null && _scene != null && !_animating);
            PickStartCommand = new RelayCommand(() => SetPick(PickMode.Start), () => _selectedTask != null);
            PickEndCommand = new RelayCommand(() => SetPick(PickMode.End), () => _selectedTask != null);
            FitViewCommand = new RelayCommand(() => FitViewRequested?.Invoke());
            ToggleOccupancyResCommand = new RelayCommand(() => OccupancyFullRes = !OccupancyFullRes);
            UtilityAllCommand = new RelayCommand(() => SetAllUtilities(true));
            UtilityClearCommand = new RelayCommand(() => SetAllUtilities(false));
            LoadProjectsCommand = new RelayCommand(() => _ = LoadProjectsAsync());
            LoadDbCommand = new RelayCommand(
                () => { if (_selectedProject != null) _ = LoadFromDbAsync(_selectedProject.ProjectId); },
                () => _selectedProject != null);
            // 좌측 드릴다운 선택을 한꺼번에 라우팅 — 선택 그룹/유틸리티의 모든 PoC를 부분집합 충돌회피 라우팅.
            // 그룹/유틸 전체 라우팅 = 그룹배관 모드(corridor:true) — 같은 유틸을 공용 트렁크에 다발로 묶는다
            //   (스텁+그룹 트렁크 회랑+유틸 순서+강한 w_corridor, 셀>50이면 자동 50mm). 단건이 아니라 '묶음'을
            //   라우팅하므로 기존설계처럼 그룹으로 나오는 게 기대값.
            RouteGroupCommand = new RelayCommand(
                () => { if (!string.IsNullOrEmpty(_selectedGroup))
                            _ = RouteRowsAsync(RowsWhere(t => GroupKey(t.Group) == _selectedGroup),
                                               $"그룹 '{_selectedGroup}'", corridor: true, showProgress: true); },
                () => _scene != null && !string.IsNullOrEmpty(_selectedGroup));
            RouteUtilityCommand = new RelayCommand(
                () => { if (!string.IsNullOrEmpty(_selectedGroup) && !string.IsNullOrEmpty(_selectedUtility))
                            _ = RouteRowsAsync(RowsWhere(t => GroupKey(t.Group) == _selectedGroup &&
                                                              UtilityKey(t.Utility) == _selectedUtility),
                                               $"유틸리티 '{_selectedUtility}'", corridor: true, showProgress: true); },
                () => _scene != null && !string.IsNullOrEmpty(_selectedGroup) && !string.IsNullOrEmpty(_selectedUtility));
            // 자동설계된(라우팅된) 모든 경로 삭제 — 결과 캐시 초기화 + 라이브 오버레이 제거 + 재렌더.
            ClearRoutesCommand = new RelayCommand(ClearAllRoutes,
                () => _scene != null && Tasks.Any(t => t.Success));

            TasksView = CollectionViewSource.GetDefaultView(Tasks);
            TasksView.Filter = TaskFilter;

            _selectedRouteScope = RouteScopes[0];   // 기본 '모두'.

            try
            {
                if (!string.IsNullOrEmpty(initialScene) && File.Exists(initialScene))
                {
                    // scene.txt 인자 / --selftest 경로: 동기 로드(셀프테스트가 vm.Status 를 즉시 읽음).
                    LoadFile(initialScene);
                }
                else
                {
                    // DB 자동 로드는 무거우므로(목록 조회 + 첫 프로젝트 전체 라우팅) 생성자에서 하지 않는다.
                    // 창이 먼저 뜬 뒤(MainWindow 의 ContentRendered) RunStartupLoadAsync 가 비동기로 수행한다.
                    // 그렇지 않으면 라우팅이 끝날 때까지 창이 아예 보이지 않는다.
                    NeedsStartupLoad = true;
                    Status = "시작 중… 창 표시 후 DB 자동 로드";
                }
            }
            catch (Exception ex) { Status = "엔진 초기화 오류: " + ex.Message; }
        }

        /// <summary>생성자에서 DB 자동 로드를 보류했는지(=scene 인자 없이 실행). 창이 뜬 뒤 코드비하인드가 확인.</summary>
        public bool NeedsStartupLoad { get; private set; }

        /// <summary>창이 처음 렌더된 뒤 호출(코드비하인드). DB 자동 로드를 비동기로 수행하고,
        /// 실패하면 내장 데모로 폴백한다 — 무거운 라우팅 동안 UI 가 멈추거나 빈 화면이 되지 않게.</summary>
        public async Task RunStartupLoadAsync()
        {
            NeedsStartupLoad = false;
            await LoadProjectsAsync();
            if (_scene == null) LoadDemo();   // DB 실패/빈 목록 → 데모 폴백.
        }

        // ---- 바인딩 속성 ----
        public Model3D? SceneModel { get => _sceneModel; private set => Set(ref _sceneModel, value); }
        public string Status { get => _status; private set => Set(ref _status, value); }
        public TaskRowVM? SelectedTask
        {
            get => _selectedTask;
            set
            {
                if (!Set(ref _selectedTask, value)) return;
                bool wasCompare = _compareMode;
                _compareMode = false;            // 새 배관 선택 → 비교 포커스 해제(숨겼던 기존배관 복원).
                UpdateSelectionHighlight();      // → UpdateComparison(): 새 배관 매칭 오버레이/분석.
                if (wasCompare && _scene != null && _engine != null) BuildModel();
            }
        }
        public PickMode PickMode { get => _pickMode; private set => Set(ref _pickMode, value); }

        public bool ShowObstacles { get => _showObstacles; set { if (Set(ref _showObstacles, value)) RebuildIfReady(); } }
        public bool ShowPaths { get => _showPaths; set { if (Set(ref _showPaths, value)) RebuildIfReady(); } }
        public bool ShowCollisions { get => _showCollisions; set { if (Set(ref _showCollisions, value)) RebuildIfReady(); } }
        public bool ShowGridFrame { get => _showGridFrame; set { if (Set(ref _showGridFrame, value)) RebuildIfReady(); } }
        public bool ShowOccupancyVoxels { get => _showOccupancyVoxels; set { if (Set(ref _showOccupancyVoxels, value)) RebuildIfReady(); } }
        public bool ShowVisitedMap { get => _showVisitedMap; set { if (Set(ref _showVisitedMap, value)) RebuildIfReady(); } }
        public bool ShowSpaces { get => _showSpaces; set { if (Set(ref _showSpaces, value)) RebuildIfReady(); } }
        public bool ShowEquipment { get => _showEquipment; set { if (Set(ref _showEquipment, value)) RebuildIfReady(); } }
        public bool ShowLaterals { get => _showLaterals; set { if (Set(ref _showLaterals, value)) RebuildIfReady(); } }
        public bool ShowDucts { get => _showDucts; set { if (Set(ref _showDucts, value)) RebuildIfReady(); } }
        public bool ShowExistingPipes { get => _showExistingPipes; set { if (Set(ref _showExistingPipes, value)) RebuildIfReady(); } }
        /// <summary>모든 작업(장비)의 시작 PoC(빨강 구)·종단 PoC(파랑 구) 마커 — 라우팅 전에도 초기 표시. 기본 ON.</summary>
        public bool ShowPocMarkers { get => _showPocMarkers; set { if (Set(ref _showPocMarkers, value)) RebuildIfReady(); } }
        /// <summary>기존설계 배관의 출발(빨강)·종단(파랑) 스텁(수직배관+엘보)을 굵은 색 튜브로 강조. 기본 ON.
        /// 학습 파이프라인(StubExtractor)과 동일 로직으로 잘라내 학습 스텁과 일치한다.</summary>
        public bool ShowStubs { get => _showStubs; set { if (Set(ref _showStubs, value)) RebuildIfReady(); } }

        /// <summary>그룹배관 강조 — 현재 로드된 기존배관 중 탐지된 번들(route_bundle_group) 멤버를 '그룹별 고유 색'으로
        /// 그리고 비멤버는 흐리게 표시한다. 동일 이격간격·2회+ 꺾임 다발이 한눈에 보인다. DB 적재(bundle_detect
        /// --write-db) 필요. 미적재면 비활성(상태 라벨에 표시). 기본 OFF.</summary>
        public bool ShowBundleGroups
        {
            get => _showBundleGroups;
            set { if (Set(ref _showBundleGroups, value)) { OnChanged(nameof(BundleGroupStatus)); RebuildIfReady(); } }
        }

        /// <summary>그룹배관 강조 상태 표시(UI 라벨).</summary>
        public string BundleGroupStatus =>
            !_showBundleGroups ? "그룹배관 강조: OFF"
            : _bundles == null || _bundles.GroupCount == 0 ? "그룹배관 강조: 없음(미적재)"
            : $"그룹배관 강조: {_bundles.GroupCount}그룹";

        /// <summary>충돌확장 — 라우팅 시 설비(메인 장비 포함)·덕트·레터럴 + 이미 설계된(라우팅 성공) 다른
        /// 배관의 경로를 장애물로 추가해 충돌을 피한다. <b>항상 ON 고정(표준 라우팅 동작, 토글 잠금)</b> —
        /// getter 전용이라 UI 에서 끌 수 없다(체크박스는 켜진 채 비활성).</summary>
        public bool IncludeFacilities => _includeFacilities;   // _includeFacilities 는 항상 true(고정).

        /// <summary>기존설계 패턴 활용 — 학습된 진출/진입 면(장비=-z·덕트=+z 등)으로 시작/종단 PoC 를
        /// 투영해 자동라우팅을 사람 설계에 맞춘다. 패턴 저장소(pgvector route_stub_pattern)가 비었거나
        /// 키 미스면 자동으로 기존 최근접-면 규칙으로 폴백(무해). 기본 ON.</summary>
        public bool UsePatterns
        {
            get => _usePatterns;
            set { if (Set(ref _usePatterns, value)) OnChanged(nameof(PatternStatus)); }
        }

        /// <summary>기존설계 회랑(L2b) — 매칭 기존배관 폴리라인을 회랑으로 주입해 새 경로가 사람 설계를
        /// 부드럽게 따라가게 한다(w_corridor 소프트 바이어스, 충돌은 여전히 회피). 경로 모양을 바꾸므로 기본 OFF.</summary>
        public bool UseDesignCorridor
        {
            get => _useDesignCorridor;
            set { if (Set(ref _useDesignCorridor, value)) OnChanged(nameof(PatternStatus)); }
        }

        /// <summary>유틸그룹 랙 번들링(L3a) — 기존배관 수평 런이 모이는 z-높이(랙)를 그룹별로 학습해 같은
        /// 그룹 새 배관을 공용 랙 높이에 뭉치게 한다(엔진 rack_levels + 가벼운 w_corridor). 경로 모양을 바꾸므로 기본 OFF.</summary>
        public bool UseRackBundling
        {
            get => _useRackBundling;
            set { if (Set(ref _useRackBundling, value)) OnChanged(nameof(PatternStatus)); }
        }

        /// <summary>그룹배관 패턴(L4) — DB 에 저장된 대표 그룹배관 패턴(공용 트렁크 고도)을 읽어, 같은 유틸
        /// 새 배관을 학습된 공용 랙 고도에 뭉치게 한다(엔진 rack_levels + 가벼운 w_corridor). 미적재/미스 시
        /// 자동 폴백(무해). 기본 ON(항상 적용).</summary>
        public bool UseBundlePattern
        {
            get => _useBundlePattern;
            set { if (Set(ref _useBundlePattern, value)) OnChanged(nameof(BundleStatus)); }
        }

        /// <summary>그룹배관 패턴 저장소 상태 표시(UI 라벨).</summary>
        public string BundleStatus =>
            !_useBundlePattern ? "그룹배관 패턴: OFF"
            : _bundles == null ? "그룹배관 패턴: 없음(미적재)"
            : $"그룹배관 패턴: {_bundles.Count}키";

        /// <summary>스텁 라우팅 — 매칭 기존배관의 출발/종단 스텁(수직+엘보)을 고정 설계 구간으로 깔고 A* 는
        /// 스텁 끝~끝만 탐색(표시 = 스텁+중간+스텁). 매칭 없으면 PoC 직접 라우팅으로 폴백. 기본 ON.</summary>
        public bool UseStubRouting
        {
            get => _useStubRouting;
            set { Set(ref _useStubRouting, value); }
        }

        /// <summary>패턴 저장소 상태 표시(UI 라벨).</summary>
        public string PatternStatus =>
            !_usePatterns ? "기존설계 패턴: OFF"
            : _patterns == null ? "기존설계 패턴: 없음(기하 폴백)"
            : $"기존설계 패턴: {_patterns.Count}키";

        /// <summary>점유맵 해상도. true=원본(전체 셀 표시, 느릴 수 있음), false=다운샘플(상한까지만).</summary>
        public bool OccupancyFullRes
        {
            get => _occupancyFullRes;
            set { if (Set(ref _occupancyFullRes, value)) { OnChanged(nameof(OccupancyResolutionLabel)); if (_showOccupancyVoxels) RebuildIfReady(); } }
        }

        /// <summary>해상도 토글 버튼 라벨(현재 모드 표시).</summary>
        public string OccupancyResolutionLabel => _occupancyFullRes ? "원본" : "샘플";

        public ObservableCollection<TaskRowVM> Tasks { get; } = new();
        public ObservableCollection<LegendItem> Legend { get; } = new();
        public ObservableCollection<UtilityFilterVM> UtilityFilters { get; } = new();

        /// <summary>3D 공간 영역 텍스트 라벨(코드비하인드가 BillboardText 로 렌더). BuildModel 에서 갱신.</summary>
        public ObservableCollection<SpaceLabel> SpaceLabels { get; } = new();

        // ---- 경로 탐색 범위 ----
        /// <summary>탐색 범위 콤보 항목.</summary>
        public ObservableCollection<RouteScopeOption> RouteScopes { get; } = new()
        {
            new RouteScopeOption { Scope = RouteScope.All,       Label = "모두 (전체 충돌회피)" },
            new RouteScopeOption { Scope = RouteScope.ByGroup,   Label = "유틸리티그룹별 (1개 선택)" },
            new RouteScopeOption { Scope = RouteScope.ByUtility, Label = "유틸리티별 (1개 선택)" },
        };

        /// <summary>범위가 그룹별/유틸별일 때 선택 가능한 대상 목록(그룹명 또는 유틸명).</summary>
        public ObservableCollection<string> RouteTargets { get; } = new();

        /// <summary>선택된 탐색 범위. 바뀌면 대상 목록을 다시 만든다.</summary>
        public RouteScopeOption SelectedRouteScope
        {
            get => _selectedRouteScope;
            set { if (Set(ref _selectedRouteScope, value)) { RebuildRouteTargets(); OnChanged(nameof(IsTargetSelectable)); } }
        }

        /// <summary>선택된 대상(그룹명/유틸명). 범위가 '모두'면 무시된다.</summary>
        public string? SelectedRouteTarget { get => _selectedRouteTarget; set => Set(ref _selectedRouteTarget, value); }

        /// <summary>대상 콤보 활성 여부(범위가 '모두'가 아닐 때만).</summary>
        public bool IsTargetSelectable => _selectedRouteScope != null && _selectedRouteScope.Scope != RouteScope.All;

        // ---- 좌측 드릴다운: 유틸리티 그룹 → 유틸리티 → 개별 PoC ----
        /// <summary>1단계: 유틸리티 그룹 목록.</summary>
        public ObservableCollection<string> GroupList { get; } = new();
        /// <summary>2단계: 선택 그룹의 유틸리티 목록.</summary>
        public ObservableCollection<string> UtilityList { get; } = new();
        /// <summary>3단계: 선택 (그룹,유틸)의 개별 PoC(작업) 목록.</summary>
        public ObservableCollection<TaskRowVM> PocList { get; } = new();

        /// <summary>선택된 유틸리티 그룹. 선택 시 유틸리티 목록을 채우고 상단 범위를 '그룹별'로 동기화.</summary>
        public string? SelectedGroup
        {
            get => _selectedGroup;
            set
            {
                if (!Set(ref _selectedGroup, value)) return;
                RebuildUtilityList();
                RebuildIfReady();   // 기존 설계배관을 선택 그룹만 표시(미선택=전체)하도록 3D 갱신.
                if (!_suppressDrillCascade && !string.IsNullOrEmpty(value))
                    SyncTopScope(RouteScope.ByGroup, value);   // '경로 탐색 실행'이 이 그룹을 라우팅.
            }
        }

        /// <summary>선택된 유틸리티. 선택 시 개별 PoC 목록을 채우고 상단 범위를 '유틸별'로 동기화.</summary>
        public string? SelectedUtility
        {
            get => _selectedUtility;
            set
            {
                if (!Set(ref _selectedUtility, value)) return;
                RebuildPocList();
                if (!_suppressDrillCascade && !string.IsNullOrEmpty(value))
                    SyncTopScope(RouteScope.ByUtility, value);   // '경로 탐색 실행'이 이 유틸을 라우팅.
            }
        }

        private bool _suppressDrillCascade;

        /// <summary>선택 PoC 강조 오버레이(시작/끝 마커). 라우팅과 무관 — '선택은 강조표시만'.</summary>
        public Model3D? SelectionModel { get => _selectionModel; private set => Set(ref _selectionModel, value); }
        public Model3D? HighlightModel { get => _highlightModel; private set => Set(ref _highlightModel, value); }

        /// <summary>A* 단계별 탐색 오버레이(방문 셀을 확장 순서대로 점진 표시).</summary>
        public Model3D? SearchModel { get => _searchModel; private set => Set(ref _searchModel, value); }

        private Model3D? _stepHighlightModel;
        /// <summary>선택한 경로 단계(구간) 강조 오버레이 — 카메라 이동 없이 해당 구간만 흰색으로.</summary>
        public Model3D? StepHighlightModel { get => _stepHighlightModel; private set => Set(ref _stepHighlightModel, value); }

        /// <summary>기존설계 비교 오버레이(기존 경로=주황 / 개발 경로=시안 굵은 튜브 + 시작/끝 마커).</summary>
        public Model3D? CompareModel { get => _compareModel; private set => Set(ref _compareModel, value); }

        // 진행 다이얼로그 라우팅 중 '배관 완료마다' 점진 표시하는 라이브 오버레이(유틸 색 튜브).
        // 라우팅이 끝나면 비우고 BuildModel 의 최종 렌더로 대체한다(중복 방지).
        private Model3D? _liveRouteModel;
        public Model3D? LiveRouteModel { get => _liveRouteModel; private set => Set(ref _liveRouteModel, value); }
        private Model3DGroup? _liveGroup;

        /// <summary>우측 '기존설계 비교 분석' 패널 텍스트(매칭 상태 + 길이/꺾임/종단점/간섭 지표). null=비활성.</summary>
        public string? ComparisonReport { get => _comparisonReport; private set => Set(ref _comparisonReport, value); }

        /// <summary>선택 경로의 직선 구간(단계) 목록. 방향이 바뀌는 지점마다 한 항목.</summary>
        public ObservableCollection<PathStep> PathSteps { get; } = new();

        private PathStep? _selectedStep;
        private bool _suppressStepNav;   // 목록 재구성 중 자동 네비게이션 방지.
        /// <summary>선택된 단계. 사용자가 목록에서 고르면 그 위치로 카메라를 이동(NavigateToRequested).</summary>
        public PathStep? SelectedStep
        {
            get => _selectedStep;
            // 단계를 고르면 카메라는 그대로 두고(현재 화면 유지) 해당 구간만 강조 표시한다.
            set { if (Set(ref _selectedStep, value) && !_suppressStepNav) HighlightStep(value); }
        }

        /// <summary>단계 클릭 시 해당 월드좌표로 카메라 이동 요청(코드비하인드가 처리).</summary>
        public event Action<Point3D>? NavigateToRequested;

        // 1단계: 그룹 목록을 작업 분포에서 채운다(새 프로젝트 로드 시).
        private void RebuildGroupList()
        {
            _suppressDrillCascade = true;
            GroupList.Clear();
            foreach (var g in Tasks.Select(t => GroupKey(t.Group)).Distinct().OrderBy(s => s, StringComparer.Ordinal))
                GroupList.Add(g);
            SelectedGroup = null;        // 사용자가 직접 고르도록(자동 라우팅 방지).
            UtilityList.Clear();
            PocList.Clear();
            SelectedUtility = null;
            _suppressDrillCascade = false;
        }

        // 2단계: 선택 그룹의 유틸리티 목록.
        private void RebuildUtilityList()
        {
            bool prev = _suppressDrillCascade;
            _suppressDrillCascade = true;
            UtilityList.Clear();
            PocList.Clear();
            _selectedUtility = null; OnChanged(nameof(SelectedUtility));
            if (!string.IsNullOrEmpty(_selectedGroup))
                foreach (var u in Tasks.Where(t => GroupKey(t.Group) == _selectedGroup)
                                       .Select(t => UtilityKey(t.Utility)).Distinct()
                                       .OrderBy(s => s, StringComparer.Ordinal))
                    UtilityList.Add(u);
            _suppressDrillCascade = prev;
        }

        // 3단계: 선택 (그룹,유틸)의 개별 PoC 작업 목록.
        private void RebuildPocList()
        {
            PocList.Clear();
            if (string.IsNullOrEmpty(_selectedGroup) || string.IsNullOrEmpty(_selectedUtility)) return;
            foreach (var row in Tasks.Where(t => GroupKey(t.Group) == _selectedGroup &&
                                                 UtilityKey(t.Utility) == _selectedUtility))
                PocList.Add(row);
        }

        // 드릴다운 선택을 상단 범위/대상으로 일방 동기화('라우팅은 상단 범위로').
        private void SyncTopScope(RouteScope scope, string target)
        {
            SelectedRouteScope = RouteScopes.First(o => o.Scope == scope);   // RebuildRouteTargets 호출 → target=첫째.
            SelectedRouteTarget = target;                                    // 원하는 대상으로 덮어씀.
        }

        // ---- 바닥 격자(GridLinesVisual3D) 파라미터 — 씬 좌표에 맞춰 코드비하인드가 읽어 갱신 ----
        // 하드코딩하면 실제 DB 좌표(수만 mm)와 떨어져 ZoomExtents 가 빗나가 객체가 구석에 작게 보인다.
        public Point3D GroundCenter { get; private set; } = new Point3D(0, 0, 0);
        public double GroundWidth { get; private set; } = 1000;
        public double GroundLength { get; private set; } = 1000;
        public double GroundMinorDistance { get; private set; } = 1000;
        public double GroundMajorDistance { get; private set; } = 5000;

        // 격자 BBOX(원점=lo, 크기=N*cell) 로부터 바닥 격자 위치/크기/간격을 산출한다.
        private void UpdateGroundGrid(GridMeta g)
        {
            double w = g.Nx * g.CellMm, l = g.Ny * g.CellMm;
            GroundCenter = new Point3D(g.Ox + w / 2, g.Oy + l / 2, g.Oz);   // z=격자 바닥.
            GroundWidth = w; GroundLength = l;
            // 큰 변 기준 ~20칸이 되도록 간격을 '예쁜 값'으로(라인 수 폭주 방지).
            GroundMinorDistance = NiceSpacing(Math.Max(w, l) / 20.0, g.CellMm);
            GroundMajorDistance = GroundMinorDistance * 5;
        }

        // size 이상의 가장 가까운 1·2·5×10^n 값(최소 cell).
        private static double NiceSpacing(double target, double cell)
        {
            if (target <= cell) return cell;
            double mag = Math.Pow(10, Math.Floor(Math.Log10(target)));
            double norm = target / mag;
            double nice = norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 5 ? 5 : 10;
            return Math.Max(cell, nice * mag);
        }

        /// <summary>유틸리티/검색 필터가 적용된 작업 목록 뷰(ListBox 바인딩).</summary>
        public ICollectionView TasksView { get; }

        /// <summary>작업 라벨 검색 문자열(부분일치, 대소문자 무시). 비우면 모두 통과.</summary>
        public string SearchText
        {
            get => _searchText;
            set { if (Set(ref _searchText, value)) { TasksView.Refresh(); OnChanged(nameof(TaskCountText)); } }
        }

        /// <summary>현재 표시되는 작업 수/전체 수(예: "120 / 208"). 좌측 패널 헤더용.</summary>
        public string TaskCountText
        {
            get
            {
                int visible = Tasks.Count(TaskFilterCore);
                return $"{visible} / {Tasks.Count}";
            }
        }

        public RelayCommand OpenCommand { get; }
        public RelayCommand DemoCommand { get; }
        public RelayCommand RunRouteCommand { get; }
        public RelayCommand RerouteCorridorCommand { get; }
        public RelayCommand RerouteSelectedCommand { get; }
        public RelayCommand CompareSelectedCommand { get; }
        public RelayCommand PickStartCommand { get; }
        public RelayCommand PickEndCommand { get; }
        public RelayCommand FitViewCommand { get; }
        public RelayCommand ToggleOccupancyResCommand { get; }
        public RelayCommand AnimateSelectedCommand { get; }
        public RelayCommand UtilityAllCommand { get; }
        public RelayCommand UtilityClearCommand { get; }
        public RelayCommand LoadProjectsCommand { get; }
        public RelayCommand LoadDbCommand { get; }
        public RelayCommand RouteGroupCommand { get; }
        public RelayCommand RouteUtilityCommand { get; }
        public RelayCommand ClearRoutesCommand { get; }

        // ---- DB 접속 설정(상단 툴바 텍스트박스 바인딩) ----
        public string DbHost { get => _dbConfig.Host; set { _dbConfig.Host = value; OnChanged(); } }
        public int DbPort { get => _dbConfig.Port; set { _dbConfig.Port = value; OnChanged(); } }
        public string DbUser { get => _dbConfig.User; set { _dbConfig.User = value; OnChanged(); } }
        public string DbPassword { get => _dbConfig.Password; set { _dbConfig.Password = value; OnChanged(); } }
        public string DbDatabase { get => _dbConfig.Database; set { _dbConfig.Database = value; OnChanged(); } }
        public double CellMm { get => _cellMm; set => Set(ref _cellMm, value); }

        public ObservableCollection<ProjectInfo> Projects { get; } = new();
        public ProjectInfo? SelectedProject
        {
            get => _selectedProject;
            set
            {
                if (!Set(ref _selectedProject, value)) return;
                if (_suppressProjectAutoLoad || value == null) return;
                _ = LoadFromDbAsync(value.ProjectId);   // 비동기(UI 비차단). 예외는 내부에서 Status 로.
            }
        }

        /// <summary>모델을 새로 만들면 발생(코드비하인드가 ZoomExtents 호출).</summary>
        public event Action? SceneRebuilt;

        /// <summary>'전체보기' 명령(코드비하인드가 ZoomExtents 호출).</summary>
        public event Action? FitViewRequested;

        /// <summary>특정 영역(Rect3D)으로 메인 뷰를 줌(코드비하인드가 CameraHelper.ZoomExtents 호출).
        /// 진행 다이얼로그에서 배관 행을 클릭하면 그 배관 로컬 영역으로 메인 뷰를 맞춘다.</summary>
        public event Action<Rect3D>? ZoomToBoxRequested;

        // ---- 필터 ----
        private bool TaskFilter(object o) => o is TaskRowVM r && TaskFilterCore(r);

        private bool TaskFilterCore(TaskRowVM r)
        {
            if (!string.IsNullOrWhiteSpace(_searchText) &&
                r.Label.IndexOf(_searchText, StringComparison.OrdinalIgnoreCase) < 0)
                return false;
            var f = UtilityFilters.FirstOrDefault(u => u.Label == r.Label);
            return f == null || f.IsVisible;
        }

        private void OnUtilityFilterChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (_suppressFilterRebuild) return;
            if (e.PropertyName != nameof(UtilityFilterVM.IsVisible)) return;
            TasksView.Refresh();
            OnChanged(nameof(TaskCountText));
            if (_scene != null && _engine != null) BuildModel();
        }

        private void SetAllUtilities(bool visible)
        {
            _suppressFilterRebuild = true;
            foreach (var u in UtilityFilters) u.IsVisible = visible;
            _suppressFilterRebuild = false;
            TasksView.Refresh();
            OnChanged(nameof(TaskCountText));
            if (_scene != null && _engine != null) BuildModel();
        }

        // ---- 로드 ----
        private void Open()
        {
            var dlg = new OpenFileDialog
            {
                Title = "scene.txt 열기",
                Filter = "scene 파일|*.scene.txt;*.txt|모든 파일|*.*"
            };
            if (dlg.ShowDialog() != true) return;
            try { LoadFile(dlg.FileName); }
            catch (Exception ex) { Status = "열기 오류: " + ex.Message; }
        }

        private void LoadFile(string path)
        {
            string text = File.ReadAllText(path, Encoding.UTF8);
            _scene = SceneTextParser.Parse(text);
            ResetEngine();
            _engine!.LoadSceneText(text);   // scene.txt 는 엔진 내부 파서로 적재(정확 동일성 보존).
            BuildTaskRows();
            // scene.txt/--selftest 경로는 동기 결과(vm.Status)를 즉시 읽으므로 '모두' 동기 라우팅.
            // LoadSceneText 가 작업을 파일 순서대로 적재하므로 엔진 인덱스 == 행 인덱스.
            try
            {
                _engine.RouteMulti(_priority);
                CacheResultsByIndex();
                BuildModel();
            }
            catch (Exception ex) { Status = "경로 탐색 오류: " + ex.Message; }
        }

        // ---- DB 로드 ----
        /// <summary>space_project_map 에서 프로젝트 목록을 읽어 Projects 에 채우고, 첫 항목 선택 시
        /// SelectedProject 의 set 이 자동으로 LoadFromDb 를 호출(전체 자동 로드 흐름).</summary>
        private async Task LoadProjectsAsync()
        {
            try
            {
                // DB 목록 조회는 네트워크 I/O → 백그라운드. (실패 시 ~TimeoutSec 후 예외.)
                Status = "DB 프로젝트 목록 조회 중…";
                var list = await Task.Run(() => ObstacleDbLoader.ListProjects(_dbConfig));
                _suppressProjectAutoLoad = true;
                Projects.Clear();
                foreach (var p in list) Projects.Add(p);
                if (Projects.Count == 0)
                {
                    _suppressProjectAutoLoad = false;
                    Status = "DB 에 프로젝트가 없습니다(space_project_map 비어 있음)";
                    return;
                }
                Status = $"프로젝트 {Projects.Count}개 로드";
                // 콤보에 첫 항목을 표시하되 setter 의 자동 로드는 억제하고, 아래에서 명시적으로 await 한다.
                SelectedProject = Projects[0];
                _suppressProjectAutoLoad = false;
                await LoadFromDbAsync(Projects[0].ProjectId);
            }
            catch (Exception ex)
            {
                _suppressProjectAutoLoad = false;
                Status = "DB 접속 실패: " + ex.Message;
            }
        }

        /// <summary>한 프로젝트의 장애물·PoC 페어를 DB 에서 읽어 엔진에 적재한다.
        /// 사용자 요청대로 <b>라우팅은 하지 않고 장애물만 전체화면으로 보여준다</b>.
        /// 경로 탐색은 사용자가 범위(모두/그룹별/유틸별)를 고르고 '경로 탐색 실행'을 눌러 시작한다.
        /// DB I/O 는 백그라운드로(연결 지연에도 UI 가 멈추지 않게).</summary>
        private async Task LoadFromDbAsync(int projectId)
        {
            try
            {
                Status = "DB 장면 로드 중…";
                var sd = await Task.Run(() => ObstacleDbLoader.LoadScene(_dbConfig, projectId, _cellMm));
                _scene = sd;
                // 기존설계 패턴 저장소(pgvector)를 1회 로드(미스도 캐시) — 학습된 진출/진입 면으로 PoC 투영(L2a).
                if (!_patternsTried)
                {
                    _patternsTried = true;
                    _patterns = await Task.Run(() => PatternStore.TryLoad(_dbConfig));
                    OnChanged(nameof(PatternStatus));
                }
                // 그룹배관 번들 템플릿(route_bundle_template)을 프로젝트별로 로드 — 신규설계 활용(L4).
                // 프로젝트마다 source_file 이 다르므로 매 로드 시 갱신(트렁크 고도는 그 프로젝트 좌표계).
                _bundlesTried = true;
                _bundles = await Task.Run(() => BundleStore.TryLoad(_dbConfig, sd.SourceFile));
                OnChanged(nameof(BundleStatus));
                ResetEngine();
                var g = sd.Grid;
                _engine!.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
                _engine.SetParams(g.CellMm, 500, 10, 2, 6);   // 기본 비용함수 파라미터.
                foreach (var o in sd.Obstacles)
                    if (o.IsPassThrough)   // 통과 객체(바닥/천장/격자보): 점유맵엔 넣되 A* 충돌엔 제외.
                        _engine.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                    else
                        _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                // 작업도 적재하되(점유맵/단일 라우팅 일관성) 자동 라우팅은 하지 않는다.
                foreach (var t in sd.Tasks)
                    _engine.AddTask(t.Sx, t.Sy, t.Sz, t.Gx, t.Gy, t.Gz, t.Utility, t.Group);
                BuildTaskRows();   // 행/필터/탐색 대상 목록 구성(경로 캐시는 비어 있음).

                BuildModel();      // 장애물만 렌더 + SceneRebuilt → ZoomExtents(전체보기).
                Status = $"장애물 {sd.Obstacles.Count} · 작업 {sd.Tasks.Count} · 격자 {g.Nx}×{g.Ny}×{g.Nz} cell={g.CellMm:0}mm   |   범위를 고르고 '▶ 경로 탐색 실행'을 누르세요";
            }
            catch (Exception ex) { Status = "DB 로드 오류: " + ex.Message; }
        }

        /// <summary>내장 데모(골든03): 120x120x60, 바닥 슬래브, 같은 통로 5개 배관.</summary>
        private void LoadDemo()
        {
            var sc = new SceneData
            {
                Grid = new GridMeta { CellMm = 50, Ox = 0, Oy = 0, Oz = 0, Nx = 120, Ny = 120, Nz = 60 }
            };
            sc.Obstacles.Add(new ObstacleBox { MinX = 0, MinY = 0, MinZ = 0, MaxX = 6000, MaxY = 6000, MaxZ = 250 });
            var utils = new (string u, string g)[]
            {
                ("UPW_S", "UPW"), ("NFW", "Waste Liquid"), ("PA", "Gas"), ("NW", "Water"), ("ACID", "Exhaust")
            };
            foreach (var (u, g) in utils)
                sc.Tasks.Add(new TaskInfo { Sx = 275, Sy = 3025, Sz = 1525, Gx = 5725, Gy = 3025, Gz = 1525, Utility = u, Group = g });
            _scene = sc;

            ResetEngine();
            var grid = sc.Grid;
            _engine!.SetGrid(grid.CellMm, grid.Ox, grid.Oy, grid.Oz, grid.Nx, grid.Ny, grid.Nz);
            _engine.SetParams(50, 500, 10, 2, 6);
            foreach (var o in sc.Obstacles)
                _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            foreach (var t in sc.Tasks)
                _engine.AddTask(t.Sx, t.Sy, t.Sz, t.Gx, t.Gy, t.Gz, t.Utility, t.Group);

            BuildTaskRows();
            // 데모는 가벼우므로(5개 배관) 동기로 '모두' 라우팅해 바로 경로를 보여준다.
            try
            {
                _engine.RouteMulti(_priority);
                CacheResultsByIndex();
                BuildModel();
            }
            catch (Exception ex) { Status = "경로 탐색 오류: " + ex.Message; }
        }

        private void ResetEngine()
        {
            _engine?.Dispose();
            _engine = new Engine();
        }

        // ---- 작업 목록 ----
        private void BuildTaskRows()
        {
            Tasks.Clear();
            var scene = _scene!;
            var colorMap = UtilityColors.Assign(scene.Tasks.Select(t => t.UtilityLabel));
            for (int i = 0; i < scene.Tasks.Count; i++)
            {
                var t = scene.Tasks[i];
                var color = colorMap.TryGetValue(t.UtilityLabel, out var c) ? c : Colors.Gray;
                Tasks.Add(new TaskRowVM
                {
                    Index = i, Label = t.UtilityLabel, Swatch = new SolidColorBrush(color),
                    Utility = t.Utility, Group = t.Group,
                    PocName = t.PocName, EndName = t.EndName,
                    Sx = t.Sx, Sy = t.Sy, Sz = t.Sz, Gx = t.Gx, Gy = t.Gy, Gz = t.Gz
                });
            }
            BuildUtilityFilters(colorMap);
            RebuildRouteTargets();
            RebuildGroupList();             // 좌측 드릴다운 1단계(그룹) 채우기.
            SearchModel = null;             // 이전 단계별 탐색 오버레이 제거.
            SelectedTask = null;            // 선택은 드릴다운(③ 개별 PoC)에서 명시적으로.
            OnChanged(nameof(TaskCountText));
            TasksView.Refresh();
        }

        // 선택 범위(그룹별/유틸별)에 따라 대상 콤보 목록을 작업 분포에서 새로 만든다.
        private void RebuildRouteTargets()
        {
            RouteTargets.Clear();
            if (_selectedRouteScope != null && Tasks.Count > 0)
            {
                IEnumerable<string> keys = _selectedRouteScope.Scope switch
                {
                    RouteScope.ByGroup => Tasks.Select(t => GroupKey(t.Group)),
                    RouteScope.ByUtility => Tasks.Select(t => UtilityKey(t.Utility)),
                    _ => Enumerable.Empty<string>(),
                };
                foreach (var k in keys.Distinct().OrderBy(s => s, StringComparer.Ordinal))
                    RouteTargets.Add(k);
            }
            SelectedRouteTarget = RouteTargets.FirstOrDefault();
            OnChanged(nameof(IsTargetSelectable));
        }

        private static string GroupKey(string? g) => string.IsNullOrEmpty(g) ? "?" : g;
        private static string UtilityKey(string? u) => string.IsNullOrEmpty(u) ? "?" : u;

        // 유틸리티별 필터 행을 작업 라벨 분포에서 새로 만든다(기존 항목은 PropertyChanged 해제).
        private void BuildUtilityFilters(Dictionary<string, Color> colorMap)
        {
            _suppressFilterRebuild = true;
            foreach (var f in UtilityFilters) f.PropertyChanged -= OnUtilityFilterChanged;
            UtilityFilters.Clear();
            var groups = Tasks.GroupBy(t => t.Label).OrderBy(g => g.Key);
            foreach (var g in groups)
            {
                var color = colorMap.TryGetValue(g.Key, out var c) ? c : Colors.Gray;
                var f = new UtilityFilterVM
                {
                    Label = g.Key,
                    Swatch = new SolidColorBrush(color),
                    Count = g.Count(),
                };
                f.PropertyChanged += OnUtilityFilterChanged;
                UtilityFilters.Add(f);
            }
            _suppressFilterRebuild = false;
        }

        // ---- 경로 탐색(범위 선택) ----
        // 모든 작업 행 위치(0..N-1).
        private List<int> AllRows() => Enumerable.Range(0, Tasks.Count).ToList();

        /// <summary>'▶ 경로 탐색 실행' — 선택된 범위(모두/그룹별/유틸별)에 해당하는 작업만 라우팅한다.
        /// 그룹별/유틸별은 선택한 대상 1개의 작업들만 부분집합으로 충돌회피 라우팅한다.</summary>
        private async Task RunRouteAsync()
        {
            if (_scene == null || _selectedRouteScope == null) return;
            List<int> rows;
            string label;
            switch (_selectedRouteScope.Scope)
            {
                case RouteScope.ByGroup:
                    if (string.IsNullOrEmpty(SelectedRouteTarget)) { Status = "라우팅할 그룹을 선택하세요"; return; }
                    rows = RowsWhere(t => GroupKey(t.Group) == SelectedRouteTarget);
                    label = $"그룹 '{SelectedRouteTarget}'";
                    break;
                case RouteScope.ByUtility:
                    if (string.IsNullOrEmpty(SelectedRouteTarget)) { Status = "라우팅할 유틸리티를 선택하세요"; return; }
                    rows = RowsWhere(t => UtilityKey(t.Utility) == SelectedRouteTarget);
                    label = $"유틸리티 '{SelectedRouteTarget}'";
                    break;
                default:
                    rows = AllRows();
                    label = "모두";
                    break;
            }
            if (rows.Count == 0) { Status = "대상 작업이 없습니다"; return; }
            // 범위(모두/그룹/유틸) 라우팅도 그룹배관 모드 — 같은 유틸을 공용 트렁크에 다발로(기존설계 유사).
            await RouteRowsAsync(rows, label, corridor: true, showProgress: true);
        }

        private List<int> RowsWhere(Func<TaskRowVM, bool> pred)
        {
            var list = new List<int>();
            for (int i = 0; i < Tasks.Count; i++) if (pred(Tasks[i])) list.Add(i);
            return list;
        }

        // '기존설계 유사' 랙 레벨 — 작업 종단점 z 의 셀 레벨 분포에서 가장 흔한 레벨(최대 3개)을
        // 선호 단으로 삼아 배관을 공용 높이로 모은다(빈 배열이면 회랑 인력만으로도 번들링됨).
        private int[] ComputeRackLevels()
        {
            if (_scene == null) return System.Array.Empty<int>();
            double oz = _scene.Grid.Oz, cell = _scene.Grid.CellMm;
            if (cell <= 0) return System.Array.Empty<int>();
            var counts = new Dictionary<int, int>();
            void Bump(double z) { int k = (int)Math.Floor((z - oz) / cell); counts[k] = counts.TryGetValue(k, out var v) ? v + 1 : 1; }
            foreach (var t in Tasks) { Bump(t.Sz); Bump(t.Gz); }
            return counts.OrderByDescending(kv => kv.Value).Take(3).Select(kv => kv.Key).ToArray();
        }

        // 엔진을 [장애물 전체 + 지정 행들의 작업]만으로 재구성한다(부분집합 충돌회피 라우팅용).
        // 반환: 적재한 행 위치 목록(순서 = 엔진 작업 인덱스). 종단점은 행에서 직접 읽는다(편집 반영).
        // groupMode=true(그룹배관 라우팅) — 매칭 기존배관 회랑(L2b)을 강제 ON 취급하고 w_corridor 를 강하게
        //   줘서, 같은 유틸 배관이 공용 트렁크에 뭉치도록 한다(끝단 스텁은 기존대로). priority "utility" 와 함께 쓴다.
        private List<int> BuildEngineForRows(IReadOnlyList<int> rowPositions, bool groupMode = false)
        {
            var scene = _scene!;
            ResetEngine();
            var g = scene.Grid;
            _engine!.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
            // 클리어런스 항상 활성. 대형 격자(25mm/10mm)는 네이티브가 ImplicitOccupancy(복셀화 없는
            // 박스 색인) + '온디맨드 클리어런스'(박스 최근접 질의)로 전환 → 전역 거리맵(배관당 size×4B
            // ~520MB BFS) 없이 저렴하게 계산. 따라서 더는 대형 격자에서 클리어런스를 끄지 않는다(품질 유지).
            // 대형 격자는 weighted A*(w_heur=1.5) — 솔리드 설비/덕트를 우회하는 어려운 경로를 탐색상한(12M)
            // 내에 찾도록 목표 지향 탐색(약간 비최적 허용). 작은 격자(데모 등)는 표준 A*(1.0, 최적).
            // weighted A* — 실데이터처럼 솔리드 설비/덕트가 많은 혼잡 격자는 목표지향 탐색(w_heur=2.0)으로
            // 어려운 우회 경로를 탐색상한(12M) 내에 빠르게 찾는다(약간 비최적 허용). 측정(project6 c100,
            // 1.39M셀·208작업): w_heur 1.5→2.0 으로 회랑 OFF 194→199, 회랑 ON 47s→6.3s(약 7.5× 단축, +1).
            // 임계를 5M→300k 로 낮춰 실데이터 격자가 weighted A* 를 받게 한다(이전엔 1.39M<5M 라 1.0 였음).
            // 작은 데모 격자(<300k셀)는 표준 A*(1.0, 최적). 골든 정확도는 C++ ctest 가 별도 검증(이 경로 무관).
            bool weighted = (long)g.Nx * g.Ny * g.Nz > 300_000;
            double wHeur = weighted ? 2.0 : 1.0;
            // 동적(수렴) 가중 A* — 목표까지 거리비로 가중을 보간한다(먼 곳=wHeur 공격적·빠름, 목표 근처=
            // wHeurNear 신중·정확). 목표/PoC 근처의 혼잡·막다른길에서 순수 그리디(w=2.0) 함정을 피해 마지막
            // 접근 경로를 찾아낸다. 준최적 상한은 여전히 wHeur. 측정(project6 c100): 스텁ON 206→208(완전),
            // 스텁OFF 199→203, 시간 동일. 엔진이 거대격자(예산-게이트 hier)에선 자동 비활성 → cell≤25 무영향.
            // 기본 = wHeur 가중일 때 1.0(=목표서 표준 A*). env R3D_WHEUR_NEAR 로 재정의(0=정적으로 끔).
            double wHeurNear = weighted ? 1.0 : 0.0;
            {
                var sNear = System.Environment.GetEnvironmentVariable("R3D_WHEUR_NEAR");
                if (!string.IsNullOrEmpty(sNear) &&
                    double.TryParse(sNear, System.Globalization.NumberStyles.Float,
                                    System.Globalization.CultureInfo.InvariantCulture, out var vNear) && vNear >= 0.0)
                    wHeurNear = vNear;
            }
            // 기존설계 회랑(L2b)/랙 번들링(L3a) ON 이면 회랑·랙 밖 셀당 가산(=½칸 비용)으로 부드럽게 유도.
            // w_heur=2.0 이 회랑 비용장을 휴리스틱에 반영해 탐색 폭을 억제 → OFF 와 거의 동일 속도로 동작
            // (이전 ~47s 폭증의 원인 해소). corridor_radius=2 는 회랑 튜브 폭(±2셀).
            // 랙 번들링(L3a): 학습된 그룹 랙 z-셀을 rack_levels(면제)로 주면 같은 그룹 배관이 공용 랙에 뭉친다.
            // 랙 페널티는 회랑(0.5)보다 부드러운 0.2(실측: project6 c100 200/208·랙 집중도 17→21%·길이↓.
            // 0.5 는 한 배관을 막아 198 로 떨어짐). 회랑+랙 동시 ON 이면 회랑의 0.5 가 우선(강한 설계추종).
            int[]? rackLevels = _useRackBundling ? BuildRackLevels(rowPositions) : null;
            // 그룹배관 패턴(L4): DB 에 저장된 유틸별 대표 트렁크 고도를 rack_levels 에 합친다(공용 랙 높이에 뭉침).
            if (_useBundlePattern && _bundles != null)
                rackLevels = MergeBundleLevels(rackLevels, rowPositions);
            bool hasRack = rackLevels != null && rackLevels.Length > 0;
            // 번들 공용 트렁크 회랑(L4) — 같은 유틸 기존배관 전체를 '하나의 공용 트렁크 회랑'으로 주입해, 새 배관
            // 들이 흩어지지 않고 한 스파인에 모이게 한다(높이만 유도하던 rack_levels 의 한계 보완: xy 트렁크 제공).
            // test_attract 가 증명한 메커니즘 — w_corridor>0 + 공유 회랑 셀이면 둘째 배관이 첫 배관 곁으로 뭉친다.
            int[] bundleCorr = (_useBundlePattern && _bundles != null)
                ? BuildBundleCorridorCells(rowPositions, 2) : System.Array.Empty<int>();
            bool hasBundleCorr = bundleCorr.Length > 0;
            // 회랑(0.5)은 랙(0.2)보다 강한 설계추종. L2b 또는 번들 트렁크 회랑이 있으면 0.5, 랙만이면 0.2.
            // 그룹 모드도 회랑 0.5(설계추종) — self-bundling(mark_pipe+add_corridor_cells)은 wCorr>0 이면
            //   작동하므로 0.5 로 충분하다. 과거 cell*2.0 은 회랑 밖 셀당 +2칸 페널티 비용장을 weighted A*
            //   휴리스틱(직선거리)이 과소평가해 탐색이 Dijkstra 처럼 폭발 → 혼잡 프로젝트(예: CMP_KSCTA08,
            //   평범한 cell=100 plain 에서도 실패 7건이 확장 11.27M 으로 12M 상한 근접)에서 첫 어려운 배관이
            //   탐색상한(12M) 초과로 실패했다. 0.5 는 동일 다발링·정상속도(ALKA 14/14: 2.0=15.2s→0.5=1.3s).
            double wCorr = (groupMode || _useDesignCorridor || hasBundleCorr) ? g.CellMm * 0.5
                         : (_useRackBundling || (_useBundlePattern && hasRack)) ? g.CellMm * 0.2 : 0.0;
            _engine.SetParams(g.CellMm, 500, 10, 2, 6, wCorridor: wCorr, corridorRadius: 2,
                              rackLevels: rackLevels, wHeur: wHeur, wHeurNear: wHeurNear);
            foreach (var o in scene.Obstacles)
                if (o.IsPassThrough)   // 통과 객체: 점유맵엔 넣되 A* 충돌엔 제외.
                    _engine.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                else
                    _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            // 충돌확장: 설비·덕트·레터럴 + 이미 설계된(라우팅된) 다른 배관을 장애물로 추가(자기 자신 제외).
            AddFacilityObstacles(_engine, new HashSet<int>(rowPositions));
            var added = new List<int>(rowPositions.Count);
            foreach (var pos in rowPositions)
            {
                var row = Tasks[pos];
                row.StartStub = null; row.EndStub = null;
                double sx, sy, sz, gx, gy, gz;

                // 스텁 라우팅: 매칭 기존배관의 출발/종단 스텁(수직+엘보)을 고정 설계 구간으로 깔고, A* 는 스텁
                // 끝(랙 위 자유공간)에서 시작/종료한다. 그러면 결과가 학습 스텁을 따른다(PoC 재탐색 문제 해소).
                var stubPipe = _useStubRouting ? FindMatchingExistingPipe(row) : null;
                if (stubPipe != null)
                {
                    var (srcStub, tgtStub) = StubExtractor.ForPipe(stubPipe);   // 배관 source/target 쪽 스텁.
                    // 방향 정합 — 작업 start 가 배관 source 에 가까우면 정방향, 아니면 역방향(스텁 스왑).
                    Pt3 ps = stubPipe.SourcePos ?? stubPipe.Points[0];
                    Pt3 pe = stubPipe.TargetPos ?? stubPipe.Points[stubPipe.Points.Count - 1];
                    var ts = new Pt3(row.Sx, row.Sy, row.Sz); var te = new Pt3(row.Gx, row.Gy, row.Gz);
                    bool forward = Dist(ts, ps) + Dist(te, pe) <= Dist(ts, pe) + Dist(te, ps);
                    var startStub = forward ? srcStub : tgtStub;
                    var endStub = forward ? tgtStub : srcStub;
                    if (startStub.Count >= 2 && endStub.Count >= 2)
                    {
                        // 표시 경로가 실제 작업 PoC 에서 시작/끝나도록 스텁 양 끝점을 작업 PoC 로 고정.
                        startStub[0] = new Pt3(row.Sx, row.Sy, row.Sz);
                        endStub[0] = new Pt3(row.Gx, row.Gy, row.Gz);
                        row.StartStub = startStub; row.EndStub = endStub;
                        var se = startStub[startStub.Count - 1];   // 출발 스텁 끝(랙) = A* 시작.
                        var ee = endStub[endStub.Count - 1];       // 종단 스텁 끝(랙) = A* 목표.
                        (sx, sy, sz) = SnapPocToFreeCell(se.X, se.Y, se.Z, null);
                        (gx, gy, gz) = SnapPocToFreeCell(ee.X, ee.Y, ee.Z, null);
                        _engine.AddTask(sx, sy, sz, gx, gy, gz, row.Utility, row.Group);
                        added.Add(pos);
                        continue;
                    }
                }

                // 폴백(매칭 배관 없음/스텁 라우팅 OFF): 기존 PoC 직접 라우팅 — 학습 면으로 PoC 를 표면 투영.
                string? startFace = LearnedFace("EQUIP", row.Group, row.Utility);
                string? endFace = LearnedDuctFace(row.Group, row.Utility, row.Gx, row.Gy, row.Gz,
                                                  row.Sx, row.Sy, row.Sz);
                (sx, sy, sz) = DropStartBelowEquipment(row.Sx, row.Sy, row.Sz);
                (sx, sy, sz) = LiftPocToSurface(sx, sy, sz, startFace);
                (sx, sy, sz) = SnapPocToFreeCell(sx, sy, sz, startFace);   // 파묻힌 시작 PoC → 최근접 자유 셀.
                (gx, gy, gz) = LiftPocToSurface(row.Gx, row.Gy, row.Gz, endFace);
                (gx, gy, gz) = SnapPocToFreeCell(gx, gy, gz, endFace);     // 파묻힌 종단 PoC → 최근접 자유 셀.
                _engine.AddTask(sx, sy, sz, gx, gy, gz, row.Utility, row.Group);
                added.Add(pos);
            }
            // 회랑 셀 주입(w_corridor>0 일 때 효력): L2b(배관별 매칭) + 번들 공용 트렁크(L4) 합집합.
            //   그룹 모드면 L2b(매칭 기존배관 추종)를 강제 ON — 그룹 패턴(L4) 트렁크 좌표와 합쳐 자유공간을 가이드.
            int[]? l2bCells = (_useDesignCorridor || groupMode) ? BuildDesignCorridorCells(rowPositions, 2) : null;
            _engine.SetCorridorCells(CombineCorridor(l2bCells, bundleCorr));
            return added;
        }

        // 지정 행들의 매칭 기존 설계배관(TB_ROUTE_PATH) 폴리라인을 격자 셀로 복셀화(±dilate 팽창)해
        // 회랑 시드(ijk 평탄 배열)로 만든다. 새 경로가 이 회랑 안을 '싸게' 지나 사람 설계를 따라가게 한다.
        private int[] BuildDesignCorridorCells(IReadOnlyList<int> rowPositions, int dilate)
        {
            var s = _scene!; var g = s.Grid; double cell = g.CellMm;
            var set = new HashSet<(int, int, int)>();
            foreach (var pos in rowPositions)
            {
                var pipe = FindMatchingExistingPipe(Tasks[pos]);
                if (pipe == null || pipe.Points.Count < 2) continue;
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                    double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                    int steps = Math.Max(1, (int)(len / (cell * 0.5)));
                    for (int sIdx = 0; sIdx <= steps; sIdx++)
                    {
                        double tt = (double)sIdx / steps;
                        int ci = (int)Math.Floor((a.X + dx * tt - g.Ox) / cell);
                        int cj = (int)Math.Floor((a.Y + dy * tt - g.Oy) / cell);
                        int ck = (int)Math.Floor((a.Z + dz * tt - g.Oz) / cell);
                        for (int di = -dilate; di <= dilate; di++)
                            for (int dj = -dilate; dj <= dilate; dj++)
                                for (int dk = -dilate; dk <= dilate; dk++)
                                {
                                    int ii = ci + di, jj = cj + dj, kk = ck + dk;
                                    if (ii < 0 || jj < 0 || kk < 0 || ii >= g.Nx || jj >= g.Ny || kk >= g.Nz) continue;
                                    set.Add((ii, jj, kk));
                                }
                    }
                }
            }
            var arr = new int[set.Count * 3];
            int n = 0;
            foreach (var (i, j, k) in set) { arr[n++] = i; arr[n++] = j; arr[n++] = k; }
            return arr;
        }

        // 번들 공용 트렁크 회랑 + pitch 레인(L4) — 같은 유틸 기존배관의 '트렁크 고도 수평 런'(=평행 랙 레인)만
        // 타이트하게(±1셀) 회랑으로 만든다. 전체 폴리라인을 넓게 깔던 옵션1과 달리, 레인만 좁게 깔면 route_multi 의
        // 충돌회피(mark_pipe)가 새 배관들을 '인접 레인'에 분산 배치 → 사람 설계처럼 등간격 평행 다발로 패킹된다.
        //   트렁크 고도(trunk_z)는 번들 템플릿에서 조회. 트렁크 밴드(±1셀) 안 수평 런만 채택(수직 라이저·팬아웃 제외).
        //   주의: 격자 셀 > pitch 면 인접 레인이 같은 셀로 뭉개진다(예 cell=100 > pitch≈56) → 셀 크기 ≤ pitch/2 권장.
        // 템플릿 미적재/트렁크 미스면 전체 폴리라인을 넓게(±2) 까는 옵션1 동작으로 폴백(무해).
        private int[] BuildBundleCorridorCells(IReadOnlyList<int> rowPositions, int dilate)
        {
            var s = _scene; if (s == null || s.ExistingPipes.Count == 0) return System.Array.Empty<int>();
            var g = s.Grid; double cell = g.CellMm, oz = g.Oz;

            var utils = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
                if (!string.IsNullOrEmpty(Tasks[pos].Utility)) utils.Add(Tasks[pos].Utility!);
            if (utils.Count == 0) return System.Array.Empty<int>();

            // 유틸별 트렁크 고도(z-셀) 집합 — 이 밴드의 수평 런만 레인으로 채택.
            var trunkZ = new HashSet<int>();
            if (_bundles != null)
                foreach (var u in utils)
                {
                    var t = _bundles.TryGet(null, u); if (t == null) continue;
                    foreach (var z in t.TrunkZs)
                    {
                        int zk = (int)Math.Floor((z - oz) / cell);
                        if (zk >= 0 && zk < g.Nz) trunkZ.Add(zk);
                    }
                }
            bool laneMode = trunkZ.Count > 0;          // 트렁크 고도를 알면 레인 모드(타이트), 아니면 옵션1 폴백.
            const double HorizTol = 0.34, MinRunMm = 800.0;
            const int BandCells = 1, LaneDilate = 1;   // 트렁크 ±1셀, 레인 두께 ±1셀(타이트).

            var set = new HashSet<(int, int, int)>();
            foreach (var pipe in s.ExistingPipes)
            {
                if (pipe.Utility == null || !utils.Contains(pipe.Utility) || pipe.Points.Count < 2) continue;
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                    double horiz = Math.Sqrt(dx * dx + dy * dy);
                    double len = Math.Sqrt(horiz * horiz + dz * dz);
                    int dl = dilate;
                    if (laneMode)
                    {
                        // 레인 모드: 트렁크 고도 밴드 안의 '수평 런(랙 레인)'만, 타이트하게.
                        if (horiz <= 1e-6 || Math.Abs(dz) > HorizTol * horiz || len < MinRunMm) continue;
                        int zk = (int)Math.Floor(((a.Z + b.Z) / 2 - oz) / cell);
                        bool nearTrunk = false;
                        foreach (var tz in trunkZ) if (Math.Abs(zk - tz) <= BandCells) { nearTrunk = true; break; }
                        if (!nearTrunk) continue;
                        dl = LaneDilate;
                    }
                    int steps = Math.Max(1, (int)(len / (cell * 0.5)));
                    for (int sIdx = 0; sIdx <= steps; sIdx++)
                    {
                        double tt = (double)sIdx / steps;
                        int ci = (int)Math.Floor((a.X + dx * tt - g.Ox) / cell);
                        int cj = (int)Math.Floor((a.Y + dy * tt - g.Oy) / cell);
                        int ck = (int)Math.Floor((a.Z + dz * tt - g.Oz) / cell);
                        for (int di = -dl; di <= dl; di++)
                            for (int dj = -dl; dj <= dl; dj++)
                                for (int dk = -dl; dk <= dl; dk++)
                                {
                                    int ii = ci + di, jj = cj + dj, kk = ck + dk;
                                    if (ii < 0 || jj < 0 || kk < 0 || ii >= g.Nx || jj >= g.Ny || kk >= g.Nz) continue;
                                    set.Add((ii, jj, kk));
                                }
                    }
                }
            }
            var arr = new int[set.Count * 3];
            int n = 0;
            foreach (var (i, j, k) in set) { arr[n++] = i; arr[n++] = j; arr[n++] = k; }
            return arr;
        }

        // 두 회랑 셀 배열(ijk 평탄)을 합친다 — 중복은 엔진이 set 으로 흡수. 둘 다 비면 null(회랑 없음).
        private static int[]? CombineCorridor(int[]? a, int[]? b)
        {
            int la = a?.Length ?? 0, lb = b?.Length ?? 0;
            if (la == 0 && lb == 0) return null;
            if (lb == 0) return a;
            if (la == 0) return b;
            var r = new int[la + lb];
            System.Array.Copy(a!, 0, r, 0, la);
            System.Array.Copy(b!, 0, r, la, lb);
            return r;
        }

        // 그룹배관 강조용 — group_id 별 구분되는 고유 색(황금비 색상환 회전으로 인접 그룹도 또렷이 구분).
        private static Color BundleGroupColor(int gid)
        {
            double h = (gid * 0.61803398875) % 1.0 * 360.0;
            double s = 0.72, v = 0.98;
            double c = v * s, x = c * (1 - Math.Abs((h / 60.0) % 2 - 1)), m = v - c;
            double r, g, b;
            if (h < 60) { r = c; g = x; b = 0; }
            else if (h < 120) { r = x; g = c; b = 0; }
            else if (h < 180) { r = 0; g = c; b = x; }
            else if (h < 240) { r = 0; g = x; b = c; }
            else if (h < 300) { r = x; g = 0; b = c; }
            else { r = c; g = 0; b = x; }
            return Color.FromRgb((byte)((r + m) * 255), (byte)((g + m) * 255), (byte)((b + m) * 255));
        }

        // 유틸그룹 랙 번들링(L3a) — 라우팅할 행들의 그룹에 속한 기존배관 수평 런이 모이는 z-셀(랙 높이)을
        // 학습해 엔진 rack_levels(면제 z-셀, 최대 8)로 만든다. Python pattern_learn.learn_rack_levels 와 동일 로직:
        //   수평(|dz| <= 0.34×수평거리)이고 800mm 이상인 세그먼트의 z-셀에 런 길이를 누적 → 그룹별 지배 랙 선정.
        private int[]? BuildRackLevels(IReadOnlyList<int> rowPositions)
        {
            var s = _scene!; var g = s.Grid; double cell = g.CellMm;
            if (s.ExistingPipes.Count == 0) return null;
            const double MinRunMm = 800.0, HorizTol = 0.34, GroupShare = 0.15;

            // 이번 라우팅에 등장하는 유틸그룹 집합.
            var groups = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
                if (!string.IsNullOrEmpty(Tasks[pos].Group)) groups.Add(Tasks[pos].Group!);
            if (groups.Count == 0) return null;

            // 그룹 → (z셀 → 누적 런 mm). 기존배관 수평 런을 z-셀에 누적.
            var byGroup = new Dictionary<string, Dictionary<int, double>>(StringComparer.OrdinalIgnoreCase);
            foreach (var pipe in s.ExistingPipes)
            {
                if (pipe.Group == null || !groups.Contains(pipe.Group) || pipe.Points.Count < 2) continue;
                if (!byGroup.TryGetValue(pipe.Group, out var zmap))
                    byGroup[pipe.Group] = zmap = new Dictionary<int, double>();
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double horiz = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
                    if (horiz <= 1e-6 || Math.Abs(b.Z - a.Z) > HorizTol * horiz) continue;   // 수직/사선 제외.
                    double len = Math.Sqrt(horiz * horiz + (b.Z - a.Z) * (b.Z - a.Z));
                    if (len < MinRunMm) continue;
                    int zk = (int)Math.Floor(((a.Z + b.Z) / 2 - g.Oz) / cell);
                    if (zk < 0 || zk >= g.Nz) continue;
                    zmap[zk] = (zmap.TryGetValue(zk, out var v) ? v : 0.0) + len;
                }
            }
            if (byGroup.Count == 0) return null;

            // 그룹마다 전체 런의 GroupShare 이상을 차지하는 z-셀을 채택(지배 랙). 전 그룹 합집합 → 런 큰 순 8개.
            var picked = new Dictionary<int, double>();   // z셀 → 누적 런(여러 그룹 합산, 정렬용).
            foreach (var (_, zmap) in byGroup)
            {
                double tot = 0; foreach (var v in zmap.Values) tot += v;
                if (tot <= 0) continue;
                foreach (var (zk, run) in zmap)
                    if (run >= GroupShare * tot)
                        picked[zk] = (picked.TryGetValue(zk, out var p) ? p : 0.0) + run;
            }
            if (picked.Count == 0) return null;
            var top = picked.OrderByDescending(kv => kv.Value).Take(8).Select(kv => kv.Key).ToArray();
            return top.Length > 0 ? top : null;
        }

        // 그룹배관 패턴(L4) — 이번 라우팅 행들의 유틸리티별 학습 트렁크 고도(route_bundle_template)를 z-셀로
        // 변환해 rackLevels 에 합친다(중복 제거, 엔진 상한 8). DB 에 저장한 번들 패턴을 신규 라우팅에 직접 활용:
        // 같은 유틸 새 배관이 사람이 설계한 공용 랙(트렁크) 고도에 뭉친다. 키 미스/미적재면 입력 그대로 반환.
        private int[]? MergeBundleLevels(int[]? rackLevels, IReadOnlyList<int> rowPositions)
        {
            var s = _scene; if (s == null || _bundles == null) return rackLevels;
            var g = s.Grid; double oz = g.Oz, cell = g.CellMm; if (cell <= 0) return rackLevels;

            var utils = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
                if (!string.IsNullOrEmpty(Tasks[pos].Utility)) utils.Add(Tasks[pos].Utility!);
            if (utils.Count == 0) return rackLevels;

            var zset = new HashSet<int>(rackLevels ?? System.Array.Empty<int>());
            foreach (var u in utils)
            {
                var t = _bundles.TryGet(null, u);   // (owner 미상 → util 폴백) 유틸별 트렁크 고도.
                if (t == null) continue;
                foreach (var z in t.TrunkZs)
                {
                    int zk = (int)Math.Floor((z - oz) / cell);
                    if (zk >= 0 && zk < g.Nz) zset.Add(zk);
                }
            }
            return zset.Count > 0 ? zset.Take(8).ToArray() : rackLevels;
        }

        // 기존설계 패턴에서 학습된 진출/진입 면(예: EQUIP=-z, DUCT=+z)을 조회한다. 패턴 OFF/미적재/미스면 null.
        private string? LearnedFace(string anchorKind, string? group, string? utility)
        {
            if (!_usePatterns || _patterns == null) return null;
            return _patterns.TryGet(anchorKind, group, utility, out var tpl) ? tpl.Face : null;
        }

        // 검색증강(L3b) — DUCT 종단 진입면을 PoC 컨텍스트(앵커 덕트 내 상대위치 + 접근방향)별 ANN 으로 분기한다.
        // 한 (그룹·유틸) 키 안에 진입면이 여럿 섞인 다중면 키(예 UPW_S = +x 57% / +z 43%)에서, 집계 대표면이
        // 틀리는 PoC 를 그 PoC 와 가장 닮은 학습 표본의 면으로 바로잡는다. 단일면/미적재/앵커 없음 → 집계 폴백.
        private string? LearnedDuctFace(string? group, string? utility, double ex, double ey, double ez,
                                        double sx, double sy, double sz)
        {
            if (!_usePatterns || _patterns == null) return null;
            var d = NearestDuctAnchor(ex, ey, ez);
            if (d != null)
            {
                var rel = new[] { RelIn(ex, d.MinX, d.MaxX), RelIn(ey, d.MinY, d.MaxY), RelIn(ez, d.MinZ, d.MaxZ) };
                // 학습 DUCT dir_unit = unit(src - tgt)(덕트로의 접근방향). 추론 = unit(start - end).
                var dir = Unit(sx - ex, sy - ey, sz - ez);
                if (_patterns.TryGetFaceAnn("DUCT", group, utility, rel, dir, out var f)) return f;
            }
            return LearnedFace("DUCT", group, utility);   // 집계 폴백.
        }

        private static double RelIn(double v, double lo, double hi)
            => hi - lo <= 1e-6 ? 0.5 : Math.Min(1.0, Math.Max(0.0, (v - lo) / (hi - lo)));

        private static double[] Unit(double x, double y, double z)
        {
            double n = Math.Sqrt(x * x + y * y + z * z);
            return n < 1e-9 ? new double[3] : new[] { x / n, y / n, z / n };
        }

        // 종단 PoC 를 포함하는 덕트, 없으면 3000mm 내 가장 가까운 덕트(Python find_duct 미러). 없으면 null.
        private DuctLateral? NearestDuctAnchor(double x, double y, double z)
        {
            var s = _scene; if (s == null) return null;
            const double eps = 1.0, maxMm = 3000.0;
            foreach (var d in s.DuctsLaterals)
                if (x >= d.MinX - eps && x <= d.MaxX + eps && y >= d.MinY - eps && y <= d.MaxY + eps &&
                    z >= d.MinZ - eps && z <= d.MaxZ + eps) return d;
            DuctLateral? best = null; double bd = maxMm;
            foreach (var d in s.DuctsLaterals)
            {
                double cx = (d.MinX + d.MaxX) / 2, cy = (d.MinY + d.MaxY) / 2, cz = (d.MinZ + d.MaxZ) / 2;
                double dist = Math.Sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy) + (z - cz) * (z - cz));
                if (dist < bd) { bd = dist; best = d; }
            }
            return best;
        }

        // 시작점이 설비 AABB 내부면 그 설비(들) 중 가장 낮은 바닥(MinZ) 한 셀 아래로 Z 를 내려, 장비 밖
        // 첫 셀을 라우팅 시작점으로 만든다. XY 는 유지. 충돌확장 OFF(설비 비충돌)이거나 설비 밖이면 원점 유지.
        private (double x, double y, double z) DropStartBelowEquipment(double x, double y, double z)
        {
            var s = _scene;
            if (s == null || !_includeFacilities) return (x, y, z);
            const double eps = 1.0;
            double lowestBottom = double.NaN;
            foreach (var e in s.Equipment)
            {
                if (x < e.MinX - eps || x > e.MaxX + eps) continue;
                if (y < e.MinY - eps || y > e.MaxY + eps) continue;
                if (z < e.MinZ - eps || z > e.MaxZ + eps) continue;
                if (double.IsNaN(lowestBottom) || e.MinZ < lowestBottom) lowestBottom = e.MinZ;
            }
            if (double.IsNaN(lowestBottom)) return (x, y, z);   // 설비 안이 아님 → 그대로.
            double cell = s.Grid.CellMm;
            double nz = lowestBottom - cell * 0.5;              // 설비 바닥 한 셀 아래.
            double gridBottom = s.Grid.Oz + cell * 0.5;
            if (nz < gridBottom) nz = gridBottom;               // 격자 하한 밖이면 클램프.
            return (x, y, nz);
        }

        // PoC 가 설비/덕트/레터럴 '솔리드 내부'에 있으면 가장 가까운 면 바로 바깥(+½셀)으로 빼낸다.
        // 덕트 상부 PoC → 윗면 위 한 셀(자유)로 투영 → 배관이 덕트 표면에 닿아 연결되고 본체를 관통하지 않는다.
        // (충돌확장이 항상 ON 이라 설비/덕트가 솔리드이므로, 표면 투영이 없으면 PoC 셀이 막혀 라우팅이 실패하거나
        //  엔진 snap(반경 2)이 엉뚱한 옆 셀로 새어 관통/우회한다.) 여러 박스에 걸치면 몇 번 반복해 탈출.
        private (double x, double y, double z) LiftPocToSurface(double x, double y, double z,
                                                                string? preferFace = null)
        {
            var s = _scene;
            if (s == null || !_includeFacilities) return (x, y, z);
            double cell = s.Grid.CellMm, eps = 1.0, m = cell * 0.5;
            for (int iter = 0; iter < 4; iter++)
            {
                bool moved = false;
                // 한 박스 안이면 탈출 면을 고른다. preferFace(학습된 진출/진입 면)가 주어지고 그 면으로 나갈
                // 수 있으면 그 면으로(사람 설계 관례: 덕트=+z 상부, 장비=-z 하부) — 없으면 침투가 가장 얕은 면.
                void TryBox(double bMinX, double bMinY, double bMinZ, double bMaxX, double bMaxY, double bMaxZ)
                {
                    if (x <= bMinX - eps || x >= bMaxX + eps) return;
                    if (y <= bMinY - eps || y >= bMaxY + eps) return;
                    if (z <= bMinZ - eps || z >= bMaxZ + eps) return;
                    if (preferFace != null)
                    {
                        switch (preferFace)
                        {
                            case "+z": z = bMaxZ + m; moved = true; return;
                            case "-z": z = bMinZ - m; moved = true; return;
                            case "+x": x = bMaxX + m; moved = true; return;
                            case "-x": x = bMinX - m; moved = true; return;
                            case "+y": y = bMaxY + m; moved = true; return;
                            case "-y": y = bMinY - m; moved = true; return;
                        }
                    }
                    double dxn = x - bMinX, dxp = bMaxX - x;
                    double dyn = y - bMinY, dyp = bMaxY - y;
                    double dzn = z - bMinZ, dzp = bMaxZ - z;
                    double mn = Math.Min(Math.Min(Math.Min(dxn, dxp), Math.Min(dyn, dyp)), Math.Min(dzn, dzp));
                    if (mn == dzp) z = bMaxZ + m;        // 윗면(덕트 상부 PoC 의 일반적 경우)
                    else if (mn == dzn) z = bMinZ - m;   // 아랫면
                    else if (mn == dxp) x = bMaxX + m;
                    else if (mn == dxn) x = bMinX - m;
                    else if (mn == dyp) y = bMaxY + m;
                    else y = bMinY - m;
                    moved = true;
                }
                foreach (var e in s.Equipment) TryBox(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
                foreach (var d in s.DuctsLaterals) TryBox(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
                if (!moved) break;
            }
            double zMin = s.Grid.Oz + m;
            if (z < zMin) z = zMin;
            return (x, y, z);
        }

        // 접근불가 PoC 전처리 — Lift 후에도 시작/종단 셀이 솔리드에 막혀 있으면(파묻힌 PoC: 여러 솔리드에
        // 둘러싸여 면 투영이 다시 솔리드 안으로 떨어지거나, 엔진 snap(반경 2)으로도 못 빠져나오는 경우)
        // 가장 가까운 '자유 셀'로 옮긴다. ① 학습된 면 법선으로 바깥 행진(도메인 방향 우선: 덕트=+z 상부 등)
        // → ② 실패 시 체비셰프 반경을 넓혀가며 최근접 자유 셀. 끝내 못 찾으면(진짜 접근불가) 원점 유지.
        // 이미 자유 셀이면 그대로 반환(이미 성공하는 작업엔 영향 0 = 회귀 없음).
        private const int SnapMarchCells = 16;   // 학습면 방향 최대 행진(셀).
        private const int SnapMaxRadius = 6;     // 반경 확장 탐색 상한(셀).

        private (double x, double y, double z) SnapPocToFreeCell(double x, double y, double z, string? preferFace)
        {
            var s = _scene;
            if (s == null || !_includeFacilities) return (x, y, z);
            double cell = s.Grid.CellMm;
            if (!CellBlocked(x, y, z)) return (x, y, z);   // 이미 자유 → 변경 없음.

            // ① 학습된 진출/진입 면 방향으로 바깥 행진(있으면) — 도메인에 맞는 방향으로 먼저 탈출 시도.
            if (preferFace != null)
            {
                var (dx, dy, dz) = FaceNormal(preferFace);
                if (dx != 0 || dy != 0 || dz != 0)
                    for (int k = 1; k <= SnapMarchCells; k++)
                    {
                        double nx = x + dx * cell * k, ny = y + dy * cell * k, nz = z + dz * cell * k;
                        if (InGrid(nx, ny, nz) && !CellBlocked(nx, ny, nz)) return (nx, ny, nz);
                    }
            }
            // ② 체비셰프 반경 확장 — 셸 단위로 넓혀가며 가장 가까운 자유 셀.
            for (int r = 1; r <= SnapMaxRadius; r++)
            {
                (double, double, double)? best = null; double bestD = double.MaxValue;
                for (int di = -r; di <= r; di++)
                    for (int dj = -r; dj <= r; dj++)
                        for (int dk = -r; dk <= r; dk++)
                        {
                            if (Math.Max(Math.Max(Math.Abs(di), Math.Abs(dj)), Math.Abs(dk)) != r) continue; // 셸만.
                            double nx = x + di * cell, ny = y + dj * cell, nz = z + dk * cell;
                            if (!InGrid(nx, ny, nz) || CellBlocked(nx, ny, nz)) continue;
                            double d = di * di + dj * dj + dk * dk;
                            if (d < bestD) { bestD = d; best = (nx, ny, nz); }
                        }
                if (best != null) return best.Value;
            }
            return (x, y, z);  // 진짜 접근불가 → 엔진 snap 에 맡김(여전히 실패할 수 있음).
        }

        private static (double dx, double dy, double dz) FaceNormal(string face) => face switch
        {
            "+x" => (1, 0, 0),
            "-x" => (-1, 0, 0),
            "+y" => (0, 1, 0),
            "-y" => (0, -1, 0),
            "+z" => (0, 0, 1),
            "-z" => (0, 0, -1),
            _ => (0, 0, 0),
        };

        private bool InGrid(double x, double y, double z)
        {
            var g = _scene!.Grid;
            return x >= g.Ox && y >= g.Oy && z >= g.Oz
                && x <= g.Ox + g.Nx * g.CellMm && y <= g.Oy + g.Ny * g.CellMm && z <= g.Oz + g.Nz * g.CellMm;
        }

        // 점이 속한 셀(반열린 [lo, lo+cell))이 어떤 솔리드(장애물 비통과 + 설비/덕트, 얇은 축은 minT 팽창)와
        // 겹치면 막힘. 엔진 복셀화(box↔cell overlap, 동일 minT)와 같은 기준 → 엔진 is_blocked 근사.
        private bool CellBlocked(double x, double y, double z)
        {
            var s = _scene!; var g = s.Grid; double cell = g.CellMm, minT = cell;
            int ci = (int)Math.Floor((x - g.Ox) / cell);
            int cj = (int)Math.Floor((y - g.Oy) / cell);
            int ck = (int)Math.Floor((z - g.Oz) / cell);
            double clx = g.Ox + ci * cell, chx = clx + cell;
            double cly = g.Oy + cj * cell, chy = cly + cell;
            double clz = g.Oz + ck * cell, chz = clz + cell;

            bool Overlap(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            {
                if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
                if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
                if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
                return clx < mxx && chx > mnx && cly < mxy && chy > mny && clz < mxz && chz > mnz;
            }
            foreach (var o in s.Obstacles)
                if (!o.IsPassThrough && Overlap(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ)) return true;
            foreach (var e in s.Equipment)
                if (Overlap(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ)) return true;
            foreach (var d in s.DuctsLaterals)
                if (Overlap(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ)) return true;
            return false;
        }

        // 충돌확장 — 라우팅 엔진에 추가 충돌 대상을 장애물로 넣는다(IncludeFacilities ON 일 때).
        //   · 설비(TB_BIM_EQUIPMENT) — 메인 장비 포함 전체
        //   · 덕트/레터럴(TB_DUCT_LATERAL)
        //   · 이미 우리 알고리즘으로 설계된(라우팅 성공) 다른 배관의 경로(currentRows 의 자기 자신은 제외)
        // 시작/끝 PoC 가 이들 표면(특히 메인 장비)에 닿아 막히면 엔진의 snap_to_free_cell(반경 2) 이
        // 인접 빈 셀로 옮겨 시작점을 확보한다.
        private void AddFacilityObstacles(Engine engine, HashSet<int> currentRows)
        {
            if (!_includeFacilities || _scene == null) return;
            var s = _scene;
            double cell = s.Grid.CellMm;
            double minT = cell;   // 두께 0 축을 최소 셀 1개로 팽창(가는 덕트/판도 셀을 막도록).

            // ★ 설비·덕트·레터럴을 '항상 솔리드 장애물'로 추가한다(제외 없음). 예전에는 종단 PoC 를 포함하는
            //   덕트/설비 박스를 통째로 제외했는데, 그러면 그 덕트가 비충돌이 되어 배관이 덕트를 '관통'했다.
            //   대신 종단/시작 PoC 가 솔리드에 갇히는 문제는 LiftPocToSurface 로 PoC 를 표면 바로 바깥으로
            //   투영해 해결한다(BuildEngineForRows) → 배관이 덕트 표면에 닿아 연결되고 본체는 관통하지 않는다.
            foreach (var e in s.Equipment)        // 메인 장비 포함 전체 설비.
                AddBoxObstacle(engine, e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ, minT);
            foreach (var d in s.DuctsLaterals)     // 덕트/레터럴(부대장비 포함).
                AddBoxObstacle(engine, d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ, minT);

            // 이미 설계된(라우팅 성공) 다른 배관의 경로 + 고정 스텁을 점유로 추가 — 새 배관이 이를 피하도록.
            // 메모리 효율: 셀별 점유가 아니라 직선 구간 AABB(반경 r 팽창)로 등록한다(엔진 호출/메모리 최소).
            double r = cell * 0.6;   // 경로/스텁 폴리라인을 약 1셀 두께 튜브로 점유.
            for (int i = 0; i < Tasks.Count; i++)
            {
                if (currentRows.Contains(i)) continue;          // 지금 라우팅하는(=자기) 배관은 제외(자기 스텁에 막히지 않게).
                var row = Tasks[i];
                if (!row.Success || row.Path.Length < 2) continue;
                AddPathObstacle(engine, row.Path, s.Grid, r);
                // 고정 출발/종단 스텁(수직+엘보)도 장애물로 — 다른 배관이 스텁을 관통/교차하지 않도록.
                if (row.StartStub != null) AddPolylineObstacle(engine, row.StartStub, r);
                if (row.EndStub != null) AddPolylineObstacle(engine, row.EndStub, r);
            }
        }

        // 월드 mm 폴리라인을 직선 구간별 AABB(반경 r 팽창)로 장애물에 추가. 셀 복셀화 없이 세그먼트당 박스 1개(메모리 효율).
        private static void AddPolylineObstacle(Engine engine, System.Collections.Generic.IReadOnlyList<Pt3> poly, double r)
        {
            for (int i = 1; i < poly.Count; i++)
            {
                var a = poly[i - 1]; var b = poly[i];
                engine.AddObstacle(
                    Math.Min(a.X, b.X) - r, Math.Min(a.Y, b.Y) - r, Math.Min(a.Z, b.Z) - r,
                    Math.Max(a.X, b.X) + r, Math.Max(a.Y, b.Y) + r, Math.Max(a.Z, b.Z) + r);
            }
        }

        // AABB 장애물 추가 — 두께 0 축은 minT 로 팽창해 반드시 셀을 점유하게 한다.
        private static void AddBoxObstacle(Engine engine, double mnx, double mny, double mnz,
                                           double mxx, double mxy, double mxz, double minT)
        {
            if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
            if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
            if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
            engine.AddObstacle(mnx, mny, mnz, mxx, mxy, mxz);
        }

        // 경로(셀 폴리라인)를 직선 구간별 AABB(반경 r 팽창) 로 장애물에 추가(호출 수 절감).
        private static void AddPathObstacle(Engine engine, PathCell[] path, GridMeta g, double r)
        {
            (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));
            int n = path.Length, seg = 0;
            var cur = Dir(path[0], path[1]);
            for (int i = 2; i <= n; i++)
            {
                var d = (i < n) ? Dir(path[i - 1], path[i]) : (int.MinValue, 0, 0);
                if (d != cur)
                {
                    var pa = CellToWorld(g, path[seg]); var pb = CellToWorld(g, path[i - 1]);
                    engine.AddObstacle(
                        Math.Min(pa.X, pb.X) - r, Math.Min(pa.Y, pb.Y) - r, Math.Min(pa.Z, pb.Z) - r,
                        Math.Max(pa.X, pb.X) + r, Math.Max(pa.Y, pb.Y) + r, Math.Max(pa.Z, pb.Z) + r);
                    seg = i - 1;
                    if (i < n) cur = Dir(path[i - 1], path[i]);
                }
            }
        }

        // 지정 행들의 자동설계 결과(경로/성공/길이/방문)를 초기화한다 — '기존설계 삭제' / 재설계 전 초기화에 사용.
        private void ClearRouteResults(IEnumerable<int> rowPositions)
        {
            foreach (var pos in rowPositions)
            {
                var r = Tasks[pos];
                r.Success = false;
                r.LengthMm = 0;
                r.Path = System.Array.Empty<PathCell>();
                r.Visited = System.Array.Empty<PathCell>();
            }
        }

        // 자동설계된 '모든' 경로 삭제(버튼) — 전체 작업 결과 초기화 + 라이브 오버레이 제거 + 재렌더.
        private void ClearAllRoutes()
        {
            if (_scene == null) return;
            var all = new List<int>(Tasks.Count);
            for (int i = 0; i < Tasks.Count; i++) all.Add(i);
            ClearRouteResults(all);
            _compareMode = false;
            ResetLiveRoute();
            BuildModel();
            System.Windows.Input.CommandManager.InvalidateRequerySuggested();  // 버튼 비활성 즉시 반영.
            Status = "자동설계된 경로를 모두 삭제했습니다.";
        }

        // 엔진 결과(엔진 인덱스 e ↔ added[e] 행)를 행 캐시에 기록. 부분집합 라우팅 후 호출.
        private void CacheResults(IReadOnlyList<int> added)
        {
            for (int e = 0; e < added.Count; e++)
            {
                var row = Tasks[added[e]];
                try
                {
                    var r = _engine!.GetResult(e);
                    row.Success = r.Success; row.LengthMm = r.LengthMm;
                    row.Path = r.Path; row.Visited = r.Visited;
                }
                catch { row.Success = false; row.LengthMm = 0; row.Path = Array.Empty<PathCell>(); row.Visited = Array.Empty<PathCell>(); }
            }
        }

        // 엔진 인덱스 == 행 인덱스(전체 작업이 파일/추가 순서대로 적재된 경우) 결과 캐시.
        private void CacheResultsByIndex()
        {
            for (int i = 0; i < Tasks.Count; i++)
            {
                var row = Tasks[i];
                try
                {
                    var r = _engine!.GetResult(i);
                    row.Success = r.Success; row.LengthMm = r.LengthMm;
                    row.Path = r.Path; row.Visited = r.Visited;
                }
                catch { row.Success = false; row.LengthMm = 0; row.Path = Array.Empty<PathCell>(); row.Visited = Array.Empty<PathCell>(); }
            }
        }

        /// <summary>지정 행들만 부분집합으로 라우팅(무거운 네이티브 호출은 백그라운드 → UI 비차단).
        /// 범위에 없는 행의 경로 캐시는 보존된다(그룹/유틸을 차례로 눌러 누적 표시 가능).</summary>
        private async Task RouteRowsAsync(IReadOnlyList<int> rowPositions, string label, bool corridor,
                                          bool showProgress = false)
        {
            if (_scene == null || rowPositions.Count == 0) return;
            RoutingProgressWindow? dlg = null;
            try
            {
                // 그룹 라우팅(corridor)에서 '다건'(2개 이상)은 셀 ≤ 50mm 라야 인접 레인이 분리돼 다발이
                // 형성된다. DB 프로젝트가 로드된 상태에서 셀이 크면 50mm 로 낮춰 재적재(재복셀화)한다. 작업(Task)
                // 목록은 셀과 무관하게 동일 순서로 재생성되므로 rowPositions 인덱스는 그대로 유효하다.
                //   단건(rowPositions.Count==1)은 재적재하지 않는다 — 재적재는 BuildTaskRows 로 모든 누적 경로를
                //   지우므로(다른 배관 결과 손실), 단건 재라우팅에는 부적절. 단건은 그룹 트렁크 회랑만 따라가면 충분.
                if (corridor && rowPositions.Count > 1 && _cellMm > 50.0 && SelectedProject != null)
                {
                    Status = "그룹 라우팅: cell 50mm 로 재적재 중…";
                    CellMm = 50.0;
                    await LoadFromDbAsync(SelectedProject.ProjectId);
                }

                // 동일 대상(동일 Start PoC)을 다시 자동설계할 때 — 기존 설계 데이터를 먼저 지우고 진행한다.
                // (대상 행만 초기화; 다른 그룹/유틸의 누적 경로는 보존 → 충돌 회피·표시 유지.)
                ClearRouteResults(rowPositions);
                BuildModel();   // 지운 상태를 즉시 3D에 반영(라이브 오버레이 위에 옛 경로가 남지 않도록).

                bool cor = corridor;   // 그룹배관 라우팅 모드(스텁+공용 트렁크 회랑+강한 w_corridor).
                var added = BuildEngineForRows(rowPositions, groupMode: cor);
                Status = $"경로 탐색 중… {label} (작업 {added.Count})";
                var engine = _engine!;
                double cellMm = _scene.Grid.CellMm;
                // 그룹 모드는 유틸 우선순위로 정렬 — 같은 유틸을 묶어 순차 라우팅하면 self-bundling 이 유틸별로
                // 일관되게 자라 공용 트렁크에 다발로 모인다. 일반 모드는 기존 정렬(_priority).
                string priority = cor ? "utility" : _priority;
                // 계층 corridor 사용 여부(그룹 모드는 route_multi 로 회랑/랙 번들링 유지).
                bool hier = _useHierarchicalCorridor && !cor;
                // coarse 셀 ≈ 160mm 가 되도록 factor 산출(4~24 클램프), 통로 반경 2 coarse 셀.
                int factor = Math.Clamp((int)Math.Round(160.0 / Math.Max(1.0, cellMm)), 4, 24);

                // 진행 다이얼로그 — 그룹 모드 또는 showProgress 일 때(계층 corridor 제외) 배관별 콜백 실시간 표시.
                bool useProgress = (showProgress || cor) && !hier;
                var grid = _scene.Grid;
                // 엔진 작업 인덱스(=added 순서)별 메타/색을 'UI 스레드에서' 미리 뽑는다(콜백은 백그라운드 스레드라
                // Brush(.Color)·컬렉션 접근이 불가). 콜백은 이 값 배열만 참조 → 스레드 안전.
                var meta = new (string grp, string util, string sp, string ep, Color col)[added.Count];
                for (int k = 0; k < added.Count; k++)
                {
                    var tr = Tasks[added[k]];
                    string sp = string.IsNullOrEmpty(tr.PocName) ? $"({tr.Sx:0},{tr.Sy:0},{tr.Sz:0})" : tr.PocName!;
                    string ep = string.IsNullOrEmpty(tr.EndName) ? $"({tr.Gx:0},{tr.Gy:0},{tr.Gz:0})" : tr.EndName!;
                    Color col = (tr.Swatch as SolidColorBrush)?.Color ?? Colors.Cyan;
                    meta[k] = (tr.Group ?? "", tr.Utility ?? "", sp, ep, col);
                }
                if (useProgress)
                {
                    ResetLiveRoute();   // 이전 라이브 오버레이 제거 → 이번 배치만 점진 표시.
                    dlg = new RoutingProgressWindow { Owner = System.Windows.Application.Current?.MainWindow };
                    dlg.Begin(label, added.Count);
                    // 표 행 클릭 → 그 배관 로컬 미니 3D/설명 생성 + 메인 뷰 줌/선택(라우팅 완료 후에도 동작).
                    // 콜백 ti = '엔진 task 인덱스'(이번 배치의 부분집합 순서) → 전역 Tasks 인덱스(added[ti])로 매핑.
                    // (부분집합 라우팅 시 엔진은 0..N-1 만 갖고, 전역 Tasks 는 전체 작업이므로 매핑 필수.)
                    var addedMap = added;
                    int MapTi(int ti) => (ti >= 0 && ti < addedMap.Count) ? addedMap[ti] : ti;
                    dlg.DetailProvider = (ti, layers) => BuildPipeDetail(MapTi(ti), layers);
                    dlg.FocusInMainView = ti => FocusPipeInMainView(MapTi(ti));
                    dlg.Show();
                }
                var disp = System.Windows.Application.Current?.Dispatcher;

                await Task.Run(() =>
                {
                    if (hier)
                    {
                        // Sparse + astar_hashed: 셀 수 배열을 안 잡아 10mm 대형 격자도 안전.
                        // priority 순차 + mark_pipe 로 배관 간 충돌 0.
                        engine.RouteCorridorMulti(factor, 2, priority, 0);
                    }
                    else if (useProgress)
                    {
                        var d = dlg!;
                        engine.RouteMultiProgress(priority, p =>
                        {
                            int ti = p.TaskIndex;
                            var m = (ti >= 0 && ti < meta.Length)
                                ? meta[ti]
                                : (grp: "", util: "", sp: "", ep: "", col: Colors.Cyan);
                            d.ReportPipe(p, m.grp, m.util, m.sp, m.ep);
                            // 배관 완료(성공) 시 즉시 3D 라이브 오버레이에 그 경로를 추가.
                            if (p.Phase == 1 && p.Success && p.Path.Length >= 1)
                            {
                                var path = p.Path; var col = m.col;
                                disp?.BeginInvoke(new Action(() => AppendLivePipe(path, col, grid)));
                            }
                        });
                    }
                    else
                    {
                        engine.RouteMulti(priority);
                    }
                });
                dlg?.Complete();
                CacheResults(added);
                ResetLiveRoute();   // 라이브 오버레이 제거 → 아래 BuildModel 의 최종 렌더로 대체(중복 방지).
                BuildModel();   // 누적(전체 씬) 기준 상태바를 먼저 갱신한 뒤,
                // 이번 배치 결과를 명확히 덮어쓴다 — "성공 16/113"(전체 대비)이 실패로 오해되지 않도록
                // "이번 라우팅 16/16"을 앞세우고 전체 누적은 괄호로 부기한다.
                int batchOk = 0;
                foreach (var pos in added) if (Tasks[pos].Success) batchOk++;
                int sceneOk = 0;
                foreach (var t in Tasks) if (t.Success) sceneOk++;
                string fail = batchOk < added.Count ? $" · 실패 {added.Count - batchOk}" : "";
                Status = $"{label} 라우팅 완료 · 성공 {batchOk}/{added.Count}{fail}   |   전체 누적 {sceneOk}/{Tasks.Count}";
            }
            catch (Exception ex)
            {
                Status = "경로 탐색 오류: " + ex.Message;
                dlg?.Complete(failedToRun: true, error: ex.Message);
            }
        }

        // ---- 단계별 탐색(선택 배관 A* 진행 애니메이션) ----
        /// <summary>선택 배관을 라우팅하고, A* 가 확장한 방문 셀을 '확장 순서대로' 점진 표시해
        /// 시작 PoC→종단 PoC 로 점유맵을 회피해 나아가는 과정을 애니메이션으로 보여준다.
        /// 탐색이 끝나면 최종 경로를 드러낸다.</summary>
        private async Task AnimateSelectedAsync()
        {
            if (_scene == null || _engine == null || SelectedTask == null || _animating) return;
            _animating = true;
            try
            {
                int idx = SelectedTask.Index;
                SearchModel = null;
                _hidePathsForAnim = true;        // 탐색 중에는 최종 경로를 숨긴다(끝에 드러냄).
                _showOccupancyVoxels = true;     // 회피 대상(점유맵)을 보여준다.
                OnChanged(nameof(ShowOccupancyVoxels));

                // 선택 작업 1개만 부분집합 라우팅 → 방문 셀(확장 순서) + 경로 산출.
                await RouteRowsAsync(new List<int> { idx }, $"단계별 탐색 #{idx}", corridor: false);

                var row = Tasks[idx];
                if (row.Visited.Length == 0)
                {
                    Status = $"#{idx}: 방문 셀이 없습니다(라우팅 실패 또는 방문 수집 off).";
                    _hidePathsForAnim = false;
                    BuildModel();
                    return;
                }

                await AnimateVisitedAsync(_scene.Grid, row.Visited, row.Success ? row.LengthMm : 0);
            }
            catch (Exception ex) { Status = "단계별 탐색 오류: " + ex.Message; }
            finally
            {
                _hidePathsForAnim = false;
                BuildModel();        // 최종 경로(튜브) 드러내기.
                _animating = false;
            }
        }

        // 방문 셀을 확장 순서대로 점진적으로 드러내는 DispatcherTimer 애니메이션.
        private Task AnimateVisitedAsync(GridMeta g, PathCell[] visited, double lengthMm)
        {
            var tcs = new TaskCompletionSource<bool>();
            // 표시 셀 상한(부드러운 재생용). 초과 시 순서를 보존한 채 균등 다운샘플.
            const int Cap = 18000;
            PathCell[] cells = visited;
            if (visited.Length > Cap)
            {
                cells = new PathCell[Cap];
                double stride = (double)visited.Length / Cap;
                for (int i = 0; i < Cap; i++) cells[i] = visited[(int)(i * stride)];
            }
            int total = cells.Length;
            const int Frames = 45;
            int perTick = Math.Max(1, (int)Math.Ceiling(total / (double)Frames));
            double s = g.CellMm * 0.55;   // 방문 큐브 변(경로보다 가늘게).
            int shown = 0;

            var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
            timer.Tick += (_, __) =>
            {
                shown = Math.Min(total, shown + perTick);
                var mb = new MeshBuilder(false, false);
                for (int i = 0; i < shown; i++) mb.AddBox(CellToWorld(g, cells[i]), s, s, s);
                SearchModel = Geometry(mb, Color.FromRgb(255, 205, 70), 110);   // 탐색 구름 = 노랑.
                Status = $"단계별 탐색… 방문 {shown:N0}/{total:N0}" + (lengthMm > 0 ? $"  (경로 {lengthMm:0} mm)" : "");
                if (shown >= total) { timer.Stop(); tcs.TrySetResult(true); }
            };
            timer.Start();
            return tcs.Task;
        }

        // ---- 3D 피킹(P3) ----
        private void SetPick(PickMode mode)
        {
            PickMode = mode;
            Status = mode == PickMode.Start ? "3D 뷰에서 시작점을 클릭하세요…" : "3D 뷰에서 끝점을 클릭하세요…";
        }

        /// <summary>3D 뷰 클릭 지점을 셀 중심으로 스냅해 선택 배관의 종단점으로 설정한다(코드비하인드 호출).</summary>
        public void ApplyPick(Point3D p)
        {
            if (_scene == null || SelectedTask == null || PickMode == PickMode.None) return;
            var g = _scene.Grid;
            int i = (int)Math.Floor((p.X - g.Ox) / g.CellMm);
            int j = (int)Math.Floor((p.Y - g.Oy) / g.CellMm);
            int k = (int)Math.Floor((p.Z - g.Oz) / g.CellMm);
            double x = g.Ox + (i + 0.5) * g.CellMm, y = g.Oy + (j + 0.5) * g.CellMm, z = g.Oz + (k + 0.5) * g.CellMm;
            if (PickMode == PickMode.Start) { SelectedTask.Sx = x; SelectedTask.Sy = y; SelectedTask.Sz = z; }
            else { SelectedTask.Gx = x; SelectedTask.Gy = y; SelectedTask.Gz = z; }
            Status = $"피킹: #{SelectedTask.Index} {(PickMode == PickMode.Start ? "시작" : "끝")}점=({x:0},{y:0},{z:0}). '선택 배관 재라우팅' 을 누르세요.";
            PickMode = PickMode.None;
        }

        private void RebuildIfReady()
        {
            if (_engine != null && _scene != null) BuildModel();
        }

        // ---- 3D 모델 구성 ----
        private void BuildModel()
        {
            var scene = _scene!;
            var grid = scene.Grid;
            UpdateGroundGrid(grid);   // 바닥 격자를 씬 좌표에 맞춤(ZoomExtents 가 객체를 중앙에 잡도록).
            var group = new Model3DGroup();
            Legend.Clear();
            SpaceLabels.Clear();

            // ⓪-A 공간 영역(토글) — TB_BIM_SPACE_INFO 의 CR/A/F/CSF 등을 선형(와이어프레임) + 텍스트로.
            if (ShowSpaces && scene.Spaces.Count > 0)
            {
                AddSpaceAreas(group, scene.Spaces);
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(230, 255, 196, 0)),
                    Label = $"공간 영역 ({scene.Spaces.Count}): {string.Join(", ", scene.Spaces.Select(s => s.Name).Distinct())}"
                });
            }

            // ⓪ 복셀 전체맵(토글) — 격자 BBOX 12변(가는 실린더). 작업 공간을 한눈에.
            if (ShowGridFrame)
            {
                AddGridFrame(group, grid);
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(220, 122, 223, 176)),
                    Label = $"복셀 전체맵 ({grid.Nx}×{grid.Ny}×{grid.Nz})"
                });
            }

            // ① 장애물(토글) — 일반 장애물(회색)과 통과 객체(청록)를 구분해 머지.
            //    통과 객체(바닥/천장/격자보)는 경로탐색 충돌에서 제외되므로 색으로 구분 표시한다.
            if (ShowObstacles && scene.Obstacles.Count > 0)
            {
                var mb = new MeshBuilder(false, false);       // 일반 장애물(충돌).
                var mbPass = new MeshBuilder(false, false);   // 통과 객체(비충돌).
                int nObs = 0, nPass = 0;
                foreach (var o in scene.Obstacles)
                {
                    var center = new Point3D((o.MinX + o.MaxX) / 2, (o.MinY + o.MaxY) / 2, (o.MinZ + o.MaxZ) / 2);
                    if (o.IsPassThrough) { mbPass.AddBox(center, o.MaxX - o.MinX, o.MaxY - o.MinY, o.MaxZ - o.MinZ); nPass++; }
                    else                 { mb.AddBox(center, o.MaxX - o.MinX, o.MaxY - o.MinY, o.MaxZ - o.MinZ); nObs++; }
                }
                if (nObs > 0)
                {
                    group.Children.Add(Geometry(mb, Color.FromRgb(150, 150, 150), 60));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(160, 150, 150, 150)), Label = $"장애물(obstacles) {nObs}" });
                }
                if (nPass > 0)
                {
                    group.Children.Add(Geometry(mbPass, Color.FromRgb(90, 200, 160), 55));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(160, 90, 200, 160)), Label = $"통과 객체(pass-through) {nPass}" });
                }
            }

            // ①-E 장비(토글) — TB_BIM_EQUIPMENT 의 AABB 를 주황 큐브 박스로(메인은 더 진하게).
            if (ShowEquipment && scene.Equipment.Count > 0)
            {
                var mbMain = new MeshBuilder(false, false);
                var mbSub = new MeshBuilder(false, false);
                int nMain = 0, nSub = 0;
                foreach (var eq in scene.Equipment)
                {
                    var center = new Point3D((eq.MinX + eq.MaxX) / 2, (eq.MinY + eq.MaxY) / 2, (eq.MinZ + eq.MaxZ) / 2);
                    if (eq.IsMain) { mbMain.AddBox(center, eq.MaxX - eq.MinX, eq.MaxY - eq.MinY, eq.MaxZ - eq.MinZ); nMain++; }
                    else           { mbSub.AddBox(center, eq.MaxX - eq.MinX, eq.MaxY - eq.MinY, eq.MaxZ - eq.MinZ); nSub++; }
                }
                if (nMain > 0) group.Children.Add(Geometry(mbMain, Color.FromRgb(255, 140, 0), 150));   // 메인=진한 주황.
                if (nSub > 0) group.Children.Add(Geometry(mbSub, Color.FromRgb(255, 190, 90), 90));      // 서브=옅은 주황.
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(190, 255, 140, 0)),
                    Label = $"장비(equipment) {scene.Equipment.Count} (메인 {nMain})"
                });
            }

            // ①-D 덕트/레터럴(각각 별도 토글) — TB_DUCT_LATERAL 의 AABB 를 박스로. 레터럴=초록, 덕트=청색.
            //    일부 덕트는 한 축 두께 0(굽힘) 이라 렌더 시 최소 두께로 클램프해 보이게 한다.
            if ((ShowLaterals || ShowDucts) && scene.DuctsLaterals.Count > 0)
            {
                var mbLat = new MeshBuilder(false, false);
                var mbDuct = new MeshBuilder(false, false);
                int nLat = 0, nDuct = 0;
                const double MinThick = 40;   // 0두께 박스 가시화용 최소 변(mm).
                foreach (var d in scene.DuctsLaterals)
                {
                    bool lateral = d.IsLateral;
                    if (lateral ? !ShowLaterals : !ShowDucts) continue;   // 해당 토글이 꺼져 있으면 스킵.
                    var center = new Point3D((d.MinX + d.MaxX) / 2, (d.MinY + d.MaxY) / 2, (d.MinZ + d.MaxZ) / 2);
                    double sx = Math.Max(d.MaxX - d.MinX, MinThick);
                    double sy = Math.Max(d.MaxY - d.MinY, MinThick);
                    double sz = Math.Max(d.MaxZ - d.MinZ, MinThick);
                    if (lateral) { mbLat.AddBox(center, sx, sy, sz); nLat++; }
                    else         { mbDuct.AddBox(center, sx, sy, sz); nDuct++; }
                }
                if (nLat > 0)
                {
                    group.Children.Add(Geometry(mbLat, Color.FromRgb(90, 210, 130), 150));   // 레터럴=초록.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(200, 90, 210, 130)), Label = $"레터럴(lateral) {nLat}" });
                }
                if (nDuct > 0)
                {
                    group.Children.Add(Geometry(mbDuct, Color.FromRgb(110, 175, 220), 130)); // 덕트=청색.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(200, 110, 175, 220)), Label = $"덕트(duct) {nDuct}" });
                }
            }

            // ①' 점유맵(토글) — 엔진이 voxelize 한 블록 셀을 셀 크기 큐브로(반투명 옅은 청회색).
            //    큐브가 맞닿아 장애물을 빈틈 없이 채운다. 상한 초과 시에만 균등 다운샘플(부하 한도).
            string occNote = string.Empty;
            if (ShowOccupancyVoxels && _engine != null)
            {
                var (rendered, occTotal) = AddOccupancyVoxels(group, grid);
                if (rendered > 0)
                {
                    bool down = rendered < occTotal;
                    Legend.Add(new LegendItem
                    {
                        Swatch = new SolidColorBrush(Color.FromArgb(170, 130, 170, 200)),
                        Label = down ? $"점유맵 (셀 {rendered:N0}/{occTotal:N0} 다운샘플)"
                                     : $"점유맵 (셀 {rendered:N0})"
                    });
                    if (down) occNote = $"   |   점유맵 {occTotal:N0}셀 중 {rendered:N0}만 표시(다운샘플)";
                }
            }

            // ② 경로 — 유틸리티별 색 튜브 + 시작/끝 구. (충돌 계산용으로 경로는 항상 수집)
            // 단계별 탐색 애니메이션 중에는 최종 경로를 숨겨 탐색 과정만 보이게 한다(_hidePathsForAnim).
            bool drawPaths = ShowPaths && !_hidePathsForAnim;
            // 색 배정은 작업 + 기존배관 라벨을 합쳐 한 번에 한다(같은 유틸=같은 색, 라우팅 경로와 기존배관 색 일치).
            var colorMap = UtilityColors.Assign(
                scene.Tasks.Select(t => t.UtilityLabel)
                    .Concat(scene.ExistingPipes.Select(p => p.Label)));
            // 자동(개발) 경로 = 유틸리티별 머지 메시. 색은 '유틸 색을 밝게(Lighten)' 한 같은 계열 색으로 그린다.
            //   같은 유틸의 기존 설계배관(원래 유틸 색)보다 밝아 '사람 설계 vs 자동 설계'를 색으로 구분한다.
            var perUtil = new Dictionary<string, MeshBuilder>();
            var perUtilVisited = new Dictionary<string, MeshBuilder>();   // 방문맵 — 유틸리티별 머지 메시.
            var perUtilVisitedCount = new Dictionary<string, int>();      // 표시 셀 카운트(다운샘플 후).
            var successPaths = new List<PathCell[]>();
            double fallbackTubeDia = grid.CellMm * 0.7;   // 관경 미상 시 격자 기반 기본 지름.
            double markerR = grid.CellMm * 0.9;
            double visitedBoxSize = grid.CellMm * 0.5;  // 방문 셀 큐브 변(작게 — 경로보다 가늘게).
            const int VisitedCapPerUtility = 12000;     // 유틸리티당 표시 상한(WPF 부하 한도).
            int ok = 0;
            double total = 0;

            // 경로는 행 캐시(TaskRowVM.Path/Visited)에서 읽는다 — 엔진은 부분집합 라우팅마다
            // 재구성되어 인덱스가 행과 1:1 이 아니므로, 렌더는 엔진 상태와 분리한다.
            foreach (var row in Tasks)
            {
                if (!row.Success || row.Path.Length == 0) continue;

                ok++;
                total += row.LengthMm;
                successPaths.Add(row.Path);

                string label = row.Label;
                var uf = UtilityFilters.FirstOrDefault(u => u.Label == label);
                bool utilVisible = uf == null || uf.IsVisible;

                // 경로 튜브(ShowPaths + 유틸 가시 일 때) — 실제 관경(매칭 기존배관 SOURCE_SIZE)으로 그린다.
                if (drawPaths && utilVisible)
                {
                    if (!perUtil.TryGetValue(label, out var mb))
                    {
                        mb = new MeshBuilder(false, false);
                        perUtil[label] = mb;
                    }
                    // 관경: 행 캐시값 우선, 없으면 매칭 기존배관에서 1회 조회해 캐시(이후 재빌드 비용 절감).
                    if (row.DiameterMm <= 0)
                    {
                        var exm = FindMatchingExistingPipe(row);
                        row.DiameterMm = (exm != null && exm.DiameterMm > 0) ? exm.DiameterMm : 0;
                    }
                    double routeDia = row.DiameterMm > 0 ? Math.Max(row.DiameterMm, 8.0) : fallbackTubeDia;
                    // 표시 경로 = [출발 스텁] + [A* 중간] + [reverse(종단 스텁)]. 스텁이 없으면 A* 경로만.
                    var pts = new List<Point3D>();
                    if (row.StartStub != null)
                        pts.AddRange(row.StartStub.Select(p => new Point3D(p.X, p.Y, p.Z)));
                    pts.AddRange(row.Path.Select(c => CellToWorld(grid, c)));
                    if (row.EndStub != null)
                        for (int k = row.EndStub.Count - 1; k >= 0; k--)
                            pts.Add(new Point3D(row.EndStub[k].X, row.EndStub[k].Y, row.EndStub[k].Z));
                    if (pts.Count >= 2) mb.AddTube(pts, routeDia, 10, false);
                    mb.AddSphere(pts[0], markerR);
                    mb.AddSphere(pts[^1], markerR);
                }

                // 방문맵 — 유틸리티별 머지 메시(다운샘플링으로 셀 수 상한).
                if (ShowVisitedMap && utilVisible && row.Visited.Length > 0)
                {
                    if (!perUtilVisited.TryGetValue(label, out var vmb))
                    {
                        vmb = new MeshBuilder(false, false);
                        perUtilVisited[label] = vmb;
                        perUtilVisitedCount[label] = 0;
                    }
                    int already = perUtilVisitedCount[label];
                    int remaining = VisitedCapPerUtility - already;
                    if (remaining > 0)
                    {
                        // 균등 다운샘플: 셀 수가 remaining 보다 많으면 stride 로 솎아낸다.
                        int len = row.Visited.Length;
                        int take = Math.Min(remaining, len);
                        double stride = (double)len / take;
                        for (int s = 0; s < take; s++)
                        {
                            int idx = (int)(s * stride);
                            var c = row.Visited[idx];
                            var p = CellToWorld(grid, c);
                            vmb.AddBox(p, visitedBoxSize, visitedBoxSize, visitedBoxSize);
                        }
                        perUtilVisitedCount[label] = already + take;
                    }
                }
            }

            if (drawPaths && perUtil.Count > 0)
            {
                foreach (var kv in perUtil)
                {
                    var baseColor = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    var lit = Lighten(baseColor, 0.55);   // 자동 경로 = 유틸 색을 밝게(기존 설계보다 밝은 같은 계열).
                    group.Children.Add(Geometry(kv.Value, lit, 255));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(lit), Label = $"자동 {kv.Key} (밝게·관경)" });
                }
            }

            // ①-X 기존 설계배관(토글) — TB_ROUTE_PATH 폴리라인을 유틸리티 색 튜브로(월드 mm 좌표 그대로).
            //   각 배관은 DB 의 실제 관경(SOURCE_SIZE→DiameterMm)으로 그린다(겹침 방지). 유틸 필터도 동일 적용.
            // 비교 포커스(_compareMode)에서는 나머지 기존배관을 숨긴다(선택 배관의 기존 경로만 CompareModel 로 강조).
            if (ShowExistingPipes && !_compareMode && scene.ExistingPipes.Count > 0)
            {
                double fallbackDia = Math.Min(grid.CellMm * 0.4, 50);   // 관경 미상 시 기본 지름(mm).
                var perUtilEx = new Dictionary<string, MeshBuilder>();
                // 그룹배관 강조 모드 — 탐지된 번들(route_bundle_group) 멤버를 그룹별 고유 색으로, 비멤버는 흐리게.
                bool bundleHi = _showBundleGroups && _bundles != null && _bundles.GroupCount > 0;
                var perGroup = new Dictionary<int, MeshBuilder>();        // group_id → 머지 메시(그룹 색).
                var nonMemberMb = new MeshBuilder(false, false);          // 번들 미소속 기존배관(흐린 회색).
                int nonMemberCnt = 0;
                // 출발(빨강)·종단(파랑) 스텁 강조 — 학습 StubExtractor 로 잘라낸 수직+엘보 구간을 굵은 색 튜브로.
                var startStubMb = new MeshBuilder(false, false);
                var endStubMb = new MeshBuilder(false, false);
                int drawn = 0, stubDrawn = 0;
                foreach (var pipe in scene.ExistingPipes)
                {
                    // 좌측에서 유틸리티 그룹을 선택했으면 그 그룹의 기존 설계배관만 표시(미선택=전체).
                    if (!string.IsNullOrEmpty(_selectedGroup) && GroupKey(pipe.Group) != _selectedGroup) continue;
                    string label = pipe.Label;
                    var uf = UtilityFilters.FirstOrDefault(u => u.Label == label);
                    if (uf != null && !uf.IsVisible) continue;   // 유틸 체크박스 필터 적용.
                    if (pipe.Points.Count < 2) continue;
                    var pts = pipe.Points.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList();
                    // 실제 관경(외경) 사용. 너무 가늘면 안 보이므로 최소 8mm 로 클램프.
                    double dia = pipe.DiameterMm > 0 ? Math.Max(pipe.DiameterMm, 8.0) : fallbackDia;
                    // 머지 대상 선택: 그룹배관 강조면 그룹 멤버=그룹 메시 / 비멤버=흐린 메시, 아니면 유틸 색 메시.
                    MeshBuilder mb;
                    if (bundleHi)
                    {
                        int gid = _bundles!.GroupIdOf(pipe.RoutePathGuid);
                        if (gid >= 0)
                        {
                            if (!perGroup.TryGetValue(gid, out mb)) { mb = new MeshBuilder(false, false); perGroup[gid] = mb; }
                        }
                        else { mb = nonMemberMb; nonMemberCnt++; }
                    }
                    else if (!perUtilEx.TryGetValue(label, out mb!))
                    {
                        mb = new MeshBuilder(false, false);
                        perUtilEx[label] = mb;
                    }
                    mb.AddTube(pts, dia, 10, false);
                    drawn++;

                    // 출발/종단 스텁(수직배관 + 엘보) — 학습과 동일 로직으로 잘라 빨강/파랑 튜브로 강조.
                    // 굵기는 배관 관경과 동일하게(색만 다르게) 그린다 — 실제 배관 형상과 일치시킨다.
                    if (ShowStubs)
                    {
                        double stubDia = dia;
                        var (startStub, endStub) = StubExtractor.ForPipe(pipe);
                        if (startStub.Count >= 2)
                        {
                            startStubMb.AddTube(startStub.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList(), stubDia, 10, false);
                            stubDrawn++;
                        }
                        if (endStub.Count >= 2)
                            endStubMb.AddTube(endStub.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList(), stubDia, 10, false);
                    }
                }
                if (bundleHi)
                {
                    // 비멤버 기존배관은 흐린 회색으로(다발이 도드라지게).
                    if (nonMemberCnt > 0) group.Children.Add(Geometry(nonMemberMb, Color.FromRgb(90, 100, 120), 90));
                    foreach (var kv in perGroup.OrderBy(k => k.Key))
                        group.Children.Add(Geometry(kv.Value, BundleGroupColor(kv.Key), 245));
                    Legend.Add(new LegendItem
                    {
                        Swatch = new SolidColorBrush(BundleGroupColor(0)),
                        Label = $"그룹배관 {perGroup.Count}그룹 (멤버 {drawn - nonMemberCnt} · 비멤버 {nonMemberCnt})"
                    });
                }
                else
                {
                    // 기존 설계배관 = 유틸리티 색(원색). 같은 유틸 자동 경로는 이 색을 밝게 한 색이라 구분된다.
                    foreach (var kv in perUtilEx)
                    {
                        var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                        group.Children.Add(Geometry(kv.Value, color, 235));
                    }
                    if (drawn > 0)
                        Legend.Add(new LegendItem
                        {
                            Swatch = new SolidColorBrush(Color.FromArgb(235, 160, 160, 160)),
                            Label = $"기존 설계배관 {drawn} (유틸 색·관경)"
                        });
                }
                if (ShowStubs && stubDrawn > 0)
                {
                    group.Children.Add(Geometry(startStubMb, Color.FromRgb(226, 48, 48), 255));   // 출발 스텁 = 빨강.
                    group.Children.Add(Geometry(endStubMb, Color.FromRgb(48, 112, 255), 255));    // 종단 스텁 = 파랑.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(226, 48, 48)), Label = $"출발 스텁 {stubDrawn}" });
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(48, 112, 255)), Label = "종단 스텁" });
                }
            }

            // PoC 마커 — 모든 작업(장비)의 시작 PoC(빨강 구)·종단 PoC(파랑 구)를 라우팅 전에도 표시.
            //   유틸 체크박스 필터 + 좌측 선택 그룹을 동일 적용(경로/기존배관 레이어와 일관). 두 색을 머지 메시로.
            if (ShowPocMarkers && Tasks.Count > 0)
            {
                double pocR = Math.Max(grid.CellMm * 0.9, 50);
                var startMb = new MeshBuilder(false, false);
                var endMb = new MeshBuilder(false, false);
                int pocN = 0;
                foreach (var row in Tasks)
                {
                    if (!string.IsNullOrEmpty(_selectedGroup) && GroupKey(row.Group) != _selectedGroup) continue;
                    var uf = UtilityFilters.FirstOrDefault(u => u.Label == row.Label);
                    if (uf != null && !uf.IsVisible) continue;
                    startMb.AddSphere(new Point3D(row.Sx, row.Sy, row.Sz), pocR);
                    endMb.AddSphere(new Point3D(row.Gx, row.Gy, row.Gz), pocR);
                    pocN++;
                }
                if (pocN > 0)
                {
                    group.Children.Add(Geometry(startMb, Color.FromRgb(255, 45, 45), 235));    // 시작 PoC = 빨강.
                    group.Children.Add(Geometry(endMb, Color.FromRgb(50, 120, 255), 235));     // 종단 PoC = 파랑.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(255, 45, 45)), Label = $"시작 PoC {pocN}" });
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(50, 120, 255)), Label = $"종단 PoC {pocN}" });
                }
            }

            // 방문맵 — 유틸리티별 색의 반투명 큐브 집합. 경로와 같은 색 규약.
            if (ShowVisitedMap && perUtilVisited.Count > 0)
            {
                int totalShown = 0;
                foreach (var kv in perUtilVisited)
                {
                    var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    group.Children.Add(Geometry(kv.Value, color, 80));   // alpha 80 = 약 31% 불투명.
                    totalShown += perUtilVisitedCount[kv.Key];
                }
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(120, 200, 200, 200)),
                    Label = $"방문맵 (셀 {totalShown:N0})"
                });
            }

            // ③ 충돌(토글) — ≥2 배관이 공유하는 셀을 빨간 큐브로.
            int collisions = 0;
            if (ShowCollisions)
            {
                var cells = CollisionFinder.Find(successPaths);
                collisions = cells.Count;
                if (cells.Count > 0)
                {
                    var cmb = new MeshBuilder(false, false);
                    double s = grid.CellMm * 0.9;
                    foreach (var (ci, cj, ck) in cells)
                        cmb.AddBox(CellToWorld(grid, new PathCell(ci, cj, ck)), s, s, s);
                    group.Children.Add(Geometry(cmb, Colors.Magenta, 235));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Colors.Magenta), Label = $"충돌(collision) {cells.Count}" });
                }
            }

            SceneModel = group;
            UpdateSelectionHighlight();   // 라우팅 후 선택 경로의 꺾임 마커·단계 목록 갱신.
            Status = $"장애물 {scene.Obstacles.Count} · 작업 {scene.Tasks.Count} · 성공 {ok}/{scene.Tasks.Count} · 총 {total:0} mm · 충돌 {collisions}{occNote}   |   engine: {Engine.Version}";
            SceneRebuilt?.Invoke();
        }

        private static Point3D CellToWorld(GridMeta g, PathCell c) =>
            new(g.Ox + (c.I + 0.5) * g.CellMm, g.Oy + (c.J + 0.5) * g.CellMm, g.Oz + (c.K + 0.5) * g.CellMm);

        // 라이브 오버레이 초기화(배치 라우팅 시작 시). UI 스레드에서 호출.
        private void ResetLiveRoute()
        {
            _liveGroup = null;
            LiveRouteModel = null;
        }

        // 배관 1개(경로 셀)를 유틸 색 튜브로 라이브 오버레이에 추가(완료 즉시 3D 갱신). UI 스레드에서 호출.
        private void AppendLivePipe(PathCell[] path, Color color, GridMeta grid)
        {
            if (path.Length < 1) return;
            if (_liveGroup == null) { _liveGroup = new Model3DGroup(); LiveRouteModel = _liveGroup; }
            var mb = new MeshBuilder(false, false);
            var pts = path.Select(c => CellToWorld(grid, c)).ToList();
            double dia = grid.CellMm * 0.7, mr = grid.CellMm * 0.9;
            if (pts.Count >= 2) mb.AddTube(pts, dia, 8, false);
            mb.AddSphere(pts[0], mr);
            mb.AddSphere(pts[^1], mr);
            // Model3DGroup 의 Children 변경은 바인딩된 ModelVisual3D 를 즉시 다시 렌더(재대입 불필요).
            _liveGroup.Children.Add(Geometry(mb, color, 235));
        }

        // 선택 PoC 의 시작(초록)·끝(노랑) 점을 강조 구로 그린다. 라우팅과 무관 — 선택 즉시 갱신,
        // 전체 모델을 다시 만들지 않고 별도 오버레이(SelectionModel)만 교체한다(대형 장면에서도 가볍게).
        private void UpdateSelectionHighlight()
        {
            _suppressStepNav = true;
            PathSteps.Clear();
            _suppressStepNav = false;
            StepHighlightModel = null;   // 새 배관 선택 → 이전 구간 강조 제거.
            var t = _selectedTask;
            if (t == null || _scene == null)
            {
                SelectionModel = null;
                CompareModel = null; ComparisonReport = null; _comparePipe = null;
                return;
            }
            var g = _scene.Grid;
            double r = Math.Max(g.CellMm * 1.6, 80);
            var grp = new Model3DGroup();
            var s = new MeshBuilder(false, false);
            s.AddSphere(new Point3D(t.Sx, t.Sy, t.Sz), r);
            grp.Children.Add(Geometry(s, Color.FromRgb(255, 45, 45), 235));    // 시작 PoC = 빨강.
            var e = new MeshBuilder(false, false);
            e.AddSphere(new Point3D(t.Gx, t.Gy, t.Gz), r);
            grp.Children.Add(Geometry(e, Color.FromRgb(50, 120, 255), 235));   // 종단 PoC = 파랑.

            // 선택 배관 경로 강조 — 다른 배관(같은 유틸 색) 사이에서 또렷이 보이도록 굵은 밝은 튜브로 덧그린다.
            // 표시 경로 = 출발 스텁 + A* 중간 + 종단 스텁(스텁 라우팅 시), 없으면 A* 경로만.
            if (t.Path.Length >= 2)
            {
                var pts = new List<Point3D>();
                if (t.StartStub != null) pts.AddRange(t.StartStub.Select(p => new Point3D(p.X, p.Y, p.Z)));
                pts.AddRange(t.Path.Select(c => CellToWorld(g, c)));
                if (t.EndStub != null)
                    for (int k = t.EndStub.Count - 1; k >= 0; k--)
                        pts.Add(new Point3D(t.EndStub[k].X, t.EndStub[k].Y, t.EndStub[k].Z));
                if (pts.Count >= 2)
                {
                    var hm = new MeshBuilder(false, false);
                    hm.AddTube(pts, Math.Max(g.CellMm * 0.5, 45), 12, false);
                    grp.Children.Add(Geometry(hm, Color.FromRgb(255, 235, 90), 255));   // 선택 경로 = 밝은 노랑 강조.
                }
            }

            // 경로가 있으면 방향 전환(꺾임) 지점을 마젠타 구로 표시 + 구간 단계 리스트 구성.
            BuildPathSteps(g, t.Path, grp);

            SelectionModel = grp;
            UpdateComparison();   // 선택 배관 ↔ 기존 설계경로 매칭 오버레이 + 비교 분석 갱신.
        }

        // ---- 기존설계 비교(선택 배관) ----
        /// <summary>'🔬 기존설계 비교' — 선택 배관을 단일 라우팅(개발 경로)하고 비교 포커스 모드로 진입한다.
        /// 비교 포커스에서는 나머지 기존 설계배관 레이어를 숨겨(선택 1개만) 두 경로를 또렷이 대조한다.</summary>
        private async Task CompareSelectedAsync()
        {
            if (_selectedTask == null || _scene == null) return;
            int idx = _selectedTask.Index;
            _compareMode = true;   // BuildModel 가드 → 나머지 기존배관 숨김.
            // RouteRowsAsync 가 단일 충돌회피 라우팅 후 BuildModel→UpdateComparison 까지 수행
            // (개발 경로 포함 전체 분석: 길이·꺾임·종단점·장애물 간섭/여유).
            await RouteRowsAsync(new List<int> { idx }, $"기존설계 비교 #{idx}", corridor: false);
            if (!_selectedTask.Success)
                Status = $"#{idx}: 라우팅 실패 — 개발 경로 없이 기존 경로만 비교합니다.";
        }

        // 선택 배관에 매칭되는 기존 설계경로를 찾아 오버레이/분석을 갱신한다.
        // (UpdateSelectionHighlight 끝과 CompareSelectedAsync→BuildModel 끝에서 호출됨.)
        private void UpdateComparison()
        {
            var t = _selectedTask;
            if (t == null || _scene == null || _scene.ExistingPipes.Count == 0)
            {
                _comparePipe = null;
                CompareModel = null;
                ComparisonReport = _scene == null ? null
                    : "기존 설계배관 데이터가 없습니다(scene.txt 로드 또는 TB_ROUTE_PATH 부재).";
                return;
            }
            _comparePipe = FindMatchingExistingPipe(t);
            BuildComparison(t, _comparePipe);
        }

        // 선택 배관(시작/끝 PoC 좌표)과 가장 잘 맞는 기존 설계경로를 찾는다.
        //   점수 = 양방향 중 작은 (시작↔SOURCE_POS 거리 + 끝↔TARGET_POS 거리). SourcePos/TargetPos 가
        //   없는 행은 폴리라인 양 끝점으로 폴백. 종단점 합산 거리가 임계 초과면 매칭 없음(null).
        private ExistingPipe? FindMatchingExistingPipe(TaskRowVM t)
        {
            var s = _scene; if (s == null || s.ExistingPipes.Count == 0) return null;
            var ts = new Pt3(t.Sx, t.Sy, t.Sz);
            var te = new Pt3(t.Gx, t.Gy, t.Gz);
            double tol = Math.Max(3 * s.Grid.CellMm, 1500.0);   // 종단점당 허용 거리(mm).
            ExistingPipe? best = null; double bestScore = double.MaxValue;
            foreach (var p in s.ExistingPipes)
            {
                if (p.Points.Count < 2) continue;
                Pt3 ps = p.SourcePos ?? p.Points[0];
                Pt3 pe = p.TargetPos ?? p.Points[p.Points.Count - 1];
                double score = Math.Min(Dist(ts, ps) + Dist(te, pe), Dist(ts, pe) + Dist(te, ps));
                if (score < bestScore) { bestScore = score; best = p; }
            }
            return (best != null && bestScore <= tol * 2) ? best : null;   // 시작+끝 합산이므로 *2.
        }

        // 매칭된 기존 경로(ex)와 선택 배관(t)의 개발 경로를 정량 비교해 ComparisonReport + CompareModel 갱신.
        private void BuildComparison(TaskRowVM t, ExistingPipe? ex)
        {
            var s = _scene!; var g = s.Grid;
            var devPts = t.Path.Length >= 2 ? t.Path.Select(c => CellToWorld(g, c)).ToList() : new List<Point3D>();
            bool devRouted = t.Success && devPts.Count >= 2;

            if (ex == null)
            {
                ComparisonReport =
                    "매칭되는 기존 설계경로 없음.\n(이 PoC에 해당하는 TB_ROUTE_PATH 가 없거나 종단점이 너무 멉니다.)"
                    + (devRouted ? $"\n\n개발 경로: {t.LengthMm:0} mm · 꺾임 {CountBends(t.Path).Text}" : "");
                CompareModel = BuildCompareOverlay(null, devRouted ? devPts : null);
                return;
            }

            var exPts = ex.Points.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList();
            double exLen = PolylineLength(exPts);
            BendStats exBends = CountBends(exPts);

            var sb = new StringBuilder();
            sb.AppendLine($"매칭: {ex.Label}");
            sb.AppendLine($"관경: {(ex.DiameterMm > 0 ? ex.DiameterMm.ToString("0") + " mm" : "미상")}");

            // 종단점 정합 — 양방향 중 가까운 쪽으로 매칭(역방향이면 표기).
            var ts = new Pt3(t.Sx, t.Sy, t.Sz); var te = new Pt3(t.Gx, t.Gy, t.Gz);
            Pt3 ps = ex.SourcePos ?? new Pt3(exPts[0].X, exPts[0].Y, exPts[0].Z);
            Pt3 pe = ex.TargetPos ?? new Pt3(exPts[exPts.Count - 1].X, exPts[exPts.Count - 1].Y, exPts[exPts.Count - 1].Z);
            bool rev = (Dist(ts, pe) + Dist(te, ps)) < (Dist(ts, ps) + Dist(te, pe));
            double sg = rev ? Dist(ts, pe) : Dist(ts, ps);
            double eg = rev ? Dist(te, ps) : Dist(te, pe);
            sb.AppendLine($"종단점 정합: 시작 {sg:0}mm · 끝 {eg:0}mm{(rev ? " (역방향)" : "")}");
            sb.AppendLine();

            sb.AppendLine("─ 경로 길이 ─");
            if (devRouted)
            {
                double diff = t.LengthMm - exLen;
                double pct = exLen > 1 ? diff / exLen * 100 : 0;
                sb.AppendLine($"  기존 {exLen:0} / 개발 {t.LengthMm:0} mm");
                sb.AppendLine($"  차이 {(diff >= 0 ? "+" : "")}{diff:0} mm ({(pct >= 0 ? "+" : "")}{pct:0.0}%) — 개발 {(diff < 0 ? "짧음" : "긺")}");
            }
            else sb.AppendLine($"  기존 {exLen:0} mm / 개발 (미라우팅)");

            sb.AppendLine();
            sb.AppendLine("─ 꺾임(엘보) 수 ─");
            sb.AppendLine($"  기존 {exBends.Text}");
            if (devRouted)
            {
                var devBends = CountBends(t.Path);
                sb.AppendLine($"  개발 {devBends.Text}");
            }
            else sb.AppendLine("  개발 —");

            sb.AppendLine();
            sb.AppendLine("─ 장애물 간섭 / 여유 ─");
            int exHit = CountObstacleHits(exPts, out double exClr, out int exSamples);
            sb.AppendLine($"  기존: 간섭 {exHit}/{exSamples}점 · 최소여유 {exClr:0} mm");
            if (devRouted)
            {
                int devHit = CountObstacleHits(devPts, out double devClr, out int devSamples);
                sb.AppendLine($"  개발: 간섭 {devHit}/{devSamples}점 · 최소여유 {devClr:0} mm");
                if (exHit > 0 && devHit == 0)
                    sb.AppendLine("  → 개발 경로가 기존의 장애물 간섭을 해소");
            }
            else sb.AppendLine("  개발: (미라우팅 — '🔬 기존설계 비교' 실행)");

            ComparisonReport = sb.ToString().TrimEnd();
            CompareModel = BuildCompareOverlay(exPts, devRouted ? devPts : null);
        }

        // 비교 오버레이: 기존 경로(주황) + 개발 경로(시안) 굵은 튜브 + 시작/끝 구 마커.
        private Model3D? BuildCompareOverlay(List<Point3D>? exPts, List<Point3D>? devPts)
        {
            if (_scene == null) return null;
            bool hasEx = exPts != null && exPts.Count >= 2;
            bool hasDev = devPts != null && devPts.Count >= 2;
            if (!hasEx && !hasDev) return null;
            double cell = _scene.Grid.CellMm;
            double dia = Math.Max(cell * 0.9, 60);   // 일반 경로보다 굵게(눈에 띄게).
            double mr = Math.Max(cell * 1.1, 70);
            var grp = new Model3DGroup();
            if (hasEx)
            {
                var mb = new MeshBuilder(false, false);
                mb.AddTube(exPts, dia, 12, false);
                mb.AddSphere(exPts![0], mr); mb.AddSphere(exPts[exPts.Count - 1], mr);
                grp.Children.Add(Geometry(mb, Color.FromRgb(255, 144, 48), 245));   // 기존 = 주황.
            }
            if (hasDev)
            {
                var mb = new MeshBuilder(false, false);
                mb.AddTube(devPts, dia, 12, false);
                mb.AddSphere(devPts![0], mr); mb.AddSphere(devPts[devPts.Count - 1], mr);
                grp.Children.Add(Geometry(mb, Color.FromRgb(48, 208, 255), 245));   // 개발 = 시안.
            }
            return grp;
        }

        // 폴리라인을 step 간격으로 샘플해 비통과 장애물 AABB 와의 간섭(내부 점 수)과 최소 여유(mm)를 잰다.
        //   간섭 = 장애물 내부에 든 샘플 점 수. 여유 = 외부 점에서 가장 가까운 장애물 표면까지 최소 거리.
        private int CountObstacleHits(List<Point3D> pts, out double minClearance, out int sampleCount)
        {
            minClearance = double.MaxValue; sampleCount = 0;
            var s = _scene;
            if (s == null || pts.Count < 2) { minClearance = 0; return 0; }
            double step = Math.Max(s.Grid.CellMm * 0.5, 25);
            int hits = 0;
            var samples = SamplePolyline(pts, step);
            sampleCount = samples.Count;
            foreach (var pt in samples)
            {
                double nearest = double.MaxValue; bool inside = false;
                foreach (var o in s.Obstacles)
                {
                    if (o.IsPassThrough) continue;
                    double d = DistToAabb(pt, o, out bool ins);
                    if (ins) { inside = true; break; }
                    if (d < nearest) nearest = d;
                }
                if (inside) hits++;
                else if (nearest < minClearance) minClearance = nearest;
            }
            if (minClearance == double.MaxValue) minClearance = 0;
            return hits;
        }

        // 점에서 AABB 까지의 외부(유클리드) 거리. 점이 박스 내부면 0 이고 inside=true.
        private static double DistToAabb(Point3D p, ObstacleBox o, out bool inside)
        {
            double dx = Math.Max(Math.Max(o.MinX - p.X, p.X - o.MaxX), 0);
            double dy = Math.Max(Math.Max(o.MinY - p.Y, p.Y - o.MaxY), 0);
            double dz = Math.Max(Math.Max(o.MinZ - p.Z, p.Z - o.MaxZ), 0);
            inside = dx == 0 && dy == 0 && dz == 0;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        // 폴리라인을 step(mm) 간격 점열로 균등 샘플(각 구간 시작점 포함, 끝점 포함).
        private static List<Point3D> SamplePolyline(List<Point3D> pts, double step)
        {
            var outp = new List<Point3D>();
            if (pts.Count == 0) return outp;
            outp.Add(pts[0]);
            for (int i = 1; i < pts.Count; i++)
            {
                Point3D a = pts[i - 1], b = pts[i];
                double len = (b - a).Length;
                int n = Math.Max(1, (int)(len / step));
                for (int k = 1; k <= n; k++)
                {
                    double f = (double)k / n;
                    outp.Add(new Point3D(a.X + (b.X - a.X) * f, a.Y + (b.Y - a.Y) * f, a.Z + (b.Z - a.Z) * f));
                }
            }
            return outp;
        }

        private static double PolylineLength(List<Point3D> pts)
        {
            double L = 0;
            for (int i = 1; i < pts.Count; i++) L += (pts[i] - pts[i - 1]).Length;
            return L;
        }

        private static double Dist(Pt3 a, Pt3 b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>꺾임(엘보) 통계 — 전체 수와 수평/수직 분류. 수직 = 인접 구간 중 하나라도 Z(상하)
        /// 방향인 꺾임(층 전환 엘보), 수평 = XY 평면 내 방향 전환.</summary>
        private readonly struct BendStats
        {
            public readonly int Total, Horiz, Vert;
            public BendStats(int total, int horiz, int vert) { Total = total; Horiz = horiz; Vert = vert; }
            /// <summary>"N회 (수평 H · 수직 V)" 표기.</summary>
            public string Text => $"{Total}회 (수평 {Horiz} · 수직 {Vert})";
        }

        // 격자 경로(셀)의 방향 전환(꺾임) 통계 — BuildPathSteps 의 Dir 부호 비교 규약과 동일.
        //   꺾임 직전/직후 구간 중 하나라도 Z 성분(dz≠0)이면 수직 꺾임, 아니면 수평 꺾임으로 센다.
        private static BendStats CountBends(PathCell[] path)
        {
            if (path.Length < 3) return new BendStats(0, 0, 0);
            (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));
            int total = 0, horiz = 0, vert = 0;
            var cur = Dir(path[0], path[1]);
            for (int i = 2; i < path.Length; i++)
            {
                var d = Dir(path[i - 1], path[i]);
                if (d != cur)
                {
                    total++;
                    if (cur.dz != 0 || d.dz != 0) vert++; else horiz++;
                    cur = d;
                }
            }
            return new BendStats(total, horiz, vert);
        }

        // 월드 폴리라인의 방향 전환(꺾임) 통계 — 인접 구간 단위벡터 내적이 ~1 미만이면 한 번 꺾인 것으로.
        //   꺾임 직전/직후 구간 중 하나라도 |Z| 성분이 지배적(>0.7≈45°↑)이면 수직 꺾임, 아니면 수평 꺾임.
        private static BendStats CountBends(List<Point3D> pts)
        {
            if (pts.Count < 3) return new BendStats(0, 0, 0);
            int total = 0, horiz = 0, vert = 0;
            Vector3D? prev = null;
            for (int i = 1; i < pts.Count; i++)
            {
                Vector3D d = pts[i] - pts[i - 1];
                if (d.Length < 1e-6) continue;
                d.Normalize();
                if (prev.HasValue)
                {
                    var p = prev.Value;
                    double dot = p.X * d.X + p.Y * d.Y + p.Z * d.Z;
                    if (dot < 0.999)   // ~2.6° 이상 차이 → 꺾임.
                    {
                        total++;
                        if (Math.Abs(p.Z) > 0.7 || Math.Abs(d.Z) > 0.7) vert++; else horiz++;
                    }
                }
                prev = d;
            }
            return new BendStats(total, horiz, vert);
        }

        // 경로 단계(구간) 강조 — 카메라는 그대로 두고(현재 화면 유지), 선택 구간만 흰색 굵은 튜브로 덧그린다.
        private void HighlightStep(PathStep? step)
        {
            if (step == null || _scene == null) { StepHighlightModel = null; return; }
            var g = _scene.Grid;
            var mb = new MeshBuilder(false, false);
            double dia = Math.Max(g.CellMm * 1.3, 70);
            if (step.A != step.B) mb.AddCylinder(step.A, step.B, dia, 12);
            double r = Math.Max(g.CellMm * 0.9, 50);
            mb.AddSphere(step.A, r);
            mb.AddSphere(step.B, r);
            StepHighlightModel = Geometry(mb, Color.FromRgb(255, 255, 255), 245);   // 선택 구간 = 흰색 강조.
        }

        // 경로 셀을 직선 구간(같은 축으로 진행)별로 나눠, 방향이 바뀌는 꺾임점을 마커로 찍고
        // 각 구간을 PathSteps 에 담는다(클릭 시 해당 위치로 카메라 이동).
        private void BuildPathSteps(GridMeta g, PathCell[] path, Model3DGroup grp)
        {
            if (path.Length < 2) return;
            double bendR = Math.Max(g.CellMm * 1.1, 60);
            var bendMb = new MeshBuilder(false, false);

            // 각 스텝의 단위 방향(부호) 벡터.
            (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));

            int segStart = 0;
            var curDir = Dir(path[0], path[1]);
            int stepNo = 1;
            for (int i = 2; i <= path.Length; i++)
            {
                var d = (i < path.Length) ? Dir(path[i - 1], path[i]) : (int.MinValue, 0, 0);
                if (d != curDir)
                {
                    // 구간 [segStart .. i-1] 종료.
                    var pStart = CellToWorld(g, path[segStart]);
                    var pEnd = CellToWorld(g, path[i - 1]);
                    double len = (pEnd - pStart).Length;
                    PathSteps.Add(new PathStep
                    {
                        Label = $"{stepNo,2}. {DirText(curDir)}  {len:0} mm",
                        Position = pStart,
                        A = pStart,
                        B = pEnd,
                    });
                    stepNo++;
                    // 꺾임점(구간 끝 = 다음 구간 시작) 마커. 마지막(끝점)은 제외.
                    if (i < path.Length) bendMb.AddSphere(pEnd, bendR);
                    segStart = i - 1;
                    if (i < path.Length) curDir = Dir(path[i - 1], path[i]);
                }
            }
            if (bendMb.Positions != null && bendMb.Positions.Count > 0)
                grp.Children.Add(Geometry(bendMb, Color.FromRgb(255, 80, 220), 240));   // 꺾임 = 마젠타.
        }

        // 단위 방향 벡터 → '수직(Z)' / '수평(X)' / '수평(Y)' 라벨.
        private static string DirText((int dx, int dy, int dz) d)
        {
            if (d.dz != 0) return d.dz > 0 ? "수직 ↑(Z+)" : "수직 ↓(Z-)";
            if (d.dx != 0) return d.dx > 0 ? "수평 →(X+)" : "수평 ←(X-)";
            if (d.dy != 0) return d.dy > 0 ? "수평 ↗(Y+)" : "수평 ↙(Y-)";
            return "—";
        }

        // 격자 BBOX 의 12 변을 가는 실린더로 그린다(복셀 전체맵 = 작업 공간 프레임).
        private static void AddGridFrame(Model3DGroup group, GridMeta g)
        {
            var lo = new Point3D(g.Ox, g.Oy, g.Oz);
            var hi = new Point3D(g.Ox + g.Nx * g.CellMm, g.Oy + g.Ny * g.CellMm, g.Oz + g.Nz * g.CellMm);
            AddBoxFrame(group, lo, hi, Color.FromRgb(122, 223, 176), Math.Max(g.CellMm * 0.08, 5), 230);
        }

        // 공간 영역(CR/A/F/CSF 등)을 영역별 색 와이어프레임으로 그리고, 중앙 상단에 텍스트 라벨을 둔다.
        private void AddSpaceAreas(Model3DGroup group, List<SpaceArea> spaces)
        {
            var colorMap = UtilityColors.Assign(spaces.Select(s => s.Name));   // 이름 기준 결정적 색.
            foreach (var sp in spaces)
            {
                var lo = new Point3D(sp.MinX, sp.MinY, sp.MinZ);
                var hi = new Point3D(sp.MaxX, sp.MaxY, sp.MaxZ);
                var color = colorMap.TryGetValue(sp.Name, out var c) ? c : Colors.Gold;
                // 변 굵기 = 영역 크기에 비례(너무 가늘면 안 보이고 너무 굵으면 장애물 가림).
                double r = Math.Max((hi.X - lo.X + hi.Y - lo.Y) * 0.0008, 25);
                AddBoxFrame(group, lo, hi, color, r, 235);
                // 텍스트 라벨 위치 = 영역 박스 '바깥'(+X 면에서 더 떨어진 곳), 각 층의 수직 중앙.
                // 층(CSF/A/F/CR)이 Z 로 쌓이므로 같은 옆면에 서로 다른 높이로 나란히 표시된다.
                double offset = Math.Max((hi.X - lo.X) * 0.06, 800);
                SpaceLabels.Add(new SpaceLabel
                {
                    Text = sp.Name,
                    Position = new Point3D(hi.X + offset, (lo.Y + hi.Y) / 2, (lo.Z + hi.Z) / 2),
                    Color = color,
                });
            }
        }

        // 임의의 AABB(lo~hi) 12 변을 실린더 와이어프레임으로 그린다(격자/공간 영역 공용).
        private static void AddBoxFrame(Model3DGroup group, Point3D lo, Point3D hi, Color color, double radius, byte alpha)
        {
            var corners = new[]
            {
                new Point3D(lo.X,lo.Y,lo.Z), new Point3D(hi.X,lo.Y,lo.Z), new Point3D(hi.X,hi.Y,lo.Z), new Point3D(lo.X,hi.Y,lo.Z),
                new Point3D(lo.X,lo.Y,hi.Z), new Point3D(hi.X,lo.Y,hi.Z), new Point3D(hi.X,hi.Y,hi.Z), new Point3D(lo.X,hi.Y,hi.Z),
            };
            var edges = new (int, int)[]
            {
                (0,1),(1,2),(2,3),(3,0), (4,5),(5,6),(6,7),(7,4), (0,4),(1,5),(2,6),(3,7)
            };
            var mb = new MeshBuilder(false, false);
            foreach (var (a, b) in edges) mb.AddCylinder(corners[a], corners[b], radius, 8);
            group.Children.Add(Geometry(mb, color, alpha));
        }

        // 점유맵 — 엔진이 voxelize 한 블록 셀을 '셀 크기' 큐브로(반투명). 큐브가 맞닿아 장애물을
        // 빈틈 없이 채운다. 상한(Cap) 초과 시에만 균등 다운샘플(메시 부하 한도).
        // 반환값 = (실제 그린 셀 수, 전체 블록 셀 수). down=rendered<total 이면 다운샘플됨.
        private (int rendered, int total) AddOccupancyVoxels(Model3DGroup group, GridMeta g)
        {
            // 큐브 1개 ≈ 12삼각형. 다운샘플 모드는 15만 상한(~180만 삼각형, 단일 병합 메시).
            // 원본 모드(_occupancyFullRes)는 상한 없이 전체 셀 표시(대형 장면에선 느릴 수 있음 — 사용자 선택).
            int cap = _occupancyFullRes ? int.MaxValue : 150_000;
            var cells = _engine!.CopyBlocked();
            if (cells.Length == 0) return (0, 0);
            int take = Math.Min(cap, cells.Length);
            double stride = (double)cells.Length / take;
            double s = g.CellMm;   // 셀 크기와 동일 → 인접 큐브가 맞닿아 빈틈 없이 채움(이전 0.9 → 점박이).
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++)
            {
                var c = cells[(int)(n * stride)];
                mb.AddBox(CellToWorld(g, c), s, s, s);
            }
            group.Children.Add(Geometry(mb, Color.FromRgb(130, 170, 200), 120));   // 옅은 청회색 반투명.
            return (take, cells.Length);
        }

        private static GeometryModel3D Geometry(MeshBuilder mb, Color color, byte alpha)
        {
            var mat = MaterialFor(color, alpha);
            return new GeometryModel3D { Geometry = mb.ToMesh(), Material = mat, BackMaterial = mat };
        }

        private static Material MaterialFor(Color color, byte alpha)
        {
            var c = Color.FromArgb(alpha, color.R, color.G, color.B);
            return new DiffuseMaterial(new SolidColorBrush(c));
        }

        // 색을 흰색 쪽으로 t(0~1) 만큼 보간해 밝게 만든다 — 자동 경로(= 유틸 색을 밝게)와 기존 설계배관
        //   (= 유틸 원색)을 같은 계열의 명도 차로 구분하는 데 쓴다. t=0 원색, t=1 흰색.
        private static Color Lighten(Color c, double t)
        {
            byte L(byte v) => (byte)Math.Round(v + (255 - v) * t);
            return Color.FromRgb(L(c.R), L(c.G), L(c.B));
        }

        // ============================================================ 진행 다이얼로그 — 선택 배관 상세
        // 표 행 클릭 시 그 배관(taskIndex)의 로컬 영역만 미니 3D 로 합성하고 결과 설명을 만든다.
        // SceneModel 과 독립(자체 Model3DGroup) — 다이얼로그 미니 뷰포트에 표시. 라우팅 완료 후
        // TaskRowVM 에 경로/방문이 캐시(CacheResults)돼 있으므로 그 데이터를 쓴다.
        // layers = [복셀맵, 점유맵, 방문맵, 최종경로].
        public Routing3D.Viewer.Views.PipeDetail BuildPipeDetail(int taskIndex, bool[] layers)
        {
            var group = new Model3DGroup();
            if (_scene == null || taskIndex < 0 || taskIndex >= Tasks.Count)
                return new Routing3D.Viewer.Views.PipeDetail(group, new Rect3D(), "데이터 없음");
            var grid = _scene.Grid;
            var row = Tasks[taskIndex];
            bool gOn = layers.Length > 0 && layers[0];
            bool oOn = layers.Length > 1 && layers[1];
            bool vOn = layers.Length > 2 && layers[2];
            bool pOn = layers.Length > 3 && layers[3];

            // 전체 격자(도메인) 기준으로 표시한다 — 메인 뷰/실제 BIM 처럼 '모든 셀'을 대상으로 그려,
            // 배관을 전체 장애물 맥락 안에서 본다(이전엔 배관 로컬 BBOX 로 잘라 부분 슬래브만 보여 혼동).
            var dlo = new Point3D(grid.Ox, grid.Oy, grid.Oz);
            var dhi = new Point3D(grid.Ox + grid.Nx * grid.CellMm,
                                  grid.Oy + grid.Ny * grid.CellMm,
                                  grid.Oz + grid.Nz * grid.CellMm);

            // 색상 규약: 복셀맵=회색 선형 틀 · 점유맵=적색 · 방문맵=노랑 · 최종경로=파랑(꺾임 셀=녹색).
            if (gOn) AddBoxFrame(group, dlo, dhi, Color.FromRgb(150, 152, 160), Math.Max(grid.CellMm * 0.12, 8), 200);
            if (oOn) AddFullOccupancy(group, grid);
            if (vOn && row.Visited.Length > 0) AddLocalVisited(group, grid, row.Visited);
            if (pOn && row.Path.Length >= 1) AddPipePath(group, grid, row);

            var box = new Rect3D(dlo.X, dlo.Y, dlo.Z, dhi.X - dlo.X, dhi.Y - dlo.Y, dhi.Z - dlo.Z);
            return new Routing3D.Viewer.Views.PipeDetail(group, box, ExplainPipe(taskIndex));
        }

        // 메인 뷰에서 그 배관을 '선택'(경로 강조 + 종단 마커) 하고 그 로컬 영역으로 줌.
        public void FocusPipeInMainView(int taskIndex)
        {
            if (_scene == null || taskIndex < 0 || taskIndex >= Tasks.Count) return;
            SelectedTask = Tasks[taskIndex];   // → UpdateSelectionHighlight: 선택 경로 밝게 강조 + 시작/종단 마커.
            var (lo, hi) = PipeWorldBounds(Tasks[taskIndex], _scene.Grid);
            ZoomToBoxRequested?.Invoke(new Rect3D(lo.X, lo.Y, lo.Z, hi.X - lo.X, hi.Y - lo.Y, hi.Z - lo.Z));
        }

        // 배관(경로+시작/종단, 실패면 방문까지) 을 감싸는 월드 AABB + 여유.
        private (Point3D lo, Point3D hi) PipeWorldBounds(TaskRowVM row, GridMeta g)
        {
            double nx = double.MaxValue, ny = double.MaxValue, nz = double.MaxValue;
            double xx = double.MinValue, xy = double.MinValue, xz = double.MinValue;
            void Acc(double x, double y, double z)
            {
                nx = Math.Min(nx, x); ny = Math.Min(ny, y); nz = Math.Min(nz, z);
                xx = Math.Max(xx, x); xy = Math.Max(xy, y); xz = Math.Max(xz, z);
            }
            foreach (var c in row.Path) { var p = CellToWorld(g, c); Acc(p.X, p.Y, p.Z); }
            Acc(row.Sx, row.Sy, row.Sz); Acc(row.Gx, row.Gy, row.Gz);
            if (row.Path.Length == 0)
                foreach (var c in row.Visited) { var p = CellToWorld(g, c); Acc(p.X, p.Y, p.Z); }
            double m = Math.Max(g.CellMm * 3, 800);
            return (new Point3D(nx - m, ny - m, nz - m), new Point3D(xx + m, xy + m, xz + m));
        }

        // 점유맵 — 전체 격자의 모든 블록(장애물) 셀을 '셀 크기' 적색 큐브로(반투명, 인접 큐브가 맞닿아
        // 빈틈 없이 채움). 상한(cap) 초과 시에만 균등 다운샘플(메인 뷰 점유맵과 동일 정책). bbox 클리핑 없음.
        private void AddFullOccupancy(Model3DGroup group, GridMeta g)
        {
            if (_engine == null) return;
            var cells = _engine.CopyBlocked();
            if (cells.Length == 0) return;
            const int cap = 150_000;
            int take = Math.Min(cap, cells.Length);
            double stride = (double)cells.Length / take;
            double s = g.CellMm;
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++) { var c = cells[(int)(n * stride)]; mb.AddBox(CellToWorld(g, c), s, s, s); }
            group.Children.Add(Geometry(mb, Color.FromRgb(220, 70, 60), 120));   // 점유맵=적색.
        }

        // 이 배관 A* 가 확장한 방문 셀(노란색 반투명, 점유보다 작은 큐브). 상한 초과 시 균등 다운샘플.
        private static void AddLocalVisited(Model3DGroup group, GridMeta g, PathCell[] visited)
        {
            const int cap = 40000;
            int take = Math.Min(cap, visited.Length);
            double stride = (double)visited.Length / take;
            double s = g.CellMm * 0.7;
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++) { var c = visited[(int)(n * stride)]; mb.AddBox(CellToWorld(g, c), s, s, s); }
            group.Children.Add(Geometry(mb, Color.FromRgb(235, 205, 60), 80));   // 방문맵=노랑.
        }

        // 최종 경로 — 경로 튜브(파랑) + 경로를 구성하는 셀 큐브(직선=파랑·꺾임=녹색) + 시작(빨강)/종단(파랑) 구.
        private static void AddPipePath(Model3DGroup group, GridMeta g, TaskRowVM row)
        {
            double dia = Math.Max(g.CellMm * 0.35, 30);
            var path = row.Path;

            // (1) 경로 튜브(스텁 포함) — 파랑.
            if (path.Length >= 1)
            {
                var pts = new List<Point3D>();
                if (row.StartStub != null) pts.AddRange(row.StartStub.Select(p => new Point3D(p.X, p.Y, p.Z)));
                pts.AddRange(path.Select(c => CellToWorld(g, c)));
                if (row.EndStub != null)
                    for (int k = row.EndStub.Count - 1; k >= 0; k--)
                        pts.Add(new Point3D(row.EndStub[k].X, row.EndStub[k].Y, row.EndStub[k].Z));
                if (pts.Count >= 2)
                {
                    var mb = new MeshBuilder(false, false);
                    mb.AddTube(pts, dia, 12, false);
                    group.Children.Add(Geometry(mb, Color.FromRgb(60, 150, 240), 255));   // 경로 튜브=파랑.
                }
            }

            // (2) 경로를 구성하는 셀 큐브 — 직선 셀=파랑, 방향이 바뀌는 꺾임 셀=녹색.
            if (path.Length >= 1)
            {
                double s = g.CellMm * 0.55;
                var straight = new MeshBuilder(false, false);
                var bend = new MeshBuilder(false, false);
                (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                    (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));
                for (int i = 0; i < path.Length; i++)
                {
                    bool isBend = i > 0 && i < path.Length - 1 && Dir(path[i - 1], path[i]) != Dir(path[i], path[i + 1]);
                    (isBend ? bend : straight).AddBox(CellToWorld(g, path[i]), s, s, s);
                }
                if (straight.Positions.Count > 0) group.Children.Add(Geometry(straight, Color.FromRgb(70, 130, 230), 205));   // 경로 셀=파랑.
                if (bend.Positions.Count > 0) group.Children.Add(Geometry(bend, Color.FromRgb(80, 210, 110), 255));            // 꺾임 셀=녹색.
            }

            // (3) 시작/종단 구 마커.
            var sm = new MeshBuilder(false, false); sm.AddSphere(new Point3D(row.Sx, row.Sy, row.Sz), dia * 1.6);
            group.Children.Add(Geometry(sm, Color.FromRgb(230, 80, 80), 255));   // 시작=빨강.
            var em = new MeshBuilder(false, false); em.AddSphere(new Point3D(row.Gx, row.Gy, row.Gz), dia * 1.6);
            group.Children.Add(Geometry(em, Color.FromRgb(80, 120, 230), 255));  // 종단=파랑.
        }

        // 라우팅 결과를 사람이 읽을 설명으로(성공: 길이/우회율/꺾임/탐색 · 실패: 막힘 유형).
        public string ExplainPipe(int taskIndex)
        {
            if (_scene == null || taskIndex < 0 || taskIndex >= Tasks.Count) return "데이터 없음";
            var row = Tasks[taskIndex];
            var sb = new System.Text.StringBuilder();
            string s = string.IsNullOrEmpty(row.PocName) ? $"({row.Sx:0},{row.Sy:0},{row.Sz:0})" : row.PocName!;
            string e = string.IsNullOrEmpty(row.EndName) ? $"({row.Gx:0},{row.Gy:0},{row.Gz:0})" : row.EndName!;
            sb.AppendLine($"[{row.Group} / {row.Utility}]");
            sb.AppendLine($"{s}  →  {e}");
            sb.AppendLine();
            double man = Math.Abs(row.Gx - row.Sx) + Math.Abs(row.Gy - row.Sy) + Math.Abs(row.Gz - row.Sz);
            int visited = row.Visited.Length;
            if (row.Success && row.Path.Length >= 2)
            {
                var b = CountBends(row.Path);
                double len = row.LengthMm;
                double detour = man > 1 ? (len / man - 1.0) * 100.0 : 0.0;
                sb.AppendLine("결과 : ✅ 성공");
                sb.AppendLine($"· 경로 길이    : {len:#,0} mm");
                sb.AppendLine($"· 직선(맨해튼) : {man:#,0} mm");
                sb.AppendLine($"· 우회율       : {detour:0.0} %");
                sb.AppendLine($"· 꺾임(엘보)   : {b.Total} 회 (수평 {b.Horiz}·수직 {b.Vert})");
                sb.AppendLine($"· 경로 셀      : {row.Path.Length:#,0}");
                sb.AppendLine($"· 탐색(방문)   : {visited:#,0} 셀");
                if (row.StartStub != null || row.EndStub != null)
                    sb.AppendLine($"· 스텁         : 출발 {(row.StartStub != null ? "○" : "—")} · 종단 {(row.EndStub != null ? "○" : "—")} (기존설계 추종)");
                sb.AppendLine();
                sb.AppendLine(detour < 15 ? "→ 직선에 가깝게 효율적으로 연결되었습니다."
                            : detour < 50 ? "→ 장애물/회랑을 우회하며 연결되었습니다."
                            : "→ 우회가 큽니다(혼잡·회랑 바이어스). 방문맵으로 탐색 범위를 확인하세요.");
            }
            else
            {
                sb.AppendLine("결과 : ❌ 실패 (경로 없음)");
                sb.AppendLine($"· 직선(맨해튼) : {man:#,0} mm");
                sb.AppendLine($"· 탐색(방문)   : {visited:#,0} 셀");
                sb.AppendLine();
                if (visited <= 1)
                    sb.AppendLine("→ 시작/종단 셀이 장애물 내부입니다(PoC가 솔리드에 파묻힘).\n   탐색이 거의 없음 = 출발조차 못함. 면 투영/스냅 대상.");
                else if (visited >= 11_000_000)
                    sb.AppendLine("→ 탐색 상한 도달: 경로가 매우 길거나 사실상 막혔습니다.\n   방문맵이 넓게 퍼진 뒤 종단에 못 닿음.");
                else
                    sb.AppendLine("→ 종단까지 완전 차단되었습니다.\n   방문맵으로 어디까지 탐색하고 막혔는지 확인하세요(rip-up/CBS 대상).");
            }
            sb.AppendLine();
            sb.AppendLine("─ 색상 ─");
            sb.AppendLine("복셀맵=회색 틀 · 점유맵=적색");
            sb.AppendLine("방문맵=노랑 · 경로=파랑(꺾임=녹색)");
            return sb.ToString();
        }
    }
}

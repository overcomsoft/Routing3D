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

            if (ShowSpaces)
                foreach (var sp in s.Spaces)
                    Consider(DescribeSpace(sp), sp.MinX, sp.MinY, sp.MinZ, sp.MaxX, sp.MaxY, sp.MaxZ);

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
        private string _searchText = string.Empty;
        private bool _suppressFilterRebuild;   // BuildTaskRows 중 IsVisible 이벤트 폭주 방지.

        // DB 접속 설정(환경변수 우선) + 선택된 프로젝트 / 격자 셀 크기.
        private readonly DbConfig _dbConfig = DbConfig.FromEnv();
        private ProjectInfo? _selectedProject;
        private double _cellMm = 100.0;
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
        private bool _hidePathsForAnim;     // 단계별 탐색 중 최종 경로를 숨겼다가 끝에 드러내기.
        private bool _animating;            // 단계별 탐색 진행 중(중복 실행 방지).

        public SceneViewModel(string? initialScene = null)
        {
            OpenCommand = new RelayCommand(Open);
            DemoCommand = new RelayCommand(LoadDemo);
            RunRouteCommand = new RelayCommand(() => _ = RunRouteAsync(), () => _scene != null);
            RerouteCorridorCommand = new RelayCommand(
                () => _ = RouteRowsAsync(AllRows(), "corridor 전체", corridor: true),
                () => _scene != null);
            RerouteSelectedCommand = new RelayCommand(
                () => { if (_selectedTask != null) _ = RouteRowsAsync(new List<int> { _selectedTask.Index }, $"선택 #{_selectedTask.Index}", corridor: false); },
                () => _selectedTask != null);
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
            set { if (Set(ref _selectedTask, value)) UpdateSelectionHighlight(); }
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

        /// <summary>선택 경로의 직선 구간(단계) 목록. 방향이 바뀌는 지점마다 한 항목.</summary>
        public ObservableCollection<PathStep> PathSteps { get; } = new();

        private PathStep? _selectedStep;
        private bool _suppressStepNav;   // 목록 재구성 중 자동 네비게이션 방지.
        /// <summary>선택된 단계. 사용자가 목록에서 고르면 그 위치로 카메라를 이동(NavigateToRequested).</summary>
        public PathStep? SelectedStep
        {
            get => _selectedStep;
            set { if (Set(ref _selectedStep, value) && value != null && !_suppressStepNav) NavigateToRequested?.Invoke(value.Position); }
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
        public RelayCommand PickStartCommand { get; }
        public RelayCommand PickEndCommand { get; }
        public RelayCommand FitViewCommand { get; }
        public RelayCommand ToggleOccupancyResCommand { get; }
        public RelayCommand AnimateSelectedCommand { get; }
        public RelayCommand UtilityAllCommand { get; }
        public RelayCommand UtilityClearCommand { get; }
        public RelayCommand LoadProjectsCommand { get; }
        public RelayCommand LoadDbCommand { get; }

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
            await RouteRowsAsync(rows, label, corridor: false);
        }

        private List<int> RowsWhere(Func<TaskRowVM, bool> pred)
        {
            var list = new List<int>();
            for (int i = 0; i < Tasks.Count; i++) if (pred(Tasks[i])) list.Add(i);
            return list;
        }

        // 엔진을 [장애물 전체 + 지정 행들의 작업]만으로 재구성한다(부분집합 충돌회피 라우팅용).
        // 반환: 적재한 행 위치 목록(순서 = 엔진 작업 인덱스). 종단점은 행에서 직접 읽는다(편집 반영).
        private List<int> BuildEngineForRows(IReadOnlyList<int> rowPositions)
        {
            var scene = _scene!;
            ResetEngine();
            var g = scene.Grid;
            _engine!.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
            _engine.SetParams(g.CellMm, 500, 10, 2, 6);
            foreach (var o in scene.Obstacles)
                if (o.IsPassThrough)   // 통과 객체: 점유맵엔 넣되 A* 충돌엔 제외.
                    _engine.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                else
                    _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            var added = new List<int>(rowPositions.Count);
            foreach (var pos in rowPositions)
            {
                var row = Tasks[pos];
                _engine.AddTask(row.Sx, row.Sy, row.Sz, row.Gx, row.Gy, row.Gz, row.Utility, row.Group);
                added.Add(pos);
            }
            return added;
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
        private async Task RouteRowsAsync(IReadOnlyList<int> rowPositions, string label, bool corridor)
        {
            if (_scene == null || rowPositions.Count == 0) return;
            try
            {
                var added = BuildEngineForRows(rowPositions);
                Status = $"경로 탐색 중… {label} (작업 {added.Count})";
                var engine = _engine!;
                bool cor = corridor;
                await Task.Run(() => { if (cor) engine.RouteCorridor(16, 2); else engine.RouteMulti(_priority); });
                CacheResults(added);
                BuildModel();
            }
            catch (Exception ex) { Status = "경로 탐색 오류: " + ex.Message; }
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
            var perUtil = new Dictionary<string, MeshBuilder>();
            var perUtilVisited = new Dictionary<string, MeshBuilder>();   // 방문맵 — 유틸리티별 머지 메시.
            var perUtilVisitedCount = new Dictionary<string, int>();      // 표시 셀 카운트(다운샘플 후).
            var successPaths = new List<PathCell[]>();
            double tubeDia = grid.CellMm * 0.7;
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

                // 경로 튜브(ShowPaths + 유틸 가시 일 때).
                if (drawPaths && utilVisible)
                {
                    if (!perUtil.TryGetValue(label, out var mb))
                    {
                        mb = new MeshBuilder(false, false);
                        perUtil[label] = mb;
                    }
                    var pts = row.Path.Select(c => CellToWorld(grid, c)).ToList();
                    if (pts.Count >= 2) mb.AddTube(pts, tubeDia, 8, false);
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

            if (drawPaths)
            {
                foreach (var kv in perUtil)
                {
                    var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    group.Children.Add(Geometry(kv.Value, color, 255));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(color), Label = kv.Key });
                }
            }

            // ①-X 기존 설계배관(토글) — TB_ROUTE_PATH 폴리라인을 유틸리티 색 튜브로(월드 mm 좌표 그대로).
            //   라우팅 경로보다 가늘게 그려 구분하고, 유틸 필터(체크박스)도 동일 적용한다.
            if (ShowExistingPipes && scene.ExistingPipes.Count > 0)
            {
                double exDia = grid.CellMm * 0.45;   // 기존배관 튜브 지름(라우팅 경로 0.7 보다 가늘게).
                var perUtilEx = new Dictionary<string, MeshBuilder>();
                int drawn = 0;
                foreach (var pipe in scene.ExistingPipes)
                {
                    string label = pipe.Label;
                    var uf = UtilityFilters.FirstOrDefault(u => u.Label == label);
                    if (uf != null && !uf.IsVisible) continue;   // 유틸 체크박스 필터 적용.
                    if (pipe.Points.Count < 2) continue;
                    if (!perUtilEx.TryGetValue(label, out var mb))
                    {
                        mb = new MeshBuilder(false, false);
                        perUtilEx[label] = mb;
                    }
                    var pts = pipe.Points.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList();
                    mb.AddTube(pts, exDia, 8, false);
                    drawn++;
                }
                int totalEx = 0;
                foreach (var kv in perUtilEx)
                {
                    var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    group.Children.Add(Geometry(kv.Value, color, 235));
                    totalEx++;
                }
                if (drawn > 0)
                    Legend.Add(new LegendItem
                    {
                        Swatch = new SolidColorBrush(Color.FromArgb(235, 200, 200, 200)),
                        Label = $"기존 설계배관 {drawn}"
                    });
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
                    group.Children.Add(Geometry(cmb, Colors.Red, 235));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Colors.Red), Label = $"충돌(collision) {cells.Count}" });
                }
            }

            SceneModel = group;
            UpdateSelectionHighlight();   // 라우팅 후 선택 경로의 꺾임 마커·단계 목록 갱신.
            Status = $"장애물 {scene.Obstacles.Count} · 작업 {scene.Tasks.Count} · 성공 {ok}/{scene.Tasks.Count} · 총 {total:0} mm · 충돌 {collisions}{occNote}   |   engine: {Engine.Version}";
            SceneRebuilt?.Invoke();
        }

        private static Point3D CellToWorld(GridMeta g, PathCell c) =>
            new(g.Ox + (c.I + 0.5) * g.CellMm, g.Oy + (c.J + 0.5) * g.CellMm, g.Oz + (c.K + 0.5) * g.CellMm);

        // 선택 PoC 의 시작(초록)·끝(노랑) 점을 강조 구로 그린다. 라우팅과 무관 — 선택 즉시 갱신,
        // 전체 모델을 다시 만들지 않고 별도 오버레이(SelectionModel)만 교체한다(대형 장면에서도 가볍게).
        private void UpdateSelectionHighlight()
        {
            _suppressStepNav = true;
            PathSteps.Clear();
            _suppressStepNav = false;
            var t = _selectedTask;
            if (t == null || _scene == null) { SelectionModel = null; return; }
            var g = _scene.Grid;
            double r = Math.Max(g.CellMm * 1.6, 80);
            var grp = new Model3DGroup();
            var s = new MeshBuilder(false, false);
            s.AddSphere(new Point3D(t.Sx, t.Sy, t.Sz), r);
            grp.Children.Add(Geometry(s, Color.FromRgb(80, 255, 120), 235));   // 시작 = 초록.
            var e = new MeshBuilder(false, false);
            e.AddSphere(new Point3D(t.Gx, t.Gy, t.Gz), r);
            grp.Children.Add(Geometry(e, Color.FromRgb(255, 225, 60), 235));   // 끝 = 노랑.

            // 경로가 있으면 방향 전환(꺾임) 지점을 마젠타 구로 표시 + 구간 단계 리스트 구성.
            BuildPathSteps(g, t.Path, grp);

            SelectionModel = grp;
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
    }
}

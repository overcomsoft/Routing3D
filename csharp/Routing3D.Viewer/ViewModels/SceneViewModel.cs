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
using System.Windows.Data;
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

    /// <summary>범례 항목(색 견본 + 라벨).</summary>
    public sealed class LegendItem
    {
        public Brush Swatch { get; init; } = Brushes.Gray;
        public string Label { get; init; } = string.Empty;
    }

    public sealed class SceneViewModel : ObservableObject
    {
        private Engine? _engine;
        private SceneData? _scene;
        private readonly string _priority = "longest";

        private Model3D? _sceneModel;
        private string _status = string.Empty;
        private TaskRowVM? _selectedTask;
        private PickMode _pickMode = PickMode.None;
        private bool _showObstacles = true;
        private bool _showPaths = true;
        private bool _showCollisions = true;
        private bool _showGridFrame = false;        // 복셀 전체맵(격자 BBOX 와이어).
        private bool _showOccupancyVoxels = false;  // 점유맵(복셀화된 장애물 셀).
        private bool _showVisitedMap = false;       // 방문맵(A* 확장 셀, 유틸리티 색).
        private string _searchText = string.Empty;
        private bool _suppressFilterRebuild;   // BuildTaskRows 중 IsVisible 이벤트 폭주 방지.

        // DB 접속 설정(환경변수 우선) + 선택된 프로젝트 / 격자 셀 크기.
        private readonly DbConfig _dbConfig = DbConfig.FromEnv();
        private ProjectInfo? _selectedProject;
        private double _cellMm = 100.0;
        private bool _suppressProjectAutoLoad;

        public SceneViewModel(string? initialScene = null)
        {
            OpenCommand = new RelayCommand(Open);
            DemoCommand = new RelayCommand(LoadDemo);
            RerouteCommand = new RelayCommand(RerouteMulti, () => _scene != null);
            RerouteCorridorCommand = new RelayCommand(RerouteCorridor, () => _scene != null);
            RerouteSelectedCommand = new RelayCommand(RerouteSelected, () => _selectedTask != null);
            PickStartCommand = new RelayCommand(() => SetPick(PickMode.Start), () => _selectedTask != null);
            PickEndCommand = new RelayCommand(() => SetPick(PickMode.End), () => _selectedTask != null);
            FitViewCommand = new RelayCommand(() => FitViewRequested?.Invoke());
            UtilityAllCommand = new RelayCommand(() => SetAllUtilities(true));
            UtilityClearCommand = new RelayCommand(() => SetAllUtilities(false));
            LoadProjectsCommand = new RelayCommand(LoadProjects);
            LoadDbCommand = new RelayCommand(
                () => { if (_selectedProject != null) LoadFromDb(_selectedProject.ProjectId); },
                () => _selectedProject != null);

            TasksView = CollectionViewSource.GetDefaultView(Tasks);
            TasksView.Filter = TaskFilter;

            try
            {
                if (!string.IsNullOrEmpty(initialScene) && File.Exists(initialScene))
                {
                    LoadFile(initialScene);
                }
                else
                {
                    // 사용자 요청: 실행 시 DB 에서 장애물·시작/끝점 자동 로드 후 라우팅·전체보기.
                    // 실패하면(연결 불가 등) 내장 데모로 폴백 — 앱이 비어 보이지 않게.
                    LoadProjects();
                    if (_scene == null) LoadDemo();
                }
            }
            catch (Exception ex) { Status = "엔진 초기화 오류: " + ex.Message; }
        }

        // ---- 바인딩 속성 ----
        public Model3D? SceneModel { get => _sceneModel; private set => Set(ref _sceneModel, value); }
        public string Status { get => _status; private set => Set(ref _status, value); }
        public TaskRowVM? SelectedTask { get => _selectedTask; set => Set(ref _selectedTask, value); }
        public PickMode PickMode { get => _pickMode; private set => Set(ref _pickMode, value); }

        public bool ShowObstacles { get => _showObstacles; set { if (Set(ref _showObstacles, value)) RebuildIfReady(); } }
        public bool ShowPaths { get => _showPaths; set { if (Set(ref _showPaths, value)) RebuildIfReady(); } }
        public bool ShowCollisions { get => _showCollisions; set { if (Set(ref _showCollisions, value)) RebuildIfReady(); } }
        public bool ShowGridFrame { get => _showGridFrame; set { if (Set(ref _showGridFrame, value)) RebuildIfReady(); } }
        public bool ShowOccupancyVoxels { get => _showOccupancyVoxels; set { if (Set(ref _showOccupancyVoxels, value)) RebuildIfReady(); } }
        public bool ShowVisitedMap { get => _showVisitedMap; set { if (Set(ref _showVisitedMap, value)) RebuildIfReady(); } }

        public ObservableCollection<TaskRowVM> Tasks { get; } = new();
        public ObservableCollection<LegendItem> Legend { get; } = new();
        public ObservableCollection<UtilityFilterVM> UtilityFilters { get; } = new();

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
        public RelayCommand RerouteCommand { get; }
        public RelayCommand RerouteCorridorCommand { get; }
        public RelayCommand RerouteSelectedCommand { get; }
        public RelayCommand PickStartCommand { get; }
        public RelayCommand PickEndCommand { get; }
        public RelayCommand FitViewCommand { get; }
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
                try { LoadFromDb(value.ProjectId); }
                catch (Exception ex) { Status = "DB 로드 오류: " + ex.Message; }
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
            _engine!.LoadSceneText(text);
            BuildTaskRows();
            RerouteMulti();
        }

        // ---- DB 로드 ----
        /// <summary>space_project_map 에서 프로젝트 목록을 읽어 Projects 에 채우고, 첫 항목 선택 시
        /// SelectedProject 의 set 이 자동으로 LoadFromDb 를 호출(전체 자동 로드 흐름).</summary>
        private void LoadProjects()
        {
            try
            {
                var list = ObstacleDbLoader.ListProjects(_dbConfig);
                _suppressProjectAutoLoad = true;
                Projects.Clear();
                foreach (var p in list) Projects.Add(p);
                _suppressProjectAutoLoad = false;
                if (Projects.Count == 0)
                {
                    Status = "DB 에 프로젝트가 없습니다(space_project_map 비어 있음)";
                    return;
                }
                Status = $"프로젝트 {Projects.Count}개 로드";
                // 자동 선택 → 자동 로드.
                SelectedProject = Projects[0];
            }
            catch (Exception ex)
            {
                _suppressProjectAutoLoad = false;
                Status = "DB 접속 실패: " + ex.Message;
            }
        }

        /// <summary>한 프로젝트의 장애물·PoC 페어를 DB 에서 읽어 엔진에 적재하고 자동 라우팅한다.</summary>
        private void LoadFromDb(int projectId)
        {
            var sd = ObstacleDbLoader.LoadScene(_dbConfig, projectId, _cellMm);
            _scene = sd;
            ResetEngine();
            var g = sd.Grid;
            _engine!.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
            _engine.SetParams(g.CellMm, 500, 10, 2, 6);   // 기본 비용함수 파라미터.
            foreach (var o in sd.Obstacles)
                _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            foreach (var t in sd.Tasks)
                _engine.AddTask(t.Sx, t.Sy, t.Sz, t.Gx, t.Gy, t.Gz, t.Utility, t.Group);
            BuildTaskRows();
            Status = $"DB 로드: 장애물 {sd.Obstacles.Count} · 작업 {sd.Tasks.Count} · 격자 {g.Nx}×{g.Ny}×{g.Nz} cell={g.CellMm:0}mm";
            RerouteMulti();   // 라우팅 → BuildModel → SceneRebuilt → ZoomExtents(전체보기).
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
            RerouteMulti();
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
                    Sx = t.Sx, Sy = t.Sy, Sz = t.Sz, Gx = t.Gx, Gy = t.Gy, Gz = t.Gz
                });
            }
            SelectedTask = Tasks.FirstOrDefault();
            BuildUtilityFilters(colorMap);
            OnChanged(nameof(TaskCountText));
            TasksView.Refresh();
        }

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

        private void SyncEndpoints()
        {
            if (_engine == null || _scene == null) return;
            foreach (var row in Tasks)
            {
                var t = _scene.Tasks[row.Index];
                t.Sx = row.Sx; t.Sy = row.Sy; t.Sz = row.Sz;
                t.Gx = row.Gx; t.Gy = row.Gy; t.Gz = row.Gz;
                _engine.SetTaskEndpoints(row.Index, row.Sx, row.Sy, row.Sz, row.Gx, row.Gy, row.Gz);
            }
        }

        // ---- 재라우팅 ----
        private void RerouteMulti()
        {
            if (_engine == null || _scene == null) return;
            try
            {
                SyncEndpoints();
                _engine.RouteMulti(_priority);
                RefreshResults();
                BuildModel();
            }
            catch (Exception ex) { Status = "재라우팅 오류: " + ex.Message; }
        }

        // 대형 장면용 corridor 라우팅(작업별 독립 → 데모 같은 좁은 통로에선 겹쳐 충돌이 보인다).
        private void RerouteCorridor()
        {
            if (_engine == null || _scene == null) return;
            try
            {
                SyncEndpoints();
                _engine.RouteCorridor(16, 2);
                RefreshResults();
                BuildModel();
            }
            catch (Exception ex) { Status = "corridor 라우팅 오류: " + ex.Message; }
        }

        private void RerouteSelected()
        {
            if (_engine == null || _scene == null || SelectedTask == null) return;
            try
            {
                SyncEndpoints();
                _engine.RouteTask(SelectedTask.Index);
                RefreshResults();
                BuildModel();
            }
            catch (Exception ex) { Status = "단일 재라우팅 오류: " + ex.Message; }
        }

        private void RefreshResults()
        {
            foreach (var row in Tasks)
            {
                try
                {
                    var r = _engine!.GetResult(row.Index);
                    row.Success = r.Success;
                    row.LengthMm = r.LengthMm;
                }
                catch { row.Success = false; row.LengthMm = 0; }
            }
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
            var group = new Model3DGroup();
            Legend.Clear();

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

            // ① 장애물(토글) — 하나의 메시로 머지(반투명 회색).
            if (ShowObstacles && scene.Obstacles.Count > 0)
            {
                var mb = new MeshBuilder(false, false);
                foreach (var o in scene.Obstacles)
                {
                    var center = new Point3D((o.MinX + o.MaxX) / 2, (o.MinY + o.MaxY) / 2, (o.MinZ + o.MaxZ) / 2);
                    mb.AddBox(center, o.MaxX - o.MinX, o.MaxY - o.MinY, o.MaxZ - o.MinZ);
                }
                group.Children.Add(Geometry(mb, Color.FromRgb(150, 150, 150), 60));
                Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(160, 150, 150, 150)), Label = "장애물(obstacles)" });
            }

            // ①' 점유맵(토글) — 엔진이 voxelize 한 블록 셀을 작은 큐브로(반투명 옅은 청회색).
            //    대형 장면에서는 자동 다운샘플링(최대 50000 개)으로 WPF 부하 한도 유지.
            if (ShowOccupancyVoxels && _engine != null)
            {
                int rendered = AddOccupancyVoxels(group, grid);
                if (rendered > 0)
                    Legend.Add(new LegendItem
                    {
                        Swatch = new SolidColorBrush(Color.FromArgb(170, 130, 170, 200)),
                        Label = $"점유맵 (셀 {rendered:N0})"
                    });
            }

            // ② 경로 — 유틸리티별 색 튜브 + 시작/끝 구. (충돌 계산용으로 경로는 항상 수집)
            var colorMap = UtilityColors.Assign(scene.Tasks.Select(t => t.UtilityLabel));
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

            for (int i = 0; i < scene.Tasks.Count; i++)
            {
                RouteResult r;
                try { r = _engine!.GetResult(i); }
                catch { continue; }
                if (!r.Success || r.Path.Length == 0) continue;

                ok++;
                total += r.LengthMm;
                successPaths.Add(r.Path);

                string label = scene.Tasks[i].UtilityLabel;
                var uf = UtilityFilters.FirstOrDefault(u => u.Label == label);
                bool utilVisible = uf == null || uf.IsVisible;

                // 경로 튜브(ShowPaths + 유틸 가시 일 때).
                if (ShowPaths && utilVisible)
                {
                    if (!perUtil.TryGetValue(label, out var mb))
                    {
                        mb = new MeshBuilder(false, false);
                        perUtil[label] = mb;
                    }
                    var pts = r.Path.Select(c => CellToWorld(grid, c)).ToList();
                    if (pts.Count >= 2) mb.AddTube(pts, tubeDia, 8, false);
                    mb.AddSphere(pts[0], markerR);
                    mb.AddSphere(pts[^1], markerR);
                }

                // 방문맵 — 유틸리티별 머지 메시(다운샘플링으로 셀 수 상한).
                if (ShowVisitedMap && utilVisible && r.Visited.Length > 0)
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
                        int len = r.Visited.Length;
                        int take = Math.Min(remaining, len);
                        double stride = (double)len / take;
                        for (int s = 0; s < take; s++)
                        {
                            int idx = (int)(s * stride);
                            var c = r.Visited[idx];
                            var p = CellToWorld(grid, c);
                            vmb.AddBox(p, visitedBoxSize, visitedBoxSize, visitedBoxSize);
                        }
                        perUtilVisitedCount[label] = already + take;
                    }
                }
            }

            if (ShowPaths)
            {
                foreach (var kv in perUtil)
                {
                    var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    group.Children.Add(Geometry(kv.Value, color, 255));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(color), Label = kv.Key });
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
                    group.Children.Add(Geometry(cmb, Colors.Red, 235));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Colors.Red), Label = $"충돌(collision) {cells.Count}" });
                }
            }

            SceneModel = group;
            Status = $"장애물 {scene.Obstacles.Count} · 작업 {scene.Tasks.Count} · 성공 {ok}/{scene.Tasks.Count} · 총 {total:0} mm · 충돌 {collisions}   |   engine: {Engine.Version}";
            SceneRebuilt?.Invoke();
        }

        private static Point3D CellToWorld(GridMeta g, PathCell c) =>
            new(g.Ox + (c.I + 0.5) * g.CellMm, g.Oy + (c.J + 0.5) * g.CellMm, g.Oz + (c.K + 0.5) * g.CellMm);

        // 격자 BBOX 의 12 변을 가는 실린더로 그린다(복셀 전체맵 = 작업 공간 프레임).
        private static void AddGridFrame(Model3DGroup group, GridMeta g)
        {
            double x0 = g.Ox, x1 = g.Ox + g.Nx * g.CellMm;
            double y0 = g.Oy, y1 = g.Oy + g.Ny * g.CellMm;
            double z0 = g.Oz, z1 = g.Oz + g.Nz * g.CellMm;
            var corners = new[]
            {
                new Point3D(x0,y0,z0), new Point3D(x1,y0,z0), new Point3D(x1,y1,z0), new Point3D(x0,y1,z0),
                new Point3D(x0,y0,z1), new Point3D(x1,y0,z1), new Point3D(x1,y1,z1), new Point3D(x0,y1,z1),
            };
            var edges = new (int, int)[]
            {
                (0,1),(1,2),(2,3),(3,0), (4,5),(5,6),(6,7),(7,4), (0,4),(1,5),(2,6),(3,7)
            };
            var mb = new MeshBuilder(false, false);
            double r = Math.Max(g.CellMm * 0.08, 5);   // 변 굵기 — 너무 가늘면 안 보임.
            foreach (var (a, b) in edges) mb.AddCylinder(corners[a], corners[b], r, 8);
            group.Children.Add(Geometry(mb, Color.FromRgb(122, 223, 176), 230));   // 청록 계열.
        }

        // 점유맵 — 엔진이 voxelize 한 블록 셀을 작은 큐브로(반투명). 50k 초과 시 자동 다운샘플.
        // 반환값 = 실제 그린 셀 수(범례 표기용).
        private int AddOccupancyVoxels(Model3DGroup group, GridMeta g)
        {
            const int Cap = 50_000;
            var cells = _engine!.CopyBlocked();
            if (cells.Length == 0) return 0;
            int take = Math.Min(Cap, cells.Length);
            double stride = (double)cells.Length / take;
            double s = g.CellMm * 0.9;
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++)
            {
                var c = cells[(int)(n * stride)];
                mb.AddBox(CellToWorld(g, c), s, s, s);
            }
            group.Children.Add(Geometry(mb, Color.FromRgb(130, 170, 200), 100));   // 옅은 청회색 반투명.
            return take;
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

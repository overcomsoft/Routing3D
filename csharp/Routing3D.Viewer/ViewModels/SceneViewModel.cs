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
using System.IO;
using System.Linq;
using System.Text;
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

        public SceneViewModel()
        {
            OpenCommand = new RelayCommand(Open);
            DemoCommand = new RelayCommand(LoadDemo);
            RerouteCommand = new RelayCommand(RerouteMulti, () => _scene != null);
            RerouteCorridorCommand = new RelayCommand(RerouteCorridor, () => _scene != null);
            RerouteSelectedCommand = new RelayCommand(RerouteSelected, () => _selectedTask != null);
            PickStartCommand = new RelayCommand(() => SetPick(PickMode.Start), () => _selectedTask != null);
            PickEndCommand = new RelayCommand(() => SetPick(PickMode.End), () => _selectedTask != null);

            try { LoadDemo(); }
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

        public ObservableCollection<TaskRowVM> Tasks { get; } = new();
        public ObservableCollection<LegendItem> Legend { get; } = new();

        public RelayCommand OpenCommand { get; }
        public RelayCommand DemoCommand { get; }
        public RelayCommand RerouteCommand { get; }
        public RelayCommand RerouteCorridorCommand { get; }
        public RelayCommand RerouteSelectedCommand { get; }
        public RelayCommand PickStartCommand { get; }
        public RelayCommand PickEndCommand { get; }

        /// <summary>모델을 새로 만들면 발생(코드비하인드가 ZoomExtents 호출).</summary>
        public event Action? SceneRebuilt;

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

            // ② 경로 — 유틸리티별 색 튜브 + 시작/끝 구. (충돌 계산용으로 경로는 항상 수집)
            var colorMap = UtilityColors.Assign(scene.Tasks.Select(t => t.UtilityLabel));
            var perUtil = new Dictionary<string, MeshBuilder>();
            var successPaths = new List<PathCell[]>();
            double tubeDia = grid.CellMm * 0.7;
            double markerR = grid.CellMm * 0.9;
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

                if (!ShowPaths) continue;
                string label = scene.Tasks[i].UtilityLabel;
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

            if (ShowPaths)
            {
                foreach (var kv in perUtil)
                {
                    var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    group.Children.Add(Geometry(kv.Value, color, 255));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(color), Label = kv.Key });
                }
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

// 뷰모델 — C++ 엔진(routing3d_capi) 라우팅 + HelixToolkit 3D 모델 구성
// =============================================================================
// [이 파일이 하는 일]
//   scene.txt(또는 내장 데모)를 읽어 격자/장애물/작업을 파싱하고, 같은 장면을 C++
//   엔진에 적재해 라우팅한 뒤(엔진=C++, 뷰어=C#), 결과 경로를 받아 HelixToolkit
//   Model3DGroup(장애물 반투명 박스 + 유틸리티별 경로 튜브)으로 만든다.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using Routing3D.Viewer.Interop;
using Routing3D.Viewer.Model;
using Routing3D.Viewer.ViewModels;

namespace Routing3D.Viewer.ViewModels
{
    /// <summary>범례 항목(색 견본 + 라벨).</summary>
    public sealed class LegendItem
    {
        public Brush Swatch { get; init; } = Brushes.Gray;
        public string Label { get; init; } = string.Empty;
    }

    public sealed class SceneViewModel : INotifyPropertyChanged
    {
        private Engine? _engine;
        private SceneData? _scene;
        private string _priority = "longest";

        private Model3D? _sceneModel;
        private string _status = string.Empty;

        public SceneViewModel()
        {
            OpenCommand = new RelayCommand(Open);
            DemoCommand = new RelayCommand(LoadDemo);
            RerouteCommand = new RelayCommand(Reroute, () => _scene != null);

            // 시작 시 내장 데모를 자동 표시(엔진 호출 검증). DLL 없으면 상태에 오류 표시.
            try { LoadDemo(); }
            catch (Exception ex) { Status = "엔진 초기화 오류: " + ex.Message; }
        }

        // ---- 바인딩 속성 ----
        public Model3D? SceneModel
        {
            get => _sceneModel;
            private set { _sceneModel = value; OnChanged(); }
        }

        public string Status
        {
            get => _status;
            private set { _status = value; OnChanged(); }
        }

        public ObservableCollection<LegendItem> Legend { get; } = new();

        public RelayCommand OpenCommand { get; }
        public RelayCommand DemoCommand { get; }
        public RelayCommand RerouteCommand { get; }

        /// <summary>모델을 새로 만들면 발생(코드비하인드가 ZoomExtents 호출).</summary>
        public event Action? SceneRebuilt;

        // ---- 명령 구현 ----
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
            Reroute();
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

            Reroute();
        }

        private void Reroute()
        {
            if (_engine == null || _scene == null) return;
            _engine.RouteMulti(_priority);
            BuildModel();
        }

        private void ResetEngine()
        {
            _engine?.Dispose();
            _engine = new Engine();
        }

        // ---- 3D 모델 구성 ----
        private void BuildModel()
        {
            var scene = _scene!;
            var grid = scene.Grid;
            var group = new Model3DGroup();
            Legend.Clear();

            // ① 장애물 — 하나의 메시로 머지(반투명 회색).
            if (scene.Obstacles.Count > 0)
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

            // ② 경로 — 유틸리티별 색 튜브 + 시작/끝 구.
            var colorMap = UtilityColors.Assign(scene.Tasks.Select(t => t.UtilityLabel));
            var perUtil = new Dictionary<string, MeshBuilder>();
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

            foreach (var kv in perUtil)
            {
                var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                group.Children.Add(Geometry(kv.Value, color, 255));
                Legend.Add(new LegendItem { Swatch = new SolidColorBrush(color), Label = kv.Key });
            }

            SceneModel = group;
            Status = $"장애물 {scene.Obstacles.Count} · 작업 {scene.Tasks.Count} · 성공 {ok}/{scene.Tasks.Count} · 총 {total:0} mm   |   engine: {Engine.Version}";
            SceneRebuilt?.Invoke();
        }

        private static Point3D CellToWorld(GridMeta g, PathCell c) =>
            new(g.Ox + (c.I + 0.5) * g.CellMm, g.Oy + (c.J + 0.5) * g.CellMm, g.Oz + (c.K + 0.5) * g.CellMm);

        private static GeometryModel3D Geometry(MeshBuilder mb, Color color, byte alpha)
        {
            var mat = MaterialFor(color, alpha);
            return new GeometryModel3D
            {
                Geometry = mb.ToMesh(),
                Material = mat,
                BackMaterial = mat
            };
        }

        private static Material MaterialFor(Color color, byte alpha)
        {
            var c = Color.FromArgb(alpha, color.R, color.G, color.B);
            return new DiffuseMaterial(new SolidColorBrush(c));
        }

        // ---- INotifyPropertyChanged ----
        public event PropertyChangedEventHandler? PropertyChanged;
        private void OnChanged([CallerMemberName] string? name = null) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}

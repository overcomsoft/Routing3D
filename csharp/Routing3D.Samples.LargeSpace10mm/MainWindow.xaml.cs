using System.Diagnostics;
using System.Globalization;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Npgsql;
using Routing3D.Engine;

namespace Routing3D.Samples.LargeSpace10mm;

public partial class MainWindow : Window
{
    private const double CellMm = 10.0;
    private const double ScopeMarginMm = 500.0;
    private const int MaxVoxelSamples = 1800;
    private const int MaxVisitedSamples = 2600;
    private const int MaxExistingRouteDraw = 250;

    private SceneData _scene = SceneData.Empty();
    private RoutingGrid _grid = new(CellMm, new Vec3(0, 0, 0), 1, 1, 1);
    private RouteResult? _result;
    private TimeSpan _managedElapsed;
    private List<PathCell> _collisionCells = new();
    private bool _uiReady;

    public MainWindow()
    {
        InitializeComponent();
        NativeText.Text = Routing3DEngine.NativeVersion;
        _uiReady = true;
        LegendText.Text =
            "white wire: project voxel frame\n" +
            "gray boxes: obstacle data\n" +
            "blue translucent boxes: occupancy map\n" +
            "orange: equipment/sub-equipment and equipment PoC\n" +
            "green/blue: lateral/duct and duct/lateral PoC\n" +
            "thin colored tubes: existing design data\n" +
            "cyan tube: newly routed path\n" +
            "yellow cubes: visited map sample\n" +
            "red cubes: collision map";
    }

    private async void OnLoaded(object sender, RoutedEventArgs e)
    {
        await LoadProjectsAsync();
    }

    private async void LoadProjectsButton_Click(object sender, RoutedEventArgs e) => await LoadProjectsAsync();
    private async void LoadProjectButton_Click(object sender, RoutedEventArgs e) => await LoadSelectedProjectAsync();
    private async void RouteButton_Click(object sender, RoutedEventArgs e) => await RouteFirstTaskAsync();
    private async void SampleButton_Click(object sender, RoutedEventArgs e) => await RunSamplePipelineAsync();
    private void LayerChanged(object sender, RoutedEventArgs e) => Render();

    private DbConfig Db() => new(
        HostBox.Text.Trim(),
        int.TryParse(PortBox.Text.Trim(), out var p) ? p : 5432,
        DatabaseBox.Text.Trim(),
        UserBox.Text.Trim(),
        PasswordBox.Text);

    private async Task LoadProjectsAsync()
    {
        SetBusy(true);
        try
        {
            SummaryText.Text = "Loading project list from DDW_AI_DB...";
            var db = Db();
            var projects = await Task.Run(() => ListProjects(db));
            ProjectCombo.ItemsSource = projects;
            if (projects.Count > 0)
            {
                ProjectCombo.SelectedIndex = 0;
                await LoadSelectedProjectAsync();
            }
            else
            {
                SummaryText.Text = "No project rows found in TB_SPACE_GROUP_INFO.";
            }
        }
        catch (Exception ex)
        {
            SummaryText.Text = "DB load failed: " + ex.Message;
        }
        finally { SetBusy(false); }
    }

    private async Task LoadSelectedProjectAsync()
    {
        if (ProjectCombo.SelectedItem is not ProjectInfo project) return;
        SetBusy(true);
        try
        {
            SummaryText.Text = $"Loading project {project.Display}...";
            var db = Db();
            _scene = await Task.Run(() => LoadScene(db, project));
            _grid = BuildGrid(_scene);
            _result = null;
            _collisionCells.Clear();
            UpdateGroundGrid();
            Render();
            View.ZoomExtents();
        }
        catch (Exception ex)
        {
            SummaryText.Text = "Project load failed: " + ex.Message;
        }
        finally { SetBusy(false); }
    }

    private async Task RouteFirstTaskAsync()
    {
        if (_scene.Tasks.Count == 0)
        {
            SummaryText.Text = "No SOURCE/TARGET PoC route task found in TB_ROUTE_PATH.";
            return;
        }

        SetBusy(true);
        try
        {
            var taskInfo = _scene.Tasks[0];
            await Task.Run(() =>
            {
                using var engine = new Routing3DEngine();
                engine.SetGrid(_grid);
                engine.SetParameters(new RoutingParameters
                {
                    CellMm = CellMm,
                    TurnCostMm = 500,
                    ClearanceCostMm = 0,
                    ClearanceRadiusCells = 0,
                    HeuristicWeight = 2.0
                });
                engine.SetCollectVisited(true);
                foreach (var box in _scene.OccupancyBoxes)
                    engine.AddObstacle(box.Box);
                var t = engine.AddTask(taskInfo.Start, taskInfo.End, taskInfo.Utility, taskInfo.Group);
                var sw = Stopwatch.StartNew();
                _result = engine.RouteTask(t);
                sw.Stop();
                _managedElapsed = sw.Elapsed;
                _collisionCells = _result == null
                    ? new List<PathCell>()
                    : FindPathObstacleCollisions(_result.Path, _scene.OccupancyBoxes.Select(o => o.Box).ToList());
            });
            Render();
        }
        catch (Exception ex)
        {
            SummaryText.Text = "Route failed: " + ex.Message;
        }
        finally { SetBusy(false); }
    }

    private async Task RunSamplePipelineAsync()
    {
        SetBusy(true);
        SummaryText.Text = "8단계 샘플 파이프라인 시작...";
        try
        {
            var runner = new Routing3D.Samples.LargeSpace10mm.Pipeline.PipelineRunner();
            runner.ProgressMessage += msg => Dispatcher.Invoke(() => SummaryText.Text = msg);

            var results = await runner.RunAllAsync();

            int ok   = results.Count(r => r.Status == Pipeline.StepStatus.Ok);
            int fail = results.Count(r => r.Status == Pipeline.StepStatus.Failed);
            SummaryText.Text = $"파이프라인 완료: {ok} 성공 / {fail} 실패 (총 {results.Count}단계)";

            var win = new ReportWindow(results) { Owner = this };
            win.Show();
        }
        catch (Exception ex)
        {
            SummaryText.Text = "샘플 파이프라인 실패: " + ex.Message;
        }
        finally { SetBusy(false); }
    }

    private void SetBusy(bool busy)
    {
        LoadProjectsButton.IsEnabled = !busy;
        LoadProjectButton.IsEnabled = !busy;
        RouteButton.IsEnabled = !busy;
        SampleButton.IsEnabled = !busy;
    }

    private void Render()
    {
        if (!_uiReady || SceneVisual == null || SummaryText == null || OverlayText == null) return;

var group = new Model3DGroup();
        AddDomainFrame(group);

        if (VoxelCheck.IsChecked == true) AddVoxelMap(group);
        if (ObstacleCheck.IsChecked == true) AddBoxes(group, _scene.Obstacles.Select(o => o.Box), Color.FromRgb(150, 150, 150), 88);
        if (OccupancyCheck.IsChecked == true) AddBoxes(group, _scene.OccupancyBoxes.Select(o => o.Box), Color.FromRgb(68, 136, 210), 42);
        if (EquipmentCheck.IsChecked == true) AddEquipment(group);
        if (DuctCheck.IsChecked == true) AddDuctsAndLaterals(group);
        if (ExistingCheck.IsChecked == true) AddExistingRoutes(group);
        if (VisitedCheck.IsChecked == true && _result != null) AddVisited(group, _result.Visited);
        if (PathCheck.IsChecked == true && _result != null) AddPath(group, _result.Path);
        if (CollisionCheck.IsChecked == true) AddCollision(group, _collisionCells);

        SceneVisual.Content = group;
        UpdateText();
    }

    private void UpdateText()
    {
        string route = _result == null
            ? "New route: not routed"
            : $"New route: success {_result.Success}, length {_result.LengthMm:#,0} mm, turns {_result.Turns:#,0}, visited {_result.Visited.Count:#,0}, native {_result.ElapsedMs:#,0.0} ms, managed {_managedElapsed.TotalMilliseconds:#,0.0} ms";

        SummaryText.Text =
            $"Project: {_scene.ProjectName}\n" +
            $"Grid: {_grid.Nx:#,0} x {_grid.Ny:#,0} x {_grid.Nz:#,0} @ 10mm\n" +
            $"Logical cells: {(long)_grid.Nx * _grid.Ny * _grid.Nz:#,0}\n" +
            $"Obstacles: {_scene.Obstacles.Count:#,0}\n" +
            $"Equipment/sub-equipment: {_scene.Equipment.Count:#,0}\n" +
            $"Equipment PoC: {_scene.EquipmentPocs.Count:#,0}\n" +
            $"Duct/lateral: {_scene.DuctsLaterals.Count:#,0}\n" +
            $"Duct/lateral PoC: {_scene.DuctPocs.Count:#,0}\n" +
            $"Existing design routes: {_scene.ExistingRoutes.Count:#,0}\n" +
            $"Route tasks: {_scene.Tasks.Count:#,0}\n" +
            route;

        OverlayText.Text =
            $"{_grid.Nx:#,0} x {_grid.Ny:#,0} x {_grid.Nz:#,0}\n" +
            $"{(long)_grid.Nx * _grid.Ny * _grid.Nz:#,0} logical cells\n" +
            "OpenVDB sparse occupancy";
    }

    private void UpdateGroundGrid()
    {
        double width = Math.Max(_grid.Nx * _grid.CellMm, 1000);
        double length = Math.Max(_grid.Ny * _grid.CellMm, 1000);
        GroundGrid.Width = width;
        GroundGrid.Length = length;
        GroundGrid.Center = new Point3D(_grid.Origin.X + width / 2, _grid.Origin.Y + length / 2, _grid.Origin.Z);
        GroundGrid.MinorDistance = Math.Max(1000, Math.Round(Math.Min(width, length) / 20 / 1000) * 1000);
        GroundGrid.MajorDistance = Math.Max(GroundGrid.MinorDistance * 4, 5000);
    }

    private void AddDomainFrame(Model3DGroup group)
    {
        if (_grid.Nx <= 1) return;
        var min = _grid.Origin;
        var max = new Vec3(_grid.Origin.X + _grid.Nx * _grid.CellMm, _grid.Origin.Y + _grid.Ny * _grid.CellMm, _grid.Origin.Z + _grid.Nz * _grid.CellMm);
        var mb = new MeshBuilder(false, false);
        AddBoxEdges(mb, min, max, Math.Max(_grid.CellMm * 3, 80));
        group.Children.Add(Geometry(mb, Colors.White, 140));
    }

    private void AddVoxelMap(Model3DGroup group)
    {
        if (_grid.Nx <= 1) return;
        var mb = new MeshBuilder(false, false);
        int nx = 12, ny = 12, nz = 6;
        double sx = _grid.Nx * _grid.CellMm / nx, sy = _grid.Ny * _grid.CellMm / ny, sz = _grid.Nz * _grid.CellMm / nz;
        int added = 0;
        for (int ix = 0; ix <= nx && added < MaxVoxelSamples; ix++)
        for (int iy = 0; iy <= ny && added < MaxVoxelSamples; iy++)
        for (int iz = 0; iz <= nz && added < MaxVoxelSamples; iz++)
        {
            if ((ix + iy + iz) % 3 != 0) continue;
            mb.AddBox(new Point3D(_grid.Origin.X + ix * sx, _grid.Origin.Y + iy * sy, _grid.Origin.Z + iz * sz), 90, 90, 90);
            added++;
        }
        group.Children.Add(Geometry(mb, Color.FromRgb(120, 132, 145), 70));
    }

    private void AddEquipment(Model3DGroup group)
    {
        AddBoxes(group, _scene.Equipment.Select(o => o.Box), Color.FromRgb(255, 150, 40), 150);
        var mb = new MeshBuilder(false, false);
        foreach (var p in _scene.EquipmentPocs)
            mb.AddSphere(ToPoint(p.Position), Math.Max(_grid.CellMm * 35, 260));
        if (_scene.EquipmentPocs.Count > 0) group.Children.Add(Geometry(mb, Color.FromRgb(255, 210, 90), 240));
    }

    private void AddDuctsAndLaterals(Model3DGroup group)
    {
        AddBoxes(group, _scene.DuctsLaterals.Where(d => d.Kind == BoxKind.Duct).Select(o => o.Box), Color.FromRgb(110, 175, 220), 150);
        AddBoxes(group, _scene.DuctsLaterals.Where(d => d.Kind == BoxKind.Lateral).Select(o => o.Box), Color.FromRgb(90, 210, 130), 150);
        var mb = new MeshBuilder(false, false);
        foreach (var p in _scene.DuctPocs)
            mb.AddSphere(ToPoint(p.Position), Math.Max(_grid.CellMm * 35, 260));
        if (_scene.DuctPocs.Count > 0) group.Children.Add(Geometry(mb, Color.FromRgb(80, 150, 255), 240));
    }

    private void AddExistingRoutes(Model3DGroup group)
    {
        var routes = _scene.ExistingRoutes.Take(MaxExistingRouteDraw).ToList();
        if (routes.Count == 0) return;
        var colors = new[] { Colors.DeepSkyBlue, Colors.LightGreen, Colors.Orange, Colors.Plum, Colors.Khaki, Colors.Tomato };
        int i = 0;
        foreach (var route in routes)
        {
            if (route.Points.Count < 2) continue;
            var mb = new MeshBuilder(false, false);
            mb.AddTube(route.Points.Select(ToPoint).ToList(), Math.Max(route.DiameterMm, 60), 8, false);
            group.Children.Add(Geometry(mb, colors[i++ % colors.Length], 185));
        }
    }

    private void AddBoxes(Model3DGroup group, IEnumerable<Aabb> boxes, Color color, byte alpha)
    {
        var mb = new MeshBuilder(false, false);
        int n = 0;
        foreach (var o in boxes)
        {
            var cx = (o.Min.X + o.Max.X) / 2;
            var cy = (o.Min.Y + o.Max.Y) / 2;
            var cz = (o.Min.Z + o.Max.Z) / 2;
            mb.AddBox(new Point3D(cx, cy, cz), Math.Max(o.Max.X - o.Min.X, CellMm), Math.Max(o.Max.Y - o.Min.Y, CellMm), Math.Max(o.Max.Z - o.Min.Z, CellMm));
            n++;
        }
        if (n > 0) group.Children.Add(Geometry(mb, color, alpha));
    }

    private void AddVisited(Model3DGroup group, IReadOnlyList<PathCell> visited)
    {
        if (visited.Count == 0) return;
        var mb = new MeshBuilder(false, false);
        int step = Math.Max(1, visited.Count / MaxVisitedSamples);
        for (int i = 0; i < visited.Count; i += step)
            mb.AddBox(CellToWorld(visited[i]), 95, 95, 95);
        group.Children.Add(Geometry(mb, Color.FromRgb(235, 205, 60), 105));
    }

    private void AddPath(Model3DGroup group, IReadOnlyList<PathCell> path)
    {
        if (path.Count < 2) return;
        var pts = SimplifyPath(path).Select(CellToWorld).ToList();
        var mb = new MeshBuilder(false, false);
        mb.AddTube(pts, 280, 12, false);
        mb.AddSphere(CellToWorld(path[0]), 520);
        mb.AddSphere(CellToWorld(path[^1]), 520);
        group.Children.Add(Geometry(mb, Color.FromRgb(35, 220, 235), 230));
    }

    private void AddCollision(Model3DGroup group, IReadOnlyList<PathCell> cells)
    {
        if (cells.Count == 0) return;
        var mb = new MeshBuilder(false, false);
        foreach (var c in cells.Take(400))
            mb.AddBox(CellToWorld(c), 420, 420, 420);
        group.Children.Add(Geometry(mb, Color.FromRgb(255, 60, 60), 210));
    }

    private Point3D CellToWorld(PathCell c) =>
        new(_grid.Origin.X + (c.I + 0.5) * CellMm, _grid.Origin.Y + (c.J + 0.5) * CellMm, _grid.Origin.Z + (c.K + 0.5) * CellMm);

    private static Point3D ToPoint(Vec3 p) => new(p.X, p.Y, p.Z);

    private static void AddBoxEdges(MeshBuilder mb, Vec3 min, Vec3 max, double diameter)
    {
        Point3D p000 = new(min.X, min.Y, min.Z), p100 = new(max.X, min.Y, min.Z), p010 = new(min.X, max.Y, min.Z), p110 = new(max.X, max.Y, min.Z);
        Point3D p001 = new(min.X, min.Y, max.Z), p101 = new(max.X, min.Y, max.Z), p011 = new(min.X, max.Y, max.Z), p111 = new(max.X, max.Y, max.Z);
        void Edge(Point3D a, Point3D b) => mb.AddTube(new[] { a, b }, diameter, 8, false);
        Edge(p000, p100); Edge(p100, p110); Edge(p110, p010); Edge(p010, p000);
        Edge(p001, p101); Edge(p101, p111); Edge(p111, p011); Edge(p011, p001);
        Edge(p000, p001); Edge(p100, p101); Edge(p110, p111); Edge(p010, p011);
    }

    private static GeometryModel3D Geometry(MeshBuilder mb, Color color, byte alpha)
    {
        var mat = MaterialHelper.CreateMaterial(Color.FromArgb(alpha, color.R, color.G, color.B));
        return new GeometryModel3D { Geometry = mb.ToMesh(), Material = mat, BackMaterial = mat };
    }

    private static IEnumerable<PathCell> SimplifyPath(IReadOnlyList<PathCell> path)
    {
        if (path.Count <= 2) return path;
        var result = new List<PathCell> { path[0] };
        var prev = Direction(path[0], path[1]);
        for (int i = 2; i < path.Count; i++)
        {
            var dir = Direction(path[i - 1], path[i]);
            if (dir != prev) result.Add(path[i - 1]);
            prev = dir;
        }
        result.Add(path[^1]);
        return result;
    }

    private static (int, int, int) Direction(PathCell a, PathCell b) =>
        (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));

    private static List<PathCell> FindPathObstacleCollisions(IReadOnlyList<PathCell> path, IReadOnlyList<Aabb> obstacles)
    {
        var hits = new List<PathCell>();
        foreach (var c in path)
        {
            double x = (c.I + 0.5) * CellMm;
            double y = (c.J + 0.5) * CellMm;
            double z = (c.K + 0.5) * CellMm;
            if (obstacles.Any(o => x >= o.Min.X && x <= o.Max.X && y >= o.Min.Y && y <= o.Max.Y && z >= o.Min.Z && z <= o.Max.Z))
                hits.Add(c);
        }
        return hits;
    }

    private static List<ProjectInfo> ListProjects(DbConfig cfg)
    {
        var list = new List<ProjectInfo>();
        using var conn = new NpgsqlConnection(cfg.ConnectionString);
        conn.Open();
        using var cmd = new NpgsqlCommand(
            @"SELECT ""TAG_GROUP_ID"",""TAG_GROUP_NM"",""BAY_GROUP_NM"",""PROCESS_GROUP_NM"",
                     ""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
              FROM ""TB_SPACE_GROUP_INFO""
              ORDER BY ""PROCESS_GROUP_NM"",""TAG_GROUP_NM""", conn);
        using var r = cmd.ExecuteReader();
        while (r.Read())
        {
            list.Add(new ProjectInfo(
                Str(r, 0), Str(r, 1), NullStr(r, 2), NullStr(r, 3),
                Dbl(r, 4), Dbl(r, 5), Dbl(r, 6), Dbl(r, 7), Dbl(r, 8), Dbl(r, 9)));
        }
        return list;
    }

    private static SceneData LoadScene(DbConfig cfg, ProjectInfo project)
    {
        var data = new SceneData(project.Display);
        double minx = project.MinX - ScopeMarginMm, maxx = project.MaxX + ScopeMarginMm;
        double miny = project.MinY - ScopeMarginMm, maxy = project.MaxY + ScopeMarginMm;
        double minz = project.MinZ - ScopeMarginMm, maxz = project.MaxZ + ScopeMarginMm;

        using var conn = new NpgsqlConnection(cfg.ConnectionString);
        conn.Open();
        const string IsectXY = @" ""AABB_MINX""<=@maxx AND ""AABB_MAXX"">=@minx AND ""AABB_MINY""<=@maxy AND ""AABB_MAXY"">=@miny ";
        void SetXY(NpgsqlCommand c) { c.Parameters.AddWithValue("@minx", minx); c.Parameters.AddWithValue("@maxx", maxx); c.Parameters.AddWithValue("@miny", miny); c.Parameters.AddWithValue("@maxy", maxy); }

        using (var cmd = new NpgsqlCommand(
            @"SELECT ""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ"",""INSTANCE_NAME"",""COLLISION_PASS""
              FROM ""TB_BIM_OBSTACLE"" WHERE" + IsectXY, conn))
        {
            SetXY(cmd);
            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                var box = ReadBox(r, 0);
                if (!Valid(box)) continue;
                bool pass = !r.IsDBNull(7) && Convert.ToInt64(r.GetValue(7), CultureInfo.InvariantCulture) != 0;
                var item = new BoxItem(Str(r, 6), BoxKind.Obstacle, ClipBox(box, minx, miny, minz, maxx, maxy, maxz), pass);
                data.Obstacles.Add(item);
                if (!pass) data.OccupancyBoxes.Add(item);
            }
        }

        using (var cmd = new NpgsqlCommand(
            @"SELECT ""INSTANCE_NAME"",""MAIN_SUB_TYPE"",""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
              FROM ""TB_EQUIPMENTS"" WHERE" + IsectXY, conn))
        {
            SetXY(cmd);
            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                var item = new BoxItem(Str(r, 0), BoxKind.Equipment, ReadBox(r, 2), false);
                if (!Valid(item.Box)) continue;
                data.Equipment.Add(item);
                data.OccupancyBoxes.Add(item);
            }
        }

        LoadDuctLateral(conn, "TB_LATERAL_PIPE", BoxKind.Lateral, IsectXY, SetXY, data);
        LoadDuctLateral(conn, "TB_DUCT", BoxKind.Duct, IsectXY, SetXY, data);
        LoadRoutes(conn, minx, maxx, miny, maxy, data);
        foreach (var t in data.Tasks)
        {
            data.EquipmentPocs.Add(new PocItem(t.StartName ?? "Equipment PoC", t.Start));
            data.DuctPocs.Add(new PocItem(t.EndName ?? "Duct PoC", t.End));
        }
        return data;
    }

    private static void LoadDuctLateral(NpgsqlConnection conn, string table, BoxKind kind, string isectXY, Action<NpgsqlCommand> setXY, SceneData data)
    {
        using var cmd = new NpgsqlCommand(
            $@"SELECT ""INSTANCE_NAME"",""UTILITY"",""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
                FROM ""{table}"" WHERE" + isectXY, conn);
        setXY(cmd);
        using var r = cmd.ExecuteReader();
        while (r.Read())
        {
            var item = new BoxItem(Str(r, 0), kind, ReadBox(r, 2), false);
            if (!Valid(item.Box)) continue;
            data.DuctsLaterals.Add(item);
            data.OccupancyBoxes.Add(item);
        }
    }

    private static void LoadRoutes(NpgsqlConnection conn, double minx, double maxx, double miny, double maxy, SceneData data)
    {
        using var cmd = new NpgsqlCommand(
            @"SELECT s.""ROUTE_PATH_GUID"", rp.""UTILITY_GROUP"", rp.""SOURCE_UTILITY"", rp.""SOURCE_SIZE"",
                     rp.""EQUIPMENT_NAME"", rp.""TARGET_OWNER_NAME"",
                     sd.""FROM_POSX"", sd.""FROM_POSY"", sd.""FROM_POSZ"",
                     sd.""TO_POSX"", sd.""TO_POSY"", sd.""TO_POSZ"",
                     rp.""SOURCE_POSX"", rp.""SOURCE_POSY"", rp.""SOURCE_POSZ"",
                     rp.""TARGET_POSX"", rp.""TARGET_POSY"", rp.""TARGET_POSZ""
                FROM ""TB_ROUTE_SEGMENT_DETAIL"" sd
                JOIN ""TB_ROUTE_SEGMENTS"" s ON s.""SEGMENT_GUID"" = sd.""SEGMENT_GUID""
                JOIN ""TB_ROUTE_PATH"" rp ON rp.""ROUTE_PATH_GUID"" = s.""ROUTE_PATH_GUID""
               WHERE rp.""SOURCE_POSX"" BETWEEN @minx AND @maxx
                 AND rp.""SOURCE_POSY"" BETWEEN @miny AND @maxy
               ORDER BY s.""ROUTE_PATH_GUID"", s.""ORDER"", sd.""ORDER""", conn);
        cmd.Parameters.AddWithValue("@minx", minx); cmd.Parameters.AddWithValue("@maxx", maxx);
        cmd.Parameters.AddWithValue("@miny", miny); cmd.Parameters.AddWithValue("@maxy", maxy);
        using var r = cmd.ExecuteReader();
        string? current = null;
        ExistingRoute? route = null;

        void Flush()
        {
            if (route == null || route.Points.Count < 2) return;
            data.ExistingRoutes.Add(route);
        }

        while (r.Read())
        {
            string guid = Str(r, 0);
            if (!string.Equals(current, guid, StringComparison.Ordinal))
            {
                Flush();
                current = guid;
                route = new ExistingRoute(guid, NullStr(r, 2), NullStr(r, 1), ParsePipeSizeMm(NullStr(r, 3)));
                if (!r.IsDBNull(12) && !r.IsDBNull(13) && !r.IsDBNull(14) && !r.IsDBNull(15) && !r.IsDBNull(16) && !r.IsDBNull(17))
                {
                    var start = new Vec3(Dbl(r, 12), Dbl(r, 13), Dbl(r, 14));
                    var end = new Vec3(Dbl(r, 15), Dbl(r, 16), Dbl(r, 17));
                    data.Tasks.Add(new RouteTaskInfo(start, end, NullStr(r, 2), NullStr(r, 1), NullStr(r, 4), NullStr(r, 5)));
                }
            }
            if (route == null) continue;
            if (!r.IsDBNull(6) && !r.IsDBNull(7) && !r.IsDBNull(8)) AddRoutePoint(route, new Vec3(Dbl(r, 6), Dbl(r, 7), Dbl(r, 8)));
            if (!r.IsDBNull(9) && !r.IsDBNull(10) && !r.IsDBNull(11)) AddRoutePoint(route, new Vec3(Dbl(r, 9), Dbl(r, 10), Dbl(r, 11)));
        }
        Flush();
    }

    private static void AddRoutePoint(ExistingRoute route, Vec3 p)
    {
        if (route.Points.Count == 0 || Dist2(route.Points[^1], p) > 1.0) route.Points.Add(p);
    }

    private RoutingGrid BuildGrid(SceneData data)
    {
        var boxes = data.Obstacles.Concat(data.Equipment).Concat(data.DuctsLaterals).Select(b => b.Box).ToList();
        foreach (var t in data.Tasks)
        {
            boxes.Add(new Aabb(t.Start, t.Start));
            boxes.Add(new Aabb(t.End, t.End));
        }
        if (boxes.Count == 0) return new RoutingGrid(CellMm, new Vec3(0, 0, 0), 5000, 5000, 3000);
        double minx = boxes.Min(b => b.Min.X) - ScopeMarginMm, miny = boxes.Min(b => b.Min.Y) - ScopeMarginMm, minz = boxes.Min(b => b.Min.Z) - ScopeMarginMm;
        double maxx = boxes.Max(b => b.Max.X) + ScopeMarginMm, maxy = boxes.Max(b => b.Max.Y) + ScopeMarginMm, maxz = boxes.Max(b => b.Max.Z) + ScopeMarginMm;
        int nx = Math.Max(1, (int)Math.Ceiling((maxx - minx) / CellMm));
        int ny = Math.Max(1, (int)Math.Ceiling((maxy - miny) / CellMm));
        int nz = Math.Max(1, (int)Math.Ceiling((maxz - minz) / CellMm));
        return new RoutingGrid(CellMm, new Vec3(minx, miny, minz), nx, ny, nz);
    }

    private static Aabb ReadBox(NpgsqlDataReader r, int start) =>
        new(new Vec3(Dbl(r, start), Dbl(r, start + 1), Dbl(r, start + 2)),
            new Vec3(Dbl(r, start + 3), Dbl(r, start + 4), Dbl(r, start + 5)));

    private static Aabb ClipBox(Aabb b, double minx, double miny, double minz, double maxx, double maxy, double maxz) =>
        new(new Vec3(Math.Max(b.Min.X, minx), Math.Max(b.Min.Y, miny), Math.Max(b.Min.Z, minz)),
            new Vec3(Math.Min(b.Max.X, maxx), Math.Min(b.Max.Y, maxy), Math.Min(b.Max.Z, maxz)));

    private static bool Valid(Aabb b) => b.Max.X > b.Min.X && b.Max.Y > b.Min.Y && b.Max.Z > b.Min.Z;
    private static double Dbl(NpgsqlDataReader r, int i) => r.IsDBNull(i) ? 0.0 : Convert.ToDouble(r.GetValue(i), CultureInfo.InvariantCulture);
    private static string Str(NpgsqlDataReader r, int i) => r.IsDBNull(i) ? string.Empty : Convert.ToString(r.GetValue(i), CultureInfo.InvariantCulture) ?? string.Empty;
    private static string? NullStr(NpgsqlDataReader r, int i) { var s = Str(r, i); return string.IsNullOrWhiteSpace(s) ? null : s; }
    private static double Dist2(Vec3 a, Vec3 b) { double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z; return dx * dx + dy * dy + dz * dz; }
    private static double ParsePipeSizeMm(string? size)
    {
        if (string.IsNullOrWhiteSpace(size)) return 80;
        string digits = new(size.TakeWhile(ch => char.IsDigit(ch) || ch == '.').ToArray());
        return double.TryParse(digits, NumberStyles.Float, CultureInfo.InvariantCulture, out var d) ? Math.Max(d, 30) : 80;
    }

    private sealed record DbConfig(string Host, int Port, string Database, string User, string Password)
    {
        public string ConnectionString => $"Host={Host};Port={Port};Database={Database};Username={User};Password={Password};Timeout=8;Encoding=UTF8";
    }

    private sealed record ProjectInfo(string GroupId, string GroupName, string? Bay, string? Process, double MinX, double MinY, double MinZ, double MaxX, double MaxY, double MaxZ)
    {
        public string Display => $"{GroupName} / {Bay ?? "?"} / {Process ?? "?"}";
    }

    private enum BoxKind { Obstacle, Equipment, Duct, Lateral }
    private sealed record BoxItem(string Name, BoxKind Kind, Aabb Box, bool PassThrough);
    private sealed record PocItem(string Name, Vec3 Position);
    private sealed record ExistingRoute(string Guid, string? Utility, string? Group, double DiameterMm)
    {
        public List<Vec3> Points { get; } = new();
    }
    private sealed record RouteTaskInfo(Vec3 Start, Vec3 End, string? Utility, string? Group, string? StartName, string? EndName);
    private sealed class SceneData
    {
        public string ProjectName { get; }
        public List<BoxItem> Obstacles { get; } = new();
        public List<BoxItem> Equipment { get; } = new();
        public List<BoxItem> DuctsLaterals { get; } = new();
        public List<BoxItem> OccupancyBoxes { get; } = new();
        public List<PocItem> EquipmentPocs { get; } = new();
        public List<PocItem> DuctPocs { get; } = new();
        public List<ExistingRoute> ExistingRoutes { get; } = new();
        public List<RouteTaskInfo> Tasks { get; } = new();
        public SceneData(string projectName) => ProjectName = projectName;
        public static SceneData Empty() => new("No project loaded");
    }
}

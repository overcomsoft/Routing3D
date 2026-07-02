using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Routing3D.AutoRouteViewer.Models;

namespace Routing3D.AutoRouteViewer.Services;

public sealed class SceneRenderOptions
{
    public bool ShowSpaces { get; init; } = true;
    public bool ShowObstacles { get; init; } = true;
    public bool ShowEquipment { get; init; } = true;
    public bool ShowDucts { get; init; } = true;
    public bool ShowLaterals { get; init; } = true;
    public bool ShowExistingPipes { get; init; } = true;
    public bool ShowAutoRoutes { get; init; } = true;
    public bool ShowPocMarkers { get; init; } = true;
    public bool ShowFittings { get; init; } = true;
    public bool ShowBoundsFrame { get; init; } = true;
}

public static class SceneModelBuilder
{
    private static readonly Color[] UtilityPalette =
    {
        Colors.Red, Colors.Blue, Colors.Green, Colors.Orange, Colors.Purple,
        Colors.DeepPink, Colors.Teal, Colors.Gold, Colors.SaddleBrown, Colors.Cyan,
        Colors.Magenta, Colors.LimeGreen, Colors.Navy, Colors.Crimson, Colors.DarkOrange,
        Colors.MediumSpringGreen, Colors.SlateBlue, Colors.Tomato, Colors.SeaGreen, Colors.RoyalBlue,
        Colors.Violet, Colors.Olive, Colors.IndianRed, Colors.Turquoise
    };

    public static Model3DGroup Build(SceneSnapshot scene, IReadOnlyList<RoutePath> routes, IReadOnlyList<RoutePath> existingPipes, RouteComparison? comparison, SceneRenderOptions options)
    {
        var group = new Model3DGroup();
        group.Children.Add(new AmbientLight(Color.FromRgb(95, 95, 105)));
        group.Children.Add(new DirectionalLight(Colors.White, new Vector3D(-1.5, -2.0, -3.0)));
        group.Children.Add(new DirectionalLight(Color.FromRgb(120, 145, 170), new Vector3D(2.0, 1.0, 1.5)));

        Rect3D bounds = ComputeBounds(scene, routes, existingPipes, comparison, options);
        if (options.ShowBoundsFrame && !bounds.IsEmpty)
            AddBoxFrame(group, BoundsMin(bounds), BoundsMax(bounds), Color.FromRgb(0, 210, 80), Math.Max(Longest(bounds) * 0.0012, 8), 210);

        if (options.ShowSpaces) AddSpaces(group, scene.Spaces);
        if (options.ShowObstacles) AddObstacles(group, scene.Obstacles);
        if (options.ShowEquipment) AddEquipment(group, scene.Equipment);
        if (options.ShowDucts || options.ShowLaterals) AddDuctsAndLaterals(group, options.ShowDucts ? scene.Ducts : Array.Empty<DuctBox>(), options.ShowLaterals ? scene.Laterals : Array.Empty<DuctBox>());
        if (options.ShowExistingPipes) AddExistingPipes(group, existingPipes);
        if (options.ShowExistingPipes) AddFilteredTaskPipes(group, routes, existingPipes);
        if (options.ShowFittings) AddFittings(group, scene.Fittings);
        if (options.ShowPocMarkers) AddPocMarkers(group, scene.Pocs, routes);

        if (options.ShowAutoRoutes && comparison != null && comparison.NewPath.Count >= 2)
            AddTube(group, comparison.NewPath, 95, Color.FromRgb(0, 230, 255), 245);

        return group;
    }

    private static void AddSpaces(Model3DGroup group, IReadOnlyList<SpaceBox> spaces)
    {
        var colorMap = AssignColors(spaces.Select(s => s.Name));
        foreach (SpaceBox space in spaces)
        {
            Color color = colorMap.TryGetValue(space.Name, out Color c) ? c : Colors.Gold;
            AddBoxFrame(group, space.Min, space.Max, color, Math.Max(BoxLongest(space.Min, space.Max) * 0.0018, 6), 235);
        }
    }

    private static void AddObstacles(Model3DGroup group, IReadOnlyList<ObstacleBox> obstacles)
    {
        var solid = new MeshBuilder(false, false);
        var pass = new MeshBuilder(false, false);
        int solidCount = 0;
        int passCount = 0;
        foreach (ObstacleBox box in obstacles)
        {
            if (!IsValidBox(box.Min, box.Max)) continue;
            if (box.IsRoutingSolid) { AddBox(solid, box.Min, box.Max); solidCount++; }
            else { AddBox(pass, box.Min, box.Max); passCount++; }
        }
        if (solidCount > 0) group.Children.Add(Geometry(solid, Color.FromRgb(150, 150, 150), 60));
        if (passCount > 0) group.Children.Add(Geometry(pass, Color.FromRgb(90, 200, 160), 55));
    }

    private static void AddEquipment(Model3DGroup group, IReadOnlyList<EquipmentBox> equipment)
    {
        var main = new MeshBuilder(false, false);
        var sub = new MeshBuilder(false, false);
        int mainCount = 0;
        int subCount = 0;
        foreach (EquipmentBox box in equipment)
        {
            if (!IsValidBox(box.Min, box.Max)) continue;
            if (box.IsMain) { AddBox(main, box.Min, box.Max); mainCount++; }
            else { AddBox(sub, box.Min, box.Max); subCount++; }
        }
        if (mainCount > 0) group.Children.Add(Geometry(main, Color.FromRgb(255, 140, 0), 150));
        if (subCount > 0) group.Children.Add(Geometry(sub, Color.FromRgb(255, 190, 90), 90));
    }

    private static void AddDuctsAndLaterals(Model3DGroup group, IReadOnlyList<DuctBox> ducts, IReadOnlyList<DuctBox> laterals)
    {
        var ductMesh = new MeshBuilder(false, false);
        var lateralMesh = new MeshBuilder(false, false);
        int ductCount = 0;
        int lateralCount = 0;
        foreach (DuctBox box in ducts)
        {
            AddBoxWithMinThickness(ductMesh, box.Min, box.Max, 40);
            ductCount++;
        }
        foreach (DuctBox box in laterals)
        {
            AddBoxWithMinThickness(lateralMesh, box.Min, box.Max, 40);
            lateralCount++;
        }
        if (lateralCount > 0) group.Children.Add(Geometry(lateralMesh, Color.FromRgb(90, 210, 130), 150));
        if (ductCount > 0) group.Children.Add(Geometry(ductMesh, Color.FromRgb(110, 175, 220), 130));
    }

    private static void AddExistingPipes(Model3DGroup group, IReadOnlyList<RoutePath> pipes)
    {
        if (pipes.Count == 0) return;
        Dictionary<string, Color> colorMap = AssignColors(pipes.Select(RouteLabel));
        var byLabel = new Dictionary<string, MeshBuilder>(StringComparer.Ordinal);
        double fallbackDia = 50;
        foreach (RoutePath pipe in pipes)
        {
            if (pipe.Points.Count < 2) continue;
            string label = RouteLabel(pipe);
            if (!byLabel.TryGetValue(label, out MeshBuilder? mesh))
            {
                mesh = new MeshBuilder(false, false);
                byLabel[label] = mesh;
            }
            mesh.AddTube(CleanPolyline(pipe.Points), Math.Max(pipe.Diameter > 0 ? pipe.Diameter : fallbackDia, 8), 10, false);
        }
        foreach ((string label, MeshBuilder mesh) in byLabel)
        {
            Color color = colorMap.TryGetValue(label, out Color c) ? c : Colors.Gray;
            group.Children.Add(Geometry(mesh, color, 235));
        }
    }

    private static void AddFilteredTaskPipes(Model3DGroup group, IReadOnlyList<RoutePath> routes, IReadOnlyList<RoutePath> existingPipes)
    {
        if (routes.Count == 0) return;
        HashSet<string> existingIds = existingPipes.Select(x => x.RouteId).ToHashSet(StringComparer.OrdinalIgnoreCase);
        var highlight = new MeshBuilder(false, false);
        int count = 0;
        foreach (RoutePath route in routes)
        {
            if (route.Points.Count < 2 || !existingIds.Contains(route.RouteId)) continue;
            highlight.AddTube(CleanPolyline(route.Points), Math.Max(route.Diameter, 8), 10, false);
            count++;
        }
        if (count > 0)
            group.Children.Add(Geometry(highlight, Color.FromRgb(255, 255, 255), 120));
    }

    private static void AddFittings(Model3DGroup group, IReadOnlyList<FittingMarker> fittings)
    {
        if (fittings.Count == 0) return;
        var mesh = new MeshBuilder(false, false);
        foreach (FittingMarker fitting in fittings)
            mesh.AddBox(fitting.Position, 90, 90, 90);
        group.Children.Add(Geometry(mesh, Colors.Magenta, 220));
    }

    private static void AddPocMarkers(Model3DGroup group, IReadOnlyList<PocMarker> pocs, IReadOnlyList<RoutePath> routes)
    {
        var starts = new MeshBuilder(false, false);
        var ends = new MeshBuilder(false, false);
        int startCount = 0;
        int endCount = 0;
        foreach (PocMarker poc in pocs)
        {
            if (poc.IsRouteStart) { starts.AddSphere(poc.Position, 70); startCount++; }
            if (poc.IsRouteEnd) { ends.AddSphere(poc.Position, 70); endCount++; }
        }

        if (startCount == 0 && endCount == 0)
        {
            foreach (RoutePath route in routes)
            {
                if (route.Points.Count < 2) continue;
                starts.AddSphere(route.Start, 70);
                ends.AddSphere(route.Goal, 70);
                startCount++;
                endCount++;
            }
        }

        if (startCount > 0) group.Children.Add(Geometry(starts, Color.FromRgb(255, 45, 45), 235));
        if (endCount > 0) group.Children.Add(Geometry(ends, Color.FromRgb(50, 120, 255), 235));
    }

    private static void AddBox(MeshBuilder builder, Point3D min, Point3D max)
    {
        Point3D center = new((min.X + max.X) * 0.5, (min.Y + max.Y) * 0.5, (min.Z + max.Z) * 0.5);
        builder.AddBox(center, Math.Max(1, max.X - min.X), Math.Max(1, max.Y - min.Y), Math.Max(1, max.Z - min.Z));
    }

    private static void AddBoxWithMinThickness(MeshBuilder builder, Point3D min, Point3D max, double minThickness)
    {
        Point3D center = new((min.X + max.X) * 0.5, (min.Y + max.Y) * 0.5, (min.Z + max.Z) * 0.5);
        builder.AddBox(center,
            Math.Max(minThickness, max.X - min.X),
            Math.Max(minThickness, max.Y - min.Y),
            Math.Max(minThickness, max.Z - min.Z));
    }

    private static void AddTube(Model3DGroup group, IReadOnlyList<Point3D> points, double diameter, Color color, byte alpha)
    {
        if (points.Count < 2) return;
        List<Point3D> clean = CleanPolyline(points);
        if (clean.Count < 2) return;
        var builder = new MeshBuilder(false, false);
        builder.AddTube(clean, Math.Max(8, diameter * 0.5), 12, false);
        group.Children.Add(Geometry(builder, color, alpha));
    }

    private static void AddBoxFrame(Model3DGroup group, Point3D lo, Point3D hi, Color color, double radius, byte alpha)
    {
        if (!IsValidBox(lo, hi)) return;
        var mb = new MeshBuilder(false, false);
        AddBoxFrameToMesh(mb, lo, hi, radius);
        group.Children.Add(Geometry(mb, color, alpha));
    }

    private static void AddBoxFrameToMesh(MeshBuilder mb, Point3D lo, Point3D hi, double radius)
    {
        Point3D[] p =
        {
            new(lo.X, lo.Y, lo.Z), new(hi.X, lo.Y, lo.Z), new(hi.X, hi.Y, lo.Z), new(lo.X, hi.Y, lo.Z),
            new(lo.X, lo.Y, hi.Z), new(hi.X, lo.Y, hi.Z), new(hi.X, hi.Y, hi.Z), new(lo.X, hi.Y, hi.Z)
        };
        int[,] e = { {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7} };
        for (int i = 0; i < e.GetLength(0); i++)
            mb.AddTube(new List<Point3D> { p[e[i, 0]], p[e[i, 1]] }, Math.Max(1, radius), 6, false);
    }

    private static GeometryModel3D Geometry(MeshBuilder mb, Color color, byte alpha)
    {
        Material material = MaterialFor(color, alpha);
        return new GeometryModel3D { Geometry = mb.ToMesh(), Material = material, BackMaterial = material };
    }

    private static Material MaterialFor(Color color, byte alpha)
    {
        Color c = Color.FromArgb(alpha, color.R, color.G, color.B);
        return new DiffuseMaterial(new SolidColorBrush(c));
    }

    private static Dictionary<string, Color> AssignColors(IEnumerable<string> labels)
    {
        List<string> sorted = labels.Where(x => !string.IsNullOrWhiteSpace(x)).Distinct().OrderBy(x => x, StringComparer.Ordinal).ToList();
        var map = new Dictionary<string, Color>(StringComparer.Ordinal);
        for (int i = 0; i < sorted.Count; i++)
            map[sorted[i]] = UtilityPalette[i % UtilityPalette.Length];
        return map;
    }

    private static string RouteLabel(RoutePath route) => $"[{(string.IsNullOrWhiteSpace(route.UtilityGroup) ? "?" : route.UtilityGroup)}] {(string.IsNullOrWhiteSpace(route.Utility) ? "?" : route.Utility)}";

    private static List<Point3D> CleanPolyline(IReadOnlyList<Point3D> points)
    {
        var result = new List<Point3D>();
        foreach (Point3D point in points)
        {
            if (result.Count == 0 || (result[^1] - point).LengthSquared > 1.0)
                result.Add(point);
        }
        return result;
    }

    private static Rect3D ComputeBounds(SceneSnapshot scene, IReadOnlyList<RoutePath> routes, IReadOnlyList<RoutePath> existingPipes, RouteComparison? comparison, SceneRenderOptions options)
    {
        Rect3D bounds = Rect3D.Empty;
        void Add(Point3D p)
        {
            if (double.IsNaN(p.X) || double.IsNaN(p.Y) || double.IsNaN(p.Z)) return;
            bounds.Union(p);
        }
        void AddBox(Point3D min, Point3D max) { Add(min); Add(max); }

        if (options.ShowSpaces) foreach (SpaceBox x in scene.Spaces) AddBox(x.Min, x.Max);
        if (options.ShowObstacles) foreach (ObstacleBox x in scene.Obstacles) AddBox(x.Min, x.Max);
        if (options.ShowEquipment) foreach (EquipmentBox x in scene.Equipment) AddBox(x.Min, x.Max);
        if (options.ShowDucts) foreach (DuctBox x in scene.Ducts) AddBox(x.Min, x.Max);
        if (options.ShowLaterals) foreach (DuctBox x in scene.Laterals) AddBox(x.Min, x.Max);
        if (options.ShowExistingPipes) foreach (RoutePath x in existingPipes) foreach (Point3D p in x.Points) Add(p);
        foreach (RoutePath x in routes) foreach (Point3D p in x.Points) Add(p);
        if (options.ShowPocMarkers) foreach (PocMarker x in scene.Pocs) Add(x.Position);
        if (options.ShowAutoRoutes && comparison != null) foreach (Point3D p in comparison.NewPath) Add(p);
        return bounds;
    }

    private static bool IsValidBox(Point3D min, Point3D max) => max.X > min.X && max.Y > min.Y && max.Z > min.Z;
    private static Point3D BoundsMin(Rect3D r) => new(r.X, r.Y, r.Z);
    private static Point3D BoundsMax(Rect3D r) => new(r.X + r.SizeX, r.Y + r.SizeY, r.Z + r.SizeZ);
    private static double Longest(Rect3D r) => Math.Max(r.SizeX, Math.Max(r.SizeY, r.SizeZ));
    private static double BoxLongest(Point3D min, Point3D max) => Math.Max(Math.Abs(max.X - min.X), Math.Max(Math.Abs(max.Y - min.Y), Math.Abs(max.Z - min.Z)));
}

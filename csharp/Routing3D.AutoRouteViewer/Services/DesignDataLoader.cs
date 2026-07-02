using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Routing3D.AutoRouteViewer.Models;
using Legacy = Routing3D.AutoRouteViewer.LegacyDb;

namespace Routing3D.AutoRouteViewer.Services;

public sealed class DesignDataLoader
{
    public const string DefaultProjectSql = "Internal Routing3D.AutoRouteViewer.LegacyDb.ObstacleDbLoader.ListProjects";

    public Task<List<ProjectOption>> ListProjectsAsync(string connectionString, CancellationToken cancellationToken)
    {
        return Task.Run(() =>
        {
            cancellationToken.ThrowIfCancellationRequested();
            var cfg = ParseDbConfig(connectionString);
            return Legacy.ObstacleDbLoader.ListProjects(cfg).Select(MapProject).ToList();
        }, cancellationToken);
    }

    public Task<SceneSnapshot> LoadProjectAsync(
        string connectionString,
        ProjectOption project,
        string routeSql,
        string obstacleSql,
        CancellationToken cancellationToken)
    {
        return Task.Run(() =>
        {
            cancellationToken.ThrowIfCancellationRequested();
            var cfg = ParseDbConfig(connectionString);
            Legacy.ProjectInfo legacyProject = new()
            {
                ProjectId = project.ProjectId,
                GroupId = project.GroupId,
                GroupName = project.GroupName,
                Bay = project.Bay,
                Process = project.Process,
                MinX = project.MinX,
                MinY = project.MinY,
                MinZ = project.MinZ,
                MaxX = project.MaxX,
                MaxY = project.MaxY,
                MaxZ = project.MaxZ
            };

            Legacy.SceneData legacy = Legacy.ObstacleDbLoader.LoadScene(cfg, legacyProject, cellMm: 25.0, connectedOnly: true);
            cancellationToken.ThrowIfCancellationRequested();
            return MapScene(legacy);
        }, cancellationToken);
    }

    public SceneSnapshot LoadDemo()
    {
        var scene = new SceneSnapshot();
        var route = new RoutePath { RouteId = "DEMO-001", UtilityGroup = "Exhaust", Utility = "ACID", SourceName = "Tool PoC", TargetName = "Duct PoC", Diameter = 100 };
        route.Points.Add(new Point3D(0, 0, 0));
        route.Points.Add(new Point3D(1200, 0, 0));
        route.Points.Add(new Point3D(1200, 0, 800));
        route.Points.Add(new Point3D(2200, 0, 800));
        scene.Routes.Add(route);

        var route2 = new RoutePath { RouteId = "DEMO-002", UtilityGroup = "Exhaust", Utility = "ORG", SourceName = "Tool PoC", TargetName = "Lateral PoC", Diameter = 80 };
        route2.Points.Add(new Point3D(0, 300, 0));
        route2.Points.Add(new Point3D(800, 300, 0));
        route2.Points.Add(new Point3D(800, 900, 0));
        route2.Points.Add(new Point3D(1800, 900, 500));
        scene.Routes.Add(route2);

        scene.Equipment.Add(new EquipmentBox { Name = "Demo main equipment", IsMain = true, Min = new Point3D(-250, -250, -100), Max = new Point3D(250, 550, 350) });
        scene.Ducts.Add(new DuctBox { Name = "Demo duct", Category = "DUCT", Utility = "ACID", Min = new Point3D(2000, -250, 650), Max = new Point3D(2400, 550, 950) });
        scene.Obstacles.Add(new ObstacleBox { Name = "Demo obstacle", Category = "OBSTACLE", Min = new Point3D(850, -150, 250), Max = new Point3D(1150, 250, 550) });
        scene.Pocs.Add(new PocMarker { Kind = PocOwnerKind.Equipment, Name = "Tool PoC", OwnerName = "Demo main equipment", IsRouteStart = true, Utility = "ACID", UtilityGroup = "Exhaust", RouteId = route.RouteId, Position = route.Start });
        scene.Pocs.Add(new PocMarker { Kind = PocOwnerKind.Duct, Name = "Duct PoC", OwnerName = "Demo duct", IsRouteEnd = true, Utility = "ACID", UtilityGroup = "Exhaust", RouteId = route.RouteId, Position = route.Goal });
        return scene;
    }

    private static SceneSnapshot MapScene(Legacy.SceneData legacy)
    {
        var scene = new SceneSnapshot();

        foreach (Legacy.ObstacleBox x in legacy.Obstacles)
        {
            scene.Obstacles.Add(new ObstacleBox
            {
                Name = x.Name,
                Category = string.IsNullOrWhiteSpace(x.OstType) ? "OBSTACLE" : x.OstType,
                Min = new Point3D(x.MinX, x.MinY, x.MinZ),
                Max = new Point3D(x.MaxX, x.MaxY, x.MaxZ),
                IsRoutingSolid = !x.IsPassThrough
            });
        }

        foreach (Legacy.EquipmentBox x in legacy.Equipment)
        {
            scene.Equipment.Add(new EquipmentBox
            {
                Name = x.Name,
                IsMain = x.IsMain,
                Min = new Point3D(x.MinX, x.MinY, x.MinZ),
                Max = new Point3D(x.MaxX, x.MaxY, x.MaxZ)
            });
        }

        foreach (Legacy.DuctLateral x in legacy.DuctsLaterals)
        {
            var box = new DuctBox
            {
                Name = x.Name,
                Category = x.IsLateral ? "LATERAL" : "DUCT",
                Utility = x.Utility,
                Min = new Point3D(x.MinX, x.MinY, x.MinZ),
                Max = new Point3D(x.MaxX, x.MaxY, x.MaxZ)
            };
            if (x.IsLateral) scene.Laterals.Add(box); else scene.Ducts.Add(box);
        }

        foreach (Legacy.SpaceArea x in legacy.Spaces)
        {
            scene.Spaces.Add(new SpaceBox
            {
                Name = x.Name,
                Min = new Point3D(x.MinX, x.MinY, x.MinZ),
                Max = new Point3D(x.MaxX, x.MaxY, x.MaxZ)
            });
        }

        foreach (Legacy.ExistingPipe pipe in legacy.ExistingPipes)
        {
            RoutePath existing = BuildRouteFromPipe(pipe, scene.ExistingPipes.Count + 1);
            if (existing.Points.Count >= 2)
                scene.ExistingPipes.Add(existing);
        }
        foreach (Legacy.TaskInfo task in legacy.Tasks)
        {
            Legacy.ExistingPipe? pipe = FindExistingPipe(legacy.ExistingPipes, task.RoutePathGuid, task);
            RoutePath route = BuildRouteFromTask(task, pipe, scene.Routes.Count + 1);
            if (route.Points.Count >= 2)
                scene.Routes.Add(route);
        }

        if (scene.Routes.Count == 0)
        {
            foreach (Legacy.ExistingPipe pipe in legacy.ExistingPipes)
            {
                RoutePath route = BuildRouteFromPipe(pipe, scene.Routes.Count + 1);
                if (route.Points.Count >= 2)
                    scene.Routes.Add(route);
            }
        }

        foreach (Legacy.PocMarker x in legacy.EquipmentPocs.Concat(legacy.DuctLateralPocs))
        {
            scene.Pocs.Add(new PocMarker
            {
                Kind = MapKind(x.Kind),
                Name = x.Name,
                OwnerName = x.OwnerName,
                OwnerId = x.OwnerId,
                Utility = x.Utility,
                UtilityGroup = x.Group,
                RouteId = x.RoutePathGuid,
                IsRouteStart = x.IsRouteStart,
                IsRouteEnd = x.IsRouteEnd,
                Position = new Point3D(x.X, x.Y, x.Z)
            });
        }

        foreach (Legacy.PipeFitting x in legacy.Fittings)
        {
            scene.Fittings.Add(new FittingMarker
            {
                Type = x.Type,
                Size = x.Size,
                Utility = x.Utility,
                Diameter = x.DiameterMm,
                Position = new Point3D(x.X, x.Y, x.Z)
            });
        }

        return scene;
    }

    private static ProjectOption MapProject(Legacy.ProjectInfo x) => new()
    {
        ProjectId = x.ProjectId,
        GroupId = x.GroupId,
        GroupName = x.GroupName,
        Bay = x.Bay,
        Process = x.Process,
        MinX = x.MinX,
        MinY = x.MinY,
        MinZ = x.MinZ,
        MaxX = x.MaxX,
        MaxY = x.MaxY,
        MaxZ = x.MaxZ
    };

    private static RoutePath BuildRouteFromTask(Legacy.TaskInfo task, Legacy.ExistingPipe? pipe, int sequence)
    {
        var route = new RoutePath
        {
            RouteId = string.IsNullOrWhiteSpace(task.RoutePathGuid) ? $"task-{sequence}" : task.RoutePathGuid!,
            UtilityGroup = task.Group,
            Utility = task.Utility,
            SourceName = task.PocName,
            TargetName = task.EndName,
            Diameter = task.DiameterMm > 0 ? task.DiameterMm : pipe?.DiameterMm > 0 ? pipe.DiameterMm : 50.0
        };

        AddPoint(route, new Point3D(task.Sx, task.Sy, task.Sz));
        if (pipe != null)
        {
            if (pipe.SourcePos.HasValue) AddPoint(route, ToPoint(pipe.SourcePos.Value));
            foreach (Legacy.Pt3 p in pipe.Points) AddPoint(route, ToPoint(p));
            if (pipe.TargetPos.HasValue) AddPoint(route, ToPoint(pipe.TargetPos.Value));
        }
        AddPoint(route, new Point3D(task.Gx, task.Gy, task.Gz));
        return route;
    }

    private static RoutePath BuildRouteFromPipe(Legacy.ExistingPipe pipe, int sequence)
    {
        var route = new RoutePath
        {
            RouteId = string.IsNullOrWhiteSpace(pipe.RoutePathGuid) ? $"route-{sequence}" : pipe.RoutePathGuid!,
            UtilityGroup = pipe.Group,
            Utility = pipe.Utility,
            Diameter = pipe.DiameterMm > 0 ? pipe.DiameterMm : 50.0
        };
        if (pipe.SourcePos.HasValue) AddPoint(route, ToPoint(pipe.SourcePos.Value));
        foreach (Legacy.Pt3 p in pipe.Points) AddPoint(route, ToPoint(p));
        if (pipe.TargetPos.HasValue) AddPoint(route, ToPoint(pipe.TargetPos.Value));
        return route;
    }

    private static Legacy.ExistingPipe? FindExistingPipe(IEnumerable<Legacy.ExistingPipe> pipes, string? routePathGuid, Legacy.TaskInfo task)
    {
        if (!string.IsNullOrWhiteSpace(routePathGuid))
        {
            Legacy.ExistingPipe? byGuid = pipes.FirstOrDefault(x => string.Equals(x.RoutePathGuid, routePathGuid, StringComparison.OrdinalIgnoreCase));
            if (byGuid != null) return byGuid;
        }

        Point3D start = new(task.Sx, task.Sy, task.Sz);
        Point3D goal = new(task.Gx, task.Gy, task.Gz);
        return pipes
            .Where(x => string.Equals(x.Group, task.Group, StringComparison.OrdinalIgnoreCase)
                && string.Equals(x.Utility, task.Utility, StringComparison.OrdinalIgnoreCase))
            .OrderBy(x => PipeEndpointDistance(x, start, goal))
            .FirstOrDefault();
    }

    private static double PipeEndpointDistance(Legacy.ExistingPipe pipe, Point3D start, Point3D goal)
    {
        Point3D a = pipe.SourcePos.HasValue ? ToPoint(pipe.SourcePos.Value) : pipe.Points.Count > 0 ? ToPoint(pipe.Points[0]) : new Point3D();
        Point3D b = pipe.TargetPos.HasValue ? ToPoint(pipe.TargetPos.Value) : pipe.Points.Count > 0 ? ToPoint(pipe.Points[^1]) : new Point3D();
        double forward = (a - start).LengthSquared + (b - goal).LengthSquared;
        double reverse = (b - start).LengthSquared + (a - goal).LengthSquared;
        return Math.Min(forward, reverse);
    }

    private static void AddPoint(RoutePath route, Point3D p)
    {
        if (route.Points.Count == 0 || (route.Points[^1] - p).LengthSquared > 1.0)
            route.Points.Add(p);
    }

    private static Point3D ToPoint(Legacy.Pt3 p) => new(p.X, p.Y, p.Z);

    private static PocOwnerKind MapKind(Legacy.PocOwnerKind kind) => kind switch
    {
        Legacy.PocOwnerKind.Equipment => PocOwnerKind.Equipment,
        Legacy.PocOwnerKind.Duct => PocOwnerKind.Duct,
        Legacy.PocOwnerKind.Lateral => PocOwnerKind.Lateral,
        _ => PocOwnerKind.Unknown
    };

    private static Legacy.DbConfig ParseDbConfig(string connectionString)
    {
        var cfg = new Legacy.DbConfig();
        foreach (string part in connectionString.Split(';', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            int eq = part.IndexOf('=');
            if (eq <= 0) continue;
            string key = part[..eq].Trim();
            string value = part[(eq + 1)..].Trim();
            if (key.Equals("Host", StringComparison.OrdinalIgnoreCase)) cfg.Host = NormalizeHost(value);
            else if (key.Equals("Port", StringComparison.OrdinalIgnoreCase) && int.TryParse(value, out int port)) cfg.Port = port;
            else if (key.Equals("Database", StringComparison.OrdinalIgnoreCase)) cfg.Database = value;
            else if (key.Equals("Username", StringComparison.OrdinalIgnoreCase) || key.Equals("User ID", StringComparison.OrdinalIgnoreCase) || key.Equals("User", StringComparison.OrdinalIgnoreCase)) cfg.User = value;
            else if (key.Equals("Password", StringComparison.OrdinalIgnoreCase)) cfg.Password = value;
            else if (key.Equals("Timeout", StringComparison.OrdinalIgnoreCase) && int.TryParse(value, out int timeout)) cfg.TimeoutSec = timeout;
        }
        cfg.Host = NormalizeHost(cfg.Host);
        return cfg;
    }

    private static string NormalizeHost(string? host)
    {
        string value = (host ?? string.Empty).Trim().Trim('\"', '\'');
        if (value.StartsWith("Host=", StringComparison.OrdinalIgnoreCase))
            value = value[5..].Trim();
        if (value.StartsWith("http://", StringComparison.OrdinalIgnoreCase))
            value = value[7..].Trim();
        if (value.StartsWith("https://", StringComparison.OrdinalIgnoreCase))
            value = value[8..].Trim();
        int slash = value.IndexOf('/');
        if (slash >= 0) value = value[..slash].Trim();
        int colon = value.IndexOf(':');
        if (colon > 0 && value.Count(c => c == ':') == 1)
            value = value[..colon].Trim();
        if (string.IsNullOrWhiteSpace(value)) return "127.0.0.1";
        if (value.Equals("localhost", StringComparison.OrdinalIgnoreCase)
            || value.Equals("local", StringComparison.OrdinalIgnoreCase)
            || value.Equals(".", StringComparison.OrdinalIgnoreCase)
            || value.Equals("(local)", StringComparison.OrdinalIgnoreCase))
            return "127.0.0.1";
        return value;
    }
}

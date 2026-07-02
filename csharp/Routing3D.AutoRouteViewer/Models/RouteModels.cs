using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace Routing3D.AutoRouteViewer.Models;

public sealed class ProjectOption
{
    public int ProjectId { get; set; }
    public string GroupId { get; set; } = string.Empty;
    public string GroupName { get; set; } = string.Empty;
    public string? Bay { get; set; }
    public string? Process { get; set; }
    public double MinX { get; set; }
    public double MinY { get; set; }
    public double MinZ { get; set; }
    public double MaxX { get; set; }
    public double MaxY { get; set; }
    public double MaxZ { get; set; }
    public string Display => $"{GroupName} / {Bay ?? "?"} / {Process ?? "?"}";
    public override string ToString() => Display;
}

public sealed class SceneSnapshot
{
    public List<RoutePath> Routes { get; } = new();
    public List<RoutePath> ExistingPipes { get; } = new();
    public List<ObstacleBox> Obstacles { get; } = new();
    public List<EquipmentBox> Equipment { get; } = new();
    public List<DuctBox> Ducts { get; } = new();
    public List<DuctBox> Laterals { get; } = new();
    public List<SpaceBox> Spaces { get; } = new();
    public List<PocMarker> Pocs { get; } = new();
    public List<FittingMarker> Fittings { get; } = new();

    public IEnumerable<ObstacleBox> RoutingSolids =>
        Obstacles.Where(x => x.IsRoutingSolid)
            .Concat(Equipment.Select(x => x.AsObstacle()))
            .Concat(Ducts.Select(x => x.AsObstacle()))
            .Concat(Laterals.Select(x => x.AsObstacle()));
}

public sealed class RoutePath
{
    public string RouteId { get; set; } = string.Empty;
    public string? UtilityGroup { get; set; }
    public string? Utility { get; set; }
    public string? SourceName { get; set; }
    public string? TargetName { get; set; }
    public double Diameter { get; set; } = 100.0;
    public List<Point3D> Points { get; } = new();

    public Point3D Start => Points.Count > 0 ? Points[0] : new Point3D();
    public Point3D Goal => Points.Count > 0 ? Points[^1] : new Point3D();
    public int SegmentCount => Points.Count > 1 ? Points.Count - 1 : 0;
}

public sealed class ObstacleBox
{
    public string Name { get; set; } = string.Empty;
    public string Category { get; set; } = "OBSTACLE";
    public bool IsRoutingSolid { get; set; } = true;
    public Point3D Min { get; set; }
    public Point3D Max { get; set; }

    public Point3D Center => new((Min.X + Max.X) * 0.5, (Min.Y + Max.Y) * 0.5, (Min.Z + Max.Z) * 0.5);
    public Vector3D Size => new(Max.X - Min.X, Max.Y - Min.Y, Max.Z - Min.Z);
}

public sealed class EquipmentBox
{
    public string Name { get; set; } = string.Empty;
    public bool IsMain { get; set; }
    public Point3D Min { get; set; }
    public Point3D Max { get; set; }
    public ObstacleBox AsObstacle() => new() { Name = Name, Category = IsMain ? "MAIN_EQUIPMENT" : "EQUIPMENT", Min = Min, Max = Max, IsRoutingSolid = true };
}

public sealed class DuctBox
{
    public string Name { get; set; } = string.Empty;
    public string Category { get; set; } = "DUCT";
    public string? Utility { get; set; }
    public Point3D Min { get; set; }
    public Point3D Max { get; set; }
    public ObstacleBox AsObstacle() => new() { Name = Name, Category = Category, Min = Min, Max = Max, IsRoutingSolid = true };
}

public sealed class SpaceBox
{
    public string Name { get; set; } = string.Empty;
    public Point3D Min { get; set; }
    public Point3D Max { get; set; }
}

public enum PocOwnerKind
{
    Unknown,
    Equipment,
    Duct,
    Lateral
}

public sealed class PocMarker
{
    public PocOwnerKind Kind { get; set; }
    public string Name { get; set; } = string.Empty;
    public string OwnerName { get; set; } = string.Empty;
    public string? OwnerId { get; set; }
    public string? Utility { get; set; }
    public string? UtilityGroup { get; set; }
    public string? RouteId { get; set; }
    public bool IsRouteStart { get; set; }
    public bool IsRouteEnd { get; set; }
    public Point3D Position { get; set; }
}

public sealed class FittingMarker
{
    public string Type { get; set; } = string.Empty;
    public string? Size { get; set; }
    public string? Utility { get; set; }
    public double Diameter { get; set; }
    public Point3D Position { get; set; }
}

public sealed class RouteComparison
{
    public string RouteId { get; set; } = string.Empty;
    public string? UtilityGroup { get; set; }
    public string? Utility { get; set; }
    public string? SourceName { get; set; }
    public string? TargetName { get; set; }
    public double ExistingLength { get; set; }
    public double NewLength { get; set; }
    public int ExistingBends { get; set; }
    public int NewBends { get; set; }
    public int ExploredNodes { get; set; }
    public double ElapsedMs { get; set; }
    public string ResultCode { get; set; } = string.Empty;
    public string Message { get; set; } = string.Empty;
    public string FailureHint { get; set; } = string.Empty;
    public string StartDiagnostic { get; set; } = string.Empty;
    public string GoalDiagnostic { get; set; } = string.Empty;
    public Point3D RequestedStart { get; set; }
    public Point3D RequestedGoal { get; set; }
    public Point3D EngineStart { get; set; }
    public Point3D EngineGoal { get; set; }
    public double StartOffsetMm { get; set; }
    public double GoalOffsetMm { get; set; }
    public List<Point3D> NewPath { get; } = new();
    public List<RouteStepRow> StepRows { get; } = new();

    public bool Success => string.Equals(ResultCode, "SUCCESS", System.StringComparison.OrdinalIgnoreCase) && NewPath.Count >= 2;
    public bool Attempted => !string.IsNullOrWhiteSpace(ResultCode);
    public string StatusText => Success ? "성공" : Attempted ? "실패" : "미선택";
    public string FailReasonText => Success ? string.Empty : ResultCode switch
    {
        "FAIL_TO_START_POINT" => "출발막힘",
        "FAIL_TO_END_POINT" => "종단막힘",
        "FAIL_TO_PATHFIND" => "경로없음",
        "CANCELLED" => "취소/시간초과",
        "FAIL_TO_INITIALIZE" => "초기화실패",
        _ => string.IsNullOrWhiteSpace(ResultCode) ? string.Empty : ResultCode
    };
    public string LengthText => Success ? $"{NewLength:N0}" : string.Empty;
    public int TurnCount => Success ? NewBends : 0;
    public string ElapsedText => ElapsedMs <= 0 ? string.Empty : ElapsedMs < 1000 ? $"{ElapsedMs:0}ms" : $"{ElapsedMs / 1000.0:0.0}s";
    public string ExpandedText => ExploredNodes > 0 ? $"{ExploredNodes:N0}" : string.Empty;
}

public sealed class RouteStepRow
{
    public string Seq { get; set; } = string.Empty;
    public string Kind { get; set; } = string.Empty;
    public string Direction { get; set; } = string.Empty;
    public string Length { get; set; } = string.Empty;
    public string Region { get; set; } = string.Empty;
    public string Reason { get; set; } = string.Empty;
    public Point3D A { get; set; }
    public Point3D B { get; set; }
}

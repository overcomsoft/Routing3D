using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using AutoRouteModule;
using AutoRouteModule.API;
using AutoRouteModule.Core;
using Routing3D.AutoRouteViewer.Models;

namespace Routing3D.AutoRouteViewer.Services;

public sealed class AutoRouteModuleRunner
{
    public async Task<RouteComparison> RouteAsync(
        RoutePath existing,
        IEnumerable<ObstacleBox> obstacles,
        float voxelSize,
        int maxSearchNodes,
        TimeSpan timeout,
        bool useSweepGeometry,
        CancellationToken cancellationToken)
    {
        if (existing.Points.Count < 2)
            throw new InvalidOperationException("Route must contain at least two points.");

        float safeVoxel = Math.Max(1f, voxelSize);
        float safeDiameter = Math.Max(1f, (float)existing.Diameter);
        List<ObstacleBox> obstacleList = obstacles.ToList();
        List<OBB> obstacleObbs = obstacleList.Select(ToObb).ToList();

        EndpointProbe startProbe = ProbeEndpoint(existing.Start, obstacleList, safeDiameter, safeVoxel);
        EndpointProbe goalProbe = ProbeEndpoint(existing.Goal, obstacleList, safeDiameter, safeVoxel);
        EndpointCandidate engineStart = ResolveEndpoint(existing.Points, fromStart: true, obstacleList, safeDiameter, safeVoxel);
        EndpointCandidate engineGoal = ResolveEndpoint(existing.Points, fromStart: false, obstacleList, safeDiameter, safeVoxel);

        await AutoRouteAPI.InitStaticObstaclesAsync(obstacleObbs).ConfigureAwait(false);

        var request = new RouteRequest
        {
            Start = ToVector3(engineStart.Point),
            Goal = ToVector3(engineGoal.Point),
            VoxelSize = safeVoxel,
            Diameter = safeDiameter,
            OutOfBoundsPolicy = OutOfBoundsPolicy.Free,
            Timeout = timeout,
            Options = new GridAStar3D.Options(maxSearchNodes: Math.Max(0, maxSearchNodes))
        };

        Stopwatch sw = Stopwatch.StartNew();
        PathResult result = await AutoRouteAPI.FindPathAsync(request, cancellationToken).ConfigureAwait(false);
        sw.Stop();

        var comparison = new RouteComparison
        {
            RouteId = existing.RouteId,
            UtilityGroup = existing.UtilityGroup,
            Utility = existing.Utility,
            SourceName = existing.SourceName,
            TargetName = existing.TargetName,
            ExistingLength = Length(existing.Points),
            ExistingBends = BendCount(existing.Points),
            ExploredNodes = result.ExploredNodeCount,
            ElapsedMs = sw.Elapsed.TotalMilliseconds,
            ResultCode = result.ResultCode.ToString(),
            RequestedStart = existing.Start,
            RequestedGoal = existing.Goal,
            EngineStart = engineStart.Point,
            EngineGoal = engineGoal.Point,
            StartOffsetMm = (engineStart.Point - existing.Start).Length,
            GoalOffsetMm = (engineGoal.Point - existing.Goal).Length,
            StartDiagnostic = BuildEndpointSummary("START", existing.Start, engineStart, startProbe),
            GoalDiagnostic = BuildEndpointSummary("GOAL", existing.Goal, engineGoal, goalProbe),
            FailureHint = BuildFailureHint(result, startProbe, goalProbe, engineStart, engineGoal)
        };

        if (result.WorldPath != null)
        {
            foreach (Vector3 p in result.WorldPath)
                AddDistinct(comparison.NewPath, ToPoint3D(p));
        }

        comparison.NewLength = comparison.NewPath.Count >= 2 ? Length(comparison.NewPath) : 0.0;
        comparison.NewBends = comparison.NewPath.Count >= 3 ? BendCount(comparison.NewPath) : 0;
        BuildStepRows(comparison, existing);
        comparison.Message = BuildReport(existing, obstacleList, request, result, comparison, sw.Elapsed, useSweepGeometry);
        return comparison;
    }

    private static EndpointCandidate ResolveEndpoint(IReadOnlyList<Point3D> points, bool fromStart, IReadOnlyList<ObstacleBox> obstacles, float diameter, float voxelSize)
    {
        Point3D requested = fromStart ? points[0] : points[^1];
        if (!ProbeEndpoint(requested, obstacles, diameter, voxelSize).IsBlocked)
            return new EndpointCandidate(requested, false, 0.0, "원본 PoC/끝점이 자유 공간입니다.");

        if (fromStart)
        {
            EndpointCandidate? verticalDrop = FindVerticalDropFromEquipment(requested, obstacles, diameter, voxelSize);
            if (verticalDrop != null)
                return verticalDrop;
        }

        double step = Math.Max(10.0, voxelSize * 0.5);
        double maxScan = Math.Max(diameter * 8.0, voxelSize * 20.0);
        int i = fromStart ? 0 : points.Count - 1;
        int end = fromStart ? points.Count - 1 : 0;
        int direction = fromStart ? 1 : -1;
        double accumulated = 0.0;
        Point3D previous = requested;

        while (i != end && accumulated <= maxScan)
        {
            Point3D next = points[i + direction];
            Vector3D segment = next - previous;
            double length = segment.Length;
            if (length > 0.0001)
            {
                int samples = Math.Max(1, (int)Math.Ceiling(length / step));
                for (int s = 1; s <= samples && accumulated <= maxScan; s++)
                {
                    double t = Math.Min(1.0, s * step / length);
                    Point3D candidate = Interpolate(previous, next, t);
                    double distance = (candidate - requested).Length;
                    accumulated = Math.Max(accumulated, distance);
                    if (!ProbeEndpoint(candidate, obstacles, diameter, voxelSize).IsBlocked)
                        return new EndpointCandidate(candidate, true, distance, $"기존 배관 방향으로 {distance:N0} mm 이동한 자유점부터 시작합니다.");
                }
            }
            previous = next;
            i += direction;
        }

        Point3D radial = FindNearbyFreePoint(requested, obstacles, diameter, voxelSize, maxScan);
        if ((radial - requested).Length > 0.001)
            return new EndpointCandidate(radial, true, (radial - requested).Length, $"주변 자유점 탐색으로 {(radial - requested).Length:N0} mm 이동했습니다.");

        return new EndpointCandidate(requested, false, 0.0, "주변에서 자유 시작점을 찾지 못해 원본 점을 사용했습니다.");
    }

    private static EndpointCandidate? FindVerticalDropFromEquipment(Point3D requested, IReadOnlyList<ObstacleBox> obstacles, float diameter, float voxelSize)
    {
        List<ObstacleBox> containingEquipment = obstacles
            .Where(x => IsEquipmentCategory(x.Category) && ContainsPoint(x, requested))
            .OrderBy(x => x.Min.Z)
            .ToList();
        if (containingEquipment.Count == 0)
            return null;

        double half = Math.Max(diameter, voxelSize) * 0.5;
        double step = Math.Max(10.0, voxelSize * 0.5);
        double targetZ = containingEquipment.Min(x => x.Min.Z) - half - voxelSize;
        double maxExtraDrop = Math.Max(diameter * 16.0, voxelSize * 160.0);
        double minZ = Math.Min(targetZ, requested.Z - maxExtraDrop);

        for (double z = requested.Z - step; z >= minZ; z -= step)
        {
            Point3D candidate = new(requested.X, requested.Y, z);
            if (!ProbeEndpoint(candidate, obstacles, diameter, voxelSize).IsBlocked)
            {
                double distance = (candidate - requested).Length;
                string equipmentNames = string.Join(", ", containingEquipment.Take(3).Select(x => x.Name));
                return new EndpointCandidate(candidate, true, distance, $"시작 PoC가 장비 바운더리 내부({equipmentNames})에 있어 -Z 수직하강 {distance:N0} mm 후 장비 밖 자유점부터 시작합니다.");
            }
        }

        return null;
    }

    private static Point3D FindNearbyFreePoint(Point3D origin, IReadOnlyList<ObstacleBox> obstacles, float diameter, float voxelSize, double maxRadius)
    {
        double step = Math.Max(voxelSize, diameter * 0.5);
        Vector3D[] dirs =
        {
            new(1, 0, 0), new(-1, 0, 0), new(0, 1, 0), new(0, -1, 0), new(0, 0, -1), new(0, 0, 1),
            new(1, 1, 0), new(1, -1, 0), new(-1, 1, 0), new(-1, -1, 0),
            new(1, 0, -1), new(-1, 0, -1), new(0, 1, -1), new(0, -1, -1)
        };

        for (double radius = step; radius <= maxRadius; radius += step)
        {
            foreach (Vector3D dir in dirs)
            {
                Vector3D d = dir;
                d.Normalize();
                Point3D candidate = origin + d * radius;
                if (!ProbeEndpoint(candidate, obstacles, diameter, voxelSize).IsBlocked)
                    return candidate;
            }
        }

        return origin;
    }

    private static EndpointProbe ProbeEndpoint(Point3D point, IReadOnlyList<ObstacleBox> obstacles, float diameter, float voxelSize)
    {
        double half = Math.Max(voxelSize, diameter) * 0.5;
        Point3D min = new(point.X - half, point.Y - half, point.Z - half);
        Point3D max = new(point.X + half, point.Y + half, point.Z + half);
        var hits = obstacles
            .Select(x => new EndpointHit(x.Name, x.Category, Intersects(min, max, x.Min, x.Max), ClearanceToBox(point, x.Min, x.Max)))
            .Where(x => x.Intersects || x.ClearanceMm <= Math.Max(diameter, voxelSize) * 2.0)
            .OrderByDescending(x => x.Intersects)
            .ThenBy(x => x.ClearanceMm)
            .Take(5)
            .ToList();
        return new EndpointProbe(hits.Any(x => x.Intersects), hits);
    }

    private static string BuildReport(RoutePath existing, IReadOnlyList<ObstacleBox> obstacles, RouteRequest request, PathResult result, RouteComparison comparison, TimeSpan elapsed, bool useSweepGeometry)
    {
        var sb = new StringBuilder();
        sb.AppendLine($"# 자동설계 경로 분석 - {comparison.RouteId}");
        sb.AppendLine();
        sb.AppendLine("## ① 개요");
        sb.AppendLine($"- 유틸리티 / 그룹 : {comparison.Utility ?? "-"} / {comparison.UtilityGroup ?? "-"}");
        sb.AppendLine($"- 시작(PoC) → 종단 : {comparison.SourceName ?? "(이름없음)"} → {comparison.TargetName ?? "(이름없음)"}");
        sb.AppendLine($"- 결과 : {comparison.StatusText}{(string.IsNullOrWhiteSpace(comparison.FailReasonText) ? string.Empty : " / " + comparison.FailReasonText)}");
        sb.AppendLine($"- 관경 : {request.Diameter:N0} mm, 격자 : {request.VoxelSize:N0} mm, Sweep : {(useSweepGeometry ? "ON" : "OFF")}");
        sb.AppendLine($"- 장애물/설비/덕트 검색 솔리드 : {obstacles.Count:N0} 개");
        sb.AppendLine($"- 기존 경로 : 길이 {comparison.ExistingLength:N0} mm, 꺾임 {comparison.ExistingBends:N0}");
        if (comparison.Success)
            sb.AppendLine($"- 자동 경로 : 길이 {comparison.NewLength:N0} mm, 꺾임 {comparison.NewBends:N0}");
        sb.AppendLine($"- 소요 시간 : {comparison.ElapsedText}, 탐색 확장 : {comparison.ExpandedText}");
        if (!string.IsNullOrWhiteSpace(result.Message)) sb.AppendLine($"- 엔진 메시지 : {result.Message}");
        sb.AppendLine();

        sb.AppendLine("## ② 적용된 보정");
        sb.AppendLine(comparison.StartDiagnostic);
        sb.AppendLine(comparison.GoalDiagnostic);
        sb.AppendLine();

        sb.AppendLine("## ③ 단계별 경로 (시작 → 꺾임 → 종단)");
        if (comparison.StepRows.Count == 0)
        {
            sb.AppendLine("- 경로가 없어 단계별 분석을 표시할 수 없습니다.");
        }
        else
        {
            foreach (RouteStepRow row in comparison.StepRows)
            {
                if (row.Kind == "구간") sb.AppendLine($"- {row.Seq}. {row.Direction} {row.Length} [{row.Region}] {row.Reason}".TrimEnd());
                else sb.AppendLine($"- {row.Kind}: {row.Region} {row.Reason}".TrimEnd());
            }
        }
        sb.AppendLine();

        if (!comparison.Success)
        {
            sb.AppendLine("## ④ 실패 원인 진단");
            sb.AppendLine(string.IsNullOrWhiteSpace(comparison.FailureHint) ? "- 추가 진단 정보가 없습니다." : comparison.FailureHint);
            sb.AppendLine();
        }

        return sb.ToString().TrimEnd();
    }

    private static string BuildEndpointSummary(string label, Point3D requested, EndpointCandidate engine, EndpointProbe probe)
    {
        var sb = new StringBuilder();
        string title = label == "START" ? "시작점" : "종단점";
        sb.AppendLine($"- {title}: 요청=({requested.X:N0}, {requested.Y:N0}, {requested.Z:N0}) → 엔진=({engine.Point.X:N0}, {engine.Point.Y:N0}, {engine.Point.Z:N0}), 보정={engine.OffsetMm:N0} mm");
        sb.AppendLine($"  - 처리: {engine.Reason}");
        sb.AppendLine($"  - 원본 충돌 상태: {(probe.IsBlocked ? "BLOCKED" : "FREE")}");
        if (probe.Hits.Count == 0)
        {
            sb.AppendLine("  - 주변/겹침 객체: 없음");
        }
        else
        {
            sb.AppendLine("  - 주변/겹침 객체:");
            foreach (EndpointHit hit in probe.Hits)
                sb.AppendLine($"    · {(hit.Intersects ? "HIT" : "NEAR")} {hit.Category} {hit.Name} clearance={hit.ClearanceMm:N0} mm");
        }
        return sb.ToString().TrimEnd();
    }

    private static string BuildFailureHint(PathResult result, EndpointProbe startProbe, EndpointProbe goalProbe, EndpointCandidate engineStart, EndpointCandidate engineGoal)
    {
        if (result.ResultCode == RESULT_CODES.SUCCESS)
            return "- SUCCESS: 경로 탐색에 성공했습니다.";

        var sb = new StringBuilder();
        if (result.ResultCode == RESULT_CODES.FAIL_TO_START_POINT)
        {
            sb.AppendLine("- 출발 셀이 막혀 있습니다. 엔진 시작점에서 관경 AABB가 장애물/설비/덕트/레터럴 검색 솔리드와 겹칩니다.");
            if (startProbe.IsBlocked) sb.AppendLine("- 원본 시작 PoC는 장비 또는 다른 충돌 형상 내부/접촉 상태입니다. 장비 내부 PoC는 정상일 수 있으므로 수직하강 보정값을 확인하세요.");
            if (!engineStart.Adjusted) sb.AppendLine("- 수직하강/기존경로/주변 탐색 모두 자유 시작점을 찾지 못했습니다.");
        }
        else if (result.ResultCode == RESULT_CODES.FAIL_TO_END_POINT)
        {
            sb.AppendLine("- 종단 셀이 막혀 있습니다. 목적지 관경 AABB가 충돌 형상과 겹칩니다.");
            if (goalProbe.IsBlocked) sb.AppendLine("- 원본 종단 PoC가 덕트/레터럴/장애물 내부 또는 접촉 상태입니다.");
            if (!engineGoal.Adjusted) sb.AppendLine("- 종단 주변 자유점을 찾지 못했습니다.");
        }
        else if (result.ResultCode == RESULT_CODES.FAIL_TO_PATHFIND)
        {
            sb.AppendLine("- 시작/종단 초기 검사는 통과했지만 현재 격자/장애물 구조에서 연결 경로를 찾지 못했습니다.");
            sb.AppendLine("- 다음 확인: voxel 크기 상향, MaxNodes 증가, 통과 가능 객체가 검색 솔리드에 포함됐는지 확인, 기존 Viewer와 동일한 탐색 공간 bounds 적용 여부 확인.");
        }
        else if (result.ResultCode == RESULT_CODES.CANCELLED)
        {
            sb.AppendLine("- 사용자가 취소했거나 timeout으로 탐색이 중단됐습니다.");
        }
        else
        {
            sb.AppendLine("- 엔진 요청 검증 또는 초기화 단계에서 실패했습니다. 요청 좌표/관경/옵션 값을 확인하세요.");
        }
        return sb.ToString().TrimEnd();
    }

    private static void BuildStepRows(RouteComparison comparison, RoutePath existing)
    {
        comparison.StepRows.Clear();
        IReadOnlyList<Point3D> pts = comparison.NewPath;
        if (pts.Count < 2) return;

        Point3D first = pts[0];
        comparison.StepRows.Add(new RouteStepRow
        {
            Seq = "시작",
            Kind = "시작",
            Region = string.IsNullOrWhiteSpace(comparison.SourceName) ? "시작 PoC" : comparison.SourceName!,
            Reason = $"({first.X:N0}, {first.Y:N0}, {first.Z:N0})",
            A = first,
            B = first
        });

        int runStart = 0;
        Vector3D current = DominantDirection(pts[1] - pts[0]);
        int seq = 1;
        for (int i = 1; i < pts.Count; i++)
        {
            Vector3D nextDir = i + 1 < pts.Count ? DominantDirection(pts[i + 1] - pts[i]) : current;
            bool closeRun = i + 1 >= pts.Count || Math.Abs(Vector3D.DotProduct(current, nextDir) - 1.0) > 0.001;
            if (!closeRun) continue;

            Point3D a = pts[runStart];
            Point3D b = pts[i];
            double length = (b - a).Length;
            string reason = i + 1 < pts.Count ? ClassifyBendReason(current, nextDir, b) : string.Empty;
            comparison.StepRows.Add(new RouteStepRow
            {
                Seq = seq.ToString(),
                Kind = "구간",
                Direction = AxisLabel(b - a),
                Length = $"{length:N0} mm",
                Region = "엔진 탐색",
                Reason = reason,
                A = a,
                B = b
            });
            seq++;
            runStart = i;
            current = nextDir;
        }

        Point3D last = pts[^1];
        comparison.StepRows.Add(new RouteStepRow
        {
            Seq = "종단",
            Kind = "종단",
            Region = string.IsNullOrWhiteSpace(comparison.TargetName) ? "종단 PoC" : comparison.TargetName!,
            Reason = $"({last.X:N0}, {last.Y:N0}, {last.Z:N0})",
            A = last,
            B = last
        });
    }

    private static string ClassifyBendReason(Vector3D dIn, Vector3D dOut, Point3D bend)
    {
        bool vertical = Math.Abs(dIn.Z) > 0.7 || Math.Abs(dOut.Z) > 0.7;
        if (vertical) return "레이어/높이 전환";
        return $"장애물 회피 또는 정렬 전환 @({bend.X:N0}, {bend.Y:N0}, {bend.Z:N0})";
    }

    private static Vector3D DominantDirection(Vector3D d)
    {
        if (d.LengthSquared < 1e-9) return new Vector3D(1, 0, 0);
        double ax = Math.Abs(d.X), ay = Math.Abs(d.Y), az = Math.Abs(d.Z);
        if (az >= ax && az >= ay) return new Vector3D(0, 0, Math.Sign(d.Z == 0 ? 1 : d.Z));
        if (ax >= ay) return new Vector3D(Math.Sign(d.X == 0 ? 1 : d.X), 0, 0);
        return new Vector3D(0, Math.Sign(d.Y == 0 ? 1 : d.Y), 0);
    }

    private static string AxisLabel(Vector3D d)
    {
        Vector3D axis = DominantDirection(d);
        if (Math.Abs(axis.Z) > 0.5) return axis.Z >= 0 ? "수직 상승(Z+)" : "수직 하강(Z-)";
        if (Math.Abs(axis.X) > 0.5) return axis.X >= 0 ? "수평 이동(X+)" : "수평 이동(X-)";
        return axis.Y >= 0 ? "수평 이동(Y+)" : "수평 이동(Y-)";
    }

    private static bool IsEquipmentCategory(string? category) =>
        string.Equals(category, "EQUIPMENT", StringComparison.OrdinalIgnoreCase)
        || string.Equals(category, "MAIN_EQUIPMENT", StringComparison.OrdinalIgnoreCase);

    private static bool ContainsPoint(ObstacleBox box, Point3D p) =>
        p.X >= box.Min.X && p.X <= box.Max.X
        && p.Y >= box.Min.Y && p.Y <= box.Max.Y
        && p.Z >= box.Min.Z && p.Z <= box.Max.Z;

    private static bool Intersects(Point3D aMin, Point3D aMax, Point3D bMin, Point3D bMax) =>
        aMin.X <= bMax.X && aMax.X >= bMin.X
        && aMin.Y <= bMax.Y && aMax.Y >= bMin.Y
        && aMin.Z <= bMax.Z && aMax.Z >= bMin.Z;

    private static double ClearanceToBox(Point3D p, Point3D min, Point3D max)
    {
        double dx = p.X < min.X ? min.X - p.X : p.X > max.X ? p.X - max.X : 0.0;
        double dy = p.Y < min.Y ? min.Y - p.Y : p.Y > max.Y ? p.Y - max.Y : 0.0;
        double dz = p.Z < min.Z ? min.Z - p.Z : p.Z > max.Z ? p.Z - max.Z : 0.0;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static OBB ToObb(ObstacleBox box)
    {
        Point3D c = box.Center;
        Vector3D s = box.Size;
        return new OBB
        {
            Center = new Vector3((float)c.X, (float)c.Y, (float)c.Z),
            Extents = new Vector3((float)Math.Abs(s.X * 0.5), (float)Math.Abs(s.Y * 0.5), (float)Math.Abs(s.Z * 0.5)),
            Axes = new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ }
        };
    }

    public static double Length(IReadOnlyList<Point3D> points)
    {
        double sum = 0;
        for (int i = 1; i < points.Count; i++)
            sum += (points[i] - points[i - 1]).Length;
        return sum;
    }

    public static int BendCount(IReadOnlyList<Point3D> points)
    {
        int count = 0;
        Vector3D? prev = null;
        for (int i = 1; i < points.Count; i++)
        {
            Vector3D d = DominantDirection(points[i] - points[i - 1]);
            if (prev.HasValue && Math.Abs(Vector3D.DotProduct(prev.Value, d) - 1.0) > 0.001)
                count++;
            prev = d;
        }
        return count;
    }

    private static void AddDistinct(List<Point3D> points, Point3D p)
    {
        if (points.Count == 0 || (points[^1] - p).LengthSquared > 1.0)
            points.Add(p);
    }

    private static Point3D Interpolate(Point3D a, Point3D b, double t) => new(
        a.X + (b.X - a.X) * t,
        a.Y + (b.Y - a.Y) * t,
        a.Z + (b.Z - a.Z) * t);

    private static Vector3 ToVector3(Point3D p) => new((float)p.X, (float)p.Y, (float)p.Z);
    private static Point3D ToPoint3D(Vector3 p) => new(p.X, p.Y, p.Z);

    private sealed record EndpointHit(string Name, string Category, bool Intersects, double ClearanceMm);
    private sealed record EndpointProbe(bool IsBlocked, IReadOnlyList<EndpointHit> Hits);
    private sealed record EndpointCandidate(Point3D Point, bool Adjusted, double OffsetMm, string Reason);
}

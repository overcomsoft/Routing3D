using System.Diagnostics;
using Routing3D.Engine;

namespace Routing3D.Samples.LargeSpace10mm.Pipeline;

/// <summary>
/// Step-by-step tutorial code for using the Routing3D native C++ engine from C#.
/// The methods mirror the API manual tutorial: configure, add obstacles, add tasks,
/// run single/octree/corridor/multi/waypoint routing, then dump a reproducible scene.
/// </summary>
public static class Routing3DTutorialSteps
{
    public sealed record StepSummary(
        int Step,
        string Name,
        bool Success,
        string Message,
        double ElapsedMs = 0,
        double LengthMm = 0,
        int Turns = 0,
        long ExpandedNodes = 0);

    public static IReadOnlyList<StepSummary> RunAll(Action<string>? log = null)
    {
        var steps = new List<StepSummary>
        {
            Step01ConfigureEngine(log),
            Step02AddObstacles(log),
            Step03AddRoutingTasks(log),
            Step04RouteSingleWithOctree(log),
            Step05RouteSingleWithHierarchicalCorridor(log),
            Step06RouteMultiplePipesWithProgress(log),
            Step07RouteViaWaypoints(log),
            Step08DumpSceneJson(log)
        };
        return steps;
    }

    public static StepSummary Step01ConfigureEngine(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        var grid = SampleDomain.Grid;
        var message = $"Grid {grid.Nx:N0} x {grid.Ny:N0} x {grid.Nz:N0}, cell={grid.CellMm}mm, native={Routing3DEngine.NativeVersion}";
        log?.Invoke(message);
        return new StepSummary(1, "Configure grid and parameters", true, message);
    }

    public static StepSummary Step02AddObstacles(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        AddObstacles(engine);
        var message = $"Added {SampleDomain.Obstacles.Count} obstacle AABBs.";
        log?.Invoke(message);
        return new StepSummary(2, "Add obstacles", true, message);
    }

    public static StepSummary Step03AddRoutingTasks(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        var taskIds = AddTasks(engine);
        var message = $"Added {taskIds.Count} route tasks with diameter and goal direction metadata.";
        log?.Invoke(message);
        return new StepSummary(3, "Add route tasks", true, message);
    }

    public static StepSummary Step04RouteSingleWithOctree(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        AddObstacles(engine);

        var task = SampleDomain.Tasks[0];
        var taskId = AddTask(engine, task);

        var sw = Stopwatch.StartNew();
        var result = engine.RouteTaskOctree(taskId, maxExpansions: 5_000_000, goalDirection: task.GoalDir);
        var leaves = engine.EnumOctreeLeaves(maxLeaves: 250_000);
        sw.Stop();

        var message = result.Success
            ? $"Octree route succeeded: length={result.LengthMm:N1}mm, turns={result.Turns}, leaves={leaves.Count:N0}."
            : $"Octree route failed: {result.Fail}, expanded={result.ExpandedNodes:N0}, leaves={leaves.Count:N0}.";
        log?.Invoke(message);
        return FromRouteResult(4, "Route one task with Octree Jump A*", result, message, sw.Elapsed.TotalMilliseconds);
    }

    public static StepSummary Step05RouteSingleWithHierarchicalCorridor(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        AddObstacles(engine);

        var task = SampleDomain.Tasks[0];
        var taskId = AddTask(engine, task);

        var sw = Stopwatch.StartNew();
        engine.RouteCorridorMulti(SampleDomain.HierFactor, SampleDomain.HierRadius, "longest", pipeRadius: 0);
        var result = engine.GetResult(taskId);
        sw.Stop();

        var message = result.Success
            ? $"Corridor route succeeded: length={result.LengthMm:N1}mm, turns={result.Turns}."
            : $"Corridor route failed: {result.Fail}.";
        log?.Invoke(message);
        return FromRouteResult(5, "Route one task with hierarchical corridor", result, message, sw.Elapsed.TotalMilliseconds);
    }

    public static StepSummary Step06RouteMultiplePipesWithProgress(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        AddObstacles(engine);
        AddTasks(engine);

        engine.SetPerTaskRadius(true);
        engine.SetPipeGap(60.0);
        engine.SetCbsDepth(2);
        engine.SetMinStraightMm(100.0);

        var completed = 0;
        var totalLength = 0.0;
        var totalTurns = 0;
        var sw = Stopwatch.StartNew();
        engine.RouteMultiProgress("diameter", progress =>
        {
            if (progress.Phase != 1) return;
            completed++;
            if (progress.Success)
            {
                totalLength += progress.LengthMm;
                totalTurns += progress.Turns;
            }
            log?.Invoke($"Task {progress.TaskIndex + 1}: success={progress.Success}, length={progress.LengthMm:N1}mm, turns={progress.Turns}");
        });
        sw.Stop();

        var ok = completed == SampleDomain.Tasks.Count;
        var message = $"Completed {completed}/{SampleDomain.Tasks.Count} tasks, totalLength={totalLength:N1}mm, totalTurns={totalTurns}.";
        log?.Invoke(message);
        return new StepSummary(6, "Route multiple pipes with progress", ok, message, sw.Elapsed.TotalMilliseconds, totalLength, totalTurns);
    }

    public static StepSummary Step07RouteViaWaypoints(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        AddObstacles(engine);

        var points = SampleDomain.WaypointPath;
        for (var i = 0; i + 1 < points.Length; i++)
        {
            var taskId = engine.AddTask(points[i], points[i + 1], "Exhaust", "Exhaust");
            engine.SetTaskDiameter(taskId, 165.0);
        }

        var sw = Stopwatch.StartNew();
        engine.RouteCorridorMulti(SampleDomain.HierFactor, SampleDomain.HierRadius, "longest", pipeRadius: 0);
        sw.Stop();

        var success = 0;
        var totalLength = 0.0;
        var totalTurns = 0;
        for (var i = 0; i + 1 < points.Length; i++)
        {
            var result = engine.GetResult(i);
            if (!result.Success) continue;
            success++;
            totalLength += result.LengthMm;
            totalTurns += result.Turns;
        }

        var message = $"Waypoint route segments {success}/{points.Length - 1}, totalLength={totalLength:N1}mm.";
        log?.Invoke(message);
        return new StepSummary(7, "Route via waypoints", success == points.Length - 1, message, sw.Elapsed.TotalMilliseconds, totalLength, totalTurns);
    }

    public static StepSummary Step08DumpSceneJson(Action<string>? log = null)
    {
        using var engine = CreateEngine();
        AddObstacles(engine);
        AddTasks(engine);

        var sceneJson = engine.DumpSceneText();
        var message = $"Dumped scene.json text: {sceneJson.Length:N0} characters.";
        log?.Invoke(message);
        return new StepSummary(8, "Dump reproducible scene.json", sceneJson.Length > 0, message);
    }

    private static Routing3DEngine CreateEngine()
    {
        var engine = new Routing3DEngine();
        engine.SetGrid(SampleDomain.Grid);
        engine.SetParameters(SampleDomain.Parameters);
        return engine;
    }

    private static void AddObstacles(Routing3DEngine engine)
    {
        foreach (var (_, box) in SampleDomain.Obstacles)
            engine.AddObstacle(box);
    }

    private static IReadOnlyList<int> AddTasks(Routing3DEngine engine)
    {
        var ids = new List<int>(SampleDomain.Tasks.Count);
        foreach (var task in SampleDomain.Tasks)
            ids.Add(AddTask(engine, task));
        return ids;
    }

    private static int AddTask(Routing3DEngine engine, SampleDomain.TaskDef task)
    {
        var taskId = engine.AddTask(task.Start, task.End, task.Utility, task.Group);
        engine.SetTaskDiameter(taskId, task.DiameterMm);
        engine.SetTaskGoalDirection(taskId, task.GoalDir);
        return taskId;
    }

    private static StepSummary FromRouteResult(int step, string name, RouteResult result, string message, double elapsedMs)
    {
        return new StepSummary(
            step,
            name,
            result.Success,
            message,
            elapsedMs,
            result.LengthMm,
            result.Turns,
            result.ExpandedNodes);
    }
}

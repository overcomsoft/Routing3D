// PipelineRunner.cs — Routing3D 8단계 파이프라인 실행기
// 각 단계는 독립적인 Routing3DEngine 인스턴스를 생성해 실행한다.
// 엔진 생성은 CreateEngine() 헬퍼로 통일 (SetGrid + SetParameters 포함).
using System.Diagnostics;
using System.IO;
using System.Text;
using Routing3D.Engine;

namespace Routing3D.Samples.LargeSpace10mm.Pipeline;

public sealed class PipelineRunner
{
    // 단계 완료마다 호출(UI 스레드 아님 — Dispatcher.Invoke 필요)
    public event Action<string>? ProgressMessage;

    // ── 전체 파이프라인 실행 ─────────────────────────────────────────────────
    public async Task<List<StepResult>> RunAllAsync(CancellationToken ct = default)
    {
        var results = new List<StepResult>();

        results.Add(await Step(1, "그리드 / 파라미터 구성",
            "SceneDoc 구성 — 격자 원점·형태·셀크기·비용 파라미터",
            r => RunStep1(r), ct));

        results.Add(await Step(2, "장애물 추가",
            "외벽·기둥·장비·덕트·슬래브 AABB 를 점유맵에 추가",
            r => RunStep2(r), ct));

        results.Add(await Step(3, "라우팅 작업 정의",
            "시작/종단 좌표·유틸리티·관경·목표 진입축(goal_dir) 설정",
            r => RunStep3(r), ct));

        results.Add(await Step(4, "단일 A* 탐색 (RouteTask)",
            "T1 배관 직접 A* 단독 탐색 — 180억 셀 탐색상한 도달 예상(정상)",
            r => RunStep4(r), ct));

        results.Add(await Step(5, "계층 Corridor 라우팅 (RouteCorridorMulti)",
            $"coarse factor={SampleDomain.HierFactor}(≈200mm) → fine 튜브 A* — 거대격자 안전",
            r => RunStep5(r), ct));

        results.Add(await Step(6, "다중 배관 라우팅 (RouteMultiProgress)",
            "5개 배관 전체 — per-task 반경·이격 60mm·CBS depth=2·최소직선 100mm",
            r => RunStep6(r), ct));

        results.Add(await Step(7, "경유점(Waypoint) 라우팅",
            "start→wp1→wp2→end 구간 분해 후 순차 탐색 → 경로 이어붙이기",
            r => RunStep7(r), ct));

        results.Add(await Step(8, "결과 리포트 Markdown 저장",
            "전 단계 결과를 내 문서\\Routing3D\\sample_pipeline_<ts>.md 에 저장",
            r => RunStep8(r, results), ct));

        return results;
    }

    // ── 단계 래퍼 ─────────────────────────────────────────────────────────
    private async Task<StepResult> Step(int n, string name, string desc,
        Action<StepResult> body, CancellationToken ct)
    {
        var r = new StepResult { StepNumber = n, StepName = name, StepDesc = desc };
        ProgressMessage?.Invoke($"▶ Step {n}: {name}...");
        var sw = Stopwatch.StartNew();
        try
        {
            ct.ThrowIfCancellationRequested();
            await Task.Run(() => body(r), ct);
        }
        catch (OperationCanceledException)
        {
            r.Status = StepStatus.Skipped;
            r.AddLog("취소됨");
        }
        catch (Exception ex)
        {
            r.Status = StepStatus.Failed;
            r.Error = ex;
            r.AddLog($"예외: {ex.GetType().Name}: {ex.Message}");
        }
        r.WallTime = sw.Elapsed;
        ProgressMessage?.Invoke($"  {r.StatusIcon} Step {n} 완료 [{r.WallTimeStr}]");
        return r;
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 1: 그리드 / 파라미터 구성
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep1(StepResult r)
    {
        var g = SampleDomain.Grid;
        var p = SampleDomain.Parameters;

        long coarse = (long)(g.Nx / SampleDomain.HierFactor)
                           * (g.Ny / SampleDomain.HierFactor)
                           * (g.Nz / SampleDomain.HierFactor);

        r.AddLog($"도메인 : {SampleDomain.DomainX/1000:F0}m × {SampleDomain.DomainY/1000:F0}m × {SampleDomain.DomainZ/1000:F0}m");
        r.AddLog($"셀 크기 : {g.CellMm} mm");
        r.AddLog($"격자    : {g.Nx:N0} × {g.Ny:N0} × {g.Nz:N0} = {SampleDomain.TotalCells:N0} 셀 (180억)");
        r.AddLog($"Dense 필요 RAM ≈ {SampleDomain.TotalCells / 1e9:F1} GB → ImplicitOccupancy 자동 전환 (>5M)");
        r.AddLog($"coarse factor={SampleDomain.HierFactor} → coarse 셀 {g.CellMm*SampleDomain.HierFactor:F0}mm");
        r.AddLog($"coarse 격자 : {g.Nx/SampleDomain.HierFactor:N0} × {g.Ny/SampleDomain.HierFactor:N0} × {g.Nz/SampleDomain.HierFactor:N0} = {coarse:N0} 셀");

        r.AddMetric("도메인",       $"30m × 30m × 20m");
        r.AddMetric("셀 크기",      $"{g.CellMm} mm");
        r.AddMetric("격자 셀 수",   $"{SampleDomain.TotalCells:N0}");
        r.AddMetric("Dense RAM",    $"≈{SampleDomain.TotalCells/1e9:F1} GB → 불가");
        r.AddMetric("점유맵 백엔드","ImplicitOccupancy (자동, >5M 셀)");
        r.AddMetric("coarse factor",$"{SampleDomain.HierFactor} (≈{g.CellMm*SampleDomain.HierFactor:F0}mm)");
        r.AddMetric("coarse 셀 수", $"{coarse:N0}");
        r.AddMetric("w_turn",       $"{p.TurnCostMm} mm / 꺾임");
        r.AddMetric("w_heur",       $"{p.HeuristicWeight} (근처→{p.NearGoalHeuristicWeight})");
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 2: 장애물 추가
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep2(StepResult r)
    {
        using var e = CreateEngine();
        foreach (var (_, box) in SampleDomain.Obstacles)
            e.AddObstacle(box);

        var obs = SampleDomain.Obstacles;
        r.AddLog($"총 장애물 {obs.Count}개 추가");
        r.AddLog($"  외벽 5면 + 바닥 슬래브");
        r.AddLog($"  기둥 {obs.Count(o => o.Name.StartsWith("기둥"))}개 (4×4 그리드, 6m 간격, 500×500mm)");
        r.AddLog($"  주요장비 {obs.Count(o => o.Name.StartsWith("장비"))}개 (펌프·압축기·열교환기·탱크)");
        r.AddLog($"  덕트 {obs.Count(o => o.Name.StartsWith("덕트"))}개 (배기·공급, 단면 1000×800/600×600mm)");
        r.AddLog($"  중간 슬래브 {obs.Count(o => o.Name.StartsWith("층"))}개 (8m·15m 고도)");
        r.AddLog("각 장애물 상세:");
        foreach (var (name, box) in obs)
            r.AddLog($"  [{name}]  ({box.Min.X:F0},{box.Min.Y:F0},{box.Min.Z:F0}) → ({box.Max.X:F0},{box.Max.Y:F0},{box.Max.Z:F0})");

        r.AddMetric("총 장애물",   $"{obs.Count}");
        r.AddMetric("외벽/슬래브", $"{obs.Count(o => o.Name.Contains("벽") || o.Name.Contains("슬래브"))}");
        r.AddMetric("기둥",        $"{obs.Count(o => o.Name.StartsWith("기둥"))}");
        r.AddMetric("주요장비",    $"{obs.Count(o => o.Name.StartsWith("장비"))}");
        r.AddMetric("덕트",        $"{obs.Count(o => o.Name.StartsWith("덕트"))}");
        r.AddMetric("중간 슬래브", $"{obs.Count(o => o.Name.StartsWith("층"))}");
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 3: 라우팅 작업 정의
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep3(StepResult r)
    {
        using var e = CreateEngine();
        for (int i = 0; i < SampleDomain.Tasks.Count; i++)
        {
            var t = SampleDomain.Tasks[i];
            int ti = e.AddTask(t.Start, t.End, t.Utility, t.Group);
            e.SetTaskDiameter(ti, t.DiameterMm);
            e.SetTaskGoalDirection(ti, t.GoalDir);

            r.AddLog($"  {t.Label}: [{t.Group}] {t.Utility} φ{t.DiameterMm:F0}mm" +
                     $"  ({t.Start.X:F0},{t.Start.Y:F0},{t.Start.Z:F0})→" +
                     $"({t.End.X:F0},{t.End.Y:F0},{t.End.Z:F0})  goal={t.GoalDir}");
            r.Tasks.Add(new TaskStepResult
            {
                TaskIndex  = i, Label = t.Label,
                Utility    = t.Utility, DiameterMm = t.DiameterMm,
            });
        }

        r.AddMetric("작업 수",   $"{SampleDomain.Tasks.Count}");
        r.AddMetric("최대 관경", $"{SampleDomain.Tasks.Max(t => t.DiameterMm):F0} mm  (200A φ219)");
        r.AddMetric("최소 관경", $"{SampleDomain.Tasks.Min(t => t.DiameterMm):F0} mm  (25A φ34)");
        r.AddMetric("우선순위",  "diameter — 굵은 배관 먼저 라우팅");
        r.AddMetric("goal_dir", "T2 배기: PositiveZ (덕트 하단 수직 진입), 나머지 Any");
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 4: 단일 A* 탐색 (RouteTask) — 180억 셀 → 탐색상한 도달 예상
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep4(StepResult r)
    {
        using var e = CreateEngine();
        AddObstacles(e);
        e.SetCollectVisited(true);  // 방문맵 수집(탐색 진도 확인용)

        var t = SampleDomain.Tasks[0];  // T1: CW 200A
        int ti = e.AddTask(t.Start, t.End, t.Utility, t.Group);
        e.SetTaskDiameter(ti, t.DiameterMm);

        r.AddLog($"T1 {t.Label} 단독 RouteTask (직접 A*)");
        r.AddLog("예상: 180억 셀 → 탐색상한(48M) 도달로 실패 → Step 5 계층 라우팅으로 해결");

        var res = e.RouteTask(ti);

        string note = res.Success
            ? "직접 A* 성공 (의외의 쾌거 — 경로가 짧아 probe 내 해결)"
            : $"실패({res.Fail}) — 탐색상한 도달, 이 도메인에서 정상 동작";

        r.AddLog($"결과: {(res.Success ? "✓ 성공" : "✗ 실패")}");
        r.AddLog(note);
        r.AddLog($"확장 노드: {res.ExpandedNodes:N0}");
        r.AddLog($"방문 셀: {res.Visited.Count:N0}");
        r.AddLog($"네이티브 {res.ElapsedMs:#,0.0} ms");

        r.AddMetric("탐색 배관",  t.Label);
        r.AddMetric("성공 여부",  res.Success ? "✓ 성공" : $"✗ {res.Fail}");
        r.AddMetric("경로 길이",  res.Success ? $"{res.LengthMm:#,0.0} mm" : "-");
        r.AddMetric("꺾임 수",    res.Success ? $"{res.Turns}" : "-");
        r.AddMetric("확장 노드",  $"{res.ExpandedNodes:N0}");
        r.AddMetric("방문 셀",    $"{res.Visited.Count:N0}");
        r.AddMetric("소요시간",   $"{res.ElapsedMs:#,0.0} ms");
        r.AddMetric("비고",       note);

        r.Tasks.Add(new TaskStepResult
        {
            TaskIndex = 0, Label = t.Label, Utility = t.Utility, DiameterMm = t.DiameterMm,
            Success = res.Success, LengthMm = res.LengthMm, Turns = res.Turns,
            ExpandedNodes = res.ExpandedNodes, ElapsedMs = res.ElapsedMs,
            FailReason = res.Fail.ToString(),
        });
        // 실패 예상 단계 — Status는 Ok 유지(예상된 결과)
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 5: 계층 Corridor 라우팅 (RouteCorridorMulti)
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep5(StepResult r)
    {
        using var e = CreateEngine();
        AddObstacles(e);

        // T1 배관 추가 (Step 4 와 동일 배관, 비교용)
        var t = SampleDomain.Tasks[0];
        int ti = e.AddTask(t.Start, t.End, t.Utility, t.Group);
        e.SetTaskDiameter(ti, t.DiameterMm);

        long coarseCells = (long)(SampleDomain.Grid.Nx / SampleDomain.HierFactor)
                                * (SampleDomain.Grid.Ny / SampleDomain.HierFactor)
                                * (SampleDomain.Grid.Nz / SampleDomain.HierFactor);

        r.AddLog($"RouteCorridorMulti: factor={SampleDomain.HierFactor}, radius={SampleDomain.HierRadius}");
        r.AddLog($"coarse 격자: {SampleDomain.Grid.Nx/SampleDomain.HierFactor}×{SampleDomain.Grid.Ny/SampleDomain.HierFactor}×{SampleDomain.Grid.Nz/SampleDomain.HierFactor} = {coarseCells:N0} 셀");
        r.AddLog($"(fine {SampleDomain.TotalCells:N0} 셀의 1/{SampleDomain.TotalCells/coarseCells:N0})");

        var sw = Stopwatch.StartNew();
        e.RouteCorridorMulti(SampleDomain.HierFactor, SampleDomain.HierRadius, "longest", 0);
        sw.Stop();

        var res = e.GetResult(ti);

        r.AddLog($"T1 결과: {(res.Success ? "✓ 성공" : "✗ 실패")}");
        if (res.Success)
            r.AddLog($"  길이={res.LengthMm:#,0.0}mm  꺾임={res.Turns}  확장={res.ExpandedNodes:N0}  {res.ElapsedMs:#,0.0}ms");
        r.AddLog($"  총 소요시간(관리): {sw.Elapsed.TotalMilliseconds:#,0.0} ms");

        r.AddMetric("탐색 방식",    "계층 corridor (coarse 최적→fine 튜브)");
        r.AddMetric("coarse factor", $"{SampleDomain.HierFactor}  (coarse 셀 ≈{SampleDomain.CellMm*SampleDomain.HierFactor:F0}mm)");
        r.AddMetric("corridor 반경", $"{SampleDomain.HierRadius} coarse 셀");
        r.AddMetric("coarse 셀 수",  $"{coarseCells:N0}");
        r.AddMetric("성공 여부",     res.Success ? "✓ 성공" : $"✗ {res.Fail}");
        r.AddMetric("경로 길이",     res.Success ? $"{res.LengthMm:#,0.0} mm" : "-");
        r.AddMetric("꺾임 수",       res.Success ? $"{res.Turns}" : "-");
        r.AddMetric("확장 노드",     $"{res.ExpandedNodes:N0}");
        r.AddMetric("소요시간",      $"{sw.Elapsed.TotalMilliseconds:#,0.0} ms");

        r.Tasks.Add(new TaskStepResult
        {
            TaskIndex = 0, Label = t.Label, Utility = t.Utility, DiameterMm = t.DiameterMm,
            Success = res.Success, LengthMm = res.LengthMm, Turns = res.Turns,
            ExpandedNodes = res.ExpandedNodes, ElapsedMs = res.ElapsedMs,
            FailReason = res.Fail.ToString(),
        });
        if (!res.Success) r.Status = StepStatus.Failed;
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 6: 다중 배관 라우팅 (RouteMultiProgress)
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep6(StepResult r)
    {
        using var e = CreateEngine();
        AddObstacles(e);

        for (int i = 0; i < SampleDomain.Tasks.Count; i++)
        {
            var t = SampleDomain.Tasks[i];
            int ti = e.AddTask(t.Start, t.End, t.Utility, t.Group);
            e.SetTaskDiameter(ti, t.DiameterMm);
            e.SetTaskGoalDirection(ti, t.GoalDir);
        }

        // 물리 배관 + 품질 옵션
        e.SetPerTaskRadius(true);   // B1: 관경별 마킹 반경 (ceil(d/cell)-1)
        e.SetPipeGap(60.0);         // P3s: 이격 gap — 센터선 ≥ r1+r2+60mm
        e.SetCbsDepth(2);           // C1: CBS-lite depth=2 (혼잡 협상)
        e.SetMinStraightMm(100.0);  // P3u: 코너 최소직선 하드 100mm

        r.AddLog("옵션: per-task 반경 ON, 이격 gap=60mm, CBS depth=2, 최소직선=100mm");
        r.AddLog("우선순위: diameter (φ219 굵은 배관 먼저 → 직선 선점 → 가는 배관 회피)");

        var taskRows = new TaskStepResult[SampleDomain.Tasks.Count];
        var sw = Stopwatch.StartNew();

        e.RouteMultiProgress("diameter", progress =>
        {
            if (progress.Phase == 1)  // 배관 완료 이벤트
            {
                var t = SampleDomain.Tasks[progress.TaskIndex];
                r.AddLog($"  [{progress.Done}/{progress.Total}] {t.Label}  order={progress.OrderIndex}" +
                         $"  {(progress.Success ? "✓" : "✗")}  길이={progress.LengthMm:#,0.0}mm" +
                         $"  꺾임={progress.Turns}  exp={progress.ExpandedNodes:N0}  {progress.ElapsedMs:#,0.0}ms");
                taskRows[progress.TaskIndex] = new TaskStepResult
                {
                    TaskIndex = progress.TaskIndex, Label = t.Label,
                    Utility = t.Utility, DiameterMm = t.DiameterMm,
                    Success = progress.Success, LengthMm = progress.LengthMm,
                    Turns = progress.Turns, ExpandedNodes = progress.ExpandedNodes,
                    ElapsedMs = progress.ElapsedMs,
                };
            }
        });

        sw.Stop();

        int   succN    = taskRows.Count(t => t?.Success == true);
        double totalLen = taskRows.Where(t => t?.Success == true).Sum(t => t!.LengthMm);
        int   totalT   = taskRows.Where(t => t?.Success == true).Sum(t => t!.Turns);

        r.AddLog($"요약: {succN}/{SampleDomain.Tasks.Count} 성공  총길이={totalLen:#,0.0}mm  총꺾임={totalT}  {sw.Elapsed.TotalMilliseconds:#,0.0}ms");

        r.AddMetric("성공/전체",    $"{succN}/{SampleDomain.Tasks.Count}");
        r.AddMetric("총 경로 길이", $"{totalLen:#,0.0} mm");
        r.AddMetric("총 꺾임",     $"{totalT}");
        r.AddMetric("총 소요시간", $"{sw.Elapsed.TotalMilliseconds:#,0.0} ms");
        r.AddMetric("per-task 반경","ON");
        r.AddMetric("이격 gap",     "60 mm");
        r.AddMetric("CBS depth",    "2");
        r.AddMetric("최소직선",     "100 mm");

        foreach (var tr in taskRows)
            if (tr != null) r.Tasks.Add(tr);

        if (succN == 0) r.Status = StepStatus.Failed;
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 7: 경유점(Waypoint) 라우팅 — 구간 분해 후 순차 탐색
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep7(StepResult r)
    {
        var path   = SampleDomain.WaypointPath;
        var labels = SampleDomain.WaypointLabels;

        using var e = CreateEngine();
        AddObstacles(e);

        r.AddLog($"경유 경로: {path.Length} 점 → {path.Length-1} 구간");
        for (int i = 0; i < path.Length; i++)
            r.AddLog($"  P{i}: ({path[i].X:F0},{path[i].Y:F0},{path[i].Z:F0})  [{labels[i]}]");

        // 각 구간을 독립 작업으로 추가 — corridor 라우팅이 구간 간 충돌 없이 순서대로 라우팅
        for (int seg = 0; seg + 1 < path.Length; seg++)
            e.AddTask(path[seg], path[seg + 1], "Exhaust", "Exhaust");

        var sw = Stopwatch.StartNew();
        e.RouteCorridorMulti(SampleDomain.HierFactor, SampleDomain.HierRadius, "longest", 0);
        sw.Stop();

        double totalLen = 0; int totalT = 0; int succSeg = 0;
        for (int seg = 0; seg + 1 < path.Length; seg++)
        {
            var res = e.GetResult(seg);
            if (res.Success) { totalLen += res.LengthMm; totalT += res.Turns; succSeg++; }
            r.AddLog($"  구간{seg}: ({path[seg].X:F0},{path[seg].Y:F0},{path[seg].Z:F0})→" +
                     $"({path[seg+1].X:F0},{path[seg+1].Y:F0},{path[seg+1].Z:F0})" +
                     $"  {(res.Success ? "✓" : "✗")}  길이={res.LengthMm:#,0.0}mm  꺾임={res.Turns}");
            r.Tasks.Add(new TaskStepResult
            {
                TaskIndex = seg, Label = $"구간{seg} ({labels[seg]}→{labels[seg+1]})",
                Utility = "Exhaust", DiameterMm = 165.0,
                Success = res.Success, LengthMm = res.LengthMm, Turns = res.Turns,
                ExpandedNodes = res.ExpandedNodes, ElapsedMs = res.ElapsedMs,
            });
        }

        r.AddLog($"이어붙인 경로: {succSeg}/{path.Length-1} 구간 성공  길이≈{totalLen:#,0.0}mm  꺾임≈{totalT}");
        r.AddLog($"소요시간: {sw.Elapsed.TotalMilliseconds:#,0.0} ms");

        r.AddMetric("구간 수",       $"{path.Length - 1}");
        r.AddMetric("성공 구간",     $"{succSeg}/{path.Length - 1}");
        r.AddMetric("이어붙인 길이", $"{totalLen:#,0.0} mm");
        r.AddMetric("이어붙인 꺾임", $"{totalT}");
        r.AddMetric("소요시간",      $"{sw.Elapsed.TotalMilliseconds:#,0.0} ms");

        if (succSeg == 0) r.Status = StepStatus.Failed;
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Step 8: 결과 리포트 Markdown 저장
    // ══════════════════════════════════════════════════════════════════════════
    private static void RunStep8(StepResult r, IReadOnlyList<StepResult> all)
    {
        string md  = BuildMarkdown(all);
        string dir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Routing3D");
        Directory.CreateDirectory(dir);
        string ts   = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string path = Path.Combine(dir, $"sample_pipeline_{ts}.md");
        File.WriteAllText(path, md, Encoding.UTF8);

        long size = new FileInfo(path).Length;
        r.AddLog($"저장 완료: {path}");
        r.AddLog($"파일 크기: {size:N0} bytes");

        r.AddMetric("저장 경로", path);
        r.AddMetric("파일 크기", $"{size:N0} bytes");
        r.AddMetric("_filepath", path);   // ReportWindow 에서 파일 열기용(숨김 key)
    }

    // ── Markdown 생성 ────────────────────────────────────────────────────────
    public static string BuildMarkdown(IReadOnlyList<StepResult> steps)
    {
        var sb = new StringBuilder();
        sb.AppendLine("# Routing3D — 8단계 파이프라인 실행 결과");
        sb.AppendLine();
        sb.AppendLine($"- **실행 시각**: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        sb.AppendLine($"- **엔진 버전**: {Routing3DEngine.NativeVersion}");
        sb.AppendLine($"- **도메인**: 30m × 30m × 20m, 셀 10mm ({SampleDomain.TotalCells:N0} 셀)");
        sb.AppendLine($"- **점유맵 백엔드**: ImplicitOccupancy (자동, >5M 셀)");
        sb.AppendLine();

        // 요약 표
        sb.AppendLine("## 단계별 요약");
        sb.AppendLine();
        sb.AppendLine("| 단계 | 이름 | 상태 | 소요시간 |");
        sb.AppendLine("|:----:|------|:----:|--------:|");
        foreach (var s in steps)
            sb.AppendLine($"| {s.StepNumber} | {s.StepName} | {s.StatusIcon} {s.Status} | {s.WallTimeStr} |");
        sb.AppendLine();

        // 단계별 상세
        foreach (var s in steps)
        {
            sb.AppendLine($"---");
            sb.AppendLine($"## Step {s.StepNumber}: {s.StepName}");
            sb.AppendLine();
            sb.AppendLine($"> {s.StepDesc}");
            sb.AppendLine();

            if (s.Metrics.Any(m => !m.Key.StartsWith("_")))
            {
                sb.AppendLine("### 메트릭");
                sb.AppendLine("| 항목 | 값 |");
                sb.AppendLine("|------|-----|");
                foreach (var (key, val) in s.Metrics.Where(m => !m.Key.StartsWith("_")))
                    sb.AppendLine($"| {key} | `{val}` |");
                sb.AppendLine();
            }

            if (s.Tasks.Count > 0)
            {
                sb.AppendLine("### 작업별 결과");
                sb.AppendLine("| # | 라벨 | 유틸 | φ(mm) | 성공 | 길이(mm) | 꺾임 | 확장 노드 | ms |");
                sb.AppendLine("|--:|------|------|------:|:----:|---------:|-----:|----------:|---:|");
                foreach (var t in s.Tasks)
                    sb.AppendLine($"| {t.TaskIndex+1} | {t.Label} | {t.Utility} | {t.DiameterMm:F0}" +
                                  $" | {t.StatusIcon} | {t.LengthStr} | {t.TurnsStr}" +
                                  $" | {t.ExpandedNodes:N0} | {t.ElapsedMs:#,0.0} |");
                sb.AppendLine();
            }

            if (s.Log.Count > 0)
            {
                sb.AppendLine("### 로그");
                sb.AppendLine("```");
                foreach (var line in s.Log) sb.AppendLine(line);
                sb.AppendLine("```");
                sb.AppendLine();
            }
        }
        return sb.ToString();
    }

    // ── 공통 헬퍼 ────────────────────────────────────────────────────────────
    private static Routing3DEngine CreateEngine()
    {
        var e = new Routing3DEngine();
        e.SetGrid(SampleDomain.Grid);
        e.SetParameters(SampleDomain.Parameters);
        return e;
    }

    private static void AddObstacles(Routing3DEngine e)
    {
        foreach (var (_, box) in SampleDomain.Obstacles)
            e.AddObstacle(box);
    }
}

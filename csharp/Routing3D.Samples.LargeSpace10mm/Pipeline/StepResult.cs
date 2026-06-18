// StepResult.cs — 8단계 파이프라인 단계별 결과 모델
namespace Routing3D.Samples.LargeSpace10mm.Pipeline;

public enum StepStatus { Ok, Failed, Skipped }

/// <summary>한 배관 작업의 라우팅 결과.</summary>
public sealed class TaskStepResult
{
    public int    TaskIndex    { get; init; }
    public string Label        { get; init; } = "";
    public string Utility      { get; init; } = "";
    public double DiameterMm   { get; init; }
    public bool   Success      { get; init; }
    public double LengthMm     { get; init; }
    public int    Turns        { get; init; }
    public long   ExpandedNodes{ get; init; }
    public double ElapsedMs    { get; init; }
    public string FailReason   { get; init; } = "";

    public string StatusIcon  => Success ? "✓" : "✗";
    public string LengthStr   => Success ? $"{LengthMm:#,0.0} mm" : "-";
    public string TurnsStr    => Success ? $"{Turns:#,0}" : "-";
    public string ExpStr      => $"{ExpandedNodes:N0}";
    public string ElapsedStr  => $"{ElapsedMs:#,0.0} ms";
}

/// <summary>한 단계(Step)의 실행 결과 — 메트릭·로그·작업별 결과 포함.</summary>
public sealed class StepResult
{
    public int    StepNumber { get; init; }
    public string StepName   { get; init; } = "";
    public string StepDesc   { get; init; } = "";

    public StepStatus  Status   { get; set; } = StepStatus.Ok;
    public TimeSpan    WallTime { get; set; }
    public Exception?  Error    { get; set; }

    public List<string>              Log     { get; } = new();
    public List<(string Key, string Value)> Metrics { get; } = new();
    public List<TaskStepResult>      Tasks   { get; } = new();

    public void AddLog(string line) => Log.Add(line);
    public void AddMetric(string key, string value) => Metrics.Add((key, value));

    public string StatusIcon => Status switch
    {
        StepStatus.Ok      => "✓",
        StepStatus.Failed  => "✗",
        StepStatus.Skipped => "–",
        _                  => "?"
    };
    public string StatusColor => Status switch
    {
        StepStatus.Ok      => "#50C878",
        StepStatus.Failed  => "#FF5555",
        StepStatus.Skipped => "#888888",
        _                  => "#AAAAAA"
    };
    public string WallTimeStr => WallTime.TotalSeconds >= 1
        ? $"{WallTime.TotalSeconds:F2} s"
        : $"{WallTime.TotalMilliseconds:F1} ms";

    /// <summary>Step 헤더 요약 한 줄.</summary>
    public string Summary => $"Step {StepNumber}  {StepName}  [{WallTimeStr}]";
}

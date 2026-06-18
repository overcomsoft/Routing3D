// SampleDomain.cs — 30m×30m×20m 샘플 도메인 상수 + 장애물·작업 정의
// 셀 10mm → 3,000×3,000×2,000 = 18,000,000,000 셀 (180억)
// Dense 필요 RAM ≈ 18GB → 엔진이 자동으로 ImplicitOccupancy 전환 (large_grid_threshold=5M)
using Routing3D.Engine;

namespace Routing3D.Samples.LargeSpace10mm.Pipeline;

public static class SampleDomain
{
    // ── 격자 상수 ───────────────────────────────────────────────────────────
    public const double CellMm  = 10.0;
    public const double DomainX = 30_000.0;   // 30m
    public const double DomainY = 30_000.0;   // 30m
    public const double DomainZ = 20_000.0;   // 20m (높이)

    public static readonly Vec3 Origin = new(0, 0, 0);
    public static int  Nx         => (int)(DomainX / CellMm);       // 3,000
    public static int  Ny         => (int)(DomainY / CellMm);       // 3,000
    public static int  Nz         => (int)(DomainZ / CellMm);       // 2,000
    public static long TotalCells => (long)Nx * Ny * Nz;            // 18,000,000,000

    // coarse factor: ceil(200mm / cell) = 20 → coarse 셀 ≈ 200mm
    public const int HierFactor = 20;
    public const int HierRadius = 2;

    public static readonly RoutingGrid Grid = new(CellMm, Origin,
        (int)(DomainX / CellMm), (int)(DomainY / CellMm), (int)(DomainZ / CellMm));

    // ImplicitOccupancy 에서 clearance_map 전역 배열 미생성 → w_clear=0
    public static readonly RoutingParameters Parameters = new()
    {
        CellMm                  = CellMm,
        TurnCostMm              = 200.0,  // 꺾임 1회 ≈ 200mm 페널티
        ClearanceCostMm         = 0.0,
        ClearanceRadiusCells    = 0,
        ClearanceConnectivity   = 6,
        HeuristicWeight         = 2.0,    // 먼 곳: 목표 지향(빠름)
        NearGoalHeuristicWeight = 1.0,    // 목표 근처: 정확
    };

    // ── 장애물 목록 (이름 + AABB, 단위 mm) ──────────────────────────────────
    public static readonly IReadOnlyList<(string Name, Aabb Box)> Obstacles = new (string, Aabb)[]
    {
        // 외벽 5면 (두께 500mm)
        ("외벽 X−",      new(new(  -500,     0,     0), new(     0, 30000, 20000))),
        ("외벽 X+",      new(new( 30000,     0,     0), new( 30500, 30000, 20000))),
        ("외벽 Y−",      new(new(     0,  -500,     0), new( 30000,     0, 20000))),
        ("외벽 Y+",      new(new(     0, 30000,     0), new( 30000, 30500, 20000))),
        ("바닥 슬래브",   new(new(     0,     0,  -300), new( 30000, 30000,     0))),
        // 기둥 4×4 grid (6m 간격, 단면 500×500mm, 전체 높이)
        ("기둥(1,1)", new(new( 5750,  5750, 0), new( 6250,  6250, 20000))),
        ("기둥(1,2)", new(new( 5750, 11750, 0), new( 6250, 12250, 20000))),
        ("기둥(1,3)", new(new( 5750, 17750, 0), new( 6250, 18250, 20000))),
        ("기둥(1,4)", new(new( 5750, 23750, 0), new( 6250, 24250, 20000))),
        ("기둥(2,1)", new(new(11750,  5750, 0), new(12250,  6250, 20000))),
        ("기둥(2,2)", new(new(11750, 11750, 0), new(12250, 12250, 20000))),
        ("기둥(2,3)", new(new(11750, 17750, 0), new(12250, 18250, 20000))),
        ("기둥(2,4)", new(new(11750, 23750, 0), new(12250, 24250, 20000))),
        ("기둥(3,1)", new(new(17750,  5750, 0), new(18250,  6250, 20000))),
        ("기둥(3,2)", new(new(17750, 11750, 0), new(18250, 12250, 20000))),
        ("기둥(3,3)", new(new(17750, 17750, 0), new(18250, 18250, 20000))),
        ("기둥(3,4)", new(new(17750, 23750, 0), new(18250, 24250, 20000))),
        ("기둥(4,1)", new(new(23750,  5750, 0), new(24250,  6250, 20000))),
        ("기둥(4,2)", new(new(23750, 11750, 0), new(24250, 12250, 20000))),
        ("기둥(4,3)", new(new(23750, 17750, 0), new(24250, 18250, 20000))),
        ("기둥(4,4)", new(new(23750, 23750, 0), new(24250, 24250, 20000))),
        // 주요장비
        ("장비A 펌프",     new(new( 3000,  3000,     0), new( 5000,  4500,  1200))),
        ("장비B 압축기",   new(new(22000,  5000,     0), new(25000,  7000,  2000))),
        ("장비C 열교환기", new(new(14000, 14000,     0), new(15500, 15000,  3000))),
        ("장비D 탱크",     new(new( 8000, 20000,     0), new(12000, 24000,  5000))),
        // 덕트
        ("덕트A 배기 (X전체)", new(new(    0, 14000, 12000), new(30000, 15000, 12800))),
        ("덕트B 공급 (Y전체)", new(new( 6800,     0, 10000), new( 7400, 30000, 10600))),
        // 중간 슬래브
        ("층1 슬래브 8m",  new(new(    0,     0,  8000), new(30000, 20000,  8300))),
        ("층2 슬래브 15m", new(new(    0,     0, 15000), new(15000, 30000, 15300))),
    };

    // ── 라우팅 작업 정의 ──────────────────────────────────────────────────────
    public readonly record struct TaskDef(
        string Label, string Utility, string Group,
        double DiameterMm, Vec3 Start, Vec3 End, GoalDirection GoalDir);

    public static readonly IReadOnlyList<TaskDef> Tasks = new[]
    {
        // T1: 냉각수 공급 (CW, 200A φ219mm) — 펌프 출구 → 압축기 입구
        new TaskDef("T1 CW 200A",   "CW",      "Cooling",     219.0,
            new Vec3( 5000,  3750, 1000), new Vec3(22000,  6000, 1000), GoalDirection.Any),
        // T2: 배기 (Exhaust, 150A φ165mm) — 압축기 배기구 → 덕트 A (+Z 진입)
        new TaskDef("T2 배기 150A", "Exhaust", "Exhaust",     165.0,
            new Vec3(23500,  6000, 2000), new Vec3(23000, 14500, 12000), GoalDirection.PositiveZ),
        // T3: 냉각수 환수 (CR, 200A φ219mm) — 압축기 → 펌프
        new TaskDef("T3 CR 200A",   "CR",      "Cooling",     219.0,
            new Vec3(22000,  6000, 1200), new Vec3( 3000,  3750, 1200), GoalDirection.Any),
        // T4: 공정수 (PW, 100A φ114mm) — 열교환기 → 저장탱크
        new TaskDef("T4 PW 100A",   "PW",      "Process",     114.0,
            new Vec3(14750, 14000, 2500), new Vec3(10000, 20000, 3000), GoalDirection.Any),
        // T5: 계장공기 (IA, 25A φ34mm) — 제어밸브 → 제어판넬
        new TaskDef("T5 IA 25A",    "IA",      "Instrument",   34.0,
            new Vec3( 4000,  4000, 1100), new Vec3( 4500,  8000, 3000), GoalDirection.Any),
    };

    // ── 경유점 데모 경로 (Step 7) ────────────────────────────────────────────
    // start → 수직 라이저 끝 → 랙 고도 경유 → 덕트 하단 PoC
    public static readonly Vec3[] WaypointPath =
    {
        new Vec3(23500,  6000,  2000),  // [0] 압축기 배기구 (start)
        new Vec3(23500,  6000,  5000),  // [1] 수직 라이저 끝 5m (wp1)
        new Vec3(20000, 14000, 10000),  // [2] 랙 고도 10m 수평 경유 (wp2)
        new Vec3(20000, 14500, 12000),  // [3] 덕트 A 하단 PoC (end)
    };

    public static readonly string[] WaypointLabels =
    {
        "압축기 배기구 (start)",
        "수직 라이저 끝 5m (wp1)",
        "랙 고도 10m 경유 (wp2)",
        "덕트 A 하단 PoC (end)",
    };
}

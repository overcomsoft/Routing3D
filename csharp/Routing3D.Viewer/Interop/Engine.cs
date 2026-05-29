// 매니지드 엔진 래퍼 — C# 코드가 실제로 사용하는 OOP 면
// =============================================================================
//   Native(P/Invoke) + R3dEngineHandle 위에 예외 기반 OOP API 를 제공한다.
//   상태 코드가 0(R3D_OK)이 아니면 예외를 던진다.
// =============================================================================
using System;

namespace Routing3D.Viewer.Interop
{
    /// <summary>경로 셀 (i, j, k).</summary>
    public readonly record struct PathCell(int I, int J, int K);

    /// <summary>한 작업의 라우팅 결과(성공/길이/회전/경로).</summary>
    public sealed class RouteResult
    {
        public bool Success { get; init; }
        public double LengthMm { get; init; }
        public double CostMm { get; init; }
        public int Turns { get; init; }
        public long ExpandedNodes { get; init; }
        public PathCell[] Path { get; init; } = Array.Empty<PathCell>();
    }

    public sealed class Engine : IDisposable
    {
        private readonly R3dEngineHandle _h = R3dEngineHandle.Create();

        public bool IsValid => !_h.IsInvalid;
        public static string Version => Native.VersionString();

        // ---- 장면 구성(Level 2) ----
        public void LoadSceneText(string sceneText)
            => Check(Native.r3d_load_scene_text(H, Native.Utf8(sceneText)), "load_scene_text");

        public void SetGrid(double cellMm, double ox, double oy, double oz, int nx, int ny, int nz)
        {
            var g = new Native.R3dGrid { cell_mm = cellMm, ox = ox, oy = oy, oz = oz, nx = nx, ny = ny, nz = nz };
            Check(Native.r3d_set_grid(H, in g), "set_grid");
        }

        public void SetParams(double cellMm, double wTurn, double wClear, int clearanceRadius, int clearanceConnectivity)
        {
            var p = new Native.R3dParams
            {
                cell_mm = cellMm, w_turn = wTurn, w_clear = wClear,
                clearance_radius = clearanceRadius, clearance_connectivity = clearanceConnectivity
            };
            Check(Native.r3d_set_params(H, in p), "set_params");
        }

        public void AddObstacle(double minx, double miny, double minz, double maxx, double maxy, double maxz)
            => Check(Native.r3d_add_obstacle(H, minx, miny, minz, maxx, maxy, maxz), "add_obstacle");

        public int AddTask(double sx, double sy, double sz, double gx, double gy, double gz,
                           string? utility, string? utilityGroup)
        {
            int idx = Native.r3d_add_task(H, sx, sy, sz, gx, gy, gz, Native.Utf8(utility), Native.Utf8(utilityGroup));
            if (idx < 0) throw new InvalidOperationException("r3d_add_task 실패");
            return idx;
        }

        public void SetTaskEndpoints(int task, double sx, double sy, double sz, double gx, double gy, double gz)
            => Check(Native.r3d_set_task_endpoints(H, task, sx, sy, sz, gx, gy, gz), "set_task_endpoints");

        // ---- 라우팅 ----
        public void RouteMulti(string priority = "longest")
            => Check(Native.r3d_route_multi(H, Native.Utf8(priority)), "route_multi");

        // ---- 결과 조회 ----
        public RouteResult GetResult(int task)
        {
            Check(Native.r3d_get_result(H, task, out var r), "get_result");
            var path = Array.Empty<PathCell>();
            if (r.path_len > 0)
            {
                var buf = new int[r.path_len * 3];
                int n = Native.r3d_copy_path(H, task, buf, r.path_len);
                path = new PathCell[n];
                for (int i = 0; i < n; i++) path[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            }
            return new RouteResult
            {
                Success = r.success != 0,
                LengthMm = r.length_mm,
                CostMm = r.cost_mm,
                Turns = r.turns,
                ExpandedNodes = r.expanded_nodes,
                Path = path
            };
        }

        public string DumpSceneText()
        {
            Check(Native.r3d_dump_scene_text(H, out var p), "dump_scene_text");
            return Native.TakeString(p);
        }

        private IntPtr H => _h.DangerousGetHandle();

        private static void Check(int status, string op)
        {
            if (status != 0) throw new InvalidOperationException($"r3d_{op} 실패 (status {status})");
        }

        public void Dispose() => _h.Dispose();
    }
}

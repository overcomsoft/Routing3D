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

    /// <summary>한 작업의 라우팅 결과(성공/길이/회전/경로/방문 셀).</summary>
    public sealed class RouteResult
    {
        public bool Success { get; init; }
        public double LengthMm { get; init; }
        public double CostMm { get; init; }
        public int Turns { get; init; }
        public long ExpandedNodes { get; init; }
        public PathCell[] Path { get; init; } = Array.Empty<PathCell>();
        /// <summary>이 작업의 A* 가 확장한 셀(가시화 '방문맵'). 엔진의 collect_visited 가 ON 일 때만.</summary>
        public PathCell[] Visited { get; init; } = Array.Empty<PathCell>();
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

        // 통과(pass-through) 객체 추가 — 경로탐색 충돌 제외, '통과 점유맵' 가시화용.
        public void AddPassthrough(double minx, double miny, double minz, double maxx, double maxy, double maxz)
            => Check(Native.r3d_add_passthrough(H, minx, miny, minz, maxx, maxy, maxz), "add_passthrough");

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

        // 단일 작업 재라우팅(원본 장애물 기준, 다른 배관 무시). 결과는 엔진에 저장된다.
        public RouteResult RouteTask(int task)
        {
            Check(Native.r3d_route_task(H, task, out _), "route_task");
            return GetResult(task);
        }

        // 대형 장면용 계층 corridor 라우팅(Sparse + coarse→fine). 작업별 독립(충돌 회피 없음).
        public void RouteCorridor(int factor = 16, int radius = 2)
            => Check(Native.r3d_route_corridor(H, factor, radius), "route_corridor");

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
            var visited = Array.Empty<PathCell>();
            if (r.visited_len > 0)
            {
                var buf = new int[r.visited_len * 3];
                int n = Native.r3d_copy_visited(H, task, buf, r.visited_len);
                visited = new PathCell[n];
                for (int i = 0; i < n; i++) visited[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            }
            return new RouteResult
            {
                Success = r.success != 0,
                LengthMm = r.length_mm,
                CostMm = r.cost_mm,
                Turns = r.turns,
                ExpandedNodes = r.expanded_nodes,
                Path = path,
                Visited = visited,
            };
        }

        /// <summary>'점유맵' 가시화 용 — 현재 doc 의 voxelize 된 블록 셀 전체를 한 번에 반환.</summary>
        public PathCell[] CopyBlocked()
        {
            int total = Native.r3d_copy_blocked(H, null!, 0);  // 사이즈 조회.
            if (total <= 0) return Array.Empty<PathCell>();
            var buf = new int[total * 3];
            int n = Native.r3d_copy_blocked(H, buf, total);
            var cells = new PathCell[n];
            for (int i = 0; i < n; i++) cells[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            return cells;
        }

        /// <summary>'통과 점유맵' 가시화 — doc.passthrough 의 voxelize 된 셀 전체 반환.</summary>
        public PathCell[] CopyPassthrough()
        {
            int total = Native.r3d_copy_passthrough(H, null!, 0);
            if (total <= 0) return Array.Empty<PathCell>();
            var buf = new int[total * 3];
            int n = Native.r3d_copy_passthrough(H, buf, total);
            var cells = new PathCell[n];
            for (int i = 0; i < n; i++) cells[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            return cells;
        }

        /// <summary>방문(확장) 셀 수집을 켜고/끄기. 기본 ON. OFF 면 라우팅 후 Visited 가 비어있다.</summary>
        public void SetCollectVisited(bool on)
            => Check(Native.r3d_set_collect_visited(H, on ? 1 : 0), "set_collect_visited");

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

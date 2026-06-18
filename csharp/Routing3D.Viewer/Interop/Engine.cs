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

    /// <summary>옥트리 리프 노드 정보 (3D 가시화용). State: 0=FREE, 1=BLOCKED.</summary>
    public readonly record struct OctreeLeaf(float X0Mm, float Y0Mm, float Z0Mm, float SizeMm, int State);

    /// <summary>라우팅 실패 사유(A1) — 엔진 RouteFail 과 1:1. Success=false 일 때만 의미.</summary>
    public enum RouteFail { None = 0, StartBlocked = 1, GoalBlocked = 2, CorridorMiss = 3, ExpansionLimit = 4, GoalDirBlocked = 5, NoPath = 6 }

    /// <summary>한 작업의 라우팅 결과(성공/길이/회전/경로/방문 셀).</summary>
    public sealed class RouteResult
    {
        public bool Success { get; init; }
        public double LengthMm { get; init; }
        public double CostMm { get; init; }
        public int Turns { get; init; }
        public long ExpandedNodes { get; init; }
        /// <summary>실패 사유(A1). 성공 시 None. UI 실패 진단(ExplainFailure)에서 정확 분류.</summary>
        public RouteFail Fail { get; init; }
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

        public void SetParams(double cellMm, double wTurn, double wClear, int clearanceRadius, int clearanceConnectivity,
                              double wCorridor = 0.0, int corridorRadius = 1, int[]? rackLevels = null,
                              double wHeur = 1.0, double wHeurNear = 0.0)
        {
            var rack = new int[8];
            int rackCount = 0;
            if (rackLevels != null)
            {
                rackCount = Math.Min(rackLevels.Length, 8);
                Array.Copy(rackLevels, rack, rackCount);
            }
            var p = new Native.R3dParams
            {
                cell_mm = cellMm, w_turn = wTurn, w_clear = wClear, w_corridor = wCorridor, w_heur = wHeur,
                w_heur_near = wHeurNear,
                clearance_radius = clearanceRadius, clearance_connectivity = clearanceConnectivity,
                corridor_radius = corridorRadius, rack_level_count = rackCount, rack_levels = rack
            };
            Check(Native.r3d_set_params(H, in p), "set_params");
        }

        /// <summary>배관 점유 팽창 반경(셀) 설정 — 배관-배관 충돌 회피(옵션1). route_multi(_progress) 가
        /// 깔린 배관을 경로 ±radius 6-이웃까지 점유로 막아 다음 배관 중심선을 띄운다(실제 관경 렌더 시 표면
        /// 겹침 방지). 0=기존 동작(경로 셀만). 관경/셀 기반 산출은 호출자(BuildEngineForRows)가 수행한다.</summary>
        public void SetPipeRadius(int radiusCells)
            => Check(Native.r3d_set_pipe_radius(H, radiusCells), "set_pipe_radius");

        /// <summary>per-task 관경 반경(B1) 활성화 — ON 이면 route_multi 가 각 배관 diameter_mm 로 마킹 반경을
        /// 자동 산출(글로벌 SetPipeRadius 책임 제거·가는 배관 과패킹 해소). 관경 미상은 글로벌 폴백.</summary>
        public void SetPerTaskRadius(bool on)
            => Check(Native.r3d_set_per_task_radius(H, on ? 1 : 0), "set_per_task_radius");

        /// <summary>C1 negotiated-congestion(CBS-lite, Phase C) 깊이 — 0=OFF(평면 rip-up만·기존 동작).
        /// >0 이면 평면 rip-up 으로도 안 풀린 실패 배관을 blocker 의 blocker 까지 재귀 양보시켜 해소(무손실·
        /// 결정적). [0,3] 클램프(엔진). 고밀도 병목의 잔여 실패를 줄인다.</summary>
        public void SetCbsDepth(int depth)
            => Check(Native.r3d_set_cbs_depth(H, depth), "set_cbs_depth");

        /// <summary>C2 코너 최소반경 배수(Phase C) — 엘보 사이 직선(런) ≥ (mult × 관경) 보장(제작성).
        /// 경로(셀) 단계에서 충돌검사 하에 짧은 단관을 흡수(꺾임 비증가일 때만, 양 끝점 고정). 0=OFF(기존
        /// 동작). 권장 2.0. PathRectifier(렌더 레벨, 되돌림)와 달리 충돌 안전.</summary>
        public void SetMinStraight(double mult)
            => Check(Native.r3d_set_min_straight(H, mult), "set_min_straight");

        /// <summary>코너 최소직선(절대 mm, 하드 제약) — A* 가 '한 번 꺾인 뒤 이 길이만큼 직진하기 전엔 다시
        /// 꺾지 못하도록' 탐색 단계에서 강제한다. 엘보 간 모든 직선 구간(단관)이 이 길이 이상 보장됨(관경 무관·
        /// 전 배관, 목표 직전 마지막 접속 구간만 면제). SetMinStraight(관경 배수·후처리 흡수)와 달리 하드 보장.
        /// 0=OFF(기존 동작·골든 불변). 권장 100mm.</summary>
        public void SetMinStraightMm(double mm)
            => Check(Native.r3d_set_min_straight_mm(H, mm), "set_min_straight_mm");

        /// <summary>배관-배관 이격(mm) — 두 배관 센터선 거리 ≥ r1 + r2 + gap 보장(표면 사이 최소 gap mm 띄움).
        /// 기존 마킹은 센터선을 ~관경만큼만 띄워 표면이 맞닿았다(겹쳐 보임). gap>0 이면 route_multi 가 깔린
        /// 배관을 쌍 반경으로 막아 정확히 띄운다. 0=기존 동작. 규격 60mm.</summary>
        public void SetPipeGap(double gapMm)
            => Check(Native.r3d_set_pipe_gap(H, gapMm), "set_pipe_gap");

        /// <summary>대형 격자 최대 탐색 확장 수 — 0=환경변수/기본값(48M). 최단경로 모드 등에서 탐색 상한을
        /// 줄여 배관당 탐색 폭발(수분 동결)을 방지한다. large_threshold(기본 5M셀) 이하 격자는 무제한(-1)이므로
        /// 이 값은 대형 격자에서만 적용된다. 권장: 최단경로 8M, 특징점/기존설계 0(기본 48M).</summary>
        public void SetMaxExpansions(long maxExp)
        {
            var opt = new Native.R3dRuntimeOptions { max_expansions = maxExp };
            Check(Native.r3d_set_runtime_options(H, in opt), "set_runtime_options");
        }

        /// <summary>대형 격자 탐색 상한 + 계층(hier) escalation 임계(hier_probe)를 함께 설정.
        /// hierProbe 를 크게(=maxExp) 주면 어려운 배관도 '직접 가중 A*'로 먼저 충분히 탐색한 뒤에야 계층
        /// corridor 로 넘어간다 → 직접 A*가 찾는 짧은(≤w_heur배) 경로를 계층의 3~4× 우회보다 우선한다.
        /// 최단경로 모드처럼 '길이'가 중요한 경우에 쓴다(대신 어려운 배관은 더 오래 탐색).</summary>
        public void SetRuntimeLimits(long maxExp, long hierProbe)
        {
            var opt = new Native.R3dRuntimeOptions { max_expansions = maxExp, hier_probe = hierProbe };
            Check(Native.r3d_set_runtime_options(H, in opt), "set_runtime_options");
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

        /// <summary>작업 관경(mm) 설정 — 우선순위 "diameter"/"utility" 정렬에서 '굵은 배관 먼저' 키.
        /// 0=관경 무시(기존 거리 정렬과 동일). 굵은 배관이 최단 경로를 선점해 가는 배관이 우회·충돌하지 않게 한다.</summary>
        public void SetTaskDiameter(int task, double diameterMm)
            => Check(Native.r3d_set_task_diameter(H, task, diameterMm), "set_task_diameter");

        /// <summary>작업 목표 진입축 제약 — A* 가 목표(end)에 axis(0..5 = +x,-x,+y,-y,+z,-z) 방향으로 진입할
        /// 때만 도달 인정. 덕트 종단 스텁 리드인 축을 주면 배관이 스텁에 일직선 진입(군더더기 꺾임 제거).
        /// -1=무제약(기본). 제약으로 막히면 엔진이 무제약 1회 폴백(연결 우선).</summary>
        public void SetTaskGoalDir(int task, int axis)
            => Check(Native.r3d_set_task_goal_dir(H, task, axis), "set_task_goal_dir");

        // ---- 라우팅 ----
        public void RouteMulti(string priority = "longest")
            => Check(Native.r3d_route_multi(H, Native.Utf8(priority)), "route_multi");

        /// <summary>학습된 회랑 셀(ijk 삼중항 평탄 배열)을 설정한다(L2b 소프트 바이어스). w_corridor>0 일 때
        /// route_multi 가 이 셀들을 회랑 시드로 삼아 배관을 그 곁으로 유도. null/빈 배열이면 회랑을 비운다.</summary>
        public void SetCorridorCells(int[]? ijk)
            => Check(Native.r3d_set_corridor_cells(H, ijk, ijk == null ? 0 : ijk.Length / 3), "set_corridor_cells");

        /// <summary>라우팅 진행 이벤트. Phase 0=탐색 진행(Progress01), 1=배관 완료(지표+Path).</summary>
        public readonly record struct RouteProgress(int Phase, int OrderIndex, int TaskIndex,
            bool Success, double LengthMm, int Turns, long ExpandedNodes, double ElapsedMs,
            int Done, int Total, double Progress01, PathCell[] Path);

        /// <summary>route_multi 와 동일(순차·충돌없음)하되 배관마다 onPipe 를 호출(처리순서·진행율·경로 실시간).
        /// 콜백은 라우팅 스레드에서 동기 호출되므로, UI 갱신은 호출자가 Dispatcher 로 마샬링한다.
        /// shouldCancel 이 true 를 반환하면 엔진이 현재 배관 탐색을 즉시 중단하고 남은 배관 없이 정상 반환한다
        /// (협력적 취소 — 약 5만 확장마다·배관 완료마다 검사). 완료된 배관 결과는 보존된다.</summary>
        public void RouteMultiProgress(string priority, Action<RouteProgress> onPipe, Func<bool>? shouldCancel = null)
        {
            // 델리게이트는 네이티브 호출이 끝날 때까지 살아 있어야 한다(지역 변수로 GC 보호).
            Native.R3dProgressFn cb = (user, phase, oi, ti, ok, len, turns, exp, ms, done, total, prog, pathPtr, pathLen) =>
            {
                var path = Array.Empty<PathCell>();
                if (phase == 1 && pathLen > 0 && pathPtr != IntPtr.Zero)
                {
                    var buf = new int[pathLen * 3];
                    System.Runtime.InteropServices.Marshal.Copy(pathPtr, buf, 0, buf.Length);  // 즉시 복사.
                    path = new PathCell[pathLen];
                    for (int i = 0; i < pathLen; i++) path[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
                }
                onPipe(new RouteProgress(phase, oi, ti, ok != 0, len, turns, exp, ms, done, total, prog, path));
                return (shouldCancel != null && shouldCancel()) ? 1 : 0;   // 0아님=취소 → 엔진 탐색 중단.
            };
            try { Check(Native.r3d_route_multi_progress(H, Native.Utf8(priority), cb, IntPtr.Zero), "route_multi_progress"); }
            finally { GC.KeepAlive(cb); }
        }

        // 단일 작업 재라우팅(원본 장애물 기준, 다른 배관 무시). 결과는 엔진에 저장된다.
        public RouteResult RouteTask(int task)
        {
            Check(Native.r3d_route_task(H, task, out _), "route_task");
            return GetResult(task);
        }

        // 대형 장면용 계층 corridor 라우팅(Sparse + coarse→fine). 작업별 독립(충돌 회피 없음).
        public void RouteCorridor(int factor = 16, int radius = 2)
            => Check(Native.r3d_route_corridor(H, factor, radius), "route_corridor");

        // 순차 계층 corridor(Sparse + astar_hashed, 셀 수 배열 미할당 → 10mm 등 대형/정밀 격자 안전).
        // priority 순서로 한 배관씩 라우팅하고 mark_pipe(pipeRadius)로 점유 추가 → 배관 간 충돌 0.
        public void RouteCorridorMulti(int factor, int radius, string priority = "longest", int pipeRadius = 0)
            => Check(Native.r3d_route_corridor_multi(H, factor, radius, Native.Utf8(priority), pipeRadius),
                     "route_corridor_multi");

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
                Fail = (RouteFail)r.fail_reason,
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

        /// <summary>옥트리 리프 노드 배열 반환 (3D 가시화용).
        /// maxLeaves: 상한(기본 1M). State: 0=FREE, 1=BLOCKED.</summary>
        public OctreeLeaf[] EnumOctreeLeaves(int maxLeaves = 1_000_000)
        {
            var buf = new Native.R3dOctreeLeaf[maxLeaves];
            int count;
            int st = Native.r3d_enum_octree_leaves(H, buf, maxLeaves, out count);
            if (st != 0 || count <= 0) return Array.Empty<OctreeLeaf>();
            var result = new OctreeLeaf[count];
            for (int i = 0; i < count; i++)
                result[i] = new OctreeLeaf(buf[i].X0Mm, buf[i].Y0Mm, buf[i].Z0Mm, buf[i].SizeMm, buf[i].State);
            return result;
        }

        private IntPtr H => _h.DangerousGetHandle();

        private static void Check(int status, string op)
        {
            if (status != 0) throw new InvalidOperationException($"r3d_{op} 실패 (status {status})");
        }

        public void Dispose() => _h.Dispose();
    }
}

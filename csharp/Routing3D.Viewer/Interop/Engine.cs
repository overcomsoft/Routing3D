// ë§¤ë‹ˆì§€???”ì§„ ?˜í¼ ??C# ì½”ë“œê°€ ?¤ì œë¡??¬ìš©?˜ëŠ” OOP ë©?
// =============================================================================
//   Native(P/Invoke) + R3dEngineHandle ?„ì— ?ˆì™¸ ê¸°ë°˜ OOP API ë¥??œê³µ?œë‹¤.
//   ?íƒœ ì½”ë“œê°€ 0(R3D_OK)???„ë‹ˆë©??ˆì™¸ë¥??˜ì§„??
// =============================================================================
using System;

namespace Routing3D.Viewer.Interop
{
    /// <summary>ê²½ë¡œ ?€ (i, j, k).</summary>
    public readonly record struct PathCell(int I, int J, int K);

    /// <summary>?¥íŠ¸ë¦?ë¦¬í”„ ?¸ë“œ ?•ë³´ (3D ê°€?œí™”??. State: 0=FREE, 1=BLOCKED.</summary>
    public readonly record struct OctreeLeaf(float X0Mm, float Y0Mm, float Z0Mm, float SizeMm, int State);

    /// <summary>?¼ìš°???¤íŒ¨ ?¬ìœ (A1) ???”ì§„ RouteFail ê³?1:1. Success=false ???Œë§Œ ?˜ë?.</summary>
    public enum RouteFail { None = 0, StartBlocked = 1, GoalBlocked = 2, CorridorMiss = 3, ExpansionLimit = 4, GoalDirBlocked = 5, NoPath = 6 }

    /// <summary>???‘ì—…???¼ìš°??ê²°ê³¼(?±ê³µ/ê¸¸ì´/?Œì „/ê²½ë¡œ/ë°©ë¬¸ ?€).</summary>
    public sealed class RouteResult
    {
        public bool Success { get; init; }
        public double LengthMm { get; init; }
        public double CostMm { get; init; }
        public int Turns { get; init; }
        public long ExpandedNodes { get; init; }
        /// <summary>?¤íŒ¨ ?¬ìœ (A1). ?±ê³µ ??None. UI ?¤íŒ¨ ì§„ë‹¨(ExplainFailure)?ì„œ ?•í™• ë¶„ë¥˜.</summary>
        public RouteFail Fail { get; init; }
        public PathCell[] Path { get; init; } = Array.Empty<PathCell>();
        /// <summary>???‘ì—…??A* ê°€ ?•ì¥???€(ê°€?œí™” 'ë°©ë¬¸ë§?). ?”ì§„??collect_visited ê°€ ON ???Œë§Œ.</summary>
        public PathCell[] Visited { get; init; } = Array.Empty<PathCell>();
    }

    public sealed class Engine : IDisposable
    {
        private readonly R3dEngineHandle _h = R3dEngineHandle.Create();
        private bool _disposed;

        public bool IsValid => !_disposed && !_h.IsInvalid;
        public static string Version => Native.VersionString();

        // ---- ?¥ë©´ êµ¬ì„±(Level 2) ----
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

        /// <summary>ë°°ê? ?ìœ  ?½ì°½ ë°˜ê²½(?€) ?¤ì • ??ë°°ê?-ë°°ê? ì¶©ëŒ ?Œí”¼(?µì…˜1). route_multi(_progress) ê°€
        /// ê¹”ë¦° ë°°ê???ê²½ë¡œ Â±radius 6-?´ì›ƒê¹Œì? ?ìœ ë¡?ë§‰ì•„ ?¤ìŒ ë°°ê? ì¤‘ì‹¬? ì„ ?„ìš´???¤ì œ ê´€ê²??Œë” ???œë©´
        /// ê²¹ì¹¨ ë°©ì?). 0=ê¸°ì¡´ ?™ì‘(ê²½ë¡œ ?€ë§?. ê´€ê²??€ ê¸°ë°˜ ?°ì¶œ?€ ?¸ì¶œ??BuildEngineForRows)ê°€ ?˜í–‰?œë‹¤.</summary>
        public void SetPipeRadius(int radiusCells)
            => Check(Native.r3d_set_pipe_radius(H, radiusCells), "set_pipe_radius");

        /// <summary>per-task ê´€ê²?ë°˜ê²½(B1) ?œì„±????ON ?´ë©´ route_multi ê°€ ê°?ë°°ê? diameter_mm ë¡?ë§ˆí‚¹ ë°˜ê²½??
        /// ?ë™ ?°ì¶œ(ê¸€ë¡œë²Œ SetPipeRadius ì±…ì„ ?œê±°Â·ê°€??ë°°ê? ê³¼íŒ¨???´ì†Œ). ê´€ê²?ë¯¸ìƒ?€ ê¸€ë¡œë²Œ ?´ë°±.</summary>
        public void SetPerTaskRadius(bool on)
            => Check(Native.r3d_set_per_task_radius(H, on ? 1 : 0), "set_per_task_radius");

        /// <summary>C1 negotiated-congestion(CBS-lite, Phase C) ê¹Šì´ ??0=OFF(?‰ë©´ rip-upë§ŒÂ·ê¸°ì¡??™ì‘).
        /// >0 ?´ë©´ ?‰ë©´ rip-up ?¼ë¡œ?????€ë¦??¤íŒ¨ ë°°ê???blocker ??blocker ê¹Œì? ?¬ê? ?‘ë³´?œì¼œ ?´ì†Œ(ë¬´ì†?¤Â?
        /// ê²°ì •??. [0,3] ?´ë¨???”ì§„). ê³ ë???ë³‘ëª©???”ì—¬ ?¤íŒ¨ë¥?ì¤„ì¸??</summary>
        public void SetCbsDepth(int depth)
            => Check(Native.r3d_set_cbs_depth(H, depth), "set_cbs_depth");

        /// <summary>C2 ì½”ë„ˆ ìµœì†Œë°˜ê²½ ë°°ìˆ˜(Phase C) ???˜ë³´ ?¬ì´ ì§ì„ (?? ??(mult Ã— ê´€ê²? ë³´ì¥(?œì‘??.
        /// ê²½ë¡œ(?€) ?¨ê³„?ì„œ ì¶©ëŒê²€???˜ì— ì§§ì? ?¨ê????¡ìˆ˜(êº¾ì„ ë¹„ì¦ê°€???Œë§Œ, ???ì  ê³ ì •). 0=OFF(ê¸°ì¡´
        /// ?™ì‘). ê¶Œì¥ 2.0. PathRectifier(?Œë” ?ˆë²¨, ?˜ëŒë¦??€ ?¬ë¦¬ ì¶©ëŒ ?ˆì „.</summary>
        public void SetMinStraight(double mult)
            => Check(Native.r3d_set_min_straight(H, mult), "set_min_straight");

        /// <summary>ì½”ë„ˆ ìµœì†Œì§ì„ (?ˆë? mm, ?˜ë“œ ?œì•½) ??A* ê°€ '??ë²?êº¾ì¸ ????ê¸¸ì´ë§Œí¼ ì§ì§„?˜ê¸° ?„ì—” ?¤ì‹œ
        /// êº¾ì? ëª»í•˜?„ë¡' ?ìƒ‰ ?¨ê³„?ì„œ ê°•ì œ?œë‹¤. ?˜ë³´ ê°?ëª¨ë“  ì§ì„  êµ¬ê°„(?¨ê?)????ê¸¸ì´ ?´ìƒ ë³´ì¥??ê´€ê²?ë¬´ê?Â·
        /// ??ë°°ê?, ëª©í‘œ ì§ì „ ë§ˆì?ë§??‘ì† êµ¬ê°„ë§?ë©´ì œ). SetMinStraight(ê´€ê²?ë°°ìˆ˜Â·?„ì²˜ë¦??¡ìˆ˜)?€ ?¬ë¦¬ ?˜ë“œ ë³´ì¥.
        /// 0=OFF(ê¸°ì¡´ ?™ì‘Â·ê³¨ë“  ë¶ˆë?). ê¶Œì¥ 100mm.</summary>
        public void SetMinStraightMm(double mm)
            => Check(Native.r3d_set_min_straight_mm(H, mm), "set_min_straight_mm");

        /// <summary>ë°°ê?-ë°°ê? ?´ê²©(mm) ????ë°°ê? ?¼í„°??ê±°ë¦¬ ??r1 + r2 + gap ë³´ì¥(?œë©´ ?¬ì´ ìµœì†Œ gap mm ?„ì?).
        /// ê¸°ì¡´ ë§ˆí‚¹?€ ?¼í„°? ì„ ~ê´€ê²½ë§Œ?¼ë§Œ ?„ì›Œ ?œë©´??ë§ë‹¿?˜ë‹¤(ê²¹ì³ ë³´ì„). gap>0 ?´ë©´ route_multi ê°€ ê¹”ë¦°
        /// ë°°ê?????ë°˜ê²½?¼ë¡œ ë§‰ì•„ ?•í™•???„ìš´?? 0=ê¸°ì¡´ ?™ì‘. ê·œê²© 60mm.</summary>
        public void SetPipeGap(double gapMm)
            => Check(Native.r3d_set_pipe_gap(H, gapMm), "set_pipe_gap");

        /// <summary>Segment A*: straight-run expansion first, existing A* fallback. maxSegmentCells controls jump length.</summary>
        public void SetSegmentAstar(bool on, int maxSegmentCells = 64)
            => Check(Native.r3d_set_segment_astar(H, on ? 1 : 0, maxSegmentCells), "set_segment_astar");

        /// <summary>Octree macro path guide: Octree Jump A* generates a corridor, fine A* produces the final route.</summary>
        public void SetOctreeGuide(bool on, int corridorRadius = 2)
            => Check(Native.r3d_set_octree_guide(H, on ? 1 : 0, corridorRadius), "set_octree_guide");

        /// <summary>TruckIn/Middle/Terminal split routing. trunkZMm <= 0 selects rack/auto trunk height.</summary>
        public void SetRouteSplit(bool on, double trunkZMm = 0.0)
            => Check(Native.r3d_set_route_split(H, on ? 1 : 0, trunkZMm), "set_route_split");

        /// <summary>?€??ê²©ì ìµœë? ?ìƒ‰ ?•ì¥ ????0=?˜ê²½ë³€??ê¸°ë³¸ê°?48M). ìµœë‹¨ê²½ë¡œ ëª¨ë“œ ?±ì—???ìƒ‰ ?í•œ??
        /// ì¤„ì—¬ ë°°ê????ìƒ‰ ??°œ(?˜ë¶„ ?™ê²°)??ë°©ì??œë‹¤. large_threshold(ê¸°ë³¸ 5M?€) ?´í•˜ ê²©ì??ë¬´ì œ??-1)?´ë?ë¡?
        /// ??ê°’ì? ?€??ê²©ì?ì„œë§??ìš©?œë‹¤. ê¶Œì¥: ìµœë‹¨ê²½ë¡œ 8M, ?¹ì§•??ê¸°ì¡´?¤ê³„ 0(ê¸°ë³¸ 48M).</summary>
        public void SetMaxExpansions(long maxExp)
        {
            var opt = new Native.R3dRuntimeOptions { max_expansions = maxExp };
            Check(Native.r3d_set_runtime_options(H, in opt), "set_runtime_options");
        }

        /// <summary>?€??ê²©ì ?ìƒ‰ ?í•œ + ê³„ì¸µ(hier) escalation ?„ê³„(hier_probe)ë¥??¨ê»˜ ?¤ì •.
        /// hierProbe ë¥??¬ê²Œ(=maxExp) ì£¼ë©´ ?´ë ¤??ë°°ê???'ì§ì ‘ ê°€ì¤?A*'ë¡?ë¨¼ì? ì¶©ë¶„???ìƒ‰???¤ì—??ê³„ì¸µ
        /// corridor ë¡??˜ì–´ê°„ë‹¤ ??ì§ì ‘ A*ê°€ ì°¾ëŠ” ì§§ì?(?¤w_heurë°? ê²½ë¡œë¥?ê³„ì¸µ??3~4Ã— ?°íšŒë³´ë‹¤ ?°ì„ ?œë‹¤.
        /// ìµœë‹¨ê²½ë¡œ ëª¨ë“œì²˜ëŸ¼ 'ê¸¸ì´'ê°€ ì¤‘ìš”??ê²½ìš°???´ë‹¤(?€???´ë ¤??ë°°ê??€ ???¤ë˜ ?ìƒ‰).</summary>
        public void SetRuntimeLimits(long maxExp, long hierProbe)
        {
            var opt = new Native.R3dRuntimeOptions { max_expansions = maxExp, hier_probe = hierProbe };
            Check(Native.r3d_set_runtime_options(H, in opt), "set_runtime_options");
        }

        public void SetTrace(string path, int level = 1, int sampleEvery = 1000,
                             bool includeOccupancy = true, bool includeRejects = false,
                             bool includePostprocess = true, int maxEventsPerTask = 20000)
        {
            if (string.IsNullOrWhiteSpace(path)) throw new ArgumentException("trace path is empty", nameof(path));
            Check(Native.r3d_set_trace_file(H, Native.Utf8(path)), "set_trace_file");
            var opt = new Native.R3dTraceOptions
            {
                enabled = 1,
                level = level,
                sample_every = sampleEvery,
                include_occupancy = includeOccupancy ? 1 : 0,
                include_rejects = includeRejects ? 1 : 0,
                include_postprocess = includePostprocess ? 1 : 0,
                max_events_per_task = maxEventsPerTask,
            };
            Check(Native.r3d_set_trace_options(H, in opt), "set_trace_options");
        }

        public void DisableTrace()
        {
            var opt = new Native.R3dTraceOptions { enabled = 0 };
            Check(Native.r3d_set_trace_options(H, in opt), "set_trace_options");
        }

        public void FlushTrace()
            => Check(Native.r3d_flush_trace(H), "flush_trace");

        public void AddObstacle(double minx, double miny, double minz, double maxx, double maxy, double maxz)
            => Check(Native.r3d_add_obstacle(H, minx, miny, minz, maxx, maxy, maxz), "add_obstacle");

        // ?µê³¼(pass-through) ê°ì²´ ì¶”ê? ??ê²½ë¡œ?ìƒ‰ ì¶©ëŒ ?œì™¸, '?µê³¼ ?ìœ ë§? ê°€?œí™”??
        public void AddPassthrough(double minx, double miny, double minz, double maxx, double maxy, double maxz)
            => Check(Native.r3d_add_passthrough(H, minx, miny, minz, maxx, maxy, maxz), "add_passthrough");

        public int AddTask(double sx, double sy, double sz, double gx, double gy, double gz,
                           string? utility, string? utilityGroup)
        {
            int idx = Native.r3d_add_task(H, sx, sy, sz, gx, gy, gz, Native.Utf8(utility), Native.Utf8(utilityGroup));
            if (idx < 0) throw new InvalidOperationException("r3d_add_task ?¤íŒ¨");
            return idx;
        }

        public void SetTaskEndpoints(int task, double sx, double sy, double sz, double gx, double gy, double gz)
            => Check(Native.r3d_set_task_endpoints(H, task, sx, sy, sz, gx, gy, gz), "set_task_endpoints");

        /// <summary>?‘ì—… ê´€ê²?mm) ?¤ì • ???°ì„ ?œìœ„ "diameter"/"utility" ?•ë ¬?ì„œ 'êµµì? ë°°ê? ë¨¼ì?' ??
        /// 0=ê´€ê²?ë¬´ì‹œ(ê¸°ì¡´ ê±°ë¦¬ ?•ë ¬ê³??™ì¼). êµµì? ë°°ê???ìµœë‹¨ ê²½ë¡œë¥?? ì ??ê°€??ë°°ê????°íšŒÂ·ì¶©ëŒ?˜ì? ?Šê²Œ ?œë‹¤.</summary>
        public void SetTaskDiameter(int task, double diameterMm)
            => Check(Native.r3d_set_task_diameter(H, task, diameterMm), "set_task_diameter");

        /// <summary>?‘ì—… ëª©í‘œ ì§„ì…ì¶??œì•½ ??A* ê°€ ëª©í‘œ(end)??axis(0..5 = +x,-x,+y,-y,+z,-z) ë°©í–¥?¼ë¡œ ì§„ì…??
        /// ?Œë§Œ ?„ë‹¬ ?¸ì •. ?•íŠ¸ ì¢…ë‹¨ ?¤í… ë¦¬ë“œ??ì¶•ì„ ì£¼ë©´ ë°°ê????¤í…???¼ì§??ì§„ì…(êµ°ë”?”ê¸° êº¾ì„ ?œê±°).
        /// -1=ë¬´ì œ??ê¸°ë³¸). ?œì•½?¼ë¡œ ë§‰íˆë©??”ì§„??ë¬´ì œ??1???´ë°±(?°ê²° ?°ì„ ).</summary>
        public void SetTaskGoalDir(int task, int axis)
            => Check(Native.r3d_set_task_goal_dir(H, task, axis), "set_task_goal_dir");

        // ---- ?¼ìš°??----
        public void RouteMulti(string priority = "longest")
            => Check(Native.r3d_route_multi(H, Native.Utf8(priority)), "route_multi");

        /// <summary>?™ìŠµ???Œë‘ ?€(ijk ?¼ì¤‘???‰íƒ„ ë°°ì—´)???¤ì •?œë‹¤(L2b ?Œí”„??ë°”ì´?´ìŠ¤). w_corridor>0 ????
        /// route_multi ê°€ ???€?¤ì„ ?Œë‘ ?œë“œë¡??¼ì•„ ë°°ê???ê·?ê³ìœ¼ë¡?? ë„. null/ë¹?ë°°ì—´?´ë©´ ?Œë‘??ë¹„ìš´??</summary>
        public void SetCorridorCells(int[]? ijk)
            => Check(Native.r3d_set_corridor_cells(H, ijk, ijk == null ? 0 : ijk.Length / 3), "set_corridor_cells");

        /// <summary>?¼ìš°??ì§„í–‰ ?´ë²¤?? Phase 0=?ìƒ‰ ì§„í–‰(Progress01), 1=ë°°ê? ?„ë£Œ(ì§€??Path).</summary>
        public readonly record struct RouteProgress(int Phase, int OrderIndex, int TaskIndex,
            bool Success, double LengthMm, int Turns, long ExpandedNodes, double ElapsedMs,
            int Done, int Total, double Progress01, PathCell[] Path);

        /// <summary>route_multi ?€ ?™ì¼(?œì°¨Â·ì¶©ëŒ?†ìŒ)?˜ë˜ ë°°ê?ë§ˆë‹¤ onPipe ë¥??¸ì¶œ(ì²˜ë¦¬?œì„œÂ·ì§„í–‰?¨Â·ê²½ë¡??¤ì‹œê°?.
        /// ì½œë°±?€ ?¼ìš°???¤ë ˆ?œì—???™ê¸° ?¸ì¶œ?˜ë?ë¡? UI ê°±ì‹ ?€ ?¸ì¶œ?ê? Dispatcher ë¡?ë§ˆìƒ¬ë§í•œ??
        /// shouldCancel ??true ë¥?ë°˜í™˜?˜ë©´ ?”ì§„???„ì¬ ë°°ê? ?ìƒ‰??ì¦‰ì‹œ ì¤‘ë‹¨?˜ê³  ?¨ì? ë°°ê? ?†ì´ ?•ìƒ ë°˜í™˜?œë‹¤
        /// (?‘ë ¥??ì·¨ì†Œ ????5ë§??•ì¥ë§ˆë‹¤Â·ë°°ê? ?„ë£Œë§ˆë‹¤ ê²€??. ?„ë£Œ??ë°°ê? ê²°ê³¼??ë³´ì¡´?œë‹¤.</summary>
        public void RouteMultiProgress(string priority, Action<RouteProgress> onPipe, Func<bool>? shouldCancel = null)
        {
            // ?¸ë¦¬ê²Œì´?¸ëŠ” ?¤ì´?°ë¸Œ ?¸ì¶œ???ë‚  ?Œê¹Œì§€ ?´ì•„ ?ˆì–´???œë‹¤(ì§€??ë³€?˜ë¡œ GC ë³´í˜¸).
            Native.R3dProgressFn cb = (user, phase, oi, ti, ok, len, turns, exp, ms, done, total, prog, pathPtr, pathLen) =>
            {
                var path = Array.Empty<PathCell>();
                if (phase == 1 && pathLen > 0 && pathPtr != IntPtr.Zero)
                {
                    var buf = new int[pathLen * 3];
                    System.Runtime.InteropServices.Marshal.Copy(pathPtr, buf, 0, buf.Length);  // ì¦‰ì‹œ ë³µì‚¬.
                    path = new PathCell[pathLen];
                    for (int i = 0; i < pathLen; i++) path[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
                }
                onPipe(new RouteProgress(phase, oi, ti, ok != 0, len, turns, exp, ms, done, total, prog, path));
                return (shouldCancel != null && shouldCancel()) ? 1 : 0;   // 0?„ë‹˜=ì·¨ì†Œ ???”ì§„ ?ìƒ‰ ì¤‘ë‹¨.
            };
            try { Check(Native.r3d_route_multi_progress(H, Native.Utf8(priority), cb, IntPtr.Zero), "route_multi_progress"); }
            finally { GC.KeepAlive(cb); }
        }

        // ?¨ì¼ ?‘ì—… ?¬ë¼?°íŒ…(?ë³¸ ?¥ì• ë¬?ê¸°ì?, ?¤ë¥¸ ë°°ê? ë¬´ì‹œ). ê²°ê³¼???”ì§„???€?¥ëœ??
        public RouteResult RouteTask(int task)
        {
            Check(Native.r3d_route_task(H, task, out _), "route_task");
            return GetResult(task);
        }

        public RouteResult RouteTaskAnytime(int task, double initialWeight, double finalWeight,
                                           double weightStep, double timeBudgetMs,
                                           long maxExpansions, int goalDir,
                                           out int iterations, out int improvements)
        {
            Check(Native.r3d_route_task_anytime(H, task, initialWeight, finalWeight, weightStep,
                                                timeBudgetMs, maxExpansions, goalDir, out _,
                                                out iterations, out improvements),
                  "route_task_anytime");
            return GetResult(task);
        }
        // ?€???¥ë©´??ê³„ì¸µ corridor ?¼ìš°??Sparse + coarse?’fine). ?‘ì—…ë³??…ë¦½(ì¶©ëŒ ?Œí”¼ ?†ìŒ).
        public void RouteCorridor(int factor = 16, int radius = 2)
            => Check(Native.r3d_route_corridor(H, factor, radius), "route_corridor");

        // ?œì°¨ ê³„ì¸µ corridor(Sparse + astar_hashed, ?€ ??ë°°ì—´ ë¯¸í• ????10mm ???€???•ë? ê²©ì ?ˆì „).
        // priority ?œì„œë¡???ë°°ê????¼ìš°?…í•˜ê³?mark_pipe(pipeRadius)ë¡??ìœ  ì¶”ê? ??ë°°ê? ê°?ì¶©ëŒ 0.
        public void RouteCorridorMulti(int factor, int radius, string priority = "longest", int pipeRadius = 0)
            => Check(Native.r3d_route_corridor_multi(H, factor, radius, Native.Utf8(priority), pipeRadius),
                     "route_corridor_multi");

        // ---- ê²°ê³¼ ì¡°íšŒ ----
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

        /// <summary>'?ìœ ë§? ê°€?œí™” ?????„ì¬ doc ??voxelize ??ë¸”ë¡ ?€ ?„ì²´ë¥???ë²ˆì— ë°˜í™˜.</summary>
        public PathCell[] CopyBlocked()
        {
            int total = Native.r3d_copy_blocked(H, null, 0);  // ?¬ì´ì¦?ì¡°íšŒ.
            if (total <= 0) return Array.Empty<PathCell>();
            var buf = new int[total * 3];
            int n = Native.r3d_copy_blocked(H, buf, total);
            var cells = new PathCell[n];
            for (int i = 0; i < n; i++) cells[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            return cells;
        }

        /// <summary>?ìœ  ?€???”ì§„?ì„œ ì§ì ‘ ê· ë“± ?˜í”Œë§í•´ ë³µì‚¬?œë‹¤. ?€???¥ë©´ ë¯¸ë¦¬ë³´ê¸°??</summary>
        public PathCell[] CopyBlockedSampled(int maxCells)
        {
            if (maxCells <= 0) return Array.Empty<PathCell>();
            var buf = new int[maxCells * 3];
            int n = Native.r3d_copy_blocked_sampled(H, maxCells, buf);
            if (n <= 0) return Array.Empty<PathCell>();
            var cells = new PathCell[n];
            for (int i = 0; i < n; i++) cells[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            return cells;
        }
        /// <summary>'?µê³¼ ?ìœ ë§? ê°€?œí™” ??doc.passthrough ??voxelize ???€ ?„ì²´ ë°˜í™˜.</summary>
        public PathCell[] CopyPassthrough()
        {
            int total = Native.r3d_copy_passthrough(H, null, 0);
            if (total <= 0) return Array.Empty<PathCell>();
            var buf = new int[total * 3];
            int n = Native.r3d_copy_passthrough(H, buf, total);
            var cells = new PathCell[n];
            for (int i = 0; i < n; i++) cells[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            return cells;
        }

        /// <summary>ë°©ë¬¸(?•ì¥) ?€ ?˜ì§‘??ì¼œê³ /?„ê¸°. ê¸°ë³¸ ON. OFF ë©??¼ìš°????Visited ê°€ ë¹„ì–´?ˆë‹¤.</summary>
        public void SetCollectVisited(bool on)
            => Check(Native.r3d_set_collect_visited(H, on ? 1 : 0), "set_collect_visited");

        public string DumpSceneText()
        {
            Check(Native.r3d_dump_scene_text(H, out var p), "dump_scene_text");
            return Native.TakeString(p);
        }

        /// <summary>?¥íŠ¸ë¦?ë¦¬í”„ ?¸ë“œ ë°°ì—´ ë°˜í™˜ (3D ê°€?œí™”??.
        /// maxLeaves: ?í•œ(ê¸°ë³¸ 1M). State: 0=FREE, 1=BLOCKED.</summary>
        public OctreeLeaf[] EnumOctreeLeaves(int maxLeaves = 1_000_000)
        {
            if (maxLeaves <= 0) return Array.Empty<OctreeLeaf>();
            int total;
            int st = Native.r3d_enum_octree_leaves(H, null, 0, out total);
            if (st != 0 || total <= 0) return Array.Empty<OctreeLeaf>();
            int take = Math.Min(total, maxLeaves);
            var buf = new Native.R3dOctreeLeaf[take];
            st = Native.r3d_enum_octree_leaves(H, buf, take, out _);
            if (st != 0) return Array.Empty<OctreeLeaf>();
            var result = new OctreeLeaf[take];
            for (int i = 0; i < take; i++)
                result[i] = new OctreeLeaf(buf[i].X0Mm, buf[i].Y0Mm, buf[i].Z0Mm, buf[i].SizeMm, buf[i].State);
            return result;
        }

        private IntPtr H
        {
            get
            {
                ObjectDisposedException.ThrowIf(_disposed, this);
                return _h.DangerousGetHandle();
            }
        }

        private static void Check(int status, string op)
        {
            if (status != 0) throw new InvalidOperationException($"r3d_{op} ?¤íŒ¨ (status {status})");
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _h.Dispose();
        }
    }
}

// 留ㅻ땲吏???붿쭊 ?섑띁 ??C# 肄붾뱶媛 ?ㅼ젣濡??ъ슜?섎뒗 OOP 硫?
// =============================================================================
//   Native(P/Invoke) + R3dEngineHandle ?꾩뿉 ?덉쇅 湲곕컲 OOP API 瑜??쒓났?쒕떎.
//   ?곹깭 肄붾뱶媛 0(R3D_OK)???꾨땲硫??덉쇅瑜??섏쭊??
// =============================================================================
using System;

namespace Routing3D.Viewer.Interop
{
    /// <summary>寃쎈줈 ? (i, j, k).</summary>
    public readonly record struct PathCell(int I, int J, int K);

    /// <summary>?ν듃由?由ы봽 ?몃뱶 ?뺣낫 (3D 媛?쒗솕??. State: 0=FREE, 1=BLOCKED.</summary>
    public readonly record struct OctreeLeaf(float X0Mm, float Y0Mm, float Z0Mm, float SizeMm, int State);

    /// <summary>?쇱슦???ㅽ뙣 ?ъ쑀(A1) ???붿쭊 RouteFail 怨?1:1. Success=false ???뚮쭔 ?섎?.</summary>
    public enum RouteFail { None = 0, StartBlocked = 1, GoalBlocked = 2, CorridorMiss = 3, ExpansionLimit = 4, GoalDirBlocked = 5, NoPath = 6 }

    /// <summary>???묒뾽???쇱슦??寃곌낵(?깃났/湲몄씠/?뚯쟾/寃쎈줈/諛⑸Ц ?).</summary>
    public sealed class RouteResult
    {
        public bool Success { get; init; }
        public double LengthMm { get; init; }
        public double CostMm { get; init; }
        public int Turns { get; init; }
        public long ExpandedNodes { get; init; }
        public double ElapsedMs { get; init; }
        /// <summary>?ㅽ뙣 ?ъ쑀(A1). ?깃났 ??None. UI ?ㅽ뙣 吏꾨떒(ExplainFailure)?먯꽌 ?뺥솗 遺꾨쪟.</summary>
        public RouteFail Fail { get; init; }
        public PathCell[] Path { get; init; } = Array.Empty<PathCell>();
        /// <summary>???묒뾽??A* 媛 ?뺤옣???(媛?쒗솕 '諛⑸Ц留?). ?붿쭊??collect_visited 媛 ON ???뚮쭔.</summary>
        public PathCell[] Visited { get; init; } = Array.Empty<PathCell>();
    }

    public sealed class Engine : IDisposable
    {
        private readonly R3dEngineHandle _h = R3dEngineHandle.Create();
        private bool _disposed;

        public bool IsValid => !_disposed && !_h.IsInvalid;
        public static string Version => Native.VersionString();

        // ---- ?λ㈃ 援ъ꽦(Level 2) ----
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

        /// <summary>諛곌? ?먯쑀 ?쎌갹 諛섍꼍(?) ?ㅼ젙 ??諛곌?-諛곌? 異⑸룎 ?뚰뵾(?듭뀡1). route_multi(_progress) 媛
        /// 源붾┛ 諛곌???寃쎈줈 짹radius 6-?댁썐源뚯? ?먯쑀濡?留됱븘 ?ㅼ쓬 諛곌? 以묒떖?좎쓣 ?꾩슫???ㅼ젣 愿寃??뚮뜑 ???쒕㈃
        /// 寃뱀묠 諛⑹?). 0=湲곗〈 ?숈옉(寃쎈줈 ?留?. 愿寃?? 湲곕컲 ?곗텧? ?몄텧??BuildEngineForRows)媛 ?섑뻾?쒕떎.</summary>
        public void SetPipeRadius(int radiusCells)
            => Check(Native.r3d_set_pipe_radius(H, radiusCells), "set_pipe_radius");

        /// <summary>per-task 愿寃?諛섍꼍(B1) ?쒖꽦????ON ?대㈃ route_multi 媛 媛?諛곌? diameter_mm 濡?留덊궧 諛섍꼍??
        /// ?먮룞 ?곗텧(湲濡쒕쾶 SetPipeRadius 梨낆엫 ?쒓굅쨌媛??諛곌? 怨쇳뙣???댁냼). 愿寃?誘몄긽? 湲濡쒕쾶 ?대갚.</summary>
        public void SetPerTaskRadius(bool on)
            => Check(Native.r3d_set_per_task_radius(H, on ? 1 : 0), "set_per_task_radius");

        /// <summary>C1 negotiated-congestion(CBS-lite, Phase C) 源딆씠 ??0=OFF(?됰㈃ rip-up留뙿룰린議??숈옉).
        /// >0 ?대㈃ ?됰㈃ rip-up ?쇰줈?????由??ㅽ뙣 諛곌???blocker ??blocker 源뚯? ?ш? ?묐낫?쒖폒 ?댁냼(臾댁넀?ㅒ?
        /// 寃곗젙??. [0,3] ?대옩???붿쭊). 怨좊???蹂묐ぉ???붿뿬 ?ㅽ뙣瑜?以꾩씤??</summary>
        public void SetCbsDepth(int depth)
            => Check(Native.r3d_set_cbs_depth(H, depth), "set_cbs_depth");

        /// <summary>C2 肄붾꼫 理쒖냼諛섍꼍 諛곗닔(Phase C) ???섎낫 ?ъ씠 吏곸꽑(?? ??(mult 횞 愿寃? 蹂댁옣(?쒖옉??.
        /// 寃쎈줈(?) ?④퀎?먯꽌 異⑸룎寃???섏뿉 吏㏃? ?④????≪닔(爰얠엫 鍮꾩쬆媛???뚮쭔, ???앹젏 怨좎젙). 0=OFF(湲곗〈
        /// ?숈옉). 沅뚯옣 2.0. PathRectifier(?뚮뜑 ?덈꺼, ?섎룎由?? ?щ━ 異⑸룎 ?덉쟾.</summary>
        public void SetMinStraight(double mult)
            => Check(Native.r3d_set_min_straight(H, mult), "set_min_straight");

        /// <summary>肄붾꼫 理쒖냼吏곸꽑(?덈? mm, ?섎뱶 ?쒖빟) ??A* 媛 '??踰?爰얠씤 ????湲몄씠留뚰겮 吏곸쭊?섍린 ?꾩뿏 ?ㅼ떆
        /// 爰얠? 紐삵븯?꾨줉' ?먯깋 ?④퀎?먯꽌 媛뺤젣?쒕떎. ?섎낫 媛?紐⑤뱺 吏곸꽑 援ш컙(?④?)????湲몄씠 ?댁긽 蹂댁옣??愿寃?臾닿?쨌
        /// ??諛곌?, 紐⑺몴 吏곸쟾 留덉?留??묒냽 援ш컙留?硫댁젣). SetMinStraight(愿寃?諛곗닔쨌?꾩쿂由??≪닔)? ?щ━ ?섎뱶 蹂댁옣.
        /// 0=OFF(湲곗〈 ?숈옉쨌怨⑤뱺 遺덈?). 沅뚯옣 100mm.</summary>
        public void SetMinStraightMm(double mm)
            => Check(Native.r3d_set_min_straight_mm(H, mm), "set_min_straight_mm");

        /// <summary>諛곌?-諛곌? ?닿꺽(mm) ????諛곌? ?쇳꽣??嫄곕━ ??r1 + r2 + gap 蹂댁옣(?쒕㈃ ?ъ씠 理쒖냼 gap mm ?꾩?).
        /// 湲곗〈 留덊궧? ?쇳꽣?좎쓣 ~愿寃쎈쭔?쇰쭔 ?꾩썙 ?쒕㈃??留욌떯?섎떎(寃뱀퀜 蹂댁엫). gap>0 ?대㈃ route_multi 媛 源붾┛
        /// 諛곌?????諛섍꼍?쇰줈 留됱븘 ?뺥솗???꾩슫?? 0=湲곗〈 ?숈옉. 洹쒓꺽 60mm.</summary>
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

        /// <summary>???寃⑹옄 理쒕? ?먯깋 ?뺤옣 ????0=?섍꼍蹂??湲곕낯媛?48M). 理쒕떒寃쎈줈 紐⑤뱶 ?깆뿉???먯깋 ?곹븳??
        /// 以꾩뿬 諛곌????먯깋 ??컻(?섎텇 ?숆껐)??諛⑹??쒕떎. large_threshold(湲곕낯 5M?) ?댄븯 寃⑹옄??臾댁젣??-1)?대?濡?
        /// ??媛믪? ???寃⑹옄?먯꽌留??곸슜?쒕떎. 沅뚯옣: 理쒕떒寃쎈줈 8M, ?뱀쭠??湲곗〈?ㅺ퀎 0(湲곕낯 48M).</summary>
        public void SetMaxExpansions(long maxExp)
        {
            var opt = new Native.R3dRuntimeOptions { max_expansions = maxExp };
            Check(Native.r3d_set_runtime_options(H, in opt), "set_runtime_options");
        }

        /// <summary>???寃⑹옄 ?먯깋 ?곹븳 + 怨꾩링(hier) escalation ?꾧퀎(hier_probe)瑜??④퍡 ?ㅼ젙.
        /// hierProbe 瑜??ш쾶(=maxExp) 二쇰㈃ ?대젮??諛곌???'吏곸젒 媛以?A*'濡?癒쇱? 異⑸텇???먯깋???ㅼ뿉??怨꾩링
        /// corridor 濡??섏뼱媛꾨떎 ??吏곸젒 A*媛 李얜뒗 吏㏃?(?쨢_heur諛? 寃쎈줈瑜?怨꾩링??3~4횞 ?고쉶蹂대떎 ?곗꽑?쒕떎.
        /// 理쒕떒寃쎈줈 紐⑤뱶泥섎읆 '湲몄씠'媛 以묒슂??寃쎌슦???대떎(????대젮??諛곌?? ???ㅻ옒 ?먯깋).</summary>
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

        public string GetRuntimeReportJson()
        {
            Check(Native.r3d_get_runtime_report(H, out var json), "get_runtime_report");
            return Native.TakeString(json);
        }

        public void AddObstacle(double minx, double miny, double minz, double maxx, double maxy, double maxz)
            => Check(Native.r3d_add_obstacle(H, minx, miny, minz, maxx, maxy, maxz), "add_obstacle");

        // ?듦낵(pass-through) 媛앹껜 異붽? ??寃쎈줈?먯깋 異⑸룎 ?쒖쇅, '?듦낵 ?먯쑀留? 媛?쒗솕??
        public void AddPassthrough(double minx, double miny, double minz, double maxx, double maxy, double maxz)
            => Check(Native.r3d_add_passthrough(H, minx, miny, minz, maxx, maxy, maxz), "add_passthrough");

        public int AddTask(double sx, double sy, double sz, double gx, double gy, double gz,
                           string? utility, string? utilityGroup)
        {
            int idx = Native.r3d_add_task(H, sx, sy, sz, gx, gy, gz, Native.Utf8(utility), Native.Utf8(utilityGroup));
            if (idx < 0) throw new InvalidOperationException("r3d_add_task ?ㅽ뙣");
            return idx;
        }

        public void SetTaskEndpoints(int task, double sx, double sy, double sz, double gx, double gy, double gz)
            => Check(Native.r3d_set_task_endpoints(H, task, sx, sy, sz, gx, gy, gz), "set_task_endpoints");

        /// <summary>?묒뾽 愿寃?mm) ?ㅼ젙 ???곗꽑?쒖쐞 "diameter"/"utility" ?뺣젹?먯꽌 '援듭? 諛곌? 癒쇱?' ??
        /// 0=愿寃?臾댁떆(湲곗〈 嫄곕━ ?뺣젹怨??숈씪). 援듭? 諛곌???理쒕떒 寃쎈줈瑜??좎젏??媛??諛곌????고쉶쨌異⑸룎?섏? ?딄쾶 ?쒕떎.</summary>
        public void SetTaskDiameter(int task, double diameterMm)
            => Check(Native.r3d_set_task_diameter(H, task, diameterMm), "set_task_diameter");

        /// <summary>?묒뾽 紐⑺몴 吏꾩엯異??쒖빟 ??A* 媛 紐⑺몴(end)??axis(0..5 = +x,-x,+y,-y,+z,-z) 諛⑺뼢?쇰줈 吏꾩엯??
        /// ?뚮쭔 ?꾨떖 ?몄젙. ?뺥듃 醫낅떒 ?ㅽ뀅 由щ뱶??異뺤쓣 二쇰㈃ 諛곌????ㅽ뀅???쇱쭅??吏꾩엯(援곕뜑?붽린 爰얠엫 ?쒓굅).
        /// -1=臾댁젣??湲곕낯). ?쒖빟?쇰줈 留됲엳硫??붿쭊??臾댁젣??1???대갚(?곌껐 ?곗꽑).</summary>
        public void SetTaskGoalDir(int task, int axis)
            => Check(Native.r3d_set_task_goal_dir(H, task, axis), "set_task_goal_dir");

        // ---- ?쇱슦??----
        public void RouteMulti(string priority = "longest")
            => Check(Native.r3d_route_multi(H, Native.Utf8(priority)), "route_multi");

        /// <summary>?숈뒿???뚮옉 ?(ijk ?쇱쨷???됲깂 諛곗뿴)???ㅼ젙?쒕떎(L2b ?뚰봽??諛붿씠?댁뒪). w_corridor>0 ????
        /// route_multi 媛 ????ㅼ쓣 ?뚮옉 ?쒕뱶濡??쇱븘 諛곌???洹?怨곸쑝濡??좊룄. null/鍮?諛곗뿴?대㈃ ?뚮옉??鍮꾩슫??</summary>
        public void SetCorridorCells(int[]? ijk)
            => Check(Native.r3d_set_corridor_cells(H, ijk, ijk == null ? 0 : ijk.Length / 3), "set_corridor_cells");

        /// <summary>?쇱슦??吏꾪뻾 ?대깽?? Phase 0=?먯깋 吏꾪뻾(Progress01), 1=諛곌? ?꾨즺(吏??Path).</summary>
        public readonly record struct RouteProgress(int Phase, int OrderIndex, int TaskIndex,
            bool Success, double LengthMm, int Turns, long ExpandedNodes, double ElapsedMs,
            int Done, int Total, double Progress01, PathCell[] Path);

        /// <summary>route_multi ? ?숈씪(?쒖감쨌異⑸룎?놁쓬)?섎릺 諛곌?留덈떎 onPipe 瑜??몄텧(泥섎━?쒖꽌쨌吏꾪뻾?㉱룰꼍濡??ㅼ떆媛?.
        /// 肄쒕갚? ?쇱슦???ㅻ젅?쒖뿉???숆린 ?몄텧?섎?濡? UI 媛깆떊? ?몄텧?먭? Dispatcher 濡?留덉꺃留곹븳??
        /// shouldCancel ??true 瑜?諛섑솚?섎㈃ ?붿쭊???꾩옱 諛곌? ?먯깋??利됱떆 以묐떒?섍퀬 ?⑥? 諛곌? ?놁씠 ?뺤긽 諛섑솚?쒕떎
        /// (?묐젰??痍⑥냼 ????5留??뺤옣留덈떎쨌諛곌? ?꾨즺留덈떎 寃??. ?꾨즺??諛곌? 寃곌낵??蹂댁〈?쒕떎.</summary>
        public void RouteMultiProgress(string priority, Action<RouteProgress> onPipe, Func<bool>? shouldCancel = null)
        {
            // ?몃━寃뚯씠?몃뒗 ?ㅼ씠?곕툕 ?몄텧???앸궇 ?뚭퉴吏 ?댁븘 ?덉뼱???쒕떎(吏??蹂?섎줈 GC 蹂댄샇).
            Native.R3dProgressFn cb = (user, phase, oi, ti, ok, len, turns, exp, ms, done, total, prog, pathPtr, pathLen) =>
            {
                var path = Array.Empty<PathCell>();
                if (phase == 1 && pathLen > 0 && pathPtr != IntPtr.Zero)
                {
                    var buf = new int[pathLen * 3];
                    System.Runtime.InteropServices.Marshal.Copy(pathPtr, buf, 0, buf.Length);  // 利됱떆 蹂듭궗.
                    path = new PathCell[pathLen];
                    for (int i = 0; i < pathLen; i++) path[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
                }
                onPipe(new RouteProgress(phase, oi, ti, ok != 0, len, turns, exp, ms, done, total, prog, path));
                return (shouldCancel != null && shouldCancel()) ? 1 : 0;   // 0?꾨떂=痍⑥냼 ???붿쭊 ?먯깋 以묐떒.
            };
            try { Check(Native.r3d_route_multi_progress(H, Native.Utf8(priority), cb, IntPtr.Zero), "route_multi_progress"); }
            finally { GC.KeepAlive(cb); }
        }

        // ?⑥씪 ?묒뾽 ?щ씪?고똿(?먮낯 ?μ븷臾?湲곗?, ?ㅻⅨ 諛곌? 臾댁떆). 寃곌낵???붿쭊????λ맂??
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
        // ????λ㈃??怨꾩링 corridor ?쇱슦??Sparse + coarse?뭚ine). ?묒뾽蹂??낅┰(異⑸룎 ?뚰뵾 ?놁쓬).
        public void RouteCorridor(int factor = 16, int radius = 2)
            => Check(Native.r3d_route_corridor(H, factor, radius), "route_corridor");

        // ?쒖감 怨꾩링 corridor(Sparse + astar_hashed, ? ??諛곗뿴 誘명븷????10mm ??????뺣? 寃⑹옄 ?덉쟾).
        // priority ?쒖꽌濡???諛곌????쇱슦?낇븯怨?mark_pipe(pipeRadius)濡??먯쑀 異붽? ??諛곌? 媛?異⑸룎 0.
        public void RouteCorridorMulti(int factor, int radius, string priority = "longest", int pipeRadius = 0)
            => Check(Native.r3d_route_corridor_multi(H, factor, radius, Native.Utf8(priority), pipeRadius),
                     "route_corridor_multi");

        // ---- 寃곌낵 議고쉶 ----
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
                ElapsedMs = r.elapsed_ms,
                Fail = (RouteFail)r.fail_reason,
                Path = path,
                Visited = visited,
            };
        }

        /// <summary>'?먯쑀留? 媛?쒗솕 ?????꾩옱 doc ??voxelize ??釉붾줉 ? ?꾩껜瑜???踰덉뿉 諛섑솚.</summary>
        public PathCell[] CopyBlocked()
        {
            int total = Native.r3d_copy_blocked(H, null, 0);  // ?ъ씠利?議고쉶.
            if (total <= 0) return Array.Empty<PathCell>();
            var buf = new int[total * 3];
            int n = Native.r3d_copy_blocked(H, buf, total);
            var cells = new PathCell[n];
            for (int i = 0; i < n; i++) cells[i] = new PathCell(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
            return cells;
        }

        /// <summary>?먯쑀 ????붿쭊?먯꽌 吏곸젒 洹좊벑 ?섑뵆留곹빐 蹂듭궗?쒕떎. ????λ㈃ 誘몃━蹂닿린??</summary>
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
        /// <summary>'?듦낵 ?먯쑀留? 媛?쒗솕 ??doc.passthrough ??voxelize ??? ?꾩껜 諛섑솚.</summary>
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

        /// <summary>諛⑸Ц(?뺤옣) ? ?섏쭛??耳쒓퀬/?꾧린. 湲곕낯 ON. OFF 硫??쇱슦????Visited 媛 鍮꾩뼱?덈떎.</summary>
        public void SetCollectVisited(bool on)
            => Check(Native.r3d_set_collect_visited(H, on ? 1 : 0), "set_collect_visited");

        public string DumpSceneText()
        {
            Check(Native.r3d_dump_scene_text(H, out var p), "dump_scene_text");
            return Native.TakeString(p);
        }

        /// <summary>?ν듃由?由ы봽 ?몃뱶 諛곗뿴 諛섑솚 (3D 媛?쒗솕??.
        /// maxLeaves: ?곹븳(湲곕낯 1M). State: 0=FREE, 1=BLOCKED.</summary>
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
            if (status != 0) throw new InvalidOperationException($"r3d_{op} ?ㅽ뙣 (status {status})");
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _h.Dispose();
        }
    }
}

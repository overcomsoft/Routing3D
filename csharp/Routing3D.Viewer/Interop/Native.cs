// Routing3D C ABI P/Invoke ?좎뼵 (routing3d_capi.dll) ??C# 酉곗뼱
// =============================================================================
// [???뚯씪???섎뒗 ??
//   cpp/capi/routing3d_capi.h ??C ABI 瑜?.NET ?먯꽌 ?몄텧?섍린 ?꾪븳 P/Invoke ?좎뼵.
//   臾몄옄?댁? 紐⑤몢 UTF-8 諛붿씠??byte[])濡??꾨떖?쒕떎(?쒓? ?대쫫 ?덉쟾 ??ANSI 留덉꺃留?湲덉?).
//   ?ㅺ퀎: docs/csharp_helix_interop_design.md 짠4.
// =============================================================================
using System;
using System.Runtime.InteropServices;
using System.Text;

namespace Routing3D.Viewer.Interop
{
    internal static class Native
    {
        // routing3d_capi.dll (??異쒕젰 ?대뜑??蹂듭궗??. ?뺤옣???놁씠 ?곸쑝硫?OS 媛 .dll ??遺숈씤??
        private const string Dll = "routing3d_capi";
        private const CallingConvention Cdecl = CallingConvention.Cdecl;

        // R3dGrid (blittable, C ?ㅻ뜑? 1:1).
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dGrid
        {
            public double cell_mm;
            public double ox, oy, oz;
            public int nx, ny, nz;
        }

        // R3dParams (blittable, C ?ㅻ뜑? 1:1).
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dParams
        {
            public double cell_mm, w_turn, w_clear;
            public double w_corridor;            // ?뚮옉 諛?? 媛??mm. 0=鍮꾪솢??湲곗〈 ?숈옉).
            public double w_heur;                // ?대━?ㅽ떛 媛以?weighted A*). 0/1=?쒖?, >1=紐⑺몴 吏??
            public double w_heur_near;           // ?숈쟻 媛以?紐⑺몴洹쇱쿂 媛? (0,w_heur)=?섎졃 媛以? 0=?뺤쟻.
            public int clearance_radius, clearance_connectivity;
            public int corridor_radius;          // ?뚮옉 ?깆옣 諛섍꼍(?).
            public int rack_level_count;         // rack_levels ?ъ슜 媛쒖닔(0~8).
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public int[] rack_levels;            // ?좏샇 ??z? ?몃뜳??, 理쒕? 8.
        }

        // R3dResult (blittable).
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dResult
        {
            public int success;
            public double length_mm;
            public double cost_mm;
            public int turns;
            public long expanded_nodes;
            public double elapsed_ms;
            public int path_len;
            public int visited_len;   // 諛⑸Ц(?뺤옣) ? ????'諛⑸Ц留? 媛?쒗솕 ?? 鍮꾪솢????0.
            public int fail_reason;   // ?ㅽ뙣 ?ъ쑀(A1) ??success=0 ???뚮쭔. 0~6(RouteFail). 援ъ“泥???異붽?.
        }

        // ---- 怨듯넻 ----
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern IntPtr r3d_version();
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern void r3d_free_string(IntPtr s);

        // ---- Level 1: 臾몄옄??ABI ----
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_route_scene_text(byte[] sceneUtf8, byte[] modeUtf8,
                                                      byte[] priorityUtf8, out IntPtr outScene);

        // ---- Level 2: ?몃뱾 ABI ----
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern IntPtr r3d_create();
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern void r3d_destroy(IntPtr e);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_load_scene_text(IntPtr e, byte[] sceneUtf8);

        // R3dRuntimeOptions (blittable, C ?ㅻ뜑 R3dRuntimeOptions ? 1:1). 0 ?꾨뱶=?붿쭊 湲곕낯媛??ъ슜.
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dRuntimeOptions
        {
            public long large_grid_threshold;   // 0=default 5,000,000 cells.
            public long max_expansions;         // 0=env R3D_MAX_EXP/default(48M).
            public long fallback_expansions;    // 0=max_expansions/env.
            public int  hier_factor;            // 0=default 8.
            public int  hier_radius;            // 0=default 2.
            public long hier_probe;             // 0=default 300,000.
            public int  ripup_enabled;          // -1=env/default, 0=off, 1=on.
            public long cbs_expansions;         // 0=engine/env default; CBS-lite per-route cap.
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct R3dTraceOptions
        {
            public int enabled;
            public int level;
            public int sample_every;
            public int include_occupancy;
            public int include_rejects;
            public int include_postprocess;
            public int max_events_per_task;
        }

        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_grid(IntPtr e, in R3dGrid g);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_params(IntPtr e, in R3dParams p);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_runtime_options(IntPtr e, in R3dRuntimeOptions opt);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_trace_options(IntPtr e, in R3dTraceOptions opt);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_trace_file(IntPtr e, byte[] pathUtf8);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_flush_trace(IntPtr e);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_get_runtime_report(IntPtr e, out IntPtr outJson);
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_add_obstacle(IntPtr e, double minx, double miny, double minz,
                                                 double maxx, double maxy, double maxz);
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_add_passthrough(IntPtr e, double minx, double miny, double minz,
                                                    double maxx, double maxy, double maxz);
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_add_task(IntPtr e, double sx, double sy, double sz,
                                             double gx, double gy, double gz,
                                             byte[] utilityUtf8, byte[] utilityGroupUtf8);
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_set_task_endpoints(IntPtr e, int task, double sx, double sy,
                                                       double sz, double gx, double gy, double gz);
        // ?묒뾽 愿寃?mm) ???곗꽑?쒖쐞 "diameter"/"utility" ??'援듭? 諛곌? 癒쇱?' ?뺣젹 ?? 0=愿寃?臾댁떆.
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_task_diameter(IntPtr e, int task, double diameterMm);
        // ?묒뾽 紐⑺몴 吏꾩엯異??쒖빟 ??A* 媛 紐⑺몴??axis(0..5=+x,-x,+y,-y,+z,-z) 諛⑺뼢?쇰줈 吏꾩엯???뚮쭔 ?꾨떖. -1=臾댁젣??
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_task_goal_dir(IntPtr e, int task, int axis);

        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_multi(IntPtr e, byte[] priorityUtf8);

        // ?숈뒿???뚮옉 ?(ijk ?쇱쨷??뾫) ?ㅼ젙 ??w_corridor>0 ????route_multi 媛 ?쒕뱶濡??ъ슜(L2b). n<=0=珥덇린??
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_corridor_cells(IntPtr e, int[]? ijk, int n);

        // 吏꾪뻾 肄쒕갚(cdecl) ??phase=0(?먯깋 吏꾪뻾)/1(諛곌? ?꾨즺). UnmanagedFunctionPointer 濡?留덉꺃留?
        // pathIjk ??phase==1 ?깃났 ??寃쎈줈 ?((i,j,k)횞pathLen) ?ъ씤??肄쒕갚 ?숈븞留??좏슚 ??利됱떆 蹂듭궗).
        // 諛섑솚 0=怨꾩냽, 0?꾨떂=痍⑥냼(abort) ???붿쭊???꾩옱 諛곌? ?먯깋??以묐떒?섍퀬 ?⑥? 諛곌? ?놁씠 ?뺤긽 諛섑솚.
        [UnmanagedFunctionPointer(Cdecl)]
        public delegate int R3dProgressFn(IntPtr user, int phase, int orderIndex, int taskIndex,
                                          int success, double lengthMm, int turns, long expandedNodes,
                                          double elapsedMs, int done, int total, double progress01,
                                          IntPtr pathIjk, int pathLen);
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_route_multi_progress(IntPtr e, byte[] priorityUtf8,
                                                          R3dProgressFn cb, IntPtr user);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_task(IntPtr e, int task, out R3dResult outRes);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_task_anytime(IntPtr e, int task, double initialWeight, double finalWeight, double weightStep, double timeBudgetMs, long maxExpansions, int goalDir, out R3dResult outRes, out int iterations, out int improvements);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_corridor(IntPtr e, int factor, int radius);
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_route_corridor_multi(IntPtr e, int factor, int radius,
                                                          byte[] priorityUtf8, int pipeRadius);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_get_result(IntPtr e, int task, out R3dResult outRes);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_path(IntPtr e, int task, [Out] int[] buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_visited(IntPtr e, int task, [Out] int[] buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_blocked(IntPtr e, [Out] int[]? buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_blocked_sampled(IntPtr e, int maxCells, [Out] int[] buf);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_passthrough(IntPtr e, [Out] int[]? buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_collect_visited(IntPtr e, int enabled);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_pipe_radius(IntPtr e, int radiusCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_per_task_radius(IntPtr e, int enabled);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_cbs_depth(IntPtr e, int depth);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_min_straight(IntPtr e, double mult);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_min_straight_mm(IntPtr e, double mm);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_pipe_gap(IntPtr e, double gapMm);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_segment_astar(IntPtr e, int enabled, int maxSegmentCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_octree_guide(IntPtr e, int enabled, int corridorRadius);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_route_split(IntPtr e, int enabled, double trunkZMm);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_dump_scene_text(IntPtr e, out IntPtr outScene);

        // ---- ?ν듃由?由ы봽 ?닿굅 (3D 媛?쒗솕) ----
        // ?붿쭊??濡쒕뱶???ъ쑝濡??ν듃由щ? 鍮뚮뱶?섍퀬 紐⑤뱺 由ы봽瑜?buf ??梨꾩슫??
        // state: 0=FREE, 1=BLOCKED. R3D_OK 硫?*outCount ???ㅼ젣 媛쒖닔.
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dOctreeLeaf
        {
            public float X0Mm, Y0Mm, Z0Mm;   // 由ы봽 ?먯젏 (world mm)
            public float SizeMm;               // 由ы봽 ??蹂 ?ш린 (mm)
            public int   State;                // 0=FREE, 1=BLOCKED
        }
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_enum_octree_leaves(IntPtr e,
            [Out] R3dOctreeLeaf[]? buf, int maxCount, out int outCount);

        // 臾몄옄????UTF-8 諛붿씠????醫낅즺). ?쒓? 蹂댁〈.
        public static byte[] Utf8(string? s) => Encoding.UTF8.GetBytes((s ?? string.Empty) + "\0");

        // 肄쒕━ ?좊떦 char* ??string ???댁젣.
        public static string TakeString(IntPtr p)
        {
            if (p == IntPtr.Zero) return string.Empty;
            try { return Marshal.PtrToStringUTF8(p) ?? string.Empty; }
            finally { r3d_free_string(p); }
        }

        public static string VersionString() => Marshal.PtrToStringUTF8(r3d_version()) ?? "(unknown)";
    }
}

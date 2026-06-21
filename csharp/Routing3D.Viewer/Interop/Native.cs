// Routing3D C ABI P/Invoke ? ì–¸ (routing3d_capi.dll) ??C# ë·°ì–´
// =============================================================================
// [???Œì¼???˜ëŠ” ??
//   cpp/capi/routing3d_capi.h ??C ABI ë¥?.NET ?ì„œ ?¸ì¶œ?˜ê¸° ?„í•œ P/Invoke ? ì–¸.
//   ë¬¸ìž?´ì? ëª¨ë‘ UTF-8 ë°”ì´??byte[])ë¡??„ë‹¬?œë‹¤(?œê? ?´ë¦„ ?ˆì „ ??ANSI ë§ˆìƒ¬ë§?ê¸ˆì?).
//   ?¤ê³„: docs/csharp_helix_interop_design.md Â§4.
// =============================================================================
using System;
using System.Runtime.InteropServices;
using System.Text;

namespace Routing3D.Viewer.Interop
{
    internal static class Native
    {
        // routing3d_capi.dll (??ì¶œë ¥ ?´ë”??ë³µì‚¬??. ?•ìž¥???†ì´ ?ìœ¼ë©?OS ê°€ .dll ??ë¶™ì¸??
        private const string Dll = "routing3d_capi";
        private const CallingConvention Cdecl = CallingConvention.Cdecl;

        // R3dGrid (blittable, C ?¤ë”?€ 1:1).
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dGrid
        {
            public double cell_mm;
            public double ox, oy, oz;
            public int nx, ny, nz;
        }

        // R3dParams (blittable, C ?¤ë”?€ 1:1).
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dParams
        {
            public double cell_mm, w_turn, w_clear;
            public double w_corridor;            // ?Œëž‘ ë°??€ ê°€??mm. 0=ë¹„í™œ??ê¸°ì¡´ ?™ìž‘).
            public double w_heur;                // ?´ë¦¬?¤í‹± ê°€ì¤?weighted A*). 0/1=?œì?, >1=ëª©í‘œ ì§€??
            public double w_heur_near;           // ?™ì  ê°€ì¤?ëª©í‘œê·¼ì²˜ ê°? (0,w_heur)=?˜ë ´ ê°€ì¤? 0=?•ì .
            public int clearance_radius, clearance_connectivity;
            public int corridor_radius;          // ?Œëž‘ ?±ìž¥ ë°˜ê²½(?€).
            public int rack_level_count;         // rack_levels ?¬ìš© ê°œìˆ˜(0~8).
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public int[] rack_levels;            // ? í˜¸ ??z?€ ?¸ë±??, ìµœë? 8.
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
            public int visited_len;   // ë°©ë¬¸(?•ìž¥) ?€ ????'ë°©ë¬¸ë§? ê°€?œí™” ?? ë¹„í™œ????0.
            public int fail_reason;   // ?¤íŒ¨ ?¬ìœ (A1) ??success=0 ???Œë§Œ. 0~6(RouteFail). êµ¬ì¡°ì²???ì¶”ê?.
        }

        // ---- ê³µí†µ ----
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern IntPtr r3d_version();
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern void r3d_free_string(IntPtr s);

        // ---- Level 1: ë¬¸ìž??ABI ----
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_route_scene_text(byte[] sceneUtf8, byte[] modeUtf8,
                                                      byte[] priorityUtf8, out IntPtr outScene);

        // ---- Level 2: ?¸ë“¤ ABI ----
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern IntPtr r3d_create();
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern void r3d_destroy(IntPtr e);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_load_scene_text(IntPtr e, byte[] sceneUtf8);

        // R3dRuntimeOptions (blittable, C ?¤ë” R3dRuntimeOptions ?€ 1:1). 0 ?„ë“œ=?”ì§„ ê¸°ë³¸ê°??¬ìš©.
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
        // ?‘ì—… ê´€ê²?mm) ???°ì„ ?œìœ„ "diameter"/"utility" ??'êµµì? ë°°ê? ë¨¼ì?' ?•ë ¬ ?? 0=ê´€ê²?ë¬´ì‹œ.
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_task_diameter(IntPtr e, int task, double diameterMm);
        // ?‘ì—… ëª©í‘œ ì§„ìž…ì¶??œì•½ ??A* ê°€ ëª©í‘œ??axis(0..5=+x,-x,+y,-y,+z,-z) ë°©í–¥?¼ë¡œ ì§„ìž…???Œë§Œ ?„ë‹¬. -1=ë¬´ì œ??
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_task_goal_dir(IntPtr e, int task, int axis);

        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_multi(IntPtr e, byte[] priorityUtf8);

        // ?™ìŠµ???Œëž‘ ?€(ijk ?¼ì¤‘??—n) ?¤ì • ??w_corridor>0 ????route_multi ê°€ ?œë“œë¡??¬ìš©(L2b). n<=0=ì´ˆê¸°??
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_corridor_cells(IntPtr e, int[]? ijk, int n);

        // ì§„í–‰ ì½œë°±(cdecl) ??phase=0(?ìƒ‰ ì§„í–‰)/1(ë°°ê? ?„ë£Œ). UnmanagedFunctionPointer ë¡?ë§ˆìƒ¬ë§?
        // pathIjk ??phase==1 ?±ê³µ ??ê²½ë¡œ ?€((i,j,k)Ã—pathLen) ?¬ì¸??ì½œë°± ?™ì•ˆë§?? íš¨ ??ì¦‰ì‹œ ë³µì‚¬).
        // ë°˜í™˜ 0=ê³„ì†, 0?„ë‹˜=ì·¨ì†Œ(abort) ???”ì§„???„ìž¬ ë°°ê? ?ìƒ‰??ì¤‘ë‹¨?˜ê³  ?¨ì? ë°°ê? ?†ì´ ?•ìƒ ë°˜í™˜.
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

        // ---- ?¥íŠ¸ë¦?ë¦¬í”„ ?´ê±° (3D ê°€?œí™”) ----
        // ?”ì§„??ë¡œë“œ???¬ìœ¼ë¡??¥íŠ¸ë¦¬ë? ë¹Œë“œ?˜ê³  ëª¨ë“  ë¦¬í”„ë¥?buf ??ì±„ìš´??
        // state: 0=FREE, 1=BLOCKED. R3D_OK ë©?*outCount ???¤ì œ ê°œìˆ˜.
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dOctreeLeaf
        {
            public float X0Mm, Y0Mm, Z0Mm;   // ë¦¬í”„ ?ì  (world mm)
            public float SizeMm;               // ë¦¬í”„ ??ë³€ ?¬ê¸° (mm)
            public int   State;                // 0=FREE, 1=BLOCKED
        }
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_enum_octree_leaves(IntPtr e,
            [Out] R3dOctreeLeaf[]? buf, int maxCount, out int outCount);

        // ë¬¸ìž????UTF-8 ë°”ì´????ì¢…ë£Œ). ?œê? ë³´ì¡´.
        public static byte[] Utf8(string? s) => Encoding.UTF8.GetBytes((s ?? string.Empty) + "\0");

        // ì½œë¦¬ ? ë‹¹ char* ??string ???´ì œ.
        public static string TakeString(IntPtr p)
        {
            if (p == IntPtr.Zero) return string.Empty;
            try { return Marshal.PtrToStringUTF8(p) ?? string.Empty; }
            finally { r3d_free_string(p); }
        }

        public static string VersionString() => Marshal.PtrToStringUTF8(r3d_version()) ?? "(unknown)";
    }
}

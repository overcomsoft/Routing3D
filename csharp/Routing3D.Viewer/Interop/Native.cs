// Routing3D C ABI P/Invoke 선언 (routing3d_capi.dll) — C# 뷰어
// =============================================================================
// [이 파일이 하는 일]
//   cpp/capi/routing3d_capi.h 의 C ABI 를 .NET 에서 호출하기 위한 P/Invoke 선언.
//   문자열은 모두 UTF-8 바이트(byte[])로 전달한다(한글 이름 안전 — ANSI 마샬링 금지).
//   설계: docs/csharp_helix_interop_design.md §4.
// =============================================================================
using System;
using System.Runtime.InteropServices;
using System.Text;

namespace Routing3D.Viewer.Interop
{
    internal static class Native
    {
        // routing3d_capi.dll (앱 출력 폴더에 복사됨). 확장자 없이 적으면 OS 가 .dll 을 붙인다.
        private const string Dll = "routing3d_capi";
        private const CallingConvention Cdecl = CallingConvention.Cdecl;

        // R3dGrid (blittable, C 헤더와 1:1).
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dGrid
        {
            public double cell_mm;
            public double ox, oy, oz;
            public int nx, ny, nz;
        }

        // R3dParams (blittable, C 헤더와 1:1).
        [StructLayout(LayoutKind.Sequential)]
        public struct R3dParams
        {
            public double cell_mm, w_turn, w_clear;
            public double w_corridor;            // 회랑 밖 셀 가산 mm. 0=비활성(기존 동작).
            public int clearance_radius, clearance_connectivity;
            public int corridor_radius;          // 회랑 성장 반경(셀).
            public int rack_level_count;         // rack_levels 사용 개수(0~8).
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public int[] rack_levels;            // 선호 단(z셀 인덱스), 최대 8.
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
            public int visited_len;   // 방문(확장) 셀 수 — '방문맵' 가시화 용. 비활성 시 0.
        }

        // ---- 공통 ----
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern IntPtr r3d_version();
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern void r3d_free_string(IntPtr s);

        // ---- Level 1: 문자열 ABI ----
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_route_scene_text(byte[] sceneUtf8, byte[] modeUtf8,
                                                      byte[] priorityUtf8, out IntPtr outScene);

        // ---- Level 2: 핸들 ABI ----
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern IntPtr r3d_create();
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern void r3d_destroy(IntPtr e);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_load_scene_text(IntPtr e, byte[] sceneUtf8);

        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_grid(IntPtr e, in R3dGrid g);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_params(IntPtr e, in R3dParams p);
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

        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_multi(IntPtr e, byte[] priorityUtf8);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_task(IntPtr e, int task, out R3dResult outRes);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_route_corridor(IntPtr e, int factor, int radius);
        [DllImport(Dll, CallingConvention = Cdecl)]
        public static extern int r3d_route_corridor_multi(IntPtr e, int factor, int radius,
                                                          byte[] priorityUtf8, int pipeRadius);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_get_result(IntPtr e, int task, out R3dResult outRes);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_path(IntPtr e, int task, [Out] int[] buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_visited(IntPtr e, int task, [Out] int[] buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_blocked(IntPtr e, [Out] int[] buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_copy_passthrough(IntPtr e, [Out] int[] buf, int bufCells);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_set_collect_visited(IntPtr e, int enabled);
        [DllImport(Dll, CallingConvention = Cdecl)] public static extern int r3d_dump_scene_text(IntPtr e, out IntPtr outScene);

        // 문자열 → UTF-8 바이트(널 종료). 한글 보존.
        public static byte[] Utf8(string? s) => Encoding.UTF8.GetBytes((s ?? string.Empty) + "\0");

        // 콜리 할당 char* → string 후 해제.
        public static string TakeString(IntPtr p)
        {
            if (p == IntPtr.Zero) return string.Empty;
            try { return Marshal.PtrToStringUTF8(p) ?? string.Empty; }
            finally { r3d_free_string(p); }
        }

        public static string VersionString() => Marshal.PtrToStringUTF8(r3d_version()) ?? "(unknown)";
    }
}

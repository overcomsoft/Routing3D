// DB 라우팅 헤드리스 진단 — corridor 0성공 원인 이분 탐색용(GUI 없이 빠른 반복)
// =============================================================================
// [실행]
//   Routing3D.Viewer.exe --dbroute <projectId> <cellMm> <utility> <outPath>
//   예: --dbroute 1 25 ALKA d:\tmp\diag.txt
// 여러 전략(장애물만 / +충돌확장 / factor 변형 / 단일 route_corridor)의 성공 수를 보고한다.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Routing3D.Viewer.Interop;
using Routing3D.Viewer.Model;

namespace Routing3D.Viewer.Diagnostics
{
    public static class DbRouteDiag
    {
        public static string Run(int projectId, double cellMm, string utility)
        {
            var sb = new StringBuilder();
            var cfg = DbConfig.FromEnv();
            SceneData sd;
            try { sd = ObstacleDbLoader.LoadScene(cfg, projectId, cellMm); }
            catch (Exception ex) { return "LOAD ERROR: " + ex; }

            var g = sd.Grid;
            int passN = sd.Obstacles.Count(o => o.IsPassThrough);
            sb.AppendLine($"grid {g.Nx}x{g.Ny}x{g.Nz} cell={g.CellMm} origin=({g.Ox:0},{g.Oy:0},{g.Oz:0})");
            sb.AppendLine($"obstacles={sd.Obstacles.Count} (passthrough {passN}) equipment={sd.Equipment.Count} ducts={sd.DuctsLaterals.Count} tasks={sd.Tasks.Count}");

            var rows = sd.Tasks.Where(t => string.Equals(t.Utility, utility, StringComparison.OrdinalIgnoreCase)).ToList();
            sb.AppendLine($"utility '{utility}': {rows.Count} tasks");
            if (rows.Count > 0)
            {
                var t0 = rows[0];
                sb.AppendLine($"  sample task0 start=({t0.Sx:0},{t0.Sy:0},{t0.Sz:0}) end=({t0.Gx:0},{t0.Gy:0},{t0.Gz:0})");
            }
            sb.AppendLine();

            int autoF = Math.Clamp((int)Math.Round(160.0 / Math.Max(1.0, cellMm)), 4, 24);
            sb.AppendLine(Try(sd, rows, fac: false, drop: false, factor: autoF, radius: 2, mode: "cm",    $"A corridorMULTI f={autoF} obstaclesOnly"));
            sb.AppendLine(Try(sd, rows, fac: false, drop: false, factor: autoF, radius: 2, mode: "multi", $"F route_multi      obstaclesOnly"));
            sb.AppendLine(Try(sd, rows, fac: true,  drop: true,  factor: autoF, radius: 2, mode: "multi", $"G route_multi      +facilities+drop"));
            return sb.ToString();
        }

        static string Try(SceneData sd, List<TaskInfo> rows, bool fac, bool drop,
                          int factor, int radius, string mode, string label)
        {
            var g = sd.Grid;
            double cell = g.CellMm;
            Engine eng;
            try
            {
                eng = new Engine();
                eng.SetGrid(cell, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
                eng.SetParams(cell, 500, 10, 2, 6);
                foreach (var o in sd.Obstacles)
                    if (o.IsPassThrough) eng.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                    else eng.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);

                var endPts = rows.Select(r => (r.Gx, r.Gy, r.Gz)).ToList();
                double em = cell, minT = cell;
                bool BlocksEnd(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
                    => endPts.Any(p => p.Gx >= mnx - em && p.Gx <= mxx + em &&
                                       p.Gy >= mny - em && p.Gy <= mxy + em &&
                                       p.Gz >= mnz - em && p.Gz <= mxz + em);
                void Box(double a, double b, double c, double d, double e2, double f2)
                {
                    if (d - a < minT) { double m = (a + d) / 2; a = m - minT / 2; d = m + minT / 2; }
                    if (e2 - b < minT) { double m = (b + e2) / 2; b = m - minT / 2; e2 = m + minT / 2; }
                    if (f2 - c < minT) { double m = (c + f2) / 2; c = m - minT / 2; f2 = m + minT / 2; }
                    eng.AddObstacle(a, b, c, d, e2, f2);
                }
                if (fac)
                {
                    foreach (var eq in sd.Equipment)
                        if (!BlocksEnd(eq.MinX, eq.MinY, eq.MinZ, eq.MaxX, eq.MaxY, eq.MaxZ))
                            Box(eq.MinX, eq.MinY, eq.MinZ, eq.MaxX, eq.MaxY, eq.MaxZ);
                    foreach (var dl in sd.DuctsLaterals)
                        if (!BlocksEnd(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ))
                            Box(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ);
                }

                foreach (var t in rows)
                {
                    double sx = t.Sx, sy = t.Sy, sz = t.Sz;
                    if (drop)
                    {
                        double lowest = double.NaN;
                        foreach (var eq in sd.Equipment)
                            if (sx >= eq.MinX - 1 && sx <= eq.MaxX + 1 && sy >= eq.MinY - 1 && sy <= eq.MaxY + 1 &&
                                sz >= eq.MinZ - 1 && sz <= eq.MaxZ + 1 && (double.IsNaN(lowest) || eq.MinZ < lowest))
                                lowest = eq.MinZ;
                        if (!double.IsNaN(lowest)) sz = Math.Max(g.Oz + cell * 0.5, lowest - cell * 0.5);
                    }
                    eng.AddTask(sx, sy, sz, t.Gx, t.Gy, t.Gz, t.Utility, t.Group);
                }
            }
            catch (Exception ex) { return $"{label}: BUILD-EXCEPTION {ex.Message}"; }

            var sw = System.Diagnostics.Stopwatch.StartNew();
            try
            {
                if (mode == "multi") eng.RouteMulti("longest");
                else if (mode == "cm") eng.RouteCorridorMulti(factor, radius, "longest", 0);
                else eng.RouteCorridor(factor, radius);
            }
            catch (Exception ex) { eng.Dispose(); return $"{label}: ROUTE-EXCEPTION {ex.Message}"; }
            sw.Stop();

            int ok = 0; double tot = 0;
            for (int i = 0; i < rows.Count; i++)
            {
                try { var r = eng.GetResult(i); if (r.Success) { ok++; tot += r.LengthMm; } }
                catch { }
            }
            eng.Dispose();
            return $"{label}: success {ok}/{rows.Count} totalLen {tot:0} ({sw.ElapsedMilliseconds} ms)";
        }
    }
}

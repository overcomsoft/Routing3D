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

            var rows = string.Equals(utility, "ALL", StringComparison.OrdinalIgnoreCase)
                ? sd.Tasks.ToList()
                : sd.Tasks.Where(t => string.Equals(t.Utility, utility, StringComparison.OrdinalIgnoreCase)).ToList();
            sb.AppendLine($"utility '{utility}': {rows.Count} tasks");
            if (rows.Count > 0)
            {
                var t0 = rows[0];
                sb.AppendLine($"  sample task0 start=({t0.Sx:0},{t0.Sy:0},{t0.Sz:0}) end=({t0.Gx:0},{t0.Gy:0},{t0.Gz:0})");
            }
            sb.AppendLine();

            // 기존설계 패턴(pgvector) — R3D_PATTERNS=off 면 비활성(A/B 비교용). 기본 ON.
            bool usePat = !string.Equals(Environment.GetEnvironmentVariable("R3D_PATTERNS"), "off",
                                         StringComparison.OrdinalIgnoreCase);
            PatternStore? patterns = usePat ? PatternStore.TryLoad(cfg) : null;
            sb.AppendLine($"기존설계 패턴: {(patterns == null ? (usePat ? "없음(기하 폴백)" : "OFF") : patterns.Count + "키")}");

            sb.AppendLine(Try(sd, rows, fac: true, drop: true, wClear: 10, mode: "multi",
                              "G route_multi +facilities+drop clearON(Implicit 온디맨드)", patterns));
            return sb.ToString();
        }

        static string Try(SceneData sd, List<TaskInfo> rows, bool fac, bool drop,
                          double wClear, string mode, string label, PatternStore? patterns = null,
                          int factor = 6, int radius = 2)
        {
            var g = sd.Grid;
            double cell = g.CellMm;
            Engine eng;
            try
            {
                eng = new Engine();
                eng.SetGrid(cell, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
                int clr = wClear > 0 ? 2 : 0;
                eng.SetParams(cell, 500, wClear, clr, 6, wHeur: 1.5);  // 대형 격자 weighted A*(GUI 와 동일).
                foreach (var o in sd.Obstacles)
                    if (o.IsPassThrough) eng.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                    else eng.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);

                double minT = cell;
                void Box(double a, double b, double c, double d, double e2, double f2)
                {
                    if (d - a < minT) { double m = (a + d) / 2; a = m - minT / 2; d = m + minT / 2; }
                    if (e2 - b < minT) { double m = (b + e2) / 2; b = m - minT / 2; e2 = m + minT / 2; }
                    if (f2 - c < minT) { double m = (c + f2) / 2; c = m - minT / 2; f2 = m + minT / 2; }
                    eng.AddObstacle(a, b, c, d, e2, f2);
                }
                if (fac)   // 설비·덕트·레터럴을 '항상' 솔리드 장애물로(제외 없음) — GUI 와 동일(관통 방지).
                {
                    foreach (var eq in sd.Equipment) Box(eq.MinX, eq.MinY, eq.MinZ, eq.MaxX, eq.MaxY, eq.MaxZ);
                    foreach (var dl in sd.DuctsLaterals) Box(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ);
                }

                // PoC 가 설비/덕트 솔리드 내부면 표면 바로 바깥(+½셀)으로 투영 → 표면 연결(관통 방지).
                // preferFace(학습된 진출/진입 면) 가 있으면 그 면으로 빼낸다(GUI LiftPocToSurface 와 동일).
                (double, double, double) Lift(double x, double y, double z, string? preferFace)
                {
                    double eps = 1.0, m = cell * 0.5;
                    void TryBox(double bx0, double by0, double bz0, double bx1, double by1, double bz1)
                    {
                        if (x <= bx0 - eps || x >= bx1 + eps) return;
                        if (y <= by0 - eps || y >= by1 + eps) return;
                        if (z <= bz0 - eps || z >= bz1 + eps) return;
                        if (preferFace != null)
                        {
                            switch (preferFace)
                            {
                                case "+z": z = bz1 + m; return;
                                case "-z": z = bz0 - m; return;
                                case "+x": x = bx1 + m; return;
                                case "-x": x = bx0 - m; return;
                                case "+y": y = by1 + m; return;
                                case "-y": y = by0 - m; return;
                            }
                        }
                        double dxn = x - bx0, dxp = bx1 - x, dyn = y - by0, dyp = by1 - y, dzn = z - bz0, dzp = bz1 - z;
                        double mn = Math.Min(Math.Min(Math.Min(dxn, dxp), Math.Min(dyn, dyp)), Math.Min(dzn, dzp));
                        if (mn == dzp) z = bz1 + m; else if (mn == dzn) z = bz0 - m;
                        else if (mn == dxp) x = bx1 + m; else if (mn == dxn) x = bx0 - m;
                        else if (mn == dyp) y = by1 + m; else y = by0 - m;
                    }
                    for (int it = 0; it < 4; it++)
                    {
                        double px = x, py = y, pz = z;
                        if (fac)
                        {
                            foreach (var eq in sd.Equipment) TryBox(eq.MinX, eq.MinY, eq.MinZ, eq.MaxX, eq.MaxY, eq.MaxZ);
                            foreach (var dl in sd.DuctsLaterals) TryBox(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ);
                        }
                        if (px == x && py == y && pz == z) break;
                    }
                    if (z < g.Oz + m) z = g.Oz + m;
                    return (x, y, z);
                }

                string? Face(string kind, TaskInfo t) =>
                    patterns != null && patterns.TryGet(kind, t.Group, t.Utility, out var tp) ? tp.Face : null;

                // 접근불가 PoC 전처리(GUI SnapPocToFreeCell 와 동일) — R3D_SNAP=off 면 비활성(A/B).
                bool useSnap = !string.Equals(Environment.GetEnvironmentVariable("R3D_SNAP"), "off",
                                              StringComparison.OrdinalIgnoreCase);
                bool InGrid(double x, double y, double z) =>
                    x >= g.Ox && y >= g.Oy && z >= g.Oz &&
                    x <= g.Ox + g.Nx * cell && y <= g.Oy + g.Ny * cell && z <= g.Oz + g.Nz * cell;
                bool CellBlocked(double x, double y, double z)
                {
                    double minT = cell;
                    int ci = (int)Math.Floor((x - g.Ox) / cell), cj = (int)Math.Floor((y - g.Oy) / cell),
                        ck = (int)Math.Floor((z - g.Oz) / cell);
                    double clx = g.Ox + ci * cell, chx = clx + cell, cly = g.Oy + cj * cell, chy = cly + cell,
                           clz = g.Oz + ck * cell, chz = clz + cell;
                    bool Ov(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
                    {
                        if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
                        if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
                        if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
                        return clx < mxx && chx > mnx && cly < mxy && chy > mny && clz < mxz && chz > mnz;
                    }
                    foreach (var o in sd.Obstacles)
                        if (!o.IsPassThrough && Ov(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ)) return true;
                    if (fac)
                    {
                        foreach (var e in sd.Equipment) if (Ov(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ)) return true;
                        foreach (var dl in sd.DuctsLaterals) if (Ov(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ)) return true;
                    }
                    return false;
                }
                (double, double, double) SnapFree(double x, double y, double z, string? face)
                {
                    if (!useSnap || !fac || !CellBlocked(x, y, z)) return (x, y, z);
                    if (face != null)
                    {
                        (double dx, double dy, double dz) = face switch
                        {
                            "+x" => (1.0, 0.0, 0.0), "-x" => (-1.0, 0.0, 0.0),
                            "+y" => (0.0, 1.0, 0.0), "-y" => (0.0, -1.0, 0.0),
                            "+z" => (0.0, 0.0, 1.0), "-z" => (0.0, 0.0, -1.0),
                            _ => (0.0, 0.0, 0.0),
                        };
                        if (dx != 0 || dy != 0 || dz != 0)
                            for (int k = 1; k <= 16; k++)
                            {
                                double nx = x + dx * cell * k, ny = y + dy * cell * k, nz = z + dz * cell * k;
                                if (InGrid(nx, ny, nz) && !CellBlocked(nx, ny, nz)) return (nx, ny, nz);
                            }
                    }
                    for (int r = 1; r <= 6; r++)
                    {
                        (double, double, double)? best = null; double bd = double.MaxValue;
                        for (int di = -r; di <= r; di++)
                            for (int dj = -r; dj <= r; dj++)
                                for (int dk = -r; dk <= r; dk++)
                                {
                                    if (Math.Max(Math.Max(Math.Abs(di), Math.Abs(dj)), Math.Abs(dk)) != r) continue;
                                    double nx = x + di * cell, ny = y + dj * cell, nz = z + dk * cell;
                                    if (!InGrid(nx, ny, nz) || CellBlocked(nx, ny, nz)) continue;
                                    double d = di * di + dj * dj + dk * dk;
                                    if (d < bd) { bd = d; best = (nx, ny, nz); }
                                }
                        if (best != null) return best.Value;
                    }
                    return (x, y, z);
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
                    (sx, sy, sz) = Lift(sx, sy, sz, Face("EQUIP", t));
                    (sx, sy, sz) = SnapFree(sx, sy, sz, Face("EQUIP", t));
                    var (gx, gy, gz) = Lift(t.Gx, t.Gy, t.Gz, Face("DUCT", t));
                    (gx, gy, gz) = SnapFree(gx, gy, gz, Face("DUCT", t));
                    eng.AddTask(sx, sy, sz, gx, gy, gz, t.Utility, t.Group);
                }
            }
            catch (Exception ex) { return $"{label}: BUILD-EXCEPTION {ex.Message}"; }

            var sw = System.Diagnostics.Stopwatch.StartNew();
            int cbCount = 0, cbFail = 0;  // 진행 콜백 검증(다이얼로그와 동일 경로).
            var failExp = new List<long>();  // 실패 배관의 확장수(상한 도달=12M 근처 / 막힘=작은 값 구분).
            try
            {
                if (mode == "multi")
                    eng.RouteMultiProgress("longest", p => { if (p.Phase == 1) { cbCount++; if (!p.Success) { cbFail++; failExp.Add(p.ExpandedNodes); } } });
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
            string cb = mode == "multi" ? $" [progress cb {cbCount}, fail {cbFail}]" : "";
            string fe = failExp.Count > 0 ? $" failExpanded=[{string.Join(",", failExp)}]" : "";
            return $"{label}: success {ok}/{rows.Count} totalLen {tot:0} ({sw.ElapsedMilliseconds} ms){cb}{fe}";
        }
    }
}

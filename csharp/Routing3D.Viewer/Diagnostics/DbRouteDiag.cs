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
            int[]? rackLevels = null;   // L3a 랙 z-셀(측정에 재사용 위해 try 밖 선언).
            Engine eng;
            try
            {
                eng = new Engine();
                eng.SetGrid(cell, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
                int clr = wClear > 0 ? 2 : 0;
                // 기존설계 회랑(L2b) — R3D_CORRIDOR=on 이면 회랑 밖 가산(GUI UseDesignCorridor 와 동일).
                bool useCorr = string.Equals(Environment.GetEnvironmentVariable("R3D_CORRIDOR"), "on",
                                             StringComparison.OrdinalIgnoreCase);
                // L2b 속도 튜닝 노브(헤드리스 A/B). 미설정이면 기존 기본값(0.5 / 1.5 / 2).
                //   R3D_WCORR  = 회랑 밖 가산 = cell × 이 배수 (작을수록 탐색 폭 축소·설계추종 약화)
                //   R3D_WHEUR  = weighted A* 휴리스틱 가중(클수록 목표지향·확장 급감·약간 비최적)
                //   R3D_CORRRAD= 회랑 팽창 반경(셀, 튜브 폭). 넓을수록 경로가 회랑 안에 머물러 가산 회피.
                double wcMul = ParseEnv("R3D_WCORR", 0.5);
                double wHeur = ParseEnv("R3D_WHEUR", 2.0);   // L2b 속도튜닝: 1.5→2.0(회랑 47s→6.3s, +1성공)
                int corrRad = (int)ParseEnv("R3D_CORRRAD", 2);
                // 유틸그룹 랙 번들링(L3a) — R3D_RACK=on 이면 학습된 그룹 랙 z-셀을 rack_levels(면제)로.
                bool useRack = string.Equals(Environment.GetEnvironmentVariable("R3D_RACK"), "on",
                                             StringComparison.OrdinalIgnoreCase);
                rackLevels = BuildRackLevels(sd, rows);   // 항상 학습(집중도 지표에 재사용). 적용은 useRack 일 때만.
                // 회랑은 cell*0.5(R3D_WCORR), 랙-단독은 부드러운 cell*0.2(GUI 와 동일). 동시 ON 이면 회랑 우선.
                double wCorr = useCorr ? cell * wcMul
                             : useRack ? cell * ParseEnv("R3D_WCORR", 0.2) : 0.0;
                eng.SetParams(cell, 500, wClear, clr, 6, wCorridor: wCorr, corridorRadius: corrRad,
                              rackLevels: useRack ? rackLevels : null, wHeur: wHeur);
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

                // 기존설계 회랑 시드(L2b) — 매칭 기존배관 폴리라인을 셀로 복셀화(±2)해 주입.
                if (useCorr)
                {
                    var set = new HashSet<(int, int, int)>();
                    foreach (var t in rows)
                    {
                        var pipe = MatchPipe(sd, t, cell);
                        if (pipe == null || pipe.Points.Count < 2) continue;
                        for (int i = 1; i < pipe.Points.Count; i++)
                        {
                            var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                            double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                            double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                            int steps = Math.Max(1, (int)(len / (cell * 0.5)));
                            for (int sIdx = 0; sIdx <= steps; sIdx++)
                            {
                                double tt = (double)sIdx / steps;
                                int ci = (int)Math.Floor((a.X + dx * tt - g.Ox) / cell);
                                int cj = (int)Math.Floor((a.Y + dy * tt - g.Oy) / cell);
                                int ck = (int)Math.Floor((a.Z + dz * tt - g.Oz) / cell);
                                for (int di = -corrRad; di <= corrRad; di++)
                                    for (int dj = -corrRad; dj <= corrRad; dj++)
                                        for (int dk = -corrRad; dk <= corrRad; dk++)
                                        {
                                            int ii = ci + di, jj = cj + dj, kk = ck + dk;
                                            if (ii < 0 || jj < 0 || kk < 0 || ii >= g.Nx || jj >= g.Ny || kk >= g.Nz) continue;
                                            set.Add((ii, jj, kk));
                                        }
                            }
                        }
                    }
                    var arr = new int[set.Count * 3]; int nn = 0;
                    foreach (var (i, j, k) in set) { arr[nn++] = i; arr[nn++] = j; arr[nn++] = k; }
                    eng.SetCorridorCells(arr);
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
            // 랙 집중도(L3a) — 성공 경로의 수평 이동 셀 중 학습된 랙 z-셀에 놓인 비율. 번들링 강화 시 증가.
            var rackSet = rackLevels != null ? new HashSet<int>(rackLevels) : null;
            long horizCells = 0, rackCells = 0;
            for (int i = 0; i < rows.Count; i++)
            {
                try
                {
                    var r = eng.GetResult(i);
                    if (!r.Success) continue;
                    ok++; tot += r.LengthMm;
                    if (rackSet != null)
                        for (int p = 1; p < r.Path.Length; p++)
                        {
                            var a = r.Path[p - 1]; var b = r.Path[p];
                            if (a.K != b.K) continue;            // 수직 이동(라이저) 제외 — 수평 셀만 집계.
                            horizCells++;
                            if (rackSet.Contains(b.K)) rackCells++;
                        }
                }
                catch { }
            }
            eng.Dispose();
            string cb = mode == "multi" ? $" [progress cb {cbCount}, fail {cbFail}]" : "";
            string fe = failExp.Count > 0 ? $" failExpanded=[{string.Join(",", failExp)}]" : "";
            string rk = (rackSet != null && horizCells > 0)
                ? $" rackZ={rackCells * 100.0 / horizCells:0.0}% (z셀 {string.Join(",", rackLevels!)})" : "";
            return $"{label}: success {ok}/{rows.Count} totalLen {tot:0} ({sw.ElapsedMilliseconds} ms){cb}{fe}{rk}";
        }

        // 유틸그룹 랙 번들링(L3a) — rows 의 그룹에 속한 기존배관 수평 런의 z-셀(랙 높이)을 학습.
        // (GUI SceneViewModel.BuildRackLevels 미러). 반환: 런 큰 순 최대 8개 z-셀 인덱스, 없으면 null.
        static int[]? BuildRackLevels(SceneData sd, List<TaskInfo> rows)
        {
            var g = sd.Grid; double cell = g.CellMm;
            if (sd.ExistingPipes.Count == 0) return null;
            const double MinRunMm = 800.0, HorizTol = 0.34, GroupShare = 0.15;
            var groups = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var t in rows) if (!string.IsNullOrEmpty(t.Group)) groups.Add(t.Group!);
            if (groups.Count == 0) return null;

            var byGroup = new Dictionary<string, Dictionary<int, double>>(StringComparer.OrdinalIgnoreCase);
            foreach (var pipe in sd.ExistingPipes)
            {
                if (pipe.Group == null || !groups.Contains(pipe.Group) || pipe.Points.Count < 2) continue;
                if (!byGroup.TryGetValue(pipe.Group, out var zmap))
                    byGroup[pipe.Group] = zmap = new Dictionary<int, double>();
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double horiz = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
                    if (horiz <= 1e-6 || Math.Abs(b.Z - a.Z) > HorizTol * horiz) continue;
                    double len = Math.Sqrt(horiz * horiz + (b.Z - a.Z) * (b.Z - a.Z));
                    if (len < MinRunMm) continue;
                    int zk = (int)Math.Floor(((a.Z + b.Z) / 2 - g.Oz) / cell);
                    if (zk < 0 || zk >= g.Nz) continue;
                    zmap[zk] = (zmap.TryGetValue(zk, out var v) ? v : 0.0) + len;
                }
            }
            if (byGroup.Count == 0) return null;
            var picked = new Dictionary<int, double>();
            foreach (var kv in byGroup)
            {
                double totRun = 0; foreach (var v in kv.Value.Values) totRun += v;
                if (totRun <= 0) continue;
                foreach (var zr in kv.Value)
                    if (zr.Value >= GroupShare * totRun)
                        picked[zr.Key] = (picked.TryGetValue(zr.Key, out var p) ? p : 0.0) + zr.Value;
            }
            if (picked.Count == 0) return null;
            var top = picked.OrderByDescending(kv => kv.Value).Take(8).Select(kv => kv.Key).ToArray();
            return top.Length > 0 ? top : null;
        }

        // 작업(TaskInfo) ↔ 기존 설계배관(TB_ROUTE_PATH) 매칭 — 양 끝 PoC 거리 합 최소(양방향), 임계 내. (GUI FindMatchingExistingPipe 미러)
        static ExistingPipe? MatchPipe(SceneData sd, TaskInfo t, double cell)
        {
            if (sd.ExistingPipes.Count == 0) return null;
            double tol = Math.Max(3 * cell, 1500.0) * 2;
            ExistingPipe? best = null; double bestScore = double.MaxValue;
            foreach (var p in sd.ExistingPipes)
            {
                if (p.Points.Count < 2) continue;
                var ps = p.SourcePos ?? p.Points[0];
                var pe = p.TargetPos ?? p.Points[p.Points.Count - 1];
                double s1 = D(t.Sx, t.Sy, t.Sz, ps) + D(t.Gx, t.Gy, t.Gz, pe);
                double s2 = D(t.Sx, t.Sy, t.Sz, pe) + D(t.Gx, t.Gy, t.Gz, ps);
                double score = Math.Min(s1, s2);
                if (score < bestScore) { bestScore = score; best = p; }
            }
            return (best != null && bestScore <= tol) ? best : null;
        }

        static double D(double x, double y, double z, Pt3 p)
        {
            double dx = x - p.X, dy = y - p.Y, dz = z - p.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        // 환경변수를 double 로 파싱(미설정/형식오류면 기본값). L2b 속도 튜닝 노브용.
        static double ParseEnv(string name, double dflt)
        {
            var v = Environment.GetEnvironmentVariable(name);
            return double.TryParse(v, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var d) ? d : dflt;
        }
    }
}

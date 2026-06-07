// AI 자동설계 비교 리포트 (헤드리스) — 메인장비별 · (장비,유틸리티그룹)별
// =============================================================================
// [실행]
//   Routing3D.Viewer.exe --autodesign-report <projectId> <cellMm> <outDir>
//   예: --autodesign-report 1 100 d:\tmp\adr
//
// [무엇을 하나]
//   한 툴 그룹(프로젝트)의 작업을 (장비, 유틸리티 그룹) '케이스'로 묶고, 각 케이스에 대해
//   자동설계 2전략(① 최단=순수 A* · ② Stub+그룹패턴=스텁+번들/랙)을 수행한 뒤,
//   사람이 설계한 '기존설계'와 함께 길이·꺾임·그룹핑 Factor 를 산출해 CSV/TXT 리포트로 낸다.
//   (3D 스냅샷 임베드[docx/pdf]는 후속 P4 — 본 단계는 지표·데이터 백본.)
//
// [그룹핑 Factor]  0~1 복합 — 여러 배관이 '다발로 깔린 정도'.
//   = 0.6×랙집중도 + 0.4×번들밀집도   (각 [0,1]; 계획서 4성분 중 pitch/lane 은 후속)
//     · 랙집중도 = 성공 경로 수평셀 중 학습 랙 z-셀 비율
//     · 번들밀집도 = 경로 쌍에서 한 배관 셀이 다른 배관 셀 ±2 안에 드는 평균 비율
//
// [재사용]  엔진(C ABI)·ObstacleDbLoader·DbRouteDiag 의 순수 헬퍼(MatchPipe·BuildRackLevels·
//   MergeBundle·BuildBundleCorridor·D)·StubExtractor·PatternStore/BundleStore.
//   PoC 도달성(Lift/SnapFree)·셀 막힘 판정은 DbRouteDiag 와 동일 규약의 컴팩트 구현(아래).
// =============================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Routing3D.Viewer.Interop;
using Routing3D.Viewer.Model;

namespace Routing3D.Viewer.Diagnostics
{
    public static class AutoDesignReport
    {
        // 자동설계 전략 — 엔진 옵션 조합.
        private enum Strategy { Shortest, StubGroup }

        // 한 (전략 또는 기존설계)의 정량 지표.
        private sealed class Metrics
        {
            public int Ok, N;
            public double TotalLenMm;
            public long TotalTurns;
            public double RackZPct = -1;     // -1 = 측정 불가(랙 미학습/수평셀 0).
            public double DensityPct = -1;   // -1 = 측정 불가(배관 <2 또는 >40).
            public int StubMatched;
            public double AvgTurns => Ok > 0 ? (double)TotalTurns / Ok : 0;
            public double AvgLenMm => Ok > 0 ? TotalLenMm / Ok : 0;
            // 그룹핑 Factor — 가용 성분(랙집중·번들밀집)의 가중 평균. 둘 다 없으면 -1.
            public double GroupingFactor
            {
                get
                {
                    double rz = RackZPct >= 0 ? RackZPct / 100.0 : -1;
                    double de = DensityPct >= 0 ? DensityPct / 100.0 : -1;
                    if (rz < 0 && de < 0) return -1;
                    if (rz < 0) return de;
                    if (de < 0) return rz;
                    return 0.6 * rz + 0.4 * de;
                }
            }
        }

        // 케이스 = (메인장비, 장비, 유틸리티그룹) + 그 작업들.
        private sealed class Case
        {
            public string MainEquip = "(미상)";
            public string Equip = "(미상)";
            public string UtilGroup = "(미상)";
            public List<TaskInfo> Tasks = new();
            public Metrics Existing = new();
            public Metrics Shortest = new();
            public Metrics StubGroup = new();
        }

        public static string Run(int projectId, double cellMm, string outDir, int maxCases = 0)
        {
            var cfg = DbConfig.FromEnv();
            SceneData sd;
            try { sd = ObstacleDbLoader.LoadScene(cfg, projectId, cellMm); }
            catch (Exception ex) { return "LOAD ERROR: " + ex; }

            Directory.CreateDirectory(outDir);
            var g = sd.Grid;
            var log = new StringBuilder();
            log.AppendLine($"AI 자동설계 비교 리포트 — project {projectId} ({sd.SourceFile})");
            log.AppendLine($"격자 {g.Nx}x{g.Ny}x{g.Nz} cell={g.CellMm:0} · 장애물 {sd.Obstacles.Count} · 장비 {sd.Equipment.Count} · 작업 {sd.Tasks.Count} · 기존배관 {sd.ExistingPipes.Count}");

            // Stub+그룹 전략용 학습 자산(없으면 자연 폴백).
            PatternStore? patterns = PatternStore.TryLoad(cfg);
            BundleStore? bundles = BundleStore.TryLoad(cfg);
            log.AppendLine($"학습: 패턴 {(patterns == null ? "없음" : patterns.Count + "키")} · 번들 {(bundles == null ? "없음" : bundles.Count + "키")}");

            var cases = BuildCases(sd);
            if (maxCases > 0 && cases.Count > maxCases) cases = cases.Take(maxCases).ToList();   // 스모크 테스트용 제한.
            log.AppendLine($"케이스(장비×유틸그룹): {cases.Count}");
            log.AppendLine();

            int idx = 0;
            foreach (var c in cases)
            {
                idx++;
                // 기존설계: 작업↔기존배관 매칭 폴리라인의 길이·꺾임 + 래스터화 셀로 그룹핑.
                c.Existing = ExistingMetrics(sd, c.Tasks);
                // 자동설계 2전략.
                c.Shortest = RunStrategy(sd, c.Tasks, Strategy.Shortest, null, null);
                c.StubGroup = RunStrategy(sd, c.Tasks, Strategy.StubGroup, patterns, bundles);
                log.AppendLine($"[{idx}/{cases.Count}] {c.MainEquip} / {c.Equip} / {c.UtilGroup} (작업 {c.Tasks.Count})"
                    + $" | 기존 len {c.Existing.TotalLenMm:0} turns {c.Existing.TotalTurns} GF {Fmt(c.Existing.GroupingFactor)}"
                    + $" | 최단 {c.Shortest.Ok}/{c.Shortest.N} len {c.Shortest.TotalLenMm:0} turns {c.Shortest.TotalTurns} GF {Fmt(c.Shortest.GroupingFactor)}"
                    + $" | Stub+그룹 {c.StubGroup.Ok}/{c.StubGroup.N} len {c.StubGroup.TotalLenMm:0} turns {c.StubGroup.TotalTurns} GF {Fmt(c.StubGroup.GroupingFactor)}");
            }

            string baseName = Sanitize(string.IsNullOrEmpty(sd.SourceFile) ? $"proj{projectId}" : sd.SourceFile);
            string csvPath = Path.Combine(outDir, baseName + "_autodesign_report.csv");
            string txtPath = Path.Combine(outDir, baseName + "_autodesign_report.txt");
            WriteCsv(csvPath, cases);
            File.WriteAllText(txtPath, BuildTextReport(projectId, sd, cases, log.ToString()), new UTF8Encoding(true));

            log.AppendLine();
            log.AppendLine("저장:");
            log.AppendLine("  " + csvPath);
            log.AppendLine("  " + txtPath);
            return log.ToString();
        }

        // ---- 케이스 구성 — (장비, 유틸리티그룹) 묶음, 메인장비 태깅 ----
        private static List<Case> BuildCases(SceneData sd)
        {
            var byKey = new Dictionary<(string, string), Case>();
            foreach (var t in sd.Tasks)
            {
                string equip = string.IsNullOrEmpty(t.PocName) ? "(미상)" : t.PocName!;
                string grp = string.IsNullOrEmpty(t.Group) ? "(미상)" : t.Group!;
                var key = (equip, grp);
                if (!byKey.TryGetValue(key, out var c))
                {
                    c = new Case { Equip = equip, UtilGroup = grp, MainEquip = MainEquipOf(sd, t, equip) };
                    byKey[key] = c;
                }
                c.Tasks.Add(t);
            }
            return byKey.Values
                .OrderBy(c => c.MainEquip, StringComparer.OrdinalIgnoreCase)
                .ThenBy(c => c.Equip, StringComparer.OrdinalIgnoreCase)
                .ThenBy(c => c.UtilGroup, StringComparer.OrdinalIgnoreCase)
                .ToList();
        }

        // 작업의 메인장비 = 시작 PoC 를 포함/최근접하는 IsMain 장비명. 없으면 장비명 그대로.
        private static string MainEquipOf(SceneData sd, TaskInfo t, string fallback)
        {
            EquipmentBox? best = null; double bd = double.MaxValue;
            foreach (var e in sd.Equipment)
            {
                if (!e.IsMain) continue;
                bool inside = t.Sx >= e.MinX - 1 && t.Sx <= e.MaxX + 1 && t.Sy >= e.MinY - 1 && t.Sy <= e.MaxY + 1
                              && t.Sz >= e.MinZ - 1 && t.Sz <= e.MaxZ + 1;
                if (inside) return string.IsNullOrEmpty(e.Name) ? fallback : e.Name;
                double cx = (e.MinX + e.MaxX) / 2, cy = (e.MinY + e.MaxY) / 2, cz = (e.MinZ + e.MaxZ) / 2;
                double d = (t.Sx - cx) * (t.Sx - cx) + (t.Sy - cy) * (t.Sy - cy) + (t.Sz - cz) * (t.Sz - cz);
                if (d < bd) { bd = d; best = e; }
            }
            return best != null && !string.IsNullOrEmpty(best.Name) ? best.Name : fallback;
        }

        // ---- 전략 실행 — 엔진 구성 + route_multi + 지표 ----
        private static Metrics RunStrategy(SceneData sd, List<TaskInfo> rows, Strategy strat,
                                           PatternStore? patterns, BundleStore? bundles)
        {
            var m = new Metrics { N = rows.Count };
            if (rows.Count == 0) return m;
            var g = sd.Grid; double cell = g.CellMm;
            bool stub = strat == Strategy.StubGroup;
            // 스텁 전략은 엔진이 '스텁끝~스텁끝'만 탐색하므로, 고정 스텁(라이저+엘보) 길이/꺾임을
            //   따로 더해야 기존/최단과 길이·꺾임을 공정 비교할 수 있다. task i = row i(행당 AddTask 1회).
            var stubAddLen = new double[rows.Count];
            var stubAddTurn = new int[rows.Count];

            int[]? rackLevels = DbRouteDiag.BuildRackLevels(sd, rows);   // 측정용 항상 학습.
            int[]? appliedRack = null;
            HashSet<(int, int, int)>? bundleSet = null;
            if (stub)
            {
                appliedRack = rackLevels;
                if (bundles != null) appliedRack = DbRouteDiag.MergeBundle(sd, rows, appliedRack, bundles);
                if (bundles != null) bundleSet = DbRouteDiag.BuildBundleCorridor(sd, rows, 2, bundles);
            }
            bool hasBundleCorr = bundleSet != null && bundleSet.Count > 0;
            double wCorr = hasBundleCorr ? cell * 0.5 : (appliedRack != null && appliedRack.Length > 0 ? cell * 0.2 : 0.0);

            Engine eng;
            try
            {
                eng = new Engine();
                eng.SetGrid(cell, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
                eng.SetParams(cell, 500, 10, 2, 6, wCorridor: wCorr, corridorRadius: 2,
                              rackLevels: appliedRack, wHeur: 2.0, wHeurNear: 1.0);
                foreach (var o in sd.Obstacles)
                    if (o.IsPassThrough) eng.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                    else eng.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                AddFacilities(sd, eng, cell);

                for (int ti = 0; ti < rows.Count; ti++)
                {
                    var t = rows[ti];
                    if (stub)
                    {
                        var pipe = DbRouteDiag.MatchPipe(sd, t, cell);
                        if (pipe != null)
                        {
                            var (srcStub, tgtStub) = StubExtractor.ForPipe(pipe);
                            var ps = pipe.SourcePos ?? pipe.Points[0];
                            var pe = pipe.TargetPos ?? pipe.Points[pipe.Points.Count - 1];
                            bool fwd = DbRouteDiag.D(t.Sx, t.Sy, t.Sz, ps) + DbRouteDiag.D(t.Gx, t.Gy, t.Gz, pe)
                                       <= DbRouteDiag.D(t.Sx, t.Sy, t.Sz, pe) + DbRouteDiag.D(t.Gx, t.Gy, t.Gz, ps);
                            var ss = fwd ? srcStub : tgtStub;
                            var es = fwd ? tgtStub : srcStub;
                            if (ss.Count >= 2 && es.Count >= 2)
                            {
                                var se = ss[ss.Count - 1]; var ee = es[es.Count - 1];
                                var (bx, by, bz) = SnapFree(sd, g, cell, se.X, se.Y, se.Z);
                                var (cx, cy, cz) = SnapFree(sd, g, cell, ee.X, ee.Y, ee.Z);
                                eng.AddTask(bx, by, bz, cx, cy, cz, t.Utility, t.Group);
                                m.StubMatched++;
                                stubAddLen[ti] = PolyLen(ss) + PolyLen(es);     // 고정 스텁 길이.
                                stubAddTurn[ti] = OrthoBends(ss) + OrthoBends(es) + 2;  // 스텁 엘보 + 스텁↔A* 접합 2.
                                continue;
                            }
                        }
                    }
                    // 시작 PoC: 장비 바닥으로 드롭(시작이 장비 박스 안일 때) + 표면 투영 + 빈셀 스냅.
                    double sx = t.Sx, sy = t.Sy, sz = t.Sz;
                    double lowest = double.NaN;
                    foreach (var eq in sd.Equipment)
                        if (sx >= eq.MinX - 1 && sx <= eq.MaxX + 1 && sy >= eq.MinY - 1 && sy <= eq.MaxY + 1 &&
                            sz >= eq.MinZ - 1 && sz <= eq.MaxZ + 1 && (double.IsNaN(lowest) || eq.MinZ < lowest))
                            lowest = eq.MinZ;
                    if (!double.IsNaN(lowest)) sz = Math.Max(g.Oz + cell * 0.5, lowest - cell * 0.5);
                    string? eqFace = patterns != null && patterns.TryGet("EQUIP", t.Group, t.Utility, out var tpE) ? tpE.Face : null;
                    string? dcFace = patterns != null && patterns.TryGet("DUCT", t.Group, t.Utility, out var tpD) ? tpD.Face : null;
                    (sx, sy, sz) = Lift(sd, g, cell, sx, sy, sz, eqFace);
                    (sx, sy, sz) = SnapFree(sd, g, cell, sx, sy, sz);
                    var (gx, gy, gz) = Lift(sd, g, cell, t.Gx, t.Gy, t.Gz, dcFace);
                    (gx, gy, gz) = SnapFree(sd, g, cell, gx, gy, gz);
                    eng.AddTask(sx, sy, sz, gx, gy, gz, t.Utility, t.Group);
                }

                if (hasBundleCorr)
                {
                    var arr = new int[bundleSet!.Count * 3]; int nn = 0;
                    foreach (var (i, j, k) in bundleSet) { arr[nn++] = i; arr[nn++] = j; arr[nn++] = k; }
                    eng.SetCorridorCells(arr);
                }
            }
            catch (Exception) { return m; }

            try { eng.RouteMultiProgress("longest", _ => { }); }
            catch (Exception) { eng.Dispose(); return m; }

            var paths = new List<PathCell[]>();
            for (int i = 0; i < rows.Count; i++)
            {
                try
                {
                    var r = eng.GetResult(i);
                    if (!r.Success) continue;
                    m.Ok++; m.TotalLenMm += r.LengthMm + stubAddLen[i]; m.TotalTurns += r.Turns + stubAddTurn[i];
                    if (r.Path.Length >= 2) paths.Add(r.Path);
                }
                catch { }
            }
            eng.Dispose();
            (m.RackZPct, m.DensityPct) = GroupMetrics(paths, rackLevels, g);
            return m;
        }

        // ---- 기존설계 지표 — 매칭 폴리라인의 길이·꺾임 + 래스터화 셀로 그룹핑 ----
        private static Metrics ExistingMetrics(SceneData sd, List<TaskInfo> rows)
        {
            var m = new Metrics { N = rows.Count };
            var g = sd.Grid; double cell = g.CellMm;
            int[]? rackLevels = DbRouteDiag.BuildRackLevels(sd, rows);
            var paths = new List<PathCell[]>();
            foreach (var t in rows)
            {
                var pipe = DbRouteDiag.MatchPipe(sd, t, cell);
                if (pipe == null || pipe.Points.Count < 2) continue;
                m.Ok++;
                m.TotalLenMm += PolyLen(pipe.Points);
                m.TotalTurns += OrthoBends(pipe.Points);
                var cells = Rasterize(pipe.Points, g);
                if (cells.Length >= 2) paths.Add(cells);
            }
            (m.RackZPct, m.DensityPct) = GroupMetrics(paths, rackLevels, g);
            return m;
        }

        // ---- 그룹핑 성분 — 랙집중도%·번들밀집도% (DbRouteDiag 와 동일 정의) ----
        private static (double rackZPct, double densityPct) GroupMetrics(List<PathCell[]> paths, int[]? rackLevels, GridMeta g)
        {
            double rackZ = -1;
            if (rackLevels != null && rackLevels.Length > 0 && paths.Count > 0)
            {
                var rackSet = new HashSet<int>(rackLevels);
                long horiz = 0, onRack = 0;
                foreach (var p in paths)
                    for (int i = 1; i < p.Length; i++)
                    {
                        var a = p[i - 1]; var b = p[i];
                        if (a.K != b.K) continue;        // 수평 셀만.
                        horiz++;
                        if (rackSet.Contains(b.K)) onRack++;
                    }
                if (horiz > 0) rackZ = onRack * 100.0 / horiz;
            }

            double density = -1;
            if (paths.Count >= 2 && paths.Count <= 40)
            {
                var sets = new List<HashSet<long>>();
                foreach (var p in paths)
                {
                    var hs = new HashSet<long>();
                    foreach (var c in p) hs.Add(((long)c.I << 40) ^ ((long)c.J << 20) ^ c.K);
                    sets.Add(hs);
                }
                double sum = 0; int pairs = 0;
                for (int i = 0; i < sets.Count; i++)
                    for (int j = 0; j < sets.Count; j++)
                    {
                        if (i == j) continue;
                        int near = 0;
                        foreach (var key in sets[i])
                        {
                            int ci = (int)((key >> 40) & 0xFFFFF), cj = (int)((key >> 20) & 0xFFFFF), ck = (int)(key & 0xFFFFF);
                            bool hit = false;
                            for (int di = -2; di <= 2 && !hit; di++)
                                for (int dj = -2; dj <= 2 && !hit; dj++)
                                    if (sets[j].Contains(((long)(ci + di) << 40) ^ ((long)(cj + dj) << 20) ^ ck)) hit = true;
                            if (hit) near++;
                        }
                        sum += sets[i].Count > 0 ? (double)near / sets[i].Count : 0; pairs++;
                    }
                density = pairs > 0 ? sum / pairs * 100.0 : -1;
            }
            return (rackZ, density);
        }

        // ---- 컴팩트 PoC 도달성(DbRouteDiag 규약 미러) ----
        private static void AddFacilities(SceneData sd, Engine eng, double cell)
        {
            void Box(double a, double b, double c, double d, double e2, double f2)
            {
                if (d - a < cell) { double mm = (a + d) / 2; a = mm - cell / 2; d = mm + cell / 2; }
                if (e2 - b < cell) { double mm = (b + e2) / 2; b = mm - cell / 2; e2 = mm + cell / 2; }
                if (f2 - c < cell) { double mm = (c + f2) / 2; c = mm - cell / 2; f2 = mm + cell / 2; }
                eng.AddObstacle(a, b, c, d, e2, f2);
            }
            foreach (var eq in sd.Equipment) Box(eq.MinX, eq.MinY, eq.MinZ, eq.MaxX, eq.MaxY, eq.MaxZ);
            foreach (var dl in sd.DuctsLaterals) Box(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ);
        }

        private static (double, double, double) Lift(SceneData sd, GridMeta g, double cell,
                                                     double x, double y, double z, string? face)
        {
            double eps = 1.0, m = cell * 0.5;
            void TryBox(double bx0, double by0, double bz0, double bx1, double by1, double bz1)
            {
                if (x <= bx0 - eps || x >= bx1 + eps) return;
                if (y <= by0 - eps || y >= by1 + eps) return;
                if (z <= bz0 - eps || z >= bz1 + eps) return;
                if (face != null)
                {
                    switch (face)
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
                foreach (var eq in sd.Equipment) TryBox(eq.MinX, eq.MinY, eq.MinZ, eq.MaxX, eq.MaxY, eq.MaxZ);
                foreach (var dl in sd.DuctsLaterals) TryBox(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ);
                if (px == x && py == y && pz == z) break;
            }
            if (z < g.Oz + m) z = g.Oz + m;
            return (x, y, z);
        }

        private static bool InGrid(GridMeta g, double cell, double x, double y, double z) =>
            x >= g.Ox && y >= g.Oy && z >= g.Oz &&
            x <= g.Ox + g.Nx * cell && y <= g.Oy + g.Ny * cell && z <= g.Oz + g.Nz * cell;

        private static bool CellBlocked(SceneData sd, GridMeta g, double cell, double x, double y, double z)
        {
            double minT = cell;
            int ci = (int)Math.Floor((x - g.Ox) / cell), cj = (int)Math.Floor((y - g.Oy) / cell), ck = (int)Math.Floor((z - g.Oz) / cell);
            double clx = g.Ox + ci * cell, chx = clx + cell, cly = g.Oy + cj * cell, chy = cly + cell, clz = g.Oz + ck * cell, chz = clz + cell;
            bool Ov(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            {
                if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
                if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
                if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
                return clx < mxx && chx > mnx && cly < mxy && chy > mny && clz < mxz && chz > mnz;
            }
            foreach (var o in sd.Obstacles)
                if (!o.IsPassThrough && Ov(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ)) return true;
            foreach (var e in sd.Equipment) if (Ov(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ)) return true;
            foreach (var dl in sd.DuctsLaterals) if (Ov(dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ)) return true;
            return false;
        }

        private static (double, double, double) SnapFree(SceneData sd, GridMeta g, double cell, double x, double y, double z)
        {
            if (!CellBlocked(sd, g, cell, x, y, z)) return (x, y, z);
            for (int r = 1; r <= 6; r++)
            {
                (double, double, double)? best = null; double bd = double.MaxValue;
                for (int di = -r; di <= r; di++)
                    for (int dj = -r; dj <= r; dj++)
                        for (int dk = -r; dk <= r; dk++)
                        {
                            if (Math.Max(Math.Max(Math.Abs(di), Math.Abs(dj)), Math.Abs(dk)) != r) continue;
                            double nx = x + di * cell, ny = y + dj * cell, nz = z + dk * cell;
                            if (!InGrid(g, cell, nx, ny, nz) || CellBlocked(sd, g, cell, nx, ny, nz)) continue;
                            double d = di * di + dj * dj + dk * dk;
                            if (d < bd) { bd = d; best = (nx, ny, nz); }
                        }
                if (best != null) return best.Value;
            }
            return (x, y, z);
        }

        // ---- 기하 보조 ----
        private static double PolyLen(IReadOnlyList<Pt3> pts)
        {
            double s = 0;
            for (int i = 1; i < pts.Count; i++)
            {
                double dx = pts[i].X - pts[i - 1].X, dy = pts[i].Y - pts[i - 1].Y, dz = pts[i].Z - pts[i - 1].Z;
                s += Math.Sqrt(dx * dx + dy * dy + dz * dz);
            }
            return s;
        }

        // 폴리라인의 직교 꺾임 수(진행 우세축 전환). 비직교 세그먼트는 우세축으로 스냅.
        private static int OrthoBends(IReadOnlyList<Pt3> pts)
        {
            int Axis(Pt3 a, Pt3 b)
            {
                double dx = Math.Abs(b.X - a.X), dy = Math.Abs(b.Y - a.Y), dz = Math.Abs(b.Z - a.Z);
                return dz >= dx && dz >= dy ? 2 : (dx >= dy ? 0 : 1);
            }
            int bends = 0, prev = -1;
            for (int i = 1; i < pts.Count; i++)
            {
                double dx = pts[i].X - pts[i - 1].X, dy = pts[i].Y - pts[i - 1].Y, dz = pts[i].Z - pts[i - 1].Z;
                if (dx * dx + dy * dy + dz * dz < 1e-6) continue;
                int a = Axis(pts[i - 1], pts[i]);
                if (prev >= 0 && a != prev) bends++;
                prev = a;
            }
            return bends;
        }

        // 월드 폴리라인 → 6-연결 셀 경로(인접 보장, 중복 제거). 그룹핑 성분 측정용.
        private static PathCell[] Rasterize(IReadOnlyList<Pt3> pts, GridMeta g)
        {
            double cell = g.CellMm;
            PathCell ToCell(Pt3 p) => new(
                Math.Clamp((int)Math.Floor((p.X - g.Ox) / cell), 0, g.Nx - 1),
                Math.Clamp((int)Math.Floor((p.Y - g.Oy) / cell), 0, g.Ny - 1),
                Math.Clamp((int)Math.Floor((p.Z - g.Oz) / cell), 0, g.Nz - 1));
            var outp = new List<PathCell>();
            void Add(PathCell c) { if (outp.Count == 0 || !outp[outp.Count - 1].Equals(c)) outp.Add(c); }
            if (pts.Count > 0) Add(ToCell(pts[0]));
            for (int i = 1; i < pts.Count; i++)
            {
                var a = ToCell(pts[i - 1]); var b = ToCell(pts[i]);
                // 축 순서대로 한 칸씩 이동(L 보간) → 6-연결.
                var cur = a;
                while (cur.I != b.I) { cur = new PathCell(cur.I + Math.Sign(b.I - cur.I), cur.J, cur.K); Add(cur); }
                while (cur.J != b.J) { cur = new PathCell(cur.I, cur.J + Math.Sign(b.J - cur.J), cur.K); Add(cur); }
                while (cur.K != b.K) { cur = new PathCell(cur.I, cur.J, cur.K + Math.Sign(b.K - cur.K)); Add(cur); }
            }
            return outp.ToArray();
        }

        // ---- 출력 ----
        private static void WriteCsv(string path, List<Case> cases)
        {
            var sb = new StringBuilder();
            sb.AppendLine("메인장비,장비,유틸리티그룹,작업수,"
                + "기존_총길이mm,기존_평균꺾임,기존_그룹핑F,"
                + "최단_성공,최단_총길이mm,최단_평균꺾임,최단_랙집중%,최단_번들밀집%,최단_그룹핑F,"
                + "Stub그룹_성공,Stub그룹_총길이mm,Stub그룹_평균꺾임,Stub그룹_랙집중%,Stub그룹_번들밀집%,Stub그룹_그룹핑F,Stub매칭");
            foreach (var c in cases)
            {
                sb.AppendLine(string.Join(",",
                    Q(c.MainEquip), Q(c.Equip), Q(c.UtilGroup), c.Tasks.Count,
                    F0(c.Existing.TotalLenMm), F1(c.Existing.AvgTurns), Fmt(c.Existing.GroupingFactor),
                    $"{c.Shortest.Ok}/{c.Shortest.N}", F0(c.Shortest.TotalLenMm), F1(c.Shortest.AvgTurns),
                    Pct(c.Shortest.RackZPct), Pct(c.Shortest.DensityPct), Fmt(c.Shortest.GroupingFactor),
                    $"{c.StubGroup.Ok}/{c.StubGroup.N}", F0(c.StubGroup.TotalLenMm), F1(c.StubGroup.AvgTurns),
                    Pct(c.StubGroup.RackZPct), Pct(c.StubGroup.DensityPct), Fmt(c.StubGroup.GroupingFactor),
                    c.StubGroup.StubMatched));
            }
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(true));   // BOM=Excel 한글.
        }

        private static string BuildTextReport(int projectId, SceneData sd, List<Case> cases, string log)
        {
            var sb = new StringBuilder();
            sb.AppendLine("================ AI 자동설계 비교 리포트 ================");
            sb.AppendLine($"프로젝트 {projectId} ({sd.SourceFile}) · 셀 {sd.Grid.CellMm:0}mm · 케이스 {cases.Count}");
            sb.AppendLine();
            // 전체 집계.
            Metrics Sum(Func<Case, Metrics> sel)
            {
                var t = new Metrics();
                foreach (var c in cases) { var x = sel(c); t.Ok += x.Ok; t.N += x.N; t.TotalLenMm += x.TotalLenMm; t.TotalTurns += x.TotalTurns; }
                return t;
            }
            var ex = Sum(c => c.Existing); var sh = Sum(c => c.Shortest); var sg = Sum(c => c.StubGroup);
            double GFavg(Func<Case, Metrics> sel) { var v = cases.Select(c => sel(c).GroupingFactor).Where(x => x >= 0).ToList(); return v.Count > 0 ? v.Average() : -1; }
            sb.AppendLine("[전체 집계]");
            sb.AppendLine($"  기존설계   : 매칭 {ex.Ok}, 총길이 {ex.TotalLenMm:N0}mm, 평균꺾임 {ex.AvgTurns:0.0}, 그룹핑F(평균) {Fmt(GFavg(c => c.Existing))}");
            sb.AppendLine($"  최단       : 성공 {sh.Ok}/{sh.N}, 총길이 {sh.TotalLenMm:N0}mm, 평균꺾임 {sh.AvgTurns:0.0}, 그룹핑F(평균) {Fmt(GFavg(c => c.Shortest))}");
            sb.AppendLine($"  Stub+그룹  : 성공 {sg.Ok}/{sg.N}, 총길이 {sg.TotalLenMm:N0}mm, 평균꺾임 {sg.AvgTurns:0.0}, 그룹핑F(평균) {Fmt(GFavg(c => c.StubGroup))}");
            sb.AppendLine();
            sb.AppendLine("[해석] 최단=길이/직선성 우위 기대, Stub+그룹=그룹핑F(다발화)·사람설계 추종 우위 기대.");
            sb.AppendLine();
            // 메인장비별.
            string? curMain = null;
            foreach (var c in cases)
            {
                if (c.MainEquip != curMain) { curMain = c.MainEquip; sb.AppendLine($"■ 메인장비: {curMain}"); }
                sb.AppendLine($"  · {c.Equip} / {c.UtilGroup} (작업 {c.Tasks.Count})");
                sb.AppendLine($"      기존     len {c.Existing.TotalLenMm,10:N0}  avgTurn {c.Existing.AvgTurns,5:0.0}  GF {Fmt(c.Existing.GroupingFactor)}");
                sb.AppendLine($"      최단     len {c.Shortest.TotalLenMm,10:N0}  avgTurn {c.Shortest.AvgTurns,5:0.0}  GF {Fmt(c.Shortest.GroupingFactor)}  성공 {c.Shortest.Ok}/{c.Shortest.N}  랙{Pct(c.Shortest.RackZPct)} 밀집{Pct(c.Shortest.DensityPct)}");
                sb.AppendLine($"      Stub+그룹 len {c.StubGroup.TotalLenMm,10:N0}  avgTurn {c.StubGroup.AvgTurns,5:0.0}  GF {Fmt(c.StubGroup.GroupingFactor)}  성공 {c.StubGroup.Ok}/{c.StubGroup.N}  랙{Pct(c.StubGroup.RackZPct)} 밀집{Pct(c.StubGroup.DensityPct)}  스텁 {c.StubGroup.StubMatched}");
            }
            sb.AppendLine();
            sb.AppendLine("[실행 로그]");
            sb.Append(log);
            return sb.ToString();
        }

        private static string Sanitize(string s)
        {
            foreach (var ch in Path.GetInvalidFileNameChars()) s = s.Replace(ch, '_');
            return s;
        }
        private static string Q(string s) => "\"" + (s ?? "").Replace("\"", "\"\"") + "\"";
        private static string F0(double v) => v.ToString("0", CultureInfo.InvariantCulture);
        private static string F1(double v) => v.ToString("0.0", CultureInfo.InvariantCulture);
        private static string Pct(double v) => v < 0 ? "N/A" : v.ToString("0.0", CultureInfo.InvariantCulture);
        private static string Fmt(double gf) => gf < 0 ? "N/A" : gf.ToString("0.000", CultureInfo.InvariantCulture);
    }
}

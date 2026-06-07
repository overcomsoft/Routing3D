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
using System.Windows.Media;
using System.Windows.Media.Media3D;
using Routing3D.Viewer.Interop;
using Routing3D.Viewer.Model;

namespace Routing3D.Viewer.Diagnostics
{
    public static class AutoDesignReport
    {
        // 그룹핑 Factor 가중(랙집중/번들밀집/pitch/lane). env R3D_ADR_W="0.35,0.30,0.20,0.15" 로 재정의(합 무관, 가용성분 재정규화).
        internal static readonly double[] GfWeights = ParseWeights();
        private static double[] ParseWeights()
        {
            var def = new[] { 0.35, 0.30, 0.20, 0.15 };
            var s = Environment.GetEnvironmentVariable("R3D_ADR_W");
            if (string.IsNullOrWhiteSpace(s)) return def;
            var parts = s.Split(',');
            if (parts.Length != 4) return def;
            var w = new double[4];
            for (int i = 0; i < 4; i++)
                if (!double.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out w[i]) || w[i] < 0)
                    return def;
            return w;
        }

        // 자동설계 전략 — 엔진 옵션 조합.
        private enum Strategy { Shortest, StubGroup }

        // 한 (전략 또는 기존설계)의 정량 지표.
        private sealed class Metrics
        {
            public int Ok, N;
            public double TotalLenMm;
            public long TotalTurns;
            public double RackZPct = -1;     // 랙집중도% — -1 = 측정 불가(랙 미학습/수평셀 0).
            public double DensityPct = -1;   // 번들밀집도% — -1 = 측정 불가(배관 <2 또는 >40).
            public double PitchPct = -1;     // pitch 일관성% (1−min(1,CV))×100 — -1 = 평행 레인<2.
            public double LanePct = -1;      // 레인 정렬도% (공용 z-고도 공유 배관 비율) — -1 = 배관<2.
            public int StubMatched;
            public double AvgTurns => Ok > 0 ? (double)TotalTurns / Ok : 0;
            public double AvgLenMm => Ok > 0 ? TotalLenMm / Ok : 0;
            // 그룹핑 Factor — 4성분(랙집중·번들밀집·pitch·lane)의 가중 평균. 가용 성분만 재정규화.
            //   가중은 GfWeights(기본 0.35/0.30/0.20/0.15, env R3D_ADR_W 로 재정의). 모두 N/A 면 -1.
            public double GroupingFactor
            {
                get
                {
                    double[] w = GfWeights;
                    double[] v = { RackZPct, DensityPct, PitchPct, LanePct };
                    double sw = 0, sv = 0;
                    for (int i = 0; i < 4; i++)
                        if (v[i] >= 0) { sw += w[i]; sv += w[i] * (v[i] / 100.0); }
                    return sw > 0 ? sv / sw : -1;
                }
            }
        }

        // 한 배관(폴리라인) 지오메트리 — 월드 mm 점열 + 유틸리티 라벨(색 결정용). 스냅샷 렌더 입력.
        private sealed class GeoPoly
        {
            public List<Point3D> Pts = new();
            public string Util = "";
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
            // 3D 스냅샷용 경로 지오메트리(월드 mm). 전략별 성공 경로만.
            public List<GeoPoly> ExistingGeo = new();
            public List<GeoPoly> ShortestGeo = new();
            public List<GeoPoly> StubGeo = new();
            // 케이스 스냅샷 파일명(img/ 상대경로). 미렌더면 null.
            public string? ImgExisting, ImgShortest, ImgStub;
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
            log.AppendLine($"그룹핑F 가중(랙/밀집/pitch/lane): {string.Join("/", GfWeights.Select(x => x.ToString("0.##", CultureInfo.InvariantCulture)))}");

            var cases = BuildCases(sd);
            if (maxCases > 0 && cases.Count > maxCases) cases = cases.Take(maxCases).ToList();   // 스모크 테스트용 제한.
            log.AppendLine($"케이스(장비×유틸그룹): {cases.Count}");
            log.AppendLine();

            int idx = 0;
            foreach (var c in cases)
            {
                idx++;
                // 기존설계: 작업↔기존배관 매칭 폴리라인의 길이·꺾임 + 래스터화 셀로 그룹핑.
                c.Existing = ExistingMetrics(sd, c.Tasks, out c.ExistingGeo);
                // 자동설계 2전략.
                c.Shortest = RunStrategy(sd, c.Tasks, Strategy.Shortest, null, null, out c.ShortestGeo);
                c.StubGroup = RunStrategy(sd, c.Tasks, Strategy.StubGroup, patterns, bundles, out c.StubGeo);
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

            // ---- 3D 스냅샷 + HTML 리포트(P4). R3D_ADR_NOIMG=1 이면 건너뜀(빠른 지표만). ----
            string? htmlPath = null;
            bool noImg = string.Equals(Environment.GetEnvironmentVariable("R3D_ADR_NOIMG"), "1");
            if (!noImg)
            {
                try
                {
                    RenderSnapshots(sd, cases, outDir, baseName, log);
                    htmlPath = Path.Combine(outDir, baseName + "_autodesign_report.html");
                    File.WriteAllText(htmlPath, BuildHtmlReport(projectId, sd, cases, baseName), new UTF8Encoding(false));
                }
                catch (Exception ex) { log.AppendLine("스냅샷/HTML 생성 실패: " + ex.Message); }
            }

            log.AppendLine();
            log.AppendLine("저장:");
            log.AppendLine("  " + csvPath);
            log.AppendLine("  " + txtPath);
            if (htmlPath != null) log.AppendLine("  " + htmlPath);
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
                                           PatternStore? patterns, BundleStore? bundles, out List<GeoPoly> geo)
        {
            geo = new List<GeoPoly>();
            var m = new Metrics { N = rows.Count };
            if (rows.Count == 0) return m;
            var g = sd.Grid; double cell = g.CellMm;
            bool stub = strat == Strategy.StubGroup;
            // 스텁 전략은 엔진이 '스텁끝~스텁끝'만 탐색하므로, 고정 스텁(라이저+엘보) 길이/꺾임을
            //   따로 더해야 기존/최단과 길이·꺾임을 공정 비교할 수 있다. task i = row i(행당 AddTask 1회).
            var stubAddLen = new double[rows.Count];
            var stubAddTurn = new int[rows.Count];
            // 스냅샷 렌더용: 매칭된 출발/종단 스텁 점열(월드 mm). 엔진 경로 앞뒤에 이어 '전체 설계'를 그린다.
            var stubSrcPts = new List<Pt3>?[rows.Count];
            var stubTgtPts = new List<Pt3>?[rows.Count];

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
                                stubSrcPts[ti] = ss; stubTgtPts[ti] = es;       // 스냅샷에 스텁 구간 포함.
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

                    // 스냅샷용 전체 폴리라인: (출발스텁) + 엔진경로(셀→월드) + (종단스텁 역순).
                    var gp = new GeoPoly { Util = rows[i].UtilityLabel };
                    if (stubSrcPts[i] != null) foreach (var p in stubSrcPts[i]!) gp.Pts.Add(W(p));
                    foreach (var c in r.Path) gp.Pts.Add(CW(g, c));
                    if (stubTgtPts[i] != null) for (int k = stubTgtPts[i]!.Count - 1; k >= 0; k--) gp.Pts.Add(W(stubTgtPts[i]![k]));
                    DedupPts(gp.Pts);
                    if (gp.Pts.Count >= 2) geo.Add(gp);
                }
                catch { }
            }
            eng.Dispose();
            (m.RackZPct, m.DensityPct, m.PitchPct, m.LanePct) = GroupMetrics(paths, rackLevels, g);
            return m;
        }

        // ---- 기존설계 지표 — 매칭 폴리라인의 길이·꺾임 + 래스터화 셀로 그룹핑 ----
        private static Metrics ExistingMetrics(SceneData sd, List<TaskInfo> rows, out List<GeoPoly> geo)
        {
            geo = new List<GeoPoly>();
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
                var gp = new GeoPoly { Util = t.UtilityLabel };
                foreach (var p in pipe.Points) gp.Pts.Add(W(p));
                if (gp.Pts.Count >= 2) geo.Add(gp);
            }
            (m.RackZPct, m.DensityPct, m.PitchPct, m.LanePct) = GroupMetrics(paths, rackLevels, g);
            return m;
        }

        // ---- 그룹핑 성분 — 랙집중도%·번들밀집도%·pitch일관성%·레인정렬도% ----
        private static (double rackZPct, double densityPct, double pitchPct, double lanePct)
            GroupMetrics(List<PathCell[]> paths, int[]? rackLevels, GridMeta g)
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

            // 레인 정렬도% — 각 배관의 '주(major) 수평 z-레벨'을 같은 z로 공유하는 배관 비율.
            //   사람 설계가 공용 랙 고도에 배관을 모으는 특성(자기 번들링)을 학습 랙 무관하게 측정.
            double lane = -1;
            if (paths.Count >= 2)
            {
                var zmaj = new List<int>();
                foreach (var p in paths)
                {
                    var cnt = new Dictionary<int, int>();
                    for (int i = 1; i < p.Length; i++)
                    {
                        var a = p[i - 1]; var b = p[i];
                        if (a.K != b.K) continue;        // 수평 셀만.
                        cnt[b.K] = cnt.GetValueOrDefault(b.K) + 1;
                    }
                    if (cnt.Count == 0) continue;
                    int best = -1, bc = -1;
                    foreach (var kv in cnt) if (kv.Value > bc) { bc = kv.Value; best = kv.Key; }
                    zmaj.Add(best);
                }
                if (zmaj.Count >= 2)
                {
                    var grp = new Dictionary<int, int>();
                    foreach (var z in zmaj) grp[z] = grp.GetValueOrDefault(z) + 1;
                    int aligned = zmaj.Count(z => grp[z] >= 2);
                    lane = aligned * 100.0 / zmaj.Count;
                }
            }

            // pitch 일관성% — 같은 (축, z-레벨)에서 평행하게 달리는 배관들의 레인 좌표 간격(피치)의
            //   변동계수 CV → 일관성 = (1 − min(1, CV))×100. 여러 (축,z) 그룹의 간격을 풀링.
            double pitch = -1;
            if (paths.Count >= 2)
            {
                // (축, z) → 그 그룹에서 각 배관이 차지하는 대표 수직(perp) 레인 좌표 집합.
                var keyLanes = new Dictionary<(int axis, int z), HashSet<int>>();
                foreach (var p in paths)
                {
                    var local = new Dictionary<(int, int), Dictionary<int, int>>();   // (축,z)→perp→런길이.
                    for (int i = 1; i < p.Length; i++)
                    {
                        var a = p[i - 1]; var b = p[i];
                        if (a.K != b.K) continue;
                        int axis, perp;
                        if (a.I != b.I && a.J == b.J) { axis = 0; perp = b.J; }       // X 진행 → perp=J.
                        else if (a.J != b.J && a.I == b.I) { axis = 1; perp = b.I; }  // Y 진행 → perp=I.
                        else continue;
                        var kk = (axis, b.K);
                        if (!local.TryGetValue(kk, out var d)) { d = new(); local[kk] = d; }
                        d[perp] = d.GetValueOrDefault(perp) + 1;
                    }
                    foreach (var kv in local)
                    {
                        int best = -1, bl = -1;          // 그 배관의 대표 레인 = 가장 긴 런의 perp.
                        foreach (var e in kv.Value) if (e.Value > bl) { bl = e.Value; best = e.Key; }
                        if (!keyLanes.TryGetValue(kv.Key, out var set)) { set = new(); keyLanes[kv.Key] = set; }
                        set.Add(best);
                    }
                }
                // 같은 묶음(한 축·한 z-레벨, 평행 배관 ≥3 → 간격 ≥2)별로 CV→일관성을 구해 평균.
                //   여러 z/축 간격을 풀링하면 서로 다른 피치가 섞여 CV 가 부풀려지므로 그룹 단위로 분리.
                //   평행 배관 3개 미만(미해상)인 그룹뿐이면 pitch 는 N/A(-1) — cell>피치/2 면 흔함.
                var consistencies = new List<double>();
                foreach (var kv in keyLanes)
                {
                    var lanes = kv.Value.OrderBy(x => x).ToList();
                    if (lanes.Count < 3) continue;
                    var gg = new List<double>();
                    for (int i = 1; i < lanes.Count; i++) gg.Add((lanes[i] - lanes[i - 1]) * g.CellMm);
                    double mean = gg.Average();
                    if (mean <= 1e-6) continue;
                    double varc = gg.Select(x => (x - mean) * (x - mean)).Average();
                    double cv = Math.Sqrt(varc) / mean;
                    consistencies.Add(1.0 - Math.Min(1.0, cv));
                }
                if (consistencies.Count > 0) pitch = consistencies.Average() * 100.0;
            }

            return (rackZ, density, pitch, lane);
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

        // ---- 좌표 변환(스냅샷용) ----
        private static Point3D W(Pt3 p) => new(p.X, p.Y, p.Z);
        private static Point3D CW(GridMeta g, PathCell c) =>
            new(g.Ox + (c.I + 0.5) * g.CellMm, g.Oy + (c.J + 0.5) * g.CellMm, g.Oz + (c.K + 0.5) * g.CellMm);
        // 인접 중복점 제거(스텁 끝 == 엔진 시작셀 중심이 겹칠 수 있음).
        private static void DedupPts(List<Point3D> pts)
        {
            for (int i = pts.Count - 1; i >= 1; i--)
            {
                var a = pts[i]; var b = pts[i - 1];
                if (Math.Abs(a.X - b.X) < 1 && Math.Abs(a.Y - b.Y) < 1 && Math.Abs(a.Z - b.Z) < 1) pts.RemoveAt(i);
            }
        }

        // ---- 3D 스냅샷(P4) — 케이스별 기존/최단/Stub+그룹 3장을 같은 카메라로 렌더 ----
        private static void RenderSnapshots(SceneData sd, List<Case> cases, string outDir, string baseName, StringBuilder log)
        {
            const int W_PX = 760, H_PX = 560;
            string imgDir = Path.Combine(outDir, "img");
            Directory.CreateDirectory(imgDir);
            var colorMap = UtilityColors.Assign(sd.Tasks.Select(t => t.UtilityLabel));
            int rendered = 0;

            for (int i = 0; i < cases.Count; i++)
            {
                var c = cases[i];
                var all = c.ExistingGeo.Concat(c.ShortestGeo).Concat(c.StubGeo).ToList();
                var allPolys = all.Select(gp => ToPoly(gp, colorMap)).ToList();
                var bounds = OffscreenRenderer.ComputeBounds(allPolys, 600);
                if (bounds == null) continue;   // 그릴 경로 없음(전부 실패/미매칭).
                var ctx = ContextBoxes(sd, bounds.Value);

                string slug = $"{baseName}_c{i + 1:000}";
                c.ImgExisting = RenderOne(Path.Combine(imgDir, slug + "_existing.png"), W_PX, H_PX, ctx,
                    c.ExistingGeo, colorMap, bounds.Value, "기존설계", MetricCaption(c.Existing), imgDir, ref rendered);
                c.ImgShortest = RenderOne(Path.Combine(imgDir, slug + "_shortest.png"), W_PX, H_PX, ctx,
                    c.ShortestGeo, colorMap, bounds.Value, "최단(A*)", MetricCaption(c.Shortest), imgDir, ref rendered);
                c.ImgStub = RenderOne(Path.Combine(imgDir, slug + "_stub.png"), W_PX, H_PX, ctx,
                    c.StubGeo, colorMap, bounds.Value, "Stub+그룹패턴", MetricCaption(c.StubGroup), imgDir, ref rendered);
            }
            log.AppendLine($"스냅샷 {rendered}장 렌더 → {imgDir}");
        }

        private static string? RenderOne(string path, int w, int h, List<OffscreenRenderer.Box> ctx,
            List<GeoPoly> geo, Dictionary<string, Color> colorMap, Rect3D bounds,
            string title, string sub, string imgDir, ref int rendered)
        {
            try
            {
                var polys = geo.Select(gp => ToPoly(gp, colorMap)).ToList();
                OffscreenRenderer.RenderToPng(path, w, h, ctx, polys, bounds, title, sub);
                rendered++;
                return "img/" + Path.GetFileName(path);
            }
            catch { return null; }
        }

        private static OffscreenRenderer.Poly ToPoly(GeoPoly gp, Dictionary<string, Color> colorMap) =>
            new() { Pts = gp.Pts, Color = colorMap.TryGetValue(gp.Util, out var col) ? col : Colors.Gray };

        private static string MetricCaption(Metrics m) =>
            $"성공 {m.Ok}/{m.N} · 총길이 {m.TotalLenMm:N0}mm · 평균꺾임 {m.AvgTurns:0.0} · GF {Fmt(m.GroupingFactor)}";

        // 카메라 bounds 와 교차하는 장비/덕트/레터럴만 옅은 맥락 박스로(작업영역만 표시, 혼잡 억제).
        private static List<OffscreenRenderer.Box> ContextBoxes(SceneData sd, Rect3D b)
        {
            var list = new List<OffscreenRenderer.Box>();
            bool Hit(double mnx, double mny, double mnz, double mxx, double mxy, double mxz) =>
                mxx >= b.X && mnx <= b.X + b.SizeX && mxy >= b.Y && mny <= b.Y + b.SizeY && mxz >= b.Z && mnz <= b.Z + b.SizeZ;
            void Add(double mnx, double mny, double mnz, double mxx, double mxy, double mxz, Color col, byte a)
            {
                if (!Hit(mnx, mny, mnz, mxx, mxy, mxz)) return;
                list.Add(new OffscreenRenderer.Box
                {
                    Center = new Point3D((mnx + mxx) / 2, (mny + mxy) / 2, (mnz + mxz) / 2),
                    Dx = Math.Max(mxx - mnx, 1), Dy = Math.Max(mxy - mny, 1), Dz = Math.Max(mxz - mnz, 1),
                    Color = col, Alpha = a
                });
            }
            foreach (var e in sd.Equipment)
                Add(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ,
                    e.IsMain ? Color.FromRgb(255, 140, 0) : Color.FromRgb(255, 190, 90), (byte)(e.IsMain ? 55 : 40));
            foreach (var d in sd.DuctsLaterals)
                Add(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ,
                    d.IsLateral ? Color.FromRgb(90, 210, 130) : Color.FromRgb(110, 175, 220), 55);
            return list;
        }

        // ---- HTML 리포트(P4) — 지표 표 + 케이스별 3-up 스냅샷 ----
        private static string BuildHtmlReport(int projectId, SceneData sd, List<Case> cases, string baseName)
        {
            string E(string? s) => System.Net.WebUtility.HtmlEncode(s ?? "");
            var sb = new StringBuilder();
            sb.AppendLine("<!doctype html><html lang=\"ko\"><head><meta charset=\"utf-8\">");
            sb.AppendLine($"<title>AI 자동설계 비교 리포트 — {E(sd.SourceFile)}</title>");
            sb.AppendLine("<style>");
            sb.AppendLine("body{font-family:'Malgun Gothic',sans-serif;margin:24px;color:#222;background:#fff;}");
            sb.AppendLine("h1{font-size:20px;border-bottom:3px solid #385b85;padding-bottom:6px;}");
            sb.AppendLine("h2{font-size:15px;color:#2b3548;margin-top:28px;}");
            sb.AppendLine("table{border-collapse:collapse;margin:8px 0;font-size:12px;}");
            sb.AppendLine("th,td{border:1px solid #ccc;padding:4px 8px;text-align:right;}");
            sb.AppendLine("th{background:#eef2f7;}td.l,th.l{text-align:left;}");
            sb.AppendLine(".best{background:#e7f3e7;font-weight:bold;}");
            sb.AppendLine(".imgs{display:flex;gap:10px;flex-wrap:wrap;margin:8px 0 4px;}");
            sb.AppendLine(".imgs figure{margin:0;}.imgs img{width:360px;border:1px solid #ccc;border-radius:4px;}");
            sb.AppendLine(".imgs figcaption{font-size:11px;color:#555;}");
            sb.AppendLine(".note{color:#666;font-size:12px;}");
            sb.AppendLine("</style></head><body>");

            sb.AppendLine($"<h1>AI 자동설계 비교 리포트</h1>");
            sb.AppendLine($"<p class=\"note\">프로젝트 {projectId} · {E(sd.SourceFile)} · 셀 {sd.Grid.CellMm:0}mm · "
                + $"격자 {sd.Grid.Nx}×{sd.Grid.Ny}×{sd.Grid.Nz} · 케이스 {cases.Count} · "
                + $"장애물 {sd.Obstacles.Count} · 장비 {sd.Equipment.Count} · 작업 {sd.Tasks.Count}</p>");

            // 전체 집계.
            Metrics Sum(Func<Case, Metrics> sel)
            {
                var t = new Metrics();
                foreach (var c in cases) { var x = sel(c); t.Ok += x.Ok; t.N += x.N; t.TotalLenMm += x.TotalLenMm; t.TotalTurns += x.TotalTurns; }
                return t;
            }
            double GFavg(Func<Case, Metrics> sel) { var v = cases.Select(c => sel(c).GroupingFactor).Where(x => x >= 0).ToList(); return v.Count > 0 ? v.Average() : -1; }
            var ex = Sum(c => c.Existing); var sh = Sum(c => c.Shortest); var sg = Sum(c => c.StubGroup);
            sb.AppendLine("<h2>전체 집계</h2><table>");
            sb.AppendLine("<tr><th class=\"l\">전략</th><th>성공</th><th>총길이(mm)</th><th>평균꺾임</th><th>그룹핑F(평균)</th></tr>");
            sb.AppendLine(Row("기존설계", ex.Ok, ex.N, ex.TotalLenMm, ex.AvgTurns, GFavg(c => c.Existing)));
            sb.AppendLine(Row("최단(A*)", sh.Ok, sh.N, sh.TotalLenMm, sh.AvgTurns, GFavg(c => c.Shortest)));
            sb.AppendLine(Row("Stub+그룹", sg.Ok, sg.N, sg.TotalLenMm, sg.AvgTurns, GFavg(c => c.StubGroup)));
            sb.AppendLine("</table>");
            sb.AppendLine("<p class=\"note\">최단=길이·직선성 우위, Stub+그룹=그룹핑F(다발화)·사람설계 추종 우위 기대. "
                + "그룹핑F = 0.35×랙집중도 + 0.30×번들밀집도 + 0.20×pitch일관성 + 0.15×레인정렬도(각 0~1, N/A 성분은 가중 재정규화).</p>");

            // 케이스별.
            string? curMain = null;
            for (int i = 0; i < cases.Count; i++)
            {
                var c = cases[i];
                if (c.MainEquip != curMain) { curMain = c.MainEquip; sb.AppendLine($"<h2>■ 메인장비: {E(curMain)}</h2>"); }
                sb.AppendLine($"<h3 style=\"font-size:13px;margin:14px 0 2px;\">케이스 {i + 1}. {E(c.Equip)} / {E(c.UtilGroup)} (작업 {c.Tasks.Count})</h3>");
                sb.AppendLine("<table>");
                sb.AppendLine("<tr><th class=\"l\">전략</th><th>성공</th><th>총길이(mm)</th><th>평균꺾임</th><th>랙집중%</th><th>번들밀집%</th><th>pitch%</th><th>lane%</th><th>그룹핑F</th></tr>");
                sb.AppendLine(Row2("기존설계", c.Existing));
                sb.AppendLine(Row2("최단(A*)", c.Shortest));
                sb.AppendLine(Row2("Stub+그룹", c.StubGroup));
                sb.AppendLine("</table>");
                if (c.ImgExisting != null || c.ImgShortest != null || c.ImgStub != null)
                {
                    sb.AppendLine("<div class=\"imgs\">");
                    Fig(sb, c.ImgExisting, "기존설계");
                    Fig(sb, c.ImgShortest, "최단(A*)");
                    Fig(sb, c.ImgStub, "Stub+그룹패턴");
                    sb.AppendLine("</div>");
                }
            }
            sb.AppendLine("</body></html>");
            return sb.ToString();

            void Fig(StringBuilder b, string? img, string cap)
            {
                if (img == null) return;
                b.AppendLine($"<figure><img src=\"{img}\" alt=\"{cap}\"><figcaption>{cap}</figcaption></figure>");
            }
            string Row(string name, int ok, int n, double len, double turns, double gf)
                => $"<tr><td class=\"l\">{name}</td><td>{ok}/{n}</td><td>{len:N0}</td><td>{turns:0.0}</td><td>{Fmt(gf)}</td></tr>";
            string Row2(string name, Metrics m)
                => $"<tr><td class=\"l\">{name}</td><td>{m.Ok}/{m.N}</td><td>{m.TotalLenMm:N0}</td><td>{m.AvgTurns:0.0}</td>"
                 + $"<td>{Pct(m.RackZPct)}</td><td>{Pct(m.DensityPct)}</td><td>{Pct(m.PitchPct)}</td><td>{Pct(m.LanePct)}</td><td>{Fmt(m.GroupingFactor)}</td></tr>";
        }

        // ---- 출력 ----
        private static void WriteCsv(string path, List<Case> cases)
        {
            var sb = new StringBuilder();
            sb.AppendLine("메인장비,장비,유틸리티그룹,작업수,"
                + "기존_총길이mm,기존_평균꺾임,기존_랙집중%,기존_번들밀집%,기존_pitch%,기존_lane%,기존_그룹핑F,"
                + "최단_성공,최단_총길이mm,최단_평균꺾임,최단_랙집중%,최단_번들밀집%,최단_pitch%,최단_lane%,최단_그룹핑F,"
                + "Stub그룹_성공,Stub그룹_총길이mm,Stub그룹_평균꺾임,Stub그룹_랙집중%,Stub그룹_번들밀집%,Stub그룹_pitch%,Stub그룹_lane%,Stub그룹_그룹핑F,Stub매칭");
            foreach (var c in cases)
            {
                sb.AppendLine(string.Join(",",
                    Q(c.MainEquip), Q(c.Equip), Q(c.UtilGroup), c.Tasks.Count,
                    F0(c.Existing.TotalLenMm), F1(c.Existing.AvgTurns),
                    Pct(c.Existing.RackZPct), Pct(c.Existing.DensityPct), Pct(c.Existing.PitchPct), Pct(c.Existing.LanePct), Fmt(c.Existing.GroupingFactor),
                    $"{c.Shortest.Ok}/{c.Shortest.N}", F0(c.Shortest.TotalLenMm), F1(c.Shortest.AvgTurns),
                    Pct(c.Shortest.RackZPct), Pct(c.Shortest.DensityPct), Pct(c.Shortest.PitchPct), Pct(c.Shortest.LanePct), Fmt(c.Shortest.GroupingFactor),
                    $"{c.StubGroup.Ok}/{c.StubGroup.N}", F0(c.StubGroup.TotalLenMm), F1(c.StubGroup.AvgTurns),
                    Pct(c.StubGroup.RackZPct), Pct(c.StubGroup.DensityPct), Pct(c.StubGroup.PitchPct), Pct(c.StubGroup.LanePct), Fmt(c.StubGroup.GroupingFactor),
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
                sb.AppendLine($"      기존     len {c.Existing.TotalLenMm,10:N0}  avgTurn {c.Existing.AvgTurns,5:0.0}  GF {Fmt(c.Existing.GroupingFactor)}  랙{Pct(c.Existing.RackZPct)} 밀집{Pct(c.Existing.DensityPct)} pitch{Pct(c.Existing.PitchPct)} lane{Pct(c.Existing.LanePct)}");
                sb.AppendLine($"      최단     len {c.Shortest.TotalLenMm,10:N0}  avgTurn {c.Shortest.AvgTurns,5:0.0}  GF {Fmt(c.Shortest.GroupingFactor)}  성공 {c.Shortest.Ok}/{c.Shortest.N}  랙{Pct(c.Shortest.RackZPct)} 밀집{Pct(c.Shortest.DensityPct)} pitch{Pct(c.Shortest.PitchPct)} lane{Pct(c.Shortest.LanePct)}");
                sb.AppendLine($"      Stub+그룹 len {c.StubGroup.TotalLenMm,10:N0}  avgTurn {c.StubGroup.AvgTurns,5:0.0}  GF {Fmt(c.StubGroup.GroupingFactor)}  성공 {c.StubGroup.Ok}/{c.StubGroup.N}  랙{Pct(c.StubGroup.RackZPct)} 밀집{Pct(c.StubGroup.DensityPct)} pitch{Pct(c.StubGroup.PitchPct)} lane{Pct(c.StubGroup.LanePct)}  스텁 {c.StubGroup.StubMatched}");
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

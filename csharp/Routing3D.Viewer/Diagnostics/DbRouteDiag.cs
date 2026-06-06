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
            sb.AppendLine($"기존설계 패턴: {(patterns == null ? (usePat ? "없음(기하 폴백)" : "OFF") : patterns.Count + "키")}"
                          + (patterns != null ? $" (검색증강 다중면 {patterns.AnnKeyCount}키)" : ""));

            // 그룹배관 패턴(번들, L4) — R3D_BUNDLE=on 이면 DB(route_bundle_template)의 유틸별 트렁크 고도를
            // rack_levels 로 주입(GUI UseBundlePattern 미러). 미적재면 폴백(무해).
            bool useBundle = string.Equals(Environment.GetEnvironmentVariable("R3D_BUNDLE"), "on",
                                           StringComparison.OrdinalIgnoreCase);
            BundleStore? bundles = useBundle ? BundleStore.TryLoad(cfg, sd.SourceFile) : null;
            sb.AppendLine($"그룹배관 패턴: {(bundles == null ? (useBundle ? "없음(미적재)" : "OFF") : bundles.Count + "키")}");

            sb.AppendLine(Try(sd, rows, fac: true, drop: true, wClear: 10, mode: "multi",
                              "G route_multi +facilities+drop clearON(Implicit 온디맨드)", patterns, bundles));
            return sb.ToString();
        }

        static string Try(SceneData sd, List<TaskInfo> rows, bool fac, bool drop,
                          double wClear, string mode, string label, PatternStore? patterns = null,
                          BundleStore? bundles = null, int factor = 6, int radius = 2)
        {
            var g = sd.Grid;
            double cell = g.CellMm;
            int[]? rackLevels = null;   // L3a 랙 z-셀(측정에 재사용 위해 try 밖 선언).
            int stubMatched = 0;        // 스텁 라우팅으로 처리된 작업 수(매칭 배관 있는 것).
            int corrCells = 0;          // 주입된 회랑 셀 수(번들/L2b). 0=회랑 미적용.
            double wCorrUsed = 0;       // 적용된 w_corridor(셀당 회랑 밖 가산).
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
                //   R3D_WHEUR_NEAR = 동적(수렴) 가중 목표근처 값. (0,wHeur) 면 거리비 보간(먼곳=wHeur·근처=이값).
                //                    목표 근처 혼잡/막다른길의 그리디 함정 완화·준최적 상한은 wHeur. 기본 1.0
                //                    (=목표서 표준 A*, GUI 와 동일·스텁ON 206→208). 0=정적으로 끔(A/B). 거대격자 자동 비활성.
                double wHeurNear = ParseEnv("R3D_WHEUR_NEAR", 1.0);
                int corrRad = (int)ParseEnv("R3D_CORRRAD", 2);
                // 유틸그룹 랙 번들링(L3a) — R3D_RACK=on 이면 학습된 그룹 랙 z-셀을 rack_levels(면제)로.
                bool useRack = string.Equals(Environment.GetEnvironmentVariable("R3D_RACK"), "on",
                                             StringComparison.OrdinalIgnoreCase);
                rackLevels = BuildRackLevels(sd, rows);   // 항상 학습(집중도 지표에 재사용). 적용은 useRack 일 때만.
                // 적용할 랙: 랙번들(L3a) 학습값 + 그룹배관 번들(L4) 유틸별 트렁크 고도 합집합.
                int[]? appliedRack = useRack ? rackLevels : null;
                if (bundles != null) appliedRack = MergeBundle(sd, rows, appliedRack, bundles);
                bool hasRack = appliedRack != null && appliedRack.Length > 0;
                // 번들 공용 트렁크 회랑(L4) — 같은 유틸 기존배관 전체를 공용 회랑으로(GUI BuildBundleCorridorCells 미러).
                var bundleSet = bundles != null ? BuildBundleCorridor(sd, rows, corrRad, bundles) : null;
                bool hasBundleCorr = bundleSet != null && bundleSet.Count > 0;
                // 회랑(L2b/번들)=cell*0.5(설계추종), 랙-단독=부드러운 cell*0.2. L2b 와 번들 동시면 L2b(R3D_WCORR) 우선.
                double wCorr = useCorr ? cell * wcMul
                             : hasBundleCorr ? cell * 0.5
                             : (useRack || hasRack) ? cell * ParseEnv("R3D_WCORR", 0.2) : 0.0;
                wCorrUsed = wCorr;
                eng.SetParams(cell, 500, wClear, clr, 6, wCorridor: wCorr, corridorRadius: corrRad,
                              rackLevels: appliedRack, wHeur: wHeur, wHeurNear: wHeurNear);
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

                // 검색증강(L3b) — DUCT 종단 면을 PoC 컨텍스트별 ANN 으로 분기(GUI LearnedDuctFace 미러).
                // R3D_ANN=off 면 비활성(집계 Face 로 폴백, A/B 비교용). 기본 ON.
                bool useAnn = !string.Equals(Environment.GetEnvironmentVariable("R3D_ANN"), "off",
                                             StringComparison.OrdinalIgnoreCase);
                string? DuctFace(TaskInfo t)
                {
                    if (useAnn && patterns != null)
                    {
                        var d = NearestDuct(sd, t.Gx, t.Gy, t.Gz);
                        if (d != null)
                        {
                            var rel = new[] { Rel(t.Gx, d.MinX, d.MaxX), Rel(t.Gy, d.MinY, d.MaxY), Rel(t.Gz, d.MinZ, d.MaxZ) };
                            var dir = Unit3(t.Sx - t.Gx, t.Sy - t.Gy, t.Sz - t.Gz);
                            if (patterns.TryGetFaceAnn("DUCT", t.Group, t.Utility, rel, dir, out var f)) return f;
                        }
                    }
                    return Face("DUCT", t);
                }

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

                // 스텁 라우팅(GUI UseStubRouting 미러) — R3D_STUB=on 이면 매칭 기존배관 스텁 끝~끝으로 A*.
                bool useStub = string.Equals(Environment.GetEnvironmentVariable("R3D_STUB"), "on",
                                             StringComparison.OrdinalIgnoreCase);
                foreach (var t in rows)
                {
                    if (useStub)
                    {
                        var pipe = MatchPipe(sd, t, cell);
                        if (pipe != null)
                        {
                            var (srcStub, tgtStub) = StubExtractor.ForPipe(pipe);
                            var ps = pipe.SourcePos ?? pipe.Points[0];
                            var pe = pipe.TargetPos ?? pipe.Points[pipe.Points.Count - 1];
                            bool fwd = D(t.Sx, t.Sy, t.Sz, ps) + D(t.Gx, t.Gy, t.Gz, pe)
                                       <= D(t.Sx, t.Sy, t.Sz, pe) + D(t.Gx, t.Gy, t.Gz, ps);
                            var ss = fwd ? srcStub : tgtStub;
                            var es = fwd ? tgtStub : srcStub;
                            if (ss.Count >= 2 && es.Count >= 2)
                            {
                                var se = ss[ss.Count - 1]; var ee = es[es.Count - 1];
                                var (bx, by, bz) = SnapFree(se.X, se.Y, se.Z, null);
                                var (cx, cy, cz) = SnapFree(ee.X, ee.Y, ee.Z, null);
                                eng.AddTask(bx, by, bz, cx, cy, cz, t.Utility, t.Group);
                                stubMatched++;
                                continue;   // 스텁 끝~끝으로 A* — PoC 직접 라우팅 분기 스킵.
                            }
                        }
                    }
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
                    string? ductFace = DuctFace(t);   // L3b ANN(다중면 키) 또는 집계 폴백.
                    var (gx, gy, gz) = Lift(t.Gx, t.Gy, t.Gz, ductFace);
                    (gx, gy, gz) = SnapFree(gx, gy, gz, ductFace);
                    eng.AddTask(sx, sy, sz, gx, gy, gz, t.Utility, t.Group);
                }

                // 회랑 시드 주입(w_corridor>0 일 때 효력): L2b(매칭, useCorr) + 번들 공용 트렁크(bundles) 합집합.
                if (useCorr || hasBundleCorr)
                {
                    var set = bundleSet != null ? new HashSet<(int, int, int)>(bundleSet)
                                                : new HashSet<(int, int, int)>();
                    if (useCorr)
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
                    corrCells = set.Count;
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

            int ok = 0; double tot = 0; long totTurns = 0;   // totTurns=성공경로 총 꺾임 수(역주행/킨크 지표).
            // 랙 집중도(L3a) — 성공 경로의 수평 이동 셀 중 학습된 랙 z-셀에 놓인 비율. 번들링 강화 시 증가.
            var rackSet = rackLevels != null ? new HashSet<int>(rackLevels) : null;
            long horizCells = 0, rackCells = 0;
            for (int i = 0; i < rows.Count; i++)
            {
                try
                {
                    var r = eng.GetResult(i);
                    if (!r.Success) continue;
                    ok++; tot += r.LengthMm; totTurns += r.Turns;
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
            // 번들 밀집도 — 성공 경로 쌍에서 한 배관 셀이 다른 배관 셀 ±2 안에 드는 평균 비율(높을수록 다발).
            // test_attract 의 near_frac 와 동일 개념. 배관 수가 많으면(>40) O(n²) 회피 위해 생략.
            double nearPct = -1;
            if (rows.Count >= 2 && rows.Count <= 40)
            {
                var paths = new List<HashSet<long>>();
                for (int i = 0; i < rows.Count; i++)
                {
                    try { var r = eng.GetResult(i); if (r.Success && r.Path.Length > 0) {
                        var hs = new HashSet<long>();
                        foreach (var c in r.Path) hs.Add(((long)c.I << 40) ^ ((long)c.J << 20) ^ c.K);
                        paths.Add(hs);
                    } } catch { }
                }
                if (paths.Count >= 2)
                {
                    double sum = 0; int pairs = 0;
                    for (int i = 0; i < paths.Count; i++)
                        for (int j = 0; j < paths.Count; j++)
                        {
                            if (i == j) continue;
                            int near = 0;
                            foreach (var key in paths[i])
                            {
                                int ci = (int)((key >> 40) & 0xFFFFF), cj = (int)((key >> 20) & 0xFFFFF), ck = (int)(key & 0xFFFFF);
                                bool hit = false;
                                for (int di = -2; di <= 2 && !hit; di++)
                                    for (int dj = -2; dj <= 2 && !hit; dj++)
                                        if (paths[j].Contains(((long)(ci + di) << 40) ^ ((long)(cj + dj) << 20) ^ ck)) hit = true;
                                if (hit) near++;
                            }
                            sum += paths[i].Count > 0 ? (double)near / paths[i].Count : 0; pairs++;
                        }
                    nearPct = pairs > 0 ? sum / pairs * 100.0 : -1;
                }
            }
            // 기존배관 복제(폴리라인) 검증 — R3D_REPLICATE=on 이면 매칭 배관 폴리라인을 셀로 복제하고 막힌
            //   구간만 eng.RouteTask 로 국소 수리, 매칭/성공/수리 통계를 보고(GUI ReplicateMatchedPipes 미러).
            string replic = "";
            if (string.Equals(Environment.GetEnvironmentVariable("R3D_REPLICATE"), "on", StringComparison.OrdinalIgnoreCase)
                && (long)g.Nx * g.Ny * g.Nz <= 300_000_000L)
            {
                int matched = 0, repOk = 0, repFail = 0; long repairs = 0, copied = 0;
                // 막힘 판정 박스(장애물 통과 제외 + 설비 + 덕트, minT 팽창) — 셀 AABB 겹침이면 막힘.
                double minT = cell;
                var boxes = new List<(double, double, double, double, double, double)>();
                void AddB(double a, double b, double c2, double d, double e2, double f2)
                {
                    if (d - a < minT) { double m = (a + d) / 2; a = m - minT / 2; d = m + minT / 2; }
                    if (e2 - b < minT) { double m = (b + e2) / 2; b = m - minT / 2; e2 = m + minT / 2; }
                    if (f2 - c2 < minT) { double m = (c2 + f2) / 2; c2 = m - minT / 2; f2 = m + minT / 2; }
                    boxes.Add((a, b, c2, d, e2, f2));
                }
                foreach (var o in sd.Obstacles) if (!o.IsPassThrough) AddB(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                foreach (var e in sd.Equipment) AddB(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
                foreach (var d in sd.DuctsLaterals) AddB(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
                bool Blk(int ci, int cj, int ck)
                {
                    double clx = g.Ox + ci * cell, chx = clx + cell, cly = g.Oy + cj * cell, chy = cly + cell,
                           clz = g.Oz + ck * cell, chz = clz + cell;
                    foreach (var (mnx, mny, mnz, mxx, mxy, mxz) in boxes)
                        if (clx < mxx && chx > mnx && cly < mxy && chy > mny && clz < mxz && chz > mnz) return true;
                    return false;
                }
                foreach (var t in rows)
                {
                    var pipe = MatchPipe(sd, t, cell);
                    if (pipe == null || pipe.Points.Count < 2) continue;
                    matched++;
                    var res = ReplicateOne(sd, t, pipe, Blk, eng);
                    if (res == null) { repFail++; continue; }
                    repOk++; copied += res.Value.cells; repairs += res.Value.repairs;
                }
                replic = $" [replicate matched {matched} ok {repOk} fail {repFail} repairs {repairs} cells {copied}]";
            }

            // 그룹패턴 경유(코너 waypoint) 검증 — R3D_GROUPCORNER=on 이면 매칭 그룹배관의 꺾임점을 추출해
            //   코너 사이를 Repair(A*)로 잇는다(GUI RouteThroughGroupCorners 미러). 매칭/성공/평균길이 보고.
            string gcorn = "";
            if (string.Equals(Environment.GetEnvironmentVariable("R3D_GROUPCORNER"), "on", StringComparison.OrdinalIgnoreCase)
                && (long)g.Nx * g.Ny * g.Nz <= 300_000_000L)
            {
                using var crep = new Engine();
                crep.SetGrid(cell, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
                // 수리 A* 는 표준 A*(w_heur=1.0) — w_turn=500 페널티가 지배해 최소꺾임 경로를 찾는다(weighted
                //   그리디는 꺾임 폭증). 수리는 국소 구간이라 w=1 도 빠르며, 막혀 12M 초과 시 null→A* 폴백.
                crep.SetParams(cell, 500, 10, 2, 6, wHeur: 1.0, wHeurNear: 0.0);
                foreach (var o in sd.Obstacles)
                    if (o.IsPassThrough) crep.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                    else crep.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                double mt = cell;
                void Bx(double a, double b, double c2, double d, double e2, double f2)
                {
                    if (d - a < mt) { double m = (a + d) / 2; a = m - mt / 2; d = m + mt / 2; }
                    if (e2 - b < mt) { double m = (b + e2) / 2; b = m - mt / 2; e2 = m + mt / 2; }
                    if (f2 - c2 < mt) { double m = (c2 + f2) / 2; c2 = m - mt / 2; f2 = m + mt / 2; }
                    crep.AddObstacle(a, b, c2, d, e2, f2);
                }
                foreach (var e in sd.Equipment) Bx(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
                foreach (var d in sd.DuctsLaterals) Bx(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
                // 막힘 판정 박스(스킵+수리용) — crep 와 동일 장애물 집합.
                var cboxes = new List<(double, double, double, double, double, double)>();
                void CBx(double a, double b, double c2, double d, double e2, double f2)
                {
                    if (d - a < mt) { double m = (a + d) / 2; a = m - mt / 2; d = m + mt / 2; }
                    if (e2 - b < mt) { double m = (b + e2) / 2; b = m - mt / 2; e2 = m + mt / 2; }
                    if (f2 - c2 < mt) { double m = (c2 + f2) / 2; c2 = m - mt / 2; f2 = m + mt / 2; }
                    cboxes.Add((a, b, c2, d, e2, f2));
                }
                foreach (var o in sd.Obstacles) if (!o.IsPassThrough) CBx(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                foreach (var e in sd.Equipment) CBx(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
                foreach (var d in sd.DuctsLaterals) CBx(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
                bool CBlk(int ci, int cj, int ck)
                {
                    double clx = g.Ox + ci * cell, chx = clx + cell, cly = g.Oy + cj * cell, chy = cly + cell,
                           clz = g.Oz + ck * cell, chz = clz + cell;
                    foreach (var (mnx, mny, mnz, mxx, mxy, mxz) in cboxes)
                        if (clx < mxx && chx > mnx && cly < mxy && chy > mny && clz < mxz && chz > mnz) return true;
                    return false;
                }

                // 경로 꺾임(turns) = 진행 축이 바뀌는 셀 수. 톱니(staircase)가 심하면 폭증 → 수정 효과 정량 지표.
                static int Turns(PathCell[] p)
                {
                    int t = 0;
                    for (int i = 2; i < p.Length; i++)
                    {
                        int a0 = (p[i - 1].I - p[i - 2].I) != 0 ? 0 : (p[i - 1].J - p[i - 2].J) != 0 ? 1 : 2;
                        int a1 = (p[i].I - p[i - 1].I) != 0 ? 0 : (p[i].J - p[i - 1].J) != 0 ? 1 : 2;
                        if (a0 != a1) t++;
                    }
                    return t;
                }
                // 번들 레인 1:1 배정(GUI AssignBundleLanes 미러) — 같은 번들 작업을 distinct 멤버에 흩어 평행
                //   다발 재현. bundles 미적재면 빈 맵 → MatchPipe 폴백. lane 분산 지표(distinct 멤버/작업)도 보고.
                var lane = AssignBundleLanes(sd, rows, cell, bundles);
                int laneTasks = lane.Count, laneDistinct = lane.Values.Distinct().Count();
                // 짧은런(말림=jog) 카운트 — 첫/끝 제외 ≤2셀 런(코너의 양자화 계단). 0 이면 자연스러운 단일 엘보.
                static int ShortJogs(PathCell[] p)
                {
                    var lens = new List<int>();
                    if (p.Length >= 2)
                    {
                        int ax(PathCell a, PathCell b) => (b.I - a.I) != 0 ? 0 : (b.J - a.J) != 0 ? 1 : 2;
                        int cur = ax(p[0], p[1]), run = 1;
                        for (int k = 2; k < p.Length; k++)
                        {
                            int a = ax(p[k - 1], p[k]);
                            if (a == cur) run++; else { lens.Add(run); cur = a; run = 1; }
                        }
                        lens.Add(run);
                    }
                    int j = 0; for (int i = 1; i < lens.Count - 1; i++) if (lens[i] <= 2) j++;
                    return j;
                }
                int gm = 0, gok = 0; double gpath = 0; long gturns = 0, gcorners = 0, greps = 0, gjogs = 0;
                foreach (var t in rows)
                {
                    var pipe = lane.TryGetValue(t, out var lp) ? lp : MatchPipe(sd, t, cell);
                    if (pipe == null || pipe.Points.Count < 2) continue;
                    gm++;
                    var path = GroupCornerRoute(sd, t, pipe, CBlk, crep, out int cc, out int rr);
                    if (path == null) continue;
                    gok++; gpath += (path.Length - 1) * cell; gturns += Turns(path); gcorners += cc; greps += rr; gjogs += ShortJogs(path);
                }
                // avgTurns≈avgCorners 면 군더더기 꺾임 0(설계 코너만), avgReps=막힘 A* 우회 빈도.
                // lane: 번들 작업 N개가 distinct 멤버 M개에 배정(M=N 이면 레인 붕괴 0=완전 평행).
                gcorn = $" [groupcorner matched {gm} ok {gok} avgLen {(gok > 0 ? gpath / gok : 0):0} avgTurns {(gok > 0 ? (double)gturns / gok : 0):0.0} avgCorners {(gok > 0 ? (double)gcorners / gok : 0):0.0} avgReps {(gok > 0 ? (double)greps / gok : 0):0.0} avgJogs {(gok > 0 ? (double)gjogs / gok : 0):0.0} lane {laneDistinct}/{laneTasks}]";
            }

            eng.Dispose();
            string cb = mode == "multi" ? $" [progress cb {cbCount}, fail {cbFail}]" : "";
            string fe = failExp.Count > 0 ? $" failExpanded=[{string.Join(",", failExp)}]" : "";
            string rk = (rackSet != null && horizCells > 0)
                ? $" rackZ={rackCells * 100.0 / horizCells:0.0}% (z셀 {string.Join(",", rackLevels!)})" : "";
            string stub = stubMatched > 0 ? $" [stub {stubMatched}/{rows.Count}]" : "";
            string corr = corrCells > 0 ? $" corridor={corrCells}셀 wCorr={wCorrUsed:0}" : "";
            string nr = nearPct >= 0 ? $" 번들밀집={nearPct:0.0}%" : "";
            return $"{label}: success {ok}/{rows.Count} totalLen {tot:0} turns {totTurns} ({sw.ElapsedMilliseconds} ms){cb}{fe}{rk}{stub}{corr}{nr}{replic}{gcorn}";
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

        // 그룹배관 패턴(L4) — rows 의 유틸별 학습 트렁크 고도(route_bundle_template)를 z-셀로 변환해 rackLevels
        // 에 합친다(GUI SceneViewModel.MergeBundleLevels 미러). 키 미스/미적재면 입력 그대로 반환.
        static int[]? MergeBundle(SceneData sd, List<TaskInfo> rows, int[]? rackLevels, BundleStore bundles)
        {
            var g = sd.Grid; double oz = g.Oz, cell = g.CellMm; if (cell <= 0) return rackLevels;
            var utils = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var t in rows) if (!string.IsNullOrEmpty(t.Utility)) utils.Add(t.Utility!);
            if (utils.Count == 0) return rackLevels;
            var zset = new HashSet<int>(rackLevels ?? Array.Empty<int>());
            foreach (var u in utils)
            {
                var t = bundles.TryGet(null, u);
                if (t == null) continue;
                foreach (var z in t.TrunkZs)
                {
                    int zk = (int)Math.Floor((z - oz) / cell);
                    if (zk >= 0 && zk < g.Nz) zset.Add(zk);
                }
            }
            return zset.Count > 0 ? zset.Take(8).ToArray() : rackLevels;
        }

        // 번들 공용 트렁크 회랑 + pitch 레인(L4) — 같은 유틸 기존배관의 '트렁크 고도 수평 런'(=평행 랙 레인)만
        // 타이트하게 회랑으로 만든다(GUI SceneViewModel.BuildBundleCorridorCells 미러). 트렁크 미스면 전체 폴리라인
        // 넓게 폴백(옵션1). 충돌회피가 새 배관을 인접 레인에 분산 → 등간격 다발 패킹.
        static HashSet<(int, int, int)> BuildBundleCorridor(SceneData sd, List<TaskInfo> rows, int corrRad, BundleStore bundles)
        {
            var set = new HashSet<(int, int, int)>();
            var g = sd.Grid; double cell = g.CellMm, oz = g.Oz;
            if (sd.ExistingPipes.Count == 0) return set;
            var utils = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var t in rows) if (!string.IsNullOrEmpty(t.Utility)) utils.Add(t.Utility!);
            if (utils.Count == 0) return set;

            var trunkZ = new HashSet<int>();
            foreach (var u in utils)
            {
                var t = bundles.TryGet(null, u); if (t == null) continue;
                foreach (var z in t.TrunkZs)
                {
                    int zk = (int)Math.Floor((z - oz) / cell);
                    if (zk >= 0 && zk < g.Nz) trunkZ.Add(zk);
                }
            }
            // 멤버십 기준(v3, GUI 미러) — 탐지 번들 멤버 배관의 '모든 런(수평·수직)'을 회랑으로(trunk_axis·
            //   trunk_z 밴드 무관). 비멤버 제외. 수평·수직 그룹배관 모두 경유지. 번들 미적재면 옛 밴드 폴백.
            bool memberAware = bundles.GroupCount > 0;
            bool laneMode = memberAware || trunkZ.Count > 0;
            const double HorizTol = 0.34, MinRunMm = 800.0;
            const int BandCells = 1, LaneDilate = 1;

            foreach (var pipe in sd.ExistingPipes)
            {
                if (pipe.Utility == null || !utils.Contains(pipe.Utility) || pipe.Points.Count < 2) continue;
                bool isMember = memberAware && pipe.RoutePathGuid != null
                                && bundles.GroupIdOf(pipe.RoutePathGuid) >= 0;
                if (memberAware && !isMember) continue;   // 멤버십 모드 — 검출된 다발만.
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                    double horiz = Math.Sqrt(dx * dx + dy * dy);
                    double len = Math.Sqrt(horiz * horiz + dz * dz);
                    bool vertical = horiz <= 1e-6 || Math.Abs(dz) > HorizTol * horiz;
                    int dl = corrRad;
                    if (memberAware)
                    {
                        if (len < (vertical ? MinRunMm * 0.3 : MinRunMm)) continue;
                        dl = LaneDilate;
                    }
                    else if (laneMode)
                    {
                        if (vertical)
                        {
                            if (len < MinRunMm * 0.3) continue;
                            int zk0 = (int)Math.Floor((Math.Min(a.Z, b.Z) - oz) / cell);
                            int zk1 = (int)Math.Floor((Math.Max(a.Z, b.Z) - oz) / cell);
                            bool touches = false;
                            foreach (var tz in trunkZ) if (zk1 >= tz - BandCells && zk0 <= tz + BandCells) { touches = true; break; }
                            if (!touches) continue;
                        }
                        else
                        {
                            if (len < MinRunMm) continue;
                            int zk = (int)Math.Floor(((a.Z + b.Z) / 2 - oz) / cell);
                            bool nearTrunk = false;
                            foreach (var tz in trunkZ) if (Math.Abs(zk - tz) <= BandCells) { nearTrunk = true; break; }
                            if (!nearTrunk) continue;
                        }
                        dl = LaneDilate;
                    }
                    int steps = Math.Max(1, (int)(len / (cell * 0.5)));
                    for (int sIdx = 0; sIdx <= steps; sIdx++)
                    {
                        double tt = (double)sIdx / steps;
                        int ci = (int)Math.Floor((a.X + dx * tt - g.Ox) / cell);
                        int cj = (int)Math.Floor((a.Y + dy * tt - g.Oy) / cell);
                        int ck = (int)Math.Floor((a.Z + dz * tt - g.Oz) / cell);
                        for (int di = -dl; di <= dl; di++)
                            for (int dj = -dl; dj <= dl; dj++)
                                for (int dk = -dl; dk <= dl; dk++)
                                {
                                    int ii = ci + di, jj = cj + dj, kk = ck + dk;
                                    if (ii < 0 || jj < 0 || kk < 0 || ii >= g.Nx || jj >= g.Ny || kk >= g.Nz) continue;
                                    set.Add((ii, jj, kk));
                                }
                    }
                }
            }
            return set;
        }

        // 작업(TaskInfo) ↔ 기존 설계배관(TB_ROUTE_PATH) 매칭 — 양 끝 PoC 거리 합 최소(양방향), 임계 내. (GUI FindMatchingExistingPipe 미러)
        // 같은 번들 그룹 작업을 distinct 멤버 배관에 1:1(탐욕 최근접) 배정(GUI SceneViewModel.AssignBundleLanes 미러).
        static Dictionary<TaskInfo, ExistingPipe> AssignBundleLanes(SceneData sd, List<TaskInfo> rows,
                                                                    double cell, BundleStore? bundles)
        {
            var result = new Dictionary<TaskInfo, ExistingPipe>();
            if (bundles == null || bundles.GroupCount == 0) return result;
            double Score(TaskInfo t, ExistingPipe p)
            {
                var ps = p.SourcePos ?? p.Points[0]; var pe = p.TargetPos ?? p.Points[p.Points.Count - 1];
                return Math.Min(D(t.Sx, t.Sy, t.Sz, ps) + D(t.Gx, t.Gy, t.Gz, pe),
                                D(t.Sx, t.Sy, t.Sz, pe) + D(t.Gx, t.Gy, t.Gz, ps));
            }
            var taskGid = new Dictionary<TaskInfo, int>();
            foreach (var t in rows)
            {
                var m = MatchPipe(sd, t, cell);
                if (m?.RoutePathGuid == null) continue;
                int gid = bundles.GroupIdOf(m.RoutePathGuid);
                if (gid >= 0) taskGid[t] = gid;
            }
            if (taskGid.Count == 0) return result;
            var members = new Dictionary<int, List<ExistingPipe>>();
            foreach (var p in sd.ExistingPipes)
            {
                if (p.RoutePathGuid == null || p.Points.Count < 2) continue;
                int gid = bundles.GroupIdOf(p.RoutePathGuid);
                if (gid < 0) continue;
                if (!members.TryGetValue(gid, out var lst)) members[gid] = lst = new List<ExistingPipe>();
                lst.Add(p);
            }
            foreach (var grp in taskGid.Values.Distinct())
            {
                var tasks = taskGid.Where(kv => kv.Value == grp).Select(kv => kv.Key).ToList();
                if (tasks.Count < 2) continue;
                if (!members.TryGetValue(grp, out var mem) || mem.Count == 0) continue;
                var pairs = new List<(double cost, TaskInfo t, ExistingPipe p)>();
                foreach (var t in tasks) foreach (var p in mem) pairs.Add((Score(t, p), t, p));
                pairs.Sort((a, b) => a.cost.CompareTo(b.cost));
                var usedPipe = new HashSet<ExistingPipe>();
                foreach (var (cost, t, p) in pairs)
                {
                    if (result.ContainsKey(t) || usedPipe.Contains(p)) continue;
                    result[t] = p; usedPipe.Add(p);
                }
                foreach (var t in tasks)
                {
                    if (result.ContainsKey(t)) continue;
                    ExistingPipe? best = null; double bs = double.MaxValue;
                    foreach (var p in mem) { double c = Score(t, p); if (c < bs) { bs = c; best = p; } }
                    if (best != null) result[t] = best;
                }
            }
            return result;
        }

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

        // 검색증강(L3b) 보조 — 종단 PoC 를 포함/최근접(3000mm) 덕트. (GUI NearestDuctAnchor 미러)
        static DuctLateral? NearestDuct(SceneData sd, double x, double y, double z)
        {
            const double eps = 1.0, maxMm = 3000.0;
            foreach (var d in sd.DuctsLaterals)
                if (x >= d.MinX - eps && x <= d.MaxX + eps && y >= d.MinY - eps && y <= d.MaxY + eps &&
                    z >= d.MinZ - eps && z <= d.MaxZ + eps) return d;
            DuctLateral? best = null; double bd = maxMm;
            foreach (var d in sd.DuctsLaterals)
            {
                double cx = (d.MinX + d.MaxX) / 2, cy = (d.MinY + d.MaxY) / 2, cz = (d.MinZ + d.MaxZ) / 2;
                double dist = Math.Sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy) + (z - cz) * (z - cz));
                if (dist < bd) { bd = dist; best = d; }
            }
            return best;
        }

        static double Rel(double v, double lo, double hi)
            => hi - lo <= 1e-6 ? 0.5 : Math.Min(1.0, Math.Max(0.0, (v - lo) / (hi - lo)));

        static double[] Unit3(double x, double y, double z)
        {
            double n = Math.Sqrt(x * x + y * y + z * z);
            return n < 1e-9 ? new double[3] : new[] { x / n, y / n, z / n };
        }

        // === 기존배관 복제(폴리라인) — GUI SceneViewModel.ReplicatePath 미러(헤드리스 검증용) ===
        // 매칭 폴리라인을 6-연결 셀로 복제하고 막힌 구간만 eng.RouteTask 로 국소 수리. 반환=(셀수, 수리횟수) 또는 null.
        static (int cells, int repairs)? ReplicateOne(SceneData sd, TaskInfo t, ExistingPipe pipe,
                                                      Func<int, int, int, bool> blk, Engine eng)
        {
            var g = sd.Grid;
            var ps = pipe.SourcePos ?? pipe.Points[0];
            var pe = pipe.TargetPos ?? pipe.Points[pipe.Points.Count - 1];
            bool fwd = D(t.Sx, t.Sy, t.Sz, ps) + D(t.Gx, t.Gy, t.Gz, pe)
                       <= D(t.Sx, t.Sy, t.Sz, pe) + D(t.Gx, t.Gy, t.Gz, ps);
            var pts = new List<Pt3>(pipe.Points); if (!fwd) pts.Reverse();
            pts.Insert(0, new Pt3(t.Sx, t.Sy, t.Sz)); pts.Add(new Pt3(t.Gx, t.Gy, t.Gz));
            var path = CellPathWithRepair(pts, g, blk, eng, out int reps);
            return path == null ? null : (path.Length, reps);
        }

        // 월드 폴리라인 → 6-연결 셀 경로: 자유 구간은 그대로, 막힌(또는 비인접) 구간만 Repair(A*) 우회.
        //   선두/꼬리 막힘 스킵(묻힌 PoC 대응). ReplicateOne(전체) + GroupCornerRoute(코너만) 공용.
        // 정점쌍을 충돌없는 직교 L(양 축순서)로 잇고, 둘 다 막힐 때만 A* 수리(+직선화). 코너는 앵커로 보존.
        //   GUI SceneViewModel.ReplicateCellPath 미러.
        static PathCell[]? CellPathWithRepair(IReadOnlyList<Pt3> pts, GridMeta g,
                                              Func<int, int, int, bool> blk, Engine eng, out int reps)
        {
            reps = 0;
            PathCell ToCell(Pt3 p) => new(
                Math.Clamp((int)Math.Floor((p.X - g.Ox) / g.CellMm), 0, g.Nx - 1),
                Math.Clamp((int)Math.Floor((p.Y - g.Oy) / g.CellMm), 0, g.Ny - 1),
                Math.Clamp((int)Math.Floor((p.Z - g.Oz) / g.CellMm), 0, g.Nz - 1));
            var wp = new List<PathCell>();
            foreach (var p in pts) { var c = ToCell(p); if (wp.Count == 0 || !wp[wp.Count - 1].Equals(c)) wp.Add(c); }
            if (wp.Count < 2) return null;
            int s = 0; while (s < wp.Count && blk(wp[s].I, wp[s].J, wp[s].K)) s++;
            if (s >= wp.Count) return null;
            var outp = new List<PathCell> { wp[s] };
            for (int w = s + 1; w < wp.Count; w++)
            {
                var target = wp[w];
                if (blk(target.I, target.J, target.K)) continue;
                var prev = outp[outp.Count - 1];
                if (prev.Equals(target)) continue;
                var L = FreeOrthoL(prev, target, blk);
                if (L != null) { outp.AddRange(L); continue; }
                var rep = Repair(eng, g, prev, target);
                if (rep == null) return null;
                rep = StraightenOrtho(rep, blk);
                for (int k = 1; k < rep.Length; k++) outp.Add(rep[k]);
                reps++;
            }
            if (outp.Count < 2) return null;
            // 짧은 수직 jog(말림)만 단일 L 로 흡수 — 큰 패턴 코너는 보존(GUI DeJog 미러).
            return DeJog(outp, blk);
        }

        // 짧은 수직 jog(같은 축·방향 두 런 사이 ≤JOG_MAX 셀 다른축 런)만 충돌없는 L 로 흡수. 큰 코너 보존.
        static PathCell[] DeJog(IReadOnlyList<PathCell> path, Func<int, int, int, bool> blk)
        {
            const int JOG_MAX = 4;
            var pts = new List<PathCell>(path);
            for (int guard = 0; guard < 128 && pts.Count >= 3; guard++)
            {
                var runs = new List<(int axis, int dir, int s, int e)>();
                for (int k = 1; k < pts.Count; k++)
                {
                    int ax = pts[k].I != pts[k - 1].I ? 0 : pts[k].J != pts[k - 1].J ? 1 : 2;
                    int dr = ax == 0 ? Math.Sign(pts[k].I - pts[k - 1].I)
                           : ax == 1 ? Math.Sign(pts[k].J - pts[k - 1].J) : Math.Sign(pts[k].K - pts[k - 1].K);
                    if (runs.Count == 0) { runs.Add((ax, dr, 0, 1)); continue; }
                    var last = runs[runs.Count - 1];
                    if (ax == last.axis && dr == last.dir) runs[runs.Count - 1] = (last.axis, last.dir, last.s, k);
                    else runs.Add((ax, dr, k - 1, k));
                }
                bool changed = false;
                for (int i = 1; i + 1 < runs.Count; i++)
                {
                    if (runs[i - 1].axis == runs[i + 1].axis && runs[i - 1].dir == runs[i + 1].dir
                        && runs[i].e - runs[i].s <= JOG_MAX)
                    {
                        int aIdx = runs[i - 1].s, bIdx = runs[i + 1].e;
                        var L = FreeOrthoL(pts[aIdx], pts[bIdx], blk);
                        if (L != null)
                        {
                            var nu = new List<PathCell>(pts.Count);
                            for (int t = 0; t <= aIdx; t++) nu.Add(pts[t]);
                            nu.AddRange(L);
                            for (int t = bIdx + 1; t < pts.Count; t++) nu.Add(pts[t]);
                            pts = nu; changed = true; break;
                        }
                    }
                }
                if (!changed) break;
            }
            return pts.ToArray();
        }

        // 매칭 그룹배관의 꺾임점(축 전환 코너, 챔퍼<250mm 병합)을 추출해 코너 사이를 Repair(A*)로 잇는다
        //   (각 코너 강제 경유). GUI RouteThroughGroupCorners + GroupCornerWaypoints + RouteThroughWaypoints 미러.
        static PathCell[]? GroupCornerRoute(SceneData sd, TaskInfo t, ExistingPipe pipe,
                                            Func<int, int, int, bool> blk, Engine rep,
                                            out int corners, out int reps)
        {
            corners = 0; reps = 0;
            var g = sd.Grid;
            var ps = pipe.SourcePos ?? pipe.Points[0];
            var pe = pipe.TargetPos ?? pipe.Points[pipe.Points.Count - 1];
            bool fwd = D(t.Sx, t.Sy, t.Sz, ps) + D(t.Gx, t.Gy, t.Gz, pe)
                       <= D(t.Sx, t.Sy, t.Sz, pe) + D(t.Gx, t.Gy, t.Gz, ps);
            var pts = new List<Pt3>(pipe.Points); if (!fwd) pts.Reverse();
            const double MergeMm = 250.0;
            static int Axis(Pt3 a, Pt3 b)
            {
                double dx = Math.Abs(b.X - a.X), dy = Math.Abs(b.Y - a.Y), dz = Math.Abs(b.Z - a.Z);
                return dz >= dx && dz >= dy ? 2 : (dx >= dy ? 0 : 1);
            }
            var wps = new List<Pt3> { new Pt3(t.Sx, t.Sy, t.Sz) };   // ts + 코너 + te (코너만 폴리라인).
            for (int i = 1; i < pts.Count - 1; i++)
                if (Axis(pts[i - 1], pts[i]) != Axis(pts[i], pts[i + 1]))
                {
                    var last = wps[wps.Count - 1];
                    if (wps.Count == 1 || D(last.X, last.Y, last.Z, pts[i]) > MergeMm) wps.Add(pts[i]);
                }
            wps.Add(new Pt3(t.Gx, t.Gy, t.Gz));
            if (wps.Count < 2) return null;
            corners = wps.Count - 2;   // 양 끝(ts/te) 제외한 설계 코너 수.
            return CellPathWithRepair(wps, g, blk, rep, out reps);   // 코너 사이 자유=직선, 막힘만 A* 우회.
        }

        static bool Adj(PathCell a, PathCell b)
            => Math.Abs(a.I - b.I) + Math.Abs(a.J - b.J) + Math.Abs(a.K - b.K) == 1;

        // 직교 경로의 계단·킨크를 충돌 없는 L 로 펴서 꺾임 제거(GUI SceneViewModel.StraightenOrtho 미러).
        static PathCell[] StraightenOrtho(IReadOnlyList<PathCell> path, Func<int, int, int, bool> blk)
        {
            int n = path.Count;
            if (n < 3) return path is PathCell[] a ? a : path.ToArray();
            var outp = new List<PathCell> { path[0] };
            int i = 0;
            while (i < n - 1)
            {
                int pick = i + 1; PathCell[]? pickL = null;
                for (int j = n - 1; j >= i + 2; j--)
                {
                    var L = FreeOrthoL(path[i], path[j], blk);
                    if (L != null) { pick = j; pickL = L; break; }
                }
                if (pickL != null) outp.AddRange(pickL); else outp.Add(path[i + 1]);
                i = pick;
            }
            return outp.ToArray();
        }

        static PathCell[]? FreeOrthoL(PathCell a, PathCell b, Func<int, int, int, bool> blk)
        {
            int dI = b.I - a.I, dJ = b.J - a.J, dK = b.K - a.K;
            int ax = (dI != 0 ? 1 : 0) + (dJ != 0 ? 1 : 0) + (dK != 0 ? 1 : 0);
            if (ax == 0 || ax == 3) return null;
            int[][] orders;
            if (ax == 1) orders = new[] { new[] { 0, 1, 2 } };
            else
            {
                var nz = new List<int>(); if (dI != 0) nz.Add(0); if (dJ != 0) nz.Add(1); if (dK != 0) nz.Add(2);
                orders = new[] { new[] { nz[0], nz[1] }, new[] { nz[1], nz[0] } };
            }
            foreach (var ord in orders)
            {
                var cells = new List<PathCell>(); var cur = a;
                foreach (var axis in ord)
                {
                    int d = axis == 0 ? dI : axis == 1 ? dJ : dK; int s = Math.Sign(d);
                    while ((axis == 0 && cur.I != b.I) || (axis == 1 && cur.J != b.J) || (axis == 2 && cur.K != b.K))
                    {
                        cur = axis == 0 ? new PathCell(cur.I + s, cur.J, cur.K)
                            : axis == 1 ? new PathCell(cur.I, cur.J + s, cur.K)
                                        : new PathCell(cur.I, cur.J, cur.K + s);
                        cells.Add(cur);
                    }
                }
                bool free = true;
                foreach (var c in cells) if (blk(c.I, c.J, c.K)) { free = false; break; }
                if (free) return cells.ToArray();
            }
            return null;
        }

        static PathCell[]? Repair(Engine eng, GridMeta g, PathCell from, PathCell to)
        {
            double ax = g.Ox + (from.I + 0.5) * g.CellMm, ay = g.Oy + (from.J + 0.5) * g.CellMm, az = g.Oz + (from.K + 0.5) * g.CellMm;
            double bx = g.Ox + (to.I + 0.5) * g.CellMm, by = g.Oy + (to.J + 0.5) * g.CellMm, bz = g.Oz + (to.K + 0.5) * g.CellMm;
            try { int task = eng.AddTask(ax, ay, az, bx, by, bz, null, null); var r = eng.RouteTask(task); return r.Success && r.Path.Length >= 1 ? r.Path : null; }
            catch { return null; }
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

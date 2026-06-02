// 기존설계 스텁 패턴 저장소(pgvector) 조회 — L2a 학습 face 적용
// =============================================================================
// [이 파일이 하는 일]
//   Python pattern_learn 이 적재한 route_stub_pattern(집계 뷰 route_stub_template)을 읽어,
//   (anchor_kind, utility_group, utility) 키별 '대표 진출/진입 면(face)'과 'rise'를 메모리에
//   올린다. 뷰어의 PoC 표면 투영(LiftPocToSurface)이 '최근접 면' 대신 '학습된 면'으로 PoC 를
//   빼내도록(예: 덕트=+z 상부, 장비=-z 하부) 일반화하는 데 쓴다(L2a, 엔진/ABI 변경 없음).
//
// [조회 대상] route_stub_template (db/schema/route_stub_pattern.sql 의 집계 뷰)
//   anchor_kind('EQUIP'|'DUCT'), utility_group, utility, face, rise_mm, n(표본수).
//
// [폴백] 정확 키(kind,group,util) → (kind,group) → (kind) 순으로 점차 느슨하게. 모두 없으면 미스.
//   미스/저장소 부재 시 호출자는 기존 최근접-면 규칙으로 자연 폴백(무해).
// =============================================================================
using System;
using System.Collections.Generic;
using Npgsql;

namespace Routing3D.Viewer.Model
{
    /// <summary>학습된 스텁 대표값(진출/진입 면 + rise).</summary>
    public readonly record struct StubTemplate(string Face, double RiseMm, int N);

    public sealed class PatternStore
    {
        // 정확 키 / 그룹 폴백 / 종류 폴백.
        private readonly Dictionary<(string, string, string), StubTemplate> _byKey = new();
        private readonly Dictionary<(string, string), StubTemplate> _byGroup = new();
        private readonly Dictionary<string, StubTemplate> _byKind = new();

        // 검색증강(L3b) — 다중면(bimodal) 키만 원시 표본을 메모리에 올린다. 표본 = [rel(3), dir(3)] 6차원.
        //   rel = 앵커 AABB 내 PoC 상대좌표[0,1], dir = 학습 접근 단위벡터. PoC 컨텍스트별 ANN 으로 면 분기.
        //   단일면 키는 집계 템플릿(_byKey)으로 충분하므로 적재하지 않는다(메모리·잡음 최소).
        private readonly Dictionary<(string, string, string), List<(double[] feat6, string face)>> _annByKey = new();

        /// <summary>적재된 키(템플릿) 수 — 0 이면 저장소가 비었거나 미설치.</summary>
        public int Count => _byKey.Count;

        /// <summary>검색증강(ANN) 적재된 다중면 키 수.</summary>
        public int AnnKeyCount => _annByKey.Count;

        private static string Norm(string? s) => s ?? "";

        /// <summary>route_stub_template 을 읽어 PatternStore 를 만든다. 실패(테이블 없음/연결 불가)면 null.</summary>
        public static PatternStore? TryLoad(DbConfig config)
        {
            try
            {
                var store = new PatternStore();
                // 폴백 집계용: (kind,group)·(kind) 별 face 표본수 누적 → 최다 face 채택.
                var groupVotes = new Dictionary<(string, string), Dictionary<string, int>>();
                var kindVotes = new Dictionary<string, Dictionary<string, int>>();
                var groupRise = new Dictionary<(string, string), (double sum, int n)>();
                var kindRise = new Dictionary<string, (double sum, int n)>();

                using var conn = new NpgsqlConnection(config.ConnectionString);
                conn.Open();
                // 첫 reader 는 블록으로 닫는다 — Npgsql 은 연결당 reader 1개라, 이걸 열어둔 채 ANN 표본용
                // 두 번째 reader(LoadAnnSamples)를 열면 충돌해 전체 로드가 실패한다.
                using (var cmd = new NpgsqlCommand(
                    "SELECT anchor_kind, utility_group, utility, face, rise_mm, n " +
                    "FROM route_stub_template", conn))
                using (var r = cmd.ExecuteReader())
                while (r.Read())
                {
                    string kind = Norm(r.IsDBNull(0) ? null : r.GetString(0));
                    string grp = Norm(r.IsDBNull(1) ? null : r.GetString(1));
                    string util = Norm(r.IsDBNull(2) ? null : r.GetString(2));
                    string face = r.IsDBNull(3) ? "" : r.GetString(3);
                    double rise = r.IsDBNull(4) ? 0 : r.GetDouble(4);
                    int n = r.IsDBNull(5) ? 0 : (int)r.GetInt64(5);
                    if (face.Length == 0) continue;

                    store._byKey[(kind, grp, util)] = new StubTemplate(face, rise, n);

                    void Vote(Dictionary<string, int> d) => d[face] = d.TryGetValue(face, out var v) ? v + n : n;
                    var gk = (kind, grp);
                    if (!groupVotes.TryGetValue(gk, out var gv)) groupVotes[gk] = gv = new();
                    Vote(gv);
                    if (!kindVotes.TryGetValue(kind, out var kv)) kindVotes[kind] = kv = new();
                    Vote(kv);
                    var gr = groupRise.TryGetValue(gk, out var grr) ? grr : (sum: 0.0, n: 0);
                    groupRise[gk] = (gr.sum + rise * n, gr.n + n);
                    var kr = kindRise.TryGetValue(kind, out var krr) ? krr : (sum: 0.0, n: 0);
                    kindRise[kind] = (kr.sum + rise * n, kr.n + n);
                }

                // 폴백 사전 확정(최다 표본 face + 가중평균 rise).
                foreach (var (gk, votes) in groupVotes)
                {
                    string best = ""; int bestN = -1;
                    foreach (var (f, c) in votes) if (c > bestN) { best = f; bestN = c; }
                    var (sum, n) = groupRise[gk];
                    store._byGroup[gk] = new StubTemplate(best, n > 0 ? sum / n : 0, n);
                }
                foreach (var (kind, votes) in kindVotes)
                {
                    string best = ""; int bestN = -1;
                    foreach (var (f, c) in votes) if (c > bestN) { best = f; bestN = c; }
                    var (sum, n) = kindRise[kind];
                    store._byKind[kind] = new StubTemplate(best, n > 0 ? sum / n : 0, n);
                }

                try { store.LoadAnnSamples(conn); }   // 검색증강(L3b): 실패해도 집계는 유지(무해).
                catch { store._annByKey.Clear(); }
                return store.Count > 0 ? store : null;
            }
            catch
            {
                return null;   // 테이블 부재/연결 불가 → 패턴 비활성(호출자 기하 폴백).
            }
        }

        // 검색증강(L3b) — route_stub_pattern 원시 표본에서 다중면(bimodal) 키만 [rel(3),dir(3)] 6차원으로 적재.
        // rel = (poc - anchor_min)/(anchor_max - anchor_min) 클램프[0,1]. dir = dir_unit(학습 접근 단위벡터).
        private void LoadAnnSamples(NpgsqlConnection conn)
        {
            var tmp = new Dictionary<(string, string, string), List<(double[] feat6, string face)>>();
            var faces = new Dictionary<(string, string, string), HashSet<string>>();
            using var cmd = new NpgsqlCommand(
                "SELECT anchor_kind, utility_group, utility, poc_pos, anchor_min, anchor_max, " +
                "dir_unit::text, face FROM route_stub_pattern WHERE face IS NOT NULL", conn);
            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                string kind = Norm(r.IsDBNull(0) ? null : r.GetString(0));
                string grp = Norm(r.IsDBNull(1) ? null : r.GetString(1));
                string util = Norm(r.IsDBNull(2) ? null : r.GetString(2));
                string face = r.IsDBNull(7) ? "" : r.GetString(7);
                if (face.Length == 0 || r.IsDBNull(3) || r.IsDBNull(4) || r.IsDBNull(5)) continue;
                double[] poc = r.GetFieldValue<double[]>(3);
                double[] lo = r.GetFieldValue<double[]>(4);
                double[] hi = r.GetFieldValue<double[]>(5);
                double[] dir = r.IsDBNull(6) ? new double[3] : ParseVec3(r.GetString(6));
                if (poc.Length < 3 || lo.Length < 3 || hi.Length < 3) continue;
                var f6 = new double[6];
                for (int i = 0; i < 3; i++)
                {
                    double span = hi[i] - lo[i];
                    f6[i] = span <= 1e-6 ? 0.5 : Math.Min(1.0, Math.Max(0.0, (poc[i] - lo[i]) / span));
                    f6[3 + i] = dir[i];
                }
                var key = (kind, grp, util);
                if (!tmp.TryGetValue(key, out var lst)) tmp[key] = lst = new();
                lst.Add((f6, face));
                if (!faces.TryGetValue(key, out var fs)) faces[key] = fs = new();
                fs.Add(face);
            }
            // 다중면 키 중 '검증된' 키만 보관. ANN([rel,dir] 최근접)이 집계 다수결보다 leave-one-out 정확도가
            // 뚜렷이(+10pp) 높은 키만 채택한다. 그렇지 않은 키는 [rel,dir] 이 면과 무상관/반상관이라 ANN 이
            // 오히려 해로우므로 집계 템플릿에 맡긴다(자기검증 게이트). 실측: NFW 만 통과(ANN 97% vs 집계 59%),
            // UPW_S/HOT DI_S 는 탈락(ANN 40/31% < 집계 57/67%) → 무분별 ANN 의 회귀(199→192)를 방지.
            const double AnnMargin = 0.10;
            foreach (var kv in tmp)
            {
                if (faces[kv.Key].Count <= 1) continue;   // 단일면 → 집계로 충분.
                var s = kv.Value;
                var vote = new Dictionary<string, int>();
                foreach (var (_, f) in s) vote[f] = vote.TryGetValue(f, out var v) ? v + 1 : 1;
                string maj = ""; int mc = -1;
                foreach (var fv in vote) if (fv.Value > mc) { maj = fv.Key; mc = fv.Value; }
                int aggOk = mc, annOk = 0;
                for (int i = 0; i < s.Count; i++)   // ANN leave-one-out: 자신 제외 최근접 표본의 면.
                {
                    double best = double.MaxValue; string pred = "";
                    for (int j = 0; j < s.Count; j++)
                    {
                        if (i == j) continue;
                        double d = 0;
                        for (int t = 0; t < 6; t++) { double a = s[i].feat6[t] - s[j].feat6[t]; d += a * a; }
                        if (d < best) { best = d; pred = s[j].face; }
                    }
                    if (pred == s[i].face) annOk++;
                }
                if ((double)annOk / s.Count >= (double)aggOk / s.Count + AnnMargin)
                    _annByKey[kv.Key] = s;   // ANN 이 검증상 더 정확한 키만.
            }
        }

        private static double[] ParseVec3(string s)
        {
            var t = s.Trim().TrimStart('[').TrimEnd(']').Split(',');
            var v = new double[3];
            for (int i = 0; i < 3 && i < t.Length; i++)
                double.TryParse(t[i], System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out v[i]);
            return v;
        }

        /// <summary>검색증강(L3b) — 다중면 키에서 PoC 컨텍스트(rel·dir)에 가장 가까운 표본의 면을 반환.
        /// 그 키가 단일면이거나 미적재면 false(호출자는 집계 TryGet 으로 폴백). rel=[0,1]³, dir=단위벡터³.</summary>
        public bool TryGetFaceAnn(string anchorKind, string? group, string? utility,
                                  double[] rel, double[] dir, out string face)
        {
            face = "";
            if (!_annByKey.TryGetValue((Norm(anchorKind), Norm(group), Norm(utility)), out var samples))
                return false;
            double best = double.MaxValue;
            foreach (var (f6, fc) in samples)
            {
                double d = 0;
                for (int i = 0; i < 3; i++) { double a = rel[i] - f6[i]; d += a * a; }
                for (int i = 0; i < 3; i++) { double a = dir[i] - f6[3 + i]; d += a * a; }
                if (d < best) { best = d; face = fc; }
            }
            return face.Length > 0;
        }

        /// <summary>키(kind,group,util)의 학습 면/라이즈를 폴백과 함께 조회. 없으면 false.</summary>
        public bool TryGet(string anchorKind, string? group, string? utility, out StubTemplate tpl)
        {
            string kind = Norm(anchorKind), grp = Norm(group), util = Norm(utility);
            if (_byKey.TryGetValue((kind, grp, util), out tpl)) return true;
            if (_byGroup.TryGetValue((kind, grp), out tpl)) return true;
            if (_byKind.TryGetValue(kind, out tpl)) return true;
            tpl = default;
            return false;
        }
    }
}

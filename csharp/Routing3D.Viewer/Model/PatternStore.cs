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

        /// <summary>적재된 키(템플릿) 수 — 0 이면 저장소가 비었거나 미설치.</summary>
        public int Count => _byKey.Count;

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
                using var cmd = new NpgsqlCommand(
                    "SELECT anchor_kind, utility_group, utility, face, rise_mm, n " +
                    "FROM route_stub_template", conn);
                using var r = cmd.ExecuteReader();
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
                return store.Count > 0 ? store : null;
            }
            catch
            {
                return null;   // 테이블 부재/연결 불가 → 패턴 비활성(호출자 기하 폴백).
            }
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

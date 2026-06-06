// 기존설계 스텁 패턴 저장소 조회 — 공식 TB_ROUTE_SEGMENT_TEMPLATE 기반(L2a 학습 face 적용)
// =============================================================================
// [이 파일이 하는 일]
//   DDW_AI_DB 의 공식 학습자산 TB_ROUTE_SEGMENT_TEMPLATE(사전 적재 20k행)를 읽어,
//   (anchor_kind, utility_group, utility) 키별 '대표 진출/진입 면(face)'을 집계해 메모리에 올린다.
//   뷰어의 PoC 표면 투영(LiftPocToSurface)이 '최근접 면' 대신 '학습된 면'으로 PoC 를 빼내도록
//   (예: 덕트=+z 상부, 장비=-z 하부) 일반화하는 데 쓴다(L2a, 엔진/ABI 변경 없음).
//
// [구 저장소 → 공식 테이블 이관]
//   구: 우리 pgvector 저장소 route_stub_template(pattern_learn --write-db 로 직접 적재 필요).
//   신: 공식 TB_ROUTE_SEGMENT_TEMPLATE — DDW 파이프라인이 미리 적재(우리 학습 불필요).
//     · SEGMENT_ROLE: 'A_EQUIP_STUB'(장비 출발 스텁)=EQUIP · 'C_DUCT_ENTRY'(덕트 진입 스텁)=DUCT
//       ('B_BRIDGE'=중간 구간은 스텁이 아니므로 무시).
//     · face 도출: EQUIP = START_DIR_UNIT 의 우세축(장비 이탈 방향=표면 법선).
//                   DUCT  = END_DIR_UNIT 우세축의 '반대'(덕트 진입 방향의 반대=덕트 표면 법선).
//       실측 검증: EQUIP=-z 지배(4267/5979)·DUCT=+z 지배(2667/5808) — 도메인 규칙과 일치.
//     · rise: LOCAL_POINTS_JSON 의 수직(z) 진폭 추정.
//
// [폴백] 정확 키(kind,group,util) → (kind,group) → (kind) 순. 모두 없으면 미스 → 호출자는
//   기존 최근접-면 규칙으로 자연 폴백(무해). 테이블 부재/연결 불가 시 TryLoad 가 null.
//
// [L3b 검색증강(ANN) 비활성] 공식 테이블에는 앵커 AABB·PoC 절대좌표가 없어 rel[0,1] 특징을
//   재구성할 수 없다 → ANN 면 분기(TryGetFaceAnn)는 항상 false(집계 면으로 폴백). 과거 LOO
//   게이트가 1키만 채택했던 한계 기능이라 회귀 위험 없이 제거.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
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

        /// <summary>검색증강(ANN) 적재 키 수 — 공식 테이블 기반에선 항상 0(L3b 비활성).</summary>
        public int AnnKeyCount => 0;

        private static string Norm(string? s) => s ?? "";

        /// <summary>공식 TB_ROUTE_SEGMENT_TEMPLATE 을 읽어 PatternStore 를 만든다. 실패/빈 결과면 null.</summary>
        public static PatternStore? TryLoad(DbConfig config)
        {
            try
            {
                var store = new PatternStore();
                // 키별 face 표 누적(다수결) + rise 합/표본수.
                var keyVotes = new Dictionary<(string, string, string), Dictionary<string, int>>();
                var keyRise = new Dictionary<(string, string, string), (double sum, int n)>();
                var groupVotes = new Dictionary<(string, string), Dictionary<string, int>>();
                var groupRise = new Dictionary<(string, string), (double sum, int n)>();
                var kindVotes = new Dictionary<string, Dictionary<string, int>>();
                var kindRise = new Dictionary<string, (double sum, int n)>();

                static void Vote(Dictionary<string, int> d, string face) =>
                    d[face] = d.TryGetValue(face, out var v) ? v + 1 : 1;

                using var conn = new NpgsqlConnection(config.ConnectionString);
                conn.Open();
                using (var cmd = new NpgsqlCommand(
                    @"SELECT ""SEGMENT_ROLE"",""UTILITY_GROUP"",""UTILITY"",
                             ""START_DIR_UNIT""::text,""END_DIR_UNIT""::text,""LOCAL_POINTS_JSON""::text
                      FROM ""TB_ROUTE_SEGMENT_TEMPLATE""
                      WHERE ""SEGMENT_ROLE"" IN ('A_EQUIP_STUB','C_DUCT_ENTRY')", conn))
                using (var r = cmd.ExecuteReader())
                while (r.Read())
                {
                    string role = r.IsDBNull(0) ? "" : r.GetString(0);
                    bool isEquip = role == "A_EQUIP_STUB";
                    string kind = isEquip ? "EQUIP" : "DUCT";
                    string grp = Norm(r.IsDBNull(1) ? null : r.GetString(1));
                    string util = Norm(r.IsDBNull(2) ? null : r.GetString(2));

                    // EQUIP=START_DIR(이탈방향), DUCT=END_DIR(진입방향). 둘 중 해당 컬럼 파싱.
                    string? dirText = isEquip
                        ? (r.IsDBNull(3) ? null : r.GetString(3))
                        : (r.IsDBNull(4) ? null : r.GetString(4));
                    var dir = ParseVec3(dirText);
                    if (dir == null) continue;

                    string face = DominantFace(dir);
                    if (!isEquip) face = OppositeFace(face);   // DUCT: 진입방향의 반대 = 표면 법선.
                    if (face.Length == 0) continue;

                    double rise = r.IsDBNull(5) ? 0 : ZExtent(r.GetString(5));

                    var key = (kind, grp, util);
                    if (!keyVotes.TryGetValue(key, out var kvv)) keyVotes[key] = kvv = new();
                    Vote(kvv, face);
                    var krr = keyRise.TryGetValue(key, out var kr0) ? kr0 : (0.0, 0);
                    keyRise[key] = (krr.Item1 + rise, krr.Item2 + 1);

                    var gk = (kind, grp);
                    if (!groupVotes.TryGetValue(gk, out var gvv)) groupVotes[gk] = gvv = new();
                    Vote(gvv, face);
                    var grr = groupRise.TryGetValue(gk, out var gr0) ? gr0 : (0.0, 0);
                    groupRise[gk] = (grr.Item1 + rise, grr.Item2 + 1);

                    if (!kindVotes.TryGetValue(kind, out var kdv)) kindVotes[kind] = kdv = new();
                    Vote(kdv, face);
                    var kdr = kindRise.TryGetValue(kind, out var kd0) ? kd0 : (0.0, 0);
                    kindRise[kind] = (kdr.Item1 + rise, kdr.Item2 + 1);
                }

                // 다수결 face + 평균 rise 로 각 사전 확정.
                static StubTemplate Best(Dictionary<string, int> votes, (double sum, int n) rise)
                {
                    string best = ""; int bestN = -1, total = 0;
                    foreach (var (f, c) in votes) { total += c; if (c > bestN) { best = f; bestN = c; } }
                    return new StubTemplate(best, rise.n > 0 ? rise.sum / rise.n : 0, total);
                }
                foreach (var (k, votes) in keyVotes) store._byKey[k] = Best(votes, keyRise[k]);
                foreach (var (k, votes) in groupVotes) store._byGroup[k] = Best(votes, groupRise[k]);
                foreach (var (k, votes) in kindVotes) store._byKind[k] = Best(votes, kindRise[k]);

                return store.Count > 0 ? store : null;
            }
            catch
            {
                return null;   // 테이블 부재/연결 불가 → 패턴 비활성(호출자 기하 폴백).
            }
        }

        // "{x,y,z}"(배열) 또는 "[x,y,z]"(jsonb) 텍스트 → double[3]. 실패 시 null.
        private static double[]? ParseVec3(string? text)
        {
            if (string.IsNullOrWhiteSpace(text)) return null;
            var t = text.Trim().Trim('{', '}', '[', ']');
            var parts = t.Split(',');
            if (parts.Length < 3) return null;
            var v = new double[3];
            for (int i = 0; i < 3; i++)
                if (!double.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out v[i]))
                    return null;
            return v;
        }

        // 우세축(|성분| 최대)의 부호 면. 예: (0,0,-1)→"-z", (0.9,0,-0.1)→"+x".
        private static string DominantFace(double[] v)
        {
            double ax = Math.Abs(v[0]), ay = Math.Abs(v[1]), az = Math.Abs(v[2]);
            if (az >= ax && az >= ay) return v[2] < 0 ? "-z" : "+z";
            if (ax >= ay) return v[0] < 0 ? "-x" : "+x";
            return v[1] < 0 ? "-y" : "+y";
        }

        private static string OppositeFace(string f) => f.Length == 2
            ? (f[0] == '+' ? "-" : "+") + f[1]
            : f;

        // LOCAL_POINTS_JSON "[[x,y,z],...]" 에서 z(매 3번째) 진폭(max-min) 추정. 파싱 실패 0.
        private static double ZExtent(string json)
        {
            double zmin = double.MaxValue, zmax = double.MinValue;
            int idx = 0;
            foreach (var tok in json.Split(new[] { '[', ']', ',', ' ', '\t' },
                                           StringSplitOptions.RemoveEmptyEntries))
            {
                if (!double.TryParse(tok, NumberStyles.Float, CultureInfo.InvariantCulture, out var val))
                    continue;
                if (idx % 3 == 2) { if (val < zmin) zmin = val; if (val > zmax) zmax = val; }
                idx++;
            }
            return zmax >= zmin ? Math.Abs(zmax - zmin) : 0.0;
        }

        /// <summary>검색증강(L3b) — 공식 테이블 기반에선 비활성. 항상 false(호출자는 집계 TryGet 폴백).</summary>
        public bool TryGetFaceAnn(string anchorKind, string? group, string? utility,
                                  double[] rel, double[] dir, out string face)
        {
            face = "";
            return false;
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

// 그룹(번들) 배관 템플릿 저장소 조회 — 신규 배관설계 활용(L4)
// =============================================================================
// [이 파일이 하는 일]
//   Python routing3d_py.bundle_detect 가 적재한 route_bundle_group(집계 뷰 route_bundle_template)을
//   읽어, (owner_name, utility) 키별 '대표 번들 패턴'(공용 트렁크 고도 trunk_zs · 이격간격 pitch)을
//   메모리에 올린다. 신규 라우팅에서 같은 유틸리티 새 배관을 학습된 공용 랙 고도(trunk_z)에 뭉치게
//   하는 데 쓴다(엔진 rack_levels 로 주입, 엔진/ABI 변경 없음).
//
// [조회 대상] route_bundle_template (db/schema/route_bundle_group.sql 의 집계 뷰)
//   owner_name, utility, trunk_zs(double[]), pitch_mm, n_members.
//
// [폴백] (owner,util) 정확 키 → (util) 유틸 단위(트렁크 고도 합집합). 미스면 빈 결과(호출자 무해 폴백).
//   저장소 부재(테이블 없음)/연결 불가 시 null → 뷰어는 번들 미적용으로 자연 폴백.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Linq;
using Npgsql;

namespace Routing3D.Viewer.Model
{
    /// <summary>학습된 대표 번들 패턴(공용 트렁크 고도 + 이격간격).</summary>
    public sealed class BundleTemplate
    {
        public string? OwnerName { get; init; }
        public string? Utility { get; init; }
        public double[] TrunkZs { get; init; } = Array.Empty<double>();  // 공용 트렁크 고도 후보(mm).
        public double PitchMm { get; init; }
        public int NMembers { get; init; }
    }

    public sealed class BundleStore
    {
        private readonly Dictionary<(string, string), BundleTemplate> _byKey = new();  // (owner,util)
        private readonly Dictionary<string, BundleTemplate> _byUtil = new();           // util 폴백(고도 합집합)
        private readonly Dictionary<string, int> _guidGroup = new();   // ROUTE_PATH_GUID → group_id(표시 강조용).
        private int _groupCount;

        /// <summary>적재된 (owner,util) 키 수 — 0 이면 저장소가 비었거나 미설치.</summary>
        public int Count => _byKey.Count;

        /// <summary>탐지된 번들 그룹 수(route_bundle_group). 0 이면 그룹 표시 비활성.</summary>
        public int GroupCount => _groupCount;

        /// <summary>기존배관 GUID 가 속한 번들 그룹 id(0..). 미소속이면 -1.</summary>
        public int GroupIdOf(string? guid) =>
            guid != null && _guidGroup.TryGetValue(guid, out var gid) ? gid : -1;

        private static string Norm(string? s) => s ?? "";

        /// <summary>route_bundle_template 을 source_file 로 읽어 저장소를 만든다. 실패/빈 결과면 null.</summary>
        public static BundleStore? TryLoad(DbConfig config, string sourceFile)
        {
            if (string.IsNullOrEmpty(sourceFile)) return null;
            try
            {
                var store = new BundleStore();
                // 유틸 폴백 집계용: util → 트렁크 고도 집합 + pitch 표본 + 멤버 합.
                var utilZ = new Dictionary<string, SortedSet<double>>();
                var utilPitch = new Dictionary<string, List<double>>();
                var utilMem = new Dictionary<string, int>();

                using var conn = new NpgsqlConnection(config.ConnectionString);
                conn.Open();
                // 첫 reader 는 '블록 스코프'로 즉시 닫아야 한다 — Npgsql 은 MARS(다중 활성 reader)를 지원하지
                //   않으므로, 이 reader 가 열린 채로 아래 route_bundle_group 두 번째 reader 를 같은 connection 에서
                //   실행하면 예외 → 그룹 멤버십(_guidGroup/_groupCount)이 통째로 유실된다(멤버십 표시·회랑·레인
                //   배정 무력화). using var(메서드 끝까지 유지) 대신 using(...){} 로 스코프 종료 시 닫는다.
                using (var cmd = new NpgsqlCommand(
                    "SELECT owner_name, utility, trunk_zs, pitch_mm, n_members " +
                    "FROM route_bundle_template WHERE source_file=@sf", conn))
                {
                    cmd.Parameters.AddWithValue("@sf", sourceFile);
                    using var r = cmd.ExecuteReader();
                    while (r.Read())
                    {
                        string owner = Norm(r.IsDBNull(0) ? null : r.GetString(0));
                        string util = Norm(r.IsDBNull(1) ? null : r.GetString(1));
                        double[] zs = r.IsDBNull(2) ? Array.Empty<double>() : r.GetFieldValue<double[]>(2);
                        double pitch = r.IsDBNull(3) ? 0 : r.GetDouble(3);
                        int nmem = r.IsDBNull(4) ? 0 : r.GetInt32(4);

                        store._byKey[(owner, util)] = new BundleTemplate
                        {
                            OwnerName = owner, Utility = util, TrunkZs = zs, PitchMm = pitch, NMembers = nmem,
                        };
                        if (!utilZ.TryGetValue(util, out var zset)) utilZ[util] = zset = new SortedSet<double>();
                        foreach (var z in zs) zset.Add(z);
                        if (!utilPitch.TryGetValue(util, out var pl)) utilPitch[util] = pl = new List<double>();
                        if (pitch > 0) pl.Add(pitch);
                        utilMem[util] = (utilMem.TryGetValue(util, out var m) ? m : 0) + nmem;
                    }
                }

                foreach (var (util, zset) in utilZ)
                {
                    var pl = utilPitch.TryGetValue(util, out var p) ? p : new List<double>();
                    store._byUtil[util] = new BundleTemplate
                    {
                        OwnerName = null, Utility = util, TrunkZs = zset.ToArray(),
                        PitchMm = pl.Count > 0 ? Median(pl) : 0.0, NMembers = utilMem[util],
                    };
                }

                // 그룹 멤버(표시 강조용) — route_bundle_group 의 group_id + member_guids 를 guid→group_id 로 적재.
                try
                {
                    using var gcmd = new NpgsqlCommand(
                        "SELECT group_id, member_guids FROM route_bundle_group WHERE source_file=@sf", conn);
                    gcmd.Parameters.AddWithValue("@sf", sourceFile);
                    using var gr = gcmd.ExecuteReader();
                    int maxGid = -1;
                    while (gr.Read())
                    {
                        if (gr.IsDBNull(0) || gr.IsDBNull(1)) continue;
                        int gid = gr.GetInt32(0);
                        var guids = gr.GetFieldValue<string[]>(1);
                        foreach (var gd in guids) if (gd != null) store._guidGroup[gd] = gid;
                        if (gid > maxGid) maxGid = gid;
                    }
                    store._groupCount = maxGid + 1;
                }
                catch { store._guidGroup.Clear(); store._groupCount = 0; }   // 테이블 부재 → 그룹 표시만 비활성(템플릿은 유지).

                return store.Count > 0 ? store : null;
            }
            catch
            {
                return null;   // 테이블/뷰 부재·연결 불가 → 번들 비활성(호출자 폴백).
            }
        }

        private static double Median(List<double> xs)
        {
            var s = xs.OrderBy(v => v).ToList();
            int n = s.Count;
            return n == 0 ? 0.0 : (n % 2 == 1 ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2);
        }

        /// <summary>(owner,util) 정확 키 → util 폴백으로 번들 템플릿을 조회. 미스면 null.</summary>
        public BundleTemplate? TryGet(string? owner, string? utility)
        {
            if (_byKey.TryGetValue((Norm(owner), Norm(utility)), out var t)) return t;
            if (_byUtil.TryGetValue(Norm(utility), out var u)) return u;
            return null;
        }
    }
}

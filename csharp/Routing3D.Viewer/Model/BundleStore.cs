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

        /// <summary>DDW_AI_DB 공식 설계그룹(TB_ROUTE_DESIGN_GROUP)을 읽어 저장소를 만든다. 실패/빈 결과면 null.
        ///   guid→group_id(MEMBER_ROUTE_GUIDS·GROUP_ID)가 핵심 — 그룹강조·레인배정·번들회랑(BuildBundleCorridorCells)을
        ///   복구한다. trunk_zs/pitch 컬럼은 공식 테이블에 없으므로 빈 값(랙 z 주입 MergeBundleLevels 는 무해 폴백).
        ///   공식 테이블엔 그룹(툴) 스코프 키가 없어 전체를 적재한다 — 표시·레인 배정은 모두 씬에 실제 존재하는
        ///   배관 GUID 와 _guidGroup 교집합으로만 동작하므로 GUID 매칭이 곧 자연 스코프(타 툴 그룹은 매칭 0 = 무시).</summary>
        public static BundleStore? TryLoad(DbConfig config)
        {
            try
            {
                var store = new BundleStore();
                var utilMem = new Dictionary<string, int>();

                using var conn = new NpgsqlConnection(config.ConnectionString);
                conn.Open();
                using (var cmd = new NpgsqlCommand(
                    @"SELECT ""GROUP_ID"",""EQUIPMENT_NAME"",""UTILITY_GROUP"",""UTILITY"",""MEMBER_COUNT"",""MEMBER_ROUTE_GUIDS""
                      FROM ""TB_ROUTE_DESIGN_GROUP""", conn))
                using (var r = cmd.ExecuteReader())
                {
                    int maxGid = -1;
                    while (r.Read())
                    {
                        if (r.IsDBNull(0)) continue;
                        int gid = r.GetInt32(0);
                        string owner = Norm(r.IsDBNull(1) ? null : r.GetString(1));
                        string util = Norm(r.IsDBNull(3) ? null : r.GetString(3));
                        int nmem = r.IsDBNull(4) ? 0 : r.GetInt32(4);
                        var guids = r.IsDBNull(5) ? Array.Empty<string>() : r.GetFieldValue<string[]>(5);

                        foreach (var gd in guids) if (gd != null) store._guidGroup[gd] = gid;
                        if (gid > maxGid) maxGid = gid;

                        // (owner,util) 키 — trunk_zs/pitch 는 공식 테이블에 없어 빈 값(멤버수만 의미).
                        store._byKey[(owner, util)] = new BundleTemplate
                        {
                            OwnerName = owner, Utility = util,
                            TrunkZs = Array.Empty<double>(), PitchMm = 0, NMembers = nmem,
                        };
                        utilMem[util] = (utilMem.TryGetValue(util, out var m) ? m : 0) + nmem;
                    }
                    store._groupCount = maxGid + 1;
                }

                foreach (var (util, n) in utilMem)
                    store._byUtil[util] = new BundleTemplate
                    {
                        OwnerName = null, Utility = util,
                        TrunkZs = Array.Empty<double>(), PitchMm = 0, NMembers = n,
                    };

                return store._guidGroup.Count > 0 ? store : null;
            }
            catch
            {
                return null;   // 테이블 부재·연결 불가 → 번들 비활성(호출자 폴백).
            }
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

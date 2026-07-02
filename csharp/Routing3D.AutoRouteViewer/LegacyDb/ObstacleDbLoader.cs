// PostgreSQL(DDW_AI_DB) ??SceneData 濡쒕뜑
// =============================================================================
// [???뚯씪???섎뒗 ??
//   DDW_AI_DB ?먯꽌 ??'?꾨줈?앺듃(=??' ???μ븷臾셋룹옣鍮꽷룹옉?끒룹쥌???뺥듃/?덊꽣??쨌怨듦컙쨌湲곗〈諛곌????쎌뼱
//   SceneData 濡??⑦궎吏뺥븳?? AUTOROUTINGV7(援?DB)? ?ㅽ궎留덇? ?꾨㈃ ?ㅻⅤ誘濡?援?濡쒕뜑瑜??꾩쟾 援먯껜?덈떎.
//
// [援?DB ????DB 留ㅽ븨 ?붿빟]  (?먯꽭?덈뒗 docs/routing3d_ddw_ai_db_migration_analysis.md)
//   쨌 ?꾨줈?앺듃 ??SOURCE_FILE ?먯? ??TB_SPACE_GROUP_INFO(???⑥쐞) + '洹몃９ AABB 怨듦컙援먯감'濡??ㅼ퐫??
//   쨌 醫뚰몴 MIN_*/MAX_* ??AABB_MIN*/AABB_MAX*.
//   쨌 ?μ븷臾?TB_BIM_OBSTACLES ??TB_BIM_OBSTACLE(+COLLISION_PASS 吏곸젒 ?듦낵?뚮옒洹?.
//   쨌 ?λ퉬 TB_BIM_EQUIPMENT(IS_MAIN,POC_LIST jsonb) ??TB_EQUIPMENTS(MAIN_SUB_TYPE).
//   쨌 ?묒뾽(start?뭙nd): ?λ퉬 PoC 諛곗뿴??醫낅떒 醫뚰몴媛 ?놁쓬 ??TB_ROUTE_PATH(SOURCE_POS?뭈ARGET_POS,
//     SOURCE_GUID=TB_POCINSTANCES.INSTANCE_ID 議곗씤)?먯꽌 ?앹꽦. ?숈떆??洹??대━?쇱씤??湲곗〈諛곌????쒕떎.
//   쨌 醫낅떒 TB_DUCT_LATERAL ??TB_LATERAL_PIPE + TB_DUCT(遺꾨━). 怨듦컙 TB_BIM_SPACE_INFO ??TB_SPACE_INFO.
//
// [湲곕낯媛???濡쒖뺄 dev]  host=localhost / 5432 / postgres / dinno / db=DDW_AI_DB.
//   ?댁쁺?먯꽌??PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD ?섍꼍蹂???곗꽑.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using Npgsql;

namespace Routing3D.AutoRouteViewer.LegacyDb
{
    /// <summary>DB ?묒냽 ?ㅼ젙(媛?媛앹껜). PGHOST/PORT/DATABASE/USER/PASSWORD ?섍꼍蹂?섎줈 ??뼱?곌린 媛??</summary>
    public sealed class DbConfig
    {
        public string Host { get; set; } = "localhost"; //"192.168.0.46"
        public int Port { get; set; } = 5432;
        public string Database { get; set; } = "DDW_AI_DB";
        public string User { get; set; } = "postgres";
        public string Password { get; set; } = "dinno"; //dinno3040
        public int TimeoutSec { get; set; } = 5;

        public static DbConfig FromEnv()
        {
            var c = new DbConfig();
            c.Host = Environment.GetEnvironmentVariable("PGHOST") ?? c.Host;
            if (int.TryParse(Environment.GetEnvironmentVariable("PGPORT"), out var p)) c.Port = p;
            c.Database = Environment.GetEnvironmentVariable("PGDATABASE") ?? c.Database;
            c.User = Environment.GetEnvironmentVariable("PGUSER") ?? c.User;
            c.Password = Environment.GetEnvironmentVariable("PGPASSWORD") ?? c.Password;
            return c;
        }

        public string ConnectionString =>
            $"Host={Host};Port={Port};Database={Database};Username={User};Password={Password};Timeout={TimeoutSec};Encoding=UTF8";
    }

    /// <summary>?꾨줈?앺듃(=?? 紐⑸줉 ??ぉ ??TB_SPACE_GROUP_INFO 1?? AABB 濡?媛앹껜瑜?怨듦컙 ?ㅼ퐫?꾪븳??</summary>
    public sealed class ProjectInfo
    {
        public int ProjectId { get; init; }            // 肄ㅻ낫/--dbroute ?명솚??1-based ?쒕쾲.
        public string GroupId { get; init; } = string.Empty;     // TAG_GROUP_ID.
        public string GroupName { get; init; } = string.Empty;   // TAG_GROUP_NM (?? WTNHJ02).
        public string? Bay { get; init; }              // BAY_GROUP_NM.
        public string? Process { get; init; }          // PROCESS_GROUP_NM (?? CLEAN/DIFF).
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;   // 洹몃９ AABB(mm) ??怨듦컙 ?ㅼ퐫??諛뺤뒪.

        /// <summary>援?API ?명솚: SceneViewModel ?깆씠 SourceFile ??李몄“ ??洹몃９紐낆쑝濡??泥?踰덈뱾 ?쒗뵆由?????.</summary>
        public string SourceFile => GroupName;

        /// <summary>肄ㅻ낫諛뺤뒪 ?쒖떆: "GroupName / Bay / Process".</summary>
        public string Display =>
            $"{GroupName} / {Bay ?? "?"} / {Process ?? "?"}";

        public override string ToString() => Display;
    }

    /// <summary>?뱀쭠 ?꾨줈???뺣낫 - route_feature_group_profile ?뚯씠釉????/summary>
    public sealed class FeatureProfileRow
    {
        public string ProjectId { get; init; } = string.Empty;
        public string UtilityGroup { get; init; } = string.Empty;
        public string PreferredSourceFace { get; init; } = "Any";
        public string PreferredTargetFace { get; init; } = "Any";
        public List<double> PreferredRackZs { get; init; } = new();
        public string TrunkCenterlineJson { get; init; } = "[]";
    }

    /// <summary>DB ??SceneData. ?뺤쟻 API.</summary>
    public static class ObstacleDbLoader
    {
        private const double ScopeMarginMm = 500.0;   // 洹몃９ AABB 怨듦컙援먯감 ??寃쎄퀎 ?ъ쑀.

        /// <summary>TB_SPACE_GROUP_INFO ??紐⑤뱺 洹몃９(=?????꾨줈?앺듃濡?諛섑솚(怨듭젙쨌?대쫫 ?? 1-based ?쒕쾲).</summary>
        public static List<ProjectInfo> ListProjects(DbConfig config)
        {
            var list = new List<ProjectInfo>();
            using var conn = new NpgsqlConnection(config.ConnectionString);
            conn.Open();
            using var cmd = new NpgsqlCommand(
                @"SELECT ""TAG_GROUP_ID"",""TAG_GROUP_NM"",""BAY_GROUP_NM"",""PROCESS_GROUP_NM"",
                         ""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
                  FROM ""TB_SPACE_GROUP_INFO""
                  ORDER BY ""PROCESS_GROUP_NM"",""TAG_GROUP_NM""", conn);
            using var r = cmd.ExecuteReader();
            int seq = 1;
            while (r.Read())
            {
                list.Add(new ProjectInfo
                {
                    ProjectId = seq++,
                    GroupId = r.IsDBNull(0) ? string.Empty : r.GetString(0),
                    GroupName = r.IsDBNull(1) ? string.Empty : r.GetString(1),
                    Bay = r.IsDBNull(2) ? null : r.GetString(2),
                    Process = r.IsDBNull(3) ? null : r.GetString(3),
                    MinX = Dbl(r, 4), MinY = Dbl(r, 5), MinZ = Dbl(r, 6),
                    MaxX = Dbl(r, 7), MaxY = Dbl(r, 8), MaxZ = Dbl(r, 9),
                });
            }
            return list;
        }

        /// <summary>?쒕쾲(ProjectId)?쇰줈 ?꾨줈?앺듃瑜?李얠븘 濡쒕뱶(援?int 湲곕컲 ?몄텧遺 ?명솚).</summary>
        public static SceneData LoadScene(DbConfig config, int projectId, double cellMm = 25.0,
                                          bool connectedOnly = true)
        {
            var projects = ListProjects(config);
            var proj = projects.Find(p => p.ProjectId == projectId)
                       ?? throw new InvalidOperationException($"?꾨줈?앺듃 ?쒕쾲 {projectId} 媛 ?놁뒿?덈떎(珥?{projects.Count}媛?.");
            return LoadScene(config, proj, cellMm, connectedOnly);
        }

        /// <summary>??洹몃９(?????μ븷臾셋룹옣鍮꽷룹옉?끒룹쥌?㉱룰났媛꽷룰린議대같愿??洹몃９ AABB 怨듦컙援먯감濡??쎌뼱 SceneData 濡?</summary>
        public static SceneData LoadScene(DbConfig config, ProjectInfo proj, double cellMm = 25.0,
                                          bool connectedOnly = true)
        {
            using var conn = new NpgsqlConnection(config.ConnectionString);
            conn.Open();

            // 洹몃９ AABB(+?ъ쑀)濡?怨듦컙 ?ㅼ퐫??+ ?대━??諛뺤뒪(=寃⑹옄 ?대옩??諛뺤뒪). 諛붾떏/泥쒖옣/湲곕뫁 ??怨듭쑀
            // 嫄댁텞臾쇱? ??諛뺤뒪? '援먯감'?섎㈃ ?ы븿?섎릺, 嫄곕? extent(嫄대Ъ ?꾩껜 443m)????諛뺤뒪濡??섎씪?몃떎.
            double minx = proj.MinX - ScopeMarginMm, maxx = proj.MaxX + ScopeMarginMm;
            double miny = proj.MinY - ScopeMarginMm, maxy = proj.MaxY + ScopeMarginMm;
            double minz = proj.MinZ - ScopeMarginMm, maxz = proj.MaxZ + ScopeMarginMm;
            var data = new SceneData { SourceFile = proj.GroupName };

            void SetXY(NpgsqlCommand c)
            {
                c.Parameters.AddWithValue("@minx", minx); c.Parameters.AddWithValue("@maxx", maxx);
                c.Parameters.AddWithValue("@miny", miny); c.Parameters.AddWithValue("@maxy", maxy);
            }
            // AABB(媛앹껜) ??洹몃９諛뺤뒪(XY) 援먯감 ?좎뼱. Z ??臾댁떆(?꾧퀬 湲곕뫁/諛붾떏 ?ы븿).
            const string IsectXY =
                @" ""AABB_MINX""<=@maxx AND ""AABB_MAXX"">=@minx AND ""AABB_MINY""<=@maxy AND ""AABB_MAXY"">=@miny ";

            // ?? 1) ?μ븷臾???TB_BIM_OBSTACLE (COLLISION_PASS=?듦낵?뚮옒洹? ??
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ"",
                         ""INSTANCE_NAME"",""OST_TYPE"",""DDWORKS_TYPE"",""COLLISION_PASS""
                  FROM ""TB_BIM_OBSTACLE"" WHERE" + IsectXY, conn))
            {
                SetXY(cmd);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    double mnx = Dbl(r, 0), mny = Dbl(r, 1), mnz = Dbl(r, 2);
                    double mxx = Dbl(r, 3), mxy = Dbl(r, 4), mxz = Dbl(r, 5);
                    if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;   // ?댄솕 諛뺤뒪 ?ㅽ궢.
                    string name = r.IsDBNull(6) ? string.Empty : r.GetString(6);
                    // ?먰띁(damper)???뺥듃 ?쇰?吏留??μ븷臾쇰줈 ?ｌ? ?딅뒗??寃쎈줈 留됲옒 諛⑹?).
                    if (name.IndexOf("damper", StringComparison.OrdinalIgnoreCase) >= 0) continue;
                    // 洹몃９ 諛뺤뒪濡??대━????諛붾떏/泥쒖옣 ??嫄대Ъ ?꾩껜 ?щ옒釉뚯쓽 嫄곕? extent 瑜??묒뾽怨듦컙 ?ш린濡??먮Ⅸ??
                    //   ?쇱슦??遺덈?: ?붿쭊? ?대? ?μ븷臾???寃⑹옄(=??諛뺤뒪)留?蹂듭??뷀븯誘濡?誘몃━ ?섎씪??寃곌낵 ?숈씪.
                    mnx = Math.Max(mnx, minx); mny = Math.Max(mny, miny); mnz = Math.Max(mnz, minz);
                    mxx = Math.Min(mxx, maxx); mxy = Math.Min(mxy, maxy); mxz = Math.Min(mxz, maxz);
                    if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;   // 諛뺤뒪 諛뽰씠硫??쒖쇅.
                    data.Obstacles.Add(new ObstacleBox
                    {
                        MinX = mnx, MinY = mny, MinZ = mnz, MaxX = mxx, MaxY = mxy, MaxZ = mxz,
                        Name = name,
                        OstType = r.IsDBNull(7) ? string.Empty : r.GetString(7),
                        DdworksType = r.IsDBNull(8) ? string.Empty : r.GetString(8),
                        PassThroughOverride = r.IsDBNull(9) ? (bool?)null : (r.GetInt64(9) != 0),
                    });
                }
            }

            // ?? 2) ?λ퉬 ??TB_EQUIPMENTS (MAIN_SUB_TYPE='MainTool'=硫붿씤) ??
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""INSTANCE_NAME"",""MAIN_SUB_TYPE"",
                         ""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
                  FROM ""TB_EQUIPMENTS"" WHERE" + IsectXY, conn))
            {
                SetXY(cmd);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    double mnx = Dbl(r, 2), mny = Dbl(r, 3), mnz = Dbl(r, 4);
                    double mxx = Dbl(r, 5), mxy = Dbl(r, 6), mxz = Dbl(r, 7);
                    if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;
                    data.Equipment.Add(new EquipmentBox
                    {
                        Name = r.IsDBNull(0) ? string.Empty : r.GetString(0),
                        IsMain = !r.IsDBNull(1) && string.Equals(r.GetString(1), "MainTool", StringComparison.OrdinalIgnoreCase),
                        MinX = mnx, MinY = mny, MinZ = mnz, MaxX = mxx, MaxY = mxy, MaxZ = mxz,
                    });
                }
            }

            // ?? 3) 醫낅떒 ??TB_LATERAL_PIPE(?덊꽣?? + TB_DUCT(?뺥듃) ??
            LoadDuctLateral(conn, "TB_LATERAL_PIPE", "LATERAL", IsectXY, SetXY, data.DuctsLaterals);
            LoadDuctLateral(conn, "TB_DUCT", "DUCT", IsectXY, SetXY, data.DuctsLaterals);

            // ?? 4) 怨듦컙 ??TB_SPACE_INFO 瑜?洹몃９ AABB(TB_SPACE_GROUP_INFO) 諛뺤뒪濡??대━????
            //   TB_SPACE_INFO ??痢?A/F쨌CSF쨌CR)? 嫄대Ъ ?꾩껜瑜???뒗 嫄곕? AABB ??洹몃?濡?洹몃━硫??덈Т ?щ떎.
            //   媛?怨듦컙 諛뺤뒪瑜?'洹몃９ 諛뺤뒪'(proj ???ㅼ젣 AABB, ?ъ쑀 ?놁쓬)? 3異?援먯감濡??섎씪 ?묒뾽怨듦컙 ?ш린濡?留뚮뱺??
            //   (= ?ъ슜???붿껌: 怨듦컙?곸뿭??TB_SPACE_GROUP_INFO ?쇰줈 ?대━?묓븯怨?媛??곸뿭??cube box 濡??쒖떆.)
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""SPACE_NAME"",""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
                  FROM ""TB_SPACE_INFO"" WHERE" + IsectXY + @" ORDER BY ""AABB_MINZ""", conn))
            {
                SetXY(cmd);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    // 怨듦컙 諛뺤뒪 ??洹몃９ 諛뺤뒪(?대━??. ??異뺤씠?쇰룄 寃뱀튂吏 ?딆쑝硫??댄솕) ?ㅽ궢.
                    double smnx = Math.Max(Dbl(r, 1), proj.MinX), smny = Math.Max(Dbl(r, 2), proj.MinY), smnz = Math.Max(Dbl(r, 3), proj.MinZ);
                    double smxx = Math.Min(Dbl(r, 4), proj.MaxX), smxy = Math.Min(Dbl(r, 5), proj.MaxY), smxz = Math.Min(Dbl(r, 6), proj.MaxZ);
                    if (smxx <= smnx || smxy <= smny || smxz <= smnz) continue;
                    data.Spaces.Add(new SpaceArea
                    {
                        Name = r.IsDBNull(0) ? string.Empty : r.GetString(0),
                        MinX = smnx, MinY = smny, MinZ = smnz,
                        MaxX = smxx, MaxY = smxy, MaxZ = smxz,
                    });
                }
            }

            // ?? 5) ?묒뾽(start?뭙nd) + 湲곗〈諛곌? ??TB_ROUTE_PATH(SOURCE_POS?뭈ARGET_POS) + ?멸렇癒쇳듃 ?대━?쇱씤 ??
            //   route_path 1??= ?묒뾽 1媛??붾뱶?ъ씤?? + 湲곗〈 ?ㅺ퀎諛곌? 1媛??대━?쇱씤). ?섏쓣 ?④퍡 留뚮뱺??
            TryLoadProjectPocs(conn, minx, maxx, miny, maxy, data);

            try { LoadRoutesAndTasks(conn, minx, maxx, miny, maxy, data); }
            catch { /* ?쇱슦???뚯씠釉?遺???ㅽ궎留?李⑥씠 ???묒뾽/湲곗〈諛곌? ?앸왂(?ㅻⅨ ?덉씠?대뒗 ?뺤긽). */ }

            // ?? 5') 諛곌? ?먯옱(?곌껐遺) ??TB_ROUTE_SEGMENT_DETAIL ???ㅼ젣 遺???섎낫/??諛몃툕/?뚮옖吏 ??. ??
            AddRouteEndpointPocs(data);

            try { LoadFittings(conn, minx, maxx, miny, maxy, minz, maxz, data); }
            catch { /* 遺??濡쒕뵫 ?ㅽ뙣 ???먯옱 ?쒖떆留??앸왂(臾댄빐). */ }

            // ?? 6) 寃⑹옄 硫뷀? ??3異?紐⑤몢 '洹몃９ AABB(+?ъ쑀)' 濡??대옩??=???묒뾽怨듦컙).
            //   怨듭쑀 嫄댁텞臾?諛붾떏 ?щ옒釉?XY ?섎갚 m쨌?꾧퀬 湲곕뫁 Z 48 m)??AABB 媛 怨듦컙?꾪꽣??嫄몃━誘濡?
            //   ?μ븷臾?extent 濡?寃⑹옄瑜??≪쑝硫??섏떗???濡???쬆?쒕떎 ??洹몃９諛뺤뒪濡??쒗븳.
            //   ?? ?묒뾽 ?앹젏(SOURCE/TARGET Z)? XY 濡쒕쭔 ?ㅼ퐫?꾨뤌 洹몃９ Z 諛대뱶瑜?踰쀬뼱?????덈떎(??
            //   ?꾩링 ?뺥듃 醫낅떒). Z 諛대뱶瑜??묒뾽 ?앹젏源뚯? ?뺤옣?쒕떎 ??? ??쬆? ?щ옒釉뚯쓽 XY extent ?볦씠??
            //   Z ?뺤옣? ?덉쟾. ???섎㈃ 寃⑹옄 諛??앹젏???쇱슦?낆뿉??議곗슜???ㅽ뙣?쒕떎.
            double gzlo = minz, gzhi = maxz;
            foreach (var t in data.Tasks)
            {
                gzlo = Math.Min(gzlo, Math.Min(t.Sz, t.Gz) - ScopeMarginMm);
                gzhi = Math.Max(gzhi, Math.Max(t.Sz, t.Gz) + ScopeMarginMm);
            }
            data.Grid = ComputeGrid(minx, miny, gzlo, maxx, maxy, gzhi, cellMm);
            return data;
        }

        // 醫낅떒(?뺥듃/?덊꽣?? 怨듯넻 濡쒕뜑 ??TB_LATERAL_PIPE / TB_DUCT ??而щ읆???숈씪(UTILITY/AABB).
        private static void LoadDuctLateral(NpgsqlConnection conn, string table, string category,
                                            string isectXY, Action<NpgsqlCommand> setXY, List<DuctLateral> outList)
        {
            using var cmd = new NpgsqlCommand(
                $@"SELECT ""INSTANCE_NAME"",""UTILITY"",
                          ""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
                   FROM ""{table}"" WHERE" + isectXY, conn);
            setXY(cmd);
            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                double mnx = Dbl(r, 2), mny = Dbl(r, 3), mnz = Dbl(r, 4);
                double mxx = Dbl(r, 5), mxy = Dbl(r, 6), mxz = Dbl(r, 7);
                if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;
                outList.Add(new DuctLateral
                {
                    Name = r.IsDBNull(0) ? string.Empty : r.GetString(0),
                    Category = category,
                    Utility = r.IsDBNull(1) ? null : r.GetString(1),
                    MinX = mnx, MinY = mny, MinZ = mnz, MaxX = mxx, MaxY = mxy, MaxZ = mxz,
                });
            }
        }

        // ?묒뾽 + 湲곗〈諛곌? ??TB_ROUTE_PATH(?ㅻ뜑: 異쒕컻/醫낅떒 PoC쨌?좏떥쨌愿寃? + ?멸렇癒쇳듃(?대━?쇱씤)瑜?
        //   ROUTE_PATH_GUID쨌ORDER ?쒖쑝濡??댁뼱 留뚮뱺?? SOURCE_POS(?λ퉬 PoC)媛 洹몃９諛뺤뒪 ?덉씤 寃쎈줈留?
        private static void LoadRoutesAndTasks(NpgsqlConnection conn,
            double minx, double maxx, double miny, double maxy, SceneData data)
        {
            using var cmd = new NpgsqlCommand(
                @"SELECT s.""ROUTE_PATH_GUID"", rp.""UTILITY_GROUP"", rp.""SOURCE_UTILITY"", rp.""SOURCE_SIZE"",
                         rp.""EQUIPMENT_NAME"", rp.""TARGET_OWNER_NAME"",
                         sd.""FROM_POSX"", sd.""FROM_POSY"", sd.""FROM_POSZ"",
                         sd.""TO_POSX"",   sd.""TO_POSY"",   sd.""TO_POSZ"",
                         rp.""SOURCE_POSX"", rp.""SOURCE_POSY"", rp.""SOURCE_POSZ"",
                         rp.""TARGET_POSX"", rp.""TARGET_POSY"", rp.""TARGET_POSZ""
                    FROM ""TB_ROUTE_SEGMENT_DETAIL"" sd
                    JOIN ""TB_ROUTE_SEGMENTS"" s ON s.""SEGMENT_GUID"" = sd.""SEGMENT_GUID""
                    JOIN ""TB_ROUTE_PATH"" rp    ON rp.""ROUTE_PATH_GUID"" = s.""ROUTE_PATH_GUID""
                   WHERE rp.""SOURCE_POSX"" BETWEEN @minx AND @maxx
                     AND rp.""SOURCE_POSY"" BETWEEN @miny AND @maxy
                   ORDER BY s.""ROUTE_PATH_GUID"", s.""ORDER"", sd.""ORDER""", conn);
            cmd.Parameters.AddWithValue("@minx", minx); cmd.Parameters.AddWithValue("@maxx", maxx);
            cmd.Parameters.AddWithValue("@miny", miny); cmd.Parameters.AddWithValue("@maxy", maxy);

            using var r = cmd.ExecuteReader();
            string? curGuid = null;
            ExistingPipe? cur = null;
            Pt3? curStart = null, curEnd = null;

            void Flush()
            {
                if (cur == null) return;
                if (curStart.HasValue && curEnd.HasValue)
                    TrimToBoundary(cur.Points, curStart.Value, curEnd.Value);
                if (cur.Points.Count >= 2) data.ExistingPipes.Add(cur);
            }
            void AddPt(Pt3 p)
            {
                if (cur!.Points.Count == 0 || Dist2(cur.Points[cur.Points.Count - 1], p) > 1.0)
                    cur.Points.Add(p);
            }

            while (r.Read())
            {
                string g = r.GetString(0);
                if (!string.Equals(curGuid, g, StringComparison.Ordinal))
                {
                    Flush();
                    curGuid = g;
                    string? util = r.IsDBNull(2) ? null : r.GetString(2);
                    string? grp = r.IsDBNull(1) ? null : r.GetString(1);
                    cur = new ExistingPipe
                    {
                        RoutePathGuid = g,
                        Group = grp,
                        Utility = util,
                        DiameterMm = r.IsDBNull(3) ? 0 : ParsePipeSizeMm(r.GetString(3)),
                    };
                    curStart = (r.IsDBNull(12) || r.IsDBNull(13) || r.IsDBNull(14))
                        ? (Pt3?)null : new Pt3(Dbl(r, 12), Dbl(r, 13), Dbl(r, 14));
                    curEnd = (r.IsDBNull(15) || r.IsDBNull(16) || r.IsDBNull(17))
                        ? (Pt3?)null : new Pt3(Dbl(r, 15), Dbl(r, 16), Dbl(r, 17));
                    cur.SourcePos = curStart;
                    cur.TargetPos = curEnd;

                    // ?묒뾽(start?뭙nd) = SOURCE_POS ??TARGET_POS. ????醫뚰몴媛 ?덉뼱???묒뾽 ?앹꽦.
                    if (curStart.HasValue && curEnd.HasValue)
                        data.Tasks.Add(new TaskInfo
                        {
                            Sx = curStart.Value.X, Sy = curStart.Value.Y, Sz = curStart.Value.Z,
                            Gx = curEnd.Value.X, Gy = curEnd.Value.Y, Gz = curEnd.Value.Z,
                            Utility = util, Group = grp,
                            DiameterMm = r.IsDBNull(3) ? 0 : ParsePipeSizeMm(r.GetString(3)),  // SOURCE_SIZE ???묒뾽 愿寃?
                            RoutePathGuid = g,                                   // ?멸렇癒쇳듃 ?곸꽭 議고쉶 ??
                            PocName = r.IsDBNull(4) ? null : r.GetString(4),     // EQUIPMENT_NAME.
                            EndName = r.IsDBNull(5) ? null : r.GetString(5),     // TARGET_OWNER_NAME.
                        });
                }
                // FROM/TO 醫뚰몴媛 NULL ?대㈃ 洹??뺤젏? 嫄대꼫?대떎 ??Dbl ??0.0 ?泥닿? ?대━?쇱씤??
                //   ?먯젏(0,0,0) ?ㅽ뙆?댄겕瑜?二쇱엯?섏? ?딄쾶(?꾩옱 ?곗씠?곗뿏 NULL 0嫄댁씠??諛⑹뼱).
                if (!(r.IsDBNull(6) || r.IsDBNull(7) || r.IsDBNull(8)))
                    AddPt(new Pt3(Dbl(r, 6), Dbl(r, 7), Dbl(r, 8)));
                if (!(r.IsDBNull(9) || r.IsDBNull(10) || r.IsDBNull(11)))
                    AddPt(new Pt3(Dbl(r, 9), Dbl(r, 10), Dbl(r, 11)));
            }
            Flush();
        }

        // ??湲곗〈諛곌?(routePathGuid)???멸렇癒쇳듃 ?곸꽭瑜?(s.ORDER, sd.ORDER) ?쒖쑝濡??쎌뼱 ?쒖떆????由ъ뒪?몃줈.
        //   ?섎떒 '?멸렇癒쇳듃 ?곸꽭' ??뿉???좏깮 諛곌? ?대┃ ???몄텧(蹂꾨룄 吏㏃? 荑쇰━). GUID 鍮덇컪/DB ?덉쇅 ??鍮?由ъ뒪??
        //   媛??멸렇癒쇳듃??INSTANCE_ID 瑜?TB_POCINSTANCES ??LEFT JOIN ??Owner(?뚯쑀 媛앹껜) ??낆쓣 媛?몄삤怨?
        //   ?쒖옉/醫낅떒 POC ?먮뒗 ?쇱슦???ㅻ뜑???ㅼ젣 owner ?대쫫(EQUIPMENT_NAME / TARGET_OWNER_NAME)???㏓텤?몃떎.

        private static void TryLoadProjectPocs(NpgsqlConnection conn,
            double minx, double maxx, double miny, double maxy, SceneData data)
        {
            try { LoadProjectPocs(conn, minx, maxx, miny, maxy, data); }
            catch { /* PoC table/column differences are tolerated; route endpoints still provide PoC markers. */ }
        }

        private static void LoadProjectPocs(NpgsqlConnection conn,
            double minx, double maxx, double miny, double maxy, SceneData data)
        {
            var cols = ColumnSet(conn, "TB_POCINSTANCES");
            if (cols.Count == 0) return;

            string? cx = Pick(cols, "POSX", "POS_X", "POSITION_X", "POINT_X", "X", "POC_POSX", "FROM_POSX");
            string? cy = Pick(cols, "POSY", "POS_Y", "POSITION_Y", "POINT_Y", "Y", "POC_POSY", "FROM_POSY");
            string? cz = Pick(cols, "POSZ", "POS_Z", "POSITION_Z", "POINT_Z", "Z", "POC_POSZ", "FROM_POSZ");
            if (cx == null || cy == null || cz == null) return;

            string? cName = Pick(cols, "POC_NAME", "NAME", "INSTANCE_NAME", "TAG_NAME");
            string? cOwner = Pick(cols, "OWNER_INSTANCE_NAME", "OWNER_NAME", "EQUIPMENT_NAME", "TARGET_OWNER_NAME");
            string? cOwnerId = Pick(cols, "OWNER_INSTANCE_ID", "OWNER_ID", "OWNER_GUID", "INSTANCE_ID");
            string? cOwnerType = Pick(cols, "OWNER_INSTANCE_TYPE", "OWNER_TYPE", "CATEGORY", "TYPE");
            string? cUtility = Pick(cols, "UTILITY", "SOURCE_UTILITY");

            string S(string? c) => c == null ? "NULL" : Q(c);
            using var cmd = new NpgsqlCommand(
                $@"SELECT {Q(cx)}, {Q(cy)}, {Q(cz)}, {S(cName)}, {S(cOwner)}, {S(cOwnerId)}, {S(cOwnerType)}, {S(cUtility)}
                    FROM ""TB_POCINSTANCES""
                   WHERE {Q(cx)} BETWEEN @minx AND @maxx
                     AND {Q(cy)} BETWEEN @miny AND @maxy", conn);
            cmd.Parameters.AddWithValue("@minx", minx); cmd.Parameters.AddWithValue("@maxx", maxx);
            cmd.Parameters.AddWithValue("@miny", miny); cmd.Parameters.AddWithValue("@maxy", maxy);

            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                double x = DblAny(r, 0), y = DblAny(r, 1), z = DblAny(r, 2);
                if (x < minx || x > maxx || y < miny || y > maxy) continue;
                string name = Str(r, 3);
                string owner = Str(r, 4);
                string? ownerId = NullStr(r, 5);
                string ownerType = Str(r, 6);
                string? util = NullStr(r, 7);
                var kind = ClassifyPoc(ownerType, owner, x, y, z, data, out var matchedOwner, out var matchedUtil);
                if (string.IsNullOrWhiteSpace(owner)) owner = matchedOwner;
                if (string.IsNullOrWhiteSpace(util)) util = matchedUtil;
                AddPoc(data, new PocMarker
                {
                    Kind = kind,
                    Name = string.IsNullOrWhiteSpace(name) ? owner : name,
                    OwnerName = owner,
                    OwnerId = ownerId,
                    Utility = util,
                    X = x, Y = y, Z = z,
                });
            }
        }

        private static HashSet<string> ColumnSet(NpgsqlConnection conn, string table)
        {
            var set = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            using var cmd = new NpgsqlCommand(
                @"SELECT column_name FROM information_schema.columns WHERE table_name = @t", conn);
            cmd.Parameters.AddWithValue("@t", table);
            using var r = cmd.ExecuteReader();
            while (r.Read()) set.Add(r.GetString(0));
            return set;
        }

        private static string? Pick(HashSet<string> cols, params string[] names)
        {
            foreach (var n in names) if (cols.Contains(n)) return n;
            return null;
        }

        private static string Q(string identifier) => "\"" + identifier.Replace("\"", "\"\"") + "\"";

        private static string Str(NpgsqlDataReader r, int i) => r.IsDBNull(i) ? string.Empty : Convert.ToString(r.GetValue(i), CultureInfo.InvariantCulture) ?? string.Empty;
        private static string? NullStr(NpgsqlDataReader r, int i)
        {
            var s = Str(r, i);
            return string.IsNullOrWhiteSpace(s) ? null : s;
        }
        private static double DblAny(NpgsqlDataReader r, int i)
        {
            if (r.IsDBNull(i)) return 0.0;
            object v = r.GetValue(i);
            if (v is double d) return d;
            if (v is float f) return f;
            if (v is decimal m) return (double)m;
            if (v is int n) return n;
            if (v is long l) return l;
            return double.TryParse(Convert.ToString(v, CultureInfo.InvariantCulture), NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed) ? parsed : 0.0;
        }

        private static PocOwnerKind ClassifyPoc(string ownerType, string ownerName, double x, double y, double z,
            SceneData data, out string matchedOwner, out string? matchedUtility)
        {
            matchedOwner = ownerName;
            matchedUtility = null;
            string key = (ownerType + " " + ownerName).ToUpperInvariant();
            if (key.Contains("LATERAL"))
            {
                var d = NearestDuctLateral(data, x, y, z, lateralOnly: true);
                if (d != null) { matchedOwner = d.Name; matchedUtility = d.Utility; }
                return PocOwnerKind.Lateral;
            }
            if (key.Contains("DUCT"))
            {
                var d = NearestDuctLateral(data, x, y, z, lateralOnly: false);
                if (d != null) { matchedOwner = d.Name; matchedUtility = d.Utility; return d.IsLateral ? PocOwnerKind.Lateral : PocOwnerKind.Duct; }
                return PocOwnerKind.Duct;
            }
            if (key.Contains("EQUIP") || key.Contains("MODEL") || key.Contains("TOOL"))
            {
                var e = NearestEquipment(data, x, y, z);
                if (e != null) matchedOwner = e.Name;
                return PocOwnerKind.Equipment;
            }

            var eq = NearestEquipment(data, x, y, z);
            var dl = NearestDuctLateral(data, x, y, z, lateralOnly: null);
            double de = eq == null ? double.MaxValue : BoxDistance2(x, y, z, eq.MinX, eq.MinY, eq.MinZ, eq.MaxX, eq.MaxY, eq.MaxZ);
            double dd = dl == null ? double.MaxValue : BoxDistance2(x, y, z, dl.MinX, dl.MinY, dl.MinZ, dl.MaxX, dl.MaxY, dl.MaxZ);
            if (de <= dd)
            {
                if (eq != null) matchedOwner = eq.Name;
                return PocOwnerKind.Equipment;
            }
            if (dl != null) { matchedOwner = dl.Name; matchedUtility = dl.Utility; return dl.IsLateral ? PocOwnerKind.Lateral : PocOwnerKind.Duct; }
            return PocOwnerKind.Unknown;
        }

        private static EquipmentBox? NearestEquipment(SceneData data, double x, double y, double z)
        {
            EquipmentBox? best = null; double bd = double.MaxValue;
            foreach (var e in data.Equipment)
            {
                double d = BoxDistance2(x, y, z, e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
                if (d < bd) { bd = d; best = e; }
            }
            return best;
        }

        private static DuctLateral? NearestDuctLateral(SceneData data, double x, double y, double z, bool? lateralOnly)
        {
            DuctLateral? best = null; double bd = double.MaxValue;
            foreach (var d in data.DuctsLaterals)
            {
                if (lateralOnly.HasValue && d.IsLateral != lateralOnly.Value) continue;
                double dist = BoxDistance2(x, y, z, d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
                if (dist < bd) { bd = dist; best = d; }
            }
            return best;
        }

        private static double BoxDistance2(double x, double y, double z, double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
        {
            double dx = x < mnx ? mnx - x : x > mxx ? x - mxx : 0;
            double dy = y < mny ? mny - y : y > mxy ? y - mxy : 0;
            double dz = z < mnz ? mnz - z : z > mxz ? z - mxz : 0;
            return dx * dx + dy * dy + dz * dz;
        }

        private static void AddPoc(SceneData data, PocMarker p)
        {
            var target = p.Kind == PocOwnerKind.Equipment ? data.EquipmentPocs : data.DuctLateralPocs;
            foreach (var old in target)
            {
                if (Math.Abs(old.X - p.X) < 1.0 && Math.Abs(old.Y - p.Y) < 1.0 && Math.Abs(old.Z - p.Z) < 1.0
                    && string.Equals(old.OwnerName, p.OwnerName, StringComparison.OrdinalIgnoreCase)
                    && string.Equals(old.Name, p.Name, StringComparison.OrdinalIgnoreCase))
                {
                    old.IsRouteStart |= p.IsRouteStart;
                    old.IsRouteEnd |= p.IsRouteEnd;
                    if (string.IsNullOrWhiteSpace(old.RoutePathGuid)) old.RoutePathGuid = p.RoutePathGuid;
                    return;
                }
            }
            target.Add(p);
        }

        private static void AddRouteEndpointPocs(SceneData data)
        {
            foreach (var t in data.Tasks)
            {
                AddPoc(data, new PocMarker
                {
                    Kind = PocOwnerKind.Equipment,
                    Name = string.IsNullOrWhiteSpace(t.PocName) ? "Start PoC" : t.PocName!,
                    OwnerName = string.IsNullOrWhiteSpace(t.PocName) ? "Equipment" : t.PocName!,
                    Utility = t.Utility,
                    Group = t.Group,
                    X = t.Sx, Y = t.Sy, Z = t.Sz,
                    IsRouteStart = true,
                    RoutePathGuid = t.RoutePathGuid,
                });
                var endKind = PocOwnerKind.Duct;
                var near = NearestDuctLateral(data, t.Gx, t.Gy, t.Gz, lateralOnly: null);
                if (near != null && near.IsLateral) endKind = PocOwnerKind.Lateral;
                AddPoc(data, new PocMarker
                {
                    Kind = endKind,
                    Name = string.IsNullOrWhiteSpace(t.EndName) ? "End PoC" : t.EndName!,
                    OwnerName = string.IsNullOrWhiteSpace(t.EndName) ? "Duct/Lateral" : t.EndName!,
                    Utility = t.Utility,
                    Group = t.Group,
                    X = t.Gx, Y = t.Gy, Z = t.Gz,
                    IsRouteEnd = true,
                    RoutePathGuid = t.RoutePathGuid,
                });
            }
        }
        public static List<SegmentDetailRow> LoadSegmentDetail(DbConfig config, string? routePathGuid,
                                                               string? equipmentName = null, string? targetOwnerName = null)
        {
            var outList = new List<SegmentDetailRow>();
            if (string.IsNullOrEmpty(routePathGuid)) return outList;
            using var conn = new NpgsqlConnection(config.ConnectionString);
            conn.Open();
            using var cmd = new NpgsqlCommand(
                @"SELECT sd.""TYPE"", sd.""SIZE"",
                         sd.""FROM_POSX"", sd.""FROM_POSY"", sd.""FROM_POSZ"",
                         sd.""TO_POSX"",   sd.""TO_POSY"",   sd.""TO_POSZ"",
                         p.""OWNER_INSTANCE_TYPE""
                    FROM ""TB_ROUTE_SEGMENT_DETAIL"" sd
                    JOIN ""TB_ROUTE_SEGMENTS"" s ON s.""SEGMENT_GUID"" = sd.""SEGMENT_GUID""
                    LEFT JOIN ""TB_POCINSTANCES"" p ON p.""INSTANCE_ID"" = sd.""INSTANCE_ID""
                   WHERE s.""ROUTE_PATH_GUID"" = @g
                   ORDER BY s.""ORDER"", sd.""ORDER""", conn);
            cmd.Parameters.AddWithValue("@g", routePathGuid);
            using var r = cmd.ExecuteReader();
            int seq = 0;
            while (r.Read())
            {
                bool fv = !(r.IsDBNull(2) || r.IsDBNull(3) || r.IsDBNull(4));
                bool tv = !(r.IsDBNull(5) || r.IsDBNull(6) || r.IsDBNull(7));
                string ownType = r.IsDBNull(8) ? "" : r.GetString(8);
                outList.Add(new SegmentDetailRow
                {
                    Seq = ++seq,
                    Type = r.IsDBNull(0) ? "" : r.GetString(0),
                    Size = r.IsDBNull(1) ? "" : r.GetString(1),
                    Owner = string.IsNullOrEmpty(ownType) ? "-" : ownType,   // 湲곕낯=PoC ?뚯쑀 ??? ?앹젏? ?꾨옒???대쫫 蹂닿컯.
                    Fx = fv ? Dbl(r, 2) : 0, Fy = fv ? Dbl(r, 3) : 0, Fz = fv ? Dbl(r, 4) : 0,
                    Tx = tv ? Dbl(r, 5) : 0, Ty = tv ? Dbl(r, 6) : 0, Tz = tv ? Dbl(r, 7) : 0,
                    FromValid = fv, ToValid = tv,
                });
            }

            // ?쒖옉/醫낅떒 POC ???쇱슦???ㅻ뜑???ㅼ젣 owner ?대쫫???㏓텤?몃떎(?? "Damper-FMPVC-150A-Duct [MODEL]").
            var pocRows = outList.FindAll(x => string.Equals(x.Type, "POC", StringComparison.OrdinalIgnoreCase));
            if (pocRows.Count > 0)
            {
                var first = pocRows[0];
                var last = pocRows[pocRows.Count - 1];
                if (!string.IsNullOrWhiteSpace(equipmentName))
                    first.Owner = $"{equipmentName} [{first.Owner}]";       // ?쒖옉(?λ퉬) PoC.
                if (!string.IsNullOrWhiteSpace(targetOwnerName) && last != first)
                    last.Owner = $"{targetOwnerName} [{last.Owner}]";       // 醫낅떒(?뺥듃/?먰띁 ?? PoC.
            }
            return outList;
        }

        // 諛곌? ?먯옱(?곌껐遺) ??TB_ROUTE_SEGMENT_DETAIL ???ㅼ젣 遺?띾쭔 濡쒕뱶?쒕떎(吏곴?: PIPE=吏곴?쨌POC=?곌껐?먃?
        //   BENDING=?꾩옣踰ㅻ뵫 ? '遺?????꾨땲誘濡??쒖쇅). ?꾩튂=?멸렇癒쇳듃?뷀뀒??FROM/TO 以묒젏, 洹몃９諛뺤뒪濡??대━??
        //   ?ㅼ퐫?꾨뒗 湲곗〈諛곌?怨??숈씪?섍쾶 rp.SOURCE_POSX/Y in bbox(?ㅻⅨ ??諛곌? ?쒖쇅).
        private static void LoadFittings(NpgsqlConnection conn,
            double minx, double maxx, double miny, double maxy, double minz, double maxz, SceneData data)
        {
            using var cmd = new NpgsqlCommand(
                @"SELECT sd.""TYPE"", sd.""SIZE"", rp.""SOURCE_UTILITY"",
                         sd.""FROM_POSX"", sd.""FROM_POSY"", sd.""FROM_POSZ"",
                         sd.""TO_POSX"",   sd.""TO_POSY"",   sd.""TO_POSZ""
                    FROM ""TB_ROUTE_SEGMENT_DETAIL"" sd
                    JOIN ""TB_ROUTE_SEGMENTS"" s ON s.""SEGMENT_GUID"" = sd.""SEGMENT_GUID""
                    JOIN ""TB_ROUTE_PATH"" rp    ON rp.""ROUTE_PATH_GUID"" = s.""ROUTE_PATH_GUID""
                   WHERE rp.""SOURCE_POSX"" BETWEEN @minx AND @maxx
                     AND rp.""SOURCE_POSY"" BETWEEN @miny AND @maxy
                     AND sd.""TYPE"" IS NOT NULL
                     AND sd.""TYPE"" NOT IN ('PIPE','POC','BENDING')", conn);
            cmd.Parameters.AddWithValue("@minx", minx); cmd.Parameters.AddWithValue("@maxx", maxx);
            cmd.Parameters.AddWithValue("@miny", miny); cmd.Parameters.AddWithValue("@maxy", maxy);

            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                string type = r.GetString(0);
                string? size = r.IsDBNull(1) ? null : r.GetString(1);
                string? util = r.IsDBNull(2) ? null : r.GetString(2);
                // 遺??以묒떖 = FROM/TO 以묒젏(?먮??띿? FROM?늇O, 踰ㅻ뵫 ??遺?띿? 吏㏃? 援ш컙).
                double cx = (Dbl(r, 3) + Dbl(r, 6)) * 0.5;
                double cy = (Dbl(r, 4) + Dbl(r, 7)) * 0.5;
                double cz = (Dbl(r, 5) + Dbl(r, 8)) * 0.5;
                // 洹몃９諛뺤뒪 諛??묒뾽?곸뿭 諛? 遺?띿? ?쒖쇅(嫄곕? ?щ옒釉??대━?묎낵 ?숈씪 痍⑥?).
                if (cx < minx || cx > maxx || cy < miny || cy > maxy || cz < minz || cz > maxz) continue;
                data.Fittings.Add(new PipeFitting
                {
                    Type = type, Size = size, Utility = util,
                    X = cx, Y = cy, Z = cz, DiameterMm = ParsePipeSizeMm(size),
                });
            }
        }

        private static double Dbl(NpgsqlDataReader r, int i) => r.IsDBNull(i) ? 0.0 : r.GetDouble(i);
        private static double Dist2(Pt3 a, Pt3 b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
            return dx * dx + dy * dy + dz * dz;
        }

        // 諛곌? ?몄묶寃?臾몄옄?????멸꼍 洹쇱궗(mm). "40A"??0 쨌 "1/2B"??2.7 쨌 ?덈???"1/4BX1/2B")??泥??좏겙.
        private static double ParsePipeSizeMm(string? size)
        {
            if (string.IsNullOrWhiteSpace(size)) return 0;
            string tok = size.Trim().Split('X', 'x')[0].Trim();
            if (tok.Length < 2) return 0;
            char unit = char.ToUpperInvariant(tok[tok.Length - 1]);
            string num = tok.Substring(0, tok.Length - 1).Trim();
            if (unit == 'A')
                return double.TryParse(num, System.Globalization.NumberStyles.Any,
                    System.Globalization.CultureInfo.InvariantCulture, out var mm) ? mm : 0;
            if (unit == 'B')
            {
                double inch = ParseInch(num);
                return inch > 0 ? inch * 25.4 : 0;
            }
            return 0;
        }

        private static double ParseInch(string s)
        {
            s = s.Trim().Replace('-', ' ');   // ?쇳빀??'1-1/4' ??'1 1/4'(?뺤닔+遺꾩닔). ???섎㈃ 0 ?쇰줈 ?뚯떛??
            if (s.Contains('/'))
            {
                var parts = s.Split(' ');
                double whole = 0; string frac = s;
                if (parts.Length == 2) { double.TryParse(parts[0], out whole); frac = parts[1]; }
                var fp = frac.Split('/');
                if (fp.Length == 2 && double.TryParse(fp[0], out var a) && double.TryParse(fp[1], out var b) && b != 0)
                    return whole + a / b;
                return whole;
            }
            return double.TryParse(s, System.Globalization.NumberStyles.Any,
                System.Globalization.CultureInfo.InvariantCulture, out var v) ? v : 0;
        }

        // ?대━?쇱씤??startPos/endPos ??媛??媛源뚯슫 ??vertex ?ъ씠濡??덈떒(SpaceAI TrimToBoundary ?댁떇).
        private static void TrimToBoundary(List<Pt3> path, Pt3 startPos, Pt3 endPos)
        {
            if (path.Count < 2) return;
            int si = 0, ei = path.Count - 1;
            double sb = double.MaxValue, eb = double.MaxValue;
            for (int i = 0; i < path.Count; i++)
            {
                double ds = Dist2(path[i], startPos);
                double de = Dist2(path[i], endPos);
                if (ds < sb) { sb = ds; si = i; }
                if (de < eb) { eb = de; ei = i; }
            }
            if (si > ei) { var t = si; si = ei; ei = t; }
            if (si == 0 && ei == path.Count - 1) return;
            var trimmed = path.GetRange(si, ei - si + 1);
            path.Clear();
            path.AddRange(trimmed);
        }

        // 寃⑹옄 ???대옩??諛뺤뒪(洹몃９ AABB+?ъ쑀) 3異뺤쑝濡??먯젏/?ш린 怨좎젙. ?μ븷臾쇱쓽 嫄곕? extent 臾닿?(?묒뾽怨듦컙留?.
        private static GridMeta ComputeGrid(double cxmin, double cymin, double czmin,
                                            double cxmax, double cymax, double czmax, double cellMm)
        {
            return new GridMeta
            {
                CellMm = cellMm,
                Ox = cxmin, Oy = cymin, Oz = czmin,
                Nx = Math.Max(1, (int)Math.Ceiling((cxmax - cxmin) / cellMm)),
                Ny = Math.Max(1, (int)Math.Ceiling((cymax - cymin) / cellMm)),
                Nz = Math.Max(1, (int)Math.Ceiling((czmax - czmin) / cellMm)),
            };
        }

        /// <summary>?꾨줈?앺듃(?λ퉬 ?쒓렇)??留ㅽ븨???좏떥由ы떚蹂??뱀쭠 ?꾨줈?꾩쓣 route_feature_group_profile ?먯꽌 濡쒕뱶.</summary>
        public static Dictionary<string, FeatureProfileRow> LoadFeatureProfiles(DbConfig config, string projectTag)
        {
            var dict = new Dictionary<string, FeatureProfileRow>(StringComparer.OrdinalIgnoreCase);
            if (string.IsNullOrWhiteSpace(projectTag)) return dict;

            try
            {
                using var conn = new NpgsqlConnection(config.ConnectionString);
                conn.Open();

                using var cmd = new NpgsqlCommand(
                    @"SELECT ""project_id"", ""utility_group"", ""preferred_source_face"",
                             ""preferred_target_face"", ""preferred_rack_zs"", ""trunk_centerline_json""
                      FROM ""route_feature_group_profile""
                      WHERE ""project_id"" = @proj", conn);
                cmd.Parameters.AddWithValue("@proj", projectTag);

                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    string projId = r.IsDBNull(0) ? string.Empty : r.GetString(0);
                    string utilGrp = r.IsDBNull(1) ? string.Empty : r.GetString(1);
                    if (string.IsNullOrEmpty(utilGrp)) continue;

                    string srcFace = r.IsDBNull(2) ? "Any" : r.GetString(2);
                    string tgtFace = r.IsDBNull(3) ? "Any" : r.GetString(3);

                    var zs = new List<double>();
                    if (!r.IsDBNull(4))
                    {
                        var rawObj = r.GetValue(4);
                        if (rawObj is double[] arr)
                        {
                            zs.AddRange(arr);
                        }
                        else if (rawObj is Array rawArr)
                        {
                            foreach (var item in rawArr)
                            {
                                if (item != null)
                                {
                                    zs.Add(Convert.ToDouble(item, CultureInfo.InvariantCulture));
                                }
                            }
                        }
                    }

                    string centerlineJson = r.IsDBNull(5) ? "[]" : r.GetString(5);

                    var row = new FeatureProfileRow
                    {
                        ProjectId = projId,
                        UtilityGroup = utilGrp,
                        PreferredSourceFace = srcFace,
                        PreferredTargetFace = tgtFace,
                        PreferredRackZs = zs,
                        TrunkCenterlineJson = centerlineJson
                    };

                    dict[utilGrp] = row;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[寃쎄퀬] route_feature_group_profile 濡쒕뵫 ?ㅽ뙣: {ex.Message}");
            }

            return dict;
        }
    }
}

// PostgreSQL(DDW_AI_DB) → SceneData 로더
// =============================================================================
// [이 파일이 하는 일]
//   DDW_AI_DB 에서 한 '프로젝트(=툴)' 의 장애물·장비·작업·종단(덕트/레터럴)·공간·기존배관을 읽어
//   SceneData 로 패키징한다. AUTOROUTINGV7(구 DB)와 스키마가 전면 다르므로 구 로더를 완전 교체했다.
//
// [구 DB → 신 DB 매핑 요약]  (자세히는 docs/routing3d_ddw_ai_db_migration_analysis.md)
//   · 프로젝트 키 SOURCE_FILE 폐지 → TB_SPACE_GROUP_INFO(툴 단위) + '그룹 AABB 공간교차'로 스코프.
//   · 좌표 MIN_*/MAX_* → AABB_MIN*/AABB_MAX*.
//   · 장애물 TB_BIM_OBSTACLES → TB_BIM_OBSTACLE(+COLLISION_PASS 직접 통과플래그).
//   · 장비 TB_BIM_EQUIPMENT(IS_MAIN,POC_LIST jsonb) → TB_EQUIPMENTS(MAIN_SUB_TYPE).
//   · 작업(start→end): 장비 PoC 배열엔 종단 좌표가 없음 → TB_ROUTE_PATH(SOURCE_POS→TARGET_POS,
//     SOURCE_GUID=TB_POCINSTANCES.INSTANCE_ID 조인)에서 생성. 동시에 그 폴리라인이 기존배관이 된다.
//   · 종단 TB_DUCT_LATERAL → TB_LATERAL_PIPE + TB_DUCT(분리). 공간 TB_BIM_SPACE_INFO → TB_SPACE_INFO.
//
// [기본값 — 로컬 dev]  host=localhost / 5432 / postgres / dinno / db=DDW_AI_DB.
//   운영에서는 PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD 환경변수 우선.
// =============================================================================
using System;
using System.Collections.Generic;
using Npgsql;

namespace Routing3D.Viewer.Model
{
    /// <summary>DB 접속 설정(값 객체). PGHOST/PORT/DATABASE/USER/PASSWORD 환경변수로 덮어쓰기 가능.</summary>
    public sealed class DbConfig
    {
        public string Host { get; set; } = "localhost";
        public int Port { get; set; } = 5432;
        public string Database { get; set; } = "DDW_AI_DB";
        public string User { get; set; } = "postgres";
        public string Password { get; set; } = "dinno";
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

    /// <summary>프로젝트(=툴) 목록 항목 — TB_SPACE_GROUP_INFO 1행. AABB 로 객체를 공간 스코프한다.</summary>
    public sealed class ProjectInfo
    {
        public int ProjectId { get; init; }            // 콤보/--dbroute 호환용 1-based 순번.
        public string GroupId { get; init; } = string.Empty;     // TAG_GROUP_ID.
        public string GroupName { get; init; } = string.Empty;   // TAG_GROUP_NM (예: WTNHJ02).
        public string? Bay { get; init; }              // BAY_GROUP_NM.
        public string? Process { get; init; }          // PROCESS_GROUP_NM (예: CLEAN/DIFF).
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;   // 그룹 AABB(mm) — 공간 스코프 박스.

        /// <summary>구 API 호환: SceneViewModel 등이 SourceFile 을 참조 → 그룹명으로 대체(번들 템플릿 키 등).</summary>
        public string SourceFile => GroupName;

        /// <summary>콤보박스 표시: "GroupName / Bay / Process".</summary>
        public string Display =>
            $"{GroupName} / {Bay ?? "?"} / {Process ?? "?"}";

        public override string ToString() => Display;
    }

    /// <summary>DB → SceneData. 정적 API.</summary>
    public static class ObstacleDbLoader
    {
        private const double ScopeMarginMm = 500.0;   // 그룹 AABB 공간교차 시 경계 여유.

        /// <summary>TB_SPACE_GROUP_INFO 의 모든 그룹(=툴)을 프로젝트로 반환(공정·이름 순, 1-based 순번).</summary>
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

        /// <summary>순번(ProjectId)으로 프로젝트를 찾아 로드(구 int 기반 호출부 호환).</summary>
        public static SceneData LoadScene(DbConfig config, int projectId, double cellMm = 25.0,
                                          bool connectedOnly = true)
        {
            var projects = ListProjects(config);
            var proj = projects.Find(p => p.ProjectId == projectId)
                       ?? throw new InvalidOperationException($"프로젝트 순번 {projectId} 가 없습니다(총 {projects.Count}개).");
            return LoadScene(config, proj, cellMm, connectedOnly);
        }

        /// <summary>한 그룹(툴)의 장애물·장비·작업·종단·공간·기존배관을 그룹 AABB 공간교차로 읽어 SceneData 로.</summary>
        public static SceneData LoadScene(DbConfig config, ProjectInfo proj, double cellMm = 25.0,
                                          bool connectedOnly = true)
        {
            using var conn = new NpgsqlConnection(config.ConnectionString);
            conn.Open();

            // 그룹 AABB(+여유)로 공간 스코프 + 클리핑 박스(=격자 클램프 박스). 바닥/천장/기둥 등 공유
            // 건축물은 이 박스와 '교차'하면 포함하되, 거대 extent(건물 전체 443m)는 이 박스로 잘라낸다.
            double minx = proj.MinX - ScopeMarginMm, maxx = proj.MaxX + ScopeMarginMm;
            double miny = proj.MinY - ScopeMarginMm, maxy = proj.MaxY + ScopeMarginMm;
            double minz = proj.MinZ - ScopeMarginMm, maxz = proj.MaxZ + ScopeMarginMm;
            var data = new SceneData { SourceFile = proj.GroupName };

            void SetXY(NpgsqlCommand c)
            {
                c.Parameters.AddWithValue("@minx", minx); c.Parameters.AddWithValue("@maxx", maxx);
                c.Parameters.AddWithValue("@miny", miny); c.Parameters.AddWithValue("@maxy", maxy);
            }
            // AABB(객체) ∩ 그룹박스(XY) 교차 술어. Z 는 무시(전고 기둥/바닥 포함).
            const string IsectXY =
                @" ""AABB_MINX""<=@maxx AND ""AABB_MAXX"">=@minx AND ""AABB_MINY""<=@maxy AND ""AABB_MAXY"">=@miny ";

            // ── 1) 장애물 — TB_BIM_OBSTACLE (COLLISION_PASS=통과플래그) ──
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
                    if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;   // 퇴화 박스 스킵.
                    string name = r.IsDBNull(6) ? string.Empty : r.GetString(6);
                    // 댐퍼(damper)는 덕트 일부지만 장애물로 넣지 않는다(경로 막힘 방지).
                    if (name.IndexOf("damper", StringComparison.OrdinalIgnoreCase) >= 0) continue;
                    // 그룹 박스로 클리핑 — 바닥/천장 등 건물 전체 슬래브의 거대 extent 를 작업공간 크기로 자른다.
                    //   라우팅 불변: 엔진은 이미 장애물 ∩ 격자(=이 박스)만 복셀화하므로 미리 잘라도 결과 동일.
                    mnx = Math.Max(mnx, minx); mny = Math.Max(mny, miny); mnz = Math.Max(mnz, minz);
                    mxx = Math.Min(mxx, maxx); mxy = Math.Min(mxy, maxy); mxz = Math.Min(mxz, maxz);
                    if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;   // 박스 밖이면 제외.
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

            // ── 2) 장비 — TB_EQUIPMENTS (MAIN_SUB_TYPE='MainTool'=메인) ──
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

            // ── 3) 종단 — TB_LATERAL_PIPE(레터럴) + TB_DUCT(덕트) ──
            LoadDuctLateral(conn, "TB_LATERAL_PIPE", "LATERAL", IsectXY, SetXY, data.DuctsLaterals);
            LoadDuctLateral(conn, "TB_DUCT", "DUCT", IsectXY, SetXY, data.DuctsLaterals);

            // ── 4) 공간 — TB_SPACE_INFO 를 그룹 AABB(TB_SPACE_GROUP_INFO) 박스로 클리핑 ──
            //   TB_SPACE_INFO 의 층(A/F·CSF·CR)은 건물 전체를 덮는 거대 AABB 라 그대로 그리면 너무 크다.
            //   각 공간 박스를 '그룹 박스'(proj 의 실제 AABB, 여유 없음)와 3축 교차로 잘라 작업공간 크기로 만든다.
            //   (= 사용자 요청: 공간영역을 TB_SPACE_GROUP_INFO 으로 클리핑하고 각 영역을 cube box 로 표시.)
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""SPACE_NAME"",""AABB_MINX"",""AABB_MINY"",""AABB_MINZ"",""AABB_MAXX"",""AABB_MAXY"",""AABB_MAXZ""
                  FROM ""TB_SPACE_INFO"" WHERE" + IsectXY + @" ORDER BY ""AABB_MINZ""", conn))
            {
                SetXY(cmd);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    // 공간 박스 ∩ 그룹 박스(클리핑). 한 축이라도 겹치지 않으면(퇴화) 스킵.
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

            // ── 5) 작업(start→end) + 기존배관 — TB_ROUTE_PATH(SOURCE_POS→TARGET_POS) + 세그먼트 폴리라인 ──
            //   route_path 1행 = 작업 1개(엔드포인트) + 기존 설계배관 1개(폴리라인). 둘을 함께 만든다.
            try { LoadRoutesAndTasks(conn, minx, maxx, miny, maxy, data); }
            catch { /* 라우트 테이블 부재/스키마 차이 → 작업/기존배관 생략(다른 레이어는 정상). */ }

            // ── 5') 배관 자재(연결부) — TB_ROUTE_SEGMENT_DETAIL 의 실제 부속(엘보/티/밸브/플랜지 등). ──
            try { LoadFittings(conn, minx, maxx, miny, maxy, minz, maxz, data); }
            catch { /* 부속 로딩 실패 → 자재 표시만 생략(무해). */ }

            // ── 6) 격자 메타 — 3축 모두 '그룹 AABB(+여유)' 로 클램프(=툴 작업공간).
            //   공유 건축물(바닥 슬래브 XY 수백 m·전고 기둥 Z 48 m)의 AABB 가 공간필터에 걸리므로,
            //   장애물 extent 로 격자를 잡으면 수십억 셀로 폭증한다 → 그룹박스로 제한(작업 Z 는 그룹 Z 가 포함).
            data.Grid = ComputeGrid(minx, miny, minz, maxx, maxy, maxz, cellMm);
            return data;
        }

        // 종단(덕트/레터럴) 공통 로더 — TB_LATERAL_PIPE / TB_DUCT 는 컬럼이 동일(UTILITY/AABB).
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

        // 작업 + 기존배관 — TB_ROUTE_PATH(헤더: 출발/종단 PoC·유틸·관경) + 세그먼트(폴리라인)를
        //   ROUTE_PATH_GUID·ORDER 순으로 이어 만든다. SOURCE_POS(장비 PoC)가 그룹박스 안인 경로만.
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

                    // 작업(start→end) = SOURCE_POS → TARGET_POS. 양 끝 좌표가 있어야 작업 생성.
                    if (curStart.HasValue && curEnd.HasValue)
                        data.Tasks.Add(new TaskInfo
                        {
                            Sx = curStart.Value.X, Sy = curStart.Value.Y, Sz = curStart.Value.Z,
                            Gx = curEnd.Value.X, Gy = curEnd.Value.Y, Gz = curEnd.Value.Z,
                            Utility = util, Group = grp,
                            PocName = r.IsDBNull(4) ? null : r.GetString(4),     // EQUIPMENT_NAME.
                            EndName = r.IsDBNull(5) ? null : r.GetString(5),     // TARGET_OWNER_NAME.
                        });
                }
                AddPt(new Pt3(Dbl(r, 6), Dbl(r, 7), Dbl(r, 8)));
                AddPt(new Pt3(Dbl(r, 9), Dbl(r, 10), Dbl(r, 11)));
            }
            Flush();
        }

        // 배관 자재(연결부) — TB_ROUTE_SEGMENT_DETAIL 의 실제 부속만 로드한다(직관: PIPE=직관·POC=연결점·
        //   BENDING=현장벤딩 은 '부속'이 아니므로 제외). 위치=세그먼트디테일 FROM/TO 중점, 그룹박스로 클리핑.
        //   스코프는 기존배관과 동일하게 rp.SOURCE_POSX/Y in bbox(다른 툴 배관 제외).
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
                // 부속 중심 = FROM/TO 중점(점부속은 FROM≈TO, 벤딩 외 부속은 짧은 구간).
                double cx = (Dbl(r, 3) + Dbl(r, 6)) * 0.5;
                double cy = (Dbl(r, 4) + Dbl(r, 7)) * 0.5;
                double cz = (Dbl(r, 5) + Dbl(r, 8)) * 0.5;
                // 그룹박스 밖(작업영역 밖) 부속은 제외(거대 슬래브 클리핑과 동일 취지).
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

        // 배관 호칭경 문자열 → 외경 근사(mm). "40A"→40 · "1/2B"→12.7 · 레듀서("1/4BX1/2B")는 첫 토큰.
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
            s = s.Trim();
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

        // 폴리라인을 startPos/endPos 에 가장 가까운 두 vertex 사이로 절단(SpaceAI TrimToBoundary 이식).
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

        // 격자 — 클램프 박스(그룹 AABB+여유) 3축으로 원점/크기 고정. 장애물의 거대 extent 무관(작업공간만).
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
    }
}

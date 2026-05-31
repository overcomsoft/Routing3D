// PostgreSQL(AUTOROUTINGV7) → SceneData 로더 — Python routing3d_py 1:1 미러
// =============================================================================
// [이 파일이 하는 일]
//   AUTOROUTINGV7 DB에서 한 프로젝트의 장애물(TB_BIM_OBSTACLES) + 메인 장비의 PoC 페어
//   (TB_BIM_EQUIPMENT.POC_LIST jsonb)를 읽어 라우팅 작업 리스트를 만들고, 장애물 BBOX
//   으로 격자(origin/shape/cell_mm)를 산출해 SceneData 로 패키징한다.
//
// [Python 레퍼런스] (1:1 매핑)
//   routing3d_py/obstacle_db.py PgConnConfig / load_obstacles
//   routing3d_py/scene.py       list_projects / load_scene
//
// [기본값 — 로컬 dev]
//   host=localhost / port=5432 / user=postgres / password=dinno / db=AUTOROUTINGV7
//   운영에서는 PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD 환경변수가 우선.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Text.Json;
using Npgsql;

namespace Routing3D.Viewer.Model
{
    /// <summary>DB 접속 설정(값 객체). PGHOST/PORT/DATABASE/USER/PASSWORD 환경변수로 덮어쓰기 가능.</summary>
    public sealed class DbConfig
    {
        public string Host { get; set; } = "localhost";
        public int Port { get; set; } = 5432;
        public string Database { get; set; } = "AUTOROUTINGV7";
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

    /// <summary>프로젝트 목록 항목(space_project_map 1행).</summary>
    public sealed class ProjectInfo
    {
        public int ProjectId { get; init; }
        public string SourceFile { get; init; } = string.Empty;
        public string? Process { get; init; }
        public string? EquipmentCode { get; init; }

        /// <summary>콤보박스 표시: "[id] process/eq — source_file" (Python ProjectInfo.__str__ 와 동일).</summary>
        public string Display =>
            $"[{ProjectId}] {Process ?? "?"}/{EquipmentCode ?? "?"} — {SourceFile}";

        public override string ToString() => Display;
    }

    /// <summary>DB → SceneData. 정적 API.</summary>
    public static class ObstacleDbLoader
    {
        /// <summary>space_project_map 의 모든 프로젝트를 project_id 오름차순으로 반환.</summary>
        public static List<ProjectInfo> ListProjects(DbConfig config)
        {
            var list = new List<ProjectInfo>();
            using var conn = new NpgsqlConnection(config.ConnectionString);
            conn.Open();
            using var cmd = new NpgsqlCommand(
                "SELECT project_id, source_file, process, equipment_code FROM space_project_map ORDER BY project_id",
                conn);
            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                list.Add(new ProjectInfo
                {
                    ProjectId = r.GetInt32(0),
                    SourceFile = r.IsDBNull(1) ? string.Empty : r.GetString(1),
                    Process = r.IsDBNull(2) ? null : r.GetString(2),
                    EquipmentCode = r.IsDBNull(3) ? null : r.GetString(3),
                });
            }
            return list;
        }

        /// <summary>
        /// 한 프로젝트의 장애물 + PoC 페어를 읽어 SceneData 로 만든다.
        ///   1) space_project_map 에서 source_file 조회
        ///   2) TB_BIM_OBSTACLES 에서 AABB 들 로드
        ///   3) TB_BIM_EQUIPMENT(IS_MAIN=true) 의 POC_LIST jsonb 를 파싱해 작업(start→end) 페어 생성
        ///   4) 장애물 BBOX 로 격자(origin/shape/cell_mm) 산출
        /// connectedOnly=true 면 PoC 의 isConnected=true 인 것만 작업으로 만든다.
        /// </summary>
        public static SceneData LoadScene(DbConfig config, int projectId, double cellMm = 100.0,
                                          bool connectedOnly = true)
        {
            using var conn = new NpgsqlConnection(config.ConnectionString);
            conn.Open();

            // ── 1) project_id → source_file ───────────────────────────────────────
            string sourceFile;
            using (var cmd = new NpgsqlCommand(
                "SELECT source_file FROM space_project_map WHERE project_id=@id", conn))
            {
                cmd.Parameters.AddWithValue("@id", projectId);
                var v = cmd.ExecuteScalar();
                if (v == null || v is DBNull)
                    throw new InvalidOperationException($"project_id={projectId} 가 space_project_map 에 없습니다.");
                sourceFile = (string)v;
            }

            var data = new SceneData();

            // ── 2) 장애물 로드 ────────────────────────────────────────────────────
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""MIN_X"",""MIN_Y"",""MIN_Z"",""MAX_X"",""MAX_Y"",""MAX_Z"",""NAME"",""OST_TYPE"",""DDWORKS_TYPE""
                  FROM ""TB_BIM_OBSTACLES"" WHERE ""SOURCE_FILE""=@sf", conn))
            {
                cmd.Parameters.AddWithValue("@sf", sourceFile);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    double mnx = r.GetDouble(0), mny = r.GetDouble(1), mnz = r.GetDouble(2);
                    double mxx = r.GetDouble(3), mxy = r.GetDouble(4), mxz = r.GetDouble(5);
                    // 퇴화(두께 0) 박스는 건너뛴다(occupancy_from_doc 와 동일).
                    if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;
                    data.Obstacles.Add(new ObstacleBox
                    {
                        MinX = mnx, MinY = mny, MinZ = mnz,
                        MaxX = mxx, MaxY = mxy, MaxZ = mxz,
                        Name = r.IsDBNull(6) ? string.Empty : r.GetString(6),
                        OstType = r.IsDBNull(7) ? string.Empty : r.GetString(7),
                        DdworksType = r.IsDBNull(8) ? string.Empty : r.GetString(8),
                    });
                }
            }

            // ── 3) 메인 장비의 POC_LIST → 작업(start→end) 페어 ────────────────────
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""POC_LIST"" FROM ""TB_BIM_EQUIPMENT""
                  WHERE ""SOURCE_FILE""=@sf AND ""IS_MAIN""=true", conn))
            {
                cmd.Parameters.AddWithValue("@sf", sourceFile);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    if (r.IsDBNull(0)) continue;
                    string pocListJson = r.GetString(0);
                    AppendTasksFromPocList(pocListJson, connectedOnly, data.Tasks);
                }
            }

            // ── 3.5) 장비 박스(TB_BIM_EQUIPMENT) — SOURCE_FILE 조건 전체 장비를 큐브로(시각화용) ──
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""NAME"",""IS_MAIN"",""MIN_X"",""MIN_Y"",""MIN_Z"",""MAX_X"",""MAX_Y"",""MAX_Z""
                  FROM ""TB_BIM_EQUIPMENT"" WHERE ""SOURCE_FILE""=@sf", conn))
            {
                cmd.Parameters.AddWithValue("@sf", sourceFile);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    double mnx = r.GetDouble(2), mny = r.GetDouble(3), mnz = r.GetDouble(4);
                    double mxx = r.GetDouble(5), mxy = r.GetDouble(6), mxz = r.GetDouble(7);
                    if (mxx <= mnx || mxy <= mny || mxz <= mnz) continue;   // 퇴화 박스 스킵.
                    data.Equipment.Add(new EquipmentBox
                    {
                        Name = r.IsDBNull(0) ? string.Empty : r.GetString(0),
                        IsMain = !r.IsDBNull(1) && r.GetBoolean(1),
                        MinX = mnx, MinY = mny, MinZ = mnz,
                        MaxX = mxx, MaxY = mxy, MaxZ = mxz,
                    });
                }
            }

            // ── 3.6) 덕트/레터럴(TB_DUCT_LATERAL) — SOURCE_FILE 조건 전체를 박스로(시각화용) ──
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""NAME"",""CATEGORY"",""UTILITY"",""MIN_X"",""MIN_Y"",""MIN_Z"",""MAX_X"",""MAX_Y"",""MAX_Z""
                  FROM ""TB_DUCT_LATERAL"" WHERE ""SOURCE_FILE""=@sf", conn))
            {
                cmd.Parameters.AddWithValue("@sf", sourceFile);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    data.DuctsLaterals.Add(new DuctLateral
                    {
                        Name = r.IsDBNull(0) ? string.Empty : r.GetString(0),
                        Category = r.IsDBNull(1) ? string.Empty : r.GetString(1),
                        Utility = r.IsDBNull(2) ? null : r.GetString(2),
                        MinX = r.GetDouble(3), MinY = r.GetDouble(4), MinZ = r.GetDouble(5),
                        MaxX = r.GetDouble(6), MaxY = r.GetDouble(7), MaxZ = r.GetDouble(8),
                    });
                }
            }

            // ── 4) 공간 영역(TB_BIM_SPACE_INFO) — CR/A/F/CSF 등 구역 AABB + 이름(시각화용) ──
            using (var cmd = new NpgsqlCommand(
                @"SELECT ""LEVEL_NAME"",""MIN_X"",""MIN_Y"",""MIN_Z"",""MAX_X"",""MAX_Y"",""MAX_Z""
                  FROM ""TB_BIM_SPACE_INFO"" WHERE ""SOURCE_FILE""=@sf
                  ORDER BY ""MIN_Z""", conn))
            {
                cmd.Parameters.AddWithValue("@sf", sourceFile);
                using var r = cmd.ExecuteReader();
                while (r.Read())
                {
                    data.Spaces.Add(new SpaceArea
                    {
                        Name = r.IsDBNull(0) ? string.Empty : r.GetString(0),
                        MinX = r.GetDouble(1), MinY = r.GetDouble(2), MinZ = r.GetDouble(3),
                        MaxX = r.GetDouble(4), MaxY = r.GetDouble(5), MaxZ = r.GetDouble(6),
                    });
                }
            }

            // ── 4.5) 기존 설계배관(TB_ROUTE_PATH) — PoC→종단 폴리라인(SpaceAI 참조) ──
            //   TB_ROUTE_SEGMENT_DETAIL(FROM/TO XYZ) → SEGMENT → PATH 조인. 메인 장비 NAME 매칭 +
            //   PoC(SOURCE_OWNER_POS)가 이 프로젝트 장애물 공간 bbox 안인 경로만(같은 공정 다른 tool 제외).
            //   라우트 테이블이 없는 DB 도 있으므로 실패해도 로드를 막지 않는다(try/catch).
            try
            {
                LoadExistingPipes(conn, sourceFile, data.Obstacles, data.ExistingPipes);
            }
            catch { /* 라우트 테이블 부재/스키마 차이 → 기존배관 생략(다른 레이어는 정상 로드). */ }

            // ── 5) 격자 메타 산출 ─────────────────────────────────────────────────
            data.Grid = ComputeGrid(data.Obstacles, cellMm);
            return data;
        }

        // 기존 설계배관 로드(SpaceAI Space3DRepository.GetExistingPaths 참조).
        //   ROUTE_PATH_GUID 별로 SEGMENT.ORDER → SEGMENT_DETAIL.ORDER 로 FROM/TO 좌표를 이어
        //   폴리라인 1개를 만든다(연속 중복점은 1mm² 이내면 생략). 유틸리티/그룹은 PATH 행에서.
        private static void LoadExistingPipes(NpgsqlConnection conn, string sourceFile,
                                              List<ObstacleBox> obstacles, List<ExistingPipe> outPipes)
        {
            // 프로젝트 공간 bbox(장애물 전체 XY) — 같은 공정의 다른 tool 라우트를 거르기 위함.
            //   장비 NAME(예 'kscta01')이 같은 공정 모든 tool 에서 동일해 NAME 매칭만으론 tool 구분 불가
            //   (CMP 한 프로젝트 선택 시 CMP 10개 tool 배관이 전부 표시되던 문제). 각 tool 은 평면상
            //   떨어져 있어 PoC(SOURCE_OWNER_POS)가 자기 tool 의 장애물 bbox 안에 든다 → bbox 로 필터.
            bool hasBox = obstacles.Count > 0;
            double minx = 0, maxx = 0, miny = 0, maxy = 0;
            if (hasBox)
            {
                minx = miny = double.MaxValue;
                maxx = maxy = double.MinValue;
                foreach (var o in obstacles)
                {
                    if (o.MinX < minx) minx = o.MinX;
                    if (o.MaxX > maxx) maxx = o.MaxX;
                    if (o.MinY < miny) miny = o.MinY;
                    if (o.MaxY > maxy) maxy = o.MaxY;
                }
                const double margin = 1000.0;   // tool 간 갭(>4000mm)보다 작은 마진(경계 PoC 포용).
                minx -= margin; maxx += margin; miny -= margin; maxy += margin;
            }

            // PoC bbox 필터(장애물이 있을 때만). owner=장비 위치이므로 SOURCE_OWNER_POS 로 tool 식별.
            string bboxFilter = hasBox
                ? @"                     AND rp.""SOURCE_OWNER_POSX"" BETWEEN @minx AND @maxx
                     AND rp.""SOURCE_OWNER_POSY"" BETWEEN @miny AND @maxy
"
                : "";

            using var cmd = new NpgsqlCommand(
                @"SELECT s.""ROUTE_PATH_GUID"", rp.""UTILITY_GROUP"", rp.""SOURCE_UTILITY"",
                         sd.""FROM_POSX"", sd.""FROM_POSY"", sd.""FROM_POSZ"",
                         sd.""TO_POSX"",   sd.""TO_POSY"",   sd.""TO_POSZ"",
                         rp.""SOURCE_POSX"", rp.""SOURCE_POSY"", rp.""SOURCE_POSZ"",
                         rp.""TARGET_POSX"", rp.""TARGET_POSY"", rp.""TARGET_POSZ"",
                         rp.""SOURCE_SIZE""
                    FROM ""TB_ROUTE_SEGMENT_DETAIL"" sd
                    JOIN ""TB_ROUTE_SEGMENTS"" s ON s.""SEGMENT_GUID"" = sd.""SEGMENT_GUID""
                    JOIN ""TB_ROUTE_PATH"" rp    ON rp.""ROUTE_PATH_GUID"" = s.""ROUTE_PATH_GUID""
                    JOIN ""TB_BIM_EQUIPMENT"" eq
                      ON eq.""NAME"" = rp.""SOURCE_OWNER_NAME""
                     AND eq.""IS_MAIN"" = true
                     AND eq.""SOURCE_FILE"" = @sf
" + bboxFilter +
                @"                   ORDER BY s.""ROUTE_PATH_GUID"", s.""ORDER"", sd.""ORDER""", conn);
            cmd.Parameters.AddWithValue("@sf", sourceFile);
            if (hasBox)
            {
                cmd.Parameters.AddWithValue("@minx", minx);
                cmd.Parameters.AddWithValue("@maxx", maxx);
                cmd.Parameters.AddWithValue("@miny", miny);
                cmd.Parameters.AddWithValue("@maxy", maxy);
            }

            using var r = cmd.ExecuteReader();
            string? curGuid = null;
            ExistingPipe? cur = null;
            // 현재 경로의 시작/끝 PoC 좌표(폴리라인을 종단 PoC 안쪽으로 자르기 위함). 라우트=BIM 동일 프레임.
            Pt3? curStart = null, curEnd = null;

            void Flush()
            {
                if (cur == null) return;
                // 종단 PoC 안쪽으로 절단(트렁크/매니폴드로 END 너머 연장된 row 제거). SpaceAI TrimToBoundary 이식.
                if (curStart.HasValue && curEnd.HasValue)
                    TrimToBoundary(cur.Points, curStart.Value, curEnd.Value);
                if (cur.Points.Count >= 2) outPipes.Add(cur);
            }

            // 연속 중복점은 추가하지 않는다(SEGMENT_DETAIL 에 길이 0 row 가 있어 그대로 두면
            // 폴리라인에 같은 점이 연달아 들어가고, HelixToolkit AddTube 가 퇴화(NaN) 메시를
            // 만들어 튜브가 통째로 안 보인다 — 기존배관이 표시되지 않던 실제 원인).
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
                    cur = new ExistingPipe
                    {
                        Group = r.IsDBNull(1) ? null : r.GetString(1),
                        Utility = r.IsDBNull(2) ? null : r.GetString(2),
                        DiameterMm = r.IsDBNull(15) ? 0 : ParsePipeSizeMm(r.GetString(15)),
                    };
                    curStart = (r.IsDBNull(9) || r.IsDBNull(10) || r.IsDBNull(11))
                        ? (Pt3?)null : new Pt3(r.GetDouble(9), r.GetDouble(10), r.GetDouble(11));
                    curEnd = (r.IsDBNull(12) || r.IsDBNull(13) || r.IsDBNull(14))
                        ? (Pt3?)null : new Pt3(r.GetDouble(12), r.GetDouble(13), r.GetDouble(14));
                    // 선택 배관(Task) ↔ 기존 설계경로 매칭 키로 종단 PoC 좌표를 보존.
                    cur.SourcePos = curStart;
                    cur.TargetPos = curEnd;
                }
                AddPt(new Pt3(r.GetDouble(3), r.GetDouble(4), r.GetDouble(5)));
                AddPt(new Pt3(r.GetDouble(6), r.GetDouble(7), r.GetDouble(8)));
            }
            Flush();
        }

        private static double Dist2(Pt3 a, Pt3 b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
            return dx * dx + dy * dy + dz * dz;
        }

        // 배관 호칭경 문자열 → 외경 근사(mm). 예: "40A"→40, "150A"→150(A=호칭 DN mm),
        //   "1/2B"→12.7, "1B"→25.4(B=인치×25.4). 레듀서("1/4BX1/2B")는 첫 토큰 사용.
        //   파싱 실패/미상이면 0(렌더에서 기본 지름으로 대체).
        private static double ParsePipeSizeMm(string? size)
        {
            if (string.IsNullOrWhiteSpace(size)) return 0;
            // 레듀서 "AxB" 는 시작측(첫 토큰)으로. 대문자 X 기준 분리.
            string tok = size.Trim().Split('X', 'x')[0].Trim();
            if (tok.Length < 2) return 0;
            char unit = char.ToUpperInvariant(tok[tok.Length - 1]);
            string num = tok.Substring(0, tok.Length - 1).Trim();
            double mm;
            if (unit == 'A')   // A 호칭 = DN(mm) 근사.
            {
                return double.TryParse(num, System.Globalization.NumberStyles.Any,
                    System.Globalization.CultureInfo.InvariantCulture, out mm) ? mm : 0;
            }
            if (unit == 'B')   // B 호칭 = 인치 × 25.4. "1/2", "3/8", "1" 같은 분수/정수.
            {
                double inch = ParseInch(num);
                return inch > 0 ? inch * 25.4 : 0;
            }
            return 0;
        }

        // "1/2", "3/8", "1", "1-1/4" 같은 인치 표기를 double 인치로.
        private static double ParseInch(string s)
        {
            s = s.Trim();
            if (s.Length == 0) return 0;
            var ci = System.Globalization.CultureInfo.InvariantCulture;
            var ns = System.Globalization.NumberStyles.Any;
            // 혼합수 "1-1/4" 또는 "1 1/4".
            s = s.Replace('-', ' ');
            double total = 0; bool any = false;
            foreach (var part in s.Split(' ', StringSplitOptions.RemoveEmptyEntries))
            {
                int slash = part.IndexOf('/');
                if (slash > 0)
                {
                    if (double.TryParse(part.Substring(0, slash), ns, ci, out var nu) &&
                        double.TryParse(part.Substring(slash + 1), ns, ci, out var de) && de != 0)
                    { total += nu / de; any = true; }
                }
                else if (double.TryParse(part, ns, ci, out var w))
                { total += w; any = true; }
            }
            return any ? total : 0;
        }

        // 폴리라인을 startPos/endPos 에 가장 가까운 두 vertex 사이로 절단(SpaceAI ExistingPathFinder.TrimToBoundary 이식).
        //   TB_ROUTE_SEGMENT_DETAIL 에 종단 PoC 너머로 연장된 트렁크/매니폴드 row 가 섞여 있어,
        //   PoC↔종단 안쪽만 남겨 자연 종단을 보장한다. 결과가 2점 미만이면 원본 유지(안전 폴백). 인플레이스 수정.
        private static void TrimToBoundary(List<Pt3> path, Pt3 startPos, Pt3 endPos)
        {
            if (path.Count <= 2) return;
            int sIdx = NearestVertexIndex(path, startPos);
            int eIdx = NearestVertexIndex(path, endPos);
            int lo = Math.Min(sIdx, eIdx);
            int hi = Math.Max(sIdx, eIdx);
            if (hi - lo + 1 < 2) return;               // 너무 좁으면 자르지 않음.
            if (lo == 0 && hi == path.Count - 1) return;  // 변화 없으면 스킵.
            var trimmed = path.GetRange(lo, hi - lo + 1);
            path.Clear();
            path.AddRange(trimmed);
        }

        private static int NearestVertexIndex(List<Pt3> path, Pt3 target)
        {
            int best = 0; double bestSq = double.MaxValue;
            for (int i = 0; i < path.Count; i++)
            {
                double d2 = Dist2(path[i], target);
                if (d2 < bestSq) { bestSq = d2; best = i; }
            }
            return best;
        }

        // POC_LIST(jsonb 문자열) → 작업(start_mm → end_mm) 다수. 각 PoC 의 endPocs 각각이 작업 1건.
        private static void AppendTasksFromPocList(string json, bool connectedOnly, List<TaskInfo> tasks)
        {
            using var doc = JsonDocument.Parse(json);
            if (doc.RootElement.ValueKind != JsonValueKind.Array) return;
            foreach (var poc in doc.RootElement.EnumerateArray())
            {
                if (connectedOnly && poc.TryGetProperty("isConnected", out var conn) &&
                    conn.ValueKind == JsonValueKind.False) continue;
                if (!poc.TryGetProperty("pocPosition", out var startPos) || !TryReadVec3(startPos, out var sx, out var sy, out var sz))
                    continue;
                string? utility = TryGetString(poc, "utility");
                string? group = TryGetString(poc, "utilityGroup");
                string? pocName = TryGetString(poc, "name");
                if (!poc.TryGetProperty("endPocs", out var ends) || ends.ValueKind != JsonValueKind.Array) continue;
                foreach (var ep in ends.EnumerateArray())
                {
                    if (!ep.TryGetProperty("endPocPosition", out var endPos) ||
                        !TryReadVec3(endPos, out var gx, out var gy, out var gz)) continue;
                    tasks.Add(new TaskInfo
                    {
                        Sx = sx, Sy = sy, Sz = sz,
                        Gx = gx, Gy = gy, Gz = gz,
                        Utility = utility,
                        Group = group,
                        PocName = pocName,
                        EndName = TryGetString(ep, "endName"),
                    });
                }
            }
        }

        private static bool TryReadVec3(JsonElement arr, out double x, out double y, out double z)
        {
            x = y = z = 0;
            if (arr.ValueKind != JsonValueKind.Array || arr.GetArrayLength() < 3) return false;
            x = arr[0].GetDouble();
            y = arr[1].GetDouble();
            z = arr[2].GetDouble();
            return true;
        }

        private static string? TryGetString(JsonElement obj, string name)
        {
            if (!obj.TryGetProperty(name, out var v)) return null;
            return v.ValueKind == JsonValueKind.String ? v.GetString() : null;
        }

        // 장애물 BBOX → 격자(origin=lo, shape=ceil((hi-lo)/cell), cell_mm).
        private static GridMeta ComputeGrid(List<ObstacleBox> obs, double cellMm)
        {
            if (obs.Count == 0)
                return new GridMeta { CellMm = cellMm, Ox = 0, Oy = 0, Oz = 0, Nx = 1, Ny = 1, Nz = 1 };
            double lx = double.PositiveInfinity, ly = double.PositiveInfinity, lz = double.PositiveInfinity;
            double hx = double.NegativeInfinity, hy = double.NegativeInfinity, hz = double.NegativeInfinity;
            foreach (var o in obs)
            {
                if (o.MinX < lx) lx = o.MinX; if (o.MinY < ly) ly = o.MinY; if (o.MinZ < lz) lz = o.MinZ;
                if (o.MaxX > hx) hx = o.MaxX; if (o.MaxY > hy) hy = o.MaxY; if (o.MaxZ > hz) hz = o.MaxZ;
            }
            return new GridMeta
            {
                CellMm = cellMm,
                Ox = lx, Oy = ly, Oz = lz,
                Nx = Math.Max(1, (int)Math.Ceiling((hx - lx) / cellMm)),
                Ny = Math.Max(1, (int)Math.Ceiling((hy - ly) / cellMm)),
                Nz = Math.Max(1, (int)Math.Ceiling((hz - lz) / cellMm)),
            };
        }
    }
}

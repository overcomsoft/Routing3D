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

            // ── 5) 격자 메타 산출 ─────────────────────────────────────────────────
            data.Grid = ComputeGrid(data.Obstacles, cellMm);
            return data;
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

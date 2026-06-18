// 자동설계 결과 저장·조회 저장소 (PostgreSQL TB_AUTOROUTING_SESSION / TB_AUTOROUTING_PATH)
// =============================================================================
// 저장: SaveSessionAsync(cfg, session, paths) → SESSION_ID 반환
// 조회: ListSessionsAsync(cfg, groupId?) → 최근 50건 목록
// 로드: LoadPathsAsync(cfg, sessionId)    → 배관별 결과 목록(POLYLINE 포함)
// DDL:  EnsureTablesAsync(cfg)            → 테이블/인덱스 없으면 자동 생성
// =============================================================================
using Npgsql;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace Routing3D.Viewer.Model
{
    /// <summary>TB_AUTOROUTING_SESSION 1행 — 자동설계 배치 세션 메타.</summary>
    public sealed class AutoRoutingSessionRow
    {
        public Guid   SessionId       { get; set; } = Guid.NewGuid();
        public string ProjectGroupId  { get; set; } = "";
        public string ProjectName     { get; set; } = "";
        public string EquipmentName   { get; set; } = "";   // TAG_GROUP_NM (장비호기명)
        public int    CellMm          { get; set; }
        public DateTime RoutedAt      { get; set; } = DateTime.UtcNow;
        public DateTime LastModifiedAt{ get; set; } = DateTime.UtcNow;   // 마지막 데이터생성시간
        public string? RouteMode      { get; set; }
        public int    TotalCount      { get; set; }
        public int    SuccessCount    { get; set; }
        public int    FailCount       { get; set; }
        public double TotalLengthMm   { get; set; }
        public int    ElapsedMs       { get; set; }
        public string? ParamsJson     { get; set; }
        public string? Note           { get; set; }

        /// <summary>세션 선택 목록 표시용.</summary>
        public string Display =>
            $"{LastModifiedAt.ToLocalTime():yyyy-MM-dd HH:mm}  [{EquipmentName}]  " +
            $"cell={CellMm}mm  성공 {SuccessCount}/{TotalCount}  " +
            $"{TotalLengthMm / 1000.0:N1}m  ({RouteMode ?? "-"})";
    }

    /// <summary>TB_AUTOROUTING_PATH 1행 — 배관 1개의 라우팅 결과.</summary>
    public sealed class AutoRoutingPathRow
    {
        public Guid    PathId        { get; set; }
        public Guid    SessionId     { get; set; }
        public int     RouteOrder    { get; set; }
        public Guid?   RoutePathGuid { get; set; }
        public string? UtilityGroup  { get; set; }
        public string? Utility       { get; set; }
        public string? SourceName    { get; set; }
        public string? TargetName    { get; set; }
        public double  DiameterMm    { get; set; }
        public bool    Success       { get; set; }
        public string? FailReason    { get; set; }
        public double  LengthMm      { get; set; }
        public int     TurnCount     { get; set; }
        public int     ElapsedMs     { get; set; }
        /// <summary>렌더 폴리라인 좌표 (world mm). null = 실패/미라우팅.</summary>
        public List<Point3D>? Polyline { get; set; }
    }

    public static class AutoRoutingRepository
    {
        // ── DDL ───────────────────────────────────────────────────────────────
        /// <summary>테이블·인덱스가 없으면 자동 생성한다. 이미 있으면 무시(IF NOT EXISTS).</summary>
        public static async Task EnsureTablesAsync(DbConfig cfg)
        {
            await using var conn = new NpgsqlConnection(cfg.ConnectionString);
            await conn.OpenAsync();
            await using var cmd = conn.CreateCommand();
            cmd.CommandText = @"
CREATE TABLE IF NOT EXISTS ""TB_AUTOROUTING_SESSION"" (
    ""SESSION_ID""        UUID         PRIMARY KEY DEFAULT gen_random_uuid(),
    ""PROJECT_GROUP_ID""  VARCHAR(100) NOT NULL,
    ""PROJECT_NAME""      VARCHAR(200),
    ""EQUIPMENT_NAME""    VARCHAR(200),
    ""CELL_MM""           INTEGER      NOT NULL,
    ""ROUTED_AT""         TIMESTAMPTZ  NOT NULL DEFAULT now(),
    ""LAST_MODIFIED_AT""  TIMESTAMPTZ  NOT NULL DEFAULT now(),
    ""ROUTE_MODE""        VARCHAR(50),
    ""TOTAL_COUNT""       INTEGER,
    ""SUCCESS_COUNT""     INTEGER,
    ""FAIL_COUNT""        INTEGER,
    ""TOTAL_LENGTH_MM""   DOUBLE PRECISION,
    ""ELAPSED_MS""        INTEGER,
    ""PARAMS_JSON""       JSONB,
    ""NOTE""              TEXT
);
CREATE TABLE IF NOT EXISTS ""TB_AUTOROUTING_PATH"" (
    ""PATH_ID""           UUID         PRIMARY KEY DEFAULT gen_random_uuid(),
    ""SESSION_ID""        UUID         NOT NULL
                              REFERENCES ""TB_AUTOROUTING_SESSION""(""SESSION_ID"") ON DELETE CASCADE,
    ""ROUTE_ORDER""       INTEGER,
    ""ROUTE_PATH_GUID""   UUID,
    ""UTILITY_GROUP""     VARCHAR(100),
    ""UTILITY""           VARCHAR(100),
    ""SOURCE_NAME""       VARCHAR(200),
    ""TARGET_NAME""       VARCHAR(200),
    ""DIAMETER_MM""       REAL,
    ""SUCCESS""           BOOLEAN      NOT NULL,
    ""FAIL_REASON""       VARCHAR(50),
    ""LENGTH_MM""         DOUBLE PRECISION,
    ""TURN_COUNT""        INTEGER,
    ""ELAPSED_MS""        INTEGER,
    ""POLYLINE""          JSONB
);
CREATE INDEX IF NOT EXISTS ""IDX_AROUTING_PATH_SESSION""
    ON ""TB_AUTOROUTING_PATH""(""SESSION_ID"");
CREATE INDEX IF NOT EXISTS ""IDX_AROUTING_PATH_GUID""
    ON ""TB_AUTOROUTING_PATH""(""ROUTE_PATH_GUID"")
    WHERE ""ROUTE_PATH_GUID"" IS NOT NULL;
CREATE INDEX IF NOT EXISTS ""IDX_AROUTING_SESSION_GID""
    ON ""TB_AUTOROUTING_SESSION""(""PROJECT_GROUP_ID"", ""LAST_MODIFIED_AT"" DESC);
";
            await cmd.ExecuteNonQueryAsync();
        }

        // ── 저장 ──────────────────────────────────────────────────────────────
        /// <summary>세션 1건과 배관 목록을 트랜잭션으로 저장한다. 반환값 = 저장된 SESSION_ID.</summary>
        public static async Task<Guid> SaveSessionAsync(
            DbConfig cfg,
            AutoRoutingSessionRow session,
            IEnumerable<AutoRoutingPathRow> paths)
        {
            await EnsureTablesAsync(cfg);

            session.SessionId      = Guid.NewGuid();
            session.RoutedAt       = DateTime.UtcNow;
            session.LastModifiedAt = DateTime.UtcNow;

            await using var conn = new NpgsqlConnection(cfg.ConnectionString);
            await conn.OpenAsync();
            await using var tx = await conn.BeginTransactionAsync();

            // 세션 헤더
            await using (var cmd = conn.CreateCommand())
            {
                cmd.CommandText = @"
INSERT INTO ""TB_AUTOROUTING_SESSION""
    (""SESSION_ID"",""PROJECT_GROUP_ID"",""PROJECT_NAME"",""EQUIPMENT_NAME"",""CELL_MM"",
     ""ROUTED_AT"",""LAST_MODIFIED_AT"",""ROUTE_MODE"",
     ""TOTAL_COUNT"",""SUCCESS_COUNT"",""FAIL_COUNT"",
     ""TOTAL_LENGTH_MM"",""ELAPSED_MS"",""PARAMS_JSON"",""NOTE"")
VALUES
    (@sid,@gid,@gname,@eqname,@cell,
     @rat,@lmat,@mode,
     @tot,@succ,@fail,
     @len,@ela,@params::jsonb,@note)";
                AddParam(cmd, "sid",    session.SessionId);
                AddParam(cmd, "gid",    session.ProjectGroupId);
                AddParam(cmd, "gname",  (object?)session.ProjectName  ?? DBNull.Value);
                AddParam(cmd, "eqname", (object?)session.EquipmentName ?? DBNull.Value);
                AddParam(cmd, "cell",   session.CellMm);
                AddParam(cmd, "rat",    session.RoutedAt);
                AddParam(cmd, "lmat",   session.LastModifiedAt);
                AddParam(cmd, "mode",   (object?)session.RouteMode    ?? DBNull.Value);
                AddParam(cmd, "tot",    session.TotalCount);
                AddParam(cmd, "succ",   session.SuccessCount);
                AddParam(cmd, "fail",   session.FailCount);
                AddParam(cmd, "len",    session.TotalLengthMm);
                AddParam(cmd, "ela",    session.ElapsedMs);
                AddParam(cmd, "params", (object?)session.ParamsJson   ?? DBNull.Value);
                AddParam(cmd, "note",   (object?)session.Note         ?? DBNull.Value);
                await cmd.ExecuteNonQueryAsync();
            }

            // 배관 행 일괄 삽입
            foreach (var path in paths)
            {
                await using var cmd = conn.CreateCommand();
                cmd.CommandText = @"
INSERT INTO ""TB_AUTOROUTING_PATH""
    (""PATH_ID"",""SESSION_ID"",""ROUTE_ORDER"",""ROUTE_PATH_GUID"",
     ""UTILITY_GROUP"",""UTILITY"",""SOURCE_NAME"",""TARGET_NAME"",
     ""DIAMETER_MM"",""SUCCESS"",""FAIL_REASON"",
     ""LENGTH_MM"",""TURN_COUNT"",""ELAPSED_MS"",""POLYLINE"")
VALUES
    (@pid,@sid,@ord,@guid,
     @ugrp,@util,@src,@tgt,
     @dia,@succ,@fail,
     @len,@turns,@ela,@poly::jsonb)";
                AddParam(cmd, "pid",   Guid.NewGuid());
                AddParam(cmd, "sid",   session.SessionId);
                AddParam(cmd, "ord",   path.RouteOrder);
                AddParam(cmd, "guid",  (object?)path.RoutePathGuid ?? DBNull.Value);
                AddParam(cmd, "ugrp",  (object?)path.UtilityGroup  ?? DBNull.Value);
                AddParam(cmd, "util",  (object?)path.Utility       ?? DBNull.Value);
                AddParam(cmd, "src",   (object?)path.SourceName    ?? DBNull.Value);
                AddParam(cmd, "tgt",   (object?)path.TargetName    ?? DBNull.Value);
                AddParam(cmd, "dia",   (float)path.DiameterMm);
                AddParam(cmd, "succ",  path.Success);
                AddParam(cmd, "fail",  (object?)path.FailReason    ?? DBNull.Value);
                AddParam(cmd, "len",   path.LengthMm);
                AddParam(cmd, "turns", path.TurnCount);
                AddParam(cmd, "ela",   path.ElapsedMs);
                string? polyJson = SerializePolyline(path.Polyline);
                AddParam(cmd, "poly",  (object?)polyJson ?? DBNull.Value);
                await cmd.ExecuteNonQueryAsync();
            }

            await tx.CommitAsync();
            return session.SessionId;
        }

        // ── 세션 목록 ─────────────────────────────────────────────────────────
        /// <summary>최근 50건. projectGroupId가 null이면 전체 프로젝트.</summary>
        public static async Task<List<AutoRoutingSessionRow>> ListSessionsAsync(
            DbConfig cfg, string? projectGroupId = null)
        {
            await using var conn = new NpgsqlConnection(cfg.ConnectionString);
            await conn.OpenAsync();
            await using var cmd = conn.CreateCommand();

            if (projectGroupId != null)
            {
                cmd.CommandText = @"
SELECT ""SESSION_ID"",""PROJECT_GROUP_ID"",""PROJECT_NAME"",""EQUIPMENT_NAME"",""CELL_MM"",
       ""ROUTED_AT"",""LAST_MODIFIED_AT"",""ROUTE_MODE"",
       ""TOTAL_COUNT"",""SUCCESS_COUNT"",""FAIL_COUNT"",
       ""TOTAL_LENGTH_MM"",""ELAPSED_MS"",""PARAMS_JSON"",""NOTE""
FROM ""TB_AUTOROUTING_SESSION""
WHERE ""PROJECT_GROUP_ID"" = @gid
ORDER BY ""LAST_MODIFIED_AT"" DESC
LIMIT 50";
                AddParam(cmd, "gid", projectGroupId);
            }
            else
            {
                cmd.CommandText = @"
SELECT ""SESSION_ID"",""PROJECT_GROUP_ID"",""PROJECT_NAME"",""EQUIPMENT_NAME"",""CELL_MM"",
       ""ROUTED_AT"",""LAST_MODIFIED_AT"",""ROUTE_MODE"",
       ""TOTAL_COUNT"",""SUCCESS_COUNT"",""FAIL_COUNT"",
       ""TOTAL_LENGTH_MM"",""ELAPSED_MS"",""PARAMS_JSON"",""NOTE""
FROM ""TB_AUTOROUTING_SESSION""
ORDER BY ""LAST_MODIFIED_AT"" DESC
LIMIT 50";
            }

            var result = new List<AutoRoutingSessionRow>();
            await using var r = await cmd.ExecuteReaderAsync();
            while (await r.ReadAsync())
                result.Add(ReadSession(r));
            return result;
        }

        // ── 배관 로드 ─────────────────────────────────────────────────────────
        /// <summary>세션에 속한 배관 목록 전체를 ROUTE_ORDER 순으로 반환.</summary>
        public static async Task<List<AutoRoutingPathRow>> LoadPathsAsync(DbConfig cfg, Guid sessionId)
        {
            await using var conn = new NpgsqlConnection(cfg.ConnectionString);
            await conn.OpenAsync();
            await using var cmd = conn.CreateCommand();
            cmd.CommandText = @"
SELECT ""PATH_ID"",""SESSION_ID"",""ROUTE_ORDER"",""ROUTE_PATH_GUID"",
       ""UTILITY_GROUP"",""UTILITY"",""SOURCE_NAME"",""TARGET_NAME"",
       ""DIAMETER_MM"",""SUCCESS"",""FAIL_REASON"",
       ""LENGTH_MM"",""TURN_COUNT"",""ELAPSED_MS"",""POLYLINE""
FROM ""TB_AUTOROUTING_PATH""
WHERE ""SESSION_ID"" = @sid
ORDER BY ""ROUTE_ORDER"", ""PATH_ID""";
            AddParam(cmd, "sid", sessionId);

            var result = new List<AutoRoutingPathRow>();
            await using var r = await cmd.ExecuteReaderAsync();
            while (await r.ReadAsync())
                result.Add(ReadPath(r));
            return result;
        }

        // ── 내부 헬퍼 ─────────────────────────────────────────────────────────
        private static void AddParam(NpgsqlCommand cmd, string name, object value)
            => cmd.Parameters.AddWithValue(name, value);

        private static AutoRoutingSessionRow ReadSession(NpgsqlDataReader r) => new()
        {
            SessionId      = r.GetGuid(0),
            ProjectGroupId = r.GetString(1),
            ProjectName    = r.IsDBNull(2)  ? "" : r.GetString(2),
            EquipmentName  = r.IsDBNull(3)  ? "" : r.GetString(3),
            CellMm         = r.GetInt32(4),
            RoutedAt       = r.GetDateTime(5),
            LastModifiedAt = r.GetDateTime(6),
            RouteMode      = r.IsDBNull(7)  ? null : r.GetString(7),
            TotalCount     = r.IsDBNull(8)  ? 0 : r.GetInt32(8),
            SuccessCount   = r.IsDBNull(9)  ? 0 : r.GetInt32(9),
            FailCount      = r.IsDBNull(10) ? 0 : r.GetInt32(10),
            TotalLengthMm  = r.IsDBNull(11) ? 0 : r.GetDouble(11),
            ElapsedMs      = r.IsDBNull(12) ? 0 : r.GetInt32(12),
            ParamsJson     = r.IsDBNull(13) ? null : r.GetString(13),
            Note           = r.IsDBNull(14) ? null : r.GetString(14),
        };

        private static AutoRoutingPathRow ReadPath(NpgsqlDataReader r) => new()
        {
            PathId        = r.GetGuid(0),
            SessionId     = r.GetGuid(1),
            RouteOrder    = r.IsDBNull(2)  ? 0    : r.GetInt32(2),
            RoutePathGuid = r.IsDBNull(3)  ? null : r.GetGuid(3),
            UtilityGroup  = r.IsDBNull(4)  ? null : r.GetString(4),
            Utility       = r.IsDBNull(5)  ? null : r.GetString(5),
            SourceName    = r.IsDBNull(6)  ? null : r.GetString(6),
            TargetName    = r.IsDBNull(7)  ? null : r.GetString(7),
            DiameterMm    = r.IsDBNull(8)  ? 0    : r.GetFloat(8),
            Success       = r.GetBoolean(9),
            FailReason    = r.IsDBNull(10) ? null : r.GetString(10),
            LengthMm      = r.IsDBNull(11) ? 0    : r.GetDouble(11),
            TurnCount     = r.IsDBNull(12) ? 0    : r.GetInt32(12),
            ElapsedMs     = r.IsDBNull(13) ? 0    : r.GetInt32(13),
            Polyline      = r.IsDBNull(14) ? null : DeserializePolyline(r.GetString(14)),
        };

        private static string? SerializePolyline(List<Point3D>? pts)
        {
            if (pts == null || pts.Count == 0) return null;
            var arr = pts.Select(p => new[] { p.X, p.Y, p.Z }).ToArray();
            return JsonSerializer.Serialize(arr);
        }

        private static List<Point3D>? DeserializePolyline(string json)
        {
            try
            {
                var arr = JsonSerializer.Deserialize<double[][]>(json);
                return arr?.Select(p => new Point3D(p[0], p[1], p[2])).ToList();
            }
            catch { return null; }
        }
    }
}

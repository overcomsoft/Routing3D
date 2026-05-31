// 씬 데이터 모델 — scene.txt 의 격자/장애물/작업(렌더 입력)
// =============================================================================
//   경로(path)는 C++ 엔진(routing3d_capi)으로부터 받으므로 여기엔 두지 않는다.
//   장애물/격자/작업만 보관해 박스·셀→월드 변환·유틸리티 색에 쓴다.
// =============================================================================
using System.Collections.Generic;

namespace Routing3D.Viewer.Model
{
    /// <summary>격자 메타(셀 크기/원점/셀 개수). 단위 mm.</summary>
    public sealed class GridMeta
    {
        public double CellMm { get; set; } = 50.0;
        public double Ox { get; set; }
        public double Oy { get; set; }
        public double Oz { get; set; }
        public int Nx { get; set; } = 1;
        public int Ny { get; set; } = 1;
        public int Nz { get; set; } = 1;
    }

    /// <summary>장애물 AABB(mm).</summary>
    public sealed class ObstacleBox
    {
        public string Name { get; set; } = string.Empty;        // NAME (DB 로드 시에만, scene.txt 엔 없음).
        public string DdworksType { get; set; } = string.Empty; // DDWORKS_TYPE (DB 로드 시에만).
        public string OstType { get; set; } = string.Empty;     // OST_TYPE (DB 로드 시에만).
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;

        // 통과(pass-through) 객체 — 공간은 차지하나 경로탐색 시 충돌로 보지 않고 배관이 통과한다.
        //   · OST_Floors / OST_Ceilings (바닥·천장 슬래브)
        //   · OST_StructuralFraming 이면서 DDWORKS_TYPE=BEAM_STRUCTURE (격자보)
        // 비교는 대소문자 무시. 통과 객체는 엔진에 장애물로 넣지 않는다(BuildModel/AddObstacle 참고).
        public bool IsPassThrough
        {
            get
            {
                var ost = (OstType ?? string.Empty).Trim();
                if (string.Equals(ost, "OST_Floors", System.StringComparison.OrdinalIgnoreCase)) return true;
                if (string.Equals(ost, "OST_Ceilings", System.StringComparison.OrdinalIgnoreCase)) return true;
                if (string.Equals(ost, "OST_StructuralFraming", System.StringComparison.OrdinalIgnoreCase) &&
                    string.Equals((DdworksType ?? string.Empty).Trim(), "BEAM_STRUCTURE", System.StringComparison.OrdinalIgnoreCase))
                    return true;
                return false;
            }
        }
    }

    /// <summary>라우팅 작업(start→end, 유틸리티 메타).</summary>
    public sealed class TaskInfo
    {
        public double Sx, Sy, Sz, Gx, Gy, Gz;
        public string? Utility { get; set; }
        public string? Group { get; set; }

        /// <summary>시작 PoC 이름(POC_LIST.name). DB 로드 시에만 채워짐(scene.txt 엔 없음 → null).</summary>
        public string? PocName { get; set; }
        /// <summary>끝 PoC 이름(POC_LIST.endPocs[].endName). DB 로드 시에만.</summary>
        public string? EndName { get; set; }

        /// <summary>유틸리티 라벨 "[그룹] 유틸"(None/빈 → ?). Python utility_label 과 동일.</summary>
        public string UtilityLabel =>
            $"[{(string.IsNullOrEmpty(Group) ? "?" : Group)}] {(string.IsNullOrEmpty(Utility) ? "?" : Utility)}";
    }

    /// <summary>공간 영역(TB_BIM_SPACE_INFO) — 층/구역(CR, A/F, CSF 등) AABB(mm) + 이름.</summary>
    public sealed class SpaceArea
    {
        public string Name { get; set; } = string.Empty;   // LEVEL_NAME (예: "CR", "A/F", "CSF").
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
    }

    /// <summary>장비(TB_BIM_EQUIPMENT) — AABB(mm) + 이름 + 메인 여부.</summary>
    public sealed class EquipmentBox
    {
        public string Name { get; set; } = string.Empty;   // NAME.
        public bool IsMain { get; set; }                    // IS_MAIN.
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
    }

    /// <summary>덕트/레터럴(TB_BIM_DUCT_LATERAL) — AABB(mm) + 카테고리(DUCT/LATERAL) + 유틸리티.</summary>
    public sealed class DuctLateral
    {
        public string Name { get; set; } = string.Empty;     // NAME.
        public string Category { get; set; } = string.Empty; // CATEGORY: "DUCT" | "LATERAL".
        public string? Utility { get; set; }                  // UTILITY (N/A 가능).
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;

        public bool IsLateral => string.Equals(Category, "LATERAL", System.StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>기존 설계배관 한 줄(TB_ROUTE_PATH 폴리라인) — DB 로드 시에만. 좌표는 월드 mm.</summary>
    public sealed class ExistingPipe
    {
        public List<Pt3> Points { get; } = new();   // PoC→종단 폴리라인(월드 mm, 순서대로).
        public string? Utility { get; set; }          // TB_ROUTE_PATH.SOURCE_UTILITY.
        public string? Group { get; set; }            // TB_ROUTE_PATH.UTILITY_GROUP.

        /// <summary>유틸리티 라벨 "[그룹] 유틸" — TaskInfo.UtilityLabel 과 동일 규약(색 일치용).</summary>
        public string Label =>
            $"[{(string.IsNullOrEmpty(Group) ? "?" : Group)}] {(string.IsNullOrEmpty(Utility) ? "?" : Utility)}";
    }

    /// <summary>3D 점(월드 mm) — Model 레이어가 WPF 의존 없이 좌표를 담는 경량 구조체.</summary>
    public struct Pt3
    {
        public double X, Y, Z;
        public Pt3(double x, double y, double z) { X = x; Y = y; Z = z; }
    }

    /// <summary>scene.txt 한 개의 렌더 입력(격자/장애물/작업 + 원문).</summary>
    public sealed class SceneData
    {
        public GridMeta Grid { get; set; } = new();
        public List<ObstacleBox> Obstacles { get; } = new();
        public List<TaskInfo> Tasks { get; } = new();
        public List<SpaceArea> Spaces { get; } = new();   // 공간 영역(시각화용). DB 로드 시에만 채워짐.
        public List<EquipmentBox> Equipment { get; } = new();   // 장비 박스(시각화용). DB 로드 시에만.
        public List<DuctLateral> DuctsLaterals { get; } = new();   // 덕트/레터럴 박스(시각화용). DB 로드 시에만.
        public List<ExistingPipe> ExistingPipes { get; } = new();   // 기존 설계배관 폴리라인(시각화용). DB 로드 시에만.
        public string RawText { get; set; } = string.Empty;
    }
}

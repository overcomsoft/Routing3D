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

        // DDW_AI_DB 의 TB_BIM_OBSTACLE.COLLISION_PASS(0/1) 직접값. null=미지정(휴리스틱 사용).
        //   1 → 통과객체(배관이 통과), 0 → 충돌. AUTOROUTINGV7 엔 없으므로 그땐 null → OST 휴리스틱.
        public bool? PassThroughOverride { get; set; }

        // 통과(pass-through) 객체 — 공간은 차지하나 경로탐색 시 충돌로 보지 않고 배관이 통과한다.
        //   DDW_AI_DB: COLLISION_PASS 컬럼(PassThroughOverride) 우선. 없으면(AUTOROUTINGV7/scene.txt) OST 휴리스틱:
        //   · OST_Floors / OST_Ceilings (바닥·천장 슬래브)
        //   · OST_StructuralFraming 이면서 DDWORKS_TYPE=BEAM_STRUCTURE (격자보)
        // 비교는 대소문자 무시. 통과 객체는 엔진에 장애물로 넣지 않는다(BuildModel/AddObstacle 참고).
        public bool IsPassThrough
        {
            get
            {
                if (PassThroughOverride.HasValue) return PassThroughOverride.Value;
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

        /// <summary>이 작업이 유래한 기존배관(TB_ROUTE_PATH.ROUTE_PATH_GUID). 세그먼트 상세 조회 키.
        /// DB 로드 시에만 채워짐(scene.txt 엔 없음 → null).</summary>
        public string? RoutePathGuid { get; set; }

        /// <summary>시작 PoC 이름(POC_LIST.name). DB 로드 시에만 채워짐(scene.txt 엔 없음 → null).</summary>
        public string? PocName { get; set; }
        /// <summary>끝 PoC 이름(POC_LIST.endPocs[].endName). DB 로드 시에만.</summary>
        public string? EndName { get; set; }

        /// <summary>유틸리티 라벨 "[그룹] 유틸"(None/빈 → ?). Python utility_label 과 동일.</summary>
        public string UtilityLabel =>
            $"[{(string.IsNullOrEmpty(Group) ? "?" : Group)}] {(string.IsNullOrEmpty(Utility) ? "?" : Utility)}";
    }

    /// <summary>기존배관 한 줄의 세그먼트 상세(TB_ROUTE_SEGMENT_DETAIL) 한 행 — 하단 '세그먼트 상세' 탭 표시용.
    /// (s.ORDER, sd.ORDER) 정렬 순서의 1-based 일련번호 Seq + 종류/관경/FROM·TO 좌표/연결 소유타입.</summary>
    public sealed class SegmentDetailRow
    {
        public int Seq { get; init; }                      // 정렬 순서(1-based 일련번호).
        public string Type { get; init; } = string.Empty;  // TYPE (POC/PIPE/ELBOW/…).
        public string Size { get; init; } = string.Empty;  // SIZE (호칭경).

        // Owner(소유 객체) 정보 — POC 세그먼트는 그 PoC 의 OWNER_INSTANCE_TYPE(DUCT/MODEL/MAIN_EQUIPMENT…),
        // 시작/종단 POC 는 라우트 헤더의 실제 owner 이름(EQUIPMENT_NAME / TARGET_OWNER_NAME)을 덧붙인다.
        // PIPE/ELBOW 등 PoC 가 아닌 세그먼트는 "-". 후처리로 채우므로 set 허용.
        public string Owner { get; set; } = "-";

        // 수치 좌표(mm) — 행 클릭 시 3D 강조·카메라 이동용(표시는 안 함). 유효 플래그로 NULL 좌표 구분.
        public double Fx { get; init; }
        public double Fy { get; init; }
        public double Fz { get; init; }
        public double Tx { get; init; }
        public double Ty { get; init; }
        public double Tz { get; init; }
        public bool FromValid { get; init; }
        public bool ToValid { get; init; }
        public bool HasPos => FromValid || ToValid;
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
        public string? RoutePathGuid { get; set; }    // TB_ROUTE_PATH.ROUTE_PATH_GUID — 번들 그룹 member_guids 매칭 키.
        public string? Utility { get; set; }          // TB_ROUTE_PATH.SOURCE_UTILITY.
        public string? Group { get; set; }            // TB_ROUTE_PATH.UTILITY_GROUP.
        public double DiameterMm { get; set; }        // 대표 관경(mm). 0 이면 미상 → 렌더에서 기본값.

        // 종단 PoC 좌표(월드 mm) — TB_ROUTE_PATH.SOURCE_POS / TARGET_POS. 선택 배관(Task)의 시작/끝
        // PoC 좌표와 기하학적으로 짝지어 '이 배관에 해당하는 기존 설계경로'를 찾는 매칭 키로 쓴다.
        // 로더가 폴리라인 절단(TrimToBoundary)에 쓰던 curStart/curEnd 와 동일 값. null 이면 폴백(폴리라인 끝점).
        public Pt3? SourcePos { get; set; }
        public Pt3? TargetPos { get; set; }

        /// <summary>유틸리티 라벨 "[그룹] 유틸" — TaskInfo.UtilityLabel 과 동일 규약(색 일치용).</summary>
        public string Label =>
            $"[{(string.IsNullOrEmpty(Group) ? "?" : Group)}] {(string.IsNullOrEmpty(Utility) ? "?" : Utility)}";
    }

    /// <summary>배관 자재(연결부) 1개 — TB_ROUTE_SEGMENT_DETAIL 의 실제 부속(ELBOW/TEE/VALVE/FLANGE 등).
    /// 위치=세그먼트디테일 FROM/TO 중점(월드 mm). TYPE=부속 분류(PIPE/POC/BENDING 은 제외).</summary>
    public sealed class PipeFitting
    {
        public string Type { get; set; } = string.Empty;   // TB_ROUTE_SEGMENT_DETAIL.TYPE (ELBOW, TEE, VALVE...).
        public string? Size { get; set; }                   // SIZE (호칭경 원문, 예 "40A").
        public double X, Y, Z;                              // 부속 중심(월드 mm).
        public string? Utility { get; set; }                // 상위 route 의 SOURCE_UTILITY(옵션).
        public double DiameterMm { get; set; }              // SIZE → 외경 근사(mm). 0=미상.
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
        public List<PipeFitting> Fittings { get; } = new();         // 배관 자재(연결부, TB_ROUTE_SEGMENT_DETAIL). DB 로드 시에만.
        public string SourceFile { get; set; } = string.Empty;      // 프로젝트 SOURCE_FILE(DB 로드 시). 번들 템플릿 조회 키.
        public string RawText { get; set; } = string.Empty;
    }
}

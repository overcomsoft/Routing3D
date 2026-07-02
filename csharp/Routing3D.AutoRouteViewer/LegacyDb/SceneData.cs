// ???곗씠??紐⑤뜽 ??scene.txt ??寃⑹옄/?μ븷臾??묒뾽(?뚮뜑 ?낅젰)
// =============================================================================
//   寃쎈줈(path)??C++ ?붿쭊(routing3d_capi)?쇰줈遺??諛쏆쑝誘濡??ш린???먯? ?딅뒗??
//   ?μ븷臾?寃⑹옄/?묒뾽留?蹂닿???諛뺤뒪쨌??믪썡??蹂?샕룹쑀?몃━???됱뿉 ?대떎.
// =============================================================================
using System.Collections.Generic;

namespace Routing3D.AutoRouteViewer.LegacyDb
{
    /// <summary>寃⑹옄 硫뷀?(? ?ш린/?먯젏/? 媛쒖닔). ?⑥쐞 mm.</summary>
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

    /// <summary>?μ븷臾?AABB(mm).</summary>
    public sealed class ObstacleBox
    {
        public string Name { get; set; } = string.Empty;        // NAME (DB 濡쒕뱶 ?쒖뿉留? scene.txt ???놁쓬).
        public string DdworksType { get; set; } = string.Empty; // DDWORKS_TYPE (DB 濡쒕뱶 ?쒖뿉留?.
        public string OstType { get; set; } = string.Empty;     // OST_TYPE (DB 濡쒕뱶 ?쒖뿉留?.
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;

        // DDW_AI_DB ??TB_BIM_OBSTACLE.COLLISION_PASS(0/1) 吏곸젒媛? null=誘몄????대━?ㅽ떛 ?ъ슜).
        //   1 ???듦낵媛앹껜(諛곌????듦낵), 0 ??異⑸룎. AUTOROUTINGV7 ???놁쑝誘濡?洹몃븧 null ??OST ?대━?ㅽ떛.
        public bool? PassThroughOverride { get; set; }

        // ?듦낵(pass-through) 媛앹껜 ??怨듦컙? 李⑥??섎굹 寃쎈줈?먯깋 ??異⑸룎濡?蹂댁? ?딄퀬 諛곌????듦낵?쒕떎.
        //   DDW_AI_DB: COLLISION_PASS 而щ읆(PassThroughOverride) ?곗꽑. ?놁쑝硫?AUTOROUTINGV7/scene.txt) OST ?대━?ㅽ떛:
        //   쨌 OST_Floors / OST_Ceilings (諛붾떏쨌泥쒖옣 ?щ옒釉?
        //   쨌 OST_StructuralFraming ?대㈃??DDWORKS_TYPE=BEAM_STRUCTURE (寃⑹옄蹂?
        // 鍮꾧탳????뚮Ц??臾댁떆. ?듦낵 媛앹껜???붿쭊???μ븷臾쇰줈 ?ｌ? ?딅뒗??BuildModel/AddObstacle 李멸퀬).
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

    /// <summary>?쇱슦???묒뾽(start?뭙nd, ?좏떥由ы떚 硫뷀?).</summary>
    public sealed class TaskInfo
    {
        public double Sx, Sy, Sz, Gx, Gy, Gz;
        public string? Utility { get; set; }
        public string? Group { get; set; }

        /// <summary>???묒뾽??愿寃??멸꼍, mm). ?묒뾽???좊옒??TB_ROUTE_PATH.SOURCE_SIZE ?먯꽌 吏곸젒 梨꾩슫??
        /// 留ㅼ묶 湲곗〈諛곌? ?먯깋???섏〈?섏? ?딄퀬 ?묒뾽 蹂몄씤??洹쒓꺽???곕?濡? 誘몃ℓ移??묒뾽???ㅺ?寃쎌쑝濡??뚮뜑?쒕떎
        /// (0 ?대㈃ ?뚮뜑媛 寃⑹옄 湲곕컲 ?대갚 ??媛???쒕툕濡?洹몃젮???댁썐怨?援듦린 遺덉씪移섑븯??'愿寃쎌씠?????먯씤).</summary>
        public double DiameterMm { get; set; }

        /// <summary>???묒뾽???좊옒??湲곗〈諛곌?(TB_ROUTE_PATH.ROUTE_PATH_GUID). ?멸렇癒쇳듃 ?곸꽭 議고쉶 ??
        /// DB 濡쒕뱶 ?쒖뿉留?梨꾩썙吏?scene.txt ???놁쓬 ??null).</summary>
        public string? RoutePathGuid { get; set; }

        /// <summary>?쒖옉 PoC ?대쫫(POC_LIST.name). DB 濡쒕뱶 ?쒖뿉留?梨꾩썙吏?scene.txt ???놁쓬 ??null).</summary>
        public string? PocName { get; set; }
        /// <summary>??PoC ?대쫫(POC_LIST.endPocs[].endName). DB 濡쒕뱶 ?쒖뿉留?</summary>
        public string? EndName { get; set; }

        /// <summary>?좏떥由ы떚 ?쇰꺼 "[洹몃９] ?좏떥"(None/鍮????). Python utility_label 怨??숈씪.</summary>
        public string UtilityLabel =>
            $"[{(string.IsNullOrEmpty(Group) ? "?" : Group)}] {(string.IsNullOrEmpty(Utility) ? "?" : Utility)}";
    }

    /// <summary>湲곗〈諛곌? ??以꾩쓽 ?멸렇癒쇳듃 ?곸꽭(TB_ROUTE_SEGMENT_DETAIL) ???????섎떒 '?멸렇癒쇳듃 ?곸꽭' ???쒖떆??
    /// (s.ORDER, sd.ORDER) ?뺣젹 ?쒖꽌??1-based ?쇰젴踰덊샇 Seq + 醫낅쪟/愿寃?FROM쨌TO 醫뚰몴/?곌껐 ?뚯쑀???</summary>
    public sealed class SegmentDetailRow
    {
        public int Seq { get; init; }                      // ?뺣젹 ?쒖꽌(1-based ?쇰젴踰덊샇).
        public string Type { get; init; } = string.Empty;  // TYPE (POC/PIPE/ELBOW/??.
        public string Size { get; init; } = string.Empty;  // SIZE (?몄묶寃?.

        // Owner(?뚯쑀 媛앹껜) ?뺣낫 ??POC ?멸렇癒쇳듃??洹?PoC ??OWNER_INSTANCE_TYPE(DUCT/MODEL/MAIN_EQUIPMENT??,
        // ?쒖옉/醫낅떒 POC ???쇱슦???ㅻ뜑???ㅼ젣 owner ?대쫫(EQUIPMENT_NAME / TARGET_OWNER_NAME)???㏓텤?몃떎.
        // PIPE/ELBOW ??PoC 媛 ?꾨땶 ?멸렇癒쇳듃??"-". ?꾩쿂由щ줈 梨꾩슦誘濡?set ?덉슜.
        public string Owner { get; set; } = "-";

        // ?섏튂 醫뚰몴(mm) ?????대┃ ??3D 媛뺤“쨌移대찓???대룞???쒖떆??????. ?좏슚 ?뚮옒洹몃줈 NULL 醫뚰몴 援щ텇.
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

    /// <summary>怨듦컙 ?곸뿭(TB_BIM_SPACE_INFO) ??痢?援ъ뿭(CR, A/F, CSF ?? AABB(mm) + ?대쫫.</summary>
    public sealed class SpaceArea
    {
        public string Name { get; set; } = string.Empty;   // LEVEL_NAME (?? "CR", "A/F", "CSF").
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
    }

    /// <summary>?λ퉬(TB_BIM_EQUIPMENT) ??AABB(mm) + ?대쫫 + 硫붿씤 ?щ?.</summary>
    public sealed class EquipmentBox
    {
        public string Name { get; set; } = string.Empty;   // NAME.
        public bool IsMain { get; set; }                    // IS_MAIN.
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
    }

    /// <summary>?뺥듃/?덊꽣??TB_BIM_DUCT_LATERAL) ??AABB(mm) + 移댄뀒怨좊━(DUCT/LATERAL) + ?좏떥由ы떚.</summary>
    public sealed class DuctLateral
    {
        public string Name { get; set; } = string.Empty;     // NAME.
        public string Category { get; set; } = string.Empty; // CATEGORY: "DUCT" | "LATERAL".
        public string? Utility { get; set; }                  // UTILITY (N/A 媛??.
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;

        public bool IsLateral => string.Equals(Category, "LATERAL", System.StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>湲곗〈 ?ㅺ퀎諛곌? ??以?TB_ROUTE_PATH ?대━?쇱씤) ??DB 濡쒕뱶 ?쒖뿉留? 醫뚰몴???붾뱶 mm.</summary>
    public sealed class ExistingPipe
    {
        public List<Pt3> Points { get; } = new();   // PoC?믪쥌???대━?쇱씤(?붾뱶 mm, ?쒖꽌?濡?.
        public string? RoutePathGuid { get; set; }    // TB_ROUTE_PATH.ROUTE_PATH_GUID ??踰덈뱾 洹몃９ member_guids 留ㅼ묶 ??
        public string? Utility { get; set; }          // TB_ROUTE_PATH.SOURCE_UTILITY.
        public string? Group { get; set; }            // TB_ROUTE_PATH.UTILITY_GROUP.
        public double DiameterMm { get; set; }        // ???愿寃?mm). 0 ?대㈃ 誘몄긽 ???뚮뜑?먯꽌 湲곕낯媛?

        // 醫낅떒 PoC 醫뚰몴(?붾뱶 mm) ??TB_ROUTE_PATH.SOURCE_POS / TARGET_POS. ?좏깮 諛곌?(Task)???쒖옉/??
        // PoC 醫뚰몴? 湲고븯?숈쟻?쇰줈 吏앹???'??諛곌????대떦?섎뒗 湲곗〈 ?ㅺ퀎寃쎈줈'瑜?李얜뒗 留ㅼ묶 ?ㅻ줈 ?대떎.
        // 濡쒕뜑媛 ?대━?쇱씤 ?덈떒(TrimToBoundary)???곕뜕 curStart/curEnd ? ?숈씪 媛? null ?대㈃ ?대갚(?대━?쇱씤 ?앹젏).
        public Pt3? SourcePos { get; set; }
        public Pt3? TargetPos { get; set; }

        /// <summary>?좏떥由ы떚 ?쇰꺼 "[洹몃９] ?좏떥" ??TaskInfo.UtilityLabel 怨??숈씪 洹쒖빟(???쇱튂??.</summary>
        public string Label =>
            $"[{(string.IsNullOrEmpty(Group) ? "?" : Group)}] {(string.IsNullOrEmpty(Utility) ? "?" : Utility)}";
    }

    /// <summary>諛곌? ?먯옱(?곌껐遺) 1媛???TB_ROUTE_SEGMENT_DETAIL ???ㅼ젣 遺??ELBOW/TEE/VALVE/FLANGE ??.
    /// ?꾩튂=?멸렇癒쇳듃?뷀뀒??FROM/TO 以묒젏(?붾뱶 mm). TYPE=遺??遺꾨쪟(PIPE/POC/BENDING ? ?쒖쇅).</summary>
    public sealed class PipeFitting
    {
        public string Type { get; set; } = string.Empty;   // TB_ROUTE_SEGMENT_DETAIL.TYPE (ELBOW, TEE, VALVE...).
        public string? Size { get; set; }                   // SIZE (?몄묶寃??먮Ц, ??"40A").
        public double X, Y, Z;                              // 遺??以묒떖(?붾뱶 mm).
        public string? Utility { get; set; }                // ?곸쐞 route ??SOURCE_UTILITY(?듭뀡).
        public double DiameterMm { get; set; }              // SIZE ???멸꼍 洹쇱궗(mm). 0=誘몄긽.
    }

    /// <summary>3D ???붾뱶 mm) ??Model ?덉씠?닿? WPF ?섏〈 ?놁씠 醫뚰몴瑜??대뒗 寃쎈웾 援ъ“泥?</summary>
    public struct Pt3
    {
        public double X, Y, Z;
        public Pt3(double x, double y, double z) { X = x; Y = y; Z = z; }
    }


    public enum PocOwnerKind
    {
        Unknown,
        Equipment,
        Duct,
        Lateral
    }

    public sealed class PocMarker
    {
        public PocOwnerKind Kind { get; set; } = PocOwnerKind.Unknown;
        public string Name { get; set; } = string.Empty;
        public string OwnerName { get; set; } = string.Empty;
        public string? OwnerId { get; set; }
        public string? Utility { get; set; }
        public string? Group { get; set; }
        public double X, Y, Z;
        public bool IsRouteStart { get; set; }
        public bool IsRouteEnd { get; set; }
        public string? RoutePathGuid { get; set; }
    }
    /// <summary>scene.txt ??媛쒖쓽 ?뚮뜑 ?낅젰(寃⑹옄/?μ븷臾??묒뾽 + ?먮Ц).</summary>
    public sealed class SceneData
    {
        public GridMeta Grid { get; set; } = new();
        public List<ObstacleBox> Obstacles { get; } = new();
        public List<TaskInfo> Tasks { get; } = new();
        public List<SpaceArea> Spaces { get; } = new();   // 怨듦컙 ?곸뿭(?쒓컖?붿슜). DB 濡쒕뱶 ?쒖뿉留?梨꾩썙吏?
        public List<EquipmentBox> Equipment { get; } = new();   // ?λ퉬 諛뺤뒪(?쒓컖?붿슜). DB 濡쒕뱶 ?쒖뿉留?
        public List<DuctLateral> DuctsLaterals { get; } = new();   // ?뺥듃/?덊꽣??諛뺤뒪(?쒓컖?붿슜). DB 濡쒕뱶 ?쒖뿉留?
        public List<PocMarker> EquipmentPocs { get; } = new();
        public List<PocMarker> DuctLateralPocs { get; } = new();
        public List<ExistingPipe> ExistingPipes { get; } = new();   // 湲곗〈 ?ㅺ퀎諛곌? ?대━?쇱씤(?쒓컖?붿슜). DB 濡쒕뱶 ?쒖뿉留?
        public List<PipeFitting> Fittings { get; } = new();         // 諛곌? ?먯옱(?곌껐遺, TB_ROUTE_SEGMENT_DETAIL). DB 濡쒕뱶 ?쒖뿉留?
        public string SourceFile { get; set; } = string.Empty;      // ?꾨줈?앺듃 SOURCE_FILE(DB 濡쒕뱶 ??. 踰덈뱾 ?쒗뵆由?議고쉶 ??
        public string RawText { get; set; } = string.Empty;
    }
}

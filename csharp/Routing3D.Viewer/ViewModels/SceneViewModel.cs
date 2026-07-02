// 뷰모델 — C++ 엔진(routing3d_capi) 라우팅 + HelixToolkit 3D + 인터랙티브 재라우팅(P2) + 충돌/토글/3D피킹(P3)
// =============================================================================
// [이 파일이 하는 일]
//   scene.txt(또는 내장 데모)를 읽어 격자/장애물/작업을 파싱하고, 같은 장면을 C++ 엔진에
//   적재해 라우팅한 뒤(엔진=C++, 뷰어=C#), 결과 경로를 받아 HelixToolkit Model3DGroup
//   (장애물 반투명 박스 + 유틸리티별 경로 튜브 + 충돌 셀 큐브)으로 만든다.
//   P2: 작업 선택 → 종단점 편집 → 단일/전체 재라우팅.
//   P3: 표시 토글(장애물/경로/충돌), 충돌 셀 시각화, 3D 클릭으로 종단점 지정.
// =============================================================================
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;
using System.Windows.Threading;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using Routing3D.Viewer.Interop;
using Routing3D.Viewer.Model;
using Routing3D.Viewer.Views;

namespace Routing3D.Viewer.ViewModels
{
    /// <summary>3D 클릭 피킹 모드.</summary>
    public enum PickMode { None, Start, End }

    /// <summary>경로 탐색 범위 — 전체 한 번에 / 유틸리티그룹 1개 / 유틸리티 1개.</summary>
    public enum RouteScope { All, ByGroup, ByUtility }

    /// <summary>자동설계 경로 탐색 방식.
    /// Shortest=최단 순수 A* / PatternApplied=특징점(스텁+패턴) 반영 / FollowExisting=기존설계 폴리라인 복제.</summary>
    public enum RoutingMode { Shortest, PatternApplied, FollowExisting }

    /// <summary>범위 콤보 항목(라벨 + enum).</summary>
    public sealed class RouteScopeOption
    {
        public RouteScope Scope { get; init; }
        public string Label { get; init; } = string.Empty;
        public override string ToString() => Label;
    }

    /// <summary>범례 항목(색 견본 + 라벨).</summary>
    public sealed class LegendItem
    {
        public Brush Swatch { get; init; } = Brushes.Gray;
        public string Label { get; init; } = string.Empty;
    }

    /// <summary>3D 공간 영역 텍스트 라벨(코드비하인드가 BillboardText 로 렌더). 위치는 월드 mm.</summary>
    public sealed class SpaceLabel
    {
        public string Text { get; init; } = string.Empty;
        public Point3D Position { get; init; }
        public Color Color { get; init; } = Colors.White;
    }

    /// <summary>선택 경로의 한 직선 구간(단계) — 방향/길이 라벨 + 시작 월드좌표(클릭 시 이동 대상).</summary>
    public sealed class PathStep
    {
        public string Label { get; init; } = string.Empty;
        public Point3D Position { get; init; }
        public Point3D A { get; init; }   // 직선 구간 시작 월드좌표(구간 강조용).
        public Point3D B { get; init; }   // 직선 구간 끝 월드좌표.
        public override string ToString() => Label;
    }

    /// <summary>'단계별 경로' 탭의 한 행 — 시작/구간/종단을 그리드로 표시(시작 → 꺾임(사유) → 종단).
    /// Kind=시작·종단이면 Name/Coord 가, Kind=구간이면 Direction/Length/Region/Reason 이 채워진다.</summary>
    public sealed class RouteStepRow
    {
        public string Seq { get; init; } = string.Empty;        // "시작" / "1" / "2" … / "종단"
        public string Kind { get; init; } = string.Empty;       // 시작 / 구간 / 종단
        public string Direction { get; init; } = string.Empty;  // 수직 ↑(Z+) 등
        public string Length { get; init; } = string.Empty;     // "1,200 mm"
        public string Region { get; init; } = string.Empty;     // 학습 스텁/엔진 탐색/복제 경로 (또는 시작·종단 좌표)
        public string Reason { get; init; } = string.Empty;     // 꺾임 사유(구간 끝 꺾임) — 시작/종단/마지막 구간은 빈칸
        // 3D 강조용 월드 좌표(구간=A→B 직선, 시작/종단=A 한 점). 행 클릭 시 자동경로에서 이 구간을 강조한다.
        public Point3D A { get; init; }
        public Point3D B { get; init; }
    }

    public sealed class SceneViewModel : ObservableObject
    {
        private Engine? _engine;
        private SceneData? _scene;
        private Dictionary<string, FeatureProfileRow> _featureProfiles = new(StringComparer.OrdinalIgnoreCase);
        // 굵은 배관이 최단 경로를 먼저 선점해 가는 배관이 우회·충돌하지 않도록 diameter 정렬.
        private readonly string _priority = "diameter";

        // ── 3D 객체 클릭 → 속성 정보 표시 ───────────────────────────────
        // 씬은 원본 mm 좌표로 렌더되므로(ApplyPick 이 픽 점을 그대로 mm 로 사용),
        // 클릭 지점을 포함하는 객체를 AABB 포함 검사로 찾는다. 표시 중(레이어 켜짐)인
        // 객체만 후보로 삼고, 겹치면 부피가 가장 작은(가장 구체적인) 객체를 고른다.
        private string? _selectedObjectInfo;
        public string? SelectedObjectInfo
        {
            get => _selectedObjectInfo;
            private set => Set(ref _selectedObjectInfo, value);
        }

        public void SelectObjectAt(Point3D p)
        {
            var s = _scene;
            if (s is null) { SelectedObjectInfo = null; return; }

            string? best = null; double bestVol = double.MaxValue;
            Point3D blo = default, bhi = default;
            void Consider(string text, double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            {
                const double eps = 1.0;
                if (p.X < mnx - eps || p.X > mxx + eps) return;
                if (p.Y < mny - eps || p.Y > mxy + eps) return;
                if (p.Z < mnz - eps || p.Z > mxz + eps) return;
                double vol = Math.Max(1, mxx - mnx) * Math.Max(1, mxy - mny) * Math.Max(1, mxz - mnz);
                if (vol < bestVol) { bestVol = vol; best = text; blo = new Point3D(mnx, mny, mnz); bhi = new Point3D(mxx, mxy, mxz); }
            }

            if (ShowEquipment)
                foreach (var e in s.Equipment)
                    Consider(DescribeEquipment(e), e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);

            foreach (var d in s.DuctsLaterals)
            {
                if (d.IsLateral ? !ShowLaterals : !ShowDucts) continue;
                Consider(DescribeDuct(d), d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
            }

            if (ShowObstacles)
                for (int i = 0; i < s.Obstacles.Count; i++)
                {
                    var o = s.Obstacles[i];
                    Consider(DescribeObstacle(i, o), o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                }

            // 공간영역(A/F·CSF·CR 등)은 씬 전체를 덮는 거대 AABB 라 클릭 선택 대상에서 제외한다.
            // (와이어프레임·라벨 렌더는 ShowSpaces 로 그대로 유지 — 표시는 하되 클릭으로는 잡히지 않음.)

            // 자동 라우팅 배관 클릭 — 중심선 최근접 거리 기반(AABB 보다 정밀). ShowPaths 켜진 성공 배관만.
            // 클릭 지점이 관경 반경 이내이고 가장 가까운 배관을 찾아 우측 리스트에서 선택한다.
            if (ShowPaths && Tasks.Count > 0)
            {
                var grid = s.Grid;
                TaskRowVM? bestPipe = null; double bestPipeDist = double.MaxValue;
                foreach (var row in Tasks)
                {
                    if (!row.Success || row.Path.Length < 2) continue;
                    double threshold = Math.Max(row.DiameterMm > 0 ? row.DiameterMm : grid.CellMm * 2, grid.CellMm * 2);
                    var pts = GetRoutedPolyline(row, grid);
                    for (int i = 1; i < pts.Count; i++)
                    {
                        double d = DistPointToSeg3D(p, pts[i - 1], pts[i]);
                        if (d < threshold && d < bestPipeDist) { bestPipeDist = d; bestPipe = row; }
                    }
                }
                if (bestPipe != null)
                {
                    SelectedTask = bestPipe;
                    SelectedObjectInfo = null;
                    HighlightModel = null;
                    Status = $"자동경로 배관 #{bestPipe.Index} ({bestPipe.Utility}) 선택됨";
                    return;
                }
            }

            SelectedObjectInfo = best;
            if (best is null) { HighlightModel = null; Status = "선택된 객체 없음(빈 공간 클릭)"; }
            else { ShowHighlight(blo, bhi); Status = "객체를 선택했습니다."; }
        }

        // 자동 라우팅 배관의 렌더 폴리라인(스텁 + A* 경로 + 종단스텁)을 월드 mm 좌표로 반환.
        // BuildModel 의 pts 빌드 로직과 동일 — 클릭 감지(SelectObjectAt)에서 재사용.
        private List<Point3D> GetRoutedPolyline(TaskRowVM row, GridMeta grid)
        {
            if (row.LoadedPolylinePts != null && row.LoadedPolylinePts.Count >= 2)
                return row.LoadedPolylinePts.ToList();

            var pts = new List<Point3D>();
            if (row.StartStub != null)
                pts.AddRange(row.StartStub.Select(p2 => new Point3D(p2.X, p2.Y, p2.Z)));
            pts.AddRange(row.Path.Select(c => CellToWorld(grid, c)));
            if (row.EndStub != null)
                for (int k = row.EndStub.Count - 1; k >= 0; k--)
                    pts.Add(new Point3D(row.EndStub[k].X, row.EndStub[k].Y, row.EndStub[k].Z));
            return pts;
        }

        // 점 p 에서 선분 a-b 까지의 최단 거리(3D 유클리드).
        private static double DistPointToSeg3D(Point3D p, Point3D a, Point3D b)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
            double len2 = dx * dx + dy * dy + dz * dz;
            if (len2 < 1e-10) { dx = p.X - a.X; dy = p.Y - a.Y; dz = p.Z - a.Z; return Math.Sqrt(dx * dx + dy * dy + dz * dz); }
            double t = Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy + (p.Z - a.Z) * dz) / len2, 0.0, 1.0);
            double ex = p.X - (a.X + t * dx), ey = p.Y - (a.Y + t * dy), ez = p.Z - (a.Z + t * dz);
            return Math.Sqrt(ex * ex + ey * ey + ez * ez);
        }

        // 선택한 객체 AABB 를 밝은 노란 와이어프레임 + 반투명 박스로 강조한다.
        private void ShowHighlight(Point3D lo, Point3D hi)
        {
            var grp = new Model3DGroup();
            double dx = hi.X - lo.X, dy = hi.Y - lo.Y, dz = hi.Z - lo.Z;
            double diag = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            double r = Math.Max(6.0, diag * 0.004);   // 선택선 굵기(얇게).
            AddBoxFrame(grp, lo, hi, Colors.Yellow, r, 255);
            var fill = new MeshBuilder(false, false);
            fill.AddBox(new Point3D((lo.X + hi.X) / 2, (lo.Y + hi.Y) / 2, (lo.Z + hi.Z) / 2), dx, dy, dz);
            grp.Children.Add(Geometry(fill, Colors.Yellow, 55));
            HighlightModel = grp;
        }

        private static string F(double v) => v.ToString("F0", System.Globalization.CultureInfo.InvariantCulture);

        private static string Dims(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            => $"크기(mm): {F(mxx - mnx)} × {F(mxy - mny)} × {F(mxz - mnz)}\n"
             + $"중심(mm): ({F((mnx + mxx) / 2)}, {F((mny + mxy) / 2)}, {F((mnz + mxz) / 2)})";

        private static string DescribeEquipment(EquipmentBox e)
            => "[장비]\n"
             + $"이름: {(string.IsNullOrEmpty(e.Name) ? "(이름없음)" : e.Name)}\n"
             + $"메인 장비: {(e.IsMain ? "예" : "아니오")}\n"
             + Dims(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);

        private static string DescribeDuct(DuctLateral d)
            => $"[{(d.IsLateral ? "레터럴" : "덕트")}]\n"
             + $"이름: {(string.IsNullOrEmpty(d.Name) ? "(이름없음)" : d.Name)}\n"
             + $"CATEGORY: {d.Category}\n"
             + $"UTILITY: {(string.IsNullOrEmpty(d.Utility) ? "N/A" : d.Utility)}\n"
             + Dims(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);

        private static string DescribeObstacle(int i, ObstacleBox o)
            => $"[장애물 #{i}]\n"
             + $"이름: {(string.IsNullOrEmpty(o.Name) ? "(이름없음)" : o.Name)}\n"
             + $"OST_TYPE: {(string.IsNullOrEmpty(o.OstType) ? "N/A" : o.OstType)}\n"
             + $"DDWORKS_TYPE: {(string.IsNullOrEmpty(o.DdworksType) ? "N/A" : o.DdworksType)}\n"
             + $"통과 객체: {(o.IsPassThrough ? "예 (경로탐색 통과)" : "아니오")}\n"
             + Dims(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ) + "\n"
             + $"AABB(mm): ({F(o.MinX)}, {F(o.MinY)}, {F(o.MinZ)})\n"
             + $"        ~ ({F(o.MaxX)}, {F(o.MaxY)}, {F(o.MaxZ)})";

        private static string DescribeSpace(SpaceArea sp)
            => "[공간영역]\n"
             + $"이름: {sp.Name}\n"
             + Dims(sp.MinX, sp.MinY, sp.MinZ, sp.MaxX, sp.MaxY, sp.MaxZ);

        private Model3D? _sceneModel;
        private string _status = string.Empty;
        private TaskRowVM? _selectedTask;
        private PickMode _pickMode = PickMode.None;
        private bool _showObstacles = true;
        private bool _showPaths = true;
        private bool _showCollisions = true;
        private bool _showGridFrame = false;        // 복셀 전체맵(격자 BBOX 와이어).
        private bool _showOccupancyVoxels = false;  // 점유맵(복셀화된 장애물 셀).
        private bool _occupancyFullRes = false;     // 점유맵 해상도: true=원본(전체 셀), false=다운샘플(상한).
        private bool _showVisitedMap = false;       // 방문맵(A* 확장 셀, 유틸리티 색).
        private bool _enableSearchTrace = false;    // C++ 엔진 탐색 trace JSONL 로그 생성.
        private string _lastSearchTraceFile = string.Empty;
        private bool _showOctreeVoxels = false;     // 옥트리 가변셀 분해도(FREE=녹색·BLOCKED=적색).
        private bool _showPassthroughVoxels = false; // 통과 점유맵(바닥/천장/격자보) 별도 토글.
        // 성능 측정 — 3D 뷰 오버레이용
        private int    _renderObjectCount = 0;      // BuildModel 이 추가한 GeometryModel3D 수
        private int    _octreeLeafCount = 0;        // 마지막 AddOctreeVoxels 가 렌더한 리프 수
        private bool _showSpaces = true;            // 공간 영역(CR/A/F/CSF) 와이어프레임 + 텍스트.
        private bool _showEquipment = true;         // 장비(TB_BIM_EQUIPMENT) 큐브 박스.
        private bool _showLaterals = true;          // 레터럴(TB_DUCT_LATERAL, CATEGORY=LATERAL) 박스.
        private bool _showDucts = true;             // 덕트(TB_DUCT_LATERAL, CATEGORY=DUCT) 박스.
        private bool _showExistingPipes = true;     // 기존 설계배관(TB_ROUTE_PATH) 폴리라인(유틸리티 색).
        private bool _showPocMarkers = true;        // 모든 작업의 시작 PoC(빨강)·종단 PoC(파랑) 마커(초기 표시).
        private bool _showStubs = true;             // 기존설계 배관의 출발(빨강)·종단(파랑) 스텁(수직+엘보) 강조.
        private bool _showBundleGroups;             // 그룹배관 강조 — 탐지 번들(route_bundle_group) 멤버를 그룹별 색으로.
        private bool _showBundlePattern;            // 그룹배관 패턴 표시 — 학습된 유틸별 공용 트렁크 레인(L4)을 반투명 큐브로.
        private readonly bool _includeFacilities = true;  // 충돌확장: 설비·덕트·레터럴 + 기설계 배관을 장애물로. 항상 ON 고정(readonly).
        // 기존설계 패턴(pgvector) — 학습된 진출/진입 면으로 시작/종단 PoC 를 투영(L2a). null=미적재(기하 폴백).
        private PatternStore? _patterns;
        private bool _patternsTried;                    // DB 1회만 조회(미스도 캐시).
        private bool _usePatterns = true;               // 기존설계 패턴 활용 토글(기본 ON, 미스 시 자동 기하 폴백).
        // 기존설계 회랑(L2b) — 매칭되는 기존 설계배관(TB_ROUTE_PATH) 폴리라인을 회랑 시드로 주입하고
        // w_corridor 로 회랑 밖을 가산 → 새 경로가 사람 설계를 부드럽게 따라간다(충돌은 여전히 회피).
        // 경로 '모양'을 바꾸므로 기본 OFF(옵트인). ON 시 BuildEngineForRows 가 회랑 셀 주입 + w_corridor>0.
        private bool _useDesignCorridor;
        // 유틸그룹 랙 번들링(L3a) — 기존배관의 수평 런이 모이는 z-높이(랙)를 그룹별로 학습해 엔진
        // rack_levels(= w_corridor 면제 z-셀)로 준다. 같은 그룹 새 배관이 공용 랙 높이에 뭉친다(사람 설계다움).
        // rack_levels 는 w_corridor>0 일 때만 효력(랙 밖 가산) → ON 시 BuildEngineForRows 가 가벼운 w_corridor 부여.
        private bool _useRackBundling;
        // 그룹배관 패턴(L4) — Python bundle_detect 가 DB(route_bundle_template)에 저장한 '대표 그룹배관 패턴'
        // (같은 유틸 배관들이 동일 이격간격·2회+ 꺾임으로 공유하는 공용 트렁크 고도)을 읽어, 신규 라우팅 시
        // 같은 유틸 새 배관을 그 트렁크 고도(rack_levels)에 뭉치게 한다. 미적재/키 미스면 자동 폴백(무해).
        // 기본 ON(항상 적용) — 학습 패턴이 없으면 폴백이라 무해하고, 있으면 사람 설계처럼 공용 랙에 다발링.
        private BundleStore? _bundles;
        private bool _useBundlePattern = true;
        // 스텁 라우팅 — 매칭 기존배관의 출발/종단 스텁(수직+엘보)을 '고정 설계 구간'으로 깔고, A* 는 스텁 끝~끝
        // (랙 위 자유공간)만 탐색한다. 표시 경로 = [출발 스텁] + [A* 중간] + [종단 스텁]. 매칭 배관 없으면 PoC
        // 직접 라우팅으로 폴백. 학습 스텁과 자동설계를 일치시킨다(기존엔 PoC 에서 A* 가 스텁을 무시하고 재탐색).
        private bool _useStubRouting = true;
        // 기존배관 복제(폴리라인) — 매칭되는 기존 설계배관이 있으면 그 폴리라인을 셀 경로로 '복제'하고, 현재
        // 점유에서 막힌(장애물이 달라진) 구간만 A* 로 국소 우회 수리한다. 결과 = 기존설계 그대로 + 변경된 곳만
        // 우회 → 가장 강한 '기존설계 유사'. 매칭 없으면 일반 A* 결과 유지(무해 폴백). 기본 OFF(충돌회피 A* 우선).
        private bool _useDesignReplicate;
        // 경로 탐색 방식(라디오버튼 3-모드 — 값 변경 시 내부 옵션 자동 동기화).
        // PatternApplied = A* 탐색 + pipe_gap 강제 → 충돌 0 보장. FollowExisting = 기존 복제(빠르나 충돌 가능).
        private RoutingMode _routingMode = RoutingMode.PatternApplied;
        private bool _useHierarchicalCorridor = false;  // false=route_multi(가중 A*, 고품질). 엔진 astar_weighted 의 closed 가 해시 기반이 되어 대형 격자(25mm 1.3억 셀)에서도 OOM 없이 동작. true=계층 corridor(이 장면에선 대부분 실패해 비권장).
        // CBS(negotiated-congestion, C1) — 평면 rip-up(직접 blocker 뜯기)을 재귀 연쇄로 확장.
        // blocker 가 재배치 못 하면 그 blocker 의 blocker 까지 bounded depth(≤3) 재귀 양보.
        // 기본 OFF(cbs_depth=0=평면 rip-up 만·골든 불변). ON 시 depth=2 — 잔여 혼잡/막힘 실패 해소.
        private bool _useCbs = false;
        private string _searchText = string.Empty;
        private bool _suppressFilterRebuild;   // BuildTaskRows 중 IsVisible 이벤트 폭주 방지.
        private int[]? _forcedRackZ;           // 강제 Z 랙 고도 필드 (기본 null)

        // 자동설계 보고서 연동
        public event Action? ShowRouteReportRequested;
        private string _routeResultReport = string.Empty;
        private string _routeResultReportPath = string.Empty;

        // DB 접속 설정(환경변수 우선) + 선택된 프로젝트 / 격자 셀 크기.
        private readonly DbConfig _dbConfig = DbConfig.FromEnv();
        private ProjectInfo? _selectedProject;
        private double _cellMm = 25.0;   // 라우팅 격자 셀 크기(mm). 작을수록 정밀하지만 셀 수가 (비율)³ 으로 폭증.
        private bool _suppressProjectAutoLoad;

        // 경로 탐색 범위(모두/그룹별/유틸별) + 선택 대상(그룹/유틸 1개).
        private RouteScopeOption _selectedRouteScope;
        private string? _selectedRouteTarget;

        // 좌측 드릴다운(그룹 → 유틸리티 → 개별 PoC) 선택 상태.
        private string? _selectedGroup;
        private string? _selectedUtility;
        private Model3D? _selectionModel;   // 선택 PoC 강조(시작/끝 마커) 오버레이.
        private Model3D? _searchModel;      // A* 단계별 탐색(방문 셀 점진 표시) 오버레이.
        private Model3D? _highlightModel;   // 3D 클릭으로 선택한 객체 강조(노란 박스) 오버레이.

        // ── 기존설계 비교(선택 배관) ────────────────────────────────────
        // 선택 배관에 매칭된 기존 설계경로(주황)와 개발 경로(시안)를 동시에 그려 정량 비교한다.
        private Model3D? _compareModel;        // 비교 오버레이(기존=주황 / 개발=시안 굵은 튜브).
        private string? _comparisonReport;     // 우측 '기존설계 비교 분석' 패널 텍스트.
        private bool _compareMode;             // 비교 포커스: true 면 나머지 기존배관 레이어 숨김.
        private ExistingPipe? _comparePipe;    // 현재 선택에 매칭된 기존 경로(없으면 null).
        private bool _hidePathsForAnim;     // 단계별 탐색 중 최종 경로를 숨겼다가 끝에 드러내기.
        private bool _animating;            // 단계별 탐색 진행 중(중복 실행 방지).

        public SceneViewModel(string? initialScene = null)
        {
            _forcedRackZ = ParseForcedRackZFromEnv();
            OpenCommand = new RelayCommand(Open);
            DemoCommand = new RelayCommand(LoadDemo);
            RunRouteCommand = new RelayCommand(() => _ = RunRouteAsync(), () => _scene != null);
            RerouteCorridorCommand = new RelayCommand(
                () => _ = RouteRowsAsync(AllRows(), "기존설계 유사(그룹배관)", corridor: true),
                () => _scene != null);
            // 단건 라우팅도 그룹배관 모드(corridor:true) — 학습 공용 트렁크 회랑(L4)+매칭 기존배관(L2b)을 따라
            //   1개 배관도 그룹 트렁크에 정렬된다. (단건은 셀 재적재 없이 = 다른 누적 경로 보존, 셀 가드는 다건만.)
            RerouteSelectedCommand = new RelayCommand(
                () => { if (_selectedTask != null) _ = RouteRowsAsync(new List<int> { _selectedTask.Index }, $"선택 #{_selectedTask.Index}", corridor: true); },
                () => _selectedTask != null);
            CompareSelectedCommand = new RelayCommand(
                () => _ = CompareSelectedAsync(),
                () => _selectedTask != null && _scene != null);
            AnimateSelectedCommand = new RelayCommand(
                () => _ = AnimateSelectedAsync(),
                () => _selectedTask != null && _scene != null && !_animating);
            PickStartCommand = new RelayCommand(() => SetPick(PickMode.Start), () => _selectedTask != null);
            PickEndCommand = new RelayCommand(() => SetPick(PickMode.End), () => _selectedTask != null);
            FitViewCommand = new RelayCommand(() => FitViewRequested?.Invoke());
            ToggleOccupancyResCommand = new RelayCommand(() => OccupancyFullRes = !OccupancyFullRes);
            UtilityAllCommand = new RelayCommand(() => SetAllUtilities(true));
            UtilityClearCommand = new RelayCommand(() => SetAllUtilities(false));
            LoadProjectsCommand = new RelayCommand(() => _ = LoadProjectsAsync());
            LoadDbCommand = new RelayCommand(
                () => { if (_selectedProject != null) _ = LoadFromDbAsync(_selectedProject.ProjectId); },
                () => _selectedProject != null);
            // 좌측 드릴다운 선택을 한꺼번에 라우팅 — 선택 그룹/유틸리티의 모든 PoC를 부분집합 충돌회피 라우팅.
            // 그룹/유틸 전체 라우팅 = 그룹배관 모드(corridor:true) — 같은 유틸을 공용 트렁크에 다발로 묶는다
            //   (스텁+그룹 트렁크 회랑+유틸 순서+강한 w_corridor, 셀>50이면 자동 50mm). 단건이 아니라 '묶음'을
            //   라우팅하므로 기존설계처럼 그룹으로 나오는 게 기대값.
            RouteGroupCommand = new RelayCommand(
                () => { if (!string.IsNullOrEmpty(_selectedGroup))
                            _ = RouteRowsAsync(RowsWhere(t => GroupKey(t.Group) == _selectedGroup),
                                               $"그룹 '{_selectedGroup}'", corridor: true, showProgress: true); },
                () => _scene != null && !string.IsNullOrEmpty(_selectedGroup));
            RouteUtilityCommand = new RelayCommand(
                () => { if (!string.IsNullOrEmpty(_selectedGroup) && !string.IsNullOrEmpty(_selectedUtility))
                            _ = RouteRowsAsync(RowsWhere(t => GroupKey(t.Group) == _selectedGroup &&
                                                              UtilityKey(t.Utility) == _selectedUtility),
                                               $"유틸리티 '{_selectedUtility}'", corridor: true, showProgress: true); },
                () => _scene != null && !string.IsNullOrEmpty(_selectedGroup) && !string.IsNullOrEmpty(_selectedUtility));
            // 자동설계된(라우팅된) 모든 경로 삭제 — 결과 캐시 초기화 + 라이브 오버레이 제거 + 재렌더.
            ClearRoutesCommand = new RelayCommand(ClearAllRoutes,
                () => _scene != null && Tasks.Any(t => t.Success));
            // 라우팅 진행 중에만 동작하는 협력적 취소(✖). 콜백이 _cancelRequested 를 보면 엔진이 멈춘다.
            CancelRoutingCommand = new RelayCommand(RequestCancelRouting, () => _isRouting && !_cancelRequested);
            // '결과 리포트' — 직전 배치 리포트를 분석결과에 다시 띄우고 리포트 창을 연다.
            RouteResultReportCommand = new RelayCommand(
                () => { AnalysisReport = string.IsNullOrEmpty(RouteResultReport) ? AnalysisReport : RouteResultReport; ShowRouteReportRequested?.Invoke(); },
                () => !_isRouting && !string.IsNullOrEmpty(RouteResultReport));
            // GLB 내보내기 — 성공한 자동경로 배관 메시를 glTF 2.0 Binary(.glb)로 저장.
            ExportGlbCommand = new RelayCommand(ExportGlb,
                () => _scene != null && Tasks.Any(t => t.Success));
            // 자동설계 결과 DB 저장 / 불러오기
            SaveRouteResultsCommand = new RelayCommand(
                () => _ = SaveRouteResultsAsync(),
                () => _scene != null && ResultList.Count > 0 && !_isRouting);
            LoadRouteResultsCommand = new RelayCommand(
                () => _ = LoadRouteResultsAsync(),
                () => _scene != null && !_isRouting);

            TasksView = CollectionViewSource.GetDefaultView(Tasks);
            TasksView.Filter = TaskFilter;

            _selectedRouteScope = RouteScopes[0];   // 기본 '모두'.

            try
            {
                if (!string.IsNullOrEmpty(initialScene) && File.Exists(initialScene))
                {
                    // scene.txt 인자 / --selftest 경로: 동기 로드(셀프테스트가 vm.Status 를 즉시 읽음).
                    LoadFile(initialScene);
                }
                else
                {
                    // DB 자동 로드는 무거우므로(목록 조회 + 첫 프로젝트 전체 라우팅) 생성자에서 하지 않는다.
                    // 창이 먼저 뜬 뒤(MainWindow 의 ContentRendered) RunStartupLoadAsync 가 비동기로 수행한다.
                    // 그렇지 않으면 라우팅이 끝날 때까지 창이 아예 보이지 않는다.
                    NeedsStartupLoad = true;
                    Status = "시작 중… 창 표시 후 DB 자동 로드";
                }
            }
            catch (Exception ex) { Status = "엔진 초기화 오류: " + ex.Message; }
        }

        private static int[]? ParseForcedRackZFromEnv()
        {
            var raw = Environment.GetEnvironmentVariable("R3D_FORCED_RACK_Z");
            if (string.IsNullOrWhiteSpace(raw)) return null;

            var values = raw.Split(new[] { ',', ';', ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
                .Select(token => int.TryParse(token, out var z) ? (int?)z : null)
                .Where(z => z.HasValue)
                .Select(z => z!.Value)
                .Distinct()
                .ToArray();
            return values.Length == 0 ? null : values;
        }
        /// <summary>생성자에서 DB 자동 로드를 보류했는지(=scene 인자 없이 실행). 창이 뜬 뒤 코드비하인드가 확인.</summary>
        public bool NeedsStartupLoad { get; private set; }

        /// <summary>창이 처음 렌더된 뒤 호출(코드비하인드). DB 자동 로드를 비동기로 수행하고,
        /// 실패하면 내장 데모로 폴백한다 — 무거운 라우팅 동안 UI 가 멈추거나 빈 화면이 되지 않게.</summary>
        public async Task RunStartupLoadAsync()
        {
            NeedsStartupLoad = false;
            await LoadProjectsAsync();
            if (_scene == null) LoadDemo();   // DB 실패/빈 목록 → 데모 폴백.
        }

        // ---- 바인딩 속성 ----
        public Model3D? SceneModel { get => _sceneModel; private set => Set(ref _sceneModel, value); }
        public string Status { get => _status; private set => Set(ref _status, value); }

        public string RouteResultReport
        {
            get => _routeResultReport;
            private set => Set(ref _routeResultReport, value);
        }

        public string RouteResultReportPath
        {
            get => _routeResultReportPath;
            private set => Set(ref _routeResultReportPath, value);
        }

        // ── 우측 패널: 자동설계 결과 경로 + 라이브 진행 + 분석결과(인라인) ─────────────────
        // (예전엔 RoutingProgressWindow/RouteReportWindow 다이얼로그로 띄웠으나, 우측 패널에 인라인
        //  표시하도록 복원: 상단 결과 리스트 + 진행바, 하단 '분석결과' 탭에 리포트.)
        /// <summary>우측 상단 '자동설계 결과 경로' DataGrid 바인딩. 라우팅 배치마다 대상 행으로 채운다
        /// (같은 TaskRowVM 인스턴스를 참조 → 행 상태/선택/3D 렌더가 Tasks 와 일관).</summary>
        public ObservableCollection<TaskRowVM> ResultList { get; } = new();

        // 라우팅 진행 중 플래그 — 취소 버튼 표시(IsRouting) 및 버튼 활성 갱신용.
        private bool _isRouting;
        public bool IsRouting
        {
            get => _isRouting;
            private set { if (Set(ref _isRouting, value)) System.Windows.Input.CommandManager.InvalidateRequerySuggested(); }
        }

        // CancelRoutingCommand(취소 ✖)가 세우고, 진행 콜백이 true 를 보면 엔진이 현재 배관 탐색을 중단한다.
        private volatile bool _cancelRequested;
        public bool IsCancelRequested => _cancelRequested;
        private void RequestCancelRouting()
        {
            if (!_isRouting || _cancelRequested) return;
            _cancelRequested = true;
            RouteProgressText = "취소 중… (현재 배관 탐색을 멈추는 중)";
            System.Windows.Input.CommandManager.InvalidateRequerySuggested();
        }

        private double _routeProgressValue;   // 0~100 (%).
        public double RouteProgressValue { get => _routeProgressValue; private set => Set(ref _routeProgressValue, value); }
        private string _routeProgressText = string.Empty;
        public string RouteProgressText { get => _routeProgressText; private set => Set(ref _routeProgressText, value); }

        // 우측 하단 '분석결과' 탭 — 라우팅 직후 결과 리포트(또는 집계)를 인라인 표시.
        private string? _analysisReport;
        public string? AnalysisReport { get => _analysisReport; private set => Set(ref _analysisReport, value); }

        /// <summary>라우팅 중이 아닐 때 진행바/텍스트를 결과 집계(성공/실패/미라우팅)로 확정한다.</summary>
        private void RefreshRouteProgress()
        {
            if (_isRouting) return;   // 라우팅 중에는 콜백이 라이브로 갱신.
            var src = ResultList.Count > 0 ? (IList<TaskRowVM>)ResultList : Tasks;
            int total = src.Count, ok = 0, fail = 0;
            double totalMs = 0;
            foreach (var r in src)
            {
                if (r.Success) ok++; else if (r.Attempted) fail++;
                totalMs += r.ElapsedMs;
            }
            int routed = ok + fail;
            RouteProgressValue = total > 0 ? 100.0 * routed / total : 0;
            string timeStr = totalMs > 0
                ? (totalMs < 1000 ? $" · 합계 {totalMs:0}ms" : $" · 합계 {totalMs / 1000.0:0.0}s")
                : "";
            RouteProgressText = total == 0
                ? "결과 없음 — 좌측에서 그룹/유틸리티 라우팅을 실행하세요."
                : $"완료 {routed}/{total} · 성공 {ok} · 실패 {fail}{timeStr}";
        }

        public void ShowStatus(string status)
        {
            Status = status;
        }

        public void SaveRouteReportAs()
        {
            if (string.IsNullOrEmpty(RouteResultReport)) return;
            var dlg = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "Markdown Files (*.md)|*.md|All Files (*.*)|*.*",
                FileName = Path.GetFileName(RouteResultReportPath)
            };
            if (dlg.ShowDialog() == true)
            {
                try
                {
                    File.WriteAllText(dlg.FileName, RouteResultReport, System.Text.Encoding.UTF8);
                    ShowStatus($"보고서를 저장했습니다: {dlg.FileName}");
                }
                catch (Exception ex)
                {
                    ShowStatus($"보고서 저장 실패: {ex.Message}");
                }
            }
        }

        // 성공한 자동경로 배관을 glTF 2.0 Binary(.glb)로 내보낸다.
        // 유틸리티별로 메시를 합쳐 하나의 GLB 파일에 저장. 좌표계: Z-up mm → Y-up m.
        private void ExportGlb()
        {
            var scene = _scene;
            if (scene == null) return;

            var successRows = Tasks.Where(t => t.Success && t.Path.Length >= 2).ToList();
            if (successRows.Count == 0) { ShowStatus("내보낼 자동경로 배관이 없습니다."); return; }

            var dlg = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "glTF Binary (*.glb)|*.glb|All Files (*.*)|*.*",
                FileName = $"autoroute_{DateTime.Now:yyyyMMdd_HHmmss}.glb"
            };
            if (dlg.ShowDialog() != true) return;

            try
            {
                var grid = scene.Grid;
                double fallbackDia = grid.CellMm * 0.7;

                // 유틸리티별 색 배정(BuildModel 과 동일 Assign 호출 = 같은 색)
                var colorMap = UtilityColors.Assign(Tasks.Select(t => t.Label));

                // 유틸리티별로 메시 MeshBuilder 를 머지한다.
                var perUtil = new Dictionary<string, (Color col, MeshBuilder mb)>();
                foreach (var row in successRows)
                {
                    string label = row.Label;
                    if (!perUtil.TryGetValue(label, out var entry))
                    {
                        var col = colorMap.TryGetValue(label, out var c) ? c : Colors.Gray;
                        entry = (col, new MeshBuilder(true, false));
                        perUtil[label] = entry;
                    }
                    var pts = GetRoutedPolyline(row, grid);
                    if (pts.Count < 2) continue;
                    double dia = row.DiameterMm > 0 ? Math.Max(row.DiameterMm, 8.0) : fallbackDia;
                    entry.mb.AddTube(pts, dia, 10, false);
                }

                // 유틸리티별 MeshGeometry3D → float 배열로 변환(Y-up, m)
                var parts = new List<(string label, Color color, float[] pos, float[] nrm, uint[] idx,
                                      float[] pmin, float[] pmax)>();
                const double mmToM = 1.0 / 1000.0;

                foreach (var kv in perUtil)
                {
                    var (col, mb) = kv.Value;
                    var mesh = mb.ToMesh();
                    if (mesh.Positions.Count == 0 || mesh.TriangleIndices.Count == 0) continue;

                    int nv = mesh.Positions.Count;
                    bool hasNormals = mesh.Normals != null && mesh.Normals.Count == nv;
                    var pos = new float[nv * 3];
                    var nrm = hasNormals ? new float[nv * 3] : Array.Empty<float>();

                    float pminX = float.MaxValue, pminY = float.MaxValue, pminZ = float.MaxValue;
                    float pmaxX = float.MinValue, pmaxY = float.MinValue, pmaxZ = float.MinValue;

                    for (int vi = 0; vi < nv; vi++)
                    {
                        // Routing3D: Z-up, mm → glTF: Y-up, m  →  (x, z, -y) / 1000
                        var p = mesh.Positions[vi];
                        float gx = (float)(p.X * mmToM);
                        float gy = (float)(p.Z * mmToM);
                        float gz = (float)(-p.Y * mmToM);
                        pos[vi * 3 + 0] = gx;
                        pos[vi * 3 + 1] = gy;
                        pos[vi * 3 + 2] = gz;
                        if (gx < pminX) pminX = gx; if (gx > pmaxX) pmaxX = gx;
                        if (gy < pminY) pminY = gy; if (gy > pmaxY) pmaxY = gy;
                        if (gz < pminZ) pminZ = gz; if (gz > pmaxZ) pmaxZ = gz;
                        if (hasNormals)
                        {
                            var n = mesh.Normals![vi];
                            nrm[vi * 3 + 0] = (float)n.X;
                            nrm[vi * 3 + 1] = (float)n.Z;
                            nrm[vi * 3 + 2] = (float)-n.Y;
                        }
                    }

                    int ni = mesh.TriangleIndices.Count;
                    var idx = new uint[ni];
                    for (int ii = 0; ii < ni; ii++) idx[ii] = (uint)mesh.TriangleIndices[ii];

                    parts.Add((kv.Key, col, pos, nrm, idx,
                               new[] { pminX, pminY, pminZ }, new[] { pmaxX, pmaxY, pmaxZ }));
                }

                if (parts.Count == 0) { ShowStatus("내보낼 유효 메시가 없습니다."); return; }

                GlbExporter.Write(dlg.FileName, parts);
                int totalPipes = successRows.Count;
                ShowStatus($"GLB 저장 완료 — {totalPipes}개 배관 · {parts.Count}개 유틸리티 → {dlg.FileName}");
            }
            catch (Exception ex)
            {
                ShowStatus($"GLB 저장 실패: {ex.Message}");
            }
        }

        private void BuildRouteReport(IReadOnlyList<int> added, string strategy, int batchOk, int sceneOk, int replicated)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("# AI 자동설계 결과 보고서");
            sb.AppendLine();
            sb.AppendLine($"- **수행 일시**: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            sb.AppendLine($"- **프로젝트**: {SelectedProject?.ProjectId.ToString() ?? "N/A"} ({_scene?.SourceFile ?? "N/A"})");
            sb.AppendLine($"- **탐색 전략**: {strategy}");
            sb.AppendLine($"- **격자 셀 크기**: {CellMm} mm");
            sb.AppendLine($"- **배치 성공**: {batchOk} / {added.Count} (실패: {added.Count - batchOk})");
            sb.AppendLine($"- **전체 누적 성공**: {sceneOk} / {Tasks.Count}");
            if (replicated != 0)
            {
                sb.AppendLine($"- **기존 설계 복제 건수**: {replicated}");
            }
            sb.AppendLine();
            sb.AppendLine("## 배치 배관 상세 내역");
            sb.AppendLine();
            sb.AppendLine("| 번호 | 유틸리티 | 그룹 | 출발지(PoC) | 목적지 | 상태 | 길이 (mm) | 꺾임 수 |");
            sb.AppendLine("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |");

            for (int i = 0; i < added.Count; i++)
            {
                var tr = Tasks[added[i]];
                string num = (i + 1).ToString();
                string util = tr.Utility ?? "(미지정)";
                string group = tr.Group ?? "(미정)";
                string sp = string.IsNullOrEmpty(tr.PocName) ? $"({tr.Sx:0}, {tr.Sy:0}, {tr.Sz:0})" : tr.PocName;
                string ep = string.IsNullOrEmpty(tr.EndName) ? $"({tr.Gx:0}, {tr.Gy:0}, {tr.Gz:0})" : tr.EndName;
                string status = tr.Success ? "성공" : "실패";
                string length = tr.Success ? tr.LengthMm.ToString("N0") : "-";
                string turns = tr.Success ? tr.TurnCount.ToString() : "-";

                sb.AppendLine($"| {num} | {util} | {group} | {sp} | {ep} | {status} | {length} | {turns} |");
            }

            RouteResultReport = sb.ToString();

            try
            {
                string dir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "out", "report");
                Directory.CreateDirectory(dir);
                string filename = $"route_result_{DateTime.Now:yyyyMMdd_HHmmss}.md";
                RouteResultReportPath = Path.Combine(dir, filename);
                File.WriteAllText(RouteResultReportPath, RouteResultReport, System.Text.Encoding.UTF8);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[경고] 임시 리포트 파일 저장 실패: {ex.Message}");
            }
        }

        public TaskRowVM? SelectedTask
        {
            get => _selectedTask;
            set
            {
                if (!Set(ref _selectedTask, value)) return;
                _compareMode = false;            // 새 배관 선택 → 비교 포커스 해제(숨겼던 기존배관 복원).
                if (_scene != null && _engine != null)
                    BuildModel();                 // 결과 행 선택 시 해당 자동/기존 경로만 보이도록 기본 경로 레이어를 다시 구성.
                else
                    UpdateSelectionHighlight();   // → UpdateComparison(): 새 배관 매칭 오버레이/분석.
            }
        }
        public PickMode PickMode { get => _pickMode; private set => Set(ref _pickMode, value); }

        public bool ShowObstacles { get => _showObstacles; set { if (Set(ref _showObstacles, value)) RebuildIfReady(); } }
        public bool ShowPaths { get => _showPaths; set { if (Set(ref _showPaths, value)) RebuildIfReady(); } }
        public bool ShowCollisions { get => _showCollisions; set { if (Set(ref _showCollisions, value)) RebuildIfReady(); } }
        public bool ShowGridFrame { get => _showGridFrame; set { if (Set(ref _showGridFrame, value)) RebuildIfReady(); } }
        public bool ShowOccupancyVoxels { get => _showOccupancyVoxels; set { if (Set(ref _showOccupancyVoxels, value)) RebuildIfReady(); } }
        public bool ShowVisitedMap { get => _showVisitedMap; set { if (Set(ref _showVisitedMap, value)) RebuildIfReady(); } }
        public bool EnableSearchTrace
        {
            get => _enableSearchTrace;
            set { if (Set(ref _enableSearchTrace, value)) OnChanged(nameof(SearchTraceStatus)); }
        }
        public string LastSearchTraceFile { get => _lastSearchTraceFile; private set { if (Set(ref _lastSearchTraceFile, value)) OnChanged(nameof(SearchTraceStatus)); } }
        public string SearchTraceStatus => string.IsNullOrEmpty(_lastSearchTraceFile) ? "탐색로그" : $"탐색로그: {Path.GetFileName(_lastSearchTraceFile)}";
        public bool ShowOctreeVoxels { get => _showOctreeVoxels; set { if (Set(ref _showOctreeVoxels, value)) RebuildIfReady(); } }
        public bool ShowPassthroughVoxels { get => _showPassthroughVoxels; set { if (Set(ref _showPassthroughVoxels, value)) RebuildIfReady(); } }
        public int  RenderObjectCount { get => _renderObjectCount; private set => Set(ref _renderObjectCount, value); }
        public int  OctreeLeafCount   { get => _octreeLeafCount;  private set => Set(ref _octreeLeafCount, value); }
        public bool ShowSpaces { get => _showSpaces; set { if (Set(ref _showSpaces, value)) RebuildIfReady(); } }
        public bool ShowEquipment { get => _showEquipment; set { if (Set(ref _showEquipment, value)) RebuildIfReady(); } }
        public bool ShowLaterals { get => _showLaterals; set { if (Set(ref _showLaterals, value)) RebuildIfReady(); } }
        public bool ShowDucts { get => _showDucts; set { if (Set(ref _showDucts, value)) RebuildIfReady(); } }
        public bool ShowExistingPipes { get => _showExistingPipes; set { if (Set(ref _showExistingPipes, value)) RebuildIfReady(); } }
        /// <summary>모든 작업(장비)의 시작 PoC(빨강 구)·종단 PoC(파랑 구) 마커 — 라우팅 전에도 초기 표시. 기본 ON.</summary>
        public bool ShowPocMarkers { get => _showPocMarkers; set { if (Set(ref _showPocMarkers, value)) RebuildIfReady(); } }
        /// <summary>기존설계 배관의 출발(빨강)·종단(파랑) 스텁(수직배관+엘보)을 굵은 색 튜브로 강조. 기본 ON.
        /// 학습 파이프라인(StubExtractor)과 동일 로직으로 잘라내 학습 스텁과 일치한다.</summary>
        public bool ShowStubs { get => _showStubs; set { if (Set(ref _showStubs, value)) RebuildIfReady(); } }

        /// <summary>그룹배관 강조 — 현재 로드된 기존배관 중 탐지된 번들(route_bundle_group) 멤버를 '그룹별 고유 색'으로
        /// 그리고 비멤버는 흐리게 표시한다. 동일 이격간격·2회+ 꺾임 다발이 한눈에 보인다. DB 적재(bundle_detect
        /// --write-db) 필요. 미적재면 비활성(상태 라벨에 표시). 기본 OFF.</summary>
        public bool ShowBundleGroups
        {
            get => _showBundleGroups;
            set { if (Set(ref _showBundleGroups, value)) { OnChanged(nameof(BundleGroupStatus)); RebuildIfReady(); } }
        }

        /// <summary>그룹배관 강조 상태 표시(UI 라벨).</summary>
        public string BundleGroupStatus =>
            !_showBundleGroups ? "그룹배관 강조: OFF"
            : _bundles == null || _bundles.GroupCount == 0 ? "그룹배관 강조: 없음(미적재)"
            : $"그룹배관 강조: {_bundles.GroupCount}그룹";

        /// <summary>그룹배관 패턴 표시 — 기존설계에서 학습한 유틸별 '공용 트렁크 레인'(L4, route_bundle_template)을
        /// 메인/미니 3D 에 보라색 반투명 큐브로 그린다. 신규 라우팅이 이 레인을 따라 다발로 뭉친다(라우팅에 주입하는
        /// 회랑 셀 = BuildBundleCorridorCells 와 동일). 드릴다운(그룹/유틸 선택) 시 그 부분집합만. 미적재면 비활성. 기본 OFF.</summary>
        public bool ShowBundlePattern
        {
            get => _showBundlePattern;
            set { if (Set(ref _showBundlePattern, value)) { OnChanged(nameof(BundlePatternStatus)); RebuildIfReady(); } }
        }

        /// <summary>그룹배관 패턴 표시 상태(UI 라벨).</summary>
        public string BundlePatternStatus =>
            !_showBundlePattern ? "그룹배관 패턴 표시: OFF"
            : _bundles == null ? "그룹배관 패턴 표시: 없음(미적재)"
            : $"그룹배관 패턴 표시: {_bundles.Count}키";

        /// <summary>충돌확장 — 라우팅 시 설비(메인 장비 포함)·덕트·레터럴 + 이미 설계된(라우팅 성공) 다른
        /// 배관의 경로를 장애물로 추가해 충돌을 피한다. <b>항상 ON 고정(표준 라우팅 동작, 토글 잠금)</b> —
        /// getter 전용이라 UI 에서 끌 수 없다(체크박스는 켜진 채 비활성).</summary>
        public bool IncludeFacilities => _includeFacilities;   // _includeFacilities 는 항상 true(고정).

        /// <summary>기존설계 패턴 활용 — 학습된 진출/진입 면(장비=-z·덕트=+z 등)으로 시작/종단 PoC 를
        /// 투영해 자동라우팅을 사람 설계에 맞춘다. 패턴 저장소(pgvector route_stub_pattern)가 비었거나
        /// 키 미스면 자동으로 기존 최근접-면 규칙으로 폴백(무해). 기본 ON.</summary>
        public bool UsePatterns
        {
            get => _usePatterns;
            set { if (Set(ref _usePatterns, value)) OnChanged(nameof(PatternStatus)); }
        }

        /// <summary>기존설계 회랑(L2b) — 매칭 기존배관 폴리라인을 회랑으로 주입해 새 경로가 사람 설계를
        /// 부드럽게 따라가게 한다(w_corridor 소프트 바이어스, 충돌은 여전히 회피). 경로 모양을 바꾸므로 기본 OFF.</summary>
        public bool UseDesignCorridor
        {
            get => _useDesignCorridor;
            set { if (Set(ref _useDesignCorridor, value)) OnChanged(nameof(PatternStatus)); }
        }

        /// <summary>유틸그룹 랙 번들링(L3a) — 기존배관 수평 런이 모이는 z-높이(랙)를 그룹별로 학습해 같은
        /// 그룹 새 배관을 공용 랙 높이에 뭉치게 한다(엔진 rack_levels + 가벼운 w_corridor). 경로 모양을 바꾸므로 기본 OFF.</summary>
        public bool UseRackBundling
        {
            get => _useRackBundling;
            set { if (Set(ref _useRackBundling, value)) OnChanged(nameof(PatternStatus)); }
        }

        /// <summary>그룹배관 패턴(L4) — DB 에 저장된 대표 그룹배관 패턴(공용 트렁크 고도)을 읽어, 같은 유틸
        /// 새 배관을 학습된 공용 랙 고도에 뭉치게 한다(엔진 rack_levels + 가벼운 w_corridor). 미적재/미스 시
        /// 자동 폴백(무해). 기본 ON(항상 적용).</summary>
        public bool UseBundlePattern
        {
            get => _useBundlePattern;
            set { if (Set(ref _useBundlePattern, value)) OnChanged(nameof(BundleStatus)); }
        }

        /// <summary>그룹배관 패턴 저장소 상태 표시(UI 라벨).</summary>
        public string BundleStatus =>
            !_useBundlePattern ? "그룹배관 패턴: OFF"
            : _bundles == null ? "그룹배관 패턴: 없음(미적재)"
            : $"그룹배관 패턴: {_bundles.Count}키";

        /// <summary>스텁 라우팅 — 매칭 기존배관의 출발/종단 스텁(수직+엘보)을 고정 설계 구간으로 깔고 A* 는
        /// 스텁 끝~끝만 탐색(표시 = 스텁+중간+스텁). 매칭 없으면 PoC 직접 라우팅으로 폴백. 기본 ON.</summary>
        public bool UseStubRouting
        {
            get => _useStubRouting;
            set { Set(ref _useStubRouting, value); }
        }

        /// <summary>기존배관 복제(폴리라인) — 매칭 기존배관의 폴리라인을 그대로 복제하고, 현재 점유에서 막힌
        /// (장애물이 달라진) 구간만 A* 로 국소 우회 수리한다. 가장 강한 '기존설계 유사'. 매칭 없으면 일반 A*
        /// 결과 유지(무해). 경로를 크게 바꾸므로 기본 OFF.</summary>
        public bool UseDesignReplicate
        {
            get => _useDesignReplicate;
            set { if (Set(ref _useDesignReplicate, value)) OnChanged(nameof(PatternStatus)); }
        }

        /// <summary>CBS(협상 라우팅) — rip-up 후에도 실패 배관이 남으면 blocker-of-blocker 까지 재귀 양보(depth=2).
        /// ON=cbs_depth 2, OFF=0(평면 rip-up 만·기본). 대·소형 격자 모두 적용.</summary>
        public bool UseCbs
        {
            get => _useCbs;
            set { Set(ref _useCbs, value); }
        }

        /// <summary>패턴 저장소 상태 표시(UI 라벨).</summary>
        public string PatternStatus =>
            !_usePatterns ? "기존설계 패턴: OFF"
            : _patterns == null ? "기존설계 패턴: 없음(기하 폴백)"
            : $"기존설계 패턴: {_patterns.Count}키";

        // ──────── 경로 탐색 방식 3-모드 (라디오버튼 바인딩) ────────
        // 모드가 바뀌면 내부 옵션(_usePatterns·_useStubRouting·_useDesignReplicate)을 자동 동기화.
        // ─ Shortest       : 패턴·스텁 OFF, 복제 OFF → 순수 최단 A*
        // ─ PatternApplied : 패턴·스텁 ON,  복제 OFF → 특징점(스텁+패턴) 반영 + 그룹 회랑
        // ─ FollowExisting : 패턴·스텁 ON,  복제 ON  → 기존배관 폴리라인 복제
        public bool IsModeShortest
        {
            get => _routingMode == RoutingMode.Shortest;
            set { if (value) ApplyRoutingMode(RoutingMode.Shortest); }
        }

        public bool IsModePatternApplied
        {
            get => _routingMode == RoutingMode.PatternApplied;
            set { if (value) ApplyRoutingMode(RoutingMode.PatternApplied); }
        }

        public bool IsModeFollowExisting
        {
            get => _routingMode == RoutingMode.FollowExisting;
            set { if (value) ApplyRoutingMode(RoutingMode.FollowExisting); }
        }

        private void ApplyRoutingMode(RoutingMode mode)
        {
            if (_routingMode == mode) return;
            _routingMode = mode;
            _usePatterns       = mode != RoutingMode.Shortest;
            _useStubRouting    = mode != RoutingMode.Shortest;
            _useDesignReplicate = mode == RoutingMode.FollowExisting;
            OnChanged(nameof(IsModeShortest));
            OnChanged(nameof(IsModePatternApplied));
            OnChanged(nameof(IsModeFollowExisting));
            OnChanged(nameof(PatternStatus));
        }

        /// <summary>점유맵 해상도. true=원본(전체 셀 표시, 느릴 수 있음), false=다운샘플(상한까지만).</summary>
        public bool OccupancyFullRes
        {
            get => _occupancyFullRes;
            set { if (Set(ref _occupancyFullRes, value)) { OnChanged(nameof(OccupancyResolutionLabel)); if (_showOccupancyVoxels) RebuildIfReady(); } }
        }

        /// <summary>해상도 토글 버튼 라벨(현재 모드 표시).</summary>
        public string OccupancyResolutionLabel => _occupancyFullRes ? "원본" : "샘플";

        public ObservableCollection<TaskRowVM> Tasks { get; } = new();
        public ObservableCollection<LegendItem> Legend { get; } = new();
        public ObservableCollection<UtilityFilterVM> UtilityFilters { get; } = new();

        /// <summary>3D 공간 영역 텍스트 라벨(코드비하인드가 BillboardText 로 렌더). BuildModel 에서 갱신.</summary>
        public ObservableCollection<SpaceLabel> SpaceLabels { get; } = new();

        // ---- 경로 탐색 범위 ----
        /// <summary>탐색 범위 콤보 항목.</summary>
        public ObservableCollection<RouteScopeOption> RouteScopes { get; } = new()
        {
            new RouteScopeOption { Scope = RouteScope.All,       Label = "모두 (전체 충돌회피)" },
            new RouteScopeOption { Scope = RouteScope.ByGroup,   Label = "유틸리티그룹별 (1개 선택)" },
            new RouteScopeOption { Scope = RouteScope.ByUtility, Label = "유틸리티별 (1개 선택)" },
        };

        /// <summary>범위가 그룹별/유틸별일 때 선택 가능한 대상 목록(그룹명 또는 유틸명).</summary>
        public ObservableCollection<string> RouteTargets { get; } = new();

        /// <summary>선택된 탐색 범위. 바뀌면 대상 목록을 다시 만든다.</summary>
        public RouteScopeOption SelectedRouteScope
        {
            get => _selectedRouteScope;
            set { if (Set(ref _selectedRouteScope, value)) { RebuildRouteTargets(); OnChanged(nameof(IsTargetSelectable)); } }
        }

        /// <summary>선택된 대상(그룹명/유틸명). 범위가 '모두'면 무시된다.</summary>
        public string? SelectedRouteTarget { get => _selectedRouteTarget; set => Set(ref _selectedRouteTarget, value); }

        /// <summary>대상 콤보 활성 여부(범위가 '모두'가 아닐 때만).</summary>
        public bool IsTargetSelectable => _selectedRouteScope != null && _selectedRouteScope.Scope != RouteScope.All;

        // ---- 좌측 드릴다운: 유틸리티 그룹 → 유틸리티 → 개별 PoC ----
        /// <summary>1단계: 유틸리티 그룹 목록.</summary>
        public ObservableCollection<string> GroupList { get; } = new();
        /// <summary>2단계: 선택 그룹의 유틸리티 목록.</summary>
        public ObservableCollection<string> UtilityList { get; } = new();
        /// <summary>3단계: 선택 (그룹,유틸)의 개별 PoC(작업) 목록.</summary>
        public ObservableCollection<TaskRowVM> PocList { get; } = new();

        /// <summary>선택된 유틸리티 그룹. 선택 시 유틸리티 목록을 채우고 상단 범위를 '그룹별'로 동기화.</summary>
        public string? SelectedGroup
        {
            get => _selectedGroup;
            set
            {
                if (!Set(ref _selectedGroup, value)) return;
                RebuildUtilityList();
                RebuildIfReady();   // 기존 설계배관을 선택 그룹만 표시(미선택=전체)하도록 3D 갱신.
                if (!_suppressDrillCascade && !string.IsNullOrEmpty(value))
                    SyncTopScope(RouteScope.ByGroup, value);   // '경로 탐색 실행'이 이 그룹을 라우팅.
            }
        }

        /// <summary>선택된 유틸리티. 선택 시 개별 PoC 목록을 채우고 상단 범위를 '유틸별'로 동기화.</summary>
        public string? SelectedUtility
        {
            get => _selectedUtility;
            set
            {
                if (!Set(ref _selectedUtility, value)) return;
                RebuildPocList();
                if (!_suppressDrillCascade && !string.IsNullOrEmpty(value))
                    SyncTopScope(RouteScope.ByUtility, value);   // '경로 탐색 실행'이 이 유틸을 라우팅.
            }
        }

        private bool _suppressDrillCascade;

        /// <summary>선택 PoC 강조 오버레이(시작/끝 마커). 라우팅과 무관 — '선택은 강조표시만'.</summary>
        public Model3D? SelectionModel { get => _selectionModel; private set => Set(ref _selectionModel, value); }
        public Model3D? HighlightModel { get => _highlightModel; private set => Set(ref _highlightModel, value); }

        /// <summary>A* 단계별 탐색 오버레이(방문 셀을 확장 순서대로 점진 표시).</summary>
        public Model3D? SearchModel { get => _searchModel; private set => Set(ref _searchModel, value); }

        private Model3D? _stepHighlightModel;
        /// <summary>선택한 경로 단계(구간) 강조 오버레이 — 카메라 이동 없이 해당 구간만 흰색으로.</summary>
        public Model3D? StepHighlightModel { get => _stepHighlightModel; private set => Set(ref _stepHighlightModel, value); }

        /// <summary>기존설계 비교 오버레이(기존 경로=주황 / 개발 경로=시안 굵은 튜브 + 시작/끝 마커).</summary>
        public Model3D? CompareModel { get => _compareModel; private set => Set(ref _compareModel, value); }

        // 진행 다이얼로그 라우팅 중 '배관 완료마다' 점진 표시하는 라이브 오버레이(유틸 색 튜브).
        // 라우팅이 끝나면 비우고 BuildModel 의 최종 렌더로 대체한다(중복 방지).
        private Model3D? _liveRouteModel;
        public Model3D? LiveRouteModel { get => _liveRouteModel; private set => Set(ref _liveRouteModel, value); }
        private Model3DGroup? _liveGroup;

        /// <summary>우측 '기존설계 비교 분석' 패널 텍스트(매칭 상태 + 길이/꺾임/종단점/간섭 지표). null=비활성.</summary>
        public string? ComparisonReport { get => _comparisonReport; private set => Set(ref _comparisonReport, value); }

        /// <summary>선택 경로의 직선 구간(단계) 목록. 방향이 바뀌는 지점마다 한 항목.</summary>
        public ObservableCollection<PathStep> PathSteps { get; } = new();

        /// <summary>'단계별 경로' 탭(그리드) 행 — 선택 배관의 시작→꺾임(사유)→종단을 구조화. BuildSelectedTaskAnalysis 가 채운다.</summary>
        public ObservableCollection<RouteStepRow> RouteStepRows { get; } = new();

        private RouteStepRow? _selectedRouteStep;
        /// <summary>'단계별 경로' 그리드에서 선택한 행 — 자동경로 3D 뷰에서 해당 구간(A→B)을 흰색 굵은 튜브로 강조한다.</summary>
        public RouteStepRow? SelectedRouteStep
        {
            get => _selectedRouteStep;
            set
            {
                if (!Set(ref _selectedRouteStep, value)) return;
                if (value == null || _scene == null) { StepHighlightModel = null; return; }
                // 시작/종단 행은 A==B(한 점=구), 구간 행은 A→B(직선) 강조. HighlightStep 가 둘 다 처리.
                HighlightStep(new PathStep { A = value.A, B = value.B, Position = value.A });
            }
        }

        private PathStep? _selectedStep;
        private bool _suppressStepNav;   // 목록 재구성 중 자동 네비게이션 방지.
        /// <summary>선택된 단계. 사용자가 목록에서 고르면 그 위치로 카메라를 이동(NavigateToRequested).</summary>
        public PathStep? SelectedStep
        {
            get => _selectedStep;
            // 단계를 고르면 해당 구간을 강조하고 카메라를 구간 시작 위치로 이동한다.
            set
            {
                if (!Set(ref _selectedStep, value) || _suppressStepNav) return;
                HighlightStep(value);
                if (value != null) NavigateToRequested?.Invoke(value.Position);
            }
        }

        /// <summary>단계 클릭 시 해당 월드좌표로 카메라 이동 요청(코드비하인드가 처리).</summary>
        public event Action<Point3D>? NavigateToRequested;

        // 1단계: 그룹 목록을 작업 분포에서 채운다(새 프로젝트 로드 시).
        private void RebuildGroupList()
        {
            _suppressDrillCascade = true;
            GroupList.Clear();
            foreach (var g in Tasks.Select(t => GroupKey(t.Group)).Distinct().OrderBy(s => s, StringComparer.Ordinal))
                GroupList.Add(g);
            SelectedGroup = null;        // 사용자가 직접 고르도록(자동 라우팅 방지).
            UtilityList.Clear();
            PocList.Clear();
            SelectedUtility = null;
            _suppressDrillCascade = false;
        }

        // 2단계: 선택 그룹의 유틸리티 목록.
        private void RebuildUtilityList()
        {
            bool prev = _suppressDrillCascade;
            _suppressDrillCascade = true;
            UtilityList.Clear();
            PocList.Clear();
            _selectedUtility = null; OnChanged(nameof(SelectedUtility));
            if (!string.IsNullOrEmpty(_selectedGroup))
                foreach (var u in Tasks.Where(t => GroupKey(t.Group) == _selectedGroup)
                                       .Select(t => UtilityKey(t.Utility)).Distinct()
                                       .OrderBy(s => s, StringComparer.Ordinal))
                    UtilityList.Add(u);
            _suppressDrillCascade = prev;
        }

        // 3단계: 선택 (그룹,유틸)의 개별 PoC 작업 목록.
        private void RebuildPocList()
        {
            PocList.Clear();
            if (string.IsNullOrEmpty(_selectedGroup) || string.IsNullOrEmpty(_selectedUtility)) return;
            foreach (var row in Tasks.Where(t => GroupKey(t.Group) == _selectedGroup &&
                                                 UtilityKey(t.Utility) == _selectedUtility))
                PocList.Add(row);
        }

        // 드릴다운 선택을 상단 범위/대상으로 일방 동기화('라우팅은 상단 범위로').
        private void SyncTopScope(RouteScope scope, string target)
        {
            SelectedRouteScope = RouteScopes.First(o => o.Scope == scope);   // RebuildRouteTargets 호출 → target=첫째.
            SelectedRouteTarget = target;                                    // 원하는 대상으로 덮어씀.
        }

        // ---- 바닥 격자(GridLinesVisual3D) 파라미터 — 씬 좌표에 맞춰 코드비하인드가 읽어 갱신 ----
        // 하드코딩하면 실제 DB 좌표(수만 mm)와 떨어져 ZoomExtents 가 빗나가 객체가 구석에 작게 보인다.
        public Point3D GroundCenter { get; private set; } = new Point3D(0, 0, 0);
        public double GroundWidth { get; private set; } = 1000;
        public double GroundLength { get; private set; } = 1000;
        public double GroundMinorDistance { get; private set; } = 1000;
        public double GroundMajorDistance { get; private set; } = 5000;

        // 격자 BBOX(원점=lo, 크기=N*cell) 로부터 바닥 격자 위치/크기/간격을 산출한다.
        private void UpdateGroundGrid(GridMeta g)
        {
            double w = g.Nx * g.CellMm, l = g.Ny * g.CellMm;
            GroundCenter = new Point3D(g.Ox + w / 2, g.Oy + l / 2, g.Oz);   // z=격자 바닥.
            GroundWidth = w; GroundLength = l;
            // 큰 변 기준 ~20칸이 되도록 간격을 '예쁜 값'으로(라인 수 폭주 방지).
            GroundMinorDistance = NiceSpacing(Math.Max(w, l) / 20.0, g.CellMm);
            GroundMajorDistance = GroundMinorDistance * 5;
        }

        // size 이상의 가장 가까운 1·2·5×10^n 값(최소 cell).
        private static double NiceSpacing(double target, double cell)
        {
            if (target <= cell) return cell;
            double mag = Math.Pow(10, Math.Floor(Math.Log10(target)));
            double norm = target / mag;
            double nice = norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 5 ? 5 : 10;
            return Math.Max(cell, nice * mag);
        }

        /// <summary>유틸리티/검색 필터가 적용된 작업 목록 뷰(ListBox 바인딩).</summary>
        public ICollectionView TasksView { get; }

        /// <summary>작업 라벨 검색 문자열(부분일치, 대소문자 무시). 비우면 모두 통과.</summary>
        public string SearchText
        {
            get => _searchText;
            set { if (Set(ref _searchText, value)) { TasksView.Refresh(); OnChanged(nameof(TaskCountText)); } }
        }

        /// <summary>현재 표시되는 작업 수/전체 수(예: "120 / 208"). 좌측 패널 헤더용.</summary>
        public string TaskCountText
        {
            get
            {
                int visible = Tasks.Count(TaskFilterCore);
                return $"{visible} / {Tasks.Count}";
            }
        }

        public RelayCommand OpenCommand { get; }
        public RelayCommand DemoCommand { get; }
        public RelayCommand RunRouteCommand { get; }
        public RelayCommand RerouteCorridorCommand { get; }
        public RelayCommand RerouteSelectedCommand { get; }
        public RelayCommand CompareSelectedCommand { get; }
        public RelayCommand PickStartCommand { get; }
        public RelayCommand PickEndCommand { get; }
        public RelayCommand FitViewCommand { get; }
        public RelayCommand ToggleOccupancyResCommand { get; }
        public RelayCommand AnimateSelectedCommand { get; }
        public RelayCommand UtilityAllCommand { get; }
        public RelayCommand UtilityClearCommand { get; }
        public RelayCommand LoadProjectsCommand { get; }
        public RelayCommand LoadDbCommand { get; }
        public RelayCommand RouteGroupCommand { get; }
        public RelayCommand RouteUtilityCommand { get; }
        public RelayCommand ClearRoutesCommand { get; }
        public RelayCommand CancelRoutingCommand { get; }
        public RelayCommand RouteResultReportCommand { get; }
        public RelayCommand ExportGlbCommand { get; }
        public RelayCommand SaveRouteResultsCommand { get; }
        public RelayCommand LoadRouteResultsCommand { get; }

        // ---- DB 접속 설정(상단 툴바 텍스트박스 바인딩) ----
        public string DbHost { get => _dbConfig.Host; set { _dbConfig.Host = value; OnChanged(); } }
        public int DbPort { get => _dbConfig.Port; set { _dbConfig.Port = value; OnChanged(); } }
        public string DbUser { get => _dbConfig.User; set { _dbConfig.User = value; OnChanged(); } }
        public string DbPassword { get => _dbConfig.Password; set { _dbConfig.Password = value; OnChanged(); } }
        public string DbDatabase { get => _dbConfig.Database; set { _dbConfig.Database = value; OnChanged(); } }
        public double CellMm { get => _cellMm; set => Set(ref _cellMm, value); }

        public ObservableCollection<ProjectInfo> Projects { get; } = new();
        public ProjectInfo? SelectedProject
        {
            get => _selectedProject;
            set
            {
                if (!Set(ref _selectedProject, value)) return;
                if (_suppressProjectAutoLoad || value == null) return;
                _ = LoadFromDbAsync(value.ProjectId);   // 비동기(UI 비차단). 예외는 내부에서 Status 로.
            }
        }

        /// <summary>모델을 새로 만들면 발생(코드비하인드가 ZoomExtents 호출).</summary>
        public event Action? SceneRebuilt;

        /// <summary>'전체보기' 명령(코드비하인드가 ZoomExtents 호출).</summary>
        public event Action? FitViewRequested;

        /// <summary>특정 영역(Rect3D)으로 메인 뷰를 줌(코드비하인드가 CameraHelper.ZoomExtents 호출).
        /// 진행 다이얼로그에서 배관 행을 클릭하면 그 배관 로컬 영역으로 메인 뷰를 맞춘다.</summary>
        public event Action<Rect3D>? ZoomToBoxRequested;

        // ---- 필터 ----
        private bool TaskFilter(object o) => o is TaskRowVM r && TaskFilterCore(r);

        private bool TaskFilterCore(TaskRowVM r)
        {
            if (!string.IsNullOrWhiteSpace(_searchText) &&
                r.Label.IndexOf(_searchText, StringComparison.OrdinalIgnoreCase) < 0)
                return false;
            var f = UtilityFilters.FirstOrDefault(u => u.Label == r.Label);
            return f == null || f.IsVisible;
        }

        private void OnUtilityFilterChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (_suppressFilterRebuild) return;
            if (e.PropertyName != nameof(UtilityFilterVM.IsVisible)) return;
            TasksView.Refresh();
            OnChanged(nameof(TaskCountText));
            if (_scene != null && _engine != null) BuildModel();
        }

        private void SetAllUtilities(bool visible)
        {
            _suppressFilterRebuild = true;
            foreach (var u in UtilityFilters) u.IsVisible = visible;
            _suppressFilterRebuild = false;
            TasksView.Refresh();
            OnChanged(nameof(TaskCountText));
            if (_scene != null && _engine != null) BuildModel();
        }

        // ---- 로드 ----
        private void Open()
        {
            var dlg = new OpenFileDialog
            {
                Title = "scene.txt 열기",
                Filter = "scene 파일|*.scene.txt;*.txt|모든 파일|*.*"
            };
            if (dlg.ShowDialog() != true) return;
            try { LoadFile(dlg.FileName); }
            catch (Exception ex) { Status = "열기 오류: " + ex.Message; }
        }

        private void LoadFile(string path)
        {
            string text = File.ReadAllText(path, Encoding.UTF8);
            _scene = SceneTextParser.Parse(text);
            ResetEngine();
            _engine!.LoadSceneText(text);   // scene.txt 는 엔진 내부 파서로 적재(정확 동일성 보존).
            BuildTaskRows();
            // scene.txt/--selftest 경로는 동기 결과(vm.Status)를 즉시 읽으므로 '모두' 동기 라우팅.
            // LoadSceneText 가 작업을 파일 순서대로 적재하므로 엔진 인덱스 == 행 인덱스.
            try
            {
                _engine.RouteMulti(_priority);
                CacheResultsByIndex();
                BuildModel();
            }
            catch (Exception ex) { Status = "경로 탐색 오류: " + ex.Message; }
        }

        // ---- DB 로드 ----
        /// <summary>space_project_map 에서 프로젝트 목록을 읽어 Projects 에 채우고, 첫 항목 선택 시
        /// SelectedProject 의 set 이 자동으로 LoadFromDb 를 호출(전체 자동 로드 흐름).</summary>
        private async Task LoadProjectsAsync()
        {
            try
            {
                // DB 목록 조회는 네트워크 I/O → 백그라운드. (실패 시 ~TimeoutSec 후 예외.)
                Status = "DB 프로젝트 목록 조회 중…";
                var list = await Task.Run(() => ObstacleDbLoader.ListProjects(_dbConfig));
                _suppressProjectAutoLoad = true;
                Projects.Clear();
                foreach (var p in list) Projects.Add(p);
                if (Projects.Count == 0)
                {
                    _suppressProjectAutoLoad = false;
                    Status = "DB 에 프로젝트가 없습니다(space_project_map 비어 있음)";
                    return;
                }
                Status = $"프로젝트 {Projects.Count}개 로드";
                // 콤보에 첫 항목을 표시하되 setter 의 자동 로드는 억제하고, 아래에서 명시적으로 await 한다.
                SelectedProject = Projects[0];
                _suppressProjectAutoLoad = false;
                await LoadFromDbAsync(Projects[0].ProjectId);
            }
            catch (Exception ex)
            {
                _suppressProjectAutoLoad = false;
                Status = "DB 접속 실패: " + ex.Message;
            }
        }

        /// <summary>한 프로젝트의 장애물·PoC 페어를 DB 에서 읽어 엔진에 적재한다.
        /// 사용자 요청대로 <b>라우팅은 하지 않고 장애물만 전체화면으로 보여준다</b>.
        /// 경로 탐색은 사용자가 범위(모두/그룹별/유틸별)를 고르고 '경로 탐색 실행'을 눌러 시작한다.
        /// DB I/O 는 백그라운드로(연결 지연에도 UI 가 멈추지 않게).</summary>
        private async Task LoadFromDbAsync(int projectId)
        {
            try
            {
                Status = "DB 장면 로드 중…";
                var sd = await Task.Run(() => ObstacleDbLoader.LoadScene(_dbConfig, projectId, _cellMm));
                _scene = sd;
                // 기존설계 패턴 저장소(pgvector)를 1회 로드(미스도 캐시) — 학습된 진출/진입 면으로 PoC 투영(L2a).
                if (!_patternsTried)
                {
                    _patternsTried = true;
                    _patterns = await Task.Run(() => PatternStore.TryLoad(_dbConfig));
                    OnChanged(nameof(PatternStatus));
                }
                // 그룹배관 번들 템플릿(route_bundle_template)을 프로젝트별로 로드 — 신규설계 활용(L4).
                // 프로젝트마다 source_file 이 다르므로 매 로드 시 갱신(트렁크 고도는 그 프로젝트 좌표계).
                _bundles = await Task.Run(() => BundleStore.TryLoad(_dbConfig));
                OnChanged(nameof(BundleStatus));
                // 기존설계 학습 특징 프로필 (route_feature_group_profile) 일괄 로드
                try
                {
                    _featureProfiles = await Task.Run(() => ObstacleDbLoader.LoadFeatureProfiles(_dbConfig, sd.SourceFile));
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[경고] route_feature_group_profile 로딩 에러: {ex.Message}");
                    _featureProfiles = new Dictionary<string, FeatureProfileRow>(StringComparer.OrdinalIgnoreCase);
                }
                ResetEngine();
                var g = sd.Grid;
                _engine!.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
                _engine.SetParams(g.CellMm, 500, 10, 2, 6);   // 기본 비용함수 파라미터.
                foreach (var o in sd.Obstacles)
                    if (o.IsPassThrough)   // 통과 객체(바닥/천장/격자보): 점유맵엔 넣되 A* 충돌엔 제외.
                        _engine.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                    else
                        _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                // 작업도 적재하되(점유맵/단일 라우팅 일관성) 자동 라우팅은 하지 않는다.
                foreach (var t in sd.Tasks)
                    _engine.AddTask(t.Sx, t.Sy, t.Sz, t.Gx, t.Gy, t.Gz, t.Utility, t.Group);
                BuildTaskRows();   // 행/필터/탐색 대상 목록 구성(경로 캐시는 비어 있음).

                BuildModel();      // 장애물만 렌더 + SceneRebuilt → ZoomExtents(전체보기).
                Status = $"장애물 {sd.Obstacles.Count} · 작업 {sd.Tasks.Count} · 격자 {g.Nx}×{g.Ny}×{g.Nz} cell={g.CellMm:0}mm   |   범위를 고르고 '▶ 경로 탐색 실행'을 누르세요";
            }
            catch (Exception ex) { Status = "DB 로드 오류: " + ex.Message; }
        }

        /// <summary>내장 데모(골든03): 120x120x60, 바닥 슬래브, 같은 통로 5개 배관.</summary>
        private void LoadDemo()
        {
            var sc = new SceneData
            {
                Grid = new GridMeta { CellMm = 50, Ox = 0, Oy = 0, Oz = 0, Nx = 120, Ny = 120, Nz = 60 }
            };
            sc.Obstacles.Add(new ObstacleBox { MinX = 0, MinY = 0, MinZ = 0, MaxX = 6000, MaxY = 6000, MaxZ = 250 });
            var utils = new (string u, string g)[]
            {
                ("UPW_S", "UPW"), ("NFW", "Waste Liquid"), ("PA", "Gas"), ("NW", "Water"), ("ACID", "Exhaust")
            };
            foreach (var (u, g) in utils)
                sc.Tasks.Add(new TaskInfo { Sx = 275, Sy = 3025, Sz = 1525, Gx = 5725, Gy = 3025, Gz = 1525, Utility = u, Group = g });
            _scene = sc;

            ResetEngine();
            var grid = sc.Grid;
            _engine!.SetGrid(grid.CellMm, grid.Ox, grid.Oy, grid.Oz, grid.Nx, grid.Ny, grid.Nz);
            _engine.SetParams(50, 500, 10, 2, 6);
            foreach (var o in sc.Obstacles)
                _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            foreach (var t in sc.Tasks)
                _engine.AddTask(t.Sx, t.Sy, t.Sz, t.Gx, t.Gy, t.Gz, t.Utility, t.Group);

            BuildTaskRows();
            // 데모는 가벼우므로(5개 배관) 동기로 '모두' 라우팅해 바로 경로를 보여준다.
            try
            {
                _engine.RouteMulti(_priority);
                CacheResultsByIndex();
                BuildModel();
            }
            catch (Exception ex) { Status = "경로 탐색 오류: " + ex.Message; }
        }

        private void ResetEngine()
        {
            _engine?.Dispose();
            _engine = new Engine();
        }

        // ---- 작업 목록 ----
        private void BuildTaskRows()
        {
            Tasks.Clear();
            var scene = _scene!;
            var colorMap = UtilityColors.Assign(scene.Tasks.Select(t => t.UtilityLabel));
            for (int i = 0; i < scene.Tasks.Count; i++)
            {
                var t = scene.Tasks[i];
                var color = colorMap.TryGetValue(t.UtilityLabel, out var c) ? c : Colors.Gray;
                Tasks.Add(new TaskRowVM
                {
                    Index = i, Label = t.UtilityLabel, Swatch = new SolidColorBrush(color),
                    Utility = t.Utility, Group = t.Group,
                    PocName = t.PocName, EndName = t.EndName,
                    RoutePathGuid = t.RoutePathGuid,
                    Sx = t.Sx, Sy = t.Sy, Sz = t.Sz, Gx = t.Gx, Gy = t.Gy, Gz = t.Gz
                });
            }
            BuildUtilityFilters(colorMap);
            RebuildRouteTargets();
            RebuildGroupList();             // 좌측 드릴다운 1단계(그룹) 채우기.
            SearchModel = null;             // 이전 단계별 탐색 오버레이 제거.
            SelectedTask = null;            // 선택은 드릴다운(③ 개별 PoC)에서 명시적으로.
            OnChanged(nameof(TaskCountText));
            TasksView.Refresh();
        }

        // 선택 범위(그룹별/유틸별)에 따라 대상 콤보 목록을 작업 분포에서 새로 만든다.
        private void RebuildRouteTargets()
        {
            RouteTargets.Clear();
            if (_selectedRouteScope != null && Tasks.Count > 0)
            {
                IEnumerable<string> keys = _selectedRouteScope.Scope switch
                {
                    RouteScope.ByGroup => Tasks.Select(t => GroupKey(t.Group)),
                    RouteScope.ByUtility => Tasks.Select(t => UtilityKey(t.Utility)),
                    _ => Enumerable.Empty<string>(),
                };
                foreach (var k in keys.Distinct().OrderBy(s => s, StringComparer.Ordinal))
                    RouteTargets.Add(k);
            }
            SelectedRouteTarget = RouteTargets.FirstOrDefault();
            OnChanged(nameof(IsTargetSelectable));
        }

        private static string GroupKey(string? g) => string.IsNullOrEmpty(g) ? "?" : g;
        private static string UtilityKey(string? u) => string.IsNullOrEmpty(u) ? "?" : u;

        // 유틸리티별 필터 행을 작업 라벨 분포에서 새로 만든다(기존 항목은 PropertyChanged 해제).
        private void BuildUtilityFilters(Dictionary<string, Color> colorMap)
        {
            _suppressFilterRebuild = true;
            foreach (var f in UtilityFilters) f.PropertyChanged -= OnUtilityFilterChanged;
            UtilityFilters.Clear();
            var groups = Tasks.GroupBy(t => t.Label).OrderBy(g => g.Key);
            foreach (var g in groups)
            {
                var color = colorMap.TryGetValue(g.Key, out var c) ? c : Colors.Gray;
                var f = new UtilityFilterVM
                {
                    Label = g.Key,
                    Swatch = new SolidColorBrush(color),
                    Count = g.Count(),
                };
                f.PropertyChanged += OnUtilityFilterChanged;
                UtilityFilters.Add(f);
            }
            _suppressFilterRebuild = false;
        }

        // ---- 경로 탐색(범위 선택) ----
        // 모든 작업 행 위치(0..N-1).
        private List<int> AllRows() => Enumerable.Range(0, Tasks.Count).ToList();

        /// <summary>'▶ 경로 탐색 실행' — 선택된 범위(모두/그룹별/유틸별)에 해당하는 작업만 라우팅한다.
        /// 그룹별/유틸별은 선택한 대상 1개의 작업들만 부분집합으로 충돌회피 라우팅한다.</summary>
        private async Task RunRouteAsync()
        {
            if (_scene == null || _selectedRouteScope == null) return;
            List<int> rows;
            string label;
            switch (_selectedRouteScope.Scope)
            {
                case RouteScope.ByGroup:
                    if (string.IsNullOrEmpty(SelectedRouteTarget)) { Status = "라우팅할 그룹을 선택하세요"; return; }
                    rows = RowsWhere(t => GroupKey(t.Group) == SelectedRouteTarget);
                    label = $"그룹 '{SelectedRouteTarget}'";
                    break;
                case RouteScope.ByUtility:
                    if (string.IsNullOrEmpty(SelectedRouteTarget)) { Status = "라우팅할 유틸리티를 선택하세요"; return; }
                    rows = RowsWhere(t => UtilityKey(t.Utility) == SelectedRouteTarget);
                    label = $"유틸리티 '{SelectedRouteTarget}'";
                    break;
                default:
                    rows = AllRows();
                    label = "모두";
                    break;
            }
            if (rows.Count == 0) { Status = "대상 작업이 없습니다"; return; }
            // 최단경로 경고 — 스텁/패턴 가이드 없이 PoC→PoC 전체를 A* 탐색하므로 미세 격자(25mm)에서
            // 배관당 탐색이 수십만~수백만 노드로 폭발한다. 실측(WTNHJ02 Exhaust 20개, cell=100): 최단경로는
            // 스텁 모드 대비 길이 4.7×(318,000 vs 67,800mm)·시간 198×(28.5s vs 0.14s)이고 25mm 에서는 그룹당
            // 수 분~10분+ 소요. '최단'이라는 이름과 달리 밀집 플랜트에서는 가장 길고 느린 결과가 나온다 —
            // 짧고 빠른 결과는 '특징점 반영' 또는 '기존설계 추종' 모드(사람설계 랙 구조 활용)가 정답.
            if (_routingMode == RoutingMode.Shortest && rows.Count > 3)
                Status = _scene?.Grid.CellMm < 50.0
                    ? $"⚠ 최단경로 + {_scene.Grid.CellMm:0}mm — 탐색 폭발로 그룹당 수 분~10분+ 소요·결과도 가장 긺(실측 스텁모드 대비 4.7×길이). '특징점 반영'/'기존설계 추종' 강력 권장."
                    : "⚠ 최단경로 — 밀집 배관에서 길이·시간 모두 불리(실측 스텁모드 대비 4.7×길이·198×시간). '특징점 반영'/'기존설계 추종' 권장.";
            // 경로 방식에 따라 corridor(그룹패턴 회랑) 여부 결정.
            // ─ PatternApplied : 특징점(스텁+패턴)+그룹 회랑 → corridor=true
            // ─ Shortest       : 순수 최단 → corridor=false
            // ─ FollowExisting : 기존배관 복제(_useDesignReplicate가 RouteRowsAsync 내 후처리) → corridor=false
            bool useGroupCorridor = _routingMode == RoutingMode.PatternApplied;
            await RouteRowsAsync(rows, label, corridor: useGroupCorridor, showProgress: true);
        }

        private List<int> RowsWhere(Func<TaskRowVM, bool> pred)
        {
            var list = new List<int>();
            for (int i = 0; i < Tasks.Count; i++) if (pred(Tasks[i])) list.Add(i);
            return list;
        }

        // '기존설계 유사' 랙 레벨 — 작업 종단점 z 의 셀 레벨 분포에서 가장 흔한 레벨(최대 3개)을
        // 선호 단으로 삼아 배관을 공용 높이로 모은다(빈 배열이면 회랑 인력만으로도 번들링됨).
        private int[] ComputeRackLevels()
        {
            if (_scene == null) return System.Array.Empty<int>();
            double oz = _scene.Grid.Oz, cell = _scene.Grid.CellMm;
            if (cell <= 0) return System.Array.Empty<int>();
            var counts = new Dictionary<int, int>();
            void Bump(double z) { int k = (int)Math.Floor((z - oz) / cell); counts[k] = counts.TryGetValue(k, out var v) ? v + 1 : 1; }
            foreach (var t in Tasks) { Bump(t.Sz); Bump(t.Gz); }
            return counts.OrderByDescending(kv => kv.Value).Take(3).Select(kv => kv.Key).ToArray();
        }

        // 엔진을 [장애물 전체 + 지정 행들의 작업]만으로 재구성한다(부분집합 충돌회피 라우팅용).
        // 반환: 적재한 행 위치 목록(순서 = 엔진 작업 인덱스). 종단점은 행에서 직접 읽는다(편집 반영).
        // groupMode=true(그룹배관 라우팅) — 매칭 기존배관 회랑(L2b)을 강제 ON 취급하고 w_corridor 를 강하게
        //   줘서, 같은 유틸 배관이 공용 트렁크에 뭉치도록 한다(끝단 스텁은 기존대로). priority "utility" 와 함께 쓴다.
        private List<int> BuildEngineForRows(IReadOnlyList<int> rowPositions, bool groupMode = false)
        {
            var scene = _scene!;
            ResetEngine();
            var g = scene.Grid;
            _engine!.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
            // 클리어런스 항상 활성. 대형 격자(25mm/10mm)는 네이티브가 ImplicitOccupancy(복셀화 없는
            // 박스 색인) + '온디맨드 클리어런스'(박스 최근접 질의)로 전환 → 전역 거리맵(배관당 size×4B
            // ~520MB BFS) 없이 저렴하게 계산. 따라서 더는 대형 격자에서 클리어런스를 끄지 않는다(품질 유지).
            // 대형 격자는 weighted A*(w_heur=1.5) — 솔리드 설비/덕트를 우회하는 어려운 경로를 탐색상한(12M)
            // 내에 찾도록 목표 지향 탐색(약간 비최적 허용). 작은 격자(데모 등)는 표준 A*(1.0, 최적).
            // weighted A* — 실데이터처럼 솔리드 설비/덕트가 많은 혼잡 격자는 목표지향 탐색(w_heur=2.0)으로
            // 어려운 우회 경로를 탐색상한(12M) 내에 빠르게 찾는다(약간 비최적 허용). 측정(project6 c100,
            // 1.39M셀·208작업): w_heur 1.5→2.0 으로 회랑 OFF 194→199, 회랑 ON 47s→6.3s(약 7.5× 단축, +1).
            // 임계를 5M→300k 로 낮춰 실데이터 격자가 weighted A* 를 받게 한다(이전엔 1.39M<5M 라 1.0 였음).
            // 작은 데모 격자(<300k셀)는 표준 A*(1.0, 최적). 골든 정확도는 C++ ctest 가 별도 검증(이 경로 무관).
            bool weighted = (long)g.Nx * g.Ny * g.Nz > 300_000;
            double wHeur = weighted ? 2.0 : 1.0;
            // 최단경로 모드: 가중을 낮춰(2.0→1.4) 비최적(뱀형) 경로를 줄여 '최단'에 가깝게 한다.
            //   w_heur=2.0 은 ε≤2 까지 비최적 + 스텁/회랑 가이드가 없어 대부분 배관이 계층(hier) corridor 로
            //   escalate 되며 3~4× 우회·느림이 생겼다(사용자 지적). 1.4 면 ε≤1.4(직접 A* 경로가 더 짧음).
            //   대신 탐색량이 늘어 어려운 배관은 더 오래 걸리거나(아래 hier_probe 상향) 상한에서 실패할 수 있다.
            //   env R3D_SHORTEST_WHEUR 로 재정의(예: 1.0=진짜 최단·매우 느림, 2.0=옛 동작).
            if (_routingMode == RoutingMode.Shortest && weighted)
            {
                wHeur = 1.4;
                var sw = System.Environment.GetEnvironmentVariable("R3D_SHORTEST_WHEUR");
                if (!string.IsNullOrEmpty(sw) &&
                    double.TryParse(sw, System.Globalization.NumberStyles.Float,
                                    System.Globalization.CultureInfo.InvariantCulture, out var vw) && vw >= 1.0)
                    wHeur = vw;
            }
            // 동적(수렴) 가중 A* — 목표까지 거리비로 가중을 보간한다(먼 곳=wHeur 공격적·빠름, 목표 근처=
            // wHeurNear 신중·정확). 목표/PoC 근처의 혼잡·막다른길에서 순수 그리디(w=2.0) 함정을 피해 마지막
            // 접근 경로를 찾아낸다. 준최적 상한은 여전히 wHeur. 측정(project6 c100): 스텁ON 206→208(완전),
            // 스텁OFF 199→203, 시간 동일. 엔진이 거대격자(예산-게이트 hier)에선 자동 비활성 → cell≤25 무영향.
            // 기본 = wHeur 가중일 때 1.0(=목표서 표준 A*). env R3D_WHEUR_NEAR 로 재정의(0=정적으로 끔).
            double wHeurNear = weighted ? 1.0 : 0.0;
            {
                var sNear = System.Environment.GetEnvironmentVariable("R3D_WHEUR_NEAR");
                if (!string.IsNullOrEmpty(sNear) &&
                    double.TryParse(sNear, System.Globalization.NumberStyles.Float,
                                    System.Globalization.CultureInfo.InvariantCulture, out var vNear) && vNear >= 0.0)
                    wHeurNear = vNear;
            }
            // 기존설계 회랑(L2b)/랙 번들링(L3a) ON 이면 회랑·랙 밖 셀당 가산(=½칸 비용)으로 부드럽게 유도.
            // w_heur=2.0 이 회랑 비용장을 휴리스틱에 반영해 탐색 폭을 억제 → OFF 와 거의 동일 속도로 동작
            // (이전 ~47s 폭증의 원인 해소). corridor_radius=2 는 회랑 튜브 폭(±2셀).
            // 랙 번들링(L3a): 학습된 그룹 랙 z-셀을 rack_levels(면제)로 주면 같은 그룹 배관이 공용 랙에 뭉친다.
            // 랙 페널티는 회랑(0.5)보다 부드러운 0.2(실측: project6 c100 200/208·랙 집중도 17→21%·길이↓.
            // 0.5 는 한 배관을 막아 198 로 떨어짐). 회랑+랙 동시 ON 이면 회랑의 0.5 가 우선(강한 설계추종).
            int[]? rackLevels = _useRackBundling ? BuildRackLevels(rowPositions) : null;
            // 그룹배관 패턴(L4): DB 에 저장된 유틸별 대표 트렁크 고도를 rack_levels 에 합친다(공용 랙 높이에 뭉침).
            if (_useBundlePattern && _bundles != null)
                rackLevels = MergeBundleLevels(rackLevels, rowPositions);
            
            // 신규 데이터베이스 학습된 Z고도(route_feature_group_profile) 추가 머지
            if (_forcedRackZ == null && _useBundlePattern)
                rackLevels = MergeFeatureProfilesRackLevels(rackLevels, rowPositions);
            bool hasRack = rackLevels != null && rackLevels.Length > 0;
            // 번들 공용 트렁크 회랑(L4) — 같은 유틸 기존배관 전체를 '하나의 공용 트렁크 회랑'으로 주입해, 새 배관
            // 들이 흩어지지 않고 한 스파인에 모이게 한다(높이만 유도하던 rack_levels 의 한계 보완: xy 트렁크 제공).
            // test_attract 가 증명한 메커니즘 — w_corridor>0 + 공유 회랑 셀이면 둘째 배관이 첫 배관 곁으로 뭉친다.
            // includeVertical:true — 수평 트렁크 레인 + 번들 멤버의 '수직 입상'까지 회랑으로 주입(v3).
            //   수직(입상) 번들도 신규 라우팅이 학습된 입상 레인을 따라가게 한다(수직 번들 라우팅 활용).
            int[] bundleCorr = (_useBundlePattern && _bundles != null)
                ? BuildBundleCorridorCells(rowPositions, 2, includeVertical: true) : System.Array.Empty<int>();
            bool hasBundleCorr = bundleCorr.Length > 0;
            
            // 신규 공용 척추선(Spine) 기반 회랑 셀 생성 및 주입 (dilate=2, 약 ±2셀 범위 팽창)
            int[] featureSpineCorr = (_forcedRackZ == null && _useBundlePattern)
                ? BuildFeatureCenterlineCorridorCells(rowPositions, 2) : System.Array.Empty<int>();
            bool hasFeatureSpine = featureSpineCorr.Length > 0;
            // 회랑(0.5)은 랙(0.2)보다 강한 설계추종. L2b 또는 번들 트렁크 회랑이 있으면 0.5, 랙만이면 0.2.
            // 그룹 모드도 회랑 0.5(설계추종) — self-bundling(mark_pipe+add_corridor_cells)은 wCorr>0 이면
            //   작동하므로 0.5 로 충분하다. 과거 cell*2.0 은 회랑 밖 셀당 +2칸 페널티 비용장을 weighted A*
            //   휴리스틱(직선거리)이 과소평가해 탐색이 Dijkstra 처럼 폭발 → 혼잡 프로젝트(예: CMP_KSCTA08,
            //   평범한 cell=100 plain 에서도 실패 7건이 확장 11.27M 으로 12M 상한 근접)에서 첫 어려운 배관이
            //   탐색상한(12M) 초과로 실패했다. 0.5 는 동일 다발링·정상속도(ALKA 14/14: 2.0=15.2s→0.5=1.3s).
            double wCorr = (groupMode || _useDesignCorridor || hasBundleCorr || hasFeatureSpine) ? g.CellMm * 0.5
                         : (_useRackBundling || (_useBundlePattern && hasRack)) ? g.CellMm * 0.2 : 0.0;
            _engine.SetParams(g.CellMm, 500, 10, 2, 6, wCorridor: wCorr, corridorRadius: 2,
                              rackLevels: rackLevels, wHeur: wHeur, wHeurNear: wHeurNear);
            bool useSegmentAstar = !string.Equals(Environment.GetEnvironmentVariable("R3D_SEGMENT_ASTAR"), "off", StringComparison.OrdinalIgnoreCase)
                                && !string.Equals(Environment.GetEnvironmentVariable("R3D_SEGMENT_ASTAR"), "0", StringComparison.OrdinalIgnoreCase);
            int segmentMax = 64;
            if (int.TryParse(Environment.GetEnvironmentVariable("R3D_SEGMENT_MAX"), out var sm) && sm > 0) segmentMax = sm;
            _engine.SetSegmentAstar(useSegmentAstar, segmentMax);
            bool useOctreeGuide = string.Equals(Environment.GetEnvironmentVariable("R3D_OCTREE_GUIDE"), "on", StringComparison.OrdinalIgnoreCase)
                               || string.Equals(Environment.GetEnvironmentVariable("R3D_OCTREE_GUIDE"), "1", StringComparison.OrdinalIgnoreCase);
            int octreeCorrRadius = 2;
            if (int.TryParse(Environment.GetEnvironmentVariable("R3D_OCTREE_CORR_RAD"), out var ogRad) && ogRad >= 0) octreeCorrRadius = ogRad;
            _engine.SetOctreeGuide(useOctreeGuide, octreeCorrRadius);
            // 배관-배관 물리 이격 — per-task 관경 반경 활성 + 표면 최소 60mm 갭.
            // per-task: route_multi 가 각 배관 diameter_mm 로 마킹 반경을 자동 산출해 가는 배관 과패킹 해소.
            // gap60: 두 배관 센터선 거리 ≥ r1+r2+60mm 를 쌍별 마킹으로 강제(표면 맞닿음→분리).
            _engine.SetPerTaskRadius(true);
            _engine.SetPipeGap(60.0);
            // C2 코너 최소반경(2×관경): 모든 라우팅·rip-up 후 최종 패스에서 엘보 간 직선 < 2×관경인
            // 짧은 단관을 충돌검사 하에 흡수(꺾임 비증가·양 끝점 고정). '비교란' 직선화 — 중간 직선화는
            // 다운스트림 점유 교란으로 꺾임이 오히려 늘었기 때문에 최종 일괄 패스로만 적용.
            _engine.SetMinStraight(2.0);
            // 코너 최소직선(하드 제약, 절대 300mm) — A* 가 '한 번 꺾인 뒤 300mm 직진 전엔 다시 꺾지 못하도록'
            // 탐색 단계에서 강제한다(엘보 간 짧은 단관/계단 꺾임 방지, 관경 무관·전 배관, 목표 직전 접속
            // 구간만 면제). SetMinStraight(관경 배수·후처리 흡수)와 달리 탐색 하드 보장이라 계단현상이 근본
            // 차단된다. 0=OFF(골든 불변). env R3D_MIN_STRAIGHT_MM 으로 재정의(0=끔).
            _engine.SetMinStraightMm(MinStraightMmForRouting());
            // C1 CBS(negotiated-congestion) — 평면 rip-up 후 잔여 실패 배관에 대해 blocker-of-blocker 까지
            // 재귀 양보(depth 2 = 최대 2단계 연쇄). 소·대형 격자 모두 적용. 기본 OFF(체크박스로 옵트인).
            _engine.SetCbsDepth(_useCbs ? 2 : 0);
            // 최단경로 모드 + 대형 격자 탐색 폭발 방지(탐색 상한 8M).
            // 스텁 없이 PoC→PoC 전체를 탐색하므로 기본 48M 이면 배관당 수분 소요. 8M 으로 낮춰 찾을 수 있는
            // 경로는 빨리 성공, 너무 복잡한 배관은 빨리 실패(UI 반응성). 계층(hier) escalation 은 기본(probe
            // 30만) 그대로 둔다 — 어려운 배관을 빠르게 계층 corridor 로 넘겨 '동결'을 막는다(직접 A* 로
            // 8M 까지 끝까지 탐색하면 배관당 수분→그룹 10분+ 동결. 실측 cell=100 에서도 12/20 배관이 30만~110만
            // 확장). 계층 corridor 는 경로가 길어지지만(연결 우선) 시간은 보장된다.
            // ⚠ 최단경로의 본질적 한계: cell=100 실측 결과 스텁 모드(67,800mm·144ms) 대비 4.7×길이·198×시간
            //   (318,000mm·28.5s). 짧고 빠른 결과는 '특징점 반영'/'기존설계 추종' 모드가 정답(§ RunRouteAsync 경고).
            if (_routingMode == RoutingMode.Shortest && (long)g.Nx * g.Ny * g.Nz > 5_000_000)
                _engine.SetMaxExpansions(ShortestMaxExpansions());
            foreach (var o in scene.Obstacles)
                if (o.IsPassThrough)   // 통과 객체: 점유맵엔 넣되 A* 충돌엔 제외.
                    _engine.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                else
                    _engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            // 충돌확장: 설비·덕트·레터럴 + 이미 설계된(라우팅된) 다른 배관을 장애물로 추가(자기 자신 제외).
            AddFacilityObstacles(_engine, new HashSet<int>(rowPositions));
            var added = new List<int>(rowPositions.Count);
            foreach (var pos in rowPositions)
            {
                var row = Tasks[pos];
                row.StartStub = null; row.EndStub = null;
                double sx, sy, sz, gx, gy, gz;

                // 스텁 라우팅: 매칭 기존배관의 출발/종단 스텁(수직+엘보)을 고정 설계 구간으로 깔고, A* 는 스텁
                // 끝(랙 위 자유공간)에서 시작/종료한다. 그러면 결과가 학습 스텁을 따른다(PoC 재탐색 문제 해소).
                var stubPipe = _useStubRouting ? FindMatchingExistingPipe(row) : null;
                if (stubPipe != null)
                {
                    var (srcStub, tgtStub) = StubExtractor.ForPipe(stubPipe);   // 배관 source/target 쪽 스텁.
                    // 방향 정합 — 작업 start 가 배관 source 에 가까우면 정방향, 아니면 역방향(스텁 스왑).
                    Pt3 ps = stubPipe.SourcePos ?? stubPipe.Points[0];
                    Pt3 pe = stubPipe.TargetPos ?? stubPipe.Points[stubPipe.Points.Count - 1];
                    var ts = new Pt3(row.Sx, row.Sy, row.Sz); var te = new Pt3(row.Gx, row.Gy, row.Gz);
                    bool forward = Dist(ts, ps) + Dist(te, pe) <= Dist(ts, pe) + Dist(te, ps);
                    var startStub = forward ? srcStub : tgtStub;
                    var endStub = forward ? tgtStub : srcStub;
                    if (startStub.Count >= 2 && endStub.Count >= 2)
                    {
                        // 표시 경로가 실제 작업 PoC 에서 시작/끝나도록 스텁 양 끝점을 작업 PoC 로 고정.
                        startStub[0] = new Pt3(row.Sx, row.Sy, row.Sz);
                        endStub[0] = new Pt3(row.Gx, row.Gy, row.Gz);
                        row.StartStub = startStub; row.EndStub = endStub;
                        var se = startStub[startStub.Count - 1];   // 출발 스텁 끝(랙) = A* 시작.
                        var ee = endStub[endStub.Count - 1];       // 종단 스텁 끝(랙) = A* 목표.
                        (sx, sy, sz) = SnapPocToFreeCell(se.X, se.Y, se.Z, null);
                        (gx, gy, gz) = SnapPocToFreeCell(ee.X, ee.Y, ee.Z, null);
                        int stubTidx = _engine.AddTask(sx, sy, sz, gx, gy, gz, row.Utility, row.Group);
                        // 관경 전달 — per-task 반경 산출 + 굵은 배관 우선 정렬 키.
                        double stubDia = stubPipe.DiameterMm > 0 ? stubPipe.DiameterMm : row.DiameterMm;
                        if (stubDia > 0) { _engine.SetTaskDiameter(stubTidx, stubDia); if (row.DiameterMm <= 0) row.DiameterMm = stubDia; }

                        // 데이터베이스 학습된 접속면 방향 제약 주입 (GoalDirection)
                        if (_useBundlePattern && _featureProfiles != null && !string.IsNullOrEmpty(row.Group) &&
                            _featureProfiles.TryGetValue(row.Group, out var stubProf))
                        {
                            int gd = ConvertFaceToGoalDir(stubProf.PreferredTargetFace);
                            if (gd >= 0) _engine.SetTaskGoalDir(stubTidx, gd);
                        }
                        added.Add(pos);
                        continue;
                    }
                }

                // 폴백(매칭 배관 없음/스텁 라우팅 OFF): 기존 PoC 직접 라우팅 — 학습 면으로 PoC 를 표면 투영.
                string? startFace = LearnedFace("EQUIP", row.Group, row.Utility);
                string? endFace = LearnedDuctFace(row.Group, row.Utility, row.Gx, row.Gy, row.Gz,
                                                  row.Sx, row.Sy, row.Sz);
                (sx, sy, sz) = DropStartBelowEquipment(row.Sx, row.Sy, row.Sz);
                (sx, sy, sz) = LiftPocToSurface(sx, sy, sz, startFace);
                (sx, sy, sz) = SnapPocToFreeCell(sx, sy, sz, startFace);   // 파묻힌 시작 PoC → 최근접 자유 셀.
                (gx, gy, gz) = LiftPocToSurface(row.Gx, row.Gy, row.Gz, endFace);
                (gx, gy, gz) = SnapPocToFreeCell(gx, gy, gz, endFace);     // 파묻힌 종단 PoC → 최근접 자유 셀.
                int tidx = _engine.AddTask(sx, sy, sz, gx, gy, gz, row.Utility, row.Group);
                // 관경 전달 — 캐시 없으면 매칭 기존배관에서 1회 조회 후 캐시.
                if (row.DiameterMm <= 0) { var exm = FindMatchingExistingPipe(row); if (exm != null && exm.DiameterMm > 0) row.DiameterMm = exm.DiameterMm; }
                if (row.DiameterMm > 0) _engine.SetTaskDiameter(tidx, row.DiameterMm);

                // 데이터베이스 학습된 접속면 방향 제약 주입 (GoalDirection)
                if (_useBundlePattern && _featureProfiles != null && !string.IsNullOrEmpty(row.Group) &&
                    _featureProfiles.TryGetValue(row.Group, out var fallbackProf))
                {
                    int gd = ConvertFaceToGoalDir(fallbackProf.PreferredTargetFace);
                    if (gd >= 0) _engine.SetTaskGoalDir(tidx, gd);
                }

                added.Add(pos);
            }
            // 회랑 셀 주입(w_corridor>0 일 때 효력): L2b(배관별 매칭) + 번들 공용 트렁크(L4) 합집합.
            //   그룹 모드면 L2b(매칭 기존배관 추종)를 강제 ON — 그룹 패턴(L4) 트렁크 좌표와 합쳐 자유공간을 가이드.
            int[]? l2bCells = (_useDesignCorridor || groupMode) ? BuildDesignCorridorCells(rowPositions, 2) : null;
            _engine.SetCorridorCells(CombineCorridor(l2bCells, CombineCorridor(bundleCorr, featureSpineCorr)));
            return added;
        }

        // 지정 행들의 매칭 기존 설계배관(TB_ROUTE_PATH) 폴리라인을 격자 셀로 복셀화(±dilate 팽창)해
        // 회랑 시드(ijk 평탄 배열)로 만든다. 새 경로가 이 회랑 안을 '싸게' 지나 사람 설계를 따라가게 한다.
        private int[] BuildDesignCorridorCells(IReadOnlyList<int> rowPositions, int dilate)
        {
            var s = _scene!; var g = s.Grid; double cell = g.CellMm;
            var set = new HashSet<(int, int, int)>();
            foreach (var pos in rowPositions)
            {
                var pipe = FindMatchingExistingPipe(Tasks[pos]);
                if (pipe == null || pipe.Points.Count < 2) continue;
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                    double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                    int steps = Math.Max(1, (int)(len / (cell * 0.5)));
                    for (int sIdx = 0; sIdx <= steps; sIdx++)
                    {
                        double tt = (double)sIdx / steps;
                        int ci = (int)Math.Floor((a.X + dx * tt - g.Ox) / cell);
                        int cj = (int)Math.Floor((a.Y + dy * tt - g.Oy) / cell);
                        int ck = (int)Math.Floor((a.Z + dz * tt - g.Oz) / cell);
                        for (int di = -dilate; di <= dilate; di++)
                            for (int dj = -dilate; dj <= dilate; dj++)
                                for (int dk = -dilate; dk <= dilate; dk++)
                                {
                                    int ii = ci + di, jj = cj + dj, kk = ck + dk;
                                    if (ii < 0 || jj < 0 || kk < 0 || ii >= g.Nx || jj >= g.Ny || kk >= g.Nz) continue;
                                    set.Add((ii, jj, kk));
                                }
                    }
                }
            }
            var arr = new int[set.Count * 3];
            int n = 0;
            foreach (var (i, j, k) in set) { arr[n++] = i; arr[n++] = j; arr[n++] = k; }
            return arr;
        }

        // 번들 회랑 + 패턴 표시 셀(L4, v3 멤버십 기준) — 탐지된 번들(route_bundle_group)의 '멤버 배관'의
        //   모든 런(수평·수직)을 타이트(±1셀) 레인 셀로 만든다. 표시(ShowBundlePattern)와 라우팅 회랑이 같은
        //   셀집합을 공유 → 보이는 패턴이 곧 신규경로의 경유지. trunk_axis 라벨·trunk_z 밴드와 무관하므로
        //   수평 그룹배관·수직(입상) 그룹배관·ㄴ자(둘 다) 번들이 모두 경유지/표시에 포함된다. 비멤버는 제외
        //   (단독 배관 가짜양성 제거). route_multi 충돌회피(mark_pipe)가 새 배관을 인접 레인에 분산 → 등간격 패킹.
        //   주의: 격자 셀 > pitch 면 인접 레인이 같은 셀로 뭉개진다(cell=100>pitch≈56) → 셀 ≤ pitch/2 권장.
        // 번들 미적재(GroupCount=0) 시에만 옛 트렁크 z밴드 휴리스틱으로 폴백(includeVertical 은 이 폴백에서만 의미).
        private int[] BuildBundleCorridorCells(IReadOnlyList<int> rowPositions, int dilate, bool includeVertical = false)
        {
            var s = _scene; if (s == null || s.ExistingPipes.Count == 0) return System.Array.Empty<int>();
            var g = s.Grid; double cell = g.CellMm, oz = g.Oz;

            var utils = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
                if (!string.IsNullOrEmpty(Tasks[pos].Utility)) utils.Add(Tasks[pos].Utility!);
            if (utils.Count == 0) return System.Array.Empty<int>();

            // 유틸별 트렁크 고도(z-셀) 집합 — 이 밴드의 수평 런만 레인으로 채택.
            var trunkZ = new HashSet<int>();
            if (_bundles != null)
                foreach (var u in utils)
                {
                    var t = _bundles.TryGet(null, u); if (t == null) continue;
                    foreach (var z in t.TrunkZs)
                    {
                        int zk = (int)Math.Floor((z - oz) / cell);
                        if (zk >= 0 && zk < g.Nz) trunkZ.Add(zk);
                    }
                }
            // 멤버십 기준(member-driven, v3) — 탐지된 번들(route_bundle_group)의 '멤버 배관'이면 그 배관의
            //   '모든 런(수평·수직)'을 trunk_axis 라벨·trunk_z 밴드와 무관하게 회랑/표시로 채택한다. 비멤버는
            //   제외(단독 배관이 그룹으로 보이던 가짜양성 제거). → 수평 그룹배관과 수직(입상) 그룹배관이 모두
            //   신규경로의 경유지가 된다. ㄴ자(수평+수직) 번들도 trunk_axis 가 z 로 라벨돼도 수평부까지 포함.
            // 번들 미적재(GroupCount=0) 시에만 옛 트렁크 z밴드 휴리스틱으로 폴백(무해).
            bool memberAware = _bundles != null && _bundles.GroupCount > 0;
            bool laneMode = memberAware || trunkZ.Count > 0;
            const double HorizTol = 0.34, MinRunMm = 800.0;
            const int LaneDilate = 1;                  // 레인 두께 ±1셀(타이트). 폴백 밴드 ±1셀은 BandCells.
            const int BandCells = 1;

            var set = new HashSet<(int, int, int)>();
            foreach (var pipe in s.ExistingPipes)
            {
                if (pipe.Utility == null || !utils.Contains(pipe.Utility) || pipe.Points.Count < 2) continue;
                bool isMember = memberAware && pipe.RoutePathGuid != null
                                && _bundles!.GroupIdOf(pipe.RoutePathGuid) >= 0;
                // 멤버십 모드에서 비멤버 배관은 통째 건너뛴다(검출된 다발만 경유지/표시).
                if (memberAware && !isMember) continue;
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                    double horiz = Math.Sqrt(dx * dx + dy * dy);
                    double len = Math.Sqrt(horiz * horiz + dz * dz);
                    bool vertical = horiz <= 1e-6 || Math.Abs(dz) > HorizTol * horiz;
                    int dl = dilate;
                    if (memberAware)
                    {
                        // 멤버 배관의 모든 런(수평·수직) 채택 — 짧은 지터만 제외(수직은 더 짧은 것까지).
                        if (len < (vertical ? MinRunMm * 0.3 : MinRunMm)) continue;
                        dl = LaneDilate;
                    }
                    else if (laneMode)
                    {
                        // 폴백(번들 미적재) — 옛 트렁크 z밴드 휴리스틱.
                        if (vertical)
                        {
                            if (!includeVertical || len < MinRunMm * 0.3) continue;
                            int zk0 = (int)Math.Floor((Math.Min(a.Z, b.Z) - oz) / cell);
                            int zk1 = (int)Math.Floor((Math.Max(a.Z, b.Z) - oz) / cell);
                            bool touches = false;
                            foreach (var tz in trunkZ) if (zk1 >= tz - BandCells && zk0 <= tz + BandCells) { touches = true; break; }
                            if (!touches) continue;
                        }
                        else
                        {
                            if (len < MinRunMm) continue;
                            int zk = (int)Math.Floor(((a.Z + b.Z) / 2 - oz) / cell);
                            bool nearTrunk = false;
                            foreach (var tz in trunkZ) if (Math.Abs(zk - tz) <= BandCells) { nearTrunk = true; break; }
                            if (!nearTrunk) continue;
                        }
                        dl = LaneDilate;
                    }
                    int steps = Math.Max(1, (int)(len / (cell * 0.5)));
                    for (int sIdx = 0; sIdx <= steps; sIdx++)
                    {
                        double tt = (double)sIdx / steps;
                        int ci = (int)Math.Floor((a.X + dx * tt - g.Ox) / cell);
                        int cj = (int)Math.Floor((a.Y + dy * tt - g.Oy) / cell);
                        int ck = (int)Math.Floor((a.Z + dz * tt - g.Oz) / cell);
                        for (int di = -dl; di <= dl; di++)
                            for (int dj = -dl; dj <= dl; dj++)
                                for (int dk = -dl; dk <= dl; dk++)
                                {
                                    int ii = ci + di, jj = cj + dj, kk = ck + dk;
                                    if (ii < 0 || jj < 0 || kk < 0 || ii >= g.Nx || jj >= g.Ny || kk >= g.Nz) continue;
                                    set.Add((ii, jj, kk));
                                }
                    }
                }
            }
            var arr = new int[set.Count * 3];
            int n = 0;
            foreach (var (i, j, k) in set) { arr[n++] = i; arr[n++] = j; arr[n++] = k; }
            return arr;
        }

        // 두 회랑 셀 배열(ijk 평탄)을 합친다 — 중복은 엔진이 set 으로 흡수. 둘 다 비면 null(회랑 없음).
        private static int[]? CombineCorridor(int[]? a, int[]? b)
        {
            int la = a?.Length ?? 0, lb = b?.Length ?? 0;
            if (la == 0 && lb == 0) return null;
            if (lb == 0) return a;
            if (la == 0) return b;
            var r = new int[la + lb];
            System.Array.Copy(a!, 0, r, 0, la);
            System.Array.Copy(b!, 0, r, la, lb);
            return r;
        }

        // 신규 데이터베이스 랙 레벨 로드 및 기존 레벨에 머지
        private int[]? MergeFeatureProfilesRackLevels(int[]? currentLevels, IReadOnlyList<int> rowPositions)
        {
            if (_scene == null || _featureProfiles == null || _featureProfiles.Count == 0) return currentLevels;
            double oz = _scene.Grid.Oz, cell = _scene.Grid.CellMm;
            if (cell <= 0) return currentLevels;

            var set = currentLevels != null ? new HashSet<int>(currentLevels) : new HashSet<int>();
            
            // 대상 행들의 유틸리티 그룹 추출
            var groups = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
            {
                if (!string.IsNullOrEmpty(Tasks[pos].Group)) groups.Add(Tasks[pos].Group!);
            }

            foreach (var g in groups)
            {
                if (_featureProfiles.TryGetValue(g, out var prof) && prof.PreferredRackZs != null)
                {
                    foreach (var z in prof.PreferredRackZs)
                    {
                        int zk = (int)Math.Floor((z - oz) / cell);
                        if (zk >= 0 && zk < _scene.Grid.Nz) set.Add(zk);
                    }
                }
            }
            return set.Count > 0 ? set.ToArray() : currentLevels;
        }

        // 신규 공용 척추선(Spine) 복셀화 및 회랑 격자 생성
        private int[] BuildFeatureCenterlineCorridorCells(IReadOnlyList<int> rowPositions, int dilate)
        {
            if (_scene == null || _featureProfiles == null || _featureProfiles.Count == 0) return System.Array.Empty<int>();
            var g = _scene.Grid; double cell = g.CellMm;
            if (cell <= 0) return System.Array.Empty<int>();

            var set = new HashSet<(int, int, int)>();
            
            // 대상 행들의 유틸리티 그룹 추출
            var groups = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
            {
                if (!string.IsNullOrEmpty(Tasks[pos].Group)) groups.Add(Tasks[pos].Group!);
            }

            foreach (var grp in groups)
            {
                if (!_featureProfiles.TryGetValue(grp, out var prof) || string.IsNullOrEmpty(prof.TrunkCenterlineJson)) continue;
                
                try
                {
                    using var doc = System.Text.Json.JsonDocument.Parse(prof.TrunkCenterlineJson);
                    var root = doc.RootElement;
                    if (root.ValueKind == System.Text.Json.JsonValueKind.Array)
                    {
                        var pts = new List<Pt3>();
                        foreach (var item in root.EnumerateArray())
                        {
                            if (item.TryGetProperty("X", out var xProp) && 
                                item.TryGetProperty("Y", out var yProp) && 
                                item.TryGetProperty("Z", out var zProp))
                            {
                                pts.Add(new Pt3(xProp.GetDouble(), yProp.GetDouble(), zProp.GetDouble()));
                            }
                        }
                        
                        for (int i = 1; i < pts.Count; i++)
                        {
                            var a = pts[i - 1]; var b = pts[i];
                            double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                            double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                            int steps = Math.Max(1, (int)(len / (cell * 0.5)));
                            for (int sIdx = 0; sIdx <= steps; sIdx++)
                            {
                                double tt = (double)sIdx / steps;
                                int ci = (int)Math.Floor((a.X + dx * tt - g.Ox) / cell);
                                int cj = (int)Math.Floor((a.Y + dy * tt - g.Oy) / cell);
                                int ck = (int)Math.Floor((a.Z + dz * tt - g.Oz) / cell);
                                
                                for (int di = -dilate; di <= dilate; di++)
                                    for (int dj = -dilate; dj <= dilate; dj++)
                                        for (int dk = -dilate; dk <= dilate; dk++)
                                        {
                                            int ii = ci + di, jj = cj + dj, kk = ck + dk;
                                            if (ii < 0 || jj < 0 || kk < 0 || ii >= g.Nx || jj >= g.Ny || kk >= g.Nz) continue;
                                            set.Add((ii, jj, kk));
                                        }
                            }
                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[경고] TrunkCenterlineJson 파싱 실패: {ex.Message}");
                }
            }

            var arr = new int[set.Count * 3];
            int n = 0;
            foreach (var (i, j, k) in set) { arr[n++] = i; arr[n++] = j; arr[n++] = k; }
            return arr;
        }

        // face 법선 문자열을 C++ 엔진 GoalDirection 축 상수로 매핑
        private static int ConvertFaceToGoalDir(string face)
        {
            if (string.IsNullOrEmpty(face)) return -1;
            return face.ToLowerInvariant().Trim() switch
            {
                "+x" => 0,
                "-x" => 1,
                "+y" => 2,
                "-y" => 3,
                "+z" => 4,
                "-z" => 5,
                _ => -1
            };
        }

        // 그룹배관 강조용 — group_id 별 구분되는 고유 색(황금비 색상환 회전으로 인접 그룹도 또렷이 구분).
        private static Color BundleGroupColor(int gid)
        {
            double h = (gid * 0.61803398875) % 1.0 * 360.0;
            double s = 0.72, v = 0.98;
            double c = v * s, x = c * (1 - Math.Abs((h / 60.0) % 2 - 1)), m = v - c;
            double r, g, b;
            if (h < 60) { r = c; g = x; b = 0; }
            else if (h < 120) { r = x; g = c; b = 0; }
            else if (h < 180) { r = 0; g = c; b = x; }
            else if (h < 240) { r = 0; g = x; b = c; }
            else if (h < 300) { r = x; g = 0; b = c; }
            else { r = c; g = 0; b = x; }
            return Color.FromRgb((byte)((r + m) * 255), (byte)((g + m) * 255), (byte)((b + m) * 255));
        }

        // 유틸그룹 랙 번들링(L3a) — 라우팅할 행들의 그룹에 속한 기존배관 수평 런이 모이는 z-셀(랙 높이)을
        // 학습해 엔진 rack_levels(면제 z-셀, 최대 8)로 만든다. Python pattern_learn.learn_rack_levels 와 동일 로직:
        //   수평(|dz| <= 0.34×수평거리)이고 800mm 이상인 세그먼트의 z-셀에 런 길이를 누적 → 그룹별 지배 랙 선정.
        private int[]? BuildRackLevels(IReadOnlyList<int> rowPositions)
        {
            var s = _scene!; var g = s.Grid; double cell = g.CellMm;
            if (s.ExistingPipes.Count == 0) return null;
            const double MinRunMm = 800.0, HorizTol = 0.34, GroupShare = 0.15;

            // 이번 라우팅에 등장하는 유틸그룹 집합.
            var groups = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
                if (!string.IsNullOrEmpty(Tasks[pos].Group)) groups.Add(Tasks[pos].Group!);
            if (groups.Count == 0) return null;

            // 그룹 → (z셀 → 누적 런 mm). 기존배관 수평 런을 z-셀에 누적.
            var byGroup = new Dictionary<string, Dictionary<int, double>>(StringComparer.OrdinalIgnoreCase);
            foreach (var pipe in s.ExistingPipes)
            {
                if (pipe.Group == null || !groups.Contains(pipe.Group) || pipe.Points.Count < 2) continue;
                if (!byGroup.TryGetValue(pipe.Group, out var zmap))
                    byGroup[pipe.Group] = zmap = new Dictionary<int, double>();
                for (int i = 1; i < pipe.Points.Count; i++)
                {
                    var a = pipe.Points[i - 1]; var b = pipe.Points[i];
                    double horiz = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
                    if (horiz <= 1e-6 || Math.Abs(b.Z - a.Z) > HorizTol * horiz) continue;   // 수직/사선 제외.
                    double len = Math.Sqrt(horiz * horiz + (b.Z - a.Z) * (b.Z - a.Z));
                    if (len < MinRunMm) continue;
                    int zk = (int)Math.Floor(((a.Z + b.Z) / 2 - g.Oz) / cell);
                    if (zk < 0 || zk >= g.Nz) continue;
                    zmap[zk] = (zmap.TryGetValue(zk, out var v) ? v : 0.0) + len;
                }
            }
            if (byGroup.Count == 0) return null;

            // 그룹마다 전체 런의 GroupShare 이상을 차지하는 z-셀을 채택(지배 랙). 전 그룹 합집합 → 런 큰 순 8개.
            var picked = new Dictionary<int, double>();   // z셀 → 누적 런(여러 그룹 합산, 정렬용).
            foreach (var (_, zmap) in byGroup)
            {
                double tot = 0; foreach (var v in zmap.Values) tot += v;
                if (tot <= 0) continue;
                foreach (var (zk, run) in zmap)
                    if (run >= GroupShare * tot)
                        picked[zk] = (picked.TryGetValue(zk, out var p) ? p : 0.0) + run;
            }
            if (picked.Count == 0) return null;
            var top = picked.OrderByDescending(kv => kv.Value).Take(8).Select(kv => kv.Key).ToArray();
            return top.Length > 0 ? top : null;
        }

        // 그룹배관 패턴(L4) — 이번 라우팅 행들의 유틸리티별 학습 트렁크 고도(route_bundle_template)를 z-셀로
        // 변환해 rackLevels 에 합친다(중복 제거, 엔진 상한 8). DB 에 저장한 번들 패턴을 신규 라우팅에 직접 활용:
        // 같은 유틸 새 배관이 사람이 설계한 공용 랙(트렁크) 고도에 뭉친다. 키 미스/미적재면 입력 그대로 반환.
        private int[]? MergeBundleLevels(int[]? rackLevels, IReadOnlyList<int> rowPositions)
        {
            var s = _scene; if (s == null || _bundles == null) return rackLevels;
            var g = s.Grid; double oz = g.Oz, cell = g.CellMm; if (cell <= 0) return rackLevels;

            var utils = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rowPositions)
                if (!string.IsNullOrEmpty(Tasks[pos].Utility)) utils.Add(Tasks[pos].Utility!);
            if (utils.Count == 0) return rackLevels;

            var zset = new HashSet<int>(rackLevels ?? System.Array.Empty<int>());
            foreach (var u in utils)
            {
                var t = _bundles.TryGet(null, u);   // (owner 미상 → util 폴백) 유틸별 트렁크 고도.
                if (t == null) continue;
                foreach (var z in t.TrunkZs)
                {
                    int zk = (int)Math.Floor((z - oz) / cell);
                    if (zk >= 0 && zk < g.Nz) zset.Add(zk);
                }
            }
            return zset.Count > 0 ? zset.Take(8).ToArray() : rackLevels;
        }

        // 기존설계 패턴에서 학습된 진출/진입 면(예: EQUIP=-z, DUCT=+z)을 조회한다. 패턴 OFF/미적재/미스면 null.
        private string? LearnedFace(string anchorKind, string? group, string? utility)
        {
            if (!_usePatterns || _patterns == null) return null;
            return _patterns.TryGet(anchorKind, group, utility, out var tpl) ? tpl.Face : null;
        }

        // 검색증강(L3b) — DUCT 종단 진입면을 PoC 컨텍스트(앵커 덕트 내 상대위치 + 접근방향)별 ANN 으로 분기한다.
        // 한 (그룹·유틸) 키 안에 진입면이 여럿 섞인 다중면 키(예 UPW_S = +x 57% / +z 43%)에서, 집계 대표면이
        // 틀리는 PoC 를 그 PoC 와 가장 닮은 학습 표본의 면으로 바로잡는다. 단일면/미적재/앵커 없음 → 집계 폴백.
        private string? LearnedDuctFace(string? group, string? utility, double ex, double ey, double ez,
                                        double sx, double sy, double sz)
        {
            if (!_usePatterns || _patterns == null) return null;
            var d = NearestDuctAnchor(ex, ey, ez);
            if (d != null)
            {
                var rel = new[] { RelIn(ex, d.MinX, d.MaxX), RelIn(ey, d.MinY, d.MaxY), RelIn(ez, d.MinZ, d.MaxZ) };
                // 학습 DUCT dir_unit = unit(src - tgt)(덕트로의 접근방향). 추론 = unit(start - end).
                var dir = Unit(sx - ex, sy - ey, sz - ez);
                if (_patterns.TryGetFaceAnn("DUCT", group, utility, rel, dir, out var f)) return f;
            }
            return LearnedFace("DUCT", group, utility);   // 집계 폴백.
        }

        private static double RelIn(double v, double lo, double hi)
            => hi - lo <= 1e-6 ? 0.5 : Math.Min(1.0, Math.Max(0.0, (v - lo) / (hi - lo)));

        private static double[] Unit(double x, double y, double z)
        {
            double n = Math.Sqrt(x * x + y * y + z * z);
            return n < 1e-9 ? new double[3] : new[] { x / n, y / n, z / n };
        }

        // 종단 PoC 를 포함하는 덕트, 없으면 3000mm 내 가장 가까운 덕트(Python find_duct 미러). 없으면 null.
        private DuctLateral? NearestDuctAnchor(double x, double y, double z)
        {
            var s = _scene; if (s == null) return null;
            const double eps = 1.0, maxMm = 3000.0;
            foreach (var d in s.DuctsLaterals)
                if (x >= d.MinX - eps && x <= d.MaxX + eps && y >= d.MinY - eps && y <= d.MaxY + eps &&
                    z >= d.MinZ - eps && z <= d.MaxZ + eps) return d;
            DuctLateral? best = null; double bd = maxMm;
            foreach (var d in s.DuctsLaterals)
            {
                double cx = (d.MinX + d.MaxX) / 2, cy = (d.MinY + d.MaxY) / 2, cz = (d.MinZ + d.MaxZ) / 2;
                double dist = Math.Sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy) + (z - cz) * (z - cz));
                if (dist < bd) { bd = dist; best = d; }
            }
            return best;
        }

        // 시작점이 설비 AABB 내부면 그 설비(들) 중 가장 낮은 바닥(MinZ) 한 셀 아래로 Z 를 내려, 장비 밖
        // 첫 셀을 라우팅 시작점으로 만든다. XY 는 유지. 충돌확장 OFF(설비 비충돌)이거나 설비 밖이면 원점 유지.
        private (double x, double y, double z) DropStartBelowEquipment(double x, double y, double z)
        {
            var s = _scene;
            if (s == null || !_includeFacilities) return (x, y, z);
            const double eps = 1.0;
            double lowestBottom = double.NaN;
            foreach (var e in s.Equipment)
            {
                if (x < e.MinX - eps || x > e.MaxX + eps) continue;
                if (y < e.MinY - eps || y > e.MaxY + eps) continue;
                if (z < e.MinZ - eps || z > e.MaxZ + eps) continue;
                if (double.IsNaN(lowestBottom) || e.MinZ < lowestBottom) lowestBottom = e.MinZ;
            }
            if (double.IsNaN(lowestBottom)) return (x, y, z);   // 설비 안이 아님 → 그대로.
            double cell = s.Grid.CellMm;
            double nz = lowestBottom - cell * 0.5;              // 설비 바닥 한 셀 아래.
            double gridBottom = s.Grid.Oz + cell * 0.5;
            if (nz < gridBottom) nz = gridBottom;               // 격자 하한 밖이면 클램프.
            return (x, y, nz);
        }

        // PoC 가 설비/덕트/레터럴 '솔리드 내부'에 있으면 가장 가까운 면 바로 바깥(+½셀)으로 빼낸다.
        // 덕트 상부 PoC → 윗면 위 한 셀(자유)로 투영 → 배관이 덕트 표면에 닿아 연결되고 본체를 관통하지 않는다.
        // (충돌확장이 항상 ON 이라 설비/덕트가 솔리드이므로, 표면 투영이 없으면 PoC 셀이 막혀 라우팅이 실패하거나
        //  엔진 snap(반경 2)이 엉뚱한 옆 셀로 새어 관통/우회한다.) 여러 박스에 걸치면 몇 번 반복해 탈출.
        private (double x, double y, double z) LiftPocToSurface(double x, double y, double z,
                                                                string? preferFace = null)
        {
            var s = _scene;
            if (s == null || !_includeFacilities) return (x, y, z);
            double cell = s.Grid.CellMm, eps = 1.0, m = cell * 0.5;
            for (int iter = 0; iter < 4; iter++)
            {
                bool moved = false;
                // 한 박스 안이면 탈출 면을 고른다. preferFace(학습된 진출/진입 면)가 주어지고 그 면으로 나갈
                // 수 있으면 그 면으로(사람 설계 관례: 덕트=+z 상부, 장비=-z 하부) — 없으면 침투가 가장 얕은 면.
                void TryBox(double bMinX, double bMinY, double bMinZ, double bMaxX, double bMaxY, double bMaxZ)
                {
                    if (x <= bMinX - eps || x >= bMaxX + eps) return;
                    if (y <= bMinY - eps || y >= bMaxY + eps) return;
                    if (z <= bMinZ - eps || z >= bMaxZ + eps) return;
                    if (preferFace != null)
                    {
                        switch (preferFace)
                        {
                            case "+z": z = bMaxZ + m; moved = true; return;
                            case "-z": z = bMinZ - m; moved = true; return;
                            case "+x": x = bMaxX + m; moved = true; return;
                            case "-x": x = bMinX - m; moved = true; return;
                            case "+y": y = bMaxY + m; moved = true; return;
                            case "-y": y = bMinY - m; moved = true; return;
                        }
                    }
                    double dxn = x - bMinX, dxp = bMaxX - x;
                    double dyn = y - bMinY, dyp = bMaxY - y;
                    double dzn = z - bMinZ, dzp = bMaxZ - z;
                    double mn = Math.Min(Math.Min(Math.Min(dxn, dxp), Math.Min(dyn, dyp)), Math.Min(dzn, dzp));
                    if (mn == dzp) z = bMaxZ + m;        // 윗면(덕트 상부 PoC 의 일반적 경우)
                    else if (mn == dzn) z = bMinZ - m;   // 아랫면
                    else if (mn == dxp) x = bMaxX + m;
                    else if (mn == dxn) x = bMinX - m;
                    else if (mn == dyp) y = bMaxY + m;
                    else y = bMinY - m;
                    moved = true;
                }
                foreach (var e in s.Equipment) TryBox(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
                foreach (var d in s.DuctsLaterals) TryBox(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
                if (!moved) break;
            }
            double zMin = s.Grid.Oz + m;
            if (z < zMin) z = zMin;
            return (x, y, z);
        }

        // 접근불가 PoC 전처리 — Lift 후에도 시작/종단 셀이 솔리드에 막혀 있으면(파묻힌 PoC: 여러 솔리드에
        // 둘러싸여 면 투영이 다시 솔리드 안으로 떨어지거나, 엔진 snap(반경 2)으로도 못 빠져나오는 경우)
        // 가장 가까운 '자유 셀'로 옮긴다. ① 학습된 면 법선으로 바깥 행진(도메인 방향 우선: 덕트=+z 상부 등)
        // → ② 실패 시 체비셰프 반경을 넓혀가며 최근접 자유 셀. 끝내 못 찾으면(진짜 접근불가) 원점 유지.
        // 이미 자유 셀이면 그대로 반환(이미 성공하는 작업엔 영향 0 = 회귀 없음).
        private const int SnapMarchCells = 16;   // 학습면 방향 최대 행진(셀).
        private const int SnapMaxRadius = 6;     // 반경 확장 탐색 상한(셀).

        private (double x, double y, double z) SnapPocToFreeCell(double x, double y, double z, string? preferFace)
        {
            var s = _scene;
            if (s == null || !_includeFacilities) return (x, y, z);
            double cell = s.Grid.CellMm;
            if (!CellBlocked(x, y, z)) return (x, y, z);   // 이미 자유 → 변경 없음.

            // ① 학습된 진출/진입 면 방향으로 바깥 행진(있으면) — 도메인에 맞는 방향으로 먼저 탈출 시도.
            if (preferFace != null)
            {
                var (dx, dy, dz) = FaceNormal(preferFace);
                if (dx != 0 || dy != 0 || dz != 0)
                    for (int k = 1; k <= SnapMarchCells; k++)
                    {
                        double nx = x + dx * cell * k, ny = y + dy * cell * k, nz = z + dz * cell * k;
                        if (InGrid(nx, ny, nz) && !CellBlocked(nx, ny, nz)) return (nx, ny, nz);
                    }
            }
            // ② 체비셰프 반경 확장 — 셸 단위로 넓혀가며 가장 가까운 자유 셀.
            for (int r = 1; r <= SnapMaxRadius; r++)
            {
                (double, double, double)? best = null; double bestD = double.MaxValue;
                for (int di = -r; di <= r; di++)
                    for (int dj = -r; dj <= r; dj++)
                        for (int dk = -r; dk <= r; dk++)
                        {
                            if (Math.Max(Math.Max(Math.Abs(di), Math.Abs(dj)), Math.Abs(dk)) != r) continue; // 셸만.
                            double nx = x + di * cell, ny = y + dj * cell, nz = z + dk * cell;
                            if (!InGrid(nx, ny, nz) || CellBlocked(nx, ny, nz)) continue;
                            double d = di * di + dj * dj + dk * dk;
                            if (d < bestD) { bestD = d; best = (nx, ny, nz); }
                        }
                if (best != null) return best.Value;
            }
            return (x, y, z);  // 진짜 접근불가 → 엔진 snap 에 맡김(여전히 실패할 수 있음).
        }

        private static (double dx, double dy, double dz) FaceNormal(string face) => face switch
        {
            "+x" => (1, 0, 0),
            "-x" => (-1, 0, 0),
            "+y" => (0, 1, 0),
            "-y" => (0, -1, 0),
            "+z" => (0, 0, 1),
            "-z" => (0, 0, -1),
            _ => (0, 0, 0),
        };

        private bool InGrid(double x, double y, double z)
        {
            var g = _scene!.Grid;
            return x >= g.Ox && y >= g.Oy && z >= g.Oz
                && x <= g.Ox + g.Nx * g.CellMm && y <= g.Oy + g.Ny * g.CellMm && z <= g.Oz + g.Nz * g.CellMm;
        }

        // 점이 속한 셀(반열린 [lo, lo+cell))이 어떤 솔리드(장애물 비통과 + 설비/덕트, 얇은 축은 minT 팽창)와
        // 겹치면 막힘. 엔진 복셀화(box↔cell overlap, 동일 minT)와 같은 기준 → 엔진 is_blocked 근사.
        private bool CellBlocked(double x, double y, double z)
        {
            var s = _scene!; var g = s.Grid; double cell = g.CellMm, minT = cell;
            int ci = (int)Math.Floor((x - g.Ox) / cell);
            int cj = (int)Math.Floor((y - g.Oy) / cell);
            int ck = (int)Math.Floor((z - g.Oz) / cell);
            double clx = g.Ox + ci * cell, chx = clx + cell;
            double cly = g.Oy + cj * cell, chy = cly + cell;
            double clz = g.Oz + ck * cell, chz = clz + cell;

            bool Overlap(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            {
                if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
                if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
                if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
                return clx < mxx && chx > mnx && cly < mxy && chy > mny && clz < mxz && chz > mnz;
            }
            foreach (var o in s.Obstacles)
                if (!o.IsPassThrough && Overlap(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ)) return true;
            foreach (var e in s.Equipment)
                if (Overlap(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ)) return true;
            foreach (var d in s.DuctsLaterals)
                if (Overlap(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ)) return true;
            return false;
        }

        private string? BlockingObjectAt(double x, double y, double z)
        {
            var s = _scene; if (s == null) return null;
            var g = s.Grid; double cell = g.CellMm, minT = cell;
            int ci = (int)Math.Floor((x - g.Ox) / cell);
            int cj = (int)Math.Floor((y - g.Oy) / cell);
            int ck = (int)Math.Floor((z - g.Oz) / cell);
            double clx = g.Ox + ci * cell, chx = clx + cell;
            double cly = g.Oy + cj * cell, chy = cly + cell;
            double clz = g.Oz + ck * cell, chz = clz + cell;

            bool Overlap(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            {
                if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
                if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
                if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
                return clx < mxx && chx > mnx && cly < mxy && chy > mny && clz < mxz && chz > mnz;
            }

            string Clean(string? name, string fallback) => string.IsNullOrWhiteSpace(name) ? fallback : name.Trim();
            double Volume(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
                => Math.Max(1, mxx - mnx) * Math.Max(1, mxy - mny) * Math.Max(1, mxz - mnz);

            string? best = null; double bestVol = double.MaxValue;
            void Consider(string label, double mnx, double mny, double mnz, double mxx, double mxy, double mxz)
            {
                if (!Overlap(mnx, mny, mnz, mxx, mxy, mxz)) return;
                double vol = Volume(mnx, mny, mnz, mxx, mxy, mxz);
                if (vol < bestVol) { bestVol = vol; best = label; }
            }

            for (int i = 0; i < s.Obstacles.Count; i++)
            {
                var o = s.Obstacles[i];
                if (o.IsPassThrough) continue;
                Consider($"장애물 #{i} {Clean(o.Name, o.OstType.Length > 0 ? o.OstType : "AABB")}",
                    o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            }
            foreach (var e in s.Equipment)
                Consider($"장비 {Clean(e.Name, e.IsMain ? "MAIN" : "EQUIPMENT")}",
                    e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
            foreach (var d in s.DuctsLaterals)
                Consider($"{(d.IsLateral ? "레터럴" : "덕트")} {Clean(d.Name, d.Category)}",
                    d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
            return best;
        }

        // 충돌확장 — 라우팅 엔진에 추가 충돌 대상을 장애물로 넣는다(IncludeFacilities ON 일 때).
        //   · 설비(TB_BIM_EQUIPMENT) — 메인 장비 포함 전체
        //   · 덕트/레터럴(TB_DUCT_LATERAL)
        //   · 이미 우리 알고리즘으로 설계된(라우팅 성공) 다른 배관의 경로(currentRows 의 자기 자신은 제외)
        // 시작/끝 PoC 가 이들 표면(특히 메인 장비)에 닿아 막히면 엔진의 snap_to_free_cell(반경 2) 이
        // 인접 빈 셀로 옮겨 시작점을 확보한다.
        private void AddFacilityObstacles(Engine engine, HashSet<int> currentRows, bool forceFacilities = false)
        {
            if ((!_includeFacilities && !forceFacilities) || _scene == null) return;
            var s = _scene;
            double cell = s.Grid.CellMm;
            double minT = cell;   // 두께 0 축을 최소 셀 1개로 팽창(가는 덕트/판도 셀을 막도록).

            // ★ 설비·덕트·레터럴을 '항상 솔리드 장애물'로 추가한다(제외 없음). 예전에는 종단 PoC 를 포함하는
            //   덕트/설비 박스를 통째로 제외했는데, 그러면 그 덕트가 비충돌이 되어 배관이 덕트를 '관통'했다.
            //   대신 종단/시작 PoC 가 솔리드에 갇히는 문제는 LiftPocToSurface 로 PoC 를 표면 바로 바깥으로
            //   투영해 해결한다(BuildEngineForRows) → 배관이 덕트 표면에 닿아 연결되고 본체는 관통하지 않는다.
            foreach (var e in s.Equipment)        // 메인 장비 포함 전체 설비.
                AddBoxObstacle(engine, e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ, minT);
            foreach (var d in s.DuctsLaterals)     // 덕트/레터럴(부대장비 포함).
                AddBoxObstacle(engine, d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ, minT);

            // 이미 설계된(라우팅 성공) 다른 배관의 경로 + 고정 스텁을 점유로 추가 — 새 배관이 이를 피하도록.
            // 메모리 효율: 셀별 점유가 아니라 직선 구간 AABB(반경 r 팽창)로 등록한다(엔진 호출/메모리 최소).
            // r = 배관 실반경(DiameterMm/2). 이 배관의 물리 공간을 정확히 막아야 새 배관이 관통하지 않는다.
            // cell*0.6(15mm) 는 150mm 관경 배관에 턱없이 부족 → 관통 에러 주원인.
            for (int i = 0; i < Tasks.Count; i++)
            {
                if (currentRows.Contains(i)) continue;          // 지금 라우팅하는(=자기) 배관은 제외(자기 스텁에 막히지 않게).
                var row = Tasks[i];
                if (!row.Success || row.Path.Length < 2) continue;
                double r = row.DiameterMm > 0 ? row.DiameterMm / 2.0 : cell;   // 실제 관경 반경. 미상이면 1셀.
                AddPathObstacle(engine, row.Path, s.Grid, r);
                // 고정 출발/종단 스텁(수직+엘보)도 장애물로 — 다른 배관이 스텁을 관통/교차하지 않도록.
                if (row.StartStub != null) AddPolylineObstacle(engine, row.StartStub, r);
                if (row.EndStub != null) AddPolylineObstacle(engine, row.EndStub, r);
            }
        }

        // 월드 mm 폴리라인을 직선 구간별 AABB(반경 r 팽창)로 장애물에 추가. 셀 복셀화 없이 세그먼트당 박스 1개(메모리 효율).
        private Engine BuildOctreePreviewEngine(GridMeta g)
        {
            var engine = new Engine();
            engine.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
            engine.SetParams(g.CellMm, 500, 10, 2, 6);

            var s = _scene;
            if (s == null) return engine;

            foreach (var o in s.Obstacles)
            {
                if (o.IsPassThrough)
                    engine.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                else
                    engine.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            }

            // Octree preview is diagnostic: include all static facility geometry shown in the viewer,
            // but skip already routed pipe paths so the display represents the DB collision space.
            AddFacilityObstacles(engine, new HashSet<int>(Enumerable.Range(0, Tasks.Count)), forceFacilities: true);
            return engine;
        }

        private static void AddPolylineObstacle(Engine engine, System.Collections.Generic.IReadOnlyList<Pt3> poly, double r)
        {
            for (int i = 1; i < poly.Count; i++)
            {
                var a = poly[i - 1]; var b = poly[i];
                engine.AddObstacle(
                    Math.Min(a.X, b.X) - r, Math.Min(a.Y, b.Y) - r, Math.Min(a.Z, b.Z) - r,
                    Math.Max(a.X, b.X) + r, Math.Max(a.Y, b.Y) + r, Math.Max(a.Z, b.Z) + r);
            }
        }

        // AABB 장애물 추가 — 두께 0 축은 minT 로 팽창해 반드시 셀을 점유하게 한다.
        private static void AddBoxObstacle(Engine engine, double mnx, double mny, double mnz,
                                           double mxx, double mxy, double mxz, double minT)
        {
            if (mxx - mnx < minT) { double c = (mnx + mxx) / 2; mnx = c - minT / 2; mxx = c + minT / 2; }
            if (mxy - mny < minT) { double c = (mny + mxy) / 2; mny = c - minT / 2; mxy = c + minT / 2; }
            if (mxz - mnz < minT) { double c = (mnz + mxz) / 2; mnz = c - minT / 2; mxz = c + minT / 2; }
            engine.AddObstacle(mnx, mny, mnz, mxx, mxy, mxz);
        }

        // 경로(셀 폴리라인)를 직선 구간별 AABB(반경 r 팽창) 로 장애물에 추가(호출 수 절감).
        private static void AddPathObstacle(Engine engine, PathCell[] path, GridMeta g, double r)
        {
            (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));
            int n = path.Length, seg = 0;
            var cur = Dir(path[0], path[1]);
            for (int i = 2; i <= n; i++)
            {
                var d = (i < n) ? Dir(path[i - 1], path[i]) : (int.MinValue, 0, 0);
                if (d != cur)
                {
                    var pa = CellToWorld(g, path[seg]); var pb = CellToWorld(g, path[i - 1]);
                    engine.AddObstacle(
                        Math.Min(pa.X, pb.X) - r, Math.Min(pa.Y, pb.Y) - r, Math.Min(pa.Z, pb.Z) - r,
                        Math.Max(pa.X, pb.X) + r, Math.Max(pa.Y, pb.Y) + r, Math.Max(pa.Z, pb.Z) + r);
                    seg = i - 1;
                    if (i < n) cur = Dir(path[i - 1], path[i]);
                }
            }
        }

        // 지정 행들의 자동설계 결과(경로/성공/길이/방문)를 초기화한다 — '기존설계 삭제' / 재설계 전 초기화에 사용.
        private void ClearRouteResults(IEnumerable<int> rowPositions)
        {
            foreach (var pos in rowPositions)
            {
                var r = Tasks[pos];
                r.Success = false;
                r.LengthMm = 0;
                r.Path = System.Array.Empty<PathCell>();
                r.Visited = System.Array.Empty<PathCell>();
                r.ExpandedNodes = 0;
                r.ElapsedMs = 0;
                r.LastFail = Interop.RouteFail.None;
                r.RouteOrder = -1;
                r.NotifyResultChanged();
            }
        }

        // 자동설계된 '모든' 경로 삭제(버튼) — 전체 작업 결과 초기화 + 라이브 오버레이 제거 + 재렌더.
        private void ClearAllRoutes()
        {
            if (_scene == null) return;
            var all = new List<int>(Tasks.Count);
            for (int i = 0; i < Tasks.Count; i++) all.Add(i);
            ClearRouteResults(all);
            _compareMode = false;
            ResetLiveRoute();
            ResultList.Clear();          // 우측 결과 리스트 비우기.
            AnalysisReport = null;       // 분석결과 탭도 초기화.
            RefreshRouteProgress();      // 진행바 0 으로.
            BuildModel();
            System.Windows.Input.CommandManager.InvalidateRequerySuggested();  // 버튼 비활성 즉시 반영.
            Status = "자동설계된 경로를 모두 삭제했습니다.";
        }

        // 엔진 결과(엔진 인덱스 e ↔ added[e] 행)를 행 캐시에 기록. 부분집합 라우팅 후 호출.
        private void CacheResults(IReadOnlyList<int> added)
        {
            for (int e = 0; e < added.Count; e++)
            {
                var row = Tasks[added[e]];
                try
                {
                    var r = _engine!.GetResult(e);
                    row.Success = r.Success; row.LengthMm = r.LengthMm;
                    row.Path = r.Path; row.Visited = r.Visited;
                    row.ExpandedNodes = r.ExpandedNodes;
                    row.LastFail = r.Success ? Interop.RouteFail.None : r.Fail;
                    row.ElapsedMs = r.ElapsedMs;
                    if (row.RouteOrder < 0) row.RouteOrder = e;
                    row.NotifyResultChanged();
                }
                catch
                {
                    row.Success = false; row.LengthMm = 0; row.Path = Array.Empty<PathCell>(); row.Visited = Array.Empty<PathCell>();
                    row.ExpandedNodes = 0; row.ElapsedMs = 0; row.LastFail = Interop.RouteFail.None;
                    if (row.RouteOrder < 0) row.RouteOrder = e;
                    row.NotifyResultChanged();
                }
            }
        }

        // ===================================================== 기존배관 복제(폴리라인) — UseDesignReplicate
        // 매칭되는 기존 설계배관이 있으면 그 폴리라인을 셀 경로로 '복제'하고, 현재 점유에서 막힌(장애물이
        // 달라진) 구간만 A* 로 국소 우회 수리한다. route_multi 완료(CacheResults) 후 호출 — 매칭 행의 row.Path
        // 를 복제경로로 덮어쓴다(스텁은 폴리라인에 포함되므로 비운다). 백그라운드 스레드에서 호출(네이티브 A*).
        //   수리용 엔진 = 현재 장애물+설비만(다른 새 배관 마크 없음 → 순수 물리 장애물). 복제는 기존설계가
        //   충돌 없었다는 전제라 새 배관끼리 충돌은 드물다(새 장애물 우회 구간에서만 가능 — v1 한계, 문서화).
        private int ReplicateMatchedPipes(IReadOnlyList<int> added)
        {
            if (_scene == null) return 0;
            var s = _scene; var g = s.Grid;
            // 수리 A*(r3d_route_task)는 호출마다 DenseOccupancy(격자 전체)를 복셀화하므로, 초대형 격자
            //   (cell≤25 등)에선 메모리/시간 폭증. 30M 셀 초과면 복제 생략(A* 결과 유지) — 그룹 라우팅은
            //   cell=50(약 11M) 로 재적재되므로 정상 동작. 필요 시 셀을 키워 복제하라고 상태바에 안내.
            if ((long)g.Nx * g.Ny * g.Nz > 30_000_000L) return -1;

            // 수리 엔진(장애물+설비, 배관 마크 없음) — 막힌 구간 국소 A* 에 재사용.
            using var rep = new Engine();
            rep.SetGrid(g.CellMm, g.Ox, g.Oy, g.Oz, g.Nx, g.Ny, g.Nz);
            bool weighted = (long)g.Nx * g.Ny * g.Nz > 300_000;
            rep.SetParams(g.CellMm, 500, 10, 2, 6, wHeur: weighted ? 2.0 : 1.0, wHeurNear: weighted ? 1.0 : 0.0);
            foreach (var o in s.Obstacles)
                if (o.IsPassThrough) rep.AddPassthrough(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
                else rep.AddObstacle(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            // currentRows = 전체 → AddFacilityObstacles 의 배관 루프가 모두 스킵(설비·덕트만 추가, 배관 마크 없음).
            AddFacilityObstacles(rep, new HashSet<int>(Enumerable.Range(0, Tasks.Count)));

            // 막힘 판정 — 셀 AABB 가 장애물(통과 제외)·설비·덕트 박스와 겹치면 막힘. 미세격자에서 전체
            //   복셀(CopyBlocked) 을 HashSet 으로 들고 있으면 수백MB 라, 폴리라인 셀만 박스질의로 검사한다.
            double minT = g.CellMm;
            var boxes = new List<(double mnx, double mny, double mnz, double mxx, double mxy, double mxz)>();
            void AddBox(double a, double b, double c2, double d, double e2, double f2)
            {
                if (d - a < minT) { double m = (a + d) / 2; a = m - minT / 2; d = m + minT / 2; }
                if (e2 - b < minT) { double m = (b + e2) / 2; b = m - minT / 2; e2 = m + minT / 2; }
                if (f2 - c2 < minT) { double m = (c2 + f2) / 2; c2 = m - minT / 2; f2 = m + minT / 2; }
                boxes.Add((a, b, c2, d, e2, f2));
            }
            foreach (var o in s.Obstacles) if (!o.IsPassThrough) AddBox(o.MinX, o.MinY, o.MinZ, o.MaxX, o.MaxY, o.MaxZ);
            foreach (var e in s.Equipment) AddBox(e.MinX, e.MinY, e.MinZ, e.MaxX, e.MaxY, e.MaxZ);
            foreach (var d in s.DuctsLaterals) AddBox(d.MinX, d.MinY, d.MinZ, d.MaxX, d.MaxY, d.MaxZ);
            bool Blocked(int ci, int cj, int ck)
            {
                double clx = g.Ox + ci * g.CellMm, chx = clx + g.CellMm;
                double cly = g.Oy + cj * g.CellMm, chy = cly + g.CellMm;
                double clz = g.Oz + ck * g.CellMm, chz = clz + g.CellMm;
                foreach (var bx in boxes)
                    if (clx < bx.mxx && chx > bx.mnx && cly < bx.mxy && chy > bx.mny && clz < bx.mxz && chz > bx.mnz)
                        return true;
                return false;
            }

            int replaced = 0;
            foreach (var pos in added)
            {
                var row = Tasks[pos];
                var pipe = FindMatchingExistingPipe(row);
                if (pipe == null || pipe.Points.Count < 2) continue;   // 매칭 없으면 A* 결과 유지.
                var path = ReplicatePath(row, pipe, g, Blocked, rep);
                if (path == null || path.Length < 2) continue;          // 복제/수리 실패 → A* 결과 유지.
                // 복제경로 직선화 — 기존배관 폴리라인 셀변환·수리 A* 로 생긴 짧은 jog/킨크 제거.
                if (path.Length >= 3) { path = StraightenOrtho(path, Blocked); path = DeJog(path, Blocked); }
                row.Path = path;
                row.StartStub = null; row.EndStub = null;               // 복제경로가 PoC~PoC 전체를 담는다.
                row.Success = true;
                row.LengthMm = (path.Length - 1) * g.CellMm;            // 직교 셀 경로 근사 길이.
                replaced++;
            }
            return replaced;
        }

        // 매칭 기존배관 폴리라인(작업 PoC 방향 정렬 + 양 끝에 실제 PoC 부착)을 6-연결 셀 경로로 복제하고,
        // 막힌 구간만 A* 로 국소 우회한다. 수리 실패/꼬리 막힘이면 null(호출자 A* 폴백).
        private PathCell[]? ReplicatePath(TaskRowVM row, ExistingPipe pipe, GridMeta g,
                                          Func<int, int, int, bool> blocked, Engine rep)
        {
            // 작업 start 가 배관 source 에 가까우면 정방향, 아니면 역방향.
            Pt3 ps = pipe.SourcePos ?? pipe.Points[0];
            Pt3 pe = pipe.TargetPos ?? pipe.Points[pipe.Points.Count - 1];
            var ts = new Pt3(row.Sx, row.Sy, row.Sz); var te = new Pt3(row.Gx, row.Gy, row.Gz);
            bool fwd = Dist(ts, ps) + Dist(te, pe) <= Dist(ts, pe) + Dist(te, ps);
            var pts = new List<Pt3>(pipe.Points);
            if (!fwd) pts.Reverse();
            pts.Insert(0, ts); pts.Add(te);   // 폴리라인 양 끝을 작업 실제 PoC 로(끝점이 약간 달라도 보정).

            var cells = PolylineToCells(pts, g);
            if (cells.Count < 2) return null;

            bool Blk(PathCell c) => blocked(c.I, c.J, c.K);
            var outp = new List<PathCell>();
            int i = 0;
            while (i < cells.Count && Blk(cells[i])) i++;   // 선두 막힘 스킵(시작 PoC 가 솔리드면).
            if (i >= cells.Count) return null;
            outp.Add(cells[i]); i++;
            while (i < cells.Count)
            {
                // 다음 '자유' 셀까지 전진(막힌 구간은 건너뛰고 j 로).
                int j = i;
                while (j < cells.Count && Blk(cells[j])) j++;
                if (j >= cells.Count) break;                // 꼬리가 막힘 → 종단 못 닿음(미스).
                var prev = outp[outp.Count - 1];
                if (j == i && Adjacent(prev, cells[i]))
                    outp.Add(cells[i]);                     // 자유·인접 → 폴리라인 그대로 복제.
                else
                {
                    // 막힌 구간 또는 비인접(엘보/끝점 보정) → 국소 A* 우회.
                    var repair = RepairAStar(prev, cells[j], rep, g);
                    if (repair == null) return null;        // 수리 실패 → 복제 포기.
                    for (int k = 1; k < repair.Length; k++) outp.Add(repair[k]);
                }
                i = j + 1;
            }
            return outp.Count >= 2 ? outp.ToArray() : null;
        }

        // 월드 폴리라인(직교 가정) → 6-연결 셀 경로. 각 구간을 셀 단위로 행진하고, 비인접(엘보·끝점)은 ortho 보간.
        private static List<PathCell> PolylineToCells(IReadOnlyList<Pt3> pts, GridMeta g)
        {
            var outc = new List<PathCell>();
            PathCell ToCell(Pt3 p) => new(
                Math.Clamp((int)Math.Floor((p.X - g.Ox) / g.CellMm), 0, g.Nx - 1),
                Math.Clamp((int)Math.Floor((p.Y - g.Oy) / g.CellMm), 0, g.Ny - 1),
                Math.Clamp((int)Math.Floor((p.Z - g.Oz) / g.CellMm), 0, g.Nz - 1));
            void Push(PathCell c)
            {
                if (outc.Count == 0) { outc.Add(c); return; }
                if (outc[outc.Count - 1].Equals(c)) return;
                if (!Adjacent(outc[outc.Count - 1], c)) AppendOrtho(outc, c);   // 점프 → ortho 채움.
                else outc.Add(c);
            }
            for (int seg = 1; seg < pts.Count; seg++)
            {
                var a = pts[seg - 1]; var b = pts[seg];
                double len = Dist(a, b);
                int steps = Math.Max(1, (int)(len / (g.CellMm * 0.5)));
                for (int sIdx = 0; sIdx <= steps; sIdx++)
                {
                    double tt = (double)sIdx / steps;
                    Push(ToCell(new Pt3(a.X + (b.X - a.X) * tt, a.Y + (b.Y - a.Y) * tt, a.Z + (b.Z - a.Z) * tt)));
                }
            }
            return outc;
        }

        // outc 의 마지막 셀에서 to 까지 한 축씩(z→x→y) ortho 로 채운다(각 셀 6-연결 보장).
        private static void AppendOrtho(List<PathCell> outc, PathCell to)
        {
            var cur = outc[outc.Count - 1];
            while (cur.K != to.K) { cur = new PathCell(cur.I, cur.J, cur.K + Math.Sign(to.K - cur.K)); outc.Add(cur); }
            while (cur.I != to.I) { cur = new PathCell(cur.I + Math.Sign(to.I - cur.I), cur.J, cur.K); outc.Add(cur); }
            while (cur.J != to.J) { cur = new PathCell(cur.I, cur.J + Math.Sign(to.J - cur.J), cur.K); outc.Add(cur); }
        }

        private static bool Adjacent(PathCell a, PathCell b)
            => Math.Abs(a.I - b.I) + Math.Abs(a.J - b.J) + Math.Abs(a.K - b.K) == 1;

        // 수리 엔진으로 from→to(셀) 사이를 단일 A* 로 잇는다(장애물 기준). 실패면 null.
        private PathCell[]? RepairAStar(PathCell from, PathCell to, Engine rep, GridMeta g)
        {
            var a = CellToWorld(g, from); var b = CellToWorld(g, to);
            try
            {
                int t = rep.AddTask(a.X, a.Y, a.Z, b.X, b.Y, b.Z, null, null);
                var r = rep.RouteTask(t);
                return r.Success && r.Path.Length >= 1 ? r.Path : null;
            }
            catch { return null; }
        }

        // 엔진 인덱스 == 행 인덱스(전체 작업이 파일/추가 순서대로 적재된 경우) 결과 캐시.
        private void CacheResultsByIndex()
        {
            for (int i = 0; i < Tasks.Count; i++)
            {
                var row = Tasks[i];
                try
                {
                    var r = _engine!.GetResult(i);
                    row.Success = r.Success; row.LengthMm = r.LengthMm;
                    row.Path = r.Path; row.Visited = r.Visited;
                    row.ExpandedNodes = r.ExpandedNodes;
                    row.LastFail = r.Success ? Interop.RouteFail.None : r.Fail;
                    row.NotifyResultChanged();
                }
                catch
                {
                    row.Success = false; row.LengthMm = 0; row.Path = Array.Empty<PathCell>(); row.Visited = Array.Empty<PathCell>();
                    row.ExpandedNodes = 0; row.LastFail = Interop.RouteFail.None; row.NotifyResultChanged();
                }
            }
        }

        /// <summary>지정 행들만 부분집합으로 라우팅(무거운 네이티브 호출은 백그라운드 → UI 비차단).
        /// 범위에 없는 행의 경로 캐시는 보존된다(그룹/유틸을 차례로 눌러 누적 표시 가능).</summary>
        private async Task RouteRowsAsync(IReadOnlyList<int> rowPositions, string label, bool corridor,
                                          bool showProgress = false)
        {
            if (_scene == null || rowPositions.Count == 0) return;
            if (_isRouting) return;   // 중복 실행 방지(이미 라우팅 중).
            _cancelRequested = false;
            IsRouting = true;
            try
            {
                // 그룹 라우팅(corridor)에서 '다건'(2개 이상)은 셀 ≤ 50mm 라야 인접 레인이 분리돼 다발이
                // 형성된다. DB 프로젝트가 로드된 상태에서 셀이 크면 50mm 로 낮춰 재적재(재복셀화)한다. 작업(Task)
                // 목록은 셀과 무관하게 동일 순서로 재생성되므로 rowPositions 인덱스는 그대로 유효하다.
                //   단건(rowPositions.Count==1)은 재적재하지 않는다 — 재적재는 BuildTaskRows 로 모든 누적 경로를
                //   지우므로(다른 배관 결과 손실), 단건 재라우팅에는 부적절. 단건은 그룹 트렁크 회랑만 따라가면 충분.
                if (corridor && rowPositions.Count > 1 && _cellMm > 50.0 && SelectedProject != null)
                {
                    Status = "그룹 라우팅: cell 50mm 로 재적재 중…";
                    CellMm = 50.0;
                    await LoadFromDbAsync(SelectedProject.ProjectId);
                }

                // 동일 대상(동일 Start PoC)을 다시 자동설계할 때 — 기존 설계 데이터를 먼저 지우고 진행한다.
                // (대상 행만 초기화; 다른 그룹/유틸의 누적 경로는 보존 → 충돌 회피·표시 유지.)
                ClearRouteResults(rowPositions);
                BuildModel();   // 지운 상태를 즉시 3D에 반영(라이브 오버레이 위에 옛 경로가 남지 않도록).

                bool cor = corridor;   // 그룹배관 라우팅 모드(스텁+공용 트렁크 회랑+강한 w_corridor).
                var added = BuildEngineForRows(rowPositions, groupMode: cor);
                ConfigureSearchTraceIfEnabled(label, added.Count);
                var batchSw = System.Diagnostics.Stopwatch.StartNew();   // 전체 배치 라우팅 시간 측정 시작.
                Status = $"경로 탐색 중… {label} (작업 {added.Count})";
                var engine = _engine!;
                double cellMm = _scene.Grid.CellMm;
                // 그룹 모드는 유틸 우선순위로 정렬 — 같은 유틸을 묶어 순차 라우팅하면 self-bundling 이 유틸별로
                // 일관되게 자라 공용 트렁크에 다발로 모인다. 일반 모드는 기존 정렬(_priority).
                string priority = cor ? "utility" : _priority;
                // 계층 corridor 사용 여부(그룹 모드는 route_multi 로 회랑/랙 번들링 유지).
                bool hier = _useHierarchicalCorridor && !cor;
                // coarse 셀 ≈ 160mm 가 되도록 factor 산출(4~24 클램프), 통로 반경 2 coarse 셀.
                int factor = Math.Clamp((int)Math.Round(160.0 / Math.Max(1.0, cellMm)), 4, 24);

                // 진행 표시 — 그룹 모드 또는 showProgress 일 때(계층 corridor 제외) 배관별 콜백 실시간 표시.
                // (예전 RoutingProgressWindow 다이얼로그 대신 우측 패널 '자동설계 결과 경로' + 진행바에 인라인 표시.)
                bool useProgress = !hier;
                var grid = _scene.Grid;
                // 엔진 작업 인덱스(=added 순서)별 메타/색을 'UI 스레드에서' 미리 뽑는다(콜백은 백그라운드 스레드라
                // Brush(.Color)·컬렉션 접근이 불가). 콜백은 이 값 배열만 참조 → 스레드 안전.
                var meta = new (string grp, string util, string sp, string ep, Color col)[added.Count];
                for (int k = 0; k < added.Count; k++)
                {
                    var tr = Tasks[added[k]];
                    string sp = string.IsNullOrEmpty(tr.PocName) ? $"({tr.Sx:0},{tr.Sy:0},{tr.Sz:0})" : tr.PocName!;
                    string ep = string.IsNullOrEmpty(tr.EndName) ? $"({tr.Gx:0},{tr.Gy:0},{tr.Gz:0})" : tr.EndName!;
                    Color col = (tr.Swatch as SolidColorBrush)?.Color ?? Colors.Cyan;
                    meta[k] = (tr.Group ?? "", tr.Utility ?? "", sp, ep, col);
                }
                // 우측 '자동설계 결과 경로' 리스트를 이번 배치 행으로 채운다(같은 TaskRowVM 인스턴스 참조 →
                // 행 상태/선택/3D 렌더가 Tasks 와 일관). 탐색 전 상태 = 대기(Queued).
                ResultList.Clear();
                foreach (var k in added)
                {
                    var tr = Tasks[k];
                    tr.RunState = RouteRunState.Queued;
                    tr.RouteOrder = -1;
                    ResultList.Add(tr);
                }
                RouteProgressValue = 0;
                RouteProgressText = $"라우팅 시작 — 0/{added.Count}";
                if (useProgress) ResetLiveRoute();   // 이전 라이브 오버레이 제거 → 이번 배치만 점진 표시.
                var disp = System.Windows.Application.Current?.Dispatcher;
                int liveOk = 0, liveFail = 0;   // 라이브 집계(콜백 → UI 마샬링 후 갱신).

                await Task.Run(() =>
                {
                    if (hier)
                    {
                        // Sparse + astar_hashed: 셀 수 배열을 안 잡아 10mm 대형 격자도 안전.
                        // priority 순차 + mark_pipe 로 배관 간 충돌 0.
                        engine.RouteCorridorMulti(factor, 2, priority, 0);
                    }
                    else if (useProgress)
                    {
                        var sw = System.Diagnostics.Stopwatch.StartNew();   // 전체 배치 경과 시간 측정.
                        engine.RouteMultiProgress(priority, p =>
                        {
                            int ti = p.TaskIndex;
                            var m = (ti >= 0 && ti < meta.Length)
                                ? meta[ti]
                                : (grp: "", util: "", sp: "", ep: "", col: Colors.Cyan);
                            int gpos = (ti >= 0 && ti < added.Count) ? added[ti] : -1;
                            // 배관 완료(성공) 시 즉시 3D 라이브 오버레이에 추가할 경로(없으면 null).
                            var path = (p.Phase == 1 && p.Success && p.Path.Length >= 1) ? p.Path : null;
                            var col = m.col;
                            int phase = p.Phase; bool ok = p.Success;
                            int done = p.Done, total = p.Total, oi = p.OrderIndex;
                            double prog01 = p.Progress01;
                            long expandedNodes = p.ExpandedNodes;
                            double pipeElapsedMs = p.ElapsedMs;   // 이 배관 한 개에 걸린 시간(엔진 내부 계측).
                            double totalElapsedMs = sw.Elapsed.TotalMilliseconds;
                            // 모든 UI/컬렉션 갱신은 UI 스레드로(콜백은 백그라운드 라우팅 스레드).
                            disp?.BeginInvoke(new Action(() =>
                            {
                                if (gpos >= 0)
                                {
                                    var row = Tasks[gpos];
                                    if (phase == 0)
                                    {
                                        row.RunState = RouteRunState.Searching;
                                        row.SearchProgress = prog01;
                                        string totalStr = totalElapsedMs < 1000
                                            ? $"{totalElapsedMs:0}ms" : $"{totalElapsedMs / 1000.0:0.0}s";
                                        string expStr = expandedNodes >= 1_000_000
                                            ? $"{expandedNodes / 1_000_000.0:0.0}M" : $"{expandedNodes / 1_000.0:0}k";
                                        // 상태바 텍스트: 중요 정보(배관번호·탐색량·경과)가 앞에, 덜 중요한 누적 통계가 뒤에.
                                        // TextTrimming 으로 뒤가 잘려도 핵심 내용은 보임. ToolTip 으로 전체 확인 가능.
                                        RouteProgressText = $"탐색 중 {done}/{total} · #{row.Index}[{row.Utility ?? "?"}] {prog01 * 100:0}%(확장 {expStr}) · {totalStr} · 성공 {liveOk} 실패 {liveFail}";
                                    }
                                    else   // phase 1 = 배관 완료.
                                    {
                                        row.RunState = RouteRunState.Idle;
                                        row.RouteOrder = oi;
                                        row.Success = ok;
                                        row.LengthMm = p.LengthMm;
                                        row.Path = p.Path;
                                        row.ExpandedNodes = expandedNodes;
                                        row.LastFail = InferProgressFail(ok, expandedNodes);
                                        row.ElapsedMs = pipeElapsedMs;   // 배관 개별 소요 시간 저장.
                                        row.NotifyResultChanged();
                                        if (ok) liveOk++; else liveFail++;
                                        double pct = total > 0 ? 100.0 * done / total : 0;
                                        RouteProgressValue = pct;
                                        string totalStr = totalElapsedMs < 1000
                                            ? $"{totalElapsedMs:0}ms" : $"{totalElapsedMs / 1000.0:0.0}s";
                                        RouteProgressText = $"완료 {done}/{total} · 성공 {liveOk} · 실패 {liveFail} · {pct:0}% · 경과 {totalStr}";
                                    }
                                }
                                if (path != null) AppendLivePipe(path, col, grid);
                            }));
                        }, shouldCancel: () => _cancelRequested);
                        sw.Stop();
                    }
                    else
                    {
                        engine.RouteMulti(priority);
                    }
                });
                CacheResults(added);
                foreach (var k in added) Tasks[k].RunState = RouteRunState.Idle;   // 진행 상태 종료(성공/실패는 Success로 표시).
                // 기존배관 복제(옵트인) — 매칭 행의 A* 결과를 '기존 폴리라인 복제 + 막힌 구간 국소 수리'로 교체.
                int replicated = 0;
                if (_useDesignReplicate)
                    replicated = await Task.Run(() => ReplicateMatchedPipes(added));
                ResetLiveRoute();   // 라이브 오버레이 제거 → 아래 BuildModel 의 최종 렌더로 대체(중복 방지).
                BuildModel();   // 누적(전체 씬) 기준 상태바를 먼저 갱신한 뒤,
                // 이번 배치 결과를 명확히 덮어쓴다 — "성공 16/113"(전체 대비)이 실패로 오해되지 않도록
                // "이번 라우팅 16/16"을 앞세우고 전체 누적은 괄호로 부기한다.
                int batchOk = 0;
                foreach (var pos in added) if (Tasks[pos].Success) batchOk++;
                int sceneOk = 0;
                foreach (var t in Tasks) if (t.Success) sceneOk++;
                batchSw.Stop();
                double batchMs = batchSw.Elapsed.TotalMilliseconds;
                string batchTimeStr = batchMs < 1000 ? $"{batchMs:0}ms" : $"{batchMs / 1000.0:0.0}s";
                string fail = batchOk < added.Count ? $" · 실패 {added.Count - batchOk}" : "";
                string rep = replicated > 0 ? $" · 기존배관 복제 {replicated}"
                           : replicated < 0 ? " · 기존배관 복제 생략(격자>30M셀 — 셀을 키우세요)" : "";
                Status = $"{label} 라우팅 완료 · 성공 {batchOk}/{added.Count}{fail}{rep} · 총 {batchTimeStr}   |   전체 누적 {sceneOk}/{Tasks.Count}";
                BuildRouteReport(added, label, batchOk, sceneOk, replicated);
                AnalysisReport = RouteResultReport;   // 우측 하단 '분석결과' 탭에 인라인 표시(다이얼로그 대신).
            }
            catch (Exception ex)
            {
                Status = "경로 탐색 오류: " + ex.Message;
            }
            finally
            {
                IsRouting = false;
                _cancelRequested = false;
                foreach (var r in ResultList) if (r.RunState != RouteRunState.Idle) r.RunState = RouteRunState.Idle;  // 진행 상태 잔류 방지(예외/취소 시).
                RefreshRouteProgress();   // 진행바를 최종 결과(완료/성공/실패)로 확정.
                System.Windows.Input.CommandManager.InvalidateRequerySuggested();
            }
        }

        // 진행 콜백에는 정확한 fail_reason 이 없으므로 즉시 표시용으로만 보수 추정한다.
        // 배치 종료 후 CacheResults 가 엔진의 LastFail 로 다시 덮어써 저장/리포트와 일치시킨다.
        private static Interop.RouteFail InferProgressFail(bool success, long expandedNodes)
        {
            if (success) return Interop.RouteFail.None;
            if (expandedNodes >= ShortestMaxExpansions() * 0.98) return Interop.RouteFail.ExpansionLimit;
            return expandedNodes <= 0 ? Interop.RouteFail.StartBlocked : Interop.RouteFail.NoPath;
        }

        private static long ShortestMaxExpansions()
        {
            var raw = System.Environment.GetEnvironmentVariable("R3D_SHORTEST_MAX_EXP");
            return long.TryParse(raw, out var value) && value > 0 ? value : 8_000_000L;
        }

        private static double MinStraightMmForRouting()
        {
            var raw = System.Environment.GetEnvironmentVariable("R3D_MIN_STRAIGHT_MM");
            return double.TryParse(raw, System.Globalization.NumberStyles.Float,
                       System.Globalization.CultureInfo.InvariantCulture, out var value) && value >= 0.0
                ? value
                : 0.0;   // 기본 OFF — 활성화 시 A* 상태가 RS배 증가(cell=25mm+300mm→RS=13, 13× 느림). env R3D_MIN_STRAIGHT_MM=300 으로 옵트인.
        }

        private void ConfigureSearchTraceIfEnabled(string label, int taskCount)
        {
            if (_engine == null) return;
            if (!EnableSearchTrace)
            {
                _engine.DisableTrace();
                return;
            }

            string dir = Path.Combine(AppContext.BaseDirectory, "logs");
            Directory.CreateDirectory(dir);
            string project = SelectedProject?.GroupName ?? SelectedProject?.Display ?? "scene";
            string safeProject = SafeFilePart(project);
            string safeLabel = SafeFilePart(label);
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string path = Path.Combine(dir, $"routing_trace_{stamp}_{safeProject}_{safeLabel}_{taskCount}tasks.r3dtrace.jsonl");
            int sampleEvery = TraceSampleEvery();
            int maxEvents = TraceMaxEventsPerTask();
            _engine.SetTrace(path, level: 1, sampleEvery: sampleEvery,
                             includeOccupancy: true, includeRejects: true,
                             includePostprocess: true, maxEventsPerTask: maxEvents);
            LastSearchTraceFile = path;
            Status = $"탐색 로그 기록: {path}";
        }

        private static int TraceSampleEvery()
        {
            var raw = Environment.GetEnvironmentVariable("R3D_TRACE_SAMPLE_EVERY");
            return int.TryParse(raw, out var value) && value > 0 ? value : 1000;
        }

        private static int TraceMaxEventsPerTask()
        {
            var raw = Environment.GetEnvironmentVariable("R3D_TRACE_MAX_EVENTS");
            return int.TryParse(raw, out var value) && value > 0 ? value : 20000;
        }

        private static string SafeFilePart(string? text)
        {
            var s = string.IsNullOrWhiteSpace(text) ? "route" : text.Trim();
            var invalid = Path.GetInvalidFileNameChars();
            var sb = new StringBuilder(s.Length);
            bool lastUnderscore = false;
            foreach (char ch in s)
            {
                bool keep = char.IsLetterOrDigit(ch) || ch == '-' || ch == '_';
                if (keep && !invalid.Contains(ch))
                {
                    sb.Append(ch);
                    lastUnderscore = false;
                }
                else if (!lastUnderscore)
                {
                    sb.Append('_');
                    lastUnderscore = true;
                }
            }
            s = sb.ToString().Trim('_');
            if (string.IsNullOrWhiteSpace(s)) s = "route";
            return s.Length > 40 ? s[..40] : s;
        }

        // ---- 단계별 탐색(선택 배관 A* 진행 애니메이션) ----
        /// <summary>선택 배관을 라우팅하고, A* 가 확장한 방문 셀을 '확장 순서대로' 점진 표시해
        /// 시작 PoC→종단 PoC 로 점유맵을 회피해 나아가는 과정을 애니메이션으로 보여준다.
        /// 탐색이 끝나면 최종 경로를 드러낸다.</summary>
        private async Task AnimateSelectedAsync()
        {
            if (_scene == null || _engine == null || SelectedTask == null || _animating) return;
            _animating = true;
            try
            {
                int idx = SelectedTask.Index;
                SearchModel = null;
                _hidePathsForAnim = true;        // 탐색 중에는 최종 경로를 숨긴다(끝에 드러냄).
                _showOccupancyVoxels = true;     // 회피 대상(점유맵)을 보여준다.
                OnChanged(nameof(ShowOccupancyVoxels));

                // 선택 작업 1개만 부분집합 라우팅 → 방문 셀(확장 순서) + 경로 산출.
                await RouteRowsAsync(new List<int> { idx }, $"단계별 탐색 #{idx}", corridor: false);

                var row = Tasks[idx];
                if (row.Visited.Length == 0)
                {
                    Status = $"#{idx}: 방문 셀이 없습니다(라우팅 실패 또는 방문 수집 off).";
                    _hidePathsForAnim = false;
                    BuildModel();
                    return;
                }

                await AnimateVisitedAsync(_scene.Grid, row.Visited, row.Success ? row.LengthMm : 0);
            }
            catch (Exception ex) { Status = "단계별 탐색 오류: " + ex.Message; }
            finally
            {
                _hidePathsForAnim = false;
                BuildModel();        // 최종 경로(튜브) 드러내기.
                _animating = false;
            }
        }

        // 방문 셀을 확장 순서대로 점진적으로 드러내는 DispatcherTimer 애니메이션.
        private Task AnimateVisitedAsync(GridMeta g, PathCell[] visited, double lengthMm)
        {
            var tcs = new TaskCompletionSource<bool>();
            // 표시 셀 상한(부드러운 재생용). 초과 시 순서를 보존한 채 균등 다운샘플.
            const int Cap = 18000;
            PathCell[] cells = visited;
            if (visited.Length > Cap)
            {
                cells = new PathCell[Cap];
                double stride = (double)visited.Length / Cap;
                for (int i = 0; i < Cap; i++) cells[i] = visited[(int)(i * stride)];
            }
            int total = cells.Length;
            const int Frames = 45;
            int perTick = Math.Max(1, (int)Math.Ceiling(total / (double)Frames));
            double s = g.CellMm * 0.55;   // 방문 큐브 변(경로보다 가늘게).
            int shown = 0;

            var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
            timer.Tick += (_, __) =>
            {
                shown = Math.Min(total, shown + perTick);
                var mb = new MeshBuilder(false, false);
                for (int i = 0; i < shown; i++) mb.AddBox(CellToWorld(g, cells[i]), s, s, s);
                SearchModel = Geometry(mb, Color.FromRgb(255, 205, 70), 110);   // 탐색 구름 = 노랑.
                Status = $"단계별 탐색… 방문 {shown:N0}/{total:N0}" + (lengthMm > 0 ? $"  (경로 {lengthMm:0} mm)" : "");
                if (shown >= total) { timer.Stop(); tcs.TrySetResult(true); }
            };
            timer.Start();
            return tcs.Task;
        }

        // ---- 3D 피킹(P3) ----
        private void SetPick(PickMode mode)
        {
            PickMode = mode;
            Status = mode == PickMode.Start ? "3D 뷰에서 시작점을 클릭하세요…" : "3D 뷰에서 끝점을 클릭하세요…";
        }

        /// <summary>3D 뷰 클릭 지점을 셀 중심으로 스냅해 선택 배관의 종단점으로 설정한다(코드비하인드 호출).</summary>
        public void ApplyPick(Point3D p)
        {
            if (_scene == null || SelectedTask == null || PickMode == PickMode.None) return;
            var g = _scene.Grid;
            int i = (int)Math.Floor((p.X - g.Ox) / g.CellMm);
            int j = (int)Math.Floor((p.Y - g.Oy) / g.CellMm);
            int k = (int)Math.Floor((p.Z - g.Oz) / g.CellMm);
            double x = g.Ox + (i + 0.5) * g.CellMm, y = g.Oy + (j + 0.5) * g.CellMm, z = g.Oz + (k + 0.5) * g.CellMm;
            if (PickMode == PickMode.Start) { SelectedTask.Sx = x; SelectedTask.Sy = y; SelectedTask.Sz = z; }
            else { SelectedTask.Gx = x; SelectedTask.Gy = y; SelectedTask.Gz = z; }
            Status = $"피킹: #{SelectedTask.Index} {(PickMode == PickMode.Start ? "시작" : "끝")}점=({x:0},{y:0},{z:0}). '선택 배관 재라우팅' 을 누르세요.";
            PickMode = PickMode.None;
        }

        private void RebuildIfReady()
        {
            if (_engine != null && _scene != null) BuildModel();
        }

        // ---- 3D 모델 구성 ----
        private void BuildModel()
        {
            var scene = _scene!;
            var grid = scene.Grid;
            UpdateGroundGrid(grid);   // 바닥 격자를 씬 좌표에 맞춤(ZoomExtents 가 객체를 중앙에 잡도록).
            var group = new Model3DGroup();
            Legend.Clear();
            SpaceLabels.Clear();

            // ⓪-A 공간 영역(토글) — TB_BIM_SPACE_INFO 의 CR/A/F/CSF 등을 선형(와이어프레임) + 텍스트로.
            if (ShowSpaces && scene.Spaces.Count > 0)
            {
                AddSpaceAreas(group, scene.Spaces);
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(230, 255, 196, 0)),
                    Label = $"공간 영역 ({scene.Spaces.Count}): {string.Join(", ", scene.Spaces.Select(s => s.Name).Distinct())}"
                });
            }

            // ⓪ 복셀 전체맵(토글) — 격자 BBOX 12변(가는 실린더). 작업 공간을 한눈에.
            if (ShowGridFrame)
            {
                AddGridFrame(group, grid);
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(220, 122, 223, 176)),
                    Label = $"복셀 전체맵 ({grid.Nx}×{grid.Ny}×{grid.Nz})"
                });
            }

            // ① 장애물(토글) — 일반 장애물(회색)과 통과 객체(청록)를 구분해 머지.
            //    통과 객체(바닥/천장/격자보)는 경로탐색 충돌에서 제외되므로 색으로 구분 표시한다.
            if (ShowObstacles && scene.Obstacles.Count > 0)
            {
                var mb = new MeshBuilder(false, false);       // 일반 장애물(충돌).
                var mbPass = new MeshBuilder(false, false);   // 통과 객체(비충돌).
                int nObs = 0, nPass = 0;
                foreach (var o in scene.Obstacles)
                {
                    var center = new Point3D((o.MinX + o.MaxX) / 2, (o.MinY + o.MaxY) / 2, (o.MinZ + o.MaxZ) / 2);
                    if (o.IsPassThrough) { mbPass.AddBox(center, o.MaxX - o.MinX, o.MaxY - o.MinY, o.MaxZ - o.MinZ); nPass++; }
                    else                 { mb.AddBox(center, o.MaxX - o.MinX, o.MaxY - o.MinY, o.MaxZ - o.MinZ); nObs++; }
                }
                if (nObs > 0)
                {
                    group.Children.Add(Geometry(mb, Color.FromRgb(150, 150, 150), 60));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(160, 150, 150, 150)), Label = $"장애물(obstacles) {nObs}" });
                }
                if (nPass > 0)
                {
                    group.Children.Add(Geometry(mbPass, Color.FromRgb(90, 200, 160), 55));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(160, 90, 200, 160)), Label = $"통과 객체(pass-through) {nPass}" });
                }
            }

            // ①-E 장비(토글) — TB_BIM_EQUIPMENT 의 AABB 를 주황 큐브 박스로(메인은 더 진하게).
            if (ShowEquipment && scene.Equipment.Count > 0)
            {
                var mbMain = new MeshBuilder(false, false);
                var mbSub = new MeshBuilder(false, false);
                int nMain = 0, nSub = 0;
                foreach (var eq in scene.Equipment)
                {
                    var center = new Point3D((eq.MinX + eq.MaxX) / 2, (eq.MinY + eq.MaxY) / 2, (eq.MinZ + eq.MaxZ) / 2);
                    if (eq.IsMain) { mbMain.AddBox(center, eq.MaxX - eq.MinX, eq.MaxY - eq.MinY, eq.MaxZ - eq.MinZ); nMain++; }
                    else           { mbSub.AddBox(center, eq.MaxX - eq.MinX, eq.MaxY - eq.MinY, eq.MaxZ - eq.MinZ); nSub++; }
                }
                if (nMain > 0) group.Children.Add(Geometry(mbMain, Color.FromRgb(255, 140, 0), 150));   // 메인=진한 주황.
                if (nSub > 0) group.Children.Add(Geometry(mbSub, Color.FromRgb(255, 190, 90), 90));      // 서브=옅은 주황.
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(190, 255, 140, 0)),
                    Label = $"장비(equipment) {scene.Equipment.Count} (메인 {nMain})"
                });
            }

            // ①-D 덕트/레터럴(각각 별도 토글) — TB_DUCT_LATERAL 의 AABB 를 박스로. 레터럴=초록, 덕트=청색.
            //    일부 덕트는 한 축 두께 0(굽힘) 이라 렌더 시 최소 두께로 클램프해 보이게 한다.
            if ((ShowLaterals || ShowDucts) && scene.DuctsLaterals.Count > 0)
            {
                var mbLat = new MeshBuilder(false, false);
                var mbDuct = new MeshBuilder(false, false);
                int nLat = 0, nDuct = 0;
                const double MinThick = 40;   // 0두께 박스 가시화용 최소 변(mm).
                foreach (var d in scene.DuctsLaterals)
                {
                    bool lateral = d.IsLateral;
                    if (lateral ? !ShowLaterals : !ShowDucts) continue;   // 해당 토글이 꺼져 있으면 스킵.
                    var center = new Point3D((d.MinX + d.MaxX) / 2, (d.MinY + d.MaxY) / 2, (d.MinZ + d.MaxZ) / 2);
                    double sx = Math.Max(d.MaxX - d.MinX, MinThick);
                    double sy = Math.Max(d.MaxY - d.MinY, MinThick);
                    double sz = Math.Max(d.MaxZ - d.MinZ, MinThick);
                    if (lateral) { mbLat.AddBox(center, sx, sy, sz); nLat++; }
                    else         { mbDuct.AddBox(center, sx, sy, sz); nDuct++; }
                }
                if (nLat > 0)
                {
                    group.Children.Add(Geometry(mbLat, Color.FromRgb(90, 210, 130), 150));   // 레터럴=초록.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(200, 90, 210, 130)), Label = $"레터럴(lateral) {nLat}" });
                }
                if (nDuct > 0)
                {
                    group.Children.Add(Geometry(mbDuct, Color.FromRgb(110, 175, 220), 130)); // 덕트=청색.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromArgb(200, 110, 175, 220)), Label = $"덕트(duct) {nDuct}" });
                }
            }

            // ①' 점유맵(토글) — 엔진이 voxelize 한 블록 셀을 셀 크기 큐브로(반투명 옅은 청회색).
            //    큐브가 맞닿아 장애물을 빈틈 없이 채운다. 상한 초과 시에만 균등 다운샘플(부하 한도).
            string occNote = string.Empty;
            if (ShowOccupancyVoxels && _engine != null)
            {
                var (rendered, occTotal) = AddOccupancyVoxels(group, grid);
                if (rendered > 0)
                {
                    bool down = rendered < occTotal;
                    Legend.Add(new LegendItem
                    {
                        Swatch = new SolidColorBrush(Color.FromArgb(170, 130, 170, 200)),
                        Label = down ? $"점유맵 (셀 {rendered:N0}/{occTotal:N0} 다운샘플)"
                                     : $"점유맵 (셀 {rendered:N0})"
                    });
                    if (down) occNote = $"   |   점유맵 {occTotal:N0}셀 중 {rendered:N0}만 표시(다운샘플)";
                }
            }

            // ①'' 통과 점유맵(별도 토글) — 바닥/천장/격자보 등 통과객체가 복셀화한 셀을 청록 큐브로.
            if (ShowPassthroughVoxels && _engine != null)
            {
                var ptCells = _engine.CopyPassthrough();
                if (ptCells.Length > 0)
                {
                    const int cap = 150_000;
                    int stride = ptCells.Length > cap ? ptCells.Length / cap : 1;
                    var mb = new MeshBuilder(false, false);
                    double s = grid.CellMm;
                    int cnt = 0;
                    for (int ci = 0; ci < ptCells.Length; ci += stride)
                    { mb.AddBox(CellToWorld(grid, ptCells[ci]), s, s, s); cnt++; }
                    if (cnt > 0)
                    {
                        group.Children.Add(Geometry(mb, Color.FromRgb(90, 200, 160), 90));
                        bool down = stride > 1;
                        Legend.Add(new LegendItem
                        {
                            Swatch = new SolidColorBrush(Color.FromArgb(170, 90, 200, 160)),
                            Label = down ? $"통과 점유맵 {cnt:N0}/{ptCells.Length:N0} 다운샘플"
                                         : $"통과 점유맵 {cnt:N0}셀"
                        });
                    }
                }
            }

            // ①''' 옥트리 가변셀 분해도(토글) — 자유공간 리프(크기별 녹색) + 장애물 리프(적색).
            //   대형 자유 리프 = 큰 박스(한 번에 점프 가능), 장애물 근처 = 작은 박스(정밀).
            _octreeLeafCount = 0;
            if (ShowOctreeVoxels && _engine != null)
                _octreeLeafCount = AddOctreeVoxels(group, grid);

            // ①'' 그룹배관 패턴(토글) — 기존설계에서 학습한 유틸별 '공용 트렁크 레인'(L4)을 보라색 반투명 큐브로.
            //    이 셀들이 곧 신규 라우팅에 주입되는 회랑(BuildBundleCorridorCells)이라, 자동경로가 이 레인을
            //    따라 다발로 뭉치는 걸 미리 볼 수 있다. 드릴다운(그룹/유틸 선택) 시 그 부분집합 유틸만 표시.
            if (ShowBundlePattern && _bundles != null)
            {
                var scopeRows = PatternScopeRows();
                var laneCells = BuildBundleCorridorCells(scopeRows, 2, includeVertical: true);   // 트렁크 레인 + 입상(표시).
                // 큐브 변 = 실제 관경×1.35(배관보다 살짝 크게) — 관경 미상이면 셀×0.9 폴백.
                double patDia = RepresentativePipeDia(scopeRows);
                double patCube = patDia > 0 ? patDia * 1.35 : grid.CellMm * 0.9;
                int shown = AddBundlePatternVoxels(group, grid, laneCells, patCube, 95, 120_000);
                if (shown > 0)
                {
                    var trunkZ = (MergeBundleLevels(null, scopeRows) ?? System.Array.Empty<int>())
                                 .Distinct().OrderBy(z => z).ToList();
                    int laneTotal = laneCells.Length / 3;
                    string tz = trunkZ.Count > 0 ? $" · 트렁크 z셀 {string.Join(",", trunkZ)}" : "";
                    string ds = shown < laneTotal ? $"/{laneTotal:N0} 다운샘플" : "";
                    Legend.Add(new LegendItem
                    {
                        Swatch = new SolidColorBrush(Color.FromArgb(170, BundlePatternColor.R, BundlePatternColor.G, BundlePatternColor.B)),
                        Label = $"그룹배관 패턴 레인 {shown:N0}{ds}셀{tz}"
                    });
                }
            }

            // ② 경로 — 유틸리티별 색 튜브 + 시작/끝 구. (충돌 계산용으로 경로는 항상 수집)
            // 단계별 탐색 애니메이션 중에는 최종 경로를 숨겨 탐색 과정만 보이게 한다(_hidePathsForAnim).
            bool selectedRouteFocus = _selectedTask != null && !_isRouting && !_animating;
            bool drawPaths = ShowPaths && !_hidePathsForAnim && !selectedRouteFocus;
            // 색 배정은 작업 + 기존배관 라벨을 합쳐 한 번에 한다(같은 유틸=같은 색, 라우팅 경로와 기존배관 색 일치).
            var colorMap = UtilityColors.Assign(
                scene.Tasks.Select(t => t.UtilityLabel)
                    .Concat(scene.ExistingPipes.Select(p => p.Label)));
            // 자동(개발) 경로 = 유틸리티별 머지 메시. 색은 '유틸 색을 밝게(Lighten)' 한 같은 계열 색으로 그린다.
            //   같은 유틸의 기존 설계배관(원래 유틸 색)보다 밝아 '사람 설계 vs 자동 설계'를 색으로 구분한다.
            var perUtil = new Dictionary<string, MeshBuilder>();
            var perUtilCenter = new Dictionary<string, MeshBuilder>(); // A* 중심선(얇은 흰 선, 반투명 파이프 내부 가시).
            var perUtilStub = new Dictionary<string, MeshBuilder>();   // 스텁 구간 강조(유틸 밝은색, 장비/덕트 진입부 식별).
            var perUtilVisited = new Dictionary<string, MeshBuilder>();   // 방문맵 — 유틸리티별 머지 메시.
            var perUtilVisitedCount = new Dictionary<string, int>();      // 표시 셀 카운트(다운샘플 후).
            var successPaths = new List<PathCell[]>();
            double fallbackTubeDia = grid.CellMm * 0.7;   // 관경 미상 시 격자 기반 기본 지름.
            double markerR = grid.CellMm * 0.9;
            double visitedBoxSize = grid.CellMm * 0.5;  // 방문 셀 큐브 변(작게 — 경로보다 가늘게).
            const int VisitedCapPerUtility = 12000;     // 유틸리티당 표시 상한(WPF 부하 한도).
            int ok = 0;
            double total = 0;

            // 경로는 행 캐시(TaskRowVM.Path/Visited)에서 읽는다 — 엔진은 부분집합 라우팅마다
            // 재구성되어 인덱스가 행과 1:1 이 아니므로, 렌더는 엔진 상태와 분리한다.
            foreach (var row in Tasks)
            {
                bool hasLoadedPoly = row.LoadedPolylinePts != null && row.LoadedPolylinePts.Count >= 2;
                if (!row.Success || (row.Path.Length == 0 && !hasLoadedPoly)) continue;

                ok++;
                total += row.LengthMm;
                successPaths.Add(row.Path);

                string label = row.Label;
                var uf = UtilityFilters.FirstOrDefault(u => u.Label == label);
                bool utilVisible = uf == null || uf.IsVisible;

                // 경로 튜브(ShowPaths + 유틸 가시 일 때) — 실제 관경(매칭 기존배관 SOURCE_SIZE)으로 그린다.
                if (drawPaths && utilVisible)
                {
                    if (!perUtil.TryGetValue(label, out var mb))
                    {
                        mb = new MeshBuilder(false, false);
                        perUtil[label] = mb;
                    }
                    // 관경: 행 캐시값 우선, 없으면 매칭 기존배관에서 1회 조회해 캐시(이후 재빌드 비용 절감).
                    if (row.DiameterMm <= 0)
                    {
                        var exm = FindMatchingExistingPipe(row);
                        row.DiameterMm = (exm != null && exm.DiameterMm > 0) ? exm.DiameterMm : 0;
                    }
                    double routeDia = row.DiameterMm > 0 ? Math.Max(row.DiameterMm, 8.0) : fallbackTubeDia;
                    // 표시 경로: DB 불러오기 좌표 우선, 없으면 [출발 스텁]+[A* 중간]+[reverse(종단 스텁)].
                    List<Point3D> pts;
                    if (row.LoadedPolylinePts != null && row.LoadedPolylinePts.Count >= 2)
                    {
                        pts = row.LoadedPolylinePts;
                    }
                    else
                    {
                        pts = new List<Point3D>();
                        if (row.StartStub != null)
                            pts.AddRange(row.StartStub.Select(p => new Point3D(p.X, p.Y, p.Z)));
                        pts.AddRange(row.Path.Select(c => CellToWorld(grid, c)));
                        if (row.EndStub != null)
                            for (int k = row.EndStub.Count - 1; k >= 0; k--)
                                pts.Add(new Point3D(row.EndStub[k].X, row.EndStub[k].Y, row.EndStub[k].Z));
                    }
                    if (pts.Count >= 2) mb.AddTube(pts, routeDia, 10, false);
                    mb.AddSphere(pts[0], markerR);
                    mb.AddSphere(pts[^1], markerR);

                    // A* 중심선 — 관경의 12%(최소 6mm) 얇은 선으로 반투명 파이프 내부 경로 뼈대를 표시.
                    double centerDia = Math.Max(routeDia * 0.12, 6.0);
                    if (!perUtilCenter.TryGetValue(label, out var ctrMb)) { ctrMb = new MeshBuilder(false, false); perUtilCenter[label] = ctrMb; }
                    var ctrPts = row.Path.Select(pc => CellToWorld(grid, pc)).ToList();
                    if (ctrPts.Count >= 2) ctrMb.AddTube(ctrPts, centerDia, 6, false);

                    // 스텁 강조 — 출발·종단 스텁 구간을 별도 밝은 색 얇은 튜브로 표시(장비면/덕트 진입부 식별).
                    if (!perUtilStub.TryGetValue(label, out var stMb)) { stMb = new MeshBuilder(false, false); perUtilStub[label] = stMb; }
                    double stubHighDia = centerDia * 2.0;
                    if (row.StartStub != null && row.StartStub.Count >= 2)
                        stMb.AddTube(row.StartStub.Select(sp => new Point3D(sp.X, sp.Y, sp.Z)).ToList(), stubHighDia, 6, false);
                    if (row.EndStub != null && row.EndStub.Count >= 2)
                        stMb.AddTube(row.EndStub.Select(ep => new Point3D(ep.X, ep.Y, ep.Z)).ToList(), stubHighDia, 6, false);
                }

                // 방문맵 — 유틸리티별 머지 메시(다운샘플링으로 셀 수 상한).
                if (ShowVisitedMap && utilVisible && row.Visited.Length > 0)
                {
                    if (!perUtilVisited.TryGetValue(label, out var vmb))
                    {
                        vmb = new MeshBuilder(false, false);
                        perUtilVisited[label] = vmb;
                        perUtilVisitedCount[label] = 0;
                    }
                    int already = perUtilVisitedCount[label];
                    int remaining = VisitedCapPerUtility - already;
                    if (remaining > 0)
                    {
                        // 균등 다운샘플: 셀 수가 remaining 보다 많으면 stride 로 솎아낸다.
                        int len = row.Visited.Length;
                        int take = Math.Min(remaining, len);
                        double stride = (double)len / take;
                        for (int s = 0; s < take; s++)
                        {
                            int idx = (int)(s * stride);
                            var c = row.Visited[idx];
                            var p = CellToWorld(grid, c);
                            vmb.AddBox(p, visitedBoxSize, visitedBoxSize, visitedBoxSize);
                        }
                        perUtilVisitedCount[label] = already + take;
                    }
                }
            }

            if (drawPaths && perUtil.Count > 0)
            {
                foreach (var kv in perUtil)
                {
                    var baseColor = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    var lit = Lighten(baseColor, 0.55);   // 자동 경로 = 유틸 색을 밝게(기존 설계보다 밝은 같은 계열).
                    group.Children.Add(Geometry(kv.Value, lit, 140)); // 반투명(alpha=140, ~55%) — 내부 중심선이 보이게.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(lit), Label = $"자동 {kv.Key} (밝게·관경)" });
                }
                // A* 경로 중심선 — 흰색 얇은 선(alpha=210), 반투명 파이프 안에서 경로 뼈대를 보여준다.
                foreach (var kv in perUtilCenter)
                    group.Children.Add(Geometry(kv.Value, Colors.White, 210));
                // 스텁 강조 — 유틸 색 매우 밝게(alpha=230), 장비면/덕트 진입부 스텁 구간을 식별.
                foreach (var kv in perUtilStub)
                {
                    var stubBase = colorMap.TryGetValue(kv.Key, out var sc) ? sc : Colors.Gray;
                    group.Children.Add(Geometry(kv.Value, Lighten(stubBase, 0.82), 230));
                }
            }

            // ①-X 기존 설계배관(토글) — TB_ROUTE_PATH 폴리라인을 유틸리티 색 튜브로(월드 mm 좌표 그대로).
            //   각 배관은 DB 의 실제 관경(SOURCE_SIZE→DiameterMm)으로 그린다(겹침 방지). 유틸 필터도 동일 적용.
            // 비교 포커스(_compareMode)에서는 나머지 기존배관을 숨긴다(선택 배관의 기존 경로만 CompareModel 로 강조).
            if (ShowExistingPipes && !_compareMode && !selectedRouteFocus && scene.ExistingPipes.Count > 0)
            {
                double fallbackDia = Math.Min(grid.CellMm * 0.4, 50);   // 관경 미상 시 기본 지름(mm).
                var perUtilEx = new Dictionary<string, MeshBuilder>();
                // 그룹배관 강조 모드 — 탐지된 번들(route_bundle_group) 멤버를 그룹별 고유 색으로, 비멤버는 흐리게.
                bool bundleHi = _showBundleGroups && _bundles != null && _bundles.GroupCount > 0;
                var perGroup = new Dictionary<int, MeshBuilder>();        // group_id → 머지 메시(그룹 색).
                var nonMemberMb = new MeshBuilder(false, false);          // 번들 미소속 기존배관(흐린 회색).
                int nonMemberCnt = 0;
                // 출발(빨강)·종단(파랑) 스텁 강조 — 학습 StubExtractor 로 잘라낸 수직+엘보 구간을 굵은 색 튜브로.
                var startStubMb = new MeshBuilder(false, false);
                var endStubMb = new MeshBuilder(false, false);
                int drawn = 0, stubDrawn = 0;
                foreach (var pipe in scene.ExistingPipes)
                {
                    // 좌측에서 유틸리티 그룹을 선택했으면 그 그룹의 기존 설계배관만 표시(미선택=전체).
                    if (!string.IsNullOrEmpty(_selectedGroup) && GroupKey(pipe.Group) != _selectedGroup) continue;
                    string label = pipe.Label;
                    var uf = UtilityFilters.FirstOrDefault(u => u.Label == label);
                    if (uf != null && !uf.IsVisible) continue;   // 유틸 체크박스 필터 적용.
                    if (pipe.Points.Count < 2) continue;
                    var pts = pipe.Points.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList();
                    // 실제 관경(외경) 사용. 너무 가늘면 안 보이므로 최소 8mm 로 클램프.
                    double dia = pipe.DiameterMm > 0 ? Math.Max(pipe.DiameterMm, 8.0) : fallbackDia;
                    // 머지 대상 선택: 그룹배관 강조면 그룹 멤버=그룹 메시 / 비멤버=흐린 메시, 아니면 유틸 색 메시.
                    MeshBuilder mb;
                    if (bundleHi)
                    {
                        int gid = _bundles!.GroupIdOf(pipe.RoutePathGuid);
                        if (gid >= 0)
                        {
                            if (!perGroup.TryGetValue(gid, out var groupMesh)) { groupMesh = new MeshBuilder(false, false); perGroup[gid] = groupMesh; }
                            mb = groupMesh;
                        }
                        else { mb = nonMemberMb; nonMemberCnt++; }
                    }
                    else
                    {
                        if (!perUtilEx.TryGetValue(label, out var utilMesh))
                        {
                            utilMesh = new MeshBuilder(false, false);
                            perUtilEx[label] = utilMesh;
                        }
                        mb = utilMesh;
                    }
                    mb.AddTube(pts, dia, 10, false);
                    drawn++;

                    // 출발/종단 스텁(수직배관 + 엘보) — 학습과 동일 로직으로 잘라 빨강/파랑 튜브로 강조.
                    // 실제 관경보다 살짝 굵게(×1.35) + 반투명으로 그린다 → 실제 배관(원색·불투명)을 감싸는
                    // 투명 셸로 보여, 스텁 강조와 배관 본체가 동시에 보인다.
                    if (ShowStubs)
                    {
                        double stubDia = dia * 1.35;
                        var (startStub, endStub) = StubExtractor.ForPipe(pipe);
                        if (startStub.Count >= 2)
                        {
                            startStubMb.AddTube(startStub.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList(), stubDia, 10, false);
                            stubDrawn++;
                        }
                        if (endStub.Count >= 2)
                            endStubMb.AddTube(endStub.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList(), stubDia, 10, false);
                    }
                }
                if (bundleHi)
                {
                    // 비멤버 기존배관은 흐린 회색으로(다발이 도드라지게).
                    if (nonMemberCnt > 0) group.Children.Add(Geometry(nonMemberMb, Color.FromRgb(90, 100, 120), 90));
                    foreach (var kv in perGroup.OrderBy(k => k.Key))
                        group.Children.Add(Geometry(kv.Value, BundleGroupColor(kv.Key), 245));
                    Legend.Add(new LegendItem
                    {
                        Swatch = new SolidColorBrush(BundleGroupColor(0)),
                        Label = $"그룹배관 {perGroup.Count}그룹 (멤버 {drawn - nonMemberCnt} · 비멤버 {nonMemberCnt})"
                    });
                }
                else
                {
                    // 기존 설계배관 = 유틸리티 색(원색). 같은 유틸 자동 경로는 이 색을 밝게 한 색이라 구분된다.
                    foreach (var kv in perUtilEx)
                    {
                        var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                        group.Children.Add(Geometry(kv.Value, color, 235));
                    }
                    if (drawn > 0)
                        Legend.Add(new LegendItem
                        {
                            Swatch = new SolidColorBrush(Color.FromArgb(235, 160, 160, 160)),
                            Label = $"기존 설계배관 {drawn} (유틸 색·관경)"
                        });
                }
                if (ShowStubs && stubDrawn > 0)
                {
                    group.Children.Add(Geometry(startStubMb, Color.FromRgb(226, 48, 48), 120));   // 출발 스텁 = 빨강(반투명 셸).
                    group.Children.Add(Geometry(endStubMb, Color.FromRgb(48, 112, 255), 120));    // 종단 스텁 = 파랑(반투명 셸).
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(226, 48, 48)), Label = $"출발 스텁 {stubDrawn}" });
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(48, 112, 255)), Label = "종단 스텁" });
                }
            }

            // PoC 마커 — 모든 작업(장비)의 시작 PoC(빨강 구)·종단 PoC(파랑 구)를 라우팅 전에도 표시.
            //   유틸 체크박스 필터 + 좌측 선택 그룹을 동일 적용(경로/기존배관 레이어와 일관). 두 색을 머지 메시로.
            if (ShowPocMarkers && !selectedRouteFocus && Tasks.Count > 0)
            {
                double pocR = Math.Max(grid.CellMm * 0.9, 50);
                var startMb = new MeshBuilder(false, false);
                var endMb = new MeshBuilder(false, false);
                int pocN = 0;
                foreach (var row in Tasks)
                {
                    if (!string.IsNullOrEmpty(_selectedGroup) && GroupKey(row.Group) != _selectedGroup) continue;
                    var uf = UtilityFilters.FirstOrDefault(u => u.Label == row.Label);
                    if (uf != null && !uf.IsVisible) continue;
                    startMb.AddSphere(new Point3D(row.Sx, row.Sy, row.Sz), pocR);
                    endMb.AddSphere(new Point3D(row.Gx, row.Gy, row.Gz), pocR);
                    pocN++;
                }
                if (pocN > 0)
                {
                    group.Children.Add(Geometry(startMb, Color.FromRgb(255, 45, 45), 235));    // 시작 PoC = 빨강.
                    group.Children.Add(Geometry(endMb, Color.FromRgb(50, 120, 255), 235));     // 종단 PoC = 파랑.
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(255, 45, 45)), Label = $"시작 PoC {pocN}" });
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Color.FromRgb(50, 120, 255)), Label = $"종단 PoC {pocN}" });
                }
            }

            // 방문맵 — 유틸리티별 색의 반투명 큐브 집합. 경로와 같은 색 규약.
            if (ShowVisitedMap && perUtilVisited.Count > 0)
            {
                int totalShown = 0;
                foreach (var kv in perUtilVisited)
                {
                    var color = colorMap.TryGetValue(kv.Key, out var c) ? c : Colors.Gray;
                    group.Children.Add(Geometry(kv.Value, color, 80));   // alpha 80 = 약 31% 불투명.
                    totalShown += perUtilVisitedCount[kv.Key];
                }
                Legend.Add(new LegendItem
                {
                    Swatch = new SolidColorBrush(Color.FromArgb(120, 200, 200, 200)),
                    Label = $"방문맵 (셀 {totalShown:N0})"
                });
            }

            // ③ 충돌(토글) — ≥2 배관이 공유하는 셀을 빨간 큐브로.
            int collisions = 0;
            if (ShowCollisions)
            {
                var cells = CollisionFinder.Find(successPaths);
                collisions = cells.Count;
                if (cells.Count > 0)
                {
                    var cmb = new MeshBuilder(false, false);
                    double s = grid.CellMm * 0.9;
                    foreach (var (ci, cj, ck) in cells)
                        cmb.AddBox(CellToWorld(grid, new PathCell(ci, cj, ck)), s, s, s);
                    group.Children.Add(Geometry(cmb, Colors.Magenta, 235));
                    Legend.Add(new LegendItem { Swatch = new SolidColorBrush(Colors.Magenta), Label = $"충돌(collision) {cells.Count}" });
                }
            }

            SceneModel = group;
            RenderObjectCount = group.Children.Count;
            OctreeLeafCount   = _octreeLeafCount;
            UpdateSelectionHighlight();   // 라우팅 후 선택 경로의 꺾임 마커·단계 목록 갱신.
            Status = $"장애물 {scene.Obstacles.Count} · 작업 {scene.Tasks.Count} · 성공 {ok}/{scene.Tasks.Count} · 총 {total:0} mm · 충돌 {collisions}{occNote}   |   engine: {Engine.Version}";
            SceneRebuilt?.Invoke();
        }

        private static Point3D CellToWorld(GridMeta g, PathCell c) =>
            new(g.Ox + (c.I + 0.5) * g.CellMm, g.Oy + (c.J + 0.5) * g.CellMm, g.Oz + (c.K + 0.5) * g.CellMm);

        // 그룹배관 패턴(L4) 시각화 — 공용 색(보라). 점유(청회)·경로(파랑)·방문(노랑)·충돌(적색)과 구별된다.
        private static readonly Color BundlePatternColor = Color.FromRgb(190, 120, 235);

        // 그룹배관 패턴 표시 범위 — 좌측 드릴다운(그룹/유틸 선택) 시 그 부분집합만, 미선택 시 전체 작업.
        // (메인 뷰는 선택된 그룹의 패턴만 보여줘 혼잡을 줄인다. 미니 뷰는 단일 배관 유틸로 별도 호출.)
        private List<int> PatternScopeRows()
        {
            if (!string.IsNullOrEmpty(_selectedGroup) && !string.IsNullOrEmpty(_selectedUtility))
                return RowsWhere(t => GroupKey(t.Group) == _selectedGroup && UtilityKey(t.Utility) == _selectedUtility);
            if (!string.IsNullOrEmpty(_selectedGroup))
                return RowsWhere(t => GroupKey(t.Group) == _selectedGroup);
            return AllRows();
        }

        // 범위 행들의 유틸리티에 해당하는 기존배관 관경의 중앙값(mm). 패턴/표시를 '실제 관경 기준'으로
        //   잡는 데 쓴다. 관경 정보가 없으면 0(호출자가 셀 기반 기본값으로 폴백).
        private double RepresentativePipeDia(IReadOnlyList<int> rows)
        {
            var s = _scene; if (s == null || s.ExistingPipes.Count == 0) return 0;
            var utils = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var pos in rows) if (!string.IsNullOrEmpty(Tasks[pos].Utility)) utils.Add(Tasks[pos].Utility!);
            var dias = new List<double>();
            foreach (var p in s.ExistingPipes)
                if (p.DiameterMm > 0 && p.Utility != null && utils.Contains(p.Utility)) dias.Add(p.DiameterMm);
            if (dias.Count == 0) return 0;
            dias.Sort();
            return dias[dias.Count / 2];
        }

        // 그룹배관 패턴 레인 셀(평탄 ijk)을 반투명 큐브로 그린다. cubeMm = 큐브 변(실제 관경보다 살짝 크게
        // 잡아 레인이 배관을 감싸는 투명 셸로 보이게). 상한 초과 시 균등 다운샘플. 반환: 실제 표시한 셀 수.
        private static int AddBundlePatternVoxels(Model3DGroup group, GridMeta g, int[] ijk, double cubeMm, byte alpha, int cap)
        {
            int total = ijk.Length / 3;
            if (total == 0) return 0;
            int take = Math.Min(cap, total);
            double stride = (double)total / take;
            double s = cubeMm > 0 ? cubeMm : g.CellMm * 0.6;
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++)
            {
                int idx = (int)(n * stride) * 3;
                mb.AddBox(CellToWorld(g, new PathCell(ijk[idx], ijk[idx + 1], ijk[idx + 2])), s, s, s);
            }
            group.Children.Add(Geometry(mb, BundlePatternColor, alpha));
            return take;
        }

        // 라이브 오버레이 초기화(배치 라우팅 시작 시). UI 스레드에서 호출.
        private void ResetLiveRoute()
        {
            _liveGroup = null;
            LiveRouteModel = null;
        }

        // 배관 1개(경로 셀)를 유틸 색 튜브로 라이브 오버레이에 추가(완료 즉시 3D 갱신). UI 스레드에서 호출.
        private void AppendLivePipe(PathCell[] path, Color color, GridMeta grid)
        {
            if (path.Length < 1) return;
            if (_liveGroup == null) { _liveGroup = new Model3DGroup(); LiveRouteModel = _liveGroup; }
            var mb = new MeshBuilder(false, false);
            var pts = path.Select(c => CellToWorld(grid, c)).ToList();
            double dia = grid.CellMm * 0.7, mr = grid.CellMm * 0.9;
            if (pts.Count >= 2) mb.AddTube(pts, dia, 8, false);
            mb.AddSphere(pts[0], mr);
            mb.AddSphere(pts[^1], mr);
            // Model3DGroup 의 Children 변경은 바인딩된 ModelVisual3D 를 즉시 다시 렌더(재대입 불필요).
            _liveGroup.Children.Add(Geometry(mb, color, 235));
        }

        // 선택 PoC 의 시작(초록)·끝(노랑) 점을 강조 구로 그린다. 라우팅과 무관 — 선택 즉시 갱신,
        // 전체 모델을 다시 만들지 않고 별도 오버레이(SelectionModel)만 교체한다(대형 장면에서도 가볍게).
        private void UpdateSelectionHighlight()
        {
            _suppressStepNav = true;
            PathSteps.Clear();
            _suppressStepNav = false;
            StepHighlightModel = null;   // 새 배관 선택 → 이전 구간 강조 제거.
            var t = _selectedTask;
            if (t == null || _scene == null)
            {
                SelectionModel = null;
                CompareModel = null; ComparisonReport = null; _comparePipe = null;
                RouteStepRows.Clear();   // 선택 해제 → 단계별 경로 그리드 비움.
                // 선택 해제 → 분석결과 탭을 배치 전체 리포트(있으면)로 되돌린다.
                if (!_isRouting)
                    AnalysisReport = string.IsNullOrEmpty(RouteResultReport) ? null : RouteResultReport;
                return;
            }
            // 결과 리스트에서 배관 클릭 → 분석결과 탭에 이 배관의 단계별 상세 분석(시작→꺾임(사유)→종단 + 특징점).
            if (!_isRouting) BuildSelectedTaskAnalysis(t);
            var g = _scene.Grid;
            double r = Math.Max(g.CellMm * 1.6, 80);
            var grp = new Model3DGroup();
            var s = new MeshBuilder(false, false);
            s.AddSphere(new Point3D(t.Sx, t.Sy, t.Sz), r);
            grp.Children.Add(Geometry(s, Color.FromRgb(255, 45, 45), 235));    // 시작 PoC = 빨강.
            var e = new MeshBuilder(false, false);
            e.AddSphere(new Point3D(t.Gx, t.Gy, t.Gz), r);
            grp.Children.Add(Geometry(e, Color.FromRgb(50, 120, 255), 235));   // 종단 PoC = 파랑.

            // 경로가 있으면 방향 전환(꺾임) 지점을 마젠타 구로 표시 + 구간 단계 리스트 구성.
            BuildPathSteps(g, t.Path, grp);

            SelectionModel = grp;
            UpdateComparison();   // 선택 배관 ↔ 기존 설계경로 매칭 오버레이 + 비교 분석 갱신.
        }

        // ========================================================================
        // 선택 배관 단계별 상세 분석 — '분석결과' 탭(시작 → 꺾임(사유) → 종단 + 특징점 적용)
        // ========================================================================
        // 결과 리스트에서 배관을 클릭하면, 자동설계가 그 한 배관을 어떻게 풀었는지를 사람이 읽을 수 있게
        // 풀어 쓴다: ① 개요(관경/길이/꺾임/순서/시간) ② 적용된 특징점(학습 스텁·표면투영·매칭·이격)
        // ③ 단계별 경로(직선 구간 + 각 꺾임의 사유 분류) ④ 실패 시 원인 진단.
        //   분석은 화면에 그려진 렌더 폴리라인([출발 스텁] + [A* 경로] + [reverse(종단 스텁)])을 그대로
        //   대상으로 한다(= 사용자가 보는 배관과 1:1). 스텁 구간은 '학습 형상', A* 구간은 '엔진 탐색'.
        private void BuildSelectedTaskAnalysis(TaskRowVM t)
        {
            var s = _scene; if (s == null) { return; }
            var g = s.Grid;
            var sb = new StringBuilder();
            RouteStepRows.Clear();   // '단계별 경로' 그리드 초기화 — AppendRouteSteps 가 다시 채운다.

            string util = string.IsNullOrEmpty(t.Utility) ? t.Label : t.Utility!;
            string grpName = string.IsNullOrEmpty(t.Group) ? "-" : t.Group!;
            sb.AppendLine($"# 자동설계 경로 분석 — #{t.Index} {util}");
            sb.AppendLine();

            // ── ① 개요 ────────────────────────────────────────────────────────
            sb.AppendLine("## ① 개요");
            sb.AppendLine($"- 유틸리티 / 그룹 : {util} / {grpName}");
            string startNm = string.IsNullOrEmpty(t.PocName) ? "(이름없음)" : t.PocName!;
            string endNm = string.IsNullOrEmpty(t.EndName) ? "(이름없음)" : t.EndName!;
            sb.AppendLine($"- 출발(PoC) → 종단 : {startNm} → {endNm}");
            sb.AppendLine($"- 관경 : {(t.DiameterMm > 0 ? t.DiameterMm.ToString("0") + " mm" : "미상(격자 기본)")}");
            string modeStr = _routingMode switch
            {
                RoutingMode.Shortest => "최단경로(순수 A*)",
                RoutingMode.FollowExisting => "기존설계 추종(복제)",
                _ => "특징점 반영(스텁+패턴)"
            };
            sb.AppendLine($"- 라우팅 모드 : {modeStr}");

            if (t.Success && t.Path.Length >= 2)
            {
                var bends = CountBends(t.Path);
                sb.AppendLine($"- 결과 : ✅ 성공 · 길이 {t.LengthMm:N0} mm · 꺾임 {bends.Text}");
            }
            else
            {
                sb.AppendLine("- 결과 : ❌ 실패 / 미라우팅");
            }
            if (t.RouteOrder >= 0)
                sb.AppendLine($"- 설계 순서 : {t.RouteOrder + 1}번째 (굵은 배관 우선)");
            if (t.Attempted && t.ElapsedMs > 0)
                sb.AppendLine($"- 소요 시간 : {(t.ElapsedMs < 1000 ? $"{t.ElapsedMs:0} ms" : $"{t.ElapsedMs / 1000.0:0.0} s")}");
            if (t.ExpandedNodes > 0)
                sb.AppendLine($"- 탐색 노드 : {t.ExpandedNodes:N0} 개 확장");
            sb.AppendLine();

            // ── ② 적용된 특징점(학습 내용) ─────────────────────────────────────
            sb.AppendLine("## ② 적용된 특징점(학습 반영)");
            bool anyFeature = false;
            if (t.StartStub != null && t.StartStub.Count >= 2)
            {
                double slen = PolylineLengthPt(t.StartStub);
                sb.AppendLine($"- 출발 스텁 : 기존설계 학습 형상(라이저+엘보) 적용 — {t.StartStub.Count}점 · 약 {slen:0} mm");
                sb.AppendLine("  └ A* 탐색은 이 스텁 끝(랙 위 자유공간)부터 시작 — 장비 출발부는 사람설계 형상 그대로.");
                anyFeature = true;
            }
            if (t.EndStub != null && t.EndStub.Count >= 2)
            {
                double elen = PolylineLengthPt(t.EndStub);
                sb.AppendLine($"- 종단 스텁 : 기존설계 학습 형상(덕트/레터럴 진입 엘보) 적용 — {t.EndStub.Count}점 · 약 {elen:0} mm");
                anyFeature = true;
            }
            // PoC 표면투영/자유셀 스냅 — 원본 PoC(장비/덕트 내부 솔리드)와 실제 A* 시작/끝이 다르면 표시.
            if (t.HasRouteEndpoints)
            {
                double ds = Dist(new Pt3(t.Sx, t.Sy, t.Sz), new Pt3(t.RouteSx, t.RouteSy, t.RouteSz));
                double de = Dist(new Pt3(t.Gx, t.Gy, t.Gz), new Pt3(t.RouteGx, t.RouteGy, t.RouteGz));
                if (ds > g.CellMm * 1.5)
                {
                    sb.AppendLine($"- 출발 PoC 보정 : 표면투영/자유셀 스냅 Δ{ds:0} mm (장비 내부 매몰 PoC → 접근 가능한 자유 셀로 이동).");
                    anyFeature = true;
                }
                if (de > g.CellMm * 1.5)
                {
                    sb.AppendLine($"- 종단 PoC 보정 : 표면투영/자유셀 스냅 Δ{de:0} mm.");
                    anyFeature = true;
                }
            }
            // 매칭 기존배관(복제/회랑 추종 근거).
            var match = FindMatchingExistingPipe(t);
            if (match != null)
            {
                sb.AppendLine($"- 기존설계 매칭 : '{match.Label}'"
                    + (_routingMode == RoutingMode.FollowExisting ? " — 이 형상을 복제(막힌 구간만 국소 수리)." : " — 회랑 소프트 바이어스로 추종."));
                anyFeature = true;
            }
            // 관경 → per-task 이격 반경(다음 배관이 이 배관 표면을 피하도록 막는 셀 반경).
            if (t.DiameterMm > 0)
            {
                int rad = Math.Clamp((int)Math.Ceiling(t.DiameterMm / g.CellMm) - 1, 0, 8);
                sb.AppendLine($"- 이격 마킹 : 관경 {t.DiameterMm:0} mm → 반경 {rad}셀 점유(인접 배관이 표면을 침범하지 않게).");
                anyFeature = true;
            }
            if (!anyFeature)
                sb.AppendLine("- (적용된 학습 특징점 없음 — 순수 A* 탐색. 최단경로 모드이거나 매칭 기존설계 부재.)");
            sb.AppendLine();

            // ── ③ 단계별 경로(시작 → 꺾임(사유) → 종단) ────────────────────────
            sb.AppendLine("## ③ 단계별 경로 (시작 → 꺾임 → 종단)");
            if (!t.Success || t.Path.Length < 2)
            {
                sb.AppendLine("- 경로가 없어 단계 분석을 표시할 수 없습니다.");
            }
            else
            {
                AppendRouteSteps(sb, t, g);
            }
            sb.AppendLine();

            // ── ④ 실패 원인 진단 ───────────────────────────────────────────────
            if (!t.Success)
            {
                sb.AppendLine("## ④ 실패 원인 진단");
                sb.AppendLine("- " + DiagnoseFailure(t, g));
                sb.AppendLine();
            }

            AnalysisReport = sb.ToString().TrimEnd();
        }

        // 렌더 폴리라인을 직선 구간으로 쪼개고, 구간 사이 꺾임마다 사유를 분류해 단계별로 출력한다.
        private void AppendRouteSteps(StringBuilder sb, TaskRowVM t, GridMeta g)
        {
            // 화면에 그려진 그대로의 폴리라인(스텁 + A* + 종단 스텁) + 구간별 영역 태그.
            var pts = new List<Point3D>();
            var region = new List<int>();   // 0=출발스텁, 1=A* 탐색, 2=종단스텁.
            if (t.StartStub != null)
                foreach (var p in t.StartStub) { pts.Add(new Point3D(p.X, p.Y, p.Z)); region.Add(0); }
            foreach (var c in t.Path) { pts.Add(CellToWorld(g, c)); region.Add(1); }
            if (t.EndStub != null)
                for (int k = t.EndStub.Count - 1; k >= 0; k--) { pts.Add(new Point3D(t.EndStub[k].X, t.EndStub[k].Y, t.EndStub[k].Z)); region.Add(2); }

            // 인접 중복점 제거.
            for (int i = pts.Count - 1; i >= 1; i--)
                if ((pts[i] - pts[i - 1]).Length < 1e-6) { pts.RemoveAt(i); region.RemoveAt(i); }
            if (pts.Count < 2) { sb.AppendLine("- 경로 점이 부족합니다."); return; }

            // 직선 구간 경계(방향이 바뀌는 정점) 인덱스 수집.
            var runEnd = new List<int>();          // 각 구간의 끝 정점 인덱스.
            Vector3D Dir(int a, int b) { var d = pts[b] - pts[a]; if (d.Length > 1e-9) d.Normalize(); return d; }
            var cur = Dir(0, 1);
            int runStart = 0;
            var runs = new List<(int a, int b)>();
            for (int i = 1; i < pts.Count; i++)
            {
                Vector3D d = (i + 1 < pts.Count) ? Dir(i, i + 1) : cur;
                double dot = cur.X * d.X + cur.Y * d.Y + cur.Z * d.Z;
                if (i + 1 >= pts.Count) { runs.Add((runStart, pts.Count - 1)); break; }
                if (dot < 0.999)   // 방향 전환 = 구간 종료(정점 i 가 꺾임점).
                {
                    runs.Add((runStart, i));
                    runStart = i;
                    cur = d;
                }
            }

            string startNm = string.IsNullOrEmpty(t.PocName) ? "출발 PoC" : t.PocName!;
            string endNm = string.IsNullOrEmpty(t.EndName) ? "종단" : t.EndName!;
            var p0 = pts[0];
            sb.AppendLine($"▶ 시작 : {startNm}  ({p0.X:0}, {p0.Y:0}, {p0.Z:0})");
            // 그리드: 시작 행.
            RouteStepRows.Add(new RouteStepRow
            {
                Seq = "시작", Kind = "시작", Region = startNm,
                Reason = $"({p0.X:0}, {p0.Y:0}, {p0.Z:0})",
                A = p0, B = p0
            });

            for (int r = 0; r < runs.Count; r++)
            {
                var (a, b) = runs[r];
                var va = pts[a]; var vb = pts[b];
                var seg = vb - va;
                double len = seg.Length;
                string axis = AxisLabel(seg);
                string midLabel = _routingMode == RoutingMode.FollowExisting ? "복제 경로" : "엔진 탐색";
                string reg = region[Math.Min(b, region.Count - 1)] switch
                {
                    0 => "학습 스텁(출발)", 2 => "학습 스텁(종단)", _ => midLabel
                };
                sb.AppendLine($"  {r + 1,2}. {axis}  {len:0} mm   [{reg}]");

                // 구간 끝이 꺾임점이면(마지막 구간 제외) 사유 분류.
                string reason = "";
                if (r < runs.Count - 1)
                {
                    var dIn = seg; if (dIn.Length > 1e-9) dIn.Normalize();
                    var nextSeg = pts[runs[r + 1].b] - pts[runs[r + 1].a];
                    var dOut = nextSeg; if (dOut.Length > 1e-9) dOut.Normalize();
                    reason = ClassifyBendReason(dIn, dOut, vb, r, runs.Count, region[Math.Min(b, region.Count - 1)], g);
                    sb.AppendLine($"      └ 꺾임 #{r + 1} : {reason}");
                }
                // 그리드: 구간 행.
                RouteStepRows.Add(new RouteStepRow
                {
                    Seq = (r + 1).ToString(), Kind = "구간",
                    Direction = axis, Length = $"{len:N0} mm", Region = reg, Reason = reason,
                    A = va, B = vb
                });
            }

            var pN = pts[pts.Count - 1];
            sb.AppendLine($"■ 종단 : {endNm}  ({pN.X:0}, {pN.Y:0}, {pN.Z:0})");
            // 그리드: 종단 행.
            RouteStepRows.Add(new RouteStepRow
            {
                Seq = "종단", Kind = "종단", Region = endNm,
                Reason = $"({pN.X:0}, {pN.Y:0}, {pN.Z:0})",
                A = pN, B = pN
            });
        }

        // 꺾임 사유 분류 — 입·출 방향, 위치(영역/끝 근접), 장애물 인접으로 4범주(라이저/랙 전환/장애물 회피/정렬).
        private string ClassifyBendReason(Vector3D dIn, Vector3D dOut, Point3D bend,
                                          int runIndex, int runCount, int region, GridMeta g)
        {
            bool vertical = Math.Abs(dIn.Z) > 0.7 || Math.Abs(dOut.Z) > 0.7;
            bool nearStart = runIndex <= 1;
            bool nearEnd = runIndex >= runCount - 2;

            if (vertical)
            {
                // 수직 성분이 있는 꺾임: 끝단 근처 = 라이저(장비/덕트 접속 높이 맞춤), 중간 = 랙 z-레벨 전환.
                if (region == 0 || region == 2 || nearStart || nearEnd)
                {
                    bool up = dIn.Z > 0.7 || dOut.Z > 0.7;
                    return $"라이저 ({(up ? "수직 상승 ↑" : "수직 하강 ↓")}) — 장비/덕트 접속 높이로 올리거나 내림";
                }
                return "랙 전환 — 다른 z-레벨(랙 단)으로 이동";
            }

            // 수평 꺾임: 직진(입력 방향 연장)이 장애물에 막혔는지 탐침 → 막혔으면 '장애물 회피'.
            for (int k = 1; k <= 3; k++)
            {
                var probe = bend + dIn * (g.CellMm * k);
                if (CellBlocked(probe.X, probe.Y, probe.Z))
                {
                    string blocker = BlockingObjectAt(probe.X, probe.Y, probe.Z) ?? "차단 객체";
                    return $"장애물 회피 - {blocker} / 직진 방향 앞 셀이 막혀 우회";
                }
            }
            return "정렬 — 랙/레인 정렬 또는 경로 직교화(다른 배관·혼잡 회피 포함)";
        }

        // 방향 벡터 → 사람이 읽는 축 라벨.
        private static string AxisLabel(Vector3D d)
        {
            double ax = Math.Abs(d.X), ay = Math.Abs(d.Y), az = Math.Abs(d.Z);
            if (az >= ax && az >= ay) return d.Z >= 0 ? "수직 ↑ (Z+)" : "수직 ↓ (Z-)";
            if (ax >= ay) return d.X >= 0 ? "수평 → (X+)" : "수평 ← (X-)";
            return d.Y >= 0 ? "수평 ↗ (Y+)" : "수평 ↙ (Y-)";
        }

        private static double PolylineLengthPt(System.Collections.Generic.List<Pt3> pts)
        {
            double len = 0;
            for (int i = 1; i < pts.Count; i++) len += Dist(pts[i - 1], pts[i]);
            return len;
        }

        // 실패 원인 간단 진단 — 탐색 노드 수/실패 사유로 분류(상세 카테고리).
        private string DiagnoseFailure(TaskRowVM t, GridMeta g)
        {
            // 엔진이 보고한 실패 사유(A1)가 있으면 우선.
            switch (t.LastFail)
            {
                case Interop.RouteFail.StartBlocked:
                    return "출발 셀 막힘 — 출발 PoC가 장애물에 매몰되어 첫 셀에서 진행 불가(표면투영/스냅 실패).";
                case Interop.RouteFail.GoalBlocked:
                    return "종단 셀 막힘 — 목적지 PoC가 장애물에 매몰되어 도달 불가.";
                case Interop.RouteFail.CorridorMiss:
                    return "회랑 이탈 — 계층 corridor 가이드 범위 안에서 경로를 찾지 못함(범위 밖 우회 필요).";
                case Interop.RouteFail.ExpansionLimit:
                    return $"탐색 상한 초과 — {t.ExpandedNodes:N0} 노드까지 확장했으나 경로 미발견(혼잡/막힘). 셀을 키우거나 CBS/rip-up 권장.";
                case Interop.RouteFail.GoalDirBlocked:
                    return "목표 진입축 막힘 — 지정한 진입 방향으로 종단 접속 불가(무제약 폴백도 실패).";
                case Interop.RouteFail.NoPath:
                    return $"경로 없음 — {t.ExpandedNodes:N0} 노드 확장 후에도 도달 실패(국소 차단/혼잡).";
            }
            if (t.ExpandedNodes == 0)
                return "출발조차 못함 — 출발 셀이 막혔거나 격자 밖일 가능성(표면투영/스냅 확인).";
            return $"경로 없음 — {t.ExpandedNodes:N0} 노드 확장 후에도 도달 실패(국소 차단/혼잡).";
        }

        // ---- 기존설계 비교(선택 배관) ----
        /// <summary>'🔬 기존설계 비교' — 선택 배관을 단일 라우팅(개발 경로)하고 비교 포커스 모드로 진입한다.
        /// 비교 포커스에서는 나머지 기존 설계배관 레이어를 숨겨(선택 1개만) 두 경로를 또렷이 대조한다.</summary>
        private async Task CompareSelectedAsync()
        {
            if (_selectedTask == null || _scene == null) return;
            int idx = _selectedTask.Index;
            _compareMode = true;   // BuildModel 가드 → 나머지 기존배관 숨김.
            // RouteRowsAsync 가 단일 충돌회피 라우팅 후 BuildModel→UpdateComparison 까지 수행
            // (개발 경로 포함 전체 분석: 길이·꺾임·종단점·장애물 간섭/여유).
            await RouteRowsAsync(new List<int> { idx }, $"기존설계 비교 #{idx}", corridor: false);
            if (!_selectedTask.Success)
                Status = $"#{idx}: 라우팅 실패 — 개발 경로 없이 기존 경로만 비교합니다.";
        }

        // 선택 배관에 매칭되는 기존 설계경로를 찾아 오버레이/분석을 갱신한다.
        // (UpdateSelectionHighlight 끝과 CompareSelectedAsync→BuildModel 끝에서 호출됨.)
        private void UpdateComparison()
        {
            var t = _selectedTask;
            if (t == null || _scene == null || _scene.ExistingPipes.Count == 0)
            {
                _comparePipe = null;
                CompareModel = null;
                ComparisonReport = _scene == null ? null
                    : "기존 설계배관 데이터가 없습니다(scene.txt 로드 또는 TB_ROUTE_PATH 부재).";
                return;
            }
            _comparePipe = FindMatchingExistingPipe(t);
            BuildComparison(t, _comparePipe);
        }

        // 선택 배관(시작/끝 PoC 좌표)과 가장 잘 맞는 기존 설계경로를 찾는다.
        //   점수 = 양방향 중 작은 (시작↔SOURCE_POS 거리 + 끝↔TARGET_POS 거리). SourcePos/TargetPos 가
        //   없는 행은 폴리라인 양 끝점으로 폴백. 종단점 합산 거리가 임계 초과면 매칭 없음(null).
        private ExistingPipe? FindMatchingExistingPipe(TaskRowVM t)
        {
            var s = _scene; if (s == null || s.ExistingPipes.Count == 0) return null;
            var ts = new Pt3(t.Sx, t.Sy, t.Sz);
            var te = new Pt3(t.Gx, t.Gy, t.Gz);
            double tol = Math.Max(3 * s.Grid.CellMm, 1500.0);   // 종단점당 허용 거리(mm).
            ExistingPipe? best = null; double bestScore = double.MaxValue;
            foreach (var p in s.ExistingPipes)
            {
                if (p.Points.Count < 2) continue;
                Pt3 ps = p.SourcePos ?? p.Points[0];
                Pt3 pe = p.TargetPos ?? p.Points[p.Points.Count - 1];
                double score = Math.Min(Dist(ts, ps) + Dist(te, pe), Dist(ts, pe) + Dist(te, ps));
                if (score < bestScore) { bestScore = score; best = p; }
            }
            return (best != null && bestScore <= tol * 2) ? best : null;   // 시작+끝 합산이므로 *2.
        }

        // 매칭된 기존 경로(ex)와 선택 배관(t)의 개발 경로를 정량 비교해 ComparisonReport + CompareModel 갱신.
        private void BuildComparison(TaskRowVM t, ExistingPipe? ex)
        {
            var s = _scene!; var g = s.Grid;
            var devPts = t.Success ? GetRoutedPolyline(t, g) : new List<Point3D>();
            bool devRouted = t.Success && devPts.Count >= 2;

            if (ex == null)
            {
                ComparisonReport =
                    "매칭되는 기존 설계경로 없음.\n(이 PoC에 해당하는 TB_ROUTE_PATH 가 없거나 종단점이 너무 멉니다.)"
                    + (devRouted ? $"\n\n개발 경로: {t.LengthMm:0} mm · 꺾임 {CountBends(t.Path).Text}" : "");
                CompareModel = BuildCompareOverlay(null, devRouted ? devPts : null);
                return;
            }

            var exPts = ex.Points.Select(p => new Point3D(p.X, p.Y, p.Z)).ToList();
            double exLen = PolylineLength(exPts);
            BendStats exBends = CountBends(exPts);

            var sb = new StringBuilder();
            sb.AppendLine($"매칭: {ex.Label}");
            sb.AppendLine($"관경: {(ex.DiameterMm > 0 ? ex.DiameterMm.ToString("0") + " mm" : "미상")}");

            // 종단점 정합 — 양방향 중 가까운 쪽으로 매칭(역방향이면 표기).
            var ts = new Pt3(t.Sx, t.Sy, t.Sz); var te = new Pt3(t.Gx, t.Gy, t.Gz);
            Pt3 ps = ex.SourcePos ?? new Pt3(exPts[0].X, exPts[0].Y, exPts[0].Z);
            Pt3 pe = ex.TargetPos ?? new Pt3(exPts[exPts.Count - 1].X, exPts[exPts.Count - 1].Y, exPts[exPts.Count - 1].Z);
            bool rev = (Dist(ts, pe) + Dist(te, ps)) < (Dist(ts, ps) + Dist(te, pe));
            double sg = rev ? Dist(ts, pe) : Dist(ts, ps);
            double eg = rev ? Dist(te, ps) : Dist(te, pe);
            sb.AppendLine($"종단점 정합: 시작 {sg:0}mm · 끝 {eg:0}mm{(rev ? " (역방향)" : "")}");
            sb.AppendLine();

            sb.AppendLine("─ 경로 길이 ─");
            if (devRouted)
            {
                double diff = t.LengthMm - exLen;
                double pct = exLen > 1 ? diff / exLen * 100 : 0;
                sb.AppendLine($"  기존 {exLen:0} / 개발 {t.LengthMm:0} mm");
                sb.AppendLine($"  차이 {(diff >= 0 ? "+" : "")}{diff:0} mm ({(pct >= 0 ? "+" : "")}{pct:0.0}%) — 개발 {(diff < 0 ? "짧음" : "긺")}");
            }
            else sb.AppendLine($"  기존 {exLen:0} mm / 개발 (미라우팅)");

            sb.AppendLine();
            sb.AppendLine("─ 꺾임(엘보) 수 ─");
            sb.AppendLine($"  기존 {exBends.Text}");
            if (devRouted)
            {
                var devBends = CountBends(t.Path);
                sb.AppendLine($"  개발 {devBends.Text}");
            }
            else sb.AppendLine("  개발 —");

            sb.AppendLine();
            sb.AppendLine("─ 장애물 간섭 / 여유 ─");
            int exHit = CountObstacleHits(exPts, out double exClr, out int exSamples);
            sb.AppendLine($"  기존: 간섭 {exHit}/{exSamples}점 · 최소여유 {exClr:0} mm");
            if (devRouted)
            {
                int devHit = CountObstacleHits(devPts, out double devClr, out int devSamples);
                sb.AppendLine($"  개발: 간섭 {devHit}/{devSamples}점 · 최소여유 {devClr:0} mm");
                if (exHit > 0 && devHit == 0)
                    sb.AppendLine("  → 개발 경로가 기존의 장애물 간섭을 해소");
            }
            else sb.AppendLine("  개발: (미라우팅 — '🔬 기존설계 비교' 실행)");

            ComparisonReport = sb.ToString().TrimEnd();
            CompareModel = BuildCompareOverlay(exPts, devRouted ? devPts : null);
        }

        // 비교 오버레이: 기존 경로(주황) + 개발 경로(시안) 굵은 튜브 + 시작/끝 구 마커.
        private Model3D? BuildCompareOverlay(List<Point3D>? exPts, List<Point3D>? devPts)
        {
            if (_scene == null) return null;
            bool hasEx = exPts != null && exPts.Count >= 2;
            bool hasDev = devPts != null && devPts.Count >= 2;
            if (!hasEx && !hasDev) return null;
            double cell = _scene.Grid.CellMm;
            double dia = Math.Max(cell * 0.9, 60);   // 일반 경로보다 굵게(눈에 띄게).
            double mr = Math.Max(cell * 1.1, 70);
            var grp = new Model3DGroup();
            if (hasEx)
            {
                var mb = new MeshBuilder(false, false);
                mb.AddTube(exPts, dia, 12, false);
                mb.AddSphere(exPts![0], mr); mb.AddSphere(exPts[exPts.Count - 1], mr);
                grp.Children.Add(Geometry(mb, Color.FromRgb(255, 144, 48), 245));   // 기존 = 주황.
            }
            if (hasDev)
            {
                var mb = new MeshBuilder(false, false);
                mb.AddTube(devPts, dia, 12, false);
                mb.AddSphere(devPts![0], mr); mb.AddSphere(devPts[devPts.Count - 1], mr);
                grp.Children.Add(Geometry(mb, Color.FromRgb(255, 64, 190), 255));   // 자동경로 = 분홍.
            }
            return grp;
        }

        // 폴리라인을 step 간격으로 샘플해 비통과 장애물 AABB 와의 간섭(내부 점 수)과 최소 여유(mm)를 잰다.
        //   간섭 = 장애물 내부에 든 샘플 점 수. 여유 = 외부 점에서 가장 가까운 장애물 표면까지 최소 거리.
        private int CountObstacleHits(List<Point3D> pts, out double minClearance, out int sampleCount)
        {
            minClearance = double.MaxValue; sampleCount = 0;
            var s = _scene;
            if (s == null || pts.Count < 2) { minClearance = 0; return 0; }
            double step = Math.Max(s.Grid.CellMm * 0.5, 25);
            int hits = 0;
            var samples = SamplePolyline(pts, step);
            sampleCount = samples.Count;
            foreach (var pt in samples)
            {
                double nearest = double.MaxValue; bool inside = false;
                foreach (var o in s.Obstacles)
                {
                    if (o.IsPassThrough) continue;
                    double d = DistToAabb(pt, o, out bool ins);
                    if (ins) { inside = true; break; }
                    if (d < nearest) nearest = d;
                }
                if (inside) hits++;
                else if (nearest < minClearance) minClearance = nearest;
            }
            if (minClearance == double.MaxValue) minClearance = 0;
            return hits;
        }

        // 점에서 AABB 까지의 외부(유클리드) 거리. 점이 박스 내부면 0 이고 inside=true.
        private static double DistToAabb(Point3D p, ObstacleBox o, out bool inside)
        {
            double dx = Math.Max(Math.Max(o.MinX - p.X, p.X - o.MaxX), 0);
            double dy = Math.Max(Math.Max(o.MinY - p.Y, p.Y - o.MaxY), 0);
            double dz = Math.Max(Math.Max(o.MinZ - p.Z, p.Z - o.MaxZ), 0);
            inside = dx == 0 && dy == 0 && dz == 0;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        // 폴리라인을 step(mm) 간격 점열로 균등 샘플(각 구간 시작점 포함, 끝점 포함).
        private static List<Point3D> SamplePolyline(List<Point3D> pts, double step)
        {
            var outp = new List<Point3D>();
            if (pts.Count == 0) return outp;
            outp.Add(pts[0]);
            for (int i = 1; i < pts.Count; i++)
            {
                Point3D a = pts[i - 1], b = pts[i];
                double len = (b - a).Length;
                int n = Math.Max(1, (int)(len / step));
                for (int k = 1; k <= n; k++)
                {
                    double f = (double)k / n;
                    outp.Add(new Point3D(a.X + (b.X - a.X) * f, a.Y + (b.Y - a.Y) * f, a.Z + (b.Z - a.Z) * f));
                }
            }
            return outp;
        }

        private static double PolylineLength(List<Point3D> pts)
        {
            double L = 0;
            for (int i = 1; i < pts.Count; i++) L += (pts[i] - pts[i - 1]).Length;
            return L;
        }

        private static double Dist(Pt3 a, Pt3 b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>꺾임(엘보) 통계 — 전체 수와 수평/수직 분류. 수직 = 인접 구간 중 하나라도 Z(상하)
        /// 방향인 꺾임(층 전환 엘보), 수평 = XY 평면 내 방향 전환.</summary>
        private readonly struct BendStats
        {
            public readonly int Total, Horiz, Vert;
            public BendStats(int total, int horiz, int vert) { Total = total; Horiz = horiz; Vert = vert; }
            /// <summary>"N회 (수평 H · 수직 V)" 표기.</summary>
            public string Text => $"{Total}회 (수평 {Horiz} · 수직 {Vert})";
        }

        // 격자 경로(셀)의 방향 전환(꺾임) 통계 — BuildPathSteps 의 Dir 부호 비교 규약과 동일.
        //   꺾임 직전/직후 구간 중 하나라도 Z 성분(dz≠0)이면 수직 꺾임, 아니면 수평 꺾임으로 센다.
        private static BendStats CountBends(PathCell[] path)
        {
            if (path.Length < 3) return new BendStats(0, 0, 0);
            (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));
            int total = 0, horiz = 0, vert = 0;
            var cur = Dir(path[0], path[1]);
            for (int i = 2; i < path.Length; i++)
            {
                var d = Dir(path[i - 1], path[i]);
                if (d != cur)
                {
                    total++;
                    if (cur.dz != 0 || d.dz != 0) vert++; else horiz++;
                    cur = d;
                }
            }
            return new BendStats(total, horiz, vert);
        }

        // 월드 폴리라인의 방향 전환(꺾임) 통계 — 인접 구간 단위벡터 내적이 ~1 미만이면 한 번 꺾인 것으로.
        //   꺾임 직전/직후 구간 중 하나라도 |Z| 성분이 지배적(>0.7≈45°↑)이면 수직 꺾임, 아니면 수평 꺾임.
        private static BendStats CountBends(List<Point3D> pts)
        {
            if (pts.Count < 3) return new BendStats(0, 0, 0);
            int total = 0, horiz = 0, vert = 0;
            Vector3D? prev = null;
            for (int i = 1; i < pts.Count; i++)
            {
                Vector3D d = pts[i] - pts[i - 1];
                if (d.Length < 1e-6) continue;
                d.Normalize();
                if (prev.HasValue)
                {
                    var p = prev.Value;
                    double dot = p.X * d.X + p.Y * d.Y + p.Z * d.Z;
                    if (dot < 0.999)   // ~2.6° 이상 차이 → 꺾임.
                    {
                        total++;
                        if (Math.Abs(p.Z) > 0.7 || Math.Abs(d.Z) > 0.7) vert++; else horiz++;
                    }
                }
                prev = d;
            }
            return new BendStats(total, horiz, vert);
        }

        // 경로 단계(구간) 강조 — 카메라는 그대로 두고(현재 화면 유지), 선택 구간만 흰색 굵은 튜브로 덧그린다.
        private void HighlightStep(PathStep? step)
        {
            if (step == null || _scene == null) { StepHighlightModel = null; return; }
            var g = _scene.Grid;
            var mb = new MeshBuilder(false, false);
            double dia = Math.Max(g.CellMm * 1.8, 95);
            if (step.A != step.B) mb.AddCylinder(step.A, step.B, dia, 12);
            double r = Math.Max(g.CellMm * 1.15, 65);
            mb.AddSphere(step.A, r);
            mb.AddSphere(step.B, r);
            StepHighlightModel = Geometry(mb, Color.FromRgb(255, 255, 255), 255);   // 선택 구간 = 흰색 강조.
        }

        // 경로 셀을 직선 구간(같은 축으로 진행)별로 나눠, 방향이 바뀌는 꺾임점을 마커로 찍고
        // 각 구간을 PathSteps 에 담는다(클릭 시 해당 위치로 카메라 이동).
        private void BuildPathSteps(GridMeta g, PathCell[] path, Model3DGroup grp)
        {
            if (path.Length < 2) return;
            double bendR = Math.Max(g.CellMm * 1.1, 60);
            var bendMb = new MeshBuilder(false, false);

            // 각 스텝의 단위 방향(부호) 벡터.
            (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));

            int segStart = 0;
            var curDir = Dir(path[0], path[1]);
            int stepNo = 1;
            for (int i = 2; i <= path.Length; i++)
            {
                var d = (i < path.Length) ? Dir(path[i - 1], path[i]) : (int.MinValue, 0, 0);
                if (d != curDir)
                {
                    // 구간 [segStart .. i-1] 종료.
                    var pStart = CellToWorld(g, path[segStart]);
                    var pEnd = CellToWorld(g, path[i - 1]);
                    double len = (pEnd - pStart).Length;
                    PathSteps.Add(new PathStep
                    {
                        Label = $"{stepNo,2}. {DirText(curDir)}  {len:0} mm",
                        Position = pStart,
                        A = pStart,
                        B = pEnd,
                    });
                    stepNo++;
                    // 꺾임점(구간 끝 = 다음 구간 시작) 마커. 마지막(끝점)은 제외.
                    if (i < path.Length) bendMb.AddSphere(pEnd, bendR);
                    segStart = i - 1;
                    if (i < path.Length) curDir = Dir(path[i - 1], path[i]);
                }
            }
            if (bendMb.Positions != null && bendMb.Positions.Count > 0)
                grp.Children.Add(Geometry(bendMb, Color.FromRgb(255, 80, 220), 240));   // 꺾임 = 마젠타.
        }

        // 단위 방향 벡터 → '수직(Z)' / '수평(X)' / '수평(Y)' 라벨.
        private static string DirText((int dx, int dy, int dz) d)
        {
            if (d.dz != 0) return d.dz > 0 ? "수직 ↑(Z+)" : "수직 ↓(Z-)";
            if (d.dx != 0) return d.dx > 0 ? "수평 →(X+)" : "수평 ←(X-)";
            if (d.dy != 0) return d.dy > 0 ? "수평 ↗(Y+)" : "수평 ↙(Y-)";
            return "—";
        }

        // 격자 BBOX 의 12 변을 가는 실린더로 그린다(복셀 전체맵 = 작업 공간 프레임).
        private static void AddGridFrame(Model3DGroup group, GridMeta g)
        {
            var lo = new Point3D(g.Ox, g.Oy, g.Oz);
            var hi = new Point3D(g.Ox + g.Nx * g.CellMm, g.Oy + g.Ny * g.CellMm, g.Oz + g.Nz * g.CellMm);
            AddBoxFrame(group, lo, hi, Color.FromRgb(122, 223, 176), Math.Max(g.CellMm * 0.08, 5), 230);
        }

        // 공간 영역(CR/A/F/CSF 등)을 영역별 색 와이어프레임으로 그리고, 중앙 상단에 텍스트 라벨을 둔다.
        private void AddSpaceAreas(Model3DGroup group, List<SpaceArea> spaces)
        {
            var colorMap = UtilityColors.Assign(spaces.Select(s => s.Name));   // 이름 기준 결정적 색.
            foreach (var sp in spaces)
            {
                var lo = new Point3D(sp.MinX, sp.MinY, sp.MinZ);
                var hi = new Point3D(sp.MaxX, sp.MaxY, sp.MaxZ);
                var color = colorMap.TryGetValue(sp.Name, out var c) ? c : Colors.Gold;
                // 변 굵기 = 영역 크기에 비례(너무 가늘면 안 보이고 너무 굵으면 장애물 가림).
                double r = Math.Max((hi.X - lo.X + hi.Y - lo.Y) * 0.0008, 25);
                AddBoxFrame(group, lo, hi, color, r, 235);
                // 텍스트 라벨 위치 = 영역 박스 '바깥'(+X 면에서 더 떨어진 곳), 각 층의 수직 중앙.
                // 층(CSF/A/F/CR)이 Z 로 쌓이므로 같은 옆면에 서로 다른 높이로 나란히 표시된다.
                double offset = Math.Max((hi.X - lo.X) * 0.06, 800);
                SpaceLabels.Add(new SpaceLabel
                {
                    Text = sp.Name,
                    Position = new Point3D(hi.X + offset, (lo.Y + hi.Y) / 2, (lo.Z + hi.Z) / 2),
                    Color = color,
                });
            }
        }

        // 임의의 AABB(lo~hi) 12 변을 실린더 와이어프레임으로 그린다(격자/공간 영역 공용).
        private static void AddBoxFrame(Model3DGroup group, Point3D lo, Point3D hi, Color color, double radius, byte alpha)
        {
            var corners = new[]
            {
                new Point3D(lo.X,lo.Y,lo.Z), new Point3D(hi.X,lo.Y,lo.Z), new Point3D(hi.X,hi.Y,lo.Z), new Point3D(lo.X,hi.Y,lo.Z),
                new Point3D(lo.X,lo.Y,hi.Z), new Point3D(hi.X,lo.Y,hi.Z), new Point3D(hi.X,hi.Y,hi.Z), new Point3D(lo.X,hi.Y,hi.Z),
            };
            var edges = new (int, int)[]
            {
                (0,1),(1,2),(2,3),(3,0), (4,5),(5,6),(6,7),(7,4), (0,4),(1,5),(2,6),(3,7)
            };
            var mb = new MeshBuilder(false, false);
            foreach (var (a, b) in edges) mb.AddCylinder(corners[a], corners[b], radius, 8);
            group.Children.Add(Geometry(mb, color, alpha));
        }

        // 점유맵 — 엔진이 voxelize 한 블록 셀을 '셀 크기' 큐브로(반투명). 큐브가 맞닿아 장애물을
        // 빈틈 없이 채운다. 상한(Cap) 초과 시에만 균등 다운샘플(메시 부하 한도).
        // 반환값 = (실제 그린 셀 수, 전체 블록 셀 수). down=rendered<total 이면 다운샘플됨.
        private (int rendered, int total) AddOccupancyVoxels(Model3DGroup group, GridMeta g)
        {
            // 큐브 1개 ≈ 12삼각형. 다운샘플 모드는 15만 상한(~180만 삼각형, 단일 병합 메시).
            // 원본 모드(_occupancyFullRes)는 상한 없이 전체 셀 표시(대형 장면에선 느릴 수 있음 — 사용자 선택).
            int cap = _occupancyFullRes ? int.MaxValue : 150_000;
            var cells = _occupancyFullRes ? _engine!.CopyBlocked() : _engine!.CopyBlockedSampled(cap);
            if (cells.Length == 0) return (0, 0);
            int take = Math.Min(cap, cells.Length);
            double stride = (double)cells.Length / take;
            double s = g.CellMm;   // 셀 크기와 동일 → 인접 큐브가 맞닿아 빈틈 없이 채움(이전 0.9 → 점박이).
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++)
            {
                var c = cells[(int)(n * stride)];
                mb.AddBox(CellToWorld(g, c), s, s, s);
            }
            group.Children.Add(Geometry(mb, Color.FromRgb(130, 170, 200), 120));   // 옅은 청회색 반투명.
            return (take, cells.Length);
        }

        // 옥트리 가변셀 분해도 — FREE 리프를 트리 레벨(깊이)별 와이어프레임 큐브로 표시.
        //   Level 0(가장 큰 박스=황색) → Level N(가장 작은 박스=회색) 방향으로 색상 변화.
        //   큰 박스 = 넓은 자유공간(점프 A* 가 한 번에 크게 이동),
        //   작은 박스 = 장애물 인근 세분화 구역(정밀 탐색 필요).
        //   BLOCKED 리프는 점유맵에서 이미 표시되므로 생략.
        //   레벨당 상한: max(50, 500/(level+1)) 균등 다운샘플.
        // 반환값: 실제 렌더한 총 리프 수.
        private int AddOctreeVoxels(Model3DGroup group, GridMeta g)
        {
            using var octreeEngine = BuildOctreePreviewEngine(g);
            var leaves = octreeEngine.EnumOctreeLeaves(500_000);
            if (leaves.Length == 0) return 0;

            // 루트 한 변(셀) = 2^ceil(log2(max(Nx,Ny,Nz))),  루트 크기(mm) = 루트한변 × cell_mm
            int maxN = Math.Max(g.Nx, Math.Max(g.Ny, g.Nz));
            int rootSideCells = 1;
            while (rootSideCells < maxN) rootSideCells <<= 1;
            double rootSizeMm = rootSideCells * g.CellMm;

            // FREE / BLOCKED 리프를 Level(트리 깊이)별로 각각 분류
            // level = round(log2(rootSizeMm / leafSizeMm))
            var freeByLevel    = new SortedDictionary<int, List<OctreeLeaf>>();
            var blockedByLevel = new SortedDictionary<int, List<OctreeLeaf>>();
            foreach (var leaf in leaves)
            {
                int level = leaf.SizeMm >= rootSizeMm * 0.99f ? 0
                    : (int)Math.Round(Math.Log(rootSizeMm / leaf.SizeMm) / Math.Log(2.0));
                level = Math.Max(0, level);
                var dict = leaf.State == 0 ? freeByLevel : blockedByLevel;
                if (!dict.TryGetValue(level, out var lst))
                    dict[level] = lst = new List<OctreeLeaf>();
                lst.Add(leaf);
            }

            // FREE 팔레트: 파스텔~선명 계열 — A*가 큰 구역 한 번에 점프
            Color[] freePalette = {
                Color.FromRgb(255, 220,  40),  // L0  황색  (대형 자유구역)
                Color.FromRgb(255, 140,  20),  // L1  주황
                Color.FromRgb( 40, 220, 200),  // L2  청록
                Color.FromRgb( 60, 180,  80),  // L3  녹색
                Color.FromRgb( 60, 120, 240),  // L4  파랑
                Color.FromRgb(180,  60, 220),  // L5  보라
                Color.FromRgb(220,  80, 130),  // L6  핑크
                Color.FromRgb(160, 160, 160),  // L7+ 회색
            };

            // BLOCKED 팔레트: 적색 계열 — 장애물이 꽉 찬 구역
            Color[] blockedPalette = {
                Color.FromRgb(220,  30,  30),  // L0  진적  (대형 장애물 — 슬래브·바닥)
                Color.FromRgb(220,  60,  40),  // L1  주황적
                Color.FromRgb(200,  80,  50),  // L2
                Color.FromRgb(180,  60,  60),  // L3
                Color.FromRgb(160,  50,  70),  // L4
                Color.FromRgb(140,  50,  80),  // L5
                Color.FromRgb(120,  40,  80),  // L6
                Color.FromRgb(100,  40,  80),  // L7+
            };

            int totalRendered = 0;

            // ── FREE 레이어 렌더 ──────────────────────────────────────────────
            foreach (var (level, lst) in freeByLevel)
            {
                int capLevel = Math.Max(50, 500 / (level + 1));
                int take     = Math.Min(capLevel, lst.Count);
                if (take <= 0) continue;
                double stride = (double)lst.Count / take;
                float  repSz  = lst[0].SizeMm;
                double radius = Math.Max(3.0, repSz * 0.008);
                Color  col    = freePalette[Math.Min(level, freePalette.Length - 1)];
                var    mb     = new MeshBuilder(false, false);
                for (int i = 0; i < take; i++)
                {
                    var lf = lst[(int)(i * stride)];
                    float sz = lf.SizeMm;
                    AddBoxFrameToMesh(mb, new(lf.X0Mm, lf.Y0Mm, lf.Z0Mm),
                                          new(lf.X0Mm+sz, lf.Y0Mm+sz, lf.Z0Mm+sz), radius);
                    totalRendered++;
                }
                group.Children.Add(Geometry(mb, col, 230));
                bool down = lst.Count > take;
                Legend.Add(new LegendItem {
                    Swatch = new SolidColorBrush(Color.FromArgb(230, col.R, col.G, col.B)),
                    Label  = $"빈공간 L{level} ({repSz:0}mm)" +
                             (down ? $" {take}/{lst.Count}" : $" {take}개")
                });
            }

            // ── BLOCKED 레이어 렌더 ───────────────────────────────────────────
            // 장애물이 꽉 찬 구역: 레벨이 낮을수록 큰 장애물(바닥·슬래브·장비), 높을수록 작은 장애물 세부
            foreach (var (level, lst) in blockedByLevel)
            {
                // BLOCKED는 수가 많아 FREE보다 낮은 상한 적용
                int capLevel = Math.Max(30, 200 / (level + 1));
                int take     = Math.Min(capLevel, lst.Count);
                if (take <= 0) continue;
                double stride = (double)lst.Count / take;
                float  repSz  = lst[0].SizeMm;
                double radius = Math.Max(3.0, repSz * 0.008);
                Color  col    = blockedPalette[Math.Min(level, blockedPalette.Length - 1)];
                var    mb     = new MeshBuilder(false, false);
                for (int i = 0; i < take; i++)
                {
                    var lf = lst[(int)(i * stride)];
                    float sz = lf.SizeMm;
                    AddBoxFrameToMesh(mb, new(lf.X0Mm, lf.Y0Mm, lf.Z0Mm),
                                          new(lf.X0Mm+sz, lf.Y0Mm+sz, lf.Z0Mm+sz), radius);
                    totalRendered++;
                }
                group.Children.Add(Geometry(mb, col, 180));   // 반투명(alpha=180)으로 FREE와 구분
                bool down = lst.Count > take;
                Legend.Add(new LegendItem {
                    Swatch = new SolidColorBrush(Color.FromArgb(180, col.R, col.G, col.B)),
                    Label  = $"장애물 L{level} ({repSz:0}mm)" +
                             (down ? $" {take}/{lst.Count}" : $" {take}개")
                });
            }

            return totalRendered;
        }

        // 공유 MeshBuilder 에 박스 와이어프레임(12개 에지 실린더)을 추가한다.
        private static void AddBoxFrameToMesh(MeshBuilder mb, Point3D lo, Point3D hi, double radius)
        {
            Point3D[] c = {
                new(lo.X, lo.Y, lo.Z), new(hi.X, lo.Y, lo.Z),
                new(hi.X, hi.Y, lo.Z), new(lo.X, hi.Y, lo.Z),
                new(lo.X, lo.Y, hi.Z), new(hi.X, lo.Y, hi.Z),
                new(hi.X, hi.Y, hi.Z), new(lo.X, hi.Y, hi.Z),
            };
            // 밑면 4에지
            mb.AddCylinder(c[0], c[1], radius, 6); mb.AddCylinder(c[1], c[2], radius, 6);
            mb.AddCylinder(c[2], c[3], radius, 6); mb.AddCylinder(c[3], c[0], radius, 6);
            // 윗면 4에지
            mb.AddCylinder(c[4], c[5], radius, 6); mb.AddCylinder(c[5], c[6], radius, 6);
            mb.AddCylinder(c[6], c[7], radius, 6); mb.AddCylinder(c[7], c[4], radius, 6);
            // 수직 4에지
            mb.AddCylinder(c[0], c[4], radius, 6); mb.AddCylinder(c[1], c[5], radius, 6);
            mb.AddCylinder(c[2], c[6], radius, 6); mb.AddCylinder(c[3], c[7], radius, 6);
        }

        // ── 경로 직선화 헬퍼 (DbRouteDiag 동일 알고리즘 · GUI 버전) ─────────────────────────────
        // 직교 경로의 계단·킨크를 충돌 없는 L 로 펴서 꺾임 제거.
        // blk = 셀 차단 판정(true=막힘). 양 끝점 고정 · 꺾임 비증가 · 충돌 안전.
        private static PathCell[] StraightenOrtho(IReadOnlyList<PathCell> path, Func<int, int, int, bool> blk)
        {
            int n = path.Count;
            if (n < 3) return path is PathCell[] a ? a : path.ToArray();
            var outp = new List<PathCell> { path[0] };
            int i = 0;
            while (i < n - 1)
            {
                int pick = i + 1; PathCell[]? pickL = null;
                for (int j = n - 1; j >= i + 2; j--)
                {
                    var L = FreeOrthoL(path[i], path[j], blk);
                    if (L != null) { pick = j; pickL = L; break; }
                }
                if (pickL != null) outp.AddRange(pickL); else outp.Add(path[i + 1]);
                i = pick;
            }
            return outp.ToArray();
        }

        // 짧은 수직 jog(같은 축·방향 두 런 사이 ≤JOG_MAX 셀 다른 축 런)을 충돌 없는 L 로 흡수. 큰 코너 보존.
        private static PathCell[] DeJog(IReadOnlyList<PathCell> path, Func<int, int, int, bool> blk)
        {
            const int JOG_MAX = 4;
            var pts = new List<PathCell>(path);
            for (int guard = 0; guard < 128 && pts.Count >= 3; guard++)
            {
                var runs = new List<(int axis, int dir, int s, int e)>();
                for (int k = 1; k < pts.Count; k++)
                {
                    int ax = pts[k].I != pts[k - 1].I ? 0 : pts[k].J != pts[k - 1].J ? 1 : 2;
                    int dr = ax == 0 ? Math.Sign(pts[k].I - pts[k - 1].I)
                           : ax == 1 ? Math.Sign(pts[k].J - pts[k - 1].J) : Math.Sign(pts[k].K - pts[k - 1].K);
                    if (runs.Count == 0) { runs.Add((ax, dr, 0, 1)); continue; }
                    var last = runs[runs.Count - 1];
                    if (ax == last.axis && dr == last.dir) runs[runs.Count - 1] = (last.axis, last.dir, last.s, k);
                    else runs.Add((ax, dr, k - 1, k));
                }
                bool changed = false;
                for (int i = 1; i + 1 < runs.Count; i++)
                {
                    if (runs[i - 1].axis == runs[i + 1].axis && runs[i - 1].dir == runs[i + 1].dir
                        && runs[i].e - runs[i].s <= JOG_MAX)
                    {
                        int aIdx = runs[i - 1].s, bIdx = runs[i + 1].e;
                        var L = FreeOrthoL(pts[aIdx], pts[bIdx], blk);
                        if (L != null)
                        {
                            var nu = new List<PathCell>(pts.Count);
                            for (int t = 0; t <= aIdx; t++) nu.Add(pts[t]);
                            nu.AddRange(L);
                            for (int t = bIdx + 1; t < pts.Count; t++) nu.Add(pts[t]);
                            pts = nu; changed = true; break;
                        }
                    }
                }
                if (!changed) break;
            }
            return pts.ToArray();
        }

        // a→b 를 최대 2축 L 경로로 잇되, 모든 중간 셀이 자유(blk=false)인 경우에만 반환.
        private static PathCell[]? FreeOrthoL(PathCell a, PathCell b, Func<int, int, int, bool> blk)
        {
            int dI = b.I - a.I, dJ = b.J - a.J, dK = b.K - a.K;
            int ax = (dI != 0 ? 1 : 0) + (dJ != 0 ? 1 : 0) + (dK != 0 ? 1 : 0);
            if (ax == 0 || ax == 3) return null;
            int[][] orders;
            if (ax == 1) orders = new[] { new[] { 0, 1, 2 } };
            else
            {
                var nz = new List<int>(); if (dI != 0) nz.Add(0); if (dJ != 0) nz.Add(1); if (dK != 0) nz.Add(2);
                orders = new[] { new[] { nz[0], nz[1] }, new[] { nz[1], nz[0] } };
            }
            foreach (var ord in orders)
            {
                var cells = new List<PathCell>(); var cur = a;
                foreach (var axis in ord)
                {
                    int d = axis == 0 ? dI : axis == 1 ? dJ : dK; int s = Math.Sign(d);
                    while ((axis == 0 && cur.I != b.I) || (axis == 1 && cur.J != b.J) || (axis == 2 && cur.K != b.K))
                    {
                        cur = axis == 0 ? new PathCell(cur.I + s, cur.J, cur.K)
                            : axis == 1 ? new PathCell(cur.I, cur.J + s, cur.K)
                                        : new PathCell(cur.I, cur.J, cur.K + s);
                        cells.Add(cur);
                    }
                }
                bool free = true;
                foreach (var c in cells) if (blk(c.I, c.J, c.K)) { free = false; break; }
                if (free) return cells.ToArray();
            }
            return null;
        }
        // ─────────────────────────────────────────────────────────────────────────────────────────

        private static GeometryModel3D Geometry(MeshBuilder mb, Color color, byte alpha)
        {
            var mat = MaterialFor(color, alpha);
            return new GeometryModel3D { Geometry = mb.ToMesh(), Material = mat, BackMaterial = mat };
        }

        private static Material MaterialFor(Color color, byte alpha)
        {
            var c = Color.FromArgb(alpha, color.R, color.G, color.B);
            return new DiffuseMaterial(new SolidColorBrush(c));
        }

        // 색을 흰색 쪽으로 t(0~1) 만큼 보간해 밝게 만든다 — 자동 경로(= 유틸 색을 밝게)와 기존 설계배관
        //   (= 유틸 원색)을 같은 계열의 명도 차로 구분하는 데 쓴다. t=0 원색, t=1 흰색.
        private static Color Lighten(Color c, double t)
        {
            byte L(byte v) => (byte)Math.Round(v + (255 - v) * t);
            return Color.FromRgb(L(c.R), L(c.G), L(c.B));
        }

        // ============================================================ 진행 다이얼로그 — 선택 배관 상세
        // 표 행 클릭 시 그 배관(taskIndex)의 로컬 영역만 미니 3D 로 합성하고 결과 설명을 만든다.
        // SceneModel 과 독립(자체 Model3DGroup) — 다이얼로그 미니 뷰포트에 표시. 라우팅 완료 후
        // TaskRowVM 에 경로/방문이 캐시(CacheResults)돼 있으므로 그 데이터를 쓴다.
        // layers = [복셀맵, 점유맵, 방문맵, 최종경로, 그룹배관패턴].
        public Routing3D.Viewer.Views.PipeDetail BuildPipeDetail(int taskIndex, bool[] layers)
        {
            var group = new Model3DGroup();
            if (_scene == null || taskIndex < 0 || taskIndex >= Tasks.Count)
                return new Routing3D.Viewer.Views.PipeDetail(group, new Rect3D(), "데이터 없음");
            var grid = _scene.Grid;
            var row = Tasks[taskIndex];
            bool gOn = layers.Length > 0 && layers[0];
            bool oOn = layers.Length > 1 && layers[1];
            bool vOn = layers.Length > 2 && layers[2];
            bool pOn = layers.Length > 3 && layers[3];
            bool patOn = layers.Length > 4 && layers[4];

            // 전체 격자(도메인) 기준으로 표시한다 — 메인 뷰/실제 BIM 처럼 '모든 셀'을 대상으로 그려,
            // 배관을 전체 장애물 맥락 안에서 본다(이전엔 배관 로컬 BBOX 로 잘라 부분 슬래브만 보여 혼동).
            var dlo = new Point3D(grid.Ox, grid.Oy, grid.Oz);
            var dhi = new Point3D(grid.Ox + grid.Nx * grid.CellMm,
                                  grid.Oy + grid.Ny * grid.CellMm,
                                  grid.Oz + grid.Nz * grid.CellMm);

            // 색상 규약: 복셀맵=회색 선형 틀 · 점유맵=적색 · 방문맵=노랑 · 최종경로=파랑(꺾임 셀=녹색).
            if (gOn) AddBoxFrame(group, dlo, dhi, Color.FromRgb(150, 152, 160), Math.Max(grid.CellMm * 0.12, 8), 200);
            if (oOn) AddFullOccupancy(group, grid);
            // 그룹배관 패턴(보라) — 이 배관 유틸의 학습 트렁크 레인(L4). 방문/경로보다 먼저 그려 뒤에 깔리게.
            //   큐브 변 = 이 배관 실제 관경×1.35(배관보다 살짝 크게·반투명) — 관경 미상이면 셀×0.9.
            if (patOn && _bundles != null)
            {
                double patCube = row.DiameterMm > 0 ? row.DiameterMm * 1.35 : grid.CellMm * 0.9;
                AddBundlePatternVoxels(group, grid, BuildBundleCorridorCells(new List<int> { taskIndex }, 2, includeVertical: true), patCube, 95, 120_000);
            }
            if (vOn && row.Visited.Length > 0) AddLocalVisited(group, grid, row.Visited);
            if (pOn && row.Path.Length >= 1) AddPipePath(group, grid, row);

            var box = new Rect3D(dlo.X, dlo.Y, dlo.Z, dhi.X - dlo.X, dhi.Y - dlo.Y, dhi.Z - dlo.Z);
            return new Routing3D.Viewer.Views.PipeDetail(group, box, ExplainPipe(taskIndex));
        }

        // 메인 뷰에서 그 배관을 '선택'(경로 강조 + 종단 마커) 하고 그 로컬 영역으로 줌.
        public void FocusPipeInMainView(int taskIndex)
        {
            if (_scene == null || taskIndex < 0 || taskIndex >= Tasks.Count) return;
            SelectedTask = Tasks[taskIndex];   // → UpdateSelectionHighlight: 선택 경로 밝게 강조 + 시작/종단 마커.
            var (lo, hi) = PipeWorldBounds(Tasks[taskIndex], _scene.Grid);
            ZoomToBoxRequested?.Invoke(new Rect3D(lo.X, lo.Y, lo.Z, hi.X - lo.X, hi.Y - lo.Y, hi.Z - lo.Z));
        }

        // 배관(경로+시작/종단, 실패면 방문까지) 을 감싸는 월드 AABB + 여유.
        private (Point3D lo, Point3D hi) PipeWorldBounds(TaskRowVM row, GridMeta g)
        {
            double nx = double.MaxValue, ny = double.MaxValue, nz = double.MaxValue;
            double xx = double.MinValue, xy = double.MinValue, xz = double.MinValue;
            void Acc(double x, double y, double z)
            {
                nx = Math.Min(nx, x); ny = Math.Min(ny, y); nz = Math.Min(nz, z);
                xx = Math.Max(xx, x); xy = Math.Max(xy, y); xz = Math.Max(xz, z);
            }
            foreach (var c in row.Path) { var p = CellToWorld(g, c); Acc(p.X, p.Y, p.Z); }
            Acc(row.Sx, row.Sy, row.Sz); Acc(row.Gx, row.Gy, row.Gz);
            if (row.Path.Length == 0)
                foreach (var c in row.Visited) { var p = CellToWorld(g, c); Acc(p.X, p.Y, p.Z); }
            double m = Math.Max(g.CellMm * 3, 800);
            return (new Point3D(nx - m, ny - m, nz - m), new Point3D(xx + m, xy + m, xz + m));
        }

        // 점유맵 — 전체 격자의 모든 블록(장애물) 셀을 '셀 크기' 적색 큐브로(반투명, 인접 큐브가 맞닿아
        // 빈틈 없이 채움). 상한(cap) 초과 시에만 균등 다운샘플(메인 뷰 점유맵과 동일 정책). bbox 클리핑 없음.
        private void AddFullOccupancy(Model3DGroup group, GridMeta g)
        {
            if (_engine == null) return;
            const int cap = 150_000;
            var cells = _engine.CopyBlockedSampled(cap);
            if (cells.Length == 0) return;
            int take = cells.Length;
            double stride = 1.0;
            double s = g.CellMm;
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++) { var c = cells[(int)(n * stride)]; mb.AddBox(CellToWorld(g, c), s, s, s); }
            group.Children.Add(Geometry(mb, Color.FromRgb(220, 70, 60), 120));   // 점유맵=적색.
        }

        // 이 배관 A* 가 확장한 방문 셀(노란색 반투명, 점유보다 작은 큐브). 상한 초과 시 균등 다운샘플.
        private static void AddLocalVisited(Model3DGroup group, GridMeta g, PathCell[] visited)
        {
            const int cap = 40000;
            int take = Math.Min(cap, visited.Length);
            double stride = (double)visited.Length / take;
            double s = g.CellMm * 0.7;
            var mb = new MeshBuilder(false, false);
            for (int n = 0; n < take; n++) { var c = visited[(int)(n * stride)]; mb.AddBox(CellToWorld(g, c), s, s, s); }
            group.Children.Add(Geometry(mb, Color.FromRgb(235, 205, 60), 80));   // 방문맵=노랑.
        }

        // 최종 경로 — 경로 튜브(파랑) + 경로를 구성하는 셀 큐브(직선=파랑·꺾임=녹색) + 시작(빨강)/종단(파랑) 구.
        private static void AddPipePath(Model3DGroup group, GridMeta g, TaskRowVM row)
        {
            double dia = Math.Max(g.CellMm * 0.35, 30);
            var path = row.Path;

            // (1) 경로 튜브(스텁 포함) — 파랑.
            if (path.Length >= 1)
            {
                var pts = new List<Point3D>();
                if (row.StartStub != null) pts.AddRange(row.StartStub.Select(p => new Point3D(p.X, p.Y, p.Z)));
                pts.AddRange(path.Select(c => CellToWorld(g, c)));
                if (row.EndStub != null)
                    for (int k = row.EndStub.Count - 1; k >= 0; k--)
                        pts.Add(new Point3D(row.EndStub[k].X, row.EndStub[k].Y, row.EndStub[k].Z));
                if (pts.Count >= 2)
                {
                    var mb = new MeshBuilder(false, false);
                    mb.AddTube(pts, dia, 12, false);
                    group.Children.Add(Geometry(mb, Color.FromRgb(60, 150, 240), 255));   // 경로 튜브=파랑.
                }
            }

            // (2) 경로를 구성하는 셀 큐브 — 직선 셀=파랑, 방향이 바뀌는 꺾임 셀=녹색.
            if (path.Length >= 1)
            {
                double s = g.CellMm * 0.55;
                var straight = new MeshBuilder(false, false);
                var bend = new MeshBuilder(false, false);
                (int dx, int dy, int dz) Dir(PathCell a, PathCell b) =>
                    (Math.Sign(b.I - a.I), Math.Sign(b.J - a.J), Math.Sign(b.K - a.K));
                for (int i = 0; i < path.Length; i++)
                {
                    bool isBend = i > 0 && i < path.Length - 1 && Dir(path[i - 1], path[i]) != Dir(path[i], path[i + 1]);
                    (isBend ? bend : straight).AddBox(CellToWorld(g, path[i]), s, s, s);
                }
                if (straight.Positions.Count > 0) group.Children.Add(Geometry(straight, Color.FromRgb(70, 130, 230), 205));   // 경로 셀=파랑.
                if (bend.Positions.Count > 0) group.Children.Add(Geometry(bend, Color.FromRgb(80, 210, 110), 255));            // 꺾임 셀=녹색.
            }

            // (3) 시작/종단 구 마커.
            var sm = new MeshBuilder(false, false); sm.AddSphere(new Point3D(row.Sx, row.Sy, row.Sz), dia * 1.6);
            group.Children.Add(Geometry(sm, Color.FromRgb(230, 80, 80), 255));   // 시작=빨강.
            var em = new MeshBuilder(false, false); em.AddSphere(new Point3D(row.Gx, row.Gy, row.Gz), dia * 1.6);
            group.Children.Add(Geometry(em, Color.FromRgb(80, 120, 230), 255));  // 종단=파랑.
        }

        // 라우팅 결과를 사람이 읽을 설명으로(성공: 길이/우회율/꺾임/탐색 · 실패: 막힘 유형).
        public string ExplainPipe(int taskIndex)
        {
            if (_scene == null || taskIndex < 0 || taskIndex >= Tasks.Count) return "데이터 없음";
            var row = Tasks[taskIndex];
            var sb = new System.Text.StringBuilder();
            string s = string.IsNullOrEmpty(row.PocName) ? $"({row.Sx:0},{row.Sy:0},{row.Sz:0})" : row.PocName!;
            string e = string.IsNullOrEmpty(row.EndName) ? $"({row.Gx:0},{row.Gy:0},{row.Gz:0})" : row.EndName!;
            sb.AppendLine($"[{row.Group} / {row.Utility}]");
            sb.AppendLine($"{s}  →  {e}");
            sb.AppendLine();
            double man = Math.Abs(row.Gx - row.Sx) + Math.Abs(row.Gy - row.Sy) + Math.Abs(row.Gz - row.Sz);
            int visited = row.Visited.Length;
            if (row.Success && row.Path.Length >= 2)
            {
                var b = CountBends(row.Path);
                double len = row.LengthMm;
                double detour = man > 1 ? (len / man - 1.0) * 100.0 : 0.0;
                sb.AppendLine("결과 : ✅ 성공");
                sb.AppendLine($"· 경로 길이    : {len:#,0} mm");
                sb.AppendLine($"· 직선(맨해튼) : {man:#,0} mm");
                sb.AppendLine($"· 우회율       : {detour:0.0} %");
                sb.AppendLine($"· 꺾임(엘보)   : {b.Total} 회 (수평 {b.Horiz}·수직 {b.Vert})");
                sb.AppendLine($"· 경로 셀      : {row.Path.Length:#,0}");
                sb.AppendLine($"· 탐색(방문)   : {visited:#,0} 셀");
                if (row.StartStub != null || row.EndStub != null)
                    sb.AppendLine($"· 스텁         : 출발 {(row.StartStub != null ? "○" : "—")} · 종단 {(row.EndStub != null ? "○" : "—")} (기존설계 추종)");
                sb.AppendLine();
                sb.AppendLine(detour < 15 ? "→ 직선에 가깝게 효율적으로 연결되었습니다."
                            : detour < 50 ? "→ 장애물/회랑을 우회하며 연결되었습니다."
                            : "→ 우회가 큽니다(혼잡·회랑 바이어스). 방문맵으로 탐색 범위를 확인하세요.");
            }
            else
            {
                sb.AppendLine("결과 : ❌ 실패 (경로 없음)");
                sb.AppendLine($"· 직선(맨해튼) : {man:#,0} mm");
                sb.AppendLine($"· 탐색(방문)   : {visited:#,0} 셀");
                sb.AppendLine();
                if (visited <= 1)
                    sb.AppendLine("→ 시작/종단 셀이 장애물 내부입니다(PoC가 솔리드에 파묻힘).\n   탐색이 거의 없음 = 출발조차 못함. 면 투영/스냅 대상.");
                else if (visited >= 11_000_000)
                    sb.AppendLine("→ 탐색 상한 도달: 경로가 매우 길거나 사실상 막혔습니다.\n   방문맵이 넓게 퍼진 뒤 종단에 못 닿음.");
                else
                    sb.AppendLine("→ 종단까지 완전 차단되었습니다.\n   방문맵으로 어디까지 탐색하고 막혔는지 확인하세요(rip-up/CBS 대상).");
            }
            sb.AppendLine();
            sb.AppendLine("─ 색상 ─");
            sb.AppendLine("복셀맵=회색 틀 · 점유맵=적색");
            sb.AppendLine("방문맵=노랑 · 경로=파랑(꺾임=녹색)");
            sb.AppendLine("그룹배관패턴=보라(공용 트렁크 레인)");
            return sb.ToString();
        }

        // ── 자동설계 결과 DB 저장 / 불러오기 ────────────────────────────────────────────────────
        // 저장: ResultList 에 있는 배관(routed batch)을 TB_AUTOROUTING_SESSION + TB_AUTOROUTING_PATH 에 저장.
        // 불러오기: 세션 목록 창(SessionPickerWindow) → 선택 → Tasks 에 매칭 → LoadedPolylinePts 설정 → 재렌더.

        private async Task SaveRouteResultsAsync()
        {
            if (_scene == null || ResultList.Count == 0) return;
            var grid = _scene.Grid;
            var proj = SelectedProject;

            // 세션 파라미터 JSON
            var paramsObj = new System.Text.Json.Nodes.JsonObject
            {
                ["min_straight_mm"] = 100.0,
                ["route_mode"]      = _routingMode.ToString(),
                ["cell_mm"]         = _cellMm,
                ["use_patterns"]    = _usePatterns,
                ["use_stub"]        = _useStubRouting,
                ["use_replicate"]   = _useDesignReplicate,
            };

            var session = new Model.AutoRoutingSessionRow
            {
                ProjectGroupId = proj?.GroupId ?? "",
                ProjectName    = proj?.GroupName ?? "",
                EquipmentName  = proj?.GroupName ?? "",   // TAG_GROUP_NM = 장비호기명
                CellMm         = (int)_cellMm,
                RouteMode      = _routingMode.ToString(),
                TotalCount     = ResultList.Count,
                SuccessCount   = ResultList.Count(r => r.Success),
                FailCount      = ResultList.Count(r => !r.Success),
                TotalLengthMm  = ResultList.Where(r => r.Success).Sum(r => r.LengthMm),
                ParamsJson     = paramsObj.ToJsonString(),
            };

            // 배관별 결과 — 폴리라인은 GetRoutedPolyline 로 world mm 추출
            var paths = new List<Model.AutoRoutingPathRow>();
            foreach (var row in ResultList)
            {
                List<System.Windows.Media.Media3D.Point3D>? poly = null;
                if (row.Success)
                {
                    var pts = row.LoadedPolylinePts ?? GetRoutedPolyline(row, grid);
                    if (pts.Count >= 2) poly = pts;
                }
                Guid? routeGuid = Guid.TryParse(row.RoutePathGuid, out var g) ? g : null;
                paths.Add(new Model.AutoRoutingPathRow
                {
                    RouteOrder    = row.RouteOrder,
                    RoutePathGuid = routeGuid,
                    UtilityGroup  = row.Group,
                    Utility       = row.Utility,
                    SourceName    = row.PocName,
                    TargetName    = row.EndName,
                    DiameterMm    = row.DiameterMm,
                    Success       = row.Success,
                    FailReason    = row.Success ? null : row.LastFail.ToString(),
                    LengthMm      = row.LengthMm,
                    TurnCount     = row.TurnCount,
                    ElapsedMs     = (int)row.ElapsedMs,
                    StartStubPointCount = row.StartStub?.Count ?? 0,
                    EndStubPointCount   = row.EndStub?.Count ?? 0,
                    Polyline      = poly,
                });
            }

            try
            {
                Status = "DB에 저장 중...";
                var sessionId = await Task.Run(() =>
                    Model.AutoRoutingRepository.SaveSessionAsync(_dbConfig, session, paths));
                Status = $"저장 완료 — SESSION_ID: {sessionId:D}  " +
                         $"(성공 {session.SuccessCount}/{session.TotalCount})";
            }
            catch (Exception ex)
            {
                Status = $"저장 실패: {ex.Message}";
                System.Windows.MessageBox.Show($"DB 저장 오류:\n{ex.Message}", "저장 실패",
                    System.Windows.MessageBoxButton.OK, System.Windows.MessageBoxImage.Error);
            }
        }

        private async Task LoadRouteResultsAsync()
        {
            if (_scene == null) return;
            var proj = SelectedProject;

            List<Model.AutoRoutingSessionRow> sessions;
            try
            {
                Status = "세션 목록 조회 중...";
                sessions = await Task.Run(() =>
                    Model.AutoRoutingRepository.ListSessionsAsync(_dbConfig, proj?.GroupId));
            }
            catch (Exception ex)
            {
                Status = $"세션 목록 조회 실패: {ex.Message}";
                System.Windows.MessageBox.Show($"DB 조회 오류:\n{ex.Message}", "오류",
                    System.Windows.MessageBoxButton.OK, System.Windows.MessageBoxImage.Error);
                return;
            }

            if (sessions.Count == 0)
            {
                Status = "저장된 세션이 없습니다.";
                System.Windows.MessageBox.Show(
                    proj != null
                        ? $"프로젝트 [{proj.GroupName}]의 저장된 자동설계 결과가 없습니다."
                        : "저장된 자동설계 결과가 없습니다.",
                    "결과 없음", System.Windows.MessageBoxButton.OK, System.Windows.MessageBoxImage.Information);
                return;
            }

            // 세션 선택 창
            var picker = new Views.SessionPickerWindow(sessions, _dbConfig)
            {
                Owner = System.Windows.Application.Current?.MainWindow
            };
            if (picker.ShowDialog() != true || picker.Selected == null) return;
            var selected = picker.Selected;

            // 배관 로드
            List<Model.AutoRoutingPathRow> dbPaths;
            try
            {
                Status = $"배관 결과 로드 중... [{selected.EquipmentName}]";
                dbPaths = await Task.Run(() =>
                    Model.AutoRoutingRepository.LoadPathsAsync(_dbConfig, selected.SessionId));
            }
            catch (Exception ex)
            {
                Status = $"로드 실패: {ex.Message}";
                System.Windows.MessageBox.Show($"DB 로드 오류:\n{ex.Message}", "오류",
                    System.Windows.MessageBoxButton.OK, System.Windows.MessageBoxImage.Error);
                return;
            }

            // Tasks 에 매칭 → LoadedPolylinePts + Success/LengthMm 갱신
            // 매칭 키: RoutePathGuid(Guid 문자열 대소문자 무관) 우선, 없으면 (PocName, EndName) 조합.
            // 주의: ToDictionary 는 중복 키 시 예외 — ToLookup 으로 안전하게 구성.
            var byGuid = new Dictionary<string, TaskRowVM>(StringComparer.OrdinalIgnoreCase);
            foreach (var t in Tasks)
            {
                if (t.RoutePathGuid != null)
                    byGuid.TryAdd(t.RoutePathGuid, t);
            }

            // 이름 매칭용 Lookup (중복 허용 — 첫 번째 항목 채택).
            var byNameLookup = Tasks.ToLookup(
                t => $"{t.PocName ?? ""}\x00{t.EndName ?? ""}",
                StringComparer.OrdinalIgnoreCase);

            // 매칭되지 않은 DB 행은 Tasks 에 새 행을 추가해 보여줄 수 있도록 색 배정 준비.
            var colorMap = UtilityColors.Assign(dbPaths.Select(d =>
            {
                string grp = d.UtilityGroup ?? "?";
                string uti = d.Utility ?? "?";
                return $"[{grp}] {uti}";
            }));

            ResultList.Clear();
            int matched = 0, unmatched = 0, created = 0;

            // Tasks 의 이전 LoadedPolylinePts 를 초기화(이전 로드 결과 잔존 방지).
            foreach (var t in Tasks) t.LoadedPolylinePts = null;

            foreach (var dp in dbPaths)
            {
                TaskRowVM? row = null;

                // 1순위: GUID 매칭
                if (dp.RoutePathGuid.HasValue)
                    byGuid.TryGetValue(dp.RoutePathGuid.Value.ToString("D"), out row);

                // 2순위: 이름 매칭 (중복 시 첫 번째 TaskRowVM 채택)
                if (row == null)
                {
                    string nameKey = $"{dp.SourceName ?? ""}\x00{dp.TargetName ?? ""}";
                    row = byNameLookup[nameKey].FirstOrDefault();
                }

                // 3순위: 매칭 실패 → DB 데이터로 새 TaskRowVM 생성해 Tasks 에 추가
                if (row == null && dp.Polyline != null && dp.Polyline.Count >= 2)
                {
                    string lbl = $"[{dp.UtilityGroup ?? "?"}] {dp.Utility ?? "?"}";
                    var clr = colorMap.TryGetValue(lbl, out var c) ? c : System.Windows.Media.Colors.Gray;
                    row = new TaskRowVM
                    {
                        Index = Tasks.Count,
                        Label = lbl,
                        Swatch = new System.Windows.Media.SolidColorBrush(clr),
                        Utility = dp.Utility,
                        Group = dp.UtilityGroup,
                        PocName = dp.SourceName,
                        EndName = dp.TargetName,
                        RoutePathGuid = dp.RoutePathGuid?.ToString("D"),
                        DiameterMm = dp.DiameterMm,
                    };
                    Tasks.Add(row);
                    created++;
                }

                if (row == null) { unmatched++; continue; }

                row.LoadedPolylinePts = dp.Polyline;
                row.Success           = dp.Success;
                row.LengthMm          = dp.LengthMm;
                row.DiameterMm        = dp.DiameterMm > 0 ? dp.DiameterMm : row.DiameterMm;
                row.RouteOrder        = dp.RouteOrder;
                row.NotifyResultChanged();
                ResultList.Add(row);
                matched++;
            }

            BuildModel();   // 불러온 폴리라인으로 재렌더
            string createdNote = created > 0 ? $"  신규 {created}" : "";
            Status = $"로드 완료 [{selected.EquipmentName} · {selected.LastModifiedAt.ToLocalTime():MM-dd HH:mm}] " +
                     $"— 매칭 {matched}/{dbPaths.Count}  (미매칭 {unmatched}{createdNote})";
        }
    }
}

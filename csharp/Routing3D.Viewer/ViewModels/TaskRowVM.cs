// 작업 행 뷰모델 — 목록 표시 + 종단점 편집(인터랙티브 재라우팅)
// =============================================================================
//   작업 1건의 유틸리티 라벨/색, 시작·끝 좌표(편집 가능), 라우팅 결과(성공/길이)를 담는다.
// =============================================================================
using System;
using System.Windows.Media;
using Routing3D.Viewer.Interop;

namespace Routing3D.Viewer.ViewModels
{
    /// <summary>자동설계 진행 중 행의 실시간 상태 — 결과 그리드 '상태' 컬럼에 라이브 표시.
    /// Idle=평상시(성공/실패/미라우팅은 Success·Path 로 판정) · Queued=대기(이번 배치 포함, 탐색 전)
    /// · Searching=탐색 중(SearchProgress %).</summary>
    public enum RouteRunState { Idle, Queued, Searching }

    public sealed class TaskRowVM : ObservableObject
    {
        public int Index { get; init; }
        public string Label { get; init; } = string.Empty;
        public Brush Swatch { get; init; } = Brushes.Gray;

        /// <summary>원본 유틸리티/그룹(부분집합 라우팅 분류용). UtilityLabel 과 별개의 원시값.</summary>
        public string? Utility { get; init; }
        public string? Group { get; init; }

        /// <summary>시작/끝 PoC 이름(DB 로드 시). 개별 PoC 목록 표시용.</summary>
        public string? PocName { get; init; }
        public string? EndName { get; init; }

        /// <summary>유래한 기존배관 GUID(TB_ROUTE_PATH.ROUTE_PATH_GUID) — '세그먼트 상세' 탭 조회 키. DB 로드 시에만.</summary>
        public string? RoutePathGuid { get; init; }

        /// <summary>개별 PoC 목록 항목 표시: "#idx 시작→끝" (이름 없으면 좌표).</summary>
        public string PocDisplay
        {
            get
            {
                string s = string.IsNullOrEmpty(PocName) ? $"({Sx:0},{Sy:0},{Sz:0})" : PocName!;
                string e = string.IsNullOrEmpty(EndName) ? $"({Gx:0},{Gy:0},{Gz:0})" : EndName!;
                string status = Success ? $"{LengthMm:0} mm" : "미라우팅";
                return $"#{Index}  {s} → {e}   [{status}]";
            }
        }

        // 라우팅 결과 경로/방문 셀을 행에 캐시한다 — 엔진은 부분집합 라우팅마다 재구성되므로
        // (엔진 인덱스가 행과 1:1 이 아님) 3D 렌더는 엔진이 아니라 이 캐시를 읽는다.
        public PathCell[] Path { get; set; } = Array.Empty<PathCell>();
        public PathCell[] Visited { get; set; } = Array.Empty<PathCell>();

        /// <summary>이 배관의 관경(외경, mm). 매칭되는 기존 설계배관(SOURCE_SIZE)에서 가져온다.
        /// 0 이면 미상 → 렌더에서 격자 기반 기본 지름으로 폴백. 자동 경로 튜브를 실제 관경으로 그리는 데 쓴다.</summary>
        public double DiameterMm { get; set; }

        // 스텁 라우팅: 출발/종단 스텁(고정 설계 구간, 월드 mm). A* 는 스텁 끝~끝만 탐색하고, 표시 경로는
        // [StartStub] + [A* 경로] + [reverse(EndStub)] 로 합성한다. null/빈 = 스텁 미적용(PoC 직접 라우팅).
        public System.Collections.Generic.List<Model.Pt3>? StartStub { get; set; }
        public System.Collections.Generic.List<Model.Pt3>? EndStub { get; set; }

        /// <summary>DB에서 불러온 렌더 폴리라인(world mm). non-null 이면 BuildModel 이 Path/Stubs 대신 이 좌표로 렌더한다.</summary>
        public System.Collections.Generic.List<System.Windows.Media.Media3D.Point3D>? LoadedPolylinePts { get; set; }

        private double _sx, _sy, _sz, _gx, _gy, _gz;
        public double Sx { get => _sx; set => Set(ref _sx, value); }
        public double Sy { get => _sy; set => Set(ref _sy, value); }
        public double Sz { get => _sz; set => Set(ref _sz, value); }
        public double Gx { get => _gx; set => Set(ref _gx, value); }
        public double Gy { get => _gy; set => Set(ref _gy, value); }
        public double Gz { get => _gz; set => Set(ref _gz, value); }

        // ── 자동설계 진행 상태(라이브) ──────────────────────────────────────
        // RouteRowsAsync 가 배치 시작 시 대상 행을 Queued 로, 진행 콜백(phase0)이 Searching+진행율로,
        // 완료(CacheResults)가 Idle 로 되돌린다. StatusText/StatusBrush 가 이를 반영한다.
        private RouteRunState _runState = RouteRunState.Idle;
        public RouteRunState RunState
        {
            get => _runState;
            set { if (Set(ref _runState, value)) { OnChanged(nameof(StatusText)); OnChanged(nameof(StatusBrush)); } }
        }
        private double _searchProgress;   // 0~1, phase0 탐색 진행율.
        public double SearchProgress
        {
            get => _searchProgress;
            set { if (Set(ref _searchProgress, value)) OnChanged(nameof(StatusText)); }
        }

        /// <summary>마지막 라우팅에서 A* 가 확장(방문)한 노드 수 — 실패 원인 진단에 쓴다(0/소수=출발 막힘,
        /// 대량=혼잡/막힘, 상한=탐색 초과). CacheResults 에서 엔진 결과로 채운다.</summary>
        public long ExpandedNodes { get; set; }

        /// <summary>이 배관 한 개에 걸린 라우팅 시간(ms). 진행 콜백 phase==1 의 ElapsedMs 로 채운다.</summary>
        public double ElapsedMs { get; set; }

        /// <summary>경로 탐색에 걸린 시간 표시 — 1초 미만은 ms, 1초 이상은 s 단위. 미라우팅이면 빈 칸.</summary>
        public string ElapsedText
        {
            get
            {
                if (!Attempted || ElapsedMs <= 0) return "";
                return ElapsedMs < 1000 ? $"{ElapsedMs:0}ms" : $"{ElapsedMs / 1000.0:0.0}s";
            }
        }

        /// <summary>마지막 라우팅의 엔진 실패 사유(A1) — 성공 시 None. ExplainFailure 가 정확 분류에 쓴다.</summary>
        public Interop.RouteFail LastFail { get; set; } = Interop.RouteFail.None;

        /// <summary>결과 그리드용 실패 사유. 미시도/성공 행은 빈칸으로 두고 실패 행만 원인을 보여준다.</summary>
        public string FailReasonText
        {
            get
            {
                if (Success || !Attempted) return "";
                return LastFail switch
                {
                    Interop.RouteFail.StartBlocked => "출발막힘",
                    Interop.RouteFail.GoalBlocked => "종단막힘",
                    Interop.RouteFail.CorridorMiss => "회랑이탈",
                    Interop.RouteFail.ExpansionLimit => "탐색상한",
                    Interop.RouteFail.GoalDirBlocked => "진입축막힘",
                    Interop.RouteFail.NoPath => "경로없음",
                    _ => ExpandedNodes > 0 ? "경로없음" : "미상"
                };
            }
        }

        /// <summary>결과 그리드용 확장 노드 수. 최단경로가 오래 걸린 원인을 바로 확인하기 위한 진단값.</summary>
        public string ExpandedText => Attempted && ExpandedNodes > 0 ? $"{ExpandedNodes:N0}" : "";

        /// <summary>이번 자동설계 배치에서 이 배관이 라우팅된 순서(0부터, 우선순위 정렬 결과 = 굵은 배관 먼저).
        /// 진행 콜백(phase==1)의 OrderIndex 로 채운다. −1=이 배치에 미포함 또는 콜백 없는 모드. 결과 리포트의
        /// '설계 순서'에 쓴다.</summary>
        public int RouteOrder { get; set; } = -1;

        // 엔진에 실제로 전달된 A* 시작/목표(월드 mm). 원본 PoC(Sx/Gx)는 장비·덕트 내부(의도적 솔리드)이고,
        // 실제 탐색은 스텁 끝(랙 위 자유공간) 또는 표면투영·스냅된 자유 셀에서 시작/종료한다. 실패 진단
        // (ExplainFailure)이 '출발조차 못함'을 오판하지 않도록 BuildEngineForRows 가 이 값을 채운다.
        // HasRouteEndpoints=false(미설정, NaN)면 진단은 원본 PoC 로 폴백한다.
        public double RouteSx { get; private set; } = double.NaN;
        public double RouteSy, RouteSz, RouteGx, RouteGy, RouteGz;
        public bool HasRouteEndpoints => !double.IsNaN(RouteSx);
        public void SetRouteEndpoints(double sx, double sy, double sz, double gx, double gy, double gz)
        { RouteSx = sx; RouteSy = sy; RouteSz = sz; RouteGx = gx; RouteGy = gy; RouteGz = gz; }
        public void ClearRouteEndpoints() => RouteSx = double.NaN;

        private bool _success;
        private double _lengthMm;
        public bool Success { get => _success; set { if (Set(ref _success, value)) { OnChanged(nameof(Display)); OnChanged(nameof(PocDisplay)); OnChanged(nameof(StatusText)); OnChanged(nameof(StatusBrush)); OnChanged(nameof(TurnCount)); } } }
        public double LengthMm { get => _lengthMm; set { if (Set(ref _lengthMm, value)) { OnChanged(nameof(Display)); OnChanged(nameof(PocDisplay)); OnChanged(nameof(LengthText)); OnChanged(nameof(TurnCount)); } } }

        public string Display => $"#{Index}  {Label}   {(Success ? $"{LengthMm:0} mm" : "실패/미라우팅")}";

        // ── 결과 리스트(DataGrid) 표시용 파생 속성 ──────────────────────────
        /// <summary>라우팅을 시도했는가 — 경로가 있거나(성공) 방문/확장 기록이 있으면(실패) 시도됨.
        /// 시도 안 한 행과 '시도했으나 실패'를 구분해 상태/색·진행집계를 정확히 표시한다.</summary>
        public bool Attempted => (Path != null && Path.Length >= 2)
                                 || (Visited != null && Visited.Length > 0) || ExpandedNodes > 0
                                 || LastFail != Interop.RouteFail.None || ElapsedMs > 0;

        /// <summary>결과 필드(Path/Visited/ExpandedNodes/Success)를 일괄 갱신한 뒤 호출 — 상태/색/길이 파생
        /// 속성을 강제로 다시 알린다. Success 값이 그대로(실패→실패)면 세터가 알림을 생략하므로, 실패 행이
        /// '시도됨'(Attempted)을 반영하지 못해 '미라우팅'으로 남는 것을 막는다.</summary>
        public void NotifyResultChanged()
        {
            OnChanged(nameof(Display)); OnChanged(nameof(PocDisplay));
            OnChanged(nameof(StatusText)); OnChanged(nameof(StatusBrush));
            OnChanged(nameof(LengthText)); OnChanged(nameof(TurnCount)); OnChanged(nameof(ElapsedText));
            OnChanged(nameof(FailReasonText)); OnChanged(nameof(ExpandedText));
        }

        /// <summary>처리 상태 텍스트 — 결과 그리드 '상태' 컬럼. 진행 중엔 대기/탐색 %, 완료 후 성공/실패/미라우팅.</summary>
        public string StatusText
        {
            get
            {
                if (_runState == RouteRunState.Searching) return $"탐색 {_searchProgress * 100:0}%";
                if (_runState == RouteRunState.Queued) return "대기";
                return Success ? "성공" : Attempted ? "실패" : "미라우팅";
            }
        }

        /// <summary>상태 컬럼 색(탐색=호박·대기=청회·성공=녹색·실패=적색·미라우팅=회색).</summary>
        public Brush StatusBrush
        {
            get
            {
                if (_runState == RouteRunState.Searching) return new SolidColorBrush(Color.FromRgb(0xF2, 0xC3, 0x4E));
                if (_runState == RouteRunState.Queued) return new SolidColorBrush(Color.FromRgb(0x8A, 0x93, 0xA8));
                return Success ? new SolidColorBrush(Color.FromRgb(0x5A, 0xD2, 0x82))
                       : Attempted ? new SolidColorBrush(Color.FromRgb(0xE6, 0x55, 0x55))
                       : new SolidColorBrush(Color.FromRgb(0x9A, 0xA3, 0xB8));
            }
        }

        /// <summary>길이 표시(성공 시 mm, 아니면 빈칸).</summary>
        public string LengthText => Success ? $"{LengthMm:N0}" : "";

        /// <summary>경로의 방향 전환(꺾임) 수 — 직교 셀 경로에서 축 변경 횟수.</summary>
        public int TurnCount
        {
            get
            {
                var p = Path;
                if (p == null || p.Length < 3) return 0;
                int turns = 0;
                for (int i = 2; i < p.Length; i++)
                {
                    int a1 = p[i - 1].I != p[i - 2].I ? 0 : p[i - 1].J != p[i - 2].J ? 1 : 2;
                    int a2 = p[i].I != p[i - 1].I ? 0 : p[i].J != p[i - 1].J ? 1 : 2;
                    if (a1 != a2) turns++;
                }
                return turns;
            }
        }
    }
}

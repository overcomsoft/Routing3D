// 작업 행 뷰모델 — 목록 표시 + 종단점 편집(인터랙티브 재라우팅)
// =============================================================================
//   작업 1건의 유틸리티 라벨/색, 시작·끝 좌표(편집 가능), 라우팅 결과(성공/길이)를 담는다.
// =============================================================================
using System;
using System.Windows.Media;
using Routing3D.Viewer.Interop;

namespace Routing3D.Viewer.ViewModels
{
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

        private double _sx, _sy, _sz, _gx, _gy, _gz;
        public double Sx { get => _sx; set => Set(ref _sx, value); }
        public double Sy { get => _sy; set => Set(ref _sy, value); }
        public double Sz { get => _sz; set => Set(ref _sz, value); }
        public double Gx { get => _gx; set => Set(ref _gx, value); }
        public double Gy { get => _gy; set => Set(ref _gy, value); }
        public double Gz { get => _gz; set => Set(ref _gz, value); }

        private bool _success;
        private double _lengthMm;
        public bool Success { get => _success; set { if (Set(ref _success, value)) { OnChanged(nameof(Display)); OnChanged(nameof(PocDisplay)); } } }
        public double LengthMm { get => _lengthMm; set { if (Set(ref _lengthMm, value)) { OnChanged(nameof(Display)); OnChanged(nameof(PocDisplay)); } } }

        public string Display => $"#{Index}  {Label}   {(Success ? $"{LengthMm:0} mm" : "실패/미라우팅")}";
    }
}

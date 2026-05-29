// 작업 행 뷰모델 — 목록 표시 + 종단점 편집(인터랙티브 재라우팅)
// =============================================================================
//   작업 1건의 유틸리티 라벨/색, 시작·끝 좌표(편집 가능), 라우팅 결과(성공/길이)를 담는다.
// =============================================================================
using System.Windows.Media;

namespace Routing3D.Viewer.ViewModels
{
    public sealed class TaskRowVM : ObservableObject
    {
        public int Index { get; init; }
        public string Label { get; init; } = string.Empty;
        public Brush Swatch { get; init; } = Brushes.Gray;

        private double _sx, _sy, _sz, _gx, _gy, _gz;
        public double Sx { get => _sx; set => Set(ref _sx, value); }
        public double Sy { get => _sy; set => Set(ref _sy, value); }
        public double Sz { get => _sz; set => Set(ref _sz, value); }
        public double Gx { get => _gx; set => Set(ref _gx, value); }
        public double Gy { get => _gy; set => Set(ref _gy, value); }
        public double Gz { get => _gz; set => Set(ref _gz, value); }

        private bool _success;
        private double _lengthMm;
        public bool Success { get => _success; set { if (Set(ref _success, value)) OnChanged(nameof(Display)); } }
        public double LengthMm { get => _lengthMm; set { if (Set(ref _lengthMm, value)) OnChanged(nameof(Display)); } }

        public string Display => $"#{Index}  {Label}   {(Success ? $"{LengthMm:0} mm" : "실패/미라우팅")}";
    }
}

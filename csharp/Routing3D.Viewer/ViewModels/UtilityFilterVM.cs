// 유틸리티 필터 행 — 작업 목록·3D 모델에서 해당 유틸리티 배관을 보이기/숨기기.
// =============================================================================
//   라벨/색/표시 여부를 보유. IsVisible 이 바뀌면 SceneViewModel 이 TasksView 를
//   갱신하고 BuildModel 을 다시 호출해 3D 에서도 해당 유틸리티 배관이 사라진다.
// =============================================================================
using System.Windows.Media;

namespace Routing3D.Viewer.ViewModels
{
    public sealed class UtilityFilterVM : ObservableObject
    {
        public string Label { get; init; } = string.Empty;
        public Brush Swatch { get; init; } = Brushes.Gray;
        public int Count { get; init; }   // 이 유틸리티의 배관 개수(라벨 옆 표시).

        private bool _isVisible = true;
        public bool IsVisible { get => _isVisible; set => Set(ref _isVisible, value); }

        public string Display => $"{Label}  ({Count})";
    }
}

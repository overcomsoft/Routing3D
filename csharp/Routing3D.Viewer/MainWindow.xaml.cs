using System;
using System.Windows;
using System.Windows.Input;
using Routing3D.Viewer.ViewModels;

namespace Routing3D.Viewer
{
    /// <summary>메인 창. 뷰모델 연결 + 모델 재구성 시 카메라 자동 맞춤 + 3D 클릭 피킹(P3).</summary>
    public partial class MainWindow : Window
    {
        private readonly SceneViewModel _vm;

        public MainWindow()
        {
            InitializeComponent();
            _vm = new SceneViewModel();
            _vm.SceneRebuilt += () => Dispatcher.BeginInvoke(new Action(() => View.ZoomExtents()));
            DataContext = _vm;

            // 피킹 모드일 때만 좌클릭으로 3D 지점을 잡아 종단점으로 설정(평소 회전은 그대로).
            View.MouseLeftButtonDown += View_MouseLeftButtonDown;
        }

        private void View_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (_vm.PickMode == PickMode.None) return;
            var screen = e.GetPosition(View);
            var hit = View.FindNearestPoint(screen);  // 지오메트리 위의 최근접 3D 점.
            if (hit.HasValue)
            {
                _vm.ApplyPick(hit.Value);
                e.Handled = true;
            }
        }
    }
}

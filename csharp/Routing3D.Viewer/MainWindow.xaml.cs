using System.Windows;
using Routing3D.Viewer.ViewModels;

namespace Routing3D.Viewer
{
    /// <summary>메인 창. 뷰모델을 연결하고, 모델 재구성 시 카메라를 자동 맞춤(ZoomExtents).</summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            var vm = new SceneViewModel();
            vm.SceneRebuilt += () => Dispatcher.BeginInvoke(new System.Action(() => View.ZoomExtents()));
            DataContext = vm;
        }
    }
}

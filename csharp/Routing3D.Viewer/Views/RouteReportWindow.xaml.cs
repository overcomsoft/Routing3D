// 자동설계 결과 리포트 모드리스 창 — 직전 배치의 순서·꺾임·성공을 Markdown 으로 표시.
// =============================================================================
//   DataContext = SceneViewModel. 본문은 RouteResultReport 바인딩으로 갱신된다.
//   버튼(다른 이름으로 저장/복사/폴더 열기)만 코드비하인드가 처리한다(모드리스: 메인 뷰 조작 가능).
// =============================================================================
using System;
using System.IO;
using System.Windows;
using Routing3D.Viewer.ViewModels;

namespace Routing3D.Viewer.Views
{
    public partial class RouteReportWindow : Window
    {
        public RouteReportWindow()
        {
            InitializeComponent();
        }

        private SceneViewModel? Vm => DataContext as SceneViewModel;

        // 사용자가 고른 위치에 .md 로 다시 저장(SaveFileDialog).
        private void OnSaveAs(object sender, RoutedEventArgs e) => Vm?.SaveRouteReportAs();

        // 리포트 본문을 클립보드로 복사.
        private void OnCopy(object sender, RoutedEventArgs e)
        {
            var md = Vm?.RouteResultReport;
            if (!string.IsNullOrEmpty(md))
            {
                try { Clipboard.SetText(md); } catch { }
            }
        }

        // 저장된 .md 가 있는 폴더를 탐색기로 연다(파일 선택 상태로).
        private void OnOpenFolder(object sender, RoutedEventArgs e)
        {
            var path = Vm?.RouteResultReportPath;
            if (string.IsNullOrEmpty(path) || !File.Exists(path)) return;
            try
            {
                System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo("explorer.exe", $"/select,\"{path}\"")
                { UseShellExecute = true });
            }
            catch { }
        }
    }
}

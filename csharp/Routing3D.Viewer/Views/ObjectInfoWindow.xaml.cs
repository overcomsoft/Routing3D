// 객체 속성 모드리스 다이얼로그 — 메인 3D 뷰 객체 클릭 시 속성 표시(창 유지·내용 갱신).
// =============================================================================
//   DataContext = SceneViewModel. 바인딩(SelectedObjectInfo)으로 클릭마다 자동 갱신되므로
//   코드비하인드는 창 생성/표시만 담당한다. 모드리스(Show)라 메인 뷰 조작을 막지 않는다.
// =============================================================================
using System.Windows;

namespace Routing3D.Viewer.Views
{
    public partial class ObjectInfoWindow : Window
    {
        public ObjectInfoWindow()
        {
            InitializeComponent();
        }
    }
}

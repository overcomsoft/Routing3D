using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Routing3D.Viewer.ViewModels;

namespace Routing3D.Viewer
{
    /// <summary>메인 창. 뷰모델 연결 + 모델 재구성 시 카메라 자동 맞춤 + 3D 클릭 피킹(P3).</summary>
    public partial class MainWindow : Window
    {
        private readonly SceneViewModel _vm;
        private readonly List<Visual3D> _spaceLabelVisuals = new();   // 공간 영역 텍스트 라벨(BillboardText).

        public MainWindow(string? initialScene = null)
        {
            InitializeComponent();
            _vm = new SceneViewModel(initialScene);
            _vm.SceneRebuilt += () => Dispatcher.BeginInvoke(new Action(OnSceneRebuilt));
            _vm.FitViewRequested += () => Dispatcher.BeginInvoke(new Action(FitToScene));
            _vm.NavigateToRequested += target => Dispatcher.BeginInvoke(new Action(() => NavigateTo(target)));
            DataContext = _vm;

            // 피킹 모드일 때만 좌클릭으로 3D 지점을 잡아 종단점으로 설정(평소 회전은 그대로).
            View.MouseLeftButtonDown += View_MouseLeftButtonDown;

            // DB 자동 로드(무거움)는 창이 처음 화면에 그려진 뒤 비동기로 시작한다.
            // 생성자에서 동기로 하면 라우팅이 끝날 때까지 창 자체가 보이지 않는다.
            if (_vm.NeedsStartupLoad)
            {
                EventHandler? handler = null;
                handler = async (_, __) =>
                {
                    ContentRendered -= handler;       // 한 번만.
                    await _vm.RunStartupLoadAsync();
                };
                ContentRendered += handler;
            }
        }

        // 마지막으로 줌을 맞춘 씬 경계(중심X·폭·길이). 경계가 바뀌었는지 비교해
        // 새 프로젝트/씬 로드일 때만 ZoomExtents 하고, 레이어 토글에서는 카메라를 유지한다.
        private double _fitCx = double.NaN, _fitW = double.NaN, _fitL = double.NaN;

        private static bool NearlyEqual(double a, double b) => Math.Abs(a - b) < 1e-6;

        // 바닥 격자를 현재 씬 좌표에 맞추고 공간 라벨을 다시 그린다(카메라는 건드리지 않음).
        private void ApplyGroundAndLabels()
        {
            GroundGrid.Center = _vm.GroundCenter;
            GroundGrid.Width = _vm.GroundWidth;
            GroundGrid.Length = _vm.GroundLength;
            GroundGrid.MinorDistance = _vm.GroundMinorDistance;
            GroundGrid.MajorDistance = _vm.GroundMajorDistance;
            RebuildSpaceLabels();
        }

        // 씬 재구성(레이어 토글 포함). 격자·라벨은 항상 갱신하되, 카메라는 씬 경계가
        // 바뀐 경우(새 씬 로드)에만 맞춘다 — 레이어 on/off 로는 현재 화면을 유지한다.
        private void OnSceneRebuilt()
        {
            ApplyGroundAndLabels();
            bool boundsChanged = !(NearlyEqual(_fitCx, _vm.GroundCenter.X)
                                && NearlyEqual(_fitW, _vm.GroundWidth)
                                && NearlyEqual(_fitL, _vm.GroundLength));
            if (boundsChanged)
            {
                _fitCx = _vm.GroundCenter.X;
                _fitW = _vm.GroundWidth;
                _fitL = _vm.GroundLength;
                View.ZoomExtents();
            }
        }

        // ↺ 전체보기 버튼(FitViewRequested) — 항상 강제로 전체보기.
        private void FitToScene()
        {
            ApplyGroundAndLabels();
            _fitCx = _vm.GroundCenter.X;
            _fitW = _vm.GroundWidth;
            _fitL = _vm.GroundLength;
            View.ZoomExtents();
        }

        // 공간 영역 텍스트 라벨(CR/A/F/CSF)을 BillboardText(항상 카메라를 향함)로 다시 그린다.
        // 텍스트는 Visual3D 라 Model3DGroup(SceneModel)에 못 넣으므로 뷰포트 자식으로 직접 관리한다.
        private void RebuildSpaceLabels()
        {
            foreach (var v in _spaceLabelVisuals) View.Children.Remove(v);
            _spaceLabelVisuals.Clear();
            foreach (var lab in _vm.SpaceLabels)
            {
                var t = new BillboardTextVisual3D
                {
                    Text = lab.Text,
                    Position = lab.Position,
                    Foreground = new SolidColorBrush(lab.Color),
                    Background = new SolidColorBrush(Color.FromArgb(120, 20, 24, 36)),
                    FontSize = 11,
                    FontWeight = FontWeights.Normal,
                };
                View.Children.Add(t);
                _spaceLabelVisuals.Add(t);
            }
        }

        // 단계(구간) 클릭 시 카메라를 그 위치로 이동(같은 거리·방향 유지하며 대상이 화면 중앙에 오게).
        private void NavigateTo(Point3D target)
        {
            if (View.Camera is not ProjectionCamera cam) return;
            var dir = cam.LookDirection;
            double dist = dir.Length;
            if (dist < 1) { dir = new Vector3D(1, 1, -1); dist = 9000; }
            dir.Normalize();
            cam.Position = target - dir * dist;
            cam.LookDirection = dir * dist;
        }

        private void View_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            var screen = e.GetPosition(View);
            var hit = View.FindNearestPoint(screen);  // 지오메트리 위의 최근접 3D 점.

            // 종단점 지정 모드가 아니면: 클릭 지점의 객체 속성을 우측 패널에 표시(회전은 그대로).
            if (_vm.PickMode == PickMode.None)
            {
                if (hit.HasValue) _vm.SelectObjectAt(hit.Value);
                return;
            }

            if (hit.HasValue)
            {
                _vm.ApplyPick(hit.Value);
                e.Handled = true;
            }
        }
    }
}

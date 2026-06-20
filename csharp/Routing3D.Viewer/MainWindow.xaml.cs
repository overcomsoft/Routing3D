using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Routing3D.Viewer.ViewModels;
using System.IO;

namespace Routing3D.Viewer
{
    /// <summary>메인 창. 뷰모델 연결 + 모델 재구성 시 카메라 자동 맞춤 + 3D 클릭 피킹(P3).</summary>
    public partial class MainWindow : Window
    {
        private readonly SceneViewModel _vm;
        private readonly List<Visual3D> _spaceLabelVisuals = new();   // 공간 영역 텍스트 라벨(BillboardText).
        private Views.ObjectInfoWindow? _objInfoWin;                  // 객체 속성 모드리스 다이얼로그(클릭마다 갱신).
        private Views.RouteReportWindow? _routeReportWin;            // 자동설계 결과 리포트 모드리스 창.
        private Views.TraceReplayWindow? _traceReplayWin;             // 탐색 trace 리플레이 창.

        public MainWindow(string? initialScene = null)
        {
            InitializeComponent();
            _vm = new SceneViewModel(initialScene);
            _vm.SceneRebuilt += () => Dispatcher.BeginInvoke(new Action(OnSceneRebuilt));
            _vm.FitViewRequested += () => Dispatcher.BeginInvoke(new Action(FitToScene));
            _vm.NavigateToRequested += target => Dispatcher.BeginInvoke(new Action(() => NavigateTo(target)));
            _vm.ZoomToBoxRequested += box => Dispatcher.BeginInvoke(new Action(() => ZoomToBox(box)));
            _vm.ShowRouteReportRequested += () => Dispatcher.BeginInvoke(new Action(ShowRouteReport));
            // 3D 클릭으로 SelectedTask 가 바뀌면 우측 DataGrid 를 해당 행으로 스크롤한다.
            _vm.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(SceneViewModel.SelectedTask) && _vm.SelectedTask != null)
                    Dispatcher.BeginInvoke(new Action(() => TaskDataGrid.ScrollIntoView(_vm.SelectedTask)));
            };
            DataContext = _vm;

            // 피킹 모드일 때만 좌클릭으로 3D 지점을 잡아 종단점으로 설정(평소 회전은 그대로).
            View.MouseLeftButtonDown += View_MouseLeftButtonDown;

            // 카메라 근/원 평면 동적 조정 — mm 단위 거대 씬에서 z-파이팅(가까이 가면 배관이 찢어져 보임) 방지.
            // 매 프레임 카메라 위치에 맞춰 깊이 범위를 씬 경계에 타이트하게 감싼다(깊이 정밀도 최대화).
            _lastFpsTick = Environment.TickCount64;
            CompositionTarget.Rendering += (_, __) => { UpdateCameraClipPlanes(); UpdatePerfOverlay(); };

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

        // 씬 경계 구(중심·반경) — 근/원 평면 동적 조정의 기준. OnSceneRebuilt 에서 갱신.
        private Point3D _sceneCenter = new Point3D(0, 0, 0);
        private double _sceneRadius = 1000;

        // ── FPS 카운터 ──────────────────────────────────────────────────────────────────────
        private int    _frameCount     = 0;
        private double _fps            = 0.0;
        private long   _lastFpsTick    = 0;   // Environment.TickCount64 기준
        private const double FpsInterval = 0.5;  // 0.5초마다 FPS 갱신

        // 매 프레임 카메라 위치/줌에 맞춰 근(Near)·원(Far) 평면을 씬 경계에 타이트하게 감싼다.
        //   원인: WPF 깊이 버퍼(24비트)는 near/far 비율이 클수록 정밀도가 급감 → 좌표가 수십만 mm 인
        //   거대 씬에서 기본 근평면(아주 작음)을 쓰면 카메라가 배관에 가까워질 때 겹친 표면이 z-파이팅으로
        //   찢어져 보인다. 깊이 범위를 '보이는 지오메트리'에만 맞추면 정밀도가 회복된다.
        private void UpdateCameraClipPlanes()
        {
            if (View.Camera is not ProjectionCamera cam) return;
            if (_sceneRadius <= 0 || double.IsNaN(_sceneRadius)) return;

            double camToCenter = (cam.Position - _sceneCenter).Length;
            double margin = _sceneRadius * 0.15 + 100.0;
            double far = camToCenter + _sceneRadius + margin;          // 씬 전체가 보이도록.

            // 근평면: 줌 거리(LookDirection 길이 = 주시점까지 거리)에 비례하되, near/far 비율을 1만 이하로
            //   유지(24비트 깊이 안정 한계)해 z-파이팅을 막는다. 동시에 배관(≈150mm)에 바짝 붙어도
            //   잘리지 않게 작은 하한을 둔다.
            double lookDist = cam.LookDirection.Length;
            double near = Math.Max(lookDist * 0.01, far / 10000.0);
            near = Math.Max(near, 1.0);
            if (near >= far) near = far * 0.001;

            if (Math.Abs(cam.NearPlaneDistance - near) > near * 0.02)
                cam.NearPlaneDistance = near;
            if (Math.Abs(cam.FarPlaneDistance - far) > far * 0.02)
                cam.FarPlaneDistance = far;
        }

        // 성능 오버레이 업데이트 — FPS + 렌더 오브젝트 수 + 옥트리 리프 수.
        // CompositionTarget.Rendering(매 프레임)에서 호출. 0.5초 간격으로 FPS 갱신.
        private void UpdatePerfOverlay()
        {
            _frameCount++;
            long now = Environment.TickCount64;
            double elapsedSec = (now - _lastFpsTick) * 0.001;
            if (elapsedSec >= FpsInterval)
            {
                _fps       = _frameCount / elapsedSec;
                _frameCount = 0;
                _lastFpsTick = now;

                // 렌더 오브젝트: SceneModel 내 GeometryModel3D 합산
                int objCount = 0;
                if (_vm.SceneModel is System.Windows.Media.Media3D.Model3DGroup grp)
                    foreach (var c in grp.Children)
                        if (c is System.Windows.Media.Media3D.GeometryModel3D) objCount++;

                // 옥트리 리프 수 (토글 OFF 면 0)
                int leafCount = _vm.OctreeLeafCount;

                PerfOverlay.Text =
                    $"FPS  {_fps:F1}\n" +
                    $"렌더 {objCount} obj\n" +
                    (leafCount > 0 ? $"옥트리 {leafCount:N0} 리프" : string.Empty);
            }
        }

        // SceneModel 등 뷰포트 콘텐츠의 합산 경계 → 씬 중심·반경 갱신(근/원 평면 기준).
        private void UpdateSceneBounds()
        {
            Rect3D b = Rect3D.Empty;
            foreach (var child in View.Children)
                if (child is ModelVisual3D mv && mv.Content is Model3D m)
                {
                    var mb = m.Bounds;
                    if (!mb.IsEmpty) b.Union(mb);
                }
            if (b.IsEmpty) return;
            _sceneCenter = new Point3D(b.X + b.SizeX / 2, b.Y + b.SizeY / 2, b.Z + b.SizeZ / 2);
            _sceneRadius = 0.5 * Math.Sqrt(b.SizeX * b.SizeX + b.SizeY * b.SizeY + b.SizeZ * b.SizeZ);
        }

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
            UpdateSceneBounds();   // 근/원 평면 기준(씬 중심·반경) 갱신.
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
            UpdateSceneBounds();   // 근/원 평면 기준(씬 중심·반경) 갱신.
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

        // 특정 영역(Rect3D)으로 줌 — 진행 다이얼로그 배관 행 클릭 시 그 배관 로컬 영역을 화면에 맞춘다.
        // 현재 시선 방향을 유지하면서, 카메라를 영역 중심에서 박스 반경에 비례한 거리(≈원근 FOV 보정)로 물린다.
        private void ZoomToBox(Rect3D box)
        {
            if (box.IsEmpty || View.Camera is not ProjectionCamera cam) return;
            var center = new Point3D(box.X + box.SizeX / 2, box.Y + box.SizeY / 2, box.Z + box.SizeZ / 2);
            double radius = 0.5 * Math.Sqrt(box.SizeX * box.SizeX + box.SizeY * box.SizeY + box.SizeZ * box.SizeZ);
            var dir = cam.LookDirection;
            if (dir.Length < 1) dir = new Vector3D(1, 1, -1);
            dir.Normalize();
            double dist = Math.Max(radius * 2.5, 1500);   // 원근 FOV~45° 기준 박스가 화면을 채우는 거리.
            cam.Position = center - dir * dist;
            cam.LookDirection = dir * dist;
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

        // 🖼 이미지 저장 — 현재 메인 3D 뷰를 PNG/JPG 로 저장(2배 해상도).
        private void OnSaveMainImage(object sender, RoutedEventArgs e)
            => ViewportImage.Save(View, "routing_mainview");

        // 📋 이미지 복사 — 현재 메인 3D 뷰를 클립보드에 복사(채팅/문서에 Ctrl+V 로 붙여넣기). 결과를 상태바에 안내.
        private void OnCopyMainImage(object sender, RoutedEventArgs e)
        {
            bool ok = ViewportImage.CopyToClipboard(View, 2.0);
            _vm.ShowStatus(ok ? "3D 뷰를 클립보드에 복사했습니다 — 붙여넣기(Ctrl+V) 하세요."
                              : "클립보드 복사에 실패했습니다(다시 시도하세요).");
        }

        // 객체 속성 모드리스 다이얼로그 — 없으면 만들어 띄우고, 이미 떠 있으면 앞으로(내용은 바인딩으로 자동 갱신).
        private void ShowObjectInfo()
        {
            if (_vm.SelectedObjectInfo == null) return;   // 잡힌 객체 없음 → 창 띄우지 않음.
            if (_objInfoWin == null)
            {
                _objInfoWin = new Views.ObjectInfoWindow { Owner = this, DataContext = _vm };
                _objInfoWin.Closed += (_, __) => _objInfoWin = null;
                // 메인 창 우측 상단 근처에 배치(메인 뷰를 가리지 않도록).
                _objInfoWin.Left = Left + Width - _objInfoWin.Width - 24;
                _objInfoWin.Top = Top + 80;
                _objInfoWin.Show();
            }
            else
            {
                _objInfoWin.Activate();
            }
        }

        // 자동설계 결과 리포트 모드리스 창 — 없으면 만들어 띄우고, 이미 떠 있으면 앞으로(내용은 바인딩으로 갱신).
        private void ShowRouteReport()
        {
            if (_routeReportWin == null)
            {
                _routeReportWin = new Views.RouteReportWindow { Owner = this, DataContext = _vm };
                _routeReportWin.Closed += (_, __) => _routeReportWin = null;
                _routeReportWin.Left = Left + Math.Max(40, Width - _routeReportWin.Width - 60);
                _routeReportWin.Top = Top + 60;
                _routeReportWin.Show();
            }
            else
            {
                _routeReportWin.Activate();
            }
        }

        private void OnOpenTraceReplay(object sender, RoutedEventArgs e)
        {
            string? path = !string.IsNullOrWhiteSpace(_vm.LastSearchTraceFile) && File.Exists(_vm.LastSearchTraceFile)
                ? _vm.LastSearchTraceFile
                : null;

            if (_traceReplayWin == null)
            {
                _traceReplayWin = new Views.TraceReplayWindow(path) { Owner = this };
                _traceReplayWin.Closed += (_, __) => _traceReplayWin = null;
                _traceReplayWin.Left = Left + 60;
                _traceReplayWin.Top = Top + 60;
                _traceReplayWin.Show();
            }
            else
            {
                _traceReplayWin.Activate();
            }
        }

        private void View_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            var screen = e.GetPosition(View);
            var hit = View.FindNearestPoint(screen);  // 지오메트리 위의 최근접 3D 점.

            // 종단점 지정 모드가 아니면: 클릭 지점의 객체 속성을 모드리스 다이얼로그에 표시(회전은 그대로).
            if (_vm.PickMode == PickMode.None)
            {
                if (hit.HasValue) { _vm.SelectObjectAt(hit.Value); ShowObjectInfo(); }
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

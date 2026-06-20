// 자동설계 결과 세션 선택 창 — 세션 목록 + 세부 배관 그리드 + 삭제
using HelixToolkit.Wpf;
using Routing3D.Viewer.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Routing3D.Viewer.Views
{
    /// <summary>DataGrid 바인딩용 — AutoRoutingSessionRow 에 표시 편의 속성 추가.</summary>
    public sealed class SessionPickerItem
    {
        public AutoRoutingSessionRow Row { get; }
        public SessionPickerItem(AutoRoutingSessionRow r) => Row = r;

        public System.DateTime LastModifiedAt => Row.LastModifiedAt.ToLocalTime();
        public string EquipmentName => Row.EquipmentName;
        public int    CellMm        => Row.CellMm;
        public string SuccessDisplay => $"{Row.SuccessCount}/{Row.TotalCount}";
        public string LengthDisplay  => $"{Row.TotalLengthMm / 1000.0:N1}";
        public string? RouteMode     => Row.RouteMode;
        public string ProjectName    => Row.ProjectName;
    }

    /// <summary>세부 배관 그리드 1행.</summary>
    public sealed class PathDetailItem
    {
        public int    RouteOrder      { get; init; }
        public string UtilityGroup    { get; init; } = "";
        public string Utility         { get; init; } = "";
        public string SourceName      { get; init; } = "";
        public string TargetName      { get; init; } = "";
        public string DiameterDisplay { get; init; } = "";
        public bool   Success         { get; init; }
        public string StatusText      => Success ? "성공" : "실패";
        public string? FailReason     { get; init; }
        public string LengthDisplay   { get; init; } = "";
        public int    TurnCount       { get; init; }
        public int    ElapsedMs       { get; init; }
        public string GeometryStatus  { get; init; } = "-";
        public AutoRoutingPathRow Row { get; init; } = new();

        public static PathDetailItem From(AutoRoutingPathRow p) => new()
        {
            RouteOrder      = p.RouteOrder,
            UtilityGroup    = p.UtilityGroup  ?? "",
            Utility         = p.Utility        ?? "",
            SourceName      = p.SourceName     ?? "",
            TargetName      = p.TargetName     ?? "",
            DiameterDisplay = p.DiameterMm > 0 ? $"{p.DiameterMm:0}" : "-",
            Success         = p.Success,
            FailReason      = p.FailReason,
            LengthDisplay   = p.Success ? $"{p.LengthMm / 1000.0:N2}" : "",
            TurnCount       = p.TurnCount,
            ElapsedMs       = p.ElapsedMs,
            GeometryStatus  = p.Polyline != null && p.Polyline.Count >= 2 ? "LineStringZ" : "-",
            Row             = p,
        };
    }

    public partial class SessionPickerWindow : Window
    {
        public AutoRoutingSessionRow? Selected { get; private set; }

        private readonly DbConfig _dbConfig;
        private readonly List<SessionPickerItem> _items;

        public SessionPickerWindow(IEnumerable<AutoRoutingSessionRow> sessions, DbConfig dbConfig)
        {
            InitializeComponent();
            _dbConfig = dbConfig;
            _items = new List<SessionPickerItem>();
            foreach (var s in sessions) _items.Add(new SessionPickerItem(s));
            SessionGrid.ItemsSource = _items;
            if (_items.Count > 0) SessionGrid.SelectedIndex = 0;
        }

        // ── 세션 선택 변경 → 세부 배관 로드 ─────────────────────────────────
        private async void OnSessionSelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (SessionGrid.SelectedItem is not SessionPickerItem item) return;
            await LoadDetailAsync(item.Row);
        }

        private async Task LoadDetailAsync(AutoRoutingSessionRow session)
        {
            DetailGrid.ItemsSource = null;
            DetailStatusText.Text = "로드 중...";
            try
            {
                var paths = await Task.Run(() =>
                    AutoRoutingRepository.LoadPathsAsync(_dbConfig, session.SessionId));

                var rows = new List<PathDetailItem>(paths.Count);
                foreach (var p in paths) rows.Add(PathDetailItem.From(p));
                DetailGrid.ItemsSource = rows;
                DetailGrid.SelectedIndex = rows.FindIndex(r => r.Row.Polyline != null && r.Row.Polyline.Count >= 2);
                if (DetailGrid.SelectedIndex < 0) ClearPreview("LineStringZ 없음");

                int success = 0;
                foreach (var p in paths) if (p.Success) success++;
                DetailStatusText.Text =
                    $"— 배관 {paths.Count}개  (성공 {success} / 실패 {paths.Count - success})";
            }
            catch (Exception ex)
            {
                DetailStatusText.Text = $"로드 실패: {ex.Message}";
            }
        }

        private void OnDetailSelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (DetailGrid.SelectedItem is PathDetailItem item) ShowPathPreview(item);
        }

        private void ShowPathPreview(PathDetailItem item)
        {
            var pts = item.Row.Polyline;
            if (pts == null || pts.Count < 2)
            {
                ClearPreview("LineStringZ 없음");
                return;
            }

            var group = new Model3DGroup();
            double diameter = item.Row.DiameterMm > 0 ? item.Row.DiameterMm : 80.0;
            double tubeDia = Math.Max(diameter, 40.0);
            double markerR = Math.Max(tubeDia * 1.4, 70.0);

            int startCount = Math.Clamp(item.Row.StartStubPointCount, 0, pts.Count);
            int endCount = Math.Clamp(item.Row.EndStubPointCount, 0, pts.Count - startCount);

            AddTubeSegment(group, pts, 0, startCount - 1, tubeDia * 1.12, Color.FromRgb(0xf5, 0x9e, 0x0b), 245);

            int mainStart = startCount >= 2 ? startCount - 1 : 0;
            int mainEnd = endCount >= 2 ? pts.Count - endCount : pts.Count - 1;
            AddTubeSegment(group, pts, mainStart, mainEnd, tubeDia, Color.FromRgb(0x38, 0xbd, 0xf8), 230);

            int endStart = pts.Count - endCount;
            AddTubeSegment(group, pts, endStart, pts.Count - 1, tubeDia * 1.12, Color.FromRgb(0xa8, 0x55, 0xf7), 245);

            var start = new MeshBuilder(false, false);
            start.AddSphere(pts[0], markerR);
            group.Children.Add(Geometry(start, Color.FromRgb(0x22, 0xc5, 0x5e), 245));

            var end = new MeshBuilder(false, false);
            end.AddSphere(pts[^1], markerR);
            group.Children.Add(Geometry(end, Color.FromRgb(0xef, 0x44, 0x44), 245));

            PreviewModelVisual.Content = group;
            PreviewStatusText.Text = $"{pts.Count}점 · 시작Stub {startCount} · 종료Stub {endCount} · {item.LengthDisplay} m";
            Dispatcher.BeginInvoke(new Action(() => { try { PreviewView.ZoomExtents(); } catch { } }),
                System.Windows.Threading.DispatcherPriority.Background);
        }

        private static void AddTubeSegment(Model3DGroup group, List<Point3D> pts, int startIndex, int endIndex,
                                           double diameter, Color color, byte alpha)
        {
            if (startIndex < 0 || endIndex >= pts.Count || endIndex - startIndex < 1) return;
            var seg = pts.Skip(startIndex).Take(endIndex - startIndex + 1).ToList();
            if (seg.Count < 2) return;
            var mb = new MeshBuilder(false, false);
            mb.AddTube(seg, diameter, 12, false);
            group.Children.Add(Geometry(mb, color, alpha));
        }
        private void ClearPreview(string text)
        {
            PreviewModelVisual.Content = null;
            PreviewStatusText.Text = text;
        }

        private static GeometryModel3D Geometry(MeshBuilder mb, Color color, byte alpha)
        {
            var brush = new SolidColorBrush(Color.FromArgb(alpha, color.R, color.G, color.B));
            brush.Freeze();
            var material = MaterialHelper.CreateMaterial(brush);
            return new GeometryModel3D(mb.ToMesh(), material) { BackMaterial = material };
        }
        // ── 삭제 ─────────────────────────────────────────────────────────────
        private async void OnDelete(object sender, RoutedEventArgs e)
        {
            if (SessionGrid.SelectedItem is not SessionPickerItem item) return;

            var msg = $"선택한 세션을 삭제하시겠습니까?\n\n" +
                      $"장비: {item.EquipmentName}\n" +
                      $"저장시간: {item.LastModifiedAt:yyyy-MM-dd HH:mm}\n" +
                      $"성공/전체: {item.SuccessDisplay}\n\n" +
                      "이 작업은 되돌릴 수 없습니다.";
            var result = MessageBox.Show(msg, "세션 삭제 확인",
                MessageBoxButton.YesNo, MessageBoxImage.Warning, MessageBoxResult.No);
            if (result != MessageBoxResult.Yes) return;

            try
            {
                await Task.Run(() =>
                    AutoRoutingRepository.DeleteSessionAsync(_dbConfig, item.Row.SessionId));

                int idx = _items.IndexOf(item);
                _items.Remove(item);
                // ItemsSource 갱신
                SessionGrid.ItemsSource = null;
                SessionGrid.ItemsSource = _items;
                DetailGrid.ItemsSource = null;
                DetailStatusText.Text = "삭제되었습니다.";

                if (_items.Count == 0)
                {
                    MessageBox.Show("저장된 세션이 없습니다.", "알림",
                        MessageBoxButton.OK, MessageBoxImage.Information);
                    DialogResult = false;
                    return;
                }
                // 인접 행 선택
                SessionGrid.SelectedIndex = Math.Min(idx, _items.Count - 1);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"삭제 실패:\n{ex.Message}", "오류",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        // ── 불러오기 / 취소 ─────────────────────────────────────────────────
        private void OnLoad(object sender, RoutedEventArgs e)
        {
            if (SessionGrid.SelectedItem is SessionPickerItem item)
            {
                Selected = item.Row;
                DialogResult = true;
            }
            else
            {
                MessageBox.Show("세션을 선택해주세요.", "선택 없음",
                    MessageBoxButton.OK, MessageBoxImage.Information);
            }
        }

        private void OnCancel(object sender, RoutedEventArgs e) => DialogResult = false;

        private void OnSessionDoubleClick(object sender, MouseButtonEventArgs e)
        {
            if (SessionGrid.SelectedItem is SessionPickerItem)
                OnLoad(sender, e);
        }
    }
}

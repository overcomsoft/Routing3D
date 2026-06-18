// 자동설계 결과 세션 선택 창 — 세션 목록 + 세부 배관 그리드 + 삭제
using Routing3D.Viewer.Model;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;

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

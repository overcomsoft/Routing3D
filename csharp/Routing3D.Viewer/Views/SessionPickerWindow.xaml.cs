// 자동설계 결과 세션 선택 창
using Routing3D.Viewer.Model;
using System.Collections.Generic;
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

    public partial class SessionPickerWindow : Window
    {
        public AutoRoutingSessionRow? Selected { get; private set; }

        public SessionPickerWindow(IEnumerable<AutoRoutingSessionRow> sessions)
        {
            InitializeComponent();
            var items = new List<SessionPickerItem>();
            foreach (var s in sessions) items.Add(new SessionPickerItem(s));
            SessionGrid.ItemsSource = items;
            if (items.Count > 0) SessionGrid.SelectedIndex = 0;
        }

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

// 라우팅 진행 다이얼로그 — 네이티브 진행 콜백(r3d_route_multi_progress)을 받아 배관별 처리 순서·
// 성공/실패·실패 추정 사유·지표를 실시간 표로 보여준다(그룹/유틸 전체 라우팅 진단용).
// =============================================================================
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using Routing3D.Viewer.Interop;

namespace Routing3D.Viewer.Views
{
    public partial class RoutingProgressWindow : Window, INotifyPropertyChanged
    {
        // 네이티브 route_multi 의 대형격자 탐색 상한(routing3d_capi.cpp 의 max_exp). 실패 사유 추정에 사용.
        private const long ExpansionCap = 12_000_000L;

        private int _success, _fail;
        private readonly Dictionary<string, int> _reasonHist = new();

        public ObservableCollection<ProgressRow> Rows { get; } = new();

        private string _headerText = "라우팅 준비 중…";
        public string HeaderText { get => _headerText; set { _headerText = value; OnPc(nameof(HeaderText)); } }
        private string _failSummary = string.Empty;
        public string FailSummary { get => _failSummary; set { _failSummary = value; OnPc(nameof(FailSummary)); } }
        private int _done;
        public int Done { get => _done; set { _done = value; OnPc(nameof(Done)); } }
        private int _total = 1;
        public int Total { get => _total; set { _total = value; OnPc(nameof(Total)); } }

        public RoutingProgressWindow()
        {
            InitializeComponent();
            DataContext = this;
        }

        /// <summary>라우팅 시작 시 1회 — 라벨/총 작업수 표시.</summary>
        public void Begin(string label, int total)
        {
            Total = Math.Max(1, total);
            HeaderText = $"{label} — 라우팅 중… 0/{total}";
        }

        /// <summary>배관 1개 처리 보고. 네이티브 콜백 스레드에서 호출돼도 안전하도록 UI 스레드로 마샬링.</summary>
        public void ReportPipe(Engine.RouteProgress p, string label)
        {
            if (!Dispatcher.CheckAccess()) { Dispatcher.BeginInvoke(new Action(() => ReportPipe(p, label))); return; }

            string reason = string.Empty;
            if (!p.Success)
            {
                if (p.ExpandedNodes <= 1) reason = "시작/끝 셀 막힘(장애물 내부)";
                else if (p.ExpandedNodes >= ExpansionCap) reason = "탐색 상한 도달(경로 길거나 없음)";
                else reason = "경로 없음(완전 차단)";
                _fail++;
                _reasonHist[reason] = _reasonHist.TryGetValue(reason, out var c) ? c + 1 : 1;
            }
            else _success++;

            Rows.Add(new ProgressRow
            {
                Order = p.OrderIndex + 1,
                Label = label,
                Success = p.Success,
                Reason = reason,
                LengthText = p.Success ? p.LengthMm.ToString("0") : "-",
                Turns = p.Success ? p.Turns.ToString() : "-",
                ExpandedText = p.ExpandedNodes.ToString("#,0"),
                ElapsedText = p.ElapsedMs.ToString("0"),
            });

            Done = p.Done;
            HeaderText = $"라우팅 중… {p.Done}/{p.Total} · 성공 {_success} · 실패 {_fail}";
            UpdateFailSummary();
        }

        /// <summary>라우팅 종료 — 최종 요약. failedToRun=true 면 예외로 중단된 경우.</summary>
        public void Complete(bool failedToRun = false, string? error = null)
        {
            if (failedToRun)
            {
                HeaderText = $"라우팅 중단(오류) · 성공 {_success} · 실패 {_fail}";
                if (!string.IsNullOrEmpty(error)) FailSummary = "오류: " + error;
                return;
            }
            HeaderText = $"완료 · 성공 {_success}/{_success + _fail} · 실패 {_fail}";
            UpdateFailSummary();
        }

        private void UpdateFailSummary()
        {
            if (_reasonHist.Count == 0) { FailSummary = string.Empty; return; }
            FailSummary = "실패 사유: " +
                string.Join(" · ", _reasonHist.OrderByDescending(kv => kv.Value)
                                              .Select(kv => $"{kv.Key} {kv.Value}건"));
        }

        private void OnClose(object sender, RoutedEventArgs e) => Close();

        public event PropertyChangedEventHandler? PropertyChanged;
        private void OnPc(string n) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(n));
    }

    /// <summary>진행 표 한 행(추가 후 불변).</summary>
    public sealed class ProgressRow
    {
        public int Order { get; init; }
        public string Label { get; init; } = string.Empty;
        public bool Success { get; init; }
        public string StatusText => Success ? "성공" : "실패";
        public Brush StatusBrush => Success
            ? new SolidColorBrush(Color.FromRgb(0x5f, 0xcf, 0x80))
            : new SolidColorBrush(Color.FromRgb(0xff, 0x6b, 0x6b));
        public string Reason { get; init; } = string.Empty;
        public string LengthText { get; init; } = string.Empty;
        public string Turns { get; init; } = string.Empty;
        public string ExpandedText { get; init; } = string.Empty;
        public string ElapsedText { get; init; } = string.Empty;
    }
}

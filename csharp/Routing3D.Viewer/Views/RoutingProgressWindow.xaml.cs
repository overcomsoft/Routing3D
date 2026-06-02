// 라우팅 진행 다이얼로그 — 네이티브 진행 콜백(r3d_route_multi_progress, phase 0/1)을 받아 배관별
// 그룹/유틸·시작/종단 PoC·처리상태(엔진 단계별 진행율 %)·성공/실패·사유·지표를 실시간 표로 보여준다.
// 행 생성/갱신은 order_index(처리 순서) 기준. 3D 화면 갱신은 VM 콜백에서 수행한다.
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
        private const long ExpansionCap = 12_000_000L;  // 네이티브 대형격자 탐색 상한(실패 사유 추정).

        private int _success, _fail;
        private readonly Dictionary<string, int> _reasonHist = new();
        private readonly Dictionary<int, ProgressRow> _byOrder = new();  // order_index → 행.

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

        public void Begin(string label, int total)
        {
            Total = Math.Max(1, total);
            HeaderText = $"{label} — 라우팅 중… 0/{total}";
        }

        /// <summary>진행 이벤트 보고. 네이티브 콜백 스레드에서 호출돼도 UI 스레드로 마샬링한다.
        /// phase 0=탐색 진행(처리상태 %), phase 1=배관 완료(성공/실패·지표).</summary>
        public void ReportPipe(Engine.RouteProgress p, string group, string utility,
                               string startPoC, string endPoC)
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.BeginInvoke(new Action(() => ReportPipe(p, group, utility, startPoC, endPoC)));
                return;
            }

            // 행 확보(탐색 시작 또는 완료 어느 쪽이든 처음 보이면 생성).
            if (!_byOrder.TryGetValue(p.OrderIndex, out var row))
            {
                row = new ProgressRow
                {
                    Order = p.OrderIndex + 1,
                    Group = group,
                    Utility = utility,
                    StartPoC = startPoC,
                    EndPoC = endPoC,
                };
                _byOrder[p.OrderIndex] = row;
                Rows.Add(row);
            }

            if (p.Phase == 0)
            {
                // 탐색 진행 — 처리상태를 진행율(%)로. 확장수도 누적 표시.
                row.SetStage(ProgressStage.Searching, $"탐색 {p.Progress01 * 100:0}%");
                row.ExpandedText = p.ExpandedNodes.ToString("#,0");
                return;
            }

            // phase == 1: 완료.
            string reason = string.Empty;
            if (p.Success)
            {
                _success++;
                row.SetStage(ProgressStage.Done, "완료");
                row.LengthText = p.LengthMm.ToString("0");
                row.TurnsText = p.Turns.ToString();
            }
            else
            {
                if (p.ExpandedNodes <= 1) reason = "시작/끝 셀 막힘(장애물 내부)";
                else if (p.ExpandedNodes >= ExpansionCap) reason = "탐색 상한 도달(경로 길거나 없음)";
                else reason = "경로 없음(완전 차단)";
                _fail++;
                _reasonHist[reason] = _reasonHist.TryGetValue(reason, out var c) ? c + 1 : 1;
                row.SetStage(ProgressStage.Failed, "실패");
                row.LengthText = "-";
                row.TurnsText = "-";
            }
            row.Reason = reason;
            row.ExpandedText = p.ExpandedNodes.ToString("#,0");
            row.ElapsedText = p.ElapsedMs.ToString("0");

            Done = p.Done;
            HeaderText = $"라우팅 중… {p.Done}/{p.Total} · 성공 {_success} · 실패 {_fail}";
            UpdateFailSummary();
        }

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
            FailSummary = _reasonHist.Count == 0 ? string.Empty
                : "실패 사유: " + string.Join(" · ", _reasonHist.OrderByDescending(kv => kv.Value)
                                                                .Select(kv => $"{kv.Key} {kv.Value}건"));
        }

        private void OnClose(object sender, RoutedEventArgs e) => Close();

        public event PropertyChangedEventHandler? PropertyChanged;
        private void OnPc(string n) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(n));
    }

    public enum ProgressStage { Queued, Searching, Done, Failed }

    /// <summary>진행 표 한 행 — 처리 중 상태/지표가 갱신되므로 INotifyPropertyChanged.</summary>
    public sealed class ProgressRow : INotifyPropertyChanged
    {
        public int Order { get; init; }
        public string Group { get; init; } = string.Empty;
        public string Utility { get; init; } = string.Empty;
        public string StartPoC { get; init; } = string.Empty;
        public string EndPoC { get; init; } = string.Empty;

        private string _statusText = "대기";
        public string StatusText { get => _statusText; private set { _statusText = value; Pc(nameof(StatusText)); } }

        private Brush _statusBrush = new SolidColorBrush(Color.FromRgb(0x9a, 0xa3, 0xb5));  // 대기=회색.
        public Brush StatusBrush { get => _statusBrush; private set { _statusBrush = value; Pc(nameof(StatusBrush)); } }

        public void SetStage(ProgressStage stage, string text)
        {
            StatusText = text;
            StatusBrush = new SolidColorBrush(stage switch
            {
                ProgressStage.Searching => Color.FromRgb(0xf0, 0xc8, 0x4a),  // 탐색=노랑.
                ProgressStage.Done => Color.FromRgb(0x5f, 0xcf, 0x80),       // 완료=녹색.
                ProgressStage.Failed => Color.FromRgb(0xff, 0x6b, 0x6b),     // 실패=적색.
                _ => Color.FromRgb(0x9a, 0xa3, 0xb5),
            });
        }

        private string _reason = string.Empty;
        public string Reason { get => _reason; set { _reason = value; Pc(nameof(Reason)); } }
        private string _lengthText = string.Empty;
        public string LengthText { get => _lengthText; set { _lengthText = value; Pc(nameof(LengthText)); } }
        private string _turnsText = string.Empty;
        public string TurnsText { get => _turnsText; set { _turnsText = value; Pc(nameof(TurnsText)); } }
        private string _expandedText = string.Empty;
        public string ExpandedText { get => _expandedText; set { _expandedText = value; Pc(nameof(ExpandedText)); } }
        private string _elapsedText = string.Empty;
        public string ElapsedText { get => _elapsedText; set { _elapsedText = value; Pc(nameof(ElapsedText)); } }

        public event PropertyChangedEventHandler? PropertyChanged;
        private void Pc(string n) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(n));
    }
}

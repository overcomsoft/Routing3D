// ReportWindow.xaml.cs — 8단계 파이프라인 실행 결과 표시 창
// 사용법: new ReportWindow(results) { Owner = this }.Show();
using System.Diagnostics;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using Routing3D.Samples.LargeSpace10mm.Pipeline;

namespace Routing3D.Samples.LargeSpace10mm;

public partial class ReportWindow : Window
{
    private readonly IReadOnlyList<StepResult> _steps;
    private string? _mdFilePath;

    public ReportWindow(IReadOnlyList<StepResult> steps)
    {
        _steps = steps;
        InitializeComponent();
        Loaded += OnWindowLoaded;
    }

    private void OnWindowLoaded(object sender, RoutedEventArgs e)
    {
        // 단계 목록 바인딩
        StepList.ItemsSource = _steps;

        // Step 8 에서 숨긴 '_filepath' 메트릭으로 저장경로 복원
        var step8 = _steps.FirstOrDefault(s => s.StepNumber == 8);
        _mdFilePath = step8?.Metrics.FirstOrDefault(m => m.Key == "_filepath").Value;

        // 하단 파일 경로 표시
        FooterText.Text = _mdFilePath is null
            ? "Markdown 파일 없음"
            : $"저장: {_mdFilePath}";

        OpenFileButton.IsEnabled = _mdFilePath is not null && File.Exists(_mdFilePath);

        // 첫 번째 단계 선택
        if (_steps.Count > 0)
            StepList.SelectedIndex = 0;
    }

    private void StepList_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (StepList.SelectedItem is not StepResult step)
        {
            ClearDetail();
            return;
        }

        DetailTitle.Text = $"Step {step.StepNumber}: {step.StepName}  {step.StatusIcon}  [{step.WallTimeStr}]";
        DetailDesc.Text  = step.StepDesc;

        // 메트릭 — '_' 접두사 숨김
        MetricsGrid.ItemsSource = step.Metrics
            .Where(m => !m.Key.StartsWith('_'))
            .Select(m => new { m.Key, m.Value })
            .ToList();

        // 작업별 결과
        TasksGrid.ItemsSource = step.Tasks.Count > 0 ? step.Tasks : null;

        // 로그
        LogBox.Text = step.Log.Count > 0 ? string.Join(Environment.NewLine, step.Log) : "(로그 없음)";
        LogBox.ScrollToHome();
    }

    private void ClearDetail()
    {
        DetailTitle.Text = "";
        DetailDesc.Text  = "";
        MetricsGrid.ItemsSource = null;
        TasksGrid.ItemsSource   = null;
        LogBox.Text = "";
    }

    private void OpenFileButton_Click(object sender, RoutedEventArgs e)
    {
        if (_mdFilePath is null || !File.Exists(_mdFilePath)) return;
        try { Process.Start(new ProcessStartInfo(_mdFilePath) { UseShellExecute = true }); }
        catch (Exception ex) { MessageBox.Show($"파일 열기 실패:\n{ex.Message}", "오류", MessageBoxButton.OK, MessageBoxImage.Warning); }
    }

    private void CopyButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            string md = PipelineRunner.BuildMarkdown(_steps);
            Clipboard.SetText(md);
            CopyButton.Content = "✓ 복사됨";
            var t = new System.Windows.Threading.DispatcherTimer { Interval = TimeSpan.FromSeconds(2) };
            t.Tick += (_, _) => { CopyButton.Content = "📋 복사"; t.Stop(); };
            t.Start();
        }
        catch (Exception ex) { MessageBox.Show($"복사 실패:\n{ex.Message}", "오류", MessageBoxButton.OK, MessageBoxImage.Warning); }
    }

    private void CloseButton_Click(object sender, RoutedEventArgs e) => Close();
}

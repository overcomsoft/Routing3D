using System;
using System.IO;
using System.Linq;
using System.Windows;
using Routing3D.Viewer.ViewModels;

namespace Routing3D.Viewer
{
    /// <summary>
    /// WPF 진입. 명령행 인자 처리:
    ///   (없음)                         → 내장 데모로 창 표시
    ///   &lt;scene.txt&gt;               → 그 scene 을 로드해 창 표시
    ///   --selftest &lt;scene&gt; &lt;out&gt; → 창 없이 전체 파이프라인(엔진 라우팅+모델 구성) 실행 후
    ///                                    상태 문자열을 &lt;out&gt; 파일에 쓰고 종료(헤드리스 검증).
    /// </summary>
    public partial class App : Application
    {
        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);
            var args = e.Args;

            // 헤드리스 DB 라우팅 진단: --dbroute <projectId> <cellMm> <utility> <outPath>
            int dr = Array.FindIndex(args, a => a == "--dbroute");
            if (dr >= 0 && dr + 4 < args.Length)
            {
                try
                {
                    int pid = int.Parse(args[dr + 1]);
                    double cell = double.Parse(args[dr + 2], System.Globalization.CultureInfo.InvariantCulture);
                    string util = args[dr + 3];
                    string outp = args[dr + 4];
                    string report = Diagnostics.DbRouteDiag.Run(pid, cell, util);
                    File.WriteAllText(outp, report);
                }
                catch (Exception ex) { File.WriteAllText(args[dr + 4], "DIAG ERROR: " + ex); }
                Shutdown(0);
                return;
            }

            // 다단 랙(z-레벨) 구조 추출 분석(헤드리스): --racktiers <projectId> <cellMm> <outPath>
            int rt = Array.FindIndex(args, a => a == "--racktiers");
            if (rt >= 0 && rt + 3 < args.Length)
            {
                try
                {
                    int pid = int.Parse(args[rt + 1]);
                    double cell = double.Parse(args[rt + 2], System.Globalization.CultureInfo.InvariantCulture);
                    string outp = args[rt + 3];
                    string report = Diagnostics.DbRouteDiag.RunRackTiers(pid, cell);
                    File.WriteAllText(outp, report, new System.Text.UTF8Encoding(true));
                }
                catch (Exception ex) { File.WriteAllText(args[rt + 3], "RACKTIERS ERROR: " + ex); }
                Shutdown(0);
                return;
            }

            // AI 자동설계 비교 리포트(헤드리스): --autodesign-report <projectId> <cellMm> <outDir> [maxCases]
            //   → CSV/TXT/HTML + img/ 3D 스냅샷(P4). env R3D_ADR_NOIMG=1 이면 스냅샷/HTML 생략.
            int ar = Array.FindIndex(args, a => a == "--autodesign-report");
            if (ar >= 0 && ar + 3 < args.Length)
            {
                try
                {
                    int pid = int.Parse(args[ar + 1]);
                    double cell = double.Parse(args[ar + 2], System.Globalization.CultureInfo.InvariantCulture);
                    string outDir = Path.GetFullPath(args[ar + 3]);   // cwd 기준 절대경로(상대경로 혼동 방지).
                    Directory.CreateDirectory(outDir);
                    int maxCases = (ar + 4 < args.Length && int.TryParse(args[ar + 4], out var mc)) ? mc : 0;
                    string report = Diagnostics.AutoDesignReport.Run(pid, cell, outDir, maxCases);
                    File.WriteAllText(Path.Combine(outDir, "_run.log"), report,
                                      new System.Text.UTF8Encoding(true));
                }
                catch (Exception ex)
                {
                    try { File.WriteAllText(Path.Combine(args[ar + 3], "_run.log"), "AUTODESIGN ERROR: " + ex); }
                    catch { }
                }
                Shutdown(0);
                return;
            }

            int st = Array.FindIndex(args, a => a == "--selftest");
            if (st >= 0 && st + 2 < args.Length)
            {
                string scene = args[st + 1];
                string outPath = args[st + 2];
                try
                {
                    var vm = new SceneViewModel(scene);   // 파싱 + P/Invoke 라우팅 + 모델 구성(창 불필요)
                    File.WriteAllText(outPath, vm.Status);
                }
                catch (Exception ex)
                {
                    File.WriteAllText(outPath, "ERROR: " + ex);
                }
                Shutdown(0);
                return;
            }

            string? initialScene = args.FirstOrDefault(a =>
                a.EndsWith(".scene.txt", StringComparison.OrdinalIgnoreCase) ||
                a.EndsWith(".txt", StringComparison.OrdinalIgnoreCase));
            new MainWindow(initialScene).Show();
        }
    }
}

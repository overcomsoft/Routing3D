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

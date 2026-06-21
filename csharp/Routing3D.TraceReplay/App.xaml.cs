using System;
using System.IO;
using System.Linq;
using System.Windows;
using Routing3D.Viewer.Views;

namespace Routing3D.TraceReplay
{
    public partial class App : Application
    {
        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            string? tracePath = e.Args.FirstOrDefault(a =>
                !string.IsNullOrWhiteSpace(a) &&
                File.Exists(a) &&
                (a.EndsWith(".r3dtrace.jsonl", StringComparison.OrdinalIgnoreCase) ||
                 a.EndsWith(".jsonl", StringComparison.OrdinalIgnoreCase)));

            var window = new TraceReplayWindow(tracePath);
            MainWindow = window;
            window.Show();
        }
    }
}

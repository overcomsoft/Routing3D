using System.IO;
using System.Windows;
using System.Windows.Threading;

namespace Routing3D.Samples.LargeSpace10mm;

public partial class App : Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        DispatcherUnhandledException += OnDispatcherUnhandledException;
        AppDomain.CurrentDomain.UnhandledException += (_, args) => WriteException(args.ExceptionObject as Exception);
        TaskScheduler.UnobservedTaskException += (_, args) => { WriteException(args.Exception); args.SetObserved(); };
        base.OnStartup(e);
    }

    private static void OnDispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
    {
        WriteException(e.Exception);
        e.Handled = true;
    }

    private static void WriteException(Exception? ex)
    {
        try
        {
            string path = Path.Combine(Path.GetTempPath(), "Routing3D.LargeSpace10mm.error.log");
            File.AppendAllText(path, DateTime.Now.ToString("s") + Environment.NewLine + ex + Environment.NewLine + Environment.NewLine);
        }
        catch { }
    }
}

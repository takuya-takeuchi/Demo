using System;
using System.Threading.Tasks;
using System.Windows;

using NLog;

namespace Demo
{
    /// <summary>
    /// App.xaml の相互作用ロジック
    /// </summary>
    public partial class App : Application
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        #region Overrids

        protected override void OnStartup(StartupEventArgs e)
        {
            DispatcherUnhandledException += (o, args) =>
            {
                Logger.Fatal($"{nameof(DispatcherUnhandledException)}");

                args.Handled = true;

                var canContinueRunning = false;
                if (canContinueRunning)
                {
                    // Notify to user
                }
                else
                {
                    // Notify to user and release resources here

                    Environment.Exit(1);
                }

            };

            AppDomain.CurrentDomain.UnhandledException += (o, args) =>
            {
                Logger.Fatal($"{nameof(AppDomain.CurrentDomain.UnhandledException)}");

                // Notify to user and release resources here

                Environment.Exit(1);
            };

            TaskScheduler.UnobservedTaskException += (o, args) =>
            {
                Logger.Fatal($"{nameof(TaskScheduler.UnobservedTaskException)}");

                // Marks the Exception as "observed," thus preventing it from triggering exception escalation policy, which, by default, terminates the process.
                args.SetObserved();
            };

            base.OnStartup(e);
        }

        #endregion

        #endregion

    }

}

using System.Windows;

using NLog;

namespace Demo
{

    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        #region Overrides

        protected override void OnStartup(StartupEventArgs e)
        {
            Logger.Info($"{nameof(this.OnStartup)}");
            base.OnStartup(e);
        }

        #endregion

        #endregion

    }

}

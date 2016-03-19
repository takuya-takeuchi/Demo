using System.Windows;
using GalaSoft.MvvmLight.Threading;
using MetroRadiance.UI;

namespace CNTK6
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        static App()
        {
            DispatcherHelper.Initialize();
        }
        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            ThemeService.Current.Register(this, Theme.Dark, Accent.Blue);
        }

    }
}

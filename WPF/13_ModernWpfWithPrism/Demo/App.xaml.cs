using System.Windows;

using Prism.Ioc;
using Prism.Unity;

namespace Demo
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : PrismApplication
    {

        #region Methods

        #region Overrids

        protected override Window CreateShell()
        {
            return Container.Resolve<MainWindow>();
        }

        protected override void RegisterTypes(IContainerRegistry containerRegistry)
        {
            //containerRegistry.RegisterForNavigation<Views.UserControl1>();
            //containerRegistry.RegisterForNavigation<Views.UserControl2>();
        }

        #endregion

        #endregion
        
    }

}

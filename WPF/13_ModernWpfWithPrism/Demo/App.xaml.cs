using System.Windows;
using System.Windows.Threading;

using Prism.Ioc;
using Prism.Unity;

using Demo.Services;
using Demo.Services.Interfaces;
using Demo.ViewModels;
using Demo.Views;

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
            this.RegisterServices(containerRegistry);
            this.RegisterViewModels(containerRegistry);

            //containerRegistry.RegisterForNavigation<Views.UserControl1>();
            //containerRegistry.RegisterForNavigation<Views.UserControl2>();
        }

        #endregion

        #region Helpers

        private void RegisterServices(IContainerRegistry containerRegistry)
        {
            containerRegistry.RegisterInstance<IDispatcherService>(new DispatcherService(Dispatcher.CurrentDispatcher));
        }

        private void RegisterViewModels(IContainerRegistry containerRegistry)
        {
            containerRegistry.Register<MainWindowViewModel>();
        }

        #endregion

        #endregion

    }

}

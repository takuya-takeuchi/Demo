using System.Windows;
using System.Windows.Threading;
using Demo.Services;
using Demo.Services.Interfaces;
using Demo.ViewModels;
using Demo.Views;
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
            this.RegisterServices(containerRegistry);
            this.RegisterViewModels(containerRegistry);
            this.RegisterViews(containerRegistry);
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

        private void RegisterViews(IContainerRegistry containerRegistry)
        {
            containerRegistry.RegisterForNavigation<Views.Modules.ModuleAView>();
            containerRegistry.RegisterForNavigation<Views.Modules.ModuleBView>();
        }

        #endregion

        #endregion

    }

}

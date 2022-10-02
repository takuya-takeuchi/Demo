using System.Windows;

using Prism.Ioc;
using Prism.Unity;

using Demo.ViewModels;
using Demo.Views;
using System;

namespace Demo
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : PrismApplication
    {

        #region Methods

        #region Overrids

        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);
        }

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
        }

        private void RegisterViewModels(IContainerRegistry containerRegistry)
        {
            containerRegistry.Register<MainWindowViewModel>();
        }

        private void RegisterViews(IContainerRegistry containerRegistry)
        {
            containerRegistry.RegisterForNavigation<Views.Modules.ModuleA>();
            containerRegistry.RegisterForNavigation<Views.Modules.ModuleB>();
        }

        #endregion

        #endregion

    }

}

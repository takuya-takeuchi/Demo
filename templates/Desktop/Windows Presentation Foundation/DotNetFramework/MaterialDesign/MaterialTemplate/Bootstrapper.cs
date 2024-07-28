using System.Windows;
using MaterialTemplate.Services;
using MaterialTemplate.Services.Interfaces;
using MaterialTemplate.Views;
using Microsoft.Practices.Unity;
using Prism.Unity;

namespace MaterialTemplate
{
    internal sealed class Bootstrapper : UnityBootstrapper
    {

        protected override DependencyObject CreateShell()
        {
            return this.Container.Resolve<MainWindow>();
        }

        protected override void InitializeShell()
        {
            Application.Current.MainWindow?.Show();
        }

        protected override void ConfigureViewModelLocator()
        {
            base.ConfigureViewModelLocator();

            // type / type
            //ViewModelLocationProvider.Register(typeof(MainWindow).ToString(), typeof(CustomViewModel));

            // type / factory
            //ViewModelLocationProvider.Register(typeof(MainWindow).ToString(), () => Container.Resolve<CustomViewModel>());

            // generic factory
            //ViewModelLocationProvider.Register<MainWindow>(() => Container.Resolve<CustomViewModel>());

            // generic type
            //ViewModelLocationProvider.Register<MainWindow, CustomViewModel>();

            this.Container.RegisterType<ILoggerService, NLogLoggerService>();
        }

    }
}

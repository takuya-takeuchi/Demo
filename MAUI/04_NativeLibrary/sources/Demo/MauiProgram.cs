using System;

using Microsoft.Maui;
using Microsoft.Maui.Controls.Hosting;
using Microsoft.Maui.Hosting;

using Prism;
using Prism.Ioc;
using Prism.Navigation;

using Demo.ViewModels;
using Demo.Views;

namespace Demo
{

    public static class MauiProgram
    {

        #region Properties

        public static Action<IContainerRegistry> PlatformRegisterTypes
        {
            get;
            set;
        }

        #endregion

        #region Methods

        public static MauiApp CreateMauiApp()
        {
            return MauiApp.CreateBuilder()
                .UseMauiApp<App>()
                       .UsePrism(prism =>
                       prism.ConfigureModuleCatalog(moduleCatalog =>
                       {
                           //moduleCatalog.AddModule<MauiAppModule>();
                           //moduleCatalog.AddModule<MauiTestRegionsModule>();
                       })
                       .RegisterTypes(containerRegistry =>
                       {
                           PlatformRegisterTypes?.Invoke(containerRegistry);

                           //containerRegistry.RegisterGlobalNavigationObserver();
                           containerRegistry.RegisterForNavigation<MainPage>();
                       })
                       .AddGlobalNavigationObserver(context => context.Subscribe(x =>
                       {
                           if (x.Type == NavigationRequestType.Navigate)
                               Console.WriteLine($"Navigation: {x.Uri}");
                           else
                               Console.WriteLine($"Navigation: {x.Type}");

                           var status = x.Cancelled ? "Cancelled" : x.Result.Success ? "Success" : "Failed";
                           Console.WriteLine($"Result: {status}");

                           if (status == "Failed" && !string.IsNullOrEmpty(x.Result?.Exception?.Message))
                               Console.Error.WriteLine(x.Result.Exception.Message);
                       }))
                       .CreateWindow(navigationService => navigationService.CreateBuilder()
                           .AddSegment<MainPageViewModel>()
                           .NavigateAsync(HandleNavigationError))
                       )
                       .ConfigureFonts(fonts =>
                       {
                           fonts.AddFont("OpenSans-Regular.ttf", "OpenSansRegular");
                           fonts.AddFont("OpenSans-Semibold.ttf", "OpenSansSemibold");
                       })
                       .Build();
        }

        #region Helpers

        private static void HandleNavigationError(Exception ex)
        {
            Console.WriteLine(ex);
            System.Diagnostics.Debugger.Break();
        }

        #endregion

        #endregion

    }

}

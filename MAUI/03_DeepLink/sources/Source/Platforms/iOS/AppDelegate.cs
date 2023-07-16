using Foundation;
using Microsoft.Maui;
using Microsoft.Maui.Hosting;

using Source.Platforms.iOS.Services;
using Source.Services.Interfaces;

namespace Source
{

    [Register("AppDelegate")]
    public class AppDelegate : MauiUIApplicationDelegate
    {

        #region Methods

        #region Overrides

        protected override MauiApp CreateMauiApp()
        {
            MauiProgram.PlatformRegisterTypes = RegisterServices;

            return MauiProgram.CreateMauiApp();
        }

        #endregion

        #region Helpers

        private static void RegisterServices(IContainerRegistry container)
        {
            container.Register<IDeepLinkService, DeepLinkService>();
        }

        #endregion

        #endregion

    }

}

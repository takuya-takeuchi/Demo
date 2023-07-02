using Foundation;
using Microsoft.Maui;
using Microsoft.Maui.Hosting;

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
        }

        #endregion

        #endregion

    }

}

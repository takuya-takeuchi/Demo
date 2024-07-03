using Foundation;
using Microsoft.Maui;
using Microsoft.Maui.Hosting;

using Demo.Platforms.iOS.Services;
using Demo.Services.Interfaces;

namespace Demo
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
            container.Register<IFolderPickerService, FolderPickerService>();
        }

        #endregion

        #endregion

    }

}

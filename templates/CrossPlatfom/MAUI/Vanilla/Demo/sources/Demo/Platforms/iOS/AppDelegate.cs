using Foundation;
using Microsoft.Maui;
using Microsoft.Maui.Hosting;

namespace Demo
{

    [Register("AppDelegate")]
    public class AppDelegate : MauiUIApplicationDelegate
    {

        #region Methods

        #region Overrides

        protected override MauiApp CreateMauiApp()
        {
            return MauiProgram.CreateMauiApp();
        }

        #endregion

        #endregion

    }

}

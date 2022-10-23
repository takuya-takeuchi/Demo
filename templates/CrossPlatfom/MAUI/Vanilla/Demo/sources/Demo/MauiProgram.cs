using Microsoft.Maui.Controls.Hosting;
using Microsoft.Maui.Hosting;

namespace Demo
{

    public static class MauiProgram
    {

        #region Methods

        public static MauiApp CreateMauiApp()
        {
            var builder = MauiApp.CreateBuilder();
            builder.UseMauiApp<App>()
                   .ConfigureFonts(fonts =>
                   {
                       fonts.AddFont("OpenSans-Regular.ttf", "OpenSansRegular");
                       fonts.AddFont("OpenSans-Semibold.ttf", "OpenSansSemibold");
                   });

            return builder.Build();
        }

        #endregion

    }

}

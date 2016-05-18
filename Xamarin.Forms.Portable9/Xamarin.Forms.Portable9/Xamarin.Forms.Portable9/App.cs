using Xamarin.Forms.Portable9.Services;
using Xamarin.Forms.Portable9.Views;

namespace Xamarin.Forms.Portable9
{
    public class App : Application
    {
        public App()
        {
            var service = DependencyService.Get<INativeControlService>();
            var view = service.Create() as View;
            view.HorizontalOptions = LayoutOptions.Center;
            view.VerticalOptions = LayoutOptions.Center;

            // The root page of your application
            MainPage = new MainPage
            {
                Content = view
            };
        }

        protected override void OnStart()
        {
            // Handle when your app starts
        }

        protected override void OnSleep()
        {
            // Handle when your app sleeps
        }

        protected override void OnResume()
        {
            // Handle when your app resumes
        }
    }
}

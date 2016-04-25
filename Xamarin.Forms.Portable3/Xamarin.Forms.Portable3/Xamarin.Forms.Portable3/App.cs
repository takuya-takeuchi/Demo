using Xamarin.Forms.Portable3.Views;

namespace Xamarin.Forms.Portable3
{
    public class App : Application
    {
        public App()
        {
            // The root page of your application
            this.MainPage = new MainPageView();
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

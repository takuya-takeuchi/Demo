using Xamarin.Forms.Portable2.Views;

namespace Xamarin.Forms.Portable2
{
    public class App : Application
    {
        public App()
        {
            // The root page of your application
            this.MainPage = new MainPage();
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

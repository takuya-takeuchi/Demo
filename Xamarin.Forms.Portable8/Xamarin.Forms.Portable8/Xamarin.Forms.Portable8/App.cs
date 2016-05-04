namespace Xamarin.Forms.Portable8
{
    public class App : Application
    {
        public App()
        {
            var bootstrapper = new Bootstrapper();
            bootstrapper.Run(this);
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

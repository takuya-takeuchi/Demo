using Android.App;
using Android.Content;
using AndroidX.AppCompat.App;

namespace Demo.Droid
{

    [Activity(Theme = "@style/MainTheme.Splash", MainLauncher = true, NoHistory = true)]
    public sealed class SplashActivity : AppCompatActivity
    {

        #region Methods

        #region Overrids

        // Launches the startup task
        protected override void OnResume()
        {
            base.OnResume();
            StartActivity(new Intent(Application.Context, typeof(MainActivity)));
        }

        #endregion

        #endregion

    }

}
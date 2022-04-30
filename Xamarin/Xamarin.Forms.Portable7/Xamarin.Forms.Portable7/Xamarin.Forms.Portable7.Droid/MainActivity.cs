using Android.App;
using Android.Content.PM;
using Android.OS;
using Xamarin.Forms.Portable7.Services;

namespace Xamarin.Forms.Portable7.Droid
{
    [Activity(Label = "Xamarin.Forms.Portable7", Icon = "@drawable/icon", MainLauncher = true, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation)]
    public class MainActivity : global::Xamarin.Forms.Platform.Android.FormsApplicationActivity
    {
        protected override void OnCreate(Bundle bundle)
        {
            base.OnCreate(bundle);

            global::Xamarin.Forms.Forms.Init(this, bundle);
            LoadApplication(new App());

            ServiceManager.Sender = "Android";
        }
    }
}


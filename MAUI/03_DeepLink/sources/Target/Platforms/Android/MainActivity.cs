using Android.App;
using Android.Content;
using Android.Content.PM;
using Microsoft.Maui;

namespace Target
{

    [Activity(Theme = "@style/Maui.SplashTheme", MainLauncher = true, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize | ConfigChanges.Density, Exported = true)]
    [IntentFilter(new[] { Intent.ActionMain }, Categories = new[] { Intent.CategoryLauncher })]
    [IntentFilter(new[] { Intent.ActionView }, Categories = new[] { Intent.CategoryDefault, Intent.CategoryBrowsable }, AutoVerify = true, DataHost = "taktak.jp", DataPathPrefix = "/buy", DataScheme = "https")]
    public class MainActivity : MauiAppCompatActivity
    {
    }

}

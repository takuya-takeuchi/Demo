using Android.App;
using Android.Content.PM;
using Android.OS;
using Prism.Ioc;

using Demo.Services.Interfaces;

namespace Demo.Droid
{

    [Activity(Theme = "@style/MainTheme",
              ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize)]
    public sealed class MainActivity : global::Xamarin.Forms.Platform.Android.FormsAppCompatActivity
    {

        #region Methods

        #region Overrids

        protected override void OnCreate(Bundle savedInstanceState)
        {
            TabLayoutResource = Resource.Layout.Tabbar;
            ToolbarResource = Resource.Layout.Toolbar;

            base.OnCreate(savedInstanceState);

            Xamarin.Essentials.Platform.Init(this, savedInstanceState);
            global::Xamarin.Forms.Forms.Init(this, savedInstanceState);

            // App invoke RegisterTypes
            var app = new App(new AndroidInitializer());

            this.InitializeLogger();

            LoadApplication(app);
        }

        public override void OnRequestPermissionsResult(int requestCode, string[] permissions, Android.Content.PM.Permission[] grantResults)
        {
            Xamarin.Essentials.Platform.OnRequestPermissionsResult(requestCode, permissions, grantResults);

            base.OnRequestPermissionsResult(requestCode, permissions, grantResults);
        }

        #endregion

        #region Helpers

        private void InitializeLogger()
        {
            var loggingService = App.Current.Container.Resolve<ILoggingService>();
            var assembly = this.GetType().Assembly;
            loggingService.Initialize(assembly);
        }

        #endregion

        #endregion

    }

}


using Android.Widget;
using Xamarin.Forms.Platform.Android;
using Xamarin.Forms.Portable9.Droid.Services;
using Xamarin.Forms.Portable9.Services;

[assembly: Xamarin.Forms.Dependency(typeof(NativeControlService))]
namespace Xamarin.Forms.Portable9.Droid.Services
{
    public sealed class NativeControlService : INativeControlService
    {
        public View Create()
        {
            var view = new TextView(Forms.Context)
            {
                Text = "Android",
                TextSize = 14,
            };

            return view.ToView();
        }

    }

}

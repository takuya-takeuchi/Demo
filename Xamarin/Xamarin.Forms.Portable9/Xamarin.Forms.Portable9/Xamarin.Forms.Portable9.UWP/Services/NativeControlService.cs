using Windows.UI.Xaml.Controls;
using Xamarin.Forms.Platform.UWP;
using Xamarin.Forms.Portable9.Services;
using Xamarin.Forms.Portable9.UWP.Services;

[assembly: Xamarin.Forms.Dependency(typeof(NativeControlService))]
namespace Xamarin.Forms.Portable9.UWP.Services
{
    public sealed class NativeControlService : INativeControlService
    {

        public View Create()
        {
            var label =  new TextBlock
            {
                Text = "UWP",
            };

            return label.ToView();
        }

    }

}

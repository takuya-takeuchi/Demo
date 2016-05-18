using UIKit;
using Xamarin.Forms.Platform.iOS;
using Xamarin.Forms.Portable9.iOS.Services;
using Xamarin.Forms.Portable9.Services;

[assembly: Xamarin.Forms.Dependency(typeof(NativeControlService))]
namespace Xamarin.Forms.Portable9.iOS.Services
{
    public sealed class NativeControlService : INativeControlService
    {

        public View Create()
        {
            var label =  new UILabel
            {
                MinimumFontSize = 14f,
                Lines = 0,
                LineBreakMode = UILineBreakMode.WordWrap,
                Text = "iOS",
            };

            return label.ToView();
        }

    }

}

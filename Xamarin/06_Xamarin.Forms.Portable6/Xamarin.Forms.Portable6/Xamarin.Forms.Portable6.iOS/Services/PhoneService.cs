using CoreTelephony;
using Foundation;
using UIKit;
using Xamarin.Forms.Portable6.iOS.Services;
using Xamarin.Forms.Portable6.Services;

[assembly: Xamarin.Forms.Dependency(typeof(PhoneService))]
namespace Xamarin.Forms.Portable6.iOS.Services
{
    public sealed class PhoneService : IPhoneService
    {
        public void ShowUI(string dialNumber, string displayName)
        {
            var ctTelephonyNetworkInfo = new CTTelephonyNetworkInfo();
            var carrier = ctTelephonyNetworkInfo.SubscriberCellularProvider;
            if (carrier?.IsoCountryCode == null)
            {
                return;
            }

            var nsUrl = new NSUrl(new System.Uri($"tel:{dialNumber}").AbsoluteUri);
            var result = UIApplication.SharedApplication.OpenUrl(nsUrl);
        }
    }

}

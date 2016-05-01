using Xamarin.Forms.Portable6.Services;
using Xamarin.Forms.Portable6.UWP.Services;

[assembly: Xamarin.Forms.Dependency(typeof(PhoneService))]
namespace Xamarin.Forms.Portable6.UWP.Services
{

    public sealed class PhoneService : IPhoneService
    {
        public void ShowUI(string dialNumber, string displayName)
        {
            Windows.ApplicationModel.Calls.PhoneCallManager.ShowPhoneCallUI(dialNumber, displayName ?? "Test Person");
        }
    }

}

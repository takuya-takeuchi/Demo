using System.Linq;
using Android.Content;
using Android.Net;
using Android.Telephony;
using Xamarin.Forms.Portable6.Droid.Services;
using Xamarin.Forms.Portable6.Services;

[assembly: Xamarin.Forms.Dependency(typeof(PhoneService))]
namespace Xamarin.Forms.Portable6.Droid.Services
{
    public sealed class PhoneService : IPhoneService
    {
        public void ShowUI(string dialNumber, string displayName)
        {
            var number = PhoneNumberUtils.FormatNumber(dialNumber.Trim(),"JP" );
            var intent = new Intent(Intent.ActionDial, Uri.Parse("tel:" + number));
            if (!IsIntentAvailable(intent))
            {
                return;
            }

            intent.SetFlags(ActivityFlags.NewTask);
            Android.App.Application.Context.StartActivity(intent);
        }

        public static bool IsIntentAvailable(Intent intent)
        {
            var context = Forms.Context;
            var packageManager = context.PackageManager;

            var list = packageManager.QueryIntentServices(intent, 0)
                .Union(packageManager.QueryIntentActivities(intent, 0));

            if (list.Any())
                return true;

            var manager = TelephonyManager.FromContext(context);
            return manager.PhoneType != PhoneType.None;
        }
    }

}

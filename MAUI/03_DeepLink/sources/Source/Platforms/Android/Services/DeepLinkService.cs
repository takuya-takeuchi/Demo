using System.Threading.Tasks;

using Microsoft.Maui.ApplicationModel;

using Android.Content;
using Android.Net;
using AndroidX.Activity;

using Source.Services.Interfaces;

namespace Source.Platforms.Android.Services
{

    internal sealed class DeepLinkService : IDeepLinkService
    {

        #region IDeepLinkService Members

        public Task Open()
        {
            if (Platform.CurrentActivity is not ComponentActivity activity)
                return Task.CompletedTask;

            var intent = new Intent(Intent.ActionView, Uri.Parse("https://taktak.jp/buy"));
            // if you do not want to open new app in new window, remove this line
            intent.AddFlags(ActivityFlags.NewTask | ActivityFlags.ClearTask);
            activity.StartActivityForResult(intent, 1);
            return Task.CompletedTask;
        }

        #endregion

    }

}

using System;
using System.Threading;
using System.Threading.Tasks;

using Android.App;
using Android.Content;
using Android.Graphics;
using AndroidX.Browser.CustomTabs;

using Xamarin.Essentials;

using IdentityModel.OidcClient.Browser;

[assembly: Xamarin.Forms.Dependency(typeof(Demo.Droid.Browser))]
namespace Demo.Droid
{

    internal sealed class Browser : IBrowser
    {

        #region Fields

        private readonly Activity _Context;

        private readonly CustomTabsActivityManager _Manager;

        #endregion

        #region Constructors

        public Browser() : this(Platform.CurrentActivity) { }

        public Browser(Activity context)
        {
            this._Context = context;
            this._Manager = new CustomTabsActivityManager(this._Context);
        }

        #endregion

        #region IBrowser Members
        
        public Task<BrowserResult> InvokeAsync(BrowserOptions options, CancellationToken cancellationToken = default)
        {
            var task = new TaskCompletionSource<BrowserResult>();
            var builder = new CustomTabsIntent.Builder(this._Manager.Session).SetToolbarColor(Color.Argb(255, 52, 152, 219))
                                                                             .SetShowTitle(true)
                                                                             .EnableUrlBarHiding();

            var customTabsIntent = builder.Build();

            // ensures the intent is not kept in the history stack, which makes
            // sure navigating away from it will close it
            customTabsIntent.Intent.AddFlags(ActivityFlags.NoHistory);

            Action<string> callback = null;
            callback = url =>
            {
                OidcCallbackActivity.Callbacks -= callback;

                task.SetResult(new BrowserResult()
                {
                    Response = url
                });
            };

            OidcCallbackActivity.Callbacks += callback;

            customTabsIntent.LaunchUrl(this._Context, Android.Net.Uri.Parse(options.StartUrl));

            return task.Task;
        }

        #endregion

    }

}
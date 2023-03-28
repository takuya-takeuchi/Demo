using System;

using Android.App;
using Android.Content;
using Android.OS;

namespace Demo.Droid
{

    [Activity(Label = "OidcCallbackActivity")]
    [IntentFilter(new[] { Intent.ActionView },
                  Categories = new[] { Intent.CategoryDefault, Intent.CategoryBrowsable },
                  DataScheme = "xamarinformsclients")]
    public sealed class OidcCallbackActivity : Activity
    {

        #region Events

        public static event Action<string> Callbacks;

        #endregion

        #region Constructors

        public OidcCallbackActivity()
        {
            //Log.Debug("OidcCallbackActivity", "constructing OidcCallbackActivity");
        }

        #endregion

        #region Methods

        #region Overrides

        protected override void OnCreate(Bundle savedInstanceState)
        {
            base.OnCreate(savedInstanceState);

            Callbacks?.Invoke(this.Intent.DataString);

            this.Finish();

            this.StartActivity(typeof(MainActivity));
        }

        #endregion

        #endregion
        
    }

}
using System;
using Android.App;
using Android.Runtime;

namespace Demo.Droid
{

    [Application(Theme = "@style/MainTheme")]
    public sealed class MainApplication : Application
    {

        #region Constructors

        public MainApplication(IntPtr javaReference, JniHandleOwnership transfer)
            : base(javaReference, transfer)
        {
        }

        #endregion

        #region Methods

        #region Overrids

        public override void OnCreate()
        {
            base.OnCreate();
            Xamarin.Essentials.Platform.Init(this);
        }

        #endregion

        #endregion

    }

}
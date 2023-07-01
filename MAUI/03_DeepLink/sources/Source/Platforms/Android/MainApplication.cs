using System;
using Android.App;
using Android.Runtime;
using Demo.Platforms.Android.Services;
using Microsoft.Maui;
using Microsoft.Maui.Hosting;

namespace Demo
{

    [Application]
    public class MainApplication : MauiApplication
    {

        #region Constructors

        public MainApplication(IntPtr handle, JniHandleOwnership ownership)
            : base(handle, ownership)
        {
        }

        #endregion

        #region Methods

        #region Overrides

        protected override MauiApp CreateMauiApp()
        {
            MauiProgram.PlatformRegisterTypes = RegisterServices;

            return MauiProgram.CreateMauiApp();
        }

        #endregion

        #region Helpers

        private static void RegisterServices(IContainerRegistry container)
        {
            container.Register<IFolderPickerService, FolderPickerService>();
        }

        #endregion

        #endregion

    }

}

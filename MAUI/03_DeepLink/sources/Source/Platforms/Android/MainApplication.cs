﻿using System;
using Android.App;
using Android.Runtime;
using Microsoft.Maui;
using Microsoft.Maui.Hosting;

using Source.Platforms.Android.Services;
using Source.Services.Interfaces;

namespace Source
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
            container.Register<IDeepLinkService, DeepLinkService>();
        }

        #endregion

        #endregion

    }

}
﻿using Microsoft.Maui;
using Microsoft.Maui.Hosting;

// To learn more about WinUI, the WinUI project structure,
// and more about our project templates, see: http://aka.ms/winui-project-info.

namespace Source.WinUI
{

    /// <summary>
    /// Provides application-specific behavior to supplement the default Application class.
    /// </summary>
    public partial class App : MauiWinUIApplication
    {

        #region Constructors

        /// <summary>
        /// Initializes the singleton application object.  This is the first line of authored code
        /// executed, and as such is the logical equivalent of main() or WinMain().
        /// </summary>
        public App()
        {
            this.InitializeComponent();
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
        }

        #endregion

        #endregion

    }

}

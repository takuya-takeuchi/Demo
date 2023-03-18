using System;
using System.Threading.Tasks;
using Foundation;
using UIKit;

using Prism;
using Prism.Ioc;

using Demo.iOS.Services;
using Demo.Services.Interfaces;

namespace Demo.iOS
{

    // The UIApplicationDelegate for the application. This class is responsible for launching the 
    // User Interface of the application, as well as listening (and optionally responding) to 
    // application events from iOS.
    [Register("AppDelegate")]
    public partial class AppDelegate : global::Xamarin.Forms.Platform.iOS.FormsApplicationDelegate, IPlatformInitializer
    {

        #region Methods

        #region Overrids

        //
        // This method is invoked when the application has loaded and is ready to run. In this 
        // method you should instantiate the window, load the UI into it and then make the window
        // visible.
        //
        // You have 17 seconds to return from this method, or iOS will terminate your application.
        //
        public override bool FinishedLaunching(UIApplication application, NSDictionary options)
        {
            global::Xamarin.Forms.Forms.Init();

            AppDomain.CurrentDomain.UnhandledException += CurrentDomainOnUnhandledException;
            TaskScheduler.UnobservedTaskException += TaskSchedulerOnUnobservedTaskException;

            // App invoke RegisterTypes
            var app = new App(new iOSInitializer());
            
            this.InitializeLogger();

            LoadApplication(app);

            return base.FinishedLaunching(application, options);
        }

        #endregion

        #region Event Handlers

        private static void TaskSchedulerOnUnobservedTaskException(object sender, UnobservedTaskExceptionEventArgs unobservedTaskExceptionEventArgs)
        {
            var newExc = new Exception("TaskSchedulerOnUnobservedTaskException", unobservedTaskExceptionEventArgs.Exception);
            LogUnhandledException(newExc);
        }

        private static void CurrentDomainOnUnhandledException(object sender, UnhandledExceptionEventArgs unhandledExceptionEventArgs)
        {
            var newExc = new Exception("CurrentDomainOnUnhandledException", unhandledExceptionEventArgs.ExceptionObject as Exception);
            LogUnhandledException(newExc);
        }

        #endregion

        #region Helpers

        private void InitializeLogger()
        {
            var loggingService = App.Current.Container.Resolve<ILoggingService>();
            var assembly = this.GetType().Assembly;
            loggingService.Initialize(assembly);
        }

        internal static void LogUnhandledException(Exception exception)
        {
            try
            {
                var loggingService = App.Current.Container.Resolve<ILoggingService>();
                loggingService.Fatal(exception, null, "error");
            }
            catch
            {
                // just suppress any error logging exceptions
            }
        }

        #endregion

        #endregion

        #region IPlatformInitializer Members

        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            containerRegistry.Register<IMediaService, MediaService>();
        }

        #endregion

    }

}

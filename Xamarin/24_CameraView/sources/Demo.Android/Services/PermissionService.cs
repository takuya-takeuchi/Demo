using System.Threading.Tasks;

using Android;
using Android.Content.PM;
using AndroidX.Core.App;
using AndroidX.Core.Content;

using Xamarin.Essentials;
using Xamarin.CommunityToolkit.Extensions;
using Xamarin.CommunityToolkit.UI.Views.Options;

using Demo.Services.Interfaces;

namespace Demo.Droid.Services
{

    public sealed class PermissionService : IPermissionService
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public PermissionService(ILoggingService loggingService)
        {
            this._LoggingService = loggingService;
        }

        #endregion

        #region Methods

        #region Helpers

        private static async Task ShowSnackBar(string message, int requestCode)
        {
            var snackBarActionOptions = new SnackBarActionOptions
            {
                Action = delegate()
                {
                    ActivityCompat.RequestPermissions(Platform.CurrentActivity, new[]
                    {
                        Manifest.Permission.Camera
                    }, requestCode);
                    return Task.CompletedTask;
                }
            };

            var snackBarOptions = new SnackBarOptions()
            {
                MessageOptions = new MessageOptions()
                {
                    Message = message
                },
                Actions = new[] { snackBarActionOptions }
            };

            await App.Current.MainPage.DisplaySnackBarAsync(snackBarOptions);
        }

        #endregion

        #endregion

        #region IPermissionService Members

        public bool CheckCameraPermission()
        {
            if (ContextCompat.CheckSelfPermission(Platform.CurrentActivity, Manifest.Permission.Camera) == (int)Permission.Granted)
            {
                this._LoggingService.Info("CAMERA permission has already been granted. Displaying camera preview.");
                return true;
            }

            return false;
        }

        public async Task RequestCameraPermission()
        {
            const int requestCamera = 0;
            if (ActivityCompat.ShouldShowRequestPermissionRationale(Platform.CurrentActivity, Manifest.Permission.Camera))
            {
                // Provide an additional rationale to the user if the permission was not granted
                // and the user would benefit from additional context for the use of the permission.
                // For example if the user has previously denied the permission.
                this._LoggingService.Info("Displaying camera permission rationale to provide additional context.");
                
                await ShowSnackBar("Camera permission is needed to show the camera preview.", requestCamera);
            }
            else
            {
                // Camera permission has not been granted yet. Request it directly.
                ActivityCompat.RequestPermissions(Platform.CurrentActivity, new[] { Manifest.Permission.Camera }, requestCamera);
            }
        }

        #endregion

    }

}
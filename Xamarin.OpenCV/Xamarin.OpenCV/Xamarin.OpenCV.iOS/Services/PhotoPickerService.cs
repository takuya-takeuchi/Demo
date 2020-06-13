using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using CoreGraphics;
using Foundation;
using UIKit;
using Xamarin.OpenCV.Services;

namespace Xamarin.OpenCV.iOS.Services
{

    public class PhotoPickerService : IPhotoPickerService
    {

        #region Fields

        private TaskCompletionSource<Stream> _TaskCompletionSource;

        private UIImagePickerController _ImagePicker;

        #endregion

        #region Constructors

        #endregion

        #region Properties

        #endregion

        #region Methods

        public Task<Stream> GetImageStreamAsync()
        {
            // Create and define UIImagePickerController
            this._ImagePicker = new UIImagePickerController
            {
                SourceType = UIImagePickerControllerSourceType.PhotoLibrary, MediaTypes = UIImagePickerController.AvailableMediaTypes(UIImagePickerControllerSourceType.PhotoLibrary)
            };
            
            new NSData()

            // Set event handlers
            this._ImagePicker.FinishedPickingMedia += OnImagePickerFinishedPickingMedia;
            this._ImagePicker.Canceled += OnImagePickerCancelled;

            // Present UIImagePickerController;
            var window = UIApplication.SharedApplication.KeyWindow;
            var viewController = window.RootViewController;
            viewController.PresentViewController(this._ImagePicker, true, null);

            // Return Task object
            this._TaskCompletionSource = new TaskCompletionSource<Stream>();

            return this._TaskCompletionSource.Task;
        }

        #region Overrids

        #endregion

        #region Event Handlers

        private void OnImagePickerFinishedPickingMedia(object sender, UIImagePickerMediaPickedEventArgs args)
        {
            var image = args.EditedImage ?? args.OriginalImage;
            if (image != null)
            {
                var extensions = new[]
                {
                    "PNG", "BMP"
                };

                // Convert UIImage to .NET Stream object
                NSData data;
                if (extensions.Any(s => args.ReferenceUrl.PathExtension.Equals(s, StringComparison.InvariantCultureIgnoreCase)))
                {
                    data = image.AsPNG();
                }
                else
                {
                    data = image.AsJPEG(1);
                }

                var stream = data.AsStream();
                this.UnRegisterEventHandlers();

                // Set the Stream as the completion of the Task
                this._TaskCompletionSource.SetResult(stream);
            }
            else
            {
                this.UnRegisterEventHandlers();
                this._TaskCompletionSource.SetResult(null);
            }

            this._ImagePicker.DismissModalViewController(true);
        }

        #endregion

        #region Helpers

        private void OnImagePickerCancelled(object sender, EventArgs args)
        {
            this.UnRegisterEventHandlers();
            this._TaskCompletionSource.SetResult(null);
            this._ImagePicker.DismissModalViewController(true);
        }

        private void UnRegisterEventHandlers()
        {
            this._ImagePicker.FinishedPickingMedia -= OnImagePickerFinishedPickingMedia;
            this._ImagePicker.Canceled -= OnImagePickerCancelled;
        }

        #endregion

        #endregion

    }

}
using System;
using System.Globalization;
using System.Threading.Tasks;

using AVFoundation;
using CoreGraphics;
using Foundation;
using UIKit;
using Xamarin.Forms;

using Prism.Ioc;

using Demo.Controls;
using Demo.Services.Interfaces;

namespace Demo.iOS.Renderers
{

    /// <summary>
    /// Camera ios.
    /// </summary>
    public sealed class CameraIOS : UIView
    {

        #region Private Properties

        /// <summary>
        /// The log.
        /// </summary>
        private readonly ILoggingService _Logger;

        /// <summary>
        /// The preview layer.
        /// </summary>
        private readonly AVCaptureVideoPreviewLayer _previewLayer;

        /// <summary>
        /// The capture session.
        /// </summary>
        private readonly AVCaptureSession _captureSession;

        /// <summary>
        /// The main view.
        /// </summary>
        private UIView _mainView;

        /// <summary>
        /// The input.
        /// </summary>
        private AVCaptureDeviceInput _input;

        /// <summary>
        /// The output.
        /// </summary>
        private AVCaptureStillImageOutput _output;

        /// <summary>
        /// The capture connection.
        /// </summary>
        private AVCaptureConnection _captureConnection;

        /// <summary>
        /// The device.
        /// </summary>
        private AVCaptureDevice _device;

        /// <summary>
        /// The camera busy.
        /// </summary>
        private bool _cameraBusy;

        /// <summary>
        /// The camera available.
        /// </summary>
        private bool _cameraAvailable;

        /// <summary>
        /// The width of the camera button container.
        /// </summary>
        private float _cameraButtonContainerWidth;

        /// <summary>
        /// The image scale.
        /// </summary>
        private float _imgScale = 1.25f;

        /// <summary>
        /// The system version.
        /// </summary>
        private double _systemVersion;

        /// <summary>
        /// The width.
        /// </summary>
        private nint _width;

        /// <summary>
        /// The height.
        /// </summary>
        private nint _height;

        #endregion

        #region Events

        /// <summary>
        /// Occurs when busy.
        /// </summary>
        public event EventHandler<bool> Busy;

        /// <summary>
        /// Occurs when available.
        /// </summary>
        public event EventHandler<bool> Available;

        /// <summary>
        /// Occurs when photo.
        /// </summary>
        public event EventHandler<byte[]> Photo;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="T:Demo.iOS.Renderers.CameraIOS"/> class.
        /// </summary>
        public CameraIOS()
        {
            this._Logger = ContainerLocator.Container.CurrentScope.Resolve<ILoggingService>();

            // retrieve system version 
            var versionParts = UIDevice.CurrentDevice.SystemVersion.Split('.');
            var versionString = versionParts[0] + "." + versionParts[1];
            this._systemVersion = Convert.ToDouble(versionString, CultureInfo.InvariantCulture);

            this._mainView = new UIView() { TranslatesAutoresizingMaskIntoConstraints = false };
            this.AutoresizingMask = UIViewAutoresizing.FlexibleMargins;

            this._captureSession = new AVCaptureSession();

            this._previewLayer = new AVCaptureVideoPreviewLayer(this._captureSession)
            {
                VideoGravity = AVLayerVideoGravity.Resize
            };

            this._mainView.Layer.AddSublayer(this._previewLayer);

            // retrieve camera device if available
            this._cameraAvailable = this.RetrieveCameraDevice();

            this.Add(this._mainView);

            // set layout constraints for main view
            this.AddConstraints(NSLayoutConstraint.FromVisualFormat("V:|[mainView]|", NSLayoutFormatOptions.DirectionLeftToRight, null, new NSDictionary("mainView", this._mainView)));
            this.AddConstraints(NSLayoutConstraint.FromVisualFormat("H:|[mainView]|", NSLayoutFormatOptions.AlignAllTop, null, new NSDictionary("mainView", this._mainView)));
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// Function will be called for IOS version less than 8
        /// </summary>
        /// <param name="orientation">Orientation.</param>
        private void AdjustPreviewLayer(Orientation orientation)
        {
            CGRect previewLayerFrame = this._previewLayer.Frame;

            switch (orientation)
            {
                case Orientation.Portrait:
                    previewLayerFrame.Height = UIScreen.MainScreen.Bounds.Height - this._cameraButtonContainerWidth;
                    previewLayerFrame.Width = UIScreen.MainScreen.Bounds.Width;
                    break;

                case Orientation.LandscapeLeft:
                case Orientation.LandscapeRight:
                    if (this._systemVersion >= 8)
                    {
                        previewLayerFrame.Width = UIScreen.MainScreen.Bounds.Width - this._cameraButtonContainerWidth;
                        previewLayerFrame.Height = UIScreen.MainScreen.Bounds.Height;
                    }
                    else
                    {
                        previewLayerFrame.Width = UIScreen.MainScreen.Bounds.Height - this._cameraButtonContainerWidth;
                        previewLayerFrame.Height = UIScreen.MainScreen.Bounds.Width;
                    }
                    break;
            }

            try
            {
                this._previewLayer.Frame = previewLayerFrame;
            }
            catch (Exception error)
            {
                this._Logger.Error(error, null, "Failed to adjust frame");
            }
        }

        /// <summary>
        /// Sets the start orientation.
        /// </summary>
        private void SetStartOrientation()
        {
            Orientation sOrientation = Orientation.None;

            switch (UIApplication.SharedApplication.StatusBarOrientation)
            {
                case UIInterfaceOrientation.Portrait:
                case UIInterfaceOrientation.PortraitUpsideDown:
                    sOrientation = Orientation.Portrait;
                    break;
                case UIInterfaceOrientation.LandscapeLeft:
                    sOrientation = Orientation.LandscapeLeft;
                    break;
                case UIInterfaceOrientation.LandscapeRight:
                    sOrientation = Orientation.LandscapeRight;
                    break;
            }

            this.HandleOrientationChange(sOrientation);
        }

        /// <summary>
        /// Sets the busy.
        /// </summary>
        /// <param name="busy">If set to <c>true</c> busy.</param>
        private void SetBusy(bool busy)
        {
            this._cameraBusy = busy;

            // set camera busy 
            this.Busy?.Invoke(this, this._cameraBusy);
        }
        
        /// <summary>
        /// Captures the image with metadata.
        /// </summary>
        /// <returns>The image with metadata.</returns>
        /// <param name="captureStillImageOutput">Capture still image output.</param>
        /// <param name="connection">Connection.</param>
        private async Task CaptureImageWithMetadata(AVCaptureStillImageOutput captureStillImageOutput, AVCaptureConnection connection)
        {
            var sampleBuffer = await captureStillImageOutput.CaptureStillImageTaskAsync(connection);
            var imageData = AVCaptureStillImageOutput.JpegStillToNSData(sampleBuffer);
            var image = UIImage.LoadFromData(imageData);

            this.RotateImage(ref image);

            try
            {
                byte[] imgData = image.AsJPEG().ToArray();

                if (this.Photo != null)
                {
                    this.Photo(this, imgData);
                }
            }
            catch (Exception error)
            {
                this._Logger.Error(error, null, "Failed to take photo");
            }
        }

        /// <summary>
        /// Rotates the image.
        /// </summary>
        /// <param name="image">Image.</param>
        private void RotateImage(ref UIImage image)
        {
            CGImage imgRef = image.CGImage;
            CGAffineTransform transform = CGAffineTransform.MakeIdentity();

            var imgHeight = imgRef.Height * this._imgScale;
            var imgWidth = imgRef.Width * this._imgScale;

            CGRect bounds = new CGRect(0, 0, imgWidth, imgHeight);
            CGSize imageSize = new CGSize(imgWidth, imgHeight);
            UIImageOrientation orient = image.Orientation;

            switch (orient)
            {
                case UIImageOrientation.Up:
                    transform = CGAffineTransform.MakeIdentity();
                    break;
                case UIImageOrientation.Down:
                    transform = CGAffineTransform.MakeTranslation(imageSize.Width, imageSize.Height);
                    transform = CGAffineTransform.Rotate(transform, (float)Math.PI);
                    break;
                case UIImageOrientation.Right:
                    bounds.Size = new CGSize(bounds.Size.Height, bounds.Size.Width);
                    transform = CGAffineTransform.MakeTranslation(imageSize.Height, 0);
                    transform = CGAffineTransform.Rotate(transform, (float)Math.PI / 2.0f);
                    break;
                default:
                    throw new Exception("Invalid image orientation");
            }

            UIGraphics.BeginImageContext(bounds.Size);
            CGContext context = UIGraphics.GetCurrentContext();

            if (orient == UIImageOrientation.Right)
            {
                context.ScaleCTM(-1, 1);
                context.TranslateCTM(-imgHeight, 0);
            }
            else
            {
                context.ScaleCTM(1, -1);
                context.TranslateCTM(0, -imgHeight);
            }

            context.ConcatCTM(transform);

            context.DrawImage(new CGRect(0, 0, imgWidth, imgHeight), imgRef);
            image = UIGraphics.GetImageFromCurrentImageContext();
            UIGraphics.EndImageContext();
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Draw the specified rect.
        /// </summary>
        /// <param name="rect">Rect.</param>
        public override void Draw(CGRect rect)
        {
            this._previewLayer.Frame = rect;

            base.Draw(rect);
        }

        /// <summary>
        /// Takes the photo.
        /// </summary>
        /// <returns>The photo.</returns>
        public async Task TakePhoto()
        {
            if (!this._cameraBusy)
            {
                this.SetBusy(true);

                try
                {
                    // set output orientation
                    this._output.Connections[0].VideoOrientation = this._previewLayer.Orientation;

                    var connection = this._output.Connections[0];

                    await this.CaptureImageWithMetadata(this._output, connection);

                    this.SetBusy(false);
                }
                catch (Exception error)
                {
                    this._Logger.Error(error, null, "Failed to take photo");
                }
            }
        }

        /// <summary>
        /// Switchs the flash.
        /// </summary>
        /// <param name="flashOn">If set to <c>true</c> flash on.</param>
        public void SwitchFlash(bool flashOn)
        {
            NSError err;

            if (this._cameraAvailable && this._device != null)
            {
                try
                {
                    this._device.LockForConfiguration(out err);
                    this._device.TorchMode = flashOn ? AVCaptureTorchMode.On : AVCaptureTorchMode.Off;
                    this._device.UnlockForConfiguration();
                }
                catch (Exception error)
                {
                    this._Logger.Error(error, null, "Failed to switch flash on/off");
                }
            }
        }

        /// <summary>
        /// Sets the bounds.
        /// </summary>
        /// <returns>The bounds.</returns>
        public void SetBounds(nint width, nint height)
        {
            this._height = height;
            this._width = width;
        }

        /// <summary>
        /// Changes the focus point.
        /// </summary>
        /// <param name="fPoint">F point.</param>
        public void ChangeFocusPoint(Point fPoint)
        {
            NSError err;

            if (this._cameraAvailable && this._device != null)
            {
                try
                {
                    this._device.LockForConfiguration(out err);

                    var focus_x = fPoint.X / this.Bounds.Width;
                    var focus_y = fPoint.Y / this.Bounds.Height;

                    // set focus point
                    if (this._device.FocusPointOfInterestSupported)
                        this._device.FocusPointOfInterest = new CGPoint(focus_x, focus_y);
                    if (this._device.ExposurePointOfInterestSupported)
                        this._device.ExposurePointOfInterest = new CGPoint(focus_x, focus_y);

                    this._device.UnlockForConfiguration();
                }
                catch (Exception error)
                {
                    this._Logger.Error(error, null, "Failed to adjust focus");
                }
            }
        }

        /// <summary>
        /// Retrieves the camera device.
        /// </summary>
        /// <returns><c>true</c>, if camera device was retrieved, <c>false</c> otherwise.</returns>
        public bool RetrieveCameraDevice()
        {
            this._device = AVCaptureDevice.DefaultDeviceWithMediaType(AVMediaType.Video);

            if (this._device == null)
            {
                this._Logger.Error("No device detected");
                return false;
            }

            return true;
        }

        /// <summary>
        /// Initializes the camera.
        /// </summary>
        /// <returns>The camera.</returns>
        public void InitializeCamera()
        {
            try
            {
                NSError error;
                NSError err;

                this._device.LockForConfiguration(out err);
                this._device.FocusMode = AVCaptureFocusMode.ContinuousAutoFocus;
                this._device.UnlockForConfiguration();

                this._input = new AVCaptureDeviceInput(this._device, out error);
                this._captureSession.AddInput(this._input);

                this._output = new AVCaptureStillImageOutput();

                var dict = new NSMutableDictionary();
                dict[AVVideo.CodecKey] = new NSNumber((int)AVVideoCodec.JPEG);
                this._captureSession.AddOutput(this._output);

                this.InvokeOnMainThread(delegate
                {
                    // capture connection used for rotating camera
                    this._captureConnection = this._previewLayer.Connection;
                    this.SetStartOrientation();
                    // set orientation before loading camera
                    this._captureSession.StartRunning();
                });
            }
            catch (Exception error)
            {
                this._Logger.Error(error, null, "Failed to initialise camera");
            }

            this.Available?.Invoke(this, this._cameraAvailable);

            this._Logger.Info("Succeeded to initialize camera");
        }

        /// <summary>
        /// Sets the widths.
        /// </summary>
        /// <param name="cameraButtonContainerWidth">Camera button container width.</param>
        public void SetWidths(float cameraButtonContainerWidth)
        {
            this._cameraButtonContainerWidth = cameraButtonContainerWidth;
        }

        /// <summary>
        /// Handles the orientation change.
        /// </summary>
        /// <param name="orientation">Orientation.</param>
        public void HandleOrientationChange(Orientation orientation)
        {
            if (this._captureConnection != null)
            {
                switch (orientation)
                {
                    case Orientation.Portrait:
                        this._captureConnection.VideoOrientation = AVCaptureVideoOrientation.Portrait;
                        break;
                    case Orientation.LandscapeLeft:
                        this._captureConnection.VideoOrientation = AVCaptureVideoOrientation.LandscapeLeft;
                        break;
                    case Orientation.LandscapeRight:
                        this._captureConnection.VideoOrientation = AVCaptureVideoOrientation.LandscapeRight;
                        break;
                }
            }

            this.AdjustPreviewLayer(orientation);
        }

        /// <summary>
        /// Stops the and dispose.
        /// </summary>
        public void StopAndDispose()
        {
            if (this._device != null)
            {
                // if flash is on turn off
                if (this._device.TorchMode == AVCaptureTorchMode.On)
                {
                    this.SwitchFlash(false);
                }
            }

            this._captureSession.StopRunning();
            // dispose output elements
            this._input.Dispose();
            this._output.Dispose();
        }

        #endregion
    }

}
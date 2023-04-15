using System;
using System.Collections.Generic;

using Android.Content;
using Android.Graphics;
using Android.Hardware.Camera2;
using Android.Hardware.Camera2.Params;
using Android.Media;
using Android.OS;
using Android.Runtime;
using Android.Util;
using Android.Views;
using Android.Widget;

using Java.Lang;
using Prism.Ioc;

using Demo.Services.Interfaces;

namespace Demo.Droid.Renderers
{

    /// <summary>
    /// Camera droid.
    /// </summary>
    public class CameraDroid : FrameLayout, TextureView.ISurfaceTextureListener
    {
        #region Static Properties

        /// <summary>
        /// The orientations.
        /// </summary>
        private static readonly SparseIntArray ORIENTATIONS = new SparseIntArray();

        #endregion

        #region Public Events

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

        #region Private Properties

        /// <summary>
        /// The tag.
        /// </summary>
        private readonly string _tag;

        /// <summary>
        /// The log.
        /// </summary>
        //private readonly ILogger _log;

        /// <summary>
        /// The m state listener.
        /// </summary>
        private CameraStateListener _stateListener;

        /// <summary>
        /// The CameraRequest.Builder for camera preview.
        /// </summary>
        private CaptureRequest.Builder _previewBuilder;

        /// <summary>
        /// The CameraCaptureSession for camera preview.
        /// </summary>
        private CameraCaptureSession _previewSession;

        /// <summary>
        /// The view surface.
        /// </summary>
        private SurfaceTexture _viewSurface;

        /// <summary>
        /// The camera texture.
        /// </summary>
        private AutoFitTextureView _cameraTexture;

        /// <summary>
        /// The media sound.
        /// </summary>
        private MediaActionSound _mediaSound;

        /// <summary>
        /// The size of the camera preview.
        /// </summary>
        private Android.Util.Size _previewSize;

        /// <summary>
        /// The context.
        /// </summary>
        private Context _context;

        /// <summary>
        /// The camera manager.
        /// </summary>
        private CameraManager _manager;

        /// <summary>
        /// The media sound loaded.
        /// </summary>
        private bool _mediaSoundLoaded;

        /// <summary>
        /// The opening camera.
        /// </summary>
        private bool _openingCamera;

        #endregion

        #region Public Properties

        /// <summary>
        /// The opening camera.
        /// </summary>
        public bool OpeningCamera
        {
            get
            {
                return this._openingCamera;
            }
            set
            {
                if (this._openingCamera != value)
                {
                    this._openingCamera = value;
                    this.Busy?.Invoke(this, value);
                }
            }
        }

        /// <summary>
        /// The reference to the opened CameraDevice.
        /// </summary>
        public CameraDevice _cameraDevice;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="Android_Shared.Controls.CameraDroid"/> class.
        /// </summary>
        /// <param name="context">Context.</param>
        public CameraDroid(Context context) : base(context)
        {
            this._context = context;
            this._mediaSoundLoaded = this.LoadShutterSound();

            //this._log = ContainerLocator.Container.CurrentScope.Resolve<ILoggingService>();
            var logger = ContainerLocator.Container.CurrentScope.Resolve<ILoggingService>();
            this._tag = $"{this.GetType()} ";

            var inflater = LayoutInflater.FromContext(context);

            if (inflater != null)
            {
                var view = inflater.Inflate(Resource.Layout.CameraLayout, this);

                this._cameraTexture = view.FindViewById<AutoFitTextureView>(Resource.Id.CameraTexture);
                this._cameraTexture.SurfaceTextureListener = this;

                this._stateListener = new CameraStateListener() { Camera = this };

                ORIENTATIONS.Append((int)SurfaceOrientation.Rotation0, 90);
                ORIENTATIONS.Append((int)SurfaceOrientation.Rotation90, 0);
                ORIENTATIONS.Append((int)SurfaceOrientation.Rotation180, 270);
                ORIENTATIONS.Append((int)SurfaceOrientation.Rotation270, 180);
            }
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// Updates the camera preview, StartPreview() needs to be called in advance
        /// </summary>
        private void UpdatePreview()
        {
            if (this._cameraDevice != null && this._previewSession != null)
            {
                try
                {
                    // The camera preview can be run in a background thread. This is a Handler for the camere preview
                    this._previewBuilder.Set(CaptureRequest.ControlMode, new Java.Lang.Integer((int)ControlMode.Auto));

                    // We create a Handler since we want to handle the resulting JPEG in a background thread
                    HandlerThread thread = new HandlerThread("CameraPicture");
                    thread.Start();
                    Handler backgroundHandler = new Handler(thread.Looper);

                    // Finally, we start displaying the camera preview
                    //if (_previewSession.IsReprocessable)
                    this._previewSession.SetRepeatingRequest(this._previewBuilder.Build(), null, backgroundHandler);
                }
                catch (CameraAccessException error)
                {
                    //this._log.WriteLineTime(this._tag + "\n" +
                    //                        "UpdatePreview() Camera access exception.  \n " +
                    //                        "ErrorMessage: \n" +
                    //                        error.Message + "\n" +
                    //                        "Stacktrace: \n " +
                    //                        error.StackTrace);
                }
                catch (IllegalStateException error)
                {
                    //this._log.WriteLineTime(this._tag + "\n" +
                    //                        "UpdatePreview() Illegal exception.  \n " +
                    //                        "ErrorMessage: \n" +
                    //                        error.Message + "\n" +
                    //                        "Stacktrace: \n " +
                    //                        error.StackTrace);
                }
            }
        }

        /// <summary>
        /// Loads the shutter sound.
        /// </summary>
        /// <returns><c>true</c>, if shutter sound was loaded, <c>false</c> otherwise.</returns>
        private bool LoadShutterSound()
        {
            try
            {
                this._mediaSound = new MediaActionSound();
                this._mediaSound.LoadAsync(MediaActionSoundType.ShutterClick);

                return true;
            }
            catch (Java.Lang.Exception error)
            {
                //this._log.WriteLineTime(this._tag + "\n" +
                //                        "LoadShutterSound() Error loading shutter sound  \n " +
                //                        "ErrorMessage: \n" +
                //                        error.Message + "\n" +
                //                        "Stacktrace: \n " +
                //                        error.StackTrace);
            }

            return false;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Opens the camera.
        /// </summary>
        public void OpenCamera()
        {
            if (this._context == null || this.OpeningCamera)
            {
                return;
            }

            this.OpeningCamera = true;

            this._manager = (CameraManager)this._context.GetSystemService(Context.CameraService);

            try
            {
                string cameraId = this._manager.GetCameraIdList()[0];

                // To get a list of available sizes of camera preview, we retrieve an instance of
                // StreamConfigurationMap from CameraCharacteristics
                CameraCharacteristics characteristics = this._manager.GetCameraCharacteristics(cameraId);
                StreamConfigurationMap map = (StreamConfigurationMap)characteristics.Get(CameraCharacteristics.ScalerStreamConfigurationMap);
                this._previewSize = map.GetOutputSizes(Java.Lang.Class.FromType(typeof(SurfaceTexture)))[0];
                Android.Content.Res.Orientation orientation = this.Resources.Configuration.Orientation;
                if (orientation == Android.Content.Res.Orientation.Landscape)
                {
                    this._cameraTexture.SetAspectRatio(this._previewSize.Width, this._previewSize.Height);
                }
                else
                {
                    this._cameraTexture.SetAspectRatio(this._previewSize.Height, this._previewSize.Width);
                }

                HandlerThread thread = new HandlerThread("CameraPreview");
                thread.Start();
                Handler backgroundHandler = new Handler(thread.Looper);

                // We are opening the camera with a listener. When it is ready, OnOpened of mStateListener is called.
                this._manager.OpenCamera(cameraId, this._stateListener, null);
            }
            catch (Java.Lang.Exception error)
            {
                //this._log.WriteLineTime(this._tag + "\n" +
                //                        "OpenCamera() Failed to open camera  \n " +
                //                        "ErrorMessage: \n" +
                //                        error.Message + "\n" +
                //                        "Stacktrace: \n " +
                //                        error.StackTrace);

                this.Available?.Invoke(this, false);
            }
            catch (System.Exception error)
            {
                //this._log.WriteLineTime(this._tag + "\n" +
                //                        "OpenCamera() Failed to open camera  \n " +
                //                        "ErrorMessage: \n" +
                //                        error.Message + "\n" +
                //                        "Stacktrace: \n " +
                //                        error.StackTrace);

                this.Available?.Invoke(this, false);
            }
        }

        /// <summary>
        /// Takes the photo.
        /// </summary>
        public void TakePhoto()
        {
            if (this._context != null && this._cameraDevice != null)
            {
                try
                {
                    this.Busy?.Invoke(this, true);

                    if (this._mediaSoundLoaded)
                    {
                        this._mediaSound.Play(MediaActionSoundType.ShutterClick);
                    }

                    // Pick the best JPEG size that can be captures with this CameraDevice
                    var characteristics = this._manager.GetCameraCharacteristics(this._cameraDevice.Id);
                    Android.Util.Size[] jpegSizes = null;
                    if (characteristics != null)
                    {
                        jpegSizes = ((StreamConfigurationMap)characteristics.Get(CameraCharacteristics.ScalerStreamConfigurationMap)).GetOutputSizes((int)ImageFormatType.Jpeg);
                    }
                    int width = 640;
                    int height = 480;

                    if (jpegSizes != null && jpegSizes.Length > 0)
                    {
                        width = jpegSizes[0].Width;
                        height = jpegSizes[0].Height;
                    }

                    // We use an ImageReader to get a JPEG from CameraDevice
                    // Here, we create a new ImageReader and prepare its Surface as an output from the camera
                    var reader = ImageReader.NewInstance(width, height, ImageFormatType.Jpeg, 1);
                    var outputSurfaces = new List<Surface>(2);
                    outputSurfaces.Add(reader.Surface);
                    outputSurfaces.Add(new Surface(this._viewSurface));

                    CaptureRequest.Builder captureBuilder = this._cameraDevice.CreateCaptureRequest(CameraTemplate.StillCapture);
                    captureBuilder.AddTarget(reader.Surface);
                    captureBuilder.Set(CaptureRequest.ControlMode, new Integer((int)ControlMode.Auto));

                    // Orientation
                    var windowManager = this._context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();
                    SurfaceOrientation rotation = windowManager.DefaultDisplay.Rotation;

                    captureBuilder.Set(CaptureRequest.JpegOrientation, new Integer(ORIENTATIONS.Get((int)rotation)));

                    // This listener is called when an image is ready in ImageReader 
                    ImageAvailableListener readerListener = new ImageAvailableListener();

                    readerListener.Photo += (sender, e) =>
                    {
                        this.Photo?.Invoke(this, e);
                    };

                    // We create a Handler since we want to handle the resulting JPEG in a background thread
                    HandlerThread thread = new HandlerThread("CameraPicture");
                    thread.Start();
                    Handler backgroundHandler = new Handler(thread.Looper);
                    reader.SetOnImageAvailableListener(readerListener, backgroundHandler);

                    var captureListener = new CameraCaptureListener();

                    captureListener.PhotoComplete += (sender, e) =>
                    {
                        this.Busy?.Invoke(this, false);
                        this.StartPreview();
                    };

                    this._cameraDevice.CreateCaptureSession(outputSurfaces, new CameraCaptureStateListener()
                    {
                        OnConfiguredAction = (CameraCaptureSession session) =>
                        {
                            try
                            {
                                this._previewSession = session;
                                session.Capture(captureBuilder.Build(), captureListener, backgroundHandler);
                            }
                            catch (CameraAccessException ex)
                            {
                                Log.WriteLine(LogPriority.Info, "Capture Session error: ", ex.ToString());
                            }
                        }
                    }, backgroundHandler);
                }
                catch (CameraAccessException error)
                {
                    //this._log.WriteLineTime(this._tag + "\n" +
                    //                        "TakePhoto() Failed to take photo  \n " +
                    //                        "ErrorMessage: \n" +
                    //                        error.Message + "\n" +
                    //                        "Stacktrace: \n " +
                    //                        error.StackTrace);
                }
                catch (Java.Lang.Exception error)
                {
                    //this._log.WriteLineTime(this._tag + "\n" +
                    //                        "TakePhoto() Failed to take photo  \n " +
                    //                        "ErrorMessage: \n" +
                    //                        error.Message + "\n" +
                    //                        "Stacktrace: \n " +
                    //                        error.StackTrace);
                }
            }
        }

        /// <summary>
        /// Changes the focus point.
        /// </summary>
        /// <param name="e">E.</param>
        public void ChangeFocusPoint(Xamarin.Forms.Point e)
        {
            string cameraId = this._manager.GetCameraIdList()[0];

            // To get a list of available sizes of camera preview, we retrieve an instance of
            // StreamConfigurationMap from CameraCharacteristics
            CameraCharacteristics characteristics = this._manager.GetCameraCharacteristics(cameraId);

            var rect = characteristics.Get(CameraCharacteristics.SensorInfoActiveArraySize) as Rect;

            int areaSize = 200;
            int right = rect.Right;
            int bottom = rect.Bottom;
            int viewWidth = this._cameraTexture.Width;
            int viewHeight = this._cameraTexture.Height;
            int ll, rr;

            Rect newRect;
            int centerX = (int)e.X;
            int centerY = (int)e.Y;

            ll = ((centerX * right) - areaSize) / viewWidth;
            rr = ((centerY * bottom) - areaSize) / viewHeight;

            int focusLeft = this.Clamp(ll, 0, right);
            int focusBottom = this.Clamp(rr, 0, bottom);

            newRect = new Rect(focusLeft, focusBottom, focusLeft + areaSize, focusBottom + areaSize);
            MeteringRectangle meteringRectangle = new MeteringRectangle(newRect, 500);
            MeteringRectangle[] meteringRectangleArr = { meteringRectangle };
            this._previewBuilder.Set(CaptureRequest.ControlAfTrigger, (int)ControlAFTrigger.Cancel);
            this._previewBuilder.Set(CaptureRequest.ControlAeRegions, meteringRectangleArr);
            this._previewBuilder.Set(CaptureRequest.ControlAfTrigger, (int)ControlAFTrigger.Start);

            this.UpdatePreview();
        }

        /// <summary>
        /// Clamp the specified value, min and max.
        /// </summary>
        /// <param name="value">Value.</param>
        /// <param name="min">Minimum.</param>
        /// <param name="max">Max.</param>
        private int Clamp(int value, int min, int max)
        {
            return (value < min) ? min : (value > max) ? max : value;
        }

        /// <summary>
        /// Starts the camera previe
        /// </summary>
        public void StartPreview()
        {
            if (this._cameraDevice != null && this._cameraTexture.IsAvailable && this._previewSize != null)
            {
                try
                {
                    var texture = this._cameraTexture.SurfaceTexture;
                    System.Diagnostics.Debug.Assert(texture != null);

                    // We configure the size of the default buffer to be the size of the camera preview we want
                    texture.SetDefaultBufferSize(this._previewSize.Width, this._previewSize.Height);

                    // This is the output Surface we need to start the preview
                    Surface surface = new Surface(texture);

                    // We set up a CaptureRequest.Builder with the output Surface
                    this._previewBuilder = this._cameraDevice.CreateCaptureRequest(CameraTemplate.Preview);
                    this._previewBuilder.AddTarget(surface);

                    // Here, we create a CameraCaptureSession for camera preview.
                    this._cameraDevice.CreateCaptureSession(new List<Surface>() { surface },
                        new CameraCaptureStateListener()
                        {
                            OnConfigureFailedAction = (CameraCaptureSession session) =>
                            {
                            },
                            OnConfiguredAction = (CameraCaptureSession session) =>
                            {
                                this._previewSession = session;
                                this.UpdatePreview();
                            }
                        },
                        null);
                }
                catch (Java.Lang.Exception error)
                {
                    //this._log.WriteLineTime(this._tag + "\n" +
                    //                        "TakePhoto() Failed to start preview \n " +
                    //                        "ErrorMessage: \n" +
                    //                        error.Message + "\n" +
                    //                        "Stacktrace: \n " +
                    //                        error.StackTrace);
                }
            }
        }

        /// <summary>
        /// Switchs the flash.
        /// </summary>
        /// <param name="flashOn">If set to <c>true</c> flash on.</param>
        public void SwitchFlash(bool flashOn)
        {
            try
            {
                this._previewBuilder.Set(CaptureRequest.FlashMode, new Integer(flashOn ? (int)FlashMode.Torch : (int)FlashMode.Off));
                this.UpdatePreview();
            }
            catch (System.Exception error)
            {
                //this._log.WriteLineTime(this._tag + "\n" +
                //                        "TakePhoto() Failed to switch flash on/off \n " +
                //                        "ErrorMessage: \n" +
                //                        error.Message + "\n" +
                //                        "Stacktrace: \n " +
                //                        error.StackTrace);

            }
        }

        /// <summary>
        /// Notifies the available.
        /// </summary>
        /// <param name="isAvailable">If set to <c>true</c> is available.</param>
        public void NotifyAvailable(bool isAvailable)
        {
            this.Available?.Invoke(this, isAvailable);
        }

        /// <summary>
        /// Ons the layout.
        /// </summary>
        /// <param name="l">L.</param>
        /// <param name="t">T.</param>
        /// <param name="r">The red component.</param>
        /// <param name="b">The blue component.</param>
        public void OnLayout(int l, int t, int r, int b)
        {
            var msw = MeasureSpec.MakeMeasureSpec(r - l, MeasureSpecMode.Exactly);
            var msh = MeasureSpec.MakeMeasureSpec(b - t, MeasureSpecMode.Exactly);

            this._cameraTexture.Measure(msw, msh);
            this._cameraTexture.Layout(0, 0, r - l, b - t);
        }

        /// <summary>
        /// Configures the transform.
        /// </summary>
        /// <param name="viewWidth">View width.</param>
        /// <param name="viewHeight">View height.</param>
        public void ConfigureTransform(int viewWidth, int viewHeight)
        {
            if (this._viewSurface != null && this._previewSize != null && this._context != null)
            {
                var windowManager = this._context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();

                var rotation = windowManager.DefaultDisplay.Rotation;
                var matrix = new Matrix();
                var viewRect = new RectF(0, 0, viewWidth, viewHeight);
                var bufferRect = new RectF(0, 0, this._previewSize.Width, this._previewSize.Height);

                var centerX = viewRect.CenterX();
                var centerY = viewRect.CenterY();

                if (rotation == SurfaceOrientation.Rotation90 || rotation == SurfaceOrientation.Rotation270)
                {
                    bufferRect.Offset(centerX - bufferRect.CenterX(), centerY - bufferRect.CenterY());
                    matrix.SetRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.Fill);

                    var scale = System.Math.Max((float)viewHeight / this._previewSize.Height, (float)viewWidth / this._previewSize.Width);
                    matrix.PostScale(scale, scale, centerX, centerY);
                    matrix.PostRotate(90 * ((int)rotation - 2), centerX, centerY);
                }

                this._cameraTexture.SetTransform(matrix);
            }
        }

        /// <summary>
        /// Raises the surface texture available event.
        /// </summary>
        /// <param name="surface">Surface.</param>
        /// <param name="w">The width.</param>
        /// <param name="h">The height.</param>
        public void OnSurfaceTextureAvailable(SurfaceTexture surface, int w, int h)
        {
            this._viewSurface = surface;

            this.ConfigureTransform(w, h);
            this.StartPreview();
        }

        /// <summary>
        /// Raises the surface texture destroyed event.
        /// </summary>
        /// <param name="surface">Surface.</param>
        public bool OnSurfaceTextureDestroyed(SurfaceTexture surface)
        {
            this._previewSession.StopRepeating();

            return true;
        }

        /// <summary>
        /// Raises the surface texture size changed event.
        /// </summary>
        /// <param name="surface">Surface.</param>
        /// <param name="width">Width.</param>
        /// <param name="height">Height.</param>
        public void OnSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height)
        {
            this.ConfigureTransform(width, height);
            this.StartPreview();
        }

        /// <summary>
        /// Raises the surface texture updated event.
        /// </summary>
        /// <param name="surface">Surface.</param>
        public void OnSurfaceTextureUpdated(SurfaceTexture surface)
        {
        }

        #endregion
    }
}
using System;

using Xamarin.Forms;

namespace Demo.Controls
{

    /// <summary>
    ///  This is the control used to embed into Xamarin Forms that will 
    /// yield a custom rendered for taking photos
    /// </summary>
    public sealed class CameraView : ContentView
    {

        #region Events

        /// <summary>
        /// Occurs when orientation change.
        /// </summary>
        public event EventHandler<Orientation> OrientationChange;

        /// <summary>
        /// Occurs when widths.
        /// </summary>
        public event EventHandler<float> Widths;

        /// <summary>
        /// Occurs when availability change.
        /// </summary>
        public event EventHandler<bool> AvailabilityChange;

        /// <summary>
        /// Occurs when focus.
        /// </summary>
        public event EventHandler<Point> Focus;

        /// <summary>
        /// Occurs when open camera.
        /// </summary>
        public event EventHandler<bool> OpenCamera;

        /// <summary>
        /// Occurs when busy.
        /// </summary>
        public event EventHandler<bool> Busy;

        /// <summary>
        /// Occurs when flash.
        /// </summary>
        public event EventHandler<bool> Flash;

        /// <summary>
        /// Occurs when torch.
        /// </summary>
        public event EventHandler<bool> Torch;

        /// <summary>
        /// Occurs when loading.
        /// </summary>
        public event EventHandler<bool> Loading;

        /// <summary>
        /// Occurs when photo.
        /// </summary>
        public event EventHandler<byte[]> Photo;

        /// <summary>
        /// Occurs when shutter.
        /// </summary>
        public event EventHandler Shutter;

        #endregion

        #region Dependencies Properties

        public static readonly BindableProperty CameraControlAreaSizeProperty = BindableProperty.Create(nameof(CameraControlAreaSize),
                                                                                                        typeof(int),
                                                                                                        typeof(View),
                                                                                                        0,
                                                                                                        propertyChanged: CameraControlAreaSizePropertyChanged);

        private static void CameraControlAreaSizePropertyChanged(BindableObject bindableObject, object oldValue, object newValue)
        {
            if (!(bindableObject is CameraView cameraView) || !(newValue is int value))
                return;
            
            if (Device.RuntimePlatform == Device.iOS)
                cameraView.Widths?.Invoke(cameraView, value);
        }

        public int CameraControlAreaSize
        {
            get => (int)GetValue(CameraControlAreaSizeProperty);
            set => this.SetValue(CameraControlAreaSizeProperty, value);
        }

        public static readonly BindableProperty CameraOpenedProperty = BindableProperty.Create(nameof(CameraOpened),
                                                                                               typeof(bool),
                                                                                               typeof(View),
                                                                                               false,
                                                                                               propertyChanged: CameraOpenedChanged);

        private static void CameraOpenedChanged(BindableObject bindableObject, object oldValue, object newValue)
        {
            if (!(bindableObject is CameraView cameraView) || !(newValue is bool open))
                return;
            
            cameraView.OpenCamera?.Invoke(cameraView, open);

            // camera must store these widths for IOS on orientation changes for camera preview layer resizing
            if (Device.RuntimePlatform == Device.iOS)
                cameraView.Widths?.Invoke(cameraView, cameraView.CameraControlAreaSize);
        }

        public bool CameraOpened
        {
            get => (bool)GetValue(CameraOpenedProperty);
            set => this.SetValue(CameraOpenedProperty, value);
        }

        public static readonly BindableProperty CameraOrientationProperty = BindableProperty.Create(nameof(CameraOrientation),
                                                                                                    typeof(Orientation),
                                                                                                    typeof(View),
                                                                                                    Orientation.None,
                                                                                                    propertyChanged: CameraOrientationChanged);

        private static void CameraOrientationChanged(BindableObject bindableObject, object oldValue, object newValue)
        {
            if (!(bindableObject is CameraView cameraView) || !(newValue is Orientation orientation))
                return;
            
            cameraView.OrientationChange?.Invoke(cameraView, orientation);
        }

        public Orientation CameraOrientation
        {
            get => (Orientation)GetValue(CameraOrientationProperty);
            set => this.SetValue(CameraOrientationProperty, value);
        }

        public static readonly BindableProperty CameraAvailableProperty = BindableProperty.Create(nameof(CameraAvailable),
                                                                                                  typeof(bool),
                                                                                                  typeof(View),
                                                                                                  false,
                                                                                                  propertyChanged: CameraAvailableChanged);

        private static void CameraAvailableChanged(BindableObject bindableObject, object oldValue, object newValue)
        {
            if (!(bindableObject is CameraView cameraView) || !(newValue is bool available))
                return;
            
            cameraView.AvailabilityChange?.Invoke(cameraView, available);
        }

        public bool CameraAvailable
        {
            get => (bool)GetValue(CameraAvailableProperty);
            private set => this.SetValue(CameraAvailableProperty, value);
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Notifies the shutter.
        /// </summary>
        public void NotifyShutter()
        {
            this.Shutter?.Invoke(this, EventArgs.Empty);
        }
        
        /// <summary>
        /// Notifies the focus.
        /// </summary>
        /// <param name="touchPoint">Touch point.</param>
        public void NotifyFocus(Point touchPoint)
        {
            this.Focus?.Invoke(this, touchPoint);
        }

        /// <summary>
        /// Notifies the busy.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="busy">If set to <c>true</c> busy.</param>
        public void NotifyBusy(object sender, bool busy)
        {
            this.Busy?.Invoke(this, busy);
        }

        /// <summary>
        /// Notifies the availability.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="isAvailable">If set to <c>true</c> is available.</param>
        public void NotifyAvailability(object sender, bool isAvailable)
        {
            this.CameraAvailable = isAvailable;
        }

        /// <summary>
        /// Notifies the photo.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="imageData">Image data.</param>
        public void NotifyPhoto(object sender, byte[] imageData)
        {
            this.Photo?.Invoke(this, imageData);
        }
        /// <summary>
        /// Notifies the flash.
        /// </summary>
        /// <param name="flashOn">If set to <c>true</c> flash on.</param>
        public void NotifyFlash(bool flashOn)
        {
            this.Flash?.Invoke(this, flashOn);
        }

        /// <summary>
        /// Notifies the torch.
        /// </summary>
        /// <param name="torchOn">If set to <c>true</c> torch on.</param>
        public void NotifyTorch(bool torchOn)
        {
            this.Torch?.Invoke(this, torchOn);
        }

        /// <summary>
        /// Notifies the loading.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="loading">If set to <c>true</c> loading.</param>
        public void NotifyLoading(object sender, bool loading)
        {
            this.Loading?.Invoke(this, loading);
        }

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="T:Demo.Controls.CameraView"/> class.
        /// </summary>
        public CameraView()
        {
            this.BackgroundColor = Color.Black;
        }

        #endregion
    }

}
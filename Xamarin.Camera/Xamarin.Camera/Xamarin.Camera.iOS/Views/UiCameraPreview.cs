using System;
using System.Linq;
using AVFoundation;
using CoreFoundation;
using CoreGraphics;
using CoreMedia;
using CoreVideo;
using Foundation;
using UIKit;
using Xamarin.Forms;

using Xamarin.Camera.Controls;
using Xamarin.Camera.Infrastructure;
using Xamarin.Camera.iOS.Controls;

namespace Xamarin.Camera.iOS.Views
{

    public class UiCameraPreview : UIView
    {

        #region Events

        public event EventHandler<EventArgs> Tapped;

        #endregion

        #region Fields

        private readonly CameraPreview _Camera;

        private readonly CameraOptions _CameraOptions;

        private float _MaxZoom;

        private const float MinZoom = 1.0f;

        private AVCaptureDevice _MainDevice;

        private readonly NSObject _Notification;

        private UIPinchGestureRecognizer _Pinch;

        private readonly AVCaptureVideoPreviewLayer _PreviewLayer;

        private DispatchQueue _Queue;

        private OutputRecorder _Recorder;

        #endregion

        #region Constructors

        public UiCameraPreview(CameraPreview camera)
        {
            this._CameraOptions = camera.Camera;
            this._IsPreviewing = camera.IsPreviewing;
            this._Camera = camera;

            this.CaptureSession = new AVCaptureSession();
            this._PreviewLayer = new AVCaptureVideoPreviewLayer(this.CaptureSession)
            {
                Frame = this.Bounds,
                VideoGravity = AVLayerVideoGravity.ResizeAspectFill
            };
            this.Layer.AddSublayer(this._PreviewLayer);

            this.Initialize();

            MessagingCenter.Subscribe<LifeCyclePayload>(this, "", (p) =>
            {
                switch (p.Status)
                {
                    case LifeCycle.OnSleep:
                        //Sleep状態になるときにリソース解放
                        this.Release();
                        break;
                    case LifeCycle.OnResume:
                        //Resume状態になるときに初期化
                        this.Initialize();
                        break;
                }
            });

            // Adjust size and orientation of preview image
            this._Notification = UIApplication.Notifications.ObserveDidChangeStatusBarOrientation((sender, args) =>
            {
                switch (UIApplication.SharedApplication.StatusBarOrientation)
                {
                    case UIInterfaceOrientation.Portrait:
                        this._PreviewLayer.Connection.VideoOrientation = AVCaptureVideoOrientation.Portrait;
                        break;
                    case UIInterfaceOrientation.PortraitUpsideDown:
                        this._PreviewLayer.Connection.VideoOrientation = AVCaptureVideoOrientation.PortraitUpsideDown;
                        break;
                    case UIInterfaceOrientation.LandscapeLeft:
                        this._PreviewLayer.Connection.VideoOrientation = AVCaptureVideoOrientation.LandscapeLeft;
                        break;
                    case UIInterfaceOrientation.LandscapeRight:
                        this._PreviewLayer.Connection.VideoOrientation = AVCaptureVideoOrientation.LandscapeRight;
                        break;
                    default:
                        break;
                }
            });
            this._Camera.SizeChanged += (sender, args) =>
            {
                var rect = this._Camera.Bounds;
                this._PreviewLayer.Frame = new CGRect(new CGPoint(rect.X, rect.Y), new CGSize(rect.Width, rect.Height));
            };
        }

        #endregion

        #region Properties

        private bool _IsPreviewing;

        public bool IsPreviewing
        {
            get => this._IsPreviewing;
            set
            {
                if (value)
                {
                    this.CaptureSession.StartRunning();
                }
                else
                {
                    this.CaptureSession.StopRunning();
                }

                this._IsPreviewing = value;
            }
        }
        
        public AVCaptureSession CaptureSession { get; }

        public AVCaptureDeviceInput Input { get; set; }

        public AVCaptureVideoDataOutput Output { get; private set; }

        #endregion

        #region Methods

        private void Initialize()
        {
            //Pinchジェスチャ登録
            this.SetPinchGesture();

            //デバイス設定
            var videoDevices = AVCaptureDevice.DevicesWithMediaType(AVMediaType.Video);
            var cameraPosition = (this._CameraOptions == CameraOptions.Front) ? AVCaptureDevicePosition.Front : AVCaptureDevicePosition.Back;
            this._MainDevice = videoDevices.FirstOrDefault(d => d.Position == cameraPosition);

            this._MainDevice.LockForConfiguration(out var deviceError);
            if (deviceError != null)
            {
                Console.WriteLine($"Error: {deviceError.LocalizedDescription}");
                this._MainDevice.UnlockForConfiguration();
                return;
            }
            //フレームレート設定
            this._MainDevice.ActiveVideoMinFrameDuration = new CMTime(1, 24);
            this._MainDevice.UnlockForConfiguration();

            if (this._MainDevice == null)
            {
                return;
            }

            //max zoom
            this._MaxZoom = (float)Math.Min(this._MainDevice.ActiveFormat.VideoMaxZoomFactor, 6);

            //入力設定
            this.Input = new AVCaptureDeviceInput(this._MainDevice, out var error);
            this.CaptureSession.AddInput(this.Input);

            //出力設定
            this.Output = new AVCaptureVideoDataOutput();

            //フレーム処理用
            this._Queue = new DispatchQueue("myQueue");
            this.Output.AlwaysDiscardsLateVideoFrames = true;
            this._Recorder = new OutputRecorder() { Camera = this._Camera };
            this.Output.SetSampleBufferDelegate(this._Recorder, this._Queue);
            var vSettings = new AVVideoSettingsUncompressed();
            // Need not use alpha channel but can not use CV24BGR
            vSettings.PixelFormatType = CVPixelFormatType.CV32BGRA;
            this.Output.WeakVideoSettings = vSettings.Dictionary;

            this.CaptureSession.AddOutput(this.Output);

            if (this.IsPreviewing)
            {
                this.CaptureSession.StartRunning();
            }
        }

        protected virtual void OnTapped()
        {
            var eventHandler = this.Tapped;
            eventHandler?.Invoke(this, new EventArgs());
        }

        public void Release()
        {
            this.CaptureSession.StopRunning();
            this._Recorder.Dispose();
            this._Queue.Dispose();
            this.CaptureSession.RemoveOutput(this.Output);
            this.CaptureSession.RemoveInput(this.Input);
            this.Output.Dispose();
            this.Input.Dispose();
            this._MainDevice.Dispose();
            this.RemoveGestureRecognizer(this._Pinch);
        }

        #region Overrids

        public override void Draw(CGRect rect)
        {
            base.Draw(rect);
            this._PreviewLayer.Frame = rect;
        }

        public override void TouchesBegan(NSSet touches, UIEvent evt)
        {
            base.TouchesBegan(touches, evt);
            this.OnTapped();
        }

        #endregion

        #region Helpers

        private void SetPinchGesture()
        {
            nfloat lastScale = 1.0f;

            this._Pinch = new UIPinchGestureRecognizer((e) =>
            {
                switch (e.State)
                {
                    case UIGestureRecognizerState.Changed:
                        {
                            this._MainDevice.LockForConfiguration(out var deviceError);
                            if (deviceError != null)
                            {
                                Console.WriteLine($"Error: {deviceError.LocalizedDescription}");
                                this._MainDevice.UnlockForConfiguration();
                                return;
                            }

                            var scale = e.Scale + (1 - lastScale);
                            var zoom = this._MainDevice.VideoZoomFactor * scale;
                            if (zoom > this._MaxZoom) zoom = this._MaxZoom;
                            if (zoom < MinZoom) zoom = MinZoom;
                            this._MainDevice.VideoZoomFactor = zoom;
                            this._MainDevice.UnlockForConfiguration();
                            lastScale = e.Scale;
                            break;
                        }
                    case UIGestureRecognizerState.Ended:
                        lastScale = 1.0f;
                        break;
                    default:
                        // nothing to do
                        break;
                }
            });

            this.AddGestureRecognizer(this._Pinch);
        }

        #endregion

        #endregion

    }

}
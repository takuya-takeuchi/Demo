using System;
using System.Reactive.Concurrency;
using System.Reactive.Linq;
using System.Windows.Media.Imaging;
using GalaSoft.MvvmLight;
using OpenCvSharp;
using OpenCvSharp.Extensions;

namespace OpenCV3.ViewModels
{
    public class MainViewModel : ViewModelBase
    {

        #region フィールド

        private IDisposable _CaptureHandler;

        private CvCapture _CvCapture;

        #endregion

        #region コンストラクタ

        public MainViewModel()
        {
            this.CameraIndex = 0;
        }

        #endregion

        #region プロパティ

        private BitmapSource _CameraImage;

        public BitmapSource CameraImage
        {
            get
            {
                return this._CameraImage;
            }
            set
            {
                this._CameraImage = value;
                this.RaisePropertyChanged();
            }
        }

        private int _CameraIndex;

        public int CameraIndex
        {
            get
            {
                return this._CameraIndex;
            }
            set
            {
                this._CameraIndex = value;
                this.RaisePropertyChanged();

                this.ExecuteStart();
            }
        }

        #endregion

        #region メソッド

        #region ヘルパーメソッド

        private CvCapture CreateCvCapture()
        {
            CvCapture cvCapture = null;

            try
            {
                cvCapture = new CvCapture(this.CameraIndex);
            }
            catch (Exception)
            {
                // ignore
            }

            return cvCapture;
        }

        private void ExecuteStart()
        {
            this.ExecuteStop();

            this._CvCapture = this.CreateCvCapture();
            if (this._CvCapture == null)
            {
                return;
            }

            var ms = 60;
            this._CaptureHandler = Observable.Interval(TimeSpan.FromMilliseconds(ms), DispatcherScheduler.Current)
                .Select(_ =>
                {
                    if (this._CvCapture == null)
                    {
                        return null;
                    }

                    return Cv.QueryFrame(this._CvCapture);
                })
                .Where(frame => frame != null)
                .Subscribe(frame =>
                {
                    var writeableBitmap = frame.ToWriteableBitmap();
                    frame.Dispose();
                    this.CameraImage = writeableBitmap;
                });
        }

        private void ExecuteStop()
        {
            if (this._CvCapture != null)
            {
                this._CvCapture.Dispose();
                this._CvCapture = null;
            }

            if (this._CaptureHandler != null)
            {
                this._CaptureHandler.Dispose();
                this._CaptureHandler = null;
            }
        }

        #endregion

        #endregion

    }

}
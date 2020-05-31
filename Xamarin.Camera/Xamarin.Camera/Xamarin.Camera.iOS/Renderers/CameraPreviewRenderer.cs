using Xamarin.Camera.Controls;
using Xamarin.Camera.Infrastructure;
using Xamarin.Camera.iOS.Renderers;
using Xamarin.Camera.iOS.Views;
using Xamarin.Forms;
using Xamarin.Forms.Platform.iOS;

[assembly: ExportRenderer(typeof(CameraPreview), typeof(CameraPreviewRenderer))]

namespace Xamarin.Camera.iOS.Renderers
{

    public sealed class CameraPreviewRenderer : ViewRenderer<CameraPreview, UiCameraPreview>
    {

        #region Fields

        private UiCameraPreview _UiCameraPreview;

        #endregion

        #region Methods

        #region Overrids

        protected override void OnElementChanged(ElementChangedEventArgs<CameraPreview> e)
        {
            base.OnElementChanged(e);

            if (this.Control == null)
            {
                this._UiCameraPreview = new UiCameraPreview(e.NewElement);
                this.SetNativeControl(this._UiCameraPreview);
            }
            if (e.OldElement != null)
            {
            }
            if (e.NewElement != null)
            {
            }
        }

        protected override void OnElementPropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            base.OnElementPropertyChanged(sender, e);
            if (this.Element == null || this.Control == null)
                return;

            // PCL側の変更をプラットフォームに反映
            if (e.PropertyName == nameof(this.Element.IsPreviewing))
            {
                this.Control.IsPreviewing = this.Element.IsPreviewing;
            }
        }

        #endregion

        #endregion

        #region IDisposable Members

        private bool _Disposed = false;

        protected override void Dispose(bool disposing)
        {
            if (this._Disposed)
                return;

            if (disposing)
            {
                // managed resources here
                this.Control.Release();
                this.Control.CaptureSession.Dispose();
                MessagingCenter.Unsubscribe<LifeCyclePayload>(this.Control, "");
                this.Control.Dispose();
            }

            // unmanaged resources here

            // Note disposing has been done.
            this._Disposed = true;

            // Call the base class implementation.
            base.Dispose(disposing);
        }

        #endregion

    }

}
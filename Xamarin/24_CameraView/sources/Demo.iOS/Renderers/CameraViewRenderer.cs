using System;
using Demo.Controls;
using Demo.iOS.Renderers;
using Xamarin.Forms;
using Xamarin.Forms.Platform.iOS;

[assembly: ExportRenderer(typeof(CameraView), typeof(CameraViewRenderer))]
namespace Demo.iOS.Renderers
{

    /// <summary>
    /// Camera renderer.
    /// </summary>
    public class CameraViewRenderer : ViewRenderer<CameraView, CameraIOS>
    {
        #region Private Properties

        /// <summary>
        /// The bodyshop camera IO.
        /// </summary>
        private CameraIOS _bodyshopCameraIOS;

        #endregion

        #region Protected Methods

        /// <summary>
        /// Raises the element changed event.
        /// </summary>
        /// <param name="e">E.</param>
        protected override void OnElementChanged(ElementChangedEventArgs<CameraView> e)
        {
            base.OnElementChanged(e);

            if (this.Control == null)
            {
                this._bodyshopCameraIOS = new CameraIOS();

                this.SetNativeControl(this._bodyshopCameraIOS);
            }

            if (e.OldElement != null)
            {
                e.OldElement.Flash -= this.HandleFlash;
                e.OldElement.OpenCamera -= this.HandleCameraInitialisation;
                e.OldElement.Focus -= this.HandleFocus;
                e.OldElement.Shutter -= this.HandleShutter;
                e.OldElement.Widths -= this.HandleWidths;

                this._bodyshopCameraIOS.Busy -= e.OldElement.NotifyBusy;
                this._bodyshopCameraIOS.Available -= e.OldElement.NotifyAvailability;
                this._bodyshopCameraIOS.Photo -= e.OldElement.NotifyPhoto;
            }

            if (e.NewElement != null)
            {
                e.NewElement.Flash += this.HandleFlash;
                e.NewElement.OpenCamera += this.HandleCameraInitialisation;
                e.NewElement.Focus += this.HandleFocus;
                e.NewElement.Shutter += this.HandleShutter;
                e.NewElement.Widths += this.HandleWidths;

                this._bodyshopCameraIOS.Busy += e.NewElement.NotifyBusy;
                this._bodyshopCameraIOS.Available += e.NewElement.NotifyAvailability;
                this._bodyshopCameraIOS.Photo += e.NewElement.NotifyPhoto;
            }
        }

        /// <summary>
        /// Dispose the specified disposing.
        /// </summary>
        /// <param name="disposing">If set to <c>true</c> disposing.</param>
        protected override void Dispose(bool disposing)
        {
            if (this._bodyshopCameraIOS != null)
            {
                // stop output session and dispose camera elements before popping page
                this._bodyshopCameraIOS.StopAndDispose();
                this._bodyshopCameraIOS.Dispose();
            }

            base.Dispose(disposing);
        }

        /// <summary>
        /// Raises the element property changed event.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="e">E.</param>
        protected override void OnElementPropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            base.OnElementPropertyChanged(sender, e);

            if (this.Element != null && this._bodyshopCameraIOS != null)
            {
                if (e.PropertyName == VisualElement.HeightProperty.PropertyName ||
                    e.PropertyName == VisualElement.WidthProperty.PropertyName)
                {
                    this._bodyshopCameraIOS.SetBounds((nint)this.Element.Width, (nint)this.Element.Height);
                }
            }
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// Handles the widths.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="e">E.</param>
        private void HandleWidths(object sender, float e)
        {
            this._bodyshopCameraIOS.SetWidths(e);
        }

        /// <summary>
        /// Handles the shutter.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="e">E.</param>
        private async void HandleShutter(object sender, EventArgs e)
        {
            await this._bodyshopCameraIOS.TakePhoto();
        }

        /// <summary>
        /// Handles the orientation change.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="e">E.</param>
        private void HandleOrientationChange(object sender, Orientation e)
        {
            this._bodyshopCameraIOS.HandleOrientationChange(e);
        }

        /// <summary>
        /// Handles the focus.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="e">E.</param>
        private void HandleFocus(object sender, Point e)
        {
            this._bodyshopCameraIOS.ChangeFocusPoint(e);
        }

        /// <summary>
        /// Handles the camera initialisation.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="args">If set to <c>true</c> arguments.</param>
        private void HandleCameraInitialisation(object sender, bool args)
        {
            this._bodyshopCameraIOS.InitializeCamera();

            this.Element.OrientationChange += this.HandleOrientationChange;
        }

        /// <summary>
        /// Handles the flash.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="args">If set to <c>true</c> arguments.</param>
        private void HandleFlash(object sender, bool args)
        {
            this._bodyshopCameraIOS.SwitchFlash(args);
        }

        /// <summary>
        /// Handles the focus change.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="args">Arguments.</param>
        private void HandleFocusChange(object sender, Point args)
        {
            this._bodyshopCameraIOS.ChangeFocusPoint(args);
        }

        #endregion
    }
}
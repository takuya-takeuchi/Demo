using System;

using Xamarin.Forms;
using Xamarin.Forms.Platform.Android;

using Demo.Controls;
using Demo.Droid.Renderers;

[assembly: ExportRenderer(typeof(CameraView), typeof(CameraViewRenderer))]
namespace Demo.Droid.Renderers
{

    /// <summary>
    /// Bodyshop camera renderer.
    /// </summary>
    public class CameraViewRenderer : ViewRenderer<CameraView, CameraDroid>
    {
        #region Private Properties

        /// <summary>
        /// The bodyshop camera droid.
        /// </summary>
        private CameraDroid Camera;

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
                this.Camera = new CameraDroid(this.Context);

                this.SetNativeControl(this.Camera);
            }

            if (e.OldElement != null)
            {
                // something wrong here, not being called on disposal
            }

            if (e.NewElement != null)
            {
                this.Camera.Available += e.NewElement.NotifyAvailability;
                this.Camera.Photo += e.NewElement.NotifyPhoto;
                this.Camera.Busy += e.NewElement.NotifyBusy;

                e.NewElement.Flash += HandleFlashChange;
                e.NewElement.OpenCamera += HandleCameraInitialisation;
                e.NewElement.Focus += HandleFocus;
                e.NewElement.Shutter += HandleShutter;
            }
        }

        /// <summary>
        /// Ons the layout.
        /// </summary>
        /// <param name="changed">If set to <c>true</c> changed.</param>
        /// <param name="l">L.</param>
        /// <param name="t">T.</param>
        /// <param name="r">The red component.</param>
        /// <param name="b">The blue component.</param>
        protected override void OnLayout(bool changed, int l, int t, int r, int b)
        {
            base.OnLayout(changed, l, t, r, b);

            this.Camera.OnLayout(l, t, r, b);
        }

        /// <summary>
        /// Dispose the specified disposing.
        /// </summary>
        /// <param name="disposing">If set to <c>true</c> disposing.</param>
        protected override void Dispose(bool disposing)
        {
            this.Element.Flash -= HandleFlashChange;
            this.Element.OpenCamera -= HandleCameraInitialisation;
            this.Element.Focus -= HandleFocus;
            this.Element.Shutter -= HandleShutter;

            this.Camera.Available -= this.Element.NotifyAvailability;
            this.Camera.Photo -= this.Element.NotifyPhoto;
            this.Camera.Busy -= this.Element.NotifyBusy;

            base.Dispose(disposing);
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// Handles the camera initialisation.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="args">If set to <c>true</c> arguments.</param>
        private void HandleCameraInitialisation(object sender, bool args)
        {
            this.Camera.OpenCamera();
        }

        /// <summary>
        /// Handles the flash change.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="args">If set to <c>true</c> arguments.</param>
        private void HandleFlashChange(object sender, bool args)
        {
            this.Camera.SwitchFlash(args);
        }

        /// <summary>
        /// Handles the shutter.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="e">E.</param>
        private void HandleShutter(object sender, EventArgs e)
        {
            this.Camera.TakePhoto();
        }

        /// <summary>
        /// Handles the focus.
        /// </summary>
        /// <param name="sender">Sender.</param>
        /// <param name="e">E.</param>
        private void HandleFocus(object sender, Point e)
        {
            this.Camera.ChangeFocusPoint(e);
        }

        #endregion

    }
}
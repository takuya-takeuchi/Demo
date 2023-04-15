using Android.Hardware.Camera2;

namespace Demo.Droid.Renderers
{

    /// <summary>
    /// Camera state listener.
    /// </summary>
    public class CameraStateListener : CameraDevice.StateCallback
    {
        /// <summary>
        /// The camera.
        /// </summary>
        public CameraDroid Camera;

        /// <summary>
        /// Called when camera is connected.
        /// </summary>
        /// <param name="camera">Camera.</param>
        public override void OnOpened(CameraDevice camera)
        {
            if (this.Camera != null)
            {
                this.Camera._cameraDevice = camera;
                this.Camera.StartPreview();
                this.Camera.OpeningCamera = false;

                this.Camera?.NotifyAvailable(true);
            }
        }

        /// <summary>
        /// Called when camera is disconnected.
        /// </summary>
        /// <param name="camera">Camera.</param>
        public override void OnDisconnected(CameraDevice camera)
        {
            if (this.Camera != null)
            {
                camera.Close();
                this.Camera._cameraDevice = null;
                this.Camera.OpeningCamera = false;

                this.Camera?.NotifyAvailable(false);
            }
        }

        /// <summary>
        /// Called when an error occurs.
        /// </summary>
        /// <param name="camera">Camera.</param>
        /// <param name="error">Error.</param>
        public override void OnError(CameraDevice camera, CameraError error)
        {
            camera.Close();

            if (this.Camera != null)
            {
                this.Camera._cameraDevice = null;
                this.Camera.OpeningCamera = false;

                this.Camera?.NotifyAvailable(false);
            }
        }
    }
}
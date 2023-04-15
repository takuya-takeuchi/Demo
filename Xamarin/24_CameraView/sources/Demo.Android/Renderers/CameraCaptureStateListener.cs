using System;
using Android.Hardware.Camera2;

namespace Demo.Droid.Renderers
{

    /// <summary>
    /// This CameraCaptureSession.StateListener uses Action delegates to allow 
    /// the methods to be defined inline, as they are defined more than once.
    /// </summary>
    public class CameraCaptureStateListener : CameraCaptureSession.StateCallback
    {
        /// <summary>
        /// The on configure failed action.
        /// </summary>
        public Action<CameraCaptureSession> OnConfigureFailedAction;

        /// <summary>
        /// The on configured action.
        /// </summary>
        public Action<CameraCaptureSession> OnConfiguredAction;

        /// <summary>
        /// Ons the configure failed.
        /// </summary>
        /// <param name="session">Session.</param>
        public override void OnConfigureFailed(CameraCaptureSession session)
        {
            if (this.OnConfigureFailedAction != null)
            {
                this.OnConfigureFailedAction(session);
            }
        }

        /// <summary>
        /// Ons the configured.
        /// </summary>
        /// <param name="session">Session.</param>
        public override void OnConfigured(CameraCaptureSession session)
        {
            if (this.OnConfiguredAction != null)
            {
                this.OnConfiguredAction(session);
            }
        }
    }
}
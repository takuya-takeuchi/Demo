using System;

using Android.Hardware.Camera2;

namespace Demo.Droid.Renderers
{

    /// <summary>
    /// This CameraCaptureSession.StateListener uses Action delegates to allow 
    /// the methods to be defined inline, as they are defined more than once.
    /// </summary>
    public class CameraCaptureListener : CameraCaptureSession.CaptureCallback
    {
        /// <summary>
        /// Occurs when photo complete.
        /// </summary>
        public event EventHandler PhotoComplete;

        /// <summary>
        /// Ons the capture completed.
        /// </summary>
        /// <param name="session">Session.</param>
        /// <param name="request">Request.</param>
        /// <param name="result">Result.</param>
        public override void OnCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
            TotalCaptureResult result)
        {
            this.PhotoComplete?.Invoke(this, EventArgs.Empty);
        }
    }
}
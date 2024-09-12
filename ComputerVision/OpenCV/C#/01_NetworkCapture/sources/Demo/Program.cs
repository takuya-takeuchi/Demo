using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using NLog;
using OpenCvSharp;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            if (args.Length != 1)
            {
                Logger.Error($"{nameof(Demo)} <url>");
                return;
            }

            var url = args[0];
            Logger.Info($"Url: {url}");

            using var videoCapture = new VideoCapture();
            // var isOpened = videoCapture.Open(url, VideoCaptureAPIs.FFMPEG);
            var isOpened = videoCapture.Open(url, VideoCaptureAPIs.GSTREAMER);
            if (!isOpened)
            {
                Logger.Error($"Failed to open {url}");
                return;
            }

            while (true)
            {
                using var frame = new Mat();
                if (!videoCapture.Read(frame))
                {
                    Logger.Error($"Failed to read frame");
                    break;
                }

                Logger.Info($"Available frame: {frame.Cols}x{frame.Rows}");
            }

            Logger.Info($"Finish");
        }

        #region Event Handlers
        #endregion

        #region Helpers
        #endregion

        #endregion

    }

}

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
            if (args.Length != 2)
            {
                Logger.Error($"{nameof(Demo)} <url> <backend: GSTREAMER, FFMPEG...>");
                return;
            }

            var url = args[0];
            var backend = args[1];
            Logger.Info($"    Url: {url}");
            Logger.Info($"Backend: {backend}");

            if (!Enum.TryParse<VideoCaptureAPIs>(backend, out var api))
            {
                Logger.Error($"'{backend}' is invalid value");
                return;
            }

            using var videoCapture = new VideoCapture();
            var isOpened = videoCapture.Open(url, api);
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

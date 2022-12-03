using CommandLine;
using NLog;
using OpenCvSharp;
using OpenCvSharp.Aruco;

namespace Demo
{

    internal class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            var parseResult = Parser.Default.ParseArguments<Options>(args);

            switch (parseResult.Tag)
            {
                case ParserResultType.Parsed:
                    var parsed = parseResult as Parsed<Options>;
                    var option = parsed.Value;

                    Logger.Info($"{nameof(Options.ConvertRgb),25}: {option.ConvertRgb}");
                    Logger.Info($"{nameof(Options.Output),25}: {option.Output}");

                    Run(option);
                    break;
                case ParserResultType.NotParsed:
                    var notParsed = parseResult as NotParsed<Options>;
                    Logger.Error($"{nameof(Demo)} --dictionary <value> --parameter <value> --size <value> [--output]");
                    break;
            }
        }

        #region Helpers

        private static void Run(Options option)
        {
            // setup camera
            // DSHOW is slow
            using var videoCapture = new VideoCapture(0, VideoCaptureAPIs.MSMF);

            // But MSFM has problem that startup is slow
            // https://github.com/opencv/opencv/pull/20327#issue-680368052
            Environment.SetEnvironmentVariable("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0");

            if (!option.ConvertRgb)
            {
                var convert = videoCapture.Get(VideoCaptureProperties.ConvertRgb);
                if (!videoCapture.Set(VideoCaptureProperties.ConvertRgb, 0))
                {
                    Logger.Error($"Failed to change CAP_PROP_CONVERT_RGB from {convert} to 0");
                }
                else
                {
                    Logger.Info($"Succeeded to change CAP_PROP_CONVERT_RGB from {convert} to 0");
                }
            }

            var mode = videoCapture.Get(VideoCaptureProperties.Mode);
            Logger.Info($"CAP_PROP_MODE: {mode}");

            var fourcc = videoCapture.Get(VideoCaptureProperties.FourCC);
            Logger.Info($"CAP_PROP_FOURCC: {fourcc} ({videoCapture.FourCC})");

            const int width = 1920;
            const int height = 1080;
            videoCapture.Fps = 30;
            videoCapture.FrameWidth = width;
            videoCapture.FrameHeight = height;
            if (!videoCapture.IsOpened())
                throw new Exception("VideoCapture initialization failed");

            var output = option.Output;

            // camera buffer frame
            using var frame = new Mat();
            using var rgb = new Mat();
            while (true)
            {
                try
                {
                    var now = DateTime.Now;
                    
                    videoCapture.Grab();
                    videoCapture.Retrieve(frame);
                    if (frame.Empty())
                        break;

                    Directory.CreateDirectory(output);
                    Logger.Info($"Width: {frame.Cols}, Height: {frame.Rows}, Channels: {frame.Channels()}, Type: {frame.Type()}, Total: {frame.Total()}");
                    if (option.ConvertRgb)
                    {
                        Cv2.ImWrite(Path.Combine(output, $"{now:yyyyMMddHHmmssfff}.jpg"), frame);
                    }
                    else
                    {
                        using var tmp = new Mat(height, width, MatType.CV_8UC2, frame.Data);
                        // ToDo: You have to change ColorConversionCodes to fit CAP_PROP_FOURCC
                        Cv2.CvtColor(tmp, rgb, ColorConversionCodes.YUV2BGR_YUY2);
                        Cv2.ImWrite(Path.Combine(output, $"{now:yyyyMMddHHmmssfff}.jpg"), rgb);
                    }
                }
                catch (Exception ex)
                {
                    Logger.Info($"Failed to capture image. Reason: {ex.Message}");
                    return;
                }
            }
        }

        #endregion

        #endregion

    }

}
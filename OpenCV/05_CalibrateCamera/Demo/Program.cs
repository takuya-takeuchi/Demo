using System;

using CommandLine;
using NLog;
using OpenCvSharp;

namespace Demo
{

    internal class Program
    {

        #region Fields

        private static Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            var parseResult = Parser.Default.ParseArguments<Options>(args);
            Options option = null;

            switch (parseResult.Tag)
            {
                case ParserResultType.Parsed:
                    var parsed = parseResult as Parsed<Options>;
                    option = parsed.Value;

                    Logger.Info($"{nameof(Options.HorizontalPatternSize),25}: {option.HorizontalPatternSize}");
                    Logger.Info($"{nameof(Options.VerticalPatternSize),25}: {option.VerticalPatternSize}");
                    Logger.Info($"{nameof(Options.Size),25}: {option.Size}");
                    Logger.Info($"{nameof(Options.Count),25}: {option.Count}");

                    // check parameter
                    var validator = new OptionsValidator();
                    var results = validator.Validate(option);
                    if (!results.IsValid)
                    {
                        foreach (var validationFailure in results.Errors)
                            Logger.Error($"{validationFailure.PropertyName}: {validationFailure.ErrorMessage}");
                        return;
                    }

                    Run(option);
                    break;
                case ParserResultType.NotParsed:
                    var notParsed = parseResult as NotParsed<Options>;
                    Logger.Error($"{nameof(Demo)} --horizontal <value> --vertical <value> --size <value> --count <value>");
                    break;
            }
        }

        #region Helpers

        private static void Run(Options option)
        {
            using var videoCapture = new VideoCapture();
            videoCapture.Set(VideoCaptureProperties.FrameWidth, 1920);
            videoCapture.Set(VideoCaptureProperties.FrameHeight, 1080);
            videoCapture.Open(0);
            if (!videoCapture.IsOpened())
                throw new Exception("VideoCapture initialization failed");

            var patterSize = new Size(option.HorizontalPatternSize, option.VerticalPatternSize);

            using var frame = new Mat();

            var windowName = "Find Chessboard Corners";
            while (true)
            {
                try
                {
                    videoCapture.Read(frame);
                    if (frame.Empty())
                        break;

                    // Cv2.ImShow(windowName, frame);
                    // Cv2.WaitKey(10);

                    var result = Cv2.FindChessboardCorners(frame, patterSize, out var corners);
                    Cv2.DrawChessboardCorners(frame, patterSize, corners, result);

                    Cv2.ImShow(windowName, frame);
                    Cv2.WaitKey(10);

                    if (!result)
                    {
                        Logger.Error("Failed to detect chessboard corners");
                        continue;
                    }
                }
                catch (Exception ex)
                {

                }
            }

        }

        #endregion

        #endregion

    }

}
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
                    Logger.Info($"{nameof(Options.Output),25}: {option.Output}");

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
                    Logger.Error($"{nameof(Demo)} --horizontal <value> --vertical <value> --size <value> --count <value> [--output]");
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

            videoCapture.FrameWidth = 1920;
            videoCapture.FrameHeight = 1080;
            if (!videoCapture.IsOpened())
                throw new Exception("VideoCapture initialization failed");

            var output = option.Output;
            var size = option.Size;
            var referenceCount = option.Count;
            var patternSize = new Size(option.HorizontalPatternSize, option.VerticalPatternSize);

            // calibration pattern points in the calibration pattern coordinate space
            var points = new List<Point3f>();
            var objectPoints = new List<Mat<Point3f>>();
            for (var v = 0; v < patternSize.Height; v++)
            for (var h = 0; h < patternSize.Width; h++)
                points.Add(new Point3f(h * size, v * size, 0.0f));

            for (var count = 0; count < referenceCount; count++)
                objectPoints.Add(Mat<Point3f>.FromArray(points));

            // camera buffer frame
            using var frame = new Mat();

            var detectedCorners = new List<Mat<Point2f>>();

            var windowName = "Find Chessboard Corners";
            while (true)
            {
                try
                {
                    var now = DateTime.Now;
                    videoCapture.Read(frame);
                    if (frame.Empty())
                        break;

                    var corners = new Mat<Point2f>();
                    var result = Cv2.FindChessboardCorners(frame, patternSize, corners);
                    Cv2.DrawChessboardCorners(frame, patternSize, corners, result);

                    // You need not to invoke Cv2.CornerSubPix
                    // Because cvFindChessboardCorners has already invoke cvFindCornerSubPix

                    Cv2.ImShow(windowName, frame);
                    Cv2.WaitKey(10);

                    if (output)
                    {
                        var directory = "outputs";
                        Directory.CreateDirectory("outputs");
                        Cv2.ImWrite(Path.Combine(directory, $"{now.ToString("yyyyMMddHHmmssfff")}.jpg"), frame);
                    }

                    if (!result)
                    {
                        Logger.Error("Failed to detect chessboard corners");

                        corners.Dispose();
                        continue;
                    }

                    if (result)
                    {
                        detectedCorners.Add(corners);
                        Logger.Info($"Succeeded to detect chessboard corners. [{detectedCorners.Count}/{referenceCount}]");
                    }

                    if (detectedCorners.Count == referenceCount)
                    {
                        Logger.Info("Finish to collect chessboard corners");
                        break;
                    }
                }
                catch (Exception ex)
                {
                    Logger.Info($"Failed to detect chessboard corners. Reason: {ex.Message}");

                    Dispose(objectPoints);
                    Dispose(detectedCorners);
                    return;
                }
            }

            Logger.Info("CalibrateCamera...");
            using var cameraIntrinsicMatrix = new Mat<double>(Mat.Eye(3, 3, MatType.CV_64FC1));
            using var distortionCoefficients = new Mat<double>();
            var calibrationFlags = CalibrationFlags.UseIntrinsicGuess | CalibrationFlags.FixK5;
            var overallRmsReProjectionError = Cv2.CalibrateCamera(objectPoints,
                                                                  detectedCorners,
                                                                  frame.Size(),
                                                                  cameraIntrinsicMatrix,
                                                                  distortionCoefficients,
                                                                  out var rotationVectors,
                                                                  out var translationVectors,
                                                                  calibrationFlags);

            Logger.Info($"Overall RMS re-projection error: {overallRmsReProjectionError}");
            Logger.Info($"        Camera Intrinsic Matrix: {cameraIntrinsicMatrix}");
            Logger.Info($"        Distortion Coefficients: {distortionCoefficients}");

            var dictionary = new Dictionary<string, object>
            {
                { "Count", referenceCount },
                { "ChessSize", size },
                { "PatternRow", patternSize.Height },
                { "PatternColumn", patternSize.Width },
                { "RMS", overallRmsReProjectionError },
                { "CameraIntrinsicMatrix", cameraIntrinsicMatrix },
                { "DistortionCoefficients", distortionCoefficients },
                // { "RotationVectors", rotationVectors },
                // { "TranslationVectors", translationVectors }
            };
            
            WriteToFile(dictionary, "calibration.yaml");
        }

        private static void Dispose(IEnumerable<IDisposable> collection)
        {
            foreach (var item in collection)
                item.Dispose();
        }

        private static void Dispose(IEnumerable<IEnumerable<IDisposable>> collections)
        {
            foreach (var collection in collections)
                Dispose(collection);
        }

        private static void WriteToFile(Dictionary<string, object> dictionary, string filename)
        {
            using var fs = new FileStorage(filename, FileStorage.Modes.Write | FileStorage.Modes.FormatYaml);
            foreach (var (key, value) in dictionary)
            {
                switch (value)
                {
                    case int i:
                        fs.Write(key, i);
                        break;
                    case float f:
                        fs.Write(key, f);
                        break;
                    case double d:
                        fs.Write(key, d);
                        break;
                    case Mat mat:
                        fs.Write(key, mat);
                        break;
                }
            }
        }

        #endregion

        #endregion

    }

}
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

                    Logger.Info($"{nameof(Options.Size),25}: {option.Size}");
                    Logger.Info($"{nameof(Options.Dictionary),25}: {option.Dictionary}");
                    Logger.Info($"{nameof(Options.Parameter),25}: {option.Parameter}");
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

            videoCapture.FrameWidth = 1920;
            videoCapture.FrameHeight = 1080;
            if (!videoCapture.IsOpened())
                throw new Exception("VideoCapture initialization failed");

            var output = option.Output;
            var size = option.Size;
            var dictionary = Enum.Parse<PredefinedDictionaryName>(option.Dictionary);
            var parameter = option.Parameter;

            // read camera intrinsic matrix and distortion coefficients
            ReadFromFile(parameter, out var cameraIntrinsicMatrix, out var distortionCoefficients);

            // setup ArUco dictinary and parameters
            using var arucoDictinary = CvAruco.GetPredefinedDictionary(dictionary);
            var detectorParameters = DetectorParameters.Create();

            // camera buffer frame
            using var frame = new Mat();
            using var undistorted = new Mat();

            using var newCamera = Cv2.GetOptimalNewCameraMatrix(cameraIntrinsicMatrix,
                                                                distortionCoefficients,
                                                                new Size(1920, 1080),
                                                                1,
                                                                new Size(1920, 1080),
                                                                out var roi);

            var windowName = "Find ArUco markers";
            while (true)
            {
                try
                {
                    var now = DateTime.Now;
                    videoCapture.Read(frame);
                    if (frame.Empty())
                        break;

                    Cv2.Undistort(frame, undistorted, cameraIntrinsicMatrix, distortionCoefficients, newCamera);
                    CvAruco.DetectMarkers(undistorted, arucoDictinary, out var corners, out var ids, detectorParameters, out var rejectedImagePoints);
                    if (ids.Length > 0)
                    {
                        CvAruco.DrawDetectedMarkers(undistorted, corners, ids, new Scalar(0, 255, 0));

                        using var rotationVectors = new Mat();
                        using var translationVectors = new Mat();
                        using var objPoints = new Mat();
                        CvAruco.EstimatePoseSingleMarkers(corners,
                                                          size,
                                                          newCamera,
                                                          distortionCoefficients,
                                                          rotationVectors,
                                                          translationVectors,
                                                          objPoints);

                        float length = size / 2f;
                        for(var index = 0; index < ids.Length; index++)
                        {
                            // project axis points
                            var axisPoints = InputArray.Create(new []
                            {
                                new Point3f(0, 0, 0),
                                new Point3f(length, 0, 0),
                                new Point3f(0, length, 0),
                                new Point3f(0, 0, length),
                            });

                            var imagePoints = new Mat();
                            Cv2.ProjectPoints(axisPoints,
                                              InputArray.Create(new[] { rotationVectors.Get<Vec3d>(index) }),
                                              InputArray.Create(new[] { translationVectors.Get<Vec3d>(index) }),
                                              newCamera,
                                              distortionCoefficients,
                                              imagePoints);

                            // draw axis lines
                            var point0 = imagePoints.Get<Point2f>(0).ToPoint();
                            var point1 = imagePoints.Get<Point2f>(1).ToPoint();
                            var point2 = imagePoints.Get<Point2f>(2).ToPoint();
                            var point3 = imagePoints.Get<Point2f>(3).ToPoint();
                            Cv2.Line(undistorted, point0, point1, new Scalar(0, 0, 255), 3);
                            Cv2.Line(undistorted, point0, point2, new Scalar(0, 255, 0), 3);
                            Cv2.Line(undistorted, point0, point3, new Scalar(255, 0, 0), 3);
                        }

                        // // convert to Rodrigues
                        // using var rotationVectorsMatrix = new Mat();
                        // Cv2.Rodrigues(rotationVectors, rotationVectorsMatrix);

                        // // transpose
                        // using var transposedTranslationVectors = translationVectors.T();

                        // // compose
                        // using var projectionMatrix  = rotationVectorsMatrix + transposedTranslationVectors;

                        // using var cameraMatrix = new Mat(); // camera intrinsic matrix rather than extrinsic matrix
                        // using var externalRotationMatrix = new Mat();
                        // using var translationVector = new Mat();
                        // using var rotationMatrixAroundXAxis = new Mat();
                        // using var rotationMatrixAroundYAxis = new Mat();
                        // using var rotationMatrixAroundZAxis = new Mat();
                        // using var eulerAngles = new Mat();
                        // Cv2.DecomposeProjectionMatrix(projectionMatrix,
                        //                               cameraMatrix,
                        //                               externalRotationMatrix,
                        //                               translationVector,
                        //                               rotationMatrixAroundXAxis,
                        //                               rotationMatrixAroundYAxis,
                        //                               rotationMatrixAroundZAxis,
                        //                               eulerAngles);
                    }

                    Cv2.ImShow(windowName, undistorted);
                    Cv2.WaitKey(10);

                    if (output)
                    {
                        var directory = "outputs";
                        Directory.CreateDirectory("outputs");
                        Cv2.ImWrite(Path.Combine(directory, $"{now:yyyyMMddHHmmssfff}.jpg"), undistorted);
                    }
                }
                catch (Exception ex)
                {
                    Logger.Info($"Failed to detect ArUco markers. Reason: {ex.Message}");
                    return;
                }
            }
        }

        private static void Dispose(IEnumerable<IDisposable> collection)
        {
            foreach (var item in collection)
                item.Dispose();
        }

        private static void ReadFromFile(string filename, out Mat cameraIntrinsicMatrix, out Mat distortionCoefficients)
        {
            using var fs = new FileStorage(filename, FileStorage.Modes.Read);
            cameraIntrinsicMatrix = fs["CameraIntrinsicMatrix"]?.ReadMat();
            distortionCoefficients = fs["DistortionCoefficients"]?.ReadMat();
        }

        #endregion

        #endregion

    }

}
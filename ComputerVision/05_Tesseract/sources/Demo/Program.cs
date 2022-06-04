using System.Diagnostics;

using NLog;
using TesseractOCR;
using TesseractOCR.Enums;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        private static readonly Stopwatch Stopwatch = new Stopwatch();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            if (args.Length != 4)
            {
                Logger.Error($"{nameof(Demo)} <tess data path> <language> <engine mode> <image file path>");
                return;
            }

            // Setup ocr engine
            var tessDataDir = args[0];
            if (!Directory.Exists(tessDataDir))
            {
                Logger.Error($"{tessDataDir} is missing");
                return;
            }

            var languageValue = args[1];
            if (!Enum.TryParse<Language>(languageValue, true, out var language))
            {
                Logger.Error($"{languageValue} is invalid. You can specify language name like Japanese, English ...");
                return;
            }

            var engineModeValue = args[2];
            if (!Enum.TryParse<EngineMode>(engineModeValue, true, out var engineMode))
            {
                Logger.Error($"{engineModeValue} is invalid. You can specify TesseractOnly, LstmOnly or TesseractAndLstm");
                return;
            }

            // Read image binary
            var path = args[3];
            var binary = GetImageBinary(path);
            if (binary == null)
                return;

            using var engine = CreateEngine(tessDataDir, language, engineMode);
            using var image = LoadTesseractImage(binary);
            RunOcr(engine, image);
        }

        #region Helpers

        private static TesseractOCR.Pix.Image LoadTesseractImage(byte[] image)
        {
            Stopwatch.Restart();

            try
            {
                return TesseractOCR.Pix.Image.LoadFromMemory(image);
            }
            catch (Exception e)
            {
                Logger.Error(e, "Failed to convert to Tesseract Image");
                return null;
            }
            finally
            {
                var ms = Stopwatch.ElapsedMilliseconds;
                Logger.Info($"{nameof(LoadTesseractImage)}: {ms} ms");
            }
        }

        private static byte[]? GetImageBinary(string path)
        {
            Stopwatch.Restart();

            try
            {
                return File.ReadAllBytes(path);
            }
            catch (Exception e)
            {
                Logger.Error(e, "Failed to get image file binary");
                return null;
            }
            finally
            {
                var ms = Stopwatch.ElapsedMilliseconds;
                Logger.Info($"{nameof(GetImageBinary)}: {ms} ms");
            }
        }

        private static void RunOcr(Engine? engine, TesseractOCR.Pix.Image image)
        {
            Stopwatch.Restart();

            try
            {
                using var page = engine.Process(image);
                Logger.Info("Mean confidence: {0}", page.MeanConfidence);
                Logger.Info("Text: \r\n{0}", page.Text);
            }
            catch (Exception e)
            {
                Logger.Error(e, "Failed to run ocr");
            }
            finally
            {
                var ms = Stopwatch.ElapsedMilliseconds;
                Logger.Info($"{nameof(RunOcr)}: {ms} ms");
            }
        }

        private static Engine? CreateEngine(string tessDataDir, Language language, EngineMode engineMode)
        {
            Stopwatch.Restart();

            try
            {
                return new Engine(tessDataDir, language, engineMode);
            }
            catch (Exception e)
            {
                Logger.Error(e, "Failed to create ocr engine");
                return null;
            }
            finally
            {
                var ms = Stopwatch.ElapsedMilliseconds;
                Logger.Info($"{nameof(CreateEngine)}: {ms} ms");
            }
        }

        #endregion

        #endregion

    }

}
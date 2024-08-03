using System;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using Windows.Graphics.Imaging;
using Windows.Media.Ocr;

using NLog;
using SkiaSharp;

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
            if (args.Length != 2)
            {
                Logger.Error($"{nameof(Demo)} <languageTag> <image file path>");
                return;
            }

            // Setup ocr engine
            var languageTag = args[0];
            if (!TrySetupOcrEngine(languageTag, out var ocrEngine))
                return;

            // Read image binary
            var path = args[1];
            var image = GetImageBinary(path);
            if (image == null)
                return;

            // Convert to SoftwareBitmap
            var bitmap = ConvertToSoftwareBitmap(image).Result;
            if (bitmap == null)
                return;

            // Run ocr and display result
            RunOcr(ocrEngine, bitmap, image);
        }

        #region Helpers

        private static async Task<SoftwareBitmap> ConvertToSoftwareBitmap(byte[] image)
        {
            Stopwatch.Restart();

            try
            {
                using (var memoryStream = new MemoryStream(image))
                using (var randomAccessStream = memoryStream.AsRandomAccessStream())
                {
                    var decoder = await BitmapDecoder.CreateAsync(randomAccessStream);
                    return await decoder.GetSoftwareBitmapAsync();
                }
            }
            catch (Exception e)
            {
                Logger.Error(e, "Failed to convert to SoftwareBitmap");
                return null;
            }
            finally
            {
                var ms = Stopwatch.ElapsedMilliseconds;
                Logger.Info($"{nameof(ConvertToSoftwareBitmap)}: {ms} ms");
            }
        }

        private static byte[] GetImageBinary(string path)
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

        private static void RunOcr(OcrEngine ocrEngine, SoftwareBitmap bitmap, byte[] image)
        {
            Stopwatch.Restart();

            try
            {
                var task = ocrEngine.RecognizeAsync(bitmap).AsTask();
                task.Wait();
                Stopwatch.Stop();

                var result = task.Result;
                var angle = result.TextAngle?.ToString() ?? "";
                Logger.Info($"Angle: {angle}");

                using var paint = new SKPaint();
                paint.Color = new SKColor(255, 0, 0);
                paint.Style = SKPaintStyle.Stroke;
                using var skBitmap = SKBitmap.Decode(image);
                using var skCanvas = new SKCanvas(skBitmap);
                var lines = result.Lines;
                foreach (var line in lines)
                {
                    Logger.Info($"{line.Text}");
                    foreach (var word in line.Words)
                    {
                        var left = word.BoundingRect.X;
                        var top = word.BoundingRect.Y;
                        var right = word.BoundingRect.Right;
                        var bottom = word.BoundingRect.Bottom;
                        Logger.Info($"\t{word.Text}\t{left}\t{top}\t{right}\t{bottom}");
                        skCanvas.DrawRect(new SKRect((float)left, (float)top, (float)right, (float)bottom), paint);
                    }
                }

                using var fileStream = new FileStream("result.png", FileMode.Create, FileAccess.Write, FileShare.Write);
                skBitmap.Encode(fileStream, SKEncodedImageFormat.Png, 100);
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

        private static bool TrySetupOcrEngine(string languageTag, out OcrEngine engine)
        {
            Stopwatch.Restart();

            engine = null;

            try
            {
                var language = new Windows.Globalization.Language(languageTag);

                // https://docs.microsoft.com/en-us/uwp/api/windows.media.ocr.ocrengine.trycreatefromlanguage?view=winrt-22621
                engine = OcrEngine.TryCreateFromLanguage(language);
                if (engine == null)
                {
                    Logger.Error("Failed to create ocr engine because it could be lack of language pack.");
                    return false;
                }

                return true;
            }
            catch (Exception e)
            {
                Logger.Error(e, "Failed to create ocr engine");
                return false;
            }
            finally
            {
                var ms = Stopwatch.ElapsedMilliseconds;
                Logger.Info($"{nameof(TrySetupOcrEngine)}: {ms} ms");
            }
        }

        #endregion

        #endregion

    }

}

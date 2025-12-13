using System.Diagnostics;

using NLog;
using SkiaSharp;
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
            RunOcr(engine, image, path);
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

        private static void RunOcr(Engine? engine, TesseractOCR.Pix.Image image, string path)
        {
            Stopwatch.Restart();

            try
            {
                using var page = engine.Process(image);
                Logger.Info("Mean confidence: {0}", page.MeanConfidence);
                Logger.Info("Text: \r\n{0}", page.Text);

                using var paintRect = new SKPaint();
                paintRect.Color = new SKColor(255, 0, 0);
                paintRect.Style = SKPaintStyle.Stroke;

                using var paintText = new SKPaint();
                using var typeface = SKTypeface.FromFile( @"NotoSansJP-Light.ttf" );
                paintText.Color = new SKColor(255, 0, 0);
                paintText.Typeface = typeface;

                var binary = File.ReadAllBytes(path);
                using var skSourceImage = SKBitmap.Decode(binary);
                int w = skSourceImage.Width;
                int h = skSourceImage.Height;
                using var skBitmap = new SKBitmap(w, h * 2, skSourceImage.ColorType, skSourceImage.AlphaType);
                using var skCanvas = new SKCanvas(skBitmap);

                skCanvas.Clear(SKColors.White);
                skCanvas.DrawBitmap(skSourceImage, 0, 0);
                            
                foreach (var layout in page.Layout)
                {
                    foreach (var paragraph in layout.Paragraphs)
                    {
                        foreach (var textLine in paragraph.TextLines)
                        {
                            var text = textLine.Text.Trim();
                            if (string.IsNullOrWhiteSpace(text))
                                continue;
                            if (!textLine.BoundingBox.HasValue)
                                continue;

                            var r = textLine.BoundingBox.Value;
                            skCanvas.DrawRect(new SKRect((float)r.X1, (float)r.Y1, (float)r.X2, (float)r.Y2), paintRect);
                            skCanvas.DrawText(text, new SKPoint((float)r.X1, (float)(r.Y1 + h)), paintText);
                        }
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
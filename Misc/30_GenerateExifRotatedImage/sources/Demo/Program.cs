using System;
using System.Drawing;
using System.Drawing.Imaging;

using ExifLibrary;
using NLog;

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
                Logger.Error("Usage: Demo.exe </path/to/image/file>");
                return;
            }

            var inputPath = args[0];
            using (var original = new Bitmap(inputPath))
            {
                var noExifPath = "output_no_exif.jpg";
                original.Save(noExifPath, ImageFormat.Jpeg);
                Logger.Info($"Saved: {noExifPath}");

                for (var orientation = 1; orientation <= 8; orientation++)
                {
                    using (var transformed = ApplyInverseOrientation(original, orientation, out var rotatedPostfix))
                    {
                        var tempPath = $"temp_{orientation}_{rotatedPostfix}.jpg";
                        transformed.Save(tempPath, ImageFormat.Jpeg);

                        var finalPath = $"output_orientation_{orientation}_{rotatedPostfix}.jpg";
                        InjectOrientationTag(tempPath, finalPath, orientation);
                        Logger.Info($"Saved: {finalPath}");

                        System.IO.File.Delete(tempPath);
                    }
                }
            }
        }

        #region Helpers
       
        private static Bitmap ApplyInverseOrientation(Bitmap bmp, int orientation, out string rotatedPostfix)
        {
            rotatedPostfix = "";

            var copy = (Bitmap)bmp.Clone();
            switch (orientation)
            {
                case 1:
                    rotatedPostfix = "RotateNone";
                    break;
                case 2:
                    copy.RotateFlip(RotateFlipType.RotateNoneFlipX);
                    rotatedPostfix = $"{nameof(RotateFlipType.RotateNoneFlipX)}";
                    break;
                case 3:
                    copy.RotateFlip(RotateFlipType.Rotate180FlipNone);
                    rotatedPostfix = $"{nameof(RotateFlipType.Rotate180FlipNone)}";
                    break;
                case 4:
                    copy.RotateFlip(RotateFlipType.RotateNoneFlipY);
                    rotatedPostfix = $"{nameof(RotateFlipType.RotateNoneFlipY)}";
                    break;
                case 5:
                    copy.RotateFlip(RotateFlipType.Rotate90FlipX);
                    rotatedPostfix = $"{nameof(RotateFlipType.Rotate90FlipX)}";
                    break;
                case 6:
                    copy.RotateFlip(RotateFlipType.Rotate270FlipNone);
                    rotatedPostfix = $"{nameof(RotateFlipType.Rotate270FlipNone)}";
                    break;
                case 7:
                    copy.RotateFlip(RotateFlipType.Rotate270FlipX);
                    rotatedPostfix = $"{nameof(RotateFlipType.Rotate270FlipX)}";
                    break;
                case 8:
                    copy.RotateFlip(RotateFlipType.Rotate90FlipNone);
                    rotatedPostfix = $"{nameof(RotateFlipType.Rotate90FlipNone)}";
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(orientation));
            }

            var dpiX = bmp.HorizontalResolution;
            var dpiY = bmp.VerticalResolution;
            copy.SetResolution(dpiX, dpiY);

            return copy;
        }

        private static void InjectOrientationTag(string inputPath, string outputPath, int orientation)
        {
            var file = ImageFile.FromFile(inputPath);
            file.Properties.Add(ExifTag.Orientation, (ushort)orientation);
            file.Save(outputPath);
        }

        #endregion

        #endregion

    }

}
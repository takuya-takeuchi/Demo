using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Graphics.Imaging;
using Windows.Media;
using Windows.Storage;

namespace YoloTest
{
    class Program
    {
        static void Main(string[] args)
        {
            // Create model 
            var modelPath = Path.GetFullPath(args[0]);
            var modelFile = StorageFile.GetFileFromPathAsync(modelPath).GetAwaiter().GetResult();
            var model = Model.CreateFromStreamAsync(modelFile).Result;

            // Create VideoFrame from SoftwareBitmap
            var imagePath = Path.GetFullPath(args[1]);
            var file = StorageFile.GetFileFromPathAsync(imagePath).GetAwaiter().GetResult();
            using var stream = file.OpenAsync(FileAccessMode.Read).GetAwaiter().GetResult();
            var decoder = BitmapDecoder.CreateAsync(stream).GetAwaiter().GetResult();
            var softwareBitmap = decoder.GetSoftwareBitmapAsync().GetAwaiter().GetResult();
            var inputImage = VideoFrame.CreateWithSoftwareBitmap(softwareBitmap);

            // Create input data
            var input = new Input
            {
                image = inputImage
            };

            // Inference
            var output = model.EvaluateAsync(input).Result;
            var boxes = new YoloParser().ParseOutputs(output.grid.GetAsVectorView().ToArray(), 0.7f);

            // Get raw byte array
            var width = inputImage.SoftwareBitmap.PixelWidth;
            var height = inputImage.SoftwareBitmap.PixelHeight;
            var imageBytes = new byte[4 * width * height];
            softwareBitmap.CopyToBuffer(imageBytes.AsBuffer());

            // Draw source image to result image as background
            using var result = new Bitmap(width, height);
            using var g = Graphics.FromImage(result);
            var data = result.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
            Marshal.Copy(imageBytes, 0, data.Scan0, imageBytes.Length);
            result.UnlockBits(data);

            // Draw bounding box
            foreach (var box in boxes)
            {
                var x = width * box.X / 416;
                var y = height * box.Y / 416;
                var w = width * box.Width / 416;
                var h = height * box.Height / 416;

                g.DrawRectangle(Pens.Red, x, y, w, h);
                g.DrawString(box.Label, SystemFonts.MessageBoxFont, Brushes.Red, x + 10, y + 10);
                g.DrawString(box.Confidence.ToString(), SystemFonts.MessageBoxFont, Brushes.Red, x + 10, y + 30);
            }

            result.Save("result.jpg", ImageFormat.Jpeg);
        }
    }
}

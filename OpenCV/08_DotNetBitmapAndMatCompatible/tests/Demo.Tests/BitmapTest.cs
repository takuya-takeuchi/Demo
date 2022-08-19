using System.Drawing;
using System.Drawing.Imaging;

using OpenCvSharp;

namespace Demo.Tests;

public class BitmapTest
{

    [Theory]
    [InlineData(20, 3, PixelFormat.Format24bppRgb)]
    [InlineData(21, 3, PixelFormat.Format24bppRgb)]
    [InlineData(22, 3, PixelFormat.Format24bppRgb)]
    [InlineData(23, 3, PixelFormat.Format24bppRgb)]
    [InlineData(20, 1, PixelFormat.Format8bppIndexed)]
    [InlineData(21, 1, PixelFormat.Format8bppIndexed)]
    [InlineData(22, 1, PixelFormat.Format8bppIndexed)]
    [InlineData(23, 1, PixelFormat.Format8bppIndexed)]
    public void CheckStride(int width, int channels, PixelFormat pixelFormat)
    {
        using var bitmap = new Bitmap(width, width, pixelFormat);
        var bitmapData = bitmap.LockBits(new Rectangle(0, 0, width, width), ImageLockMode.ReadOnly, pixelFormat);
        var stride = Lib.RoundUp(width * channels, 4);
        Assert.Equal(stride, bitmapData.Stride);
    }

    [Fact]
    public void CheckRgbOrder()
    {
        // R is 255, G is 127, B is 39
        const byte r = 255;
        const byte g = 127;
        const byte b = 39;
        var path = Path.Combine("testdata", "orange.png");
        using var bitmap = Image.FromFile(path) as Bitmap;
        using var mat = Cv2.ImRead(path);

        var width = mat.Width;
        var height = mat.Height;
        var channels = mat.Channels();
        var pixelCount = width * height;

        var bitmapPixelDataWithPadding = Lib.GetPixelDataWithPadding(bitmap);
        var bitmapPixelDataWithoutPadding = Lib.GetPixelDataWithoutPadding(bitmap);
        var matPixelDataWithoutPadding = Lib.GetPixelDataWithoutPadding(mat);

        Assert.Equal(matPixelDataWithoutPadding.Length, bitmapPixelDataWithoutPadding.Length);
        Assert.NotEqual(matPixelDataWithoutPadding.Length, bitmapPixelDataWithPadding.Length);

        for (int i = 0; i < pixelCount; i++)
        {
            // In Windows Bitmap and OpenCV mat, Pixel order is BGR rather than RGB
            var index = i * channels;
            var bitmapPixel1 = bitmapPixelDataWithoutPadding[index + 0];
            var bitmapPixel2 = bitmapPixelDataWithoutPadding[index + 1];
            var bitmapPixel3 = bitmapPixelDataWithoutPadding[index + 2];
            var matPixel1 = matPixelDataWithoutPadding[index + 0];
            var matPixel2 = matPixelDataWithoutPadding[index + 1];
            var matPixel3 = matPixelDataWithoutPadding[index + 2];

            Assert.Equal(bitmapPixel1, matPixel1);
            Assert.Equal(bitmapPixel2, matPixel2);
            Assert.Equal(bitmapPixel3, matPixel3);

            Assert.Equal(b, matPixel1);
            Assert.Equal(g, matPixel2);
            Assert.Equal(r, matPixel3);
        }
    }

}
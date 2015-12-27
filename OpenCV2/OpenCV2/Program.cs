using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Threading.Tasks;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using OpenCV2Lib;

namespace OpenCV2
{
    class Program
    {
        static void Main(string[] args)
        {
            // 画像は https://commons.wikimedia.org/wiki/File:Landscape_of_Shadegan.jpg
            var bitmap = (Bitmap)Image.FromFile("images\\Landscape_of_Shadegan.jpg");

            var loop = int.Parse(args[0]);

            var sw = new Stopwatch();

            Console.WriteLine("Sequential");

            sw.Reset();
            sw.Start();

            for (var i = 0; i < loop; i++)
            {
                //using (var cloned = (Bitmap)bitmap.Clone())
                {
                    Sequential(bitmap);
                }
            }

            sw.Stop();
            Console.WriteLine("Total: {0}ms, {1} ticks", sw.ElapsedMilliseconds, sw.ElapsedTicks);
            Console.WriteLine();

            Console.WriteLine("OpenCv");

            sw.Reset();
            sw.Start();

            for (var i = 0; i < loop; i++)
            {
                //using (var cloned = (Bitmap)bitmap.Clone())
                {
                    Optimized(bitmap);
                }
            }

            sw.Stop();
            Console.WriteLine("Total: {0}ms, {1} ticks", sw.ElapsedMilliseconds, sw.ElapsedTicks);
            Console.WriteLine();

            Console.WriteLine("Optimized");

            sw.Reset();
            sw.Start();
            for (var i = 0; i < loop; i++)
            {
                //using (var cloned = (Bitmap)bitmap.Clone())
                {
                    OpenCv(bitmap, 0.5d);
                }
            }

            sw.Stop();
            Console.WriteLine("Total: {0}ms, {1} ticks", sw.ElapsedMilliseconds, sw.ElapsedTicks);
            Console.WriteLine();

            Console.WriteLine("Optimized (Parallel)");

            sw.Reset();
            sw.Start();
            for (var i = 0; i < loop; i++)
            {
                //using (var cloned = (Bitmap)bitmap.Clone())
                {
                    OpenCv(bitmap, 0.5d);
                }
            }

            sw.Stop();
            Console.WriteLine("Total: {0}ms, {1} ticks", sw.ElapsedMilliseconds, sw.ElapsedTicks);
            Console.WriteLine();

            Console.WriteLine("Optimized (C++/CLI)");

            sw.Reset();
            sw.Start();
            for (var i = 0; i < loop; i++)
            {
                //using (var cloned = (Bitmap)bitmap.Clone())
                {
                    Test.Optimized(bitmap);
                }
            }

            sw.Stop();
            Console.WriteLine("Total: {0}ms, {1} ticks", sw.ElapsedMilliseconds, sw.ElapsedTicks);
            Console.WriteLine();
        }

        private static unsafe void Grayscale(byte* pBitmap, byte* oBitmap, int width, int stride, int height, int bitCount)
        {
            var ptr = pBitmap;

            for (var y = 0; y < height; y++)
            {
                var currentY = y * stride;
                var currentGrayY = y * width;
                for (var x = 0; x < width; x++)
                {
                    var currentX = x * bitCount;

                    var b = ptr[currentY + currentX + 0];
                    var g = ptr[currentY + currentX + 1];
                    var r = ptr[currentY + currentX + 2];

                    var value = (byte)(0.299 * r + 0.587 * g + 0.114 * b);
                    oBitmap[currentGrayY + x] = value;
                }
            }
        }

        private static unsafe void HalfScale(byte* pBitmap, byte* oBitmap, int width, int height)
        {
            var ptr = pBitmap;

            width /= 2;
            height /= 2;
            for (var y = 0; y < height; y++)
            {
                var currentY = y * 2 * width * 2;
                var currentHalfY = y * width;
                for (var x = 0; x < width; x++)
                {
                    var value = ptr[currentY + x * 2];
                    oBitmap[currentHalfY + x] = value;
                }
            }
        }

        private static unsafe void Equalizer(byte* pBitmap, int width, int stride, int height)
        {
            var ptr = pBitmap;

            var histogram = new byte[256];
            var accumulated = new double[256];
            fixed (byte* h = &histogram[0])
            fixed (double* a = &accumulated[0])
            {
                // ヒストグラム計算
                for (var y = 0; y < height; y++)
                {
                    var currentY = y * stride;
                    for (var x = 0; x < width; x++)
                    {
                        var value = ptr[currentY + x];
                        h[value]++;
                    }
                }

                // 累積値計算
                var total = 0d;
                for (var i = 0; i < 256; i++)
                {
                    total += h[i];
                    a[i] = total;
                }

                for (var i = 0; i < 256; i++)
                {
                    a[i] = (a[i] * 255) / total;
                }

                // 補正
                for (var y = 0; y < height; y++)
                {
                    var currentY = y * stride;
                    for (var x = 0; x < width; x++)
                    {
                        var value = ptr[currentY + x];
                        ptr[currentY + x] = (byte)a[value];
                    }
                }
            }
        }

        private static unsafe void Sequential(Bitmap bitmap)
        {
            var format = bitmap.PixelFormat;
            var rectangle = new Rectangle(Point.Empty, bitmap.Size);
            var bitmapData = bitmap.LockBits(rectangle, ImageLockMode.ReadOnly, format);
            var pBitmap = (byte*)bitmapData.Scan0;

            using (var gray = new Bitmap(rectangle.Width, rectangle.Height, PixelFormat.Format8bppIndexed))
            using (var output = new Bitmap(rectangle.Width / 2, rectangle.Height / 2, PixelFormat.Format8bppIndexed))
            {
                var rectangleGray = new Rectangle(Point.Empty, gray.Size);
                var bitmapGrayData = gray.LockBits(rectangleGray, ImageLockMode.ReadWrite, PixelFormat.Format8bppIndexed);
                var gBitmap = (byte*)bitmapGrayData.Scan0;

                var rectangleHalf = new Rectangle(Point.Empty, output.Size);
                var bitmapHalfData = output.LockBits(rectangleHalf, ImageLockMode.ReadWrite, PixelFormat.Format8bppIndexed);
                var oBitmap = (byte*)bitmapHalfData.Scan0;

                Grayscale(pBitmap, gBitmap, bitmapData.Width, bitmapData.Stride, bitmapData.Height, 3);
                HalfScale(gBitmap, oBitmap, bitmapGrayData.Width, bitmapGrayData.Height);
                Equalizer(oBitmap, bitmapHalfData.Width, bitmapHalfData.Stride, bitmapHalfData.Height);

                output.UnlockBits(bitmapHalfData);
                gray.UnlockBits(bitmapGrayData);
                bitmap.UnlockBits(bitmapData);

#if DEBUG
                CorrectColorPalette(gray);
                CorrectColorPalette(output);
#endif
            }
        }

        private static unsafe void Optimized(byte* pBitmap, byte* oBitmap, int width, int stride, int height, int bitCount, bool parallel)
        {
            var ptr = pBitmap;

            var histogram = new byte[256];
            var accumulated = new double[256];
            fixed (byte* h = &histogram[0])
            fixed (double* a = &accumulated[0])
            {
                var pH = h;
                var pA = a;

                width /= 2;
                height /= 2;
                if (parallel)
                {
                    Parallel.For(0, height, y =>
                    {
                        var currentY = y * 2 * stride;
                        var currentOutputY = y * width;
                        for (var x = 0; x < width; x++)
                        {
                            var currentX = x * 2 * bitCount;

                            // グレイスケール化
                            var b = ptr[currentY + currentX + 0];
                            var g = ptr[currentY + currentX + 1];
                            var r = ptr[currentY + currentX + 2];

                            var value = (byte)(0.299 * r + 0.587 * g + 0.114 * b);
                            oBitmap[currentOutputY + x] = value;

                            // ヒストグラム
                            pH[value]++;
                        }
                    });
                }
                else
                {
                    for (var y = 0; y < height; y++)
                    {
                        var currentY = y * 2 * stride;
                        var currentOutputY = y * width;
                        for (var x = 0; x < width; x++)
                        {
                            var currentX = x * 2 * bitCount;

                            // グレイスケール化
                            var b = ptr[currentY + currentX + 0];
                            var g = ptr[currentY + currentX + 1];
                            var r = ptr[currentY + currentX + 2];

                            var value = (byte)(0.299 * r + 0.587 * g + 0.114 * b);
                            oBitmap[currentOutputY + x] = value;

                            // ヒストグラム
                            h[value]++;
                        }
                    }
                }

                // 累積値計算
                var total = 0d;
                for (var i = 0; i < 256; i++)
                {
                    total += h[i];
                    a[i] = total;
                }

                for (var i = 0; i < 256; i++)
                {
                    a[i] = (a[i] * 255) / total;
                }

                // 補正
                if (parallel)
                {
                    Parallel.For(0, height, y =>
                    {
                        var currentY = y * width;
                        for (var x = 0; x < width; x++)
                        {
                            var value = oBitmap[currentY + x];
                            oBitmap[currentY + x] = (byte)pA[value];
                        }
                    });
                }
                else
                {
                    for (var y = 0; y < height; y++)
                    {
                        var currentY = y * width;
                        for (var x = 0; x < width; x++)
                        {
                            var value = oBitmap[currentY + x];
                            oBitmap[currentY + x] = (byte)a[value];
                        }
                    } 
                }
            }
        }

        private static unsafe void Optimized(Bitmap bitmap, bool parallel = false)
        {
            var format = bitmap.PixelFormat;
            var rectangle = new Rectangle(Point.Empty, bitmap.Size);
            var bitmapData = bitmap.LockBits(rectangle, ImageLockMode.ReadOnly, format);
            var pBitmap = (byte*)bitmapData.Scan0;

            using (var gray = new Bitmap(rectangle.Width / 2, rectangle.Height / 2, PixelFormat.Format8bppIndexed))
            {
                var rectangleGray = new Rectangle(Point.Empty, gray.Size);
                var bitmapGrayData = gray.LockBits(rectangleGray, ImageLockMode.ReadWrite, PixelFormat.Format8bppIndexed);
                var gBitmap = (byte*)bitmapGrayData.Scan0;

                Optimized(pBitmap, gBitmap, bitmapData.Width, bitmapData.Stride, bitmapData.Height, 3, parallel);

                gray.UnlockBits(bitmapGrayData);
                bitmap.UnlockBits(bitmapData);

#if DEBUG
                CorrectColorPalette(gray);
#endif
            }
        }

        private static void OpenCv(Bitmap bitmap, double scale)
        {
            using (var img = bitmap.ToIplImage())
            using (var smallImg = new IplImage(new CvSize((int)(img.Width * scale), (int)(img.Height * scale)), BitDepth.U8, 1))
            {
                using (var gray = new IplImage(img.Size, BitDepth.U8, 1))
                {
                    Cv.CvtColor(img, gray, ColorConversion.BgrToGray);
                    Cv.Resize(gray, smallImg, Interpolation.NearestNeighbor);
                    Cv.EqualizeHist(smallImg, smallImg);
                }
            }
        }

        private static void CorrectColorPalette(Bitmap bitmap)
        {
            // パレットの更新
            var palette = new Color[256];
            for (int i = 0, length = palette.Length; i < length; i++)
            {
                palette[i] = Color.FromArgb((byte)i, (byte)i, (byte)i);
            }
            var tmp = bitmap.Palette;
            for (int i = 0, length = tmp.Entries.Length; i < length; i++)
            {
                tmp.Entries[i] = palette[i];
            }
            bitmap.Palette = tmp;
        }

    }
}

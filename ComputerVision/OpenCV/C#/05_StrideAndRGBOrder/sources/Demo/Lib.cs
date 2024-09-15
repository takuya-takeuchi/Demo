using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;

using OpenCvSharp;

namespace Demo
{
    
    public class Lib
    {

        public static int RoundUp(int width, int factor)
        {
            return width + (factor - width % factor) % factor;
        }

        public static byte[] GetPixelDataWithoutPadding(Mat mat)
        {
            byte[] result = null;
            var width = mat.Width;
            var height = mat.Height;
            
            var data = mat.Data;
            var channels = mat.Channels();

            var srcStride = (int)mat.Step();
            var dstStride = width * channels;

            result = new byte[dstStride * height];
            for (var h = 0; h < height; h++)
                Marshal.Copy(IntPtr.Add(data, srcStride * h), result, h * dstStride, dstStride);

            return result;
        }

        public static byte[] GetPixelDataWithoutPadding(Bitmap bitmap)
        {
            byte[] result = null;
            var width = bitmap.Width;
            var height = bitmap.Height;

            BitmapData bitmapData = null;
            try
            {
                bitmapData = bitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.ReadOnly, bitmap.PixelFormat);

                var scan0 = bitmapData.Scan0;
                var channels = bitmap.PixelFormat == PixelFormat.Format24bppRgb ? 3 : 4;

                var srcStride = bitmapData.Stride;
                var dstStride = width * channels;

                result = new byte[dstStride * height];
                for (var h = 0; h < height; h++)
                    Marshal.Copy(IntPtr.Add(scan0, srcStride * h), result, h * dstStride, dstStride);
            }
            finally
            {
                if (bitmapData != null)
                    bitmap.UnlockBits(bitmapData);
            }

            return result;
        }

        public static byte[] GetPixelDataWithPadding(Bitmap bitmap)
        {
            byte[] result = null;
            var width = bitmap.Width;
            var height = bitmap.Height;

            BitmapData bitmapData = null;
            try
            {
                bitmapData = bitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.ReadOnly, bitmap.PixelFormat);

                var scan0 = bitmapData.Scan0;
                var channels = bitmap.PixelFormat == PixelFormat.Format24bppRgb ? 3 : 4;

                var srcStride = bitmapData.Stride;
                var dstStride = width * channels;

                result = new byte[srcStride * height];
                for (var h = 0; h < height; h++)
                    Marshal.Copy(IntPtr.Add(scan0, srcStride * h), result, h * dstStride, dstStride);
            }
            finally
            {
                if (bitmapData != null)
                    bitmap.UnlockBits(bitmapData);
            }

            return result;
        }

    }

}

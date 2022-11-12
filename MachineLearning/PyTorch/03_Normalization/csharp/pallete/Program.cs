using System.Drawing;

namespace Pallete
{
    class Program
    {
        static void Main(string[] args)
        {
            var index = 0;
            using var bitmap = new Bitmap(16, 16, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            for (var h = 0; h < 16; h++)
            for (var w = 0; w < 16; w++)
            {
                bitmap.SetPixel(w, h, Color.FromArgb(index, index, index));
                index++;
            }
            bitmap.Save("pallete_0-255.png");

            index = 0;
            using var bitmap2 = new Bitmap(16, 16, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            for (var h = 0; h < 16; h++)
            for (var w = 0; w < 16; w++)
            {
                bitmap2.SetPixel(w, h, Color.FromArgb(index, index, index));
                if (w % 2 == 0)
                    index++;
            }
            bitmap2.Save("pallete_0-128.png");

            index = 0;
            using var bitmap3 = new Bitmap(16, 16, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            for (var h = 0; h < 16; h++)
            for (var w = 0; w < 16; w++)
            {
                bitmap3.SetPixel(w, h, Color.FromArgb(index, 0, 255 - index));
                index++;
            }
            bitmap3.Save("pallete_blue2red.png");
        }
    }
}

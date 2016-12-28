using System;
using System.Diagnostics;

namespace Image2
{
    class Program
    {
        static void Main(string[] args)
        {
            float h, s, v;
            var sw = new Stopwatch();

            sw.Start();
            for (var r = 0; r <= 255; r++)
                for (var g = 0; g <= 255; g++)
                    for (var b = 0; b <= 255; b++)
                        FromRgb((byte)r, (byte)g, (byte)b, out h, out s, out v);

            sw.Stop();
            Console.WriteLine($"{sw.ElapsedMilliseconds} ms");
        }

        private static void FromRgb(byte r, byte g, byte b, out float h, out float s, out float v)
        {
            var fr = r / 255f;
            var fg = g / 255f;
            var fb = b / 255f;

            var max = Math.Max(r, Math.Max(g, b));
            var min = Math.Min(r, Math.Min(g, b));

            var brightness = max;
            float hue, saturation;
            if (Math.Abs(max - min) < float.Epsilon)
            {
                hue = 0f;
                saturation = 0f;
            }
            else
            {
                float c = max - min;
                if (Math.Abs(max - fr) < float.Epsilon)
                    hue = (fg - fb) / c;
                else if (Math.Abs(max - fg) < float.Epsilon)
                    hue = (fb - fr) / c + 2f;
                else
                    hue = (fr - fg) / c + 4f;

                hue *= 60f;
                if (hue < 0f)
                    hue += 360f;

                saturation = c / max;
            }

            h = hue;
            s = saturation;
            v = brightness;
        }

    }
}

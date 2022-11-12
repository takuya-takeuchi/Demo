using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace Normalize
{

    class Program
    {
        
        static void Main(string[] args)
        {
            var file = args[0];

            var mean = new float[]
            {
                0.485f, 0.456f, 0.406f
            };
            var std = new float[]
            {
                0.229f, 0.224f, 0.225f
            };

            using var bitmap = (Bitmap)Image.FromFile(file);
            var width = bitmap.Width;
            var height = bitmap.Height;
            var tensor = new float[width * height * 3];
            for (var h = 0; h < height; h++)
            for (var w = 0; w < width; w++)
            {
                var color = bitmap.GetPixel(w, h);
                var r = (float)(color.R / 255f);
                var g = (float)(color.G / 255f);
                var b = (float)(color.B / 255f);

                r = (r - mean[2]) / std[2];
                g = (g - mean[1]) / std[1];
                b = (b - mean[0]) / std[0];

                tensor[h * width + w + width * height * 2] = r;
                tensor[h * width + w + width * height * 1] = g;
                tensor[h * width + w + width * height * 0] = b;
            }

            for (var c = 0; c < 3; c++)
            {
                Console.WriteLine($"tensor[{c}]");

                for (var h = 0; h < height; h++)
                {
                    if (h == 0)
                        Console.Write("[");
                    else
                        Console.Write(" ");

                    var values = new List<float>();
                    for (var w = 0; w < width; w++)
                    {
                        var value = tensor[h * width + w + width * height * c];
                        values.Add(value);
                    }

                    var line = string.Join(" ", values.Select(t => t.ToString("F4")));
                    Console.Write($"[{line}]");

                    if (h == height - 1)
                        Console.WriteLine("]");
                    else
                        Console.WriteLine();
                }
            }
        }

    }

}

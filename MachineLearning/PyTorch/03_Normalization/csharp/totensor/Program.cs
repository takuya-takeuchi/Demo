using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace ToTensor
{

    class Program
    {
        
        static void Main(string[] args)
        {
            var file = args[0];

            using var bitmap = (Bitmap)Image.FromFile(file);
            var width = bitmap.Width;
            var height = bitmap.Height;
            var tensor = new float[width * height * 3];
            for (var h = 0; h < height; h++)
            for (var w = 0; w < width; w++)
            {
                var color = bitmap.GetPixel(w, h);
                tensor[h * width + w + width * height * 2] = (float)(color.R / 255f);
                tensor[h * width + w + width * height * 1] = (float)(color.G / 255f);
                tensor[h * width + w + width * height * 0] = (float)(color.B / 255f);
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

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Text;

namespace CNTK4
{
    class Program
    {

        static void Main(string[] args)
        {
            // This program for CIFAR-100 dataset binary version

            var fineLabelNames = "fine_label_names.txt";
            var coarseLabelNames = "coarse_label_names.txt";
            var test = "test.bin";
            var train = "train.bin";

            // format is 
            // <1 x coarse label><1 x fine label><3072 x pixel>
            // ..
            // <1 x coarse label><1 x fine label><3072 x pixel>

            var fineLabels = CreateDictionary(fineLabelNames);
            var coarseLabels = CreateDictionary(coarseLabelNames);

            Output("test.bin", "test", coarseLabels, fineLabels);
            Output("train.bin", "train", coarseLabels, fineLabels);
        }

        private static unsafe void Output(string file, string directory, Dictionary<int, string> coarseLabels, Dictionary<int, string> fineLabels)
        {
            var data = File.ReadAllBytes(file);
            const int imageLength = 3072;
            const int channelCount = 3;
            const int channelLength = 1024;
            const int imageSize = 32; // image is 24bit color. 32 * 32 * 3 = 3072
            const int dataLength = 3072 + 1 + 1;
            if (data.Length % dataLength != 0)
                throw new ArgumentException("Data is not expected format");

            // 出力先のメインディレクトリを作成
            Directory.CreateDirectory(directory);

            var sb = new StringBuilder();
            var sbLabels = new StringBuilder();
            var counts = new Dictionary<string, int>();
            var pixels = new string[imageLength];

            using (var ms = new MemoryStream(data))
            {
                var image = new byte[imageLength];
                var rect = new Rectangle(0, 0, imageSize, imageSize);

                // 1 回確保すればそれで良い
                using (var bmp = new Bitmap(imageSize, imageSize, PixelFormat.Format24bppRgb))
                {
                    while (true)
                    {
                        var coarse = ms.ReadByte();
                        if (coarse == -1)
                            break;

                        var fine = ms.ReadByte();
                        var length = ms.Read(image, 0, imageLength);
                        if (length != imageLength)
                            throw new ArgumentException();

                        var path = string.Format("{0}\\{1}\\{2}", directory, coarseLabels[coarse], fineLabels[fine]);
                        Directory.CreateDirectory(path);

                        var name = 0;
                        if (!counts.TryGetValue(path, out name))
                        {
                            name = 1;
                            counts.Add(path, name);
                        }
                        else
                        {
                            name++;
                            counts[path] = name;
                        }

                        fixed (byte* src = &image[0])
                        {
                            var bitmapData = bmp.LockBits(rect, ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

                            var dst = (byte*)bitmapData.Scan0;
                            for (var index = 0; index < channelLength; index++)
                            {
                                dst[index * channelCount + 2] = src[channelLength * 0 + index];
                                dst[index * channelCount + 1] = src[channelLength * 1 + index];
                                dst[index * channelCount + 0] = src[channelLength * 2 + index];
                            }

                            sb.Append(string.Format("{0}\t", coarse));
                            sb.Append(string.Format("{0}\t", fine));

                            sbLabels.Append(string.Format("{0}\t", coarse));
                            sbLabels.Append(string.Format("{0}\t\n", fine));

                            for (var index = 0; index < imageLength; index++)
                            {
                                var b = dst[index];
                                pixels[index] = b.ToString();
                            }

                            sb.AppendLine(string.Join("\t", pixels));

                            bmp.UnlockBits(bitmapData);

                            // Write bitmap to file
                            bmp.Save(string.Format("{0}\\{1:D10}.bmp", path, name));
                        }
                    }
                }

                File.WriteAllText(string.Format("{0}.txt", directory), sb.ToString());
                File.WriteAllText(string.Format("{0}_Label.txt", directory), sbLabels.ToString());
            }
        }

        private static Dictionary<int, string> CreateDictionary(string path)
        {
            var dictionary = new Dictionary<int, string>();

            var text = File.ReadAllText(path).Replace("\n", "\r\n").Replace("\r\r", "\r");
            var lines = text.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries);
            for (int index = 0, count = lines.Length; index < count; index++)
            {
                dictionary.Add(index, lines[index]);
            }

            return dictionary;
        }

    }
}

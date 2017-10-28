using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NAudio.Wave;

namespace NAudioDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            if (args.Length != 1)
            {
                Console.WriteLine("Usage: NAudioDemo.exe <*.wav>");
                return;
            }

            var path = args[0];
            if (!File.Exists(path))
            {
                Console.WriteLine($"{path} is not found.");
                return;
            }

            using (var wav = new WaveFileReader(path))
            {
                var channels = wav.WaveFormat.Channels;
                var bitsPerSample = wav.WaveFormat.BitsPerSample;
                var sampleRate = wav.WaveFormat.SampleRate;
                var length = (int)(wav.Length / channels);
                var bitLength = bitsPerSample / 8;
                if (channels <= 1)
                {
                    Console.WriteLine("Input file does not contain multiple channels.");
                    return;
                }

                int bytesRead;
                var buffer = new byte[bitLength * sampleRate * channels];

                var buffers = new List<MemoryStream>();
                for (var index = 0; index < channels; index++)
                    buffers.Add(new MemoryStream(length));

                while ((bytesRead = wav.Read(buffer, 0, buffer.Length)) > 0)
                {
                    var offset = 0;
                    while (offset < bytesRead)
                    {
                        for (var n = 0; n < channels; n++)
                        {
                            buffers[n].Write(buffer, offset, bitLength);
                            offset += bitLength;
                        }
                    }
                }

                for (var index = 0; index < channels; index++)
                {
                    using (var writer = new WaveFileWriter($"{index + 1}.wav", new WaveFormat(sampleRate, bitsPerSample, 1)))
                    {
                        var buf = buffers[index].GetBuffer();
                        writer.Write(buf, 0, buf.Length);
                        buffers[index].Dispose();
                    }
                }
            }
        }
    }
}

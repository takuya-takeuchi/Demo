using System;
using System.Diagnostics;

using Demo;
using Google.Protobuf;
using Grpc.Core;

namespace LegacyClient
{

    internal class Program
    {

        private static void Main(string[] args)
        {
            if (!(args.Length == 2 &&
                  int.TryParse(args[0], out var count) &&
                  int.TryParse(args[1], out var size)))
            {
                Console.WriteLine($"{nameof(Program)} <count> <data size>");
                return;
            }

            var targets = new[]
            {
                //new { Address = "localhost", Port = 5001, Ssl = true },
                new { Address = "localhost", Port = 5000, Ssl = false },
            };

            var content = new SendRequest
            {
                Content = ByteString.CopyFrom(new byte[size])
            };

            foreach (var target in targets)
            {
                var credentials = target.Ssl ? ChannelCredentials.SecureSsl : ChannelCredentials.Insecure;
                var channel = new Channel(target.Address, target.Port, credentials);
                var client = new Test.TestClient(channel);

                var stopwatch = new Stopwatch();
                stopwatch.Start();
                for (var i = 0; i < count; i++)
                {
                    var _ = client.Send(content);
                }
                stopwatch.Stop();

                var total = stopwatch.ElapsedMilliseconds;
                var average = total / (double)count;

                Console.WriteLine($"  Total: {total} ms");
                Console.WriteLine($"Average: {average} ms");
            }
        }

    }

}

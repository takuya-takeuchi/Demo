using System;
using System.Diagnostics;
using System.IO;

using Google.Protobuf;
using Grpc.Core;

using Demo;

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
                new { Address = "localhost", Port = 5001, UseSsl = true },
                new { Address = "localhost", Port = 5000, UseSsl = false },
            };

            var content = new SendRequest
            {
                Content = ByteString.CopyFrom(new byte[size])
            };

            foreach (var target in targets)
            {
                Channel channel;
                if (target.UseSsl)
                {
                    var rootCert = File.ReadAllText(Path.Combine("certificates", "ca.crt"));
                    var clientCert = File.ReadAllText(Path.Combine("certificates", "client.crt"));
                    var clientKey = File.ReadAllText(Path.Combine("certificates", "client.key"));

                    var keyPair = new KeyCertificatePair(clientCert, clientKey);
                    var credentials = new SslCredentials(rootCert, keyPair);
                    
                    channel = new Channel(target.Address, target.Port, credentials);
                }
                else
                {
                    channel = new Channel(target.Address, target.Port, ChannelCredentials.Insecure);
                }

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

﻿using System;
using System.Diagnostics;
using System.IO;
using System.Net.Http;
using System.Security.Cryptography.X509Certificates;

using Google.Protobuf;
using Grpc.Net.Client;

using Demo;

namespace Client
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
                new { Url = "https://localhost:5001", UseSsl = true },
                new { Url = "http://localhost:5000", UseSsl = false },
            };

            var content = new SendRequest
            {
                Content = ByteString.CopyFrom(new byte[size])
            };

            foreach (var target in targets)
            {
                GrpcChannel channel;
                if (target.UseSsl)
                {
                    var certificate = X509Certificate2.CreateFromPemFile(Path.Combine("certificates", "client.crt"),
                                                                         Path.Combine("certificates", "client.key"));

                    // Add client cert to the handler
                    var handler = new HttpClientHandler();
                    handler.ClientCertificates.Add(certificate);

                    channel = GrpcChannel.ForAddress(target.Url, new GrpcChannelOptions
                    {
                        HttpHandler = handler
                    });
                }
                else
                {
                    channel = GrpcChannel.ForAddress(target.Url);
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

                Console.WriteLine($" UseSsl: {target.UseSsl}");
                Console.WriteLine($"  Count: {count}");
                Console.WriteLine($" Length: {size}");
                Console.WriteLine($"  Total: {total} ms");
                Console.WriteLine($"Average: {average} ms");
            }
        }

    }

}
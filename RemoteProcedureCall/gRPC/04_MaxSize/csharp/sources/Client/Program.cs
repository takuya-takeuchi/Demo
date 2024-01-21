using System;

using CommandLine;
using Google.Protobuf;
using Grpc.Net.Client;
using NLog;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            Parser.Default.ParseArguments<Options>(args)
                .WithParsed(options =>
                {
                    var channel = GrpcChannel.ForAddress("http://localhost:7000", new GrpcChannelOptions
                    {
                        MaxReceiveMessageSize = options.MaxReceiveMessageLength,
                        MaxSendMessageSize = options.MaxSendMMessageLength
                    });
                    var client = new Example.ExampleClient(channel);

                    try
                    {
                        Logger.Info($"Start {nameof(client.UnaryCall)}");

                        var request = new ExampleRequest
                        {
                            Date = ByteString.CopyFrom(new byte[options.RequestDataSize]),
                            ResponseSize = options.ResponseDataSize
                        };

                        var response = new ExampleResponse
                        {
                            Date = ByteString.CopyFrom(new byte[options.ResponseDataSize]),
                        };

                        Logger.Info($"Request Message size: {request.CalculateSize()}");
                        Logger.Info($"Response Message size: {response.CalculateSize()}");

                        client.UnaryCall(request);

                        Logger.Info($"End {nameof(client.UnaryCall)}");
                    }
                    catch (Exception e)
                    {
                        Logger.Error( $"{nameof(client.UnaryCall)}: {e.Message}");
                    }
                }).WithNotParsed(_ =>
                {
                    Logger.Error("Usage:");
                    Logger.Error("\tServer <send message size> <response message size>");
                });
        }

        #endregion

    }

}
using System;
using System.Collections.Generic;

using CommandLine;
using Grpc.Core;
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
                    var receiveMessageLength = options.MaxReceiveMessageLength;
                    var sendMessageLength = options.MaxSendMessageLength;

                    Logger.Info($"max_receive_message_length: {receiveMessageLength}");
                    Logger.Info($"max_send_message_length: {sendMessageLength}");

                    Logger.Info("Start");

                    // default value of max_receive_message_length is 4194304 
                    var channelOptions = new List<ChannelOption>();
                    if (receiveMessageLength.HasValue)
                        channelOptions.Add(new ChannelOption("grpc.max_receive_message_length", receiveMessageLength.Value));
                    if (sendMessageLength.HasValue)
                        channelOptions.Add(new ChannelOption("grpc.max_send_message_length", sendMessageLength.Value));

                    var server = new Server(channelOptions)
                    {
                        Ports =
                        {
                            new ServerPort("localhost", 7000, ServerCredentials.Insecure)
                        },
                        Services =
                        {
                            Example.BindService(new ExampleService(Logger))
                        }
                    };
                    server.Start();

                    Logger.Info("Press any key to shutdown the server...");
                    Console.ReadKey();

                    Logger.Info("Shutdown");
                    server.ShutdownAsync().Wait();
                }).WithNotParsed(_ =>
                {
                    Logger.Error("Usage:");
                    Logger.Error("\tServer <max length of receive message> <max length of send message>");
                });
        }

        #endregion

    }

}
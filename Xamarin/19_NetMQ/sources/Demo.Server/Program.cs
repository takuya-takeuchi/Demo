using System;
using System.Threading;

using NetMQ;
using NetMQ.Sockets;
using NLog;

namespace Demo.Server
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main()
        {
            var rand = new Random(50);
            using var pubSocket = new PublisherSocket();
            Logger.Info("Publisher socket binding...");
            pubSocket.Options.SendHighWatermark = 1000;
            pubSocket.Bind("tcp://*:12345");

            for (var i = 0; ; i++)
            {
                var randomizedTopic = rand.NextDouble();
                if (randomizedTopic > 0.5)
                {
                    var msg = "TopicA msg-" + i;
                    Logger.Info(@"Sending message : {0}", msg);
                    pubSocket.SendMoreFrame("TopicA").SendFrame(msg);
                }
                else
                {
                    var msg = "TopicB msg-" + i;
                    Logger.Info(@"Sending message : {0}", msg);
                    pubSocket.SendMoreFrame("TopicB").SendFrame(msg);
                }

                Thread.Sleep(500);
            }
        }

        #endregion

    }

}

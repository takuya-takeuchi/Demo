using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Confluent.Kafka;
using NLog;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static async Task Main(string[] args)
        {
            if (args.Length != 3 )
            {
                Logger.Error("Usage: Demo <kafka server> <topic name> <message>");
                return;
            }

            var server = args[0];
            var topic = args[1];
            var message = args[2];

            var config = new ProducerConfig
            {
                BootstrapServers = server
            };

            using (var p = new ProducerBuilder<Null, string>(config).Build())
            {
                try
                {
                    var dr = await p.ProduceAsync(topic, new Message<Null, string> { Value=message });
                    Logger.Info($"Delivered '{dr.Value}' to '{dr.TopicPartitionOffset}'");
                }
                catch (ProduceException<Null, string> e)
                {
                    Logger.Error($"Delivery failed: {e.Error.Reason}");
                }
            }
        }

        #endregion

    }

}

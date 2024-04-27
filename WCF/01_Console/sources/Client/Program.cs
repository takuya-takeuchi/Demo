using System.ServiceModel;

using NLog;

using Client.Service;

namespace Client
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            var uri = args[0];
            Logger.Info("Start");
            Logger.Info($"Connect to {uri}");

            var endpointAddress = new EndpointAddress(uri);
            var binding = new BasicHttpBinding
            {
                Security =
                {
                    Mode = BasicHttpSecurityMode.None,
                    Transport =
                    {
                        ClientCredentialType = HttpClientCredentialType.None
                    }
                }
            };

            var channel = ChannelFactory<IGreetingService>.CreateChannel(binding, endpointAddress);
            var reply = channel.Greet(new Greeting(){Message = "Hello, service"});

            Logger.Info($"Server says '{reply.Message}'");
        }

        #endregion

    }

}

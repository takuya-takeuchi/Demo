using System;
using System.ServiceModel;
using System.Text;
using System.Threading.Tasks;

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

        private static async Task Main(string[] args)
        {
            try
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

                var client = new GreetingServiceClient(binding, endpointAddress);
                var reply = await client.GreetAsync(new Greeting()
                {
                    Message = "Hello, service"
                });

                Logger.Info($"Server says '{reply.Message}'");
            }
            catch (CommunicationException ce)
            {
                var sb = new StringBuilder();
                Exception e = ce;
                while (e != null)
                {
                    sb.Append($"\tMessage: {e.Message}, HResult: 0x{e.HResult:X}");
                    e = e.InnerException;
                    if (e != null)
                        sb.AppendLine();
                }
                Logger.Error($"Error{Environment.NewLine}{sb}");
            }
            catch (Exception e)
            {
                Logger.Error(e, "Error");
            }

            Console.ReadLine();
        }

        #endregion

    }

}
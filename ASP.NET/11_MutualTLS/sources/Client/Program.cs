using System.Net.Security;
using System.Net;
using System.Security.Cryptography.X509Certificates;
using NLog;
using System.Runtime.ConstrainedExecution;

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
                var url = args[0];

                HttpClient client;
                if (args.Length == 3) 
                {
                    var certificatePath = args[1];
                    var certificatePassword = args[2];
                    var handler = new SocketsHttpHandler
                    {
                        MaxConnectionsPerServer = 100,
                        AutomaticDecompression = DecompressionMethods.GZip | DecompressionMethods.Deflate,
                        PooledConnectionLifetime = TimeSpan.FromMinutes(1),
                        ConnectTimeout = TimeSpan.FromSeconds(10),
                        PooledConnectionIdleTimeout = TimeSpan.FromSeconds(10),
                        ResponseDrainTimeout = TimeSpan.FromSeconds(10),
                    };
                    handler.SslOptions = new SslClientAuthenticationOptions()
                    {
                        ClientCertificates = new X509CertificateCollection(),
                    };

                    var cert = new X509Certificate2(certificatePath, certificatePassword);
                    Logger.Info($"Thumbprint: {cert.Thumbprint}");

                    handler.SslOptions.ClientCertificates.Add(cert);
                    
                    client = new HttpClient(handler);
                }
                else
                {
                    Logger.Info($"No certificate");
                    client = new HttpClient();
                }

                var response = await client.GetAsync(url);
                response.EnsureSuccessStatusCode();

                Logger.Info("OK");
            }
            catch (Exception e)
            {
                Logger.Error(e, "Failed to connect");
            }
        }

        #endregion

    }

}

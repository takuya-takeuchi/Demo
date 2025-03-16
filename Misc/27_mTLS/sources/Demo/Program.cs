using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Security.Cryptography.X509Certificates;
using System.Threading.Tasks;
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
            var url = args[0];

            X509Certificate2 certificate;
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                var pfx = args[1];
                var passowrd = args[2];

                if (!File.Exists(pfx))
                {
                    Logger.Error($"{pfx} is not found.");
                    return;
                }

                certificate = new X509Certificate2(pfx, passowrd);
            }
            else
            {
                var cert = args[1];
                var key = args[2];

                if (!File.Exists(cert))
                {
                    Logger.Error($"{cert} is not found.");
                    return;
                }

                if (!File.Exists(key))
                {
                    Logger.Error($"{key} is not found.");
                    return;
                }

                certificate = X509Certificate2.CreateFromPemFile(cert, key);
            }

            HttpClient httpClient = new HttpClient(new HttpClientHandler
            {
                ClientCertificates = { certificate }
            });

            var response = await httpClient.GetAsync(url);
            var responseBody = await response.Content.ReadAsStringAsync();
            Logger.Info($"{responseBody}");
        }

        #endregion

    }

}

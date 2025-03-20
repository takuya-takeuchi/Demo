using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Net.Security;
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
            string  thumbprint = null;

            X509Certificate2 certificate;
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                if (args.Length != 4)
                {
                    Logger.Error($"Demo <url> </path/to/pfx> <password> <thumbprint>");
                    return;
                }

                var pfx = args[1];
                var passowrd = args[2];
                thumbprint = args[3];

                if (!File.Exists(pfx))
                {
                    Logger.Error($"{pfx} is not found.");
                    return;
                }

                certificate = new X509Certificate2(pfx, passowrd);
            }
            else
            {
                if (args.Length != 4)
                {
                    Logger.Error($"Demo <url> </path/to/cert> </path/to/key> <thumbprint>");
                    return;
                }

                var cert = args[1];
                var key = args[2];
                thumbprint = args[3];

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

            var handler = new HttpClientHandler
            {
                ClientCertificates = { certificate }
            };
            handler.ServerCertificateCustomValidationCallback = (httpRequestMessage, cert, chain, sslPolicyErrors) =>
            {
                if (sslPolicyErrors != SslPolicyErrors.None)
                {
                    Logger.Error("SSL error occurred");
                    return false;
                }

                // Get SHA-1 hash value for the X.509v3 certificate as a hexadecimal string
                var actualThumbprint = cert.GetCertHashString();
                Logger.Info($"Thumbprint of server certificate: {actualThumbprint}");
                if (thumbprint.Equals(actualThumbprint, StringComparison.OrdinalIgnoreCase))
                    return true;
                
                Logger.Error("Certificate pinning failed");
                return false;
            };
            HttpClient httpClient = new HttpClient(handler);

            var response = await httpClient.GetAsync(url);
            var responseBody = await response.Content.ReadAsStringAsync();
            Logger.Info($"{responseBody}");
        }

        #endregion

    }

}

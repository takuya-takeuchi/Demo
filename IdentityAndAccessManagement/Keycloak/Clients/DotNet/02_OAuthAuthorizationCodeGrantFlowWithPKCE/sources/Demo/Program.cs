using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;

using NLog;

namespace Demo
{

    internal sealed class Program
    {

        #region P/Invokes

        [DllImport("kernel32.dll", ExactSpelling = true)]
        private static extern IntPtr GetConsoleWindow();

        [DllImport("user32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool SetForegroundWindow(IntPtr hWnd);

        #endregion

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static async Task Main(string[] args)
        {
            Logger.Info("Start");

            var p = new Program();
            await p.DoOAuthAsync("console-pkce", "olAOSRZh04pYnLoyLre3vzsEzSLjNUOx");
        }

        #region Helpers

        private static string Base64UrlEncodeNoPadding(byte[] buffer)
        {
            var base64 = Convert.ToBase64String(buffer);

            // Converts base64 to base64url.
            base64 = base64.Replace("+", "-");
            base64 = base64.Replace("/", "_");
            // Strips padding.
            base64 = base64.Replace("=", "");

            return base64;
        }

        private static void BringConsoleToFront()
        {
            SetForegroundWindow(GetConsoleWindow());
        }

        private async Task DoOAuthAsync(string clientId, string clientSecret)
        {
            var state = GenerateRandomDataBase64url(32);
            var codeVerifier = GenerateRandomDataBase64url(32);
            var codeChallenge = Base64UrlEncodeNoPadding(Sha256Ascii(codeVerifier));
            const string codeChallengeMethod = "S256";

            // Creates a redirect URI using an available port on the loopback address.
            var redirectUri = $"http://{IPAddress.Loopback}:{GetRandomUnusedPort()}/";
            Logger.Info($"Redirect URI: {redirectUri}");

            // Creates an HttpListener to listen for requests on that redirect URI.
            var http = new HttpListener();
            http.Prefixes.Add(redirectUri);
            Logger.Info("Listening...");
            http.Start();

            var requestParameter = new List<string>();
            requestParameter.Add("http://127.0.0.1:8080/realms/dotnet-sample/protocol/openid-connect/auth?");
            requestParameter.Add($"client_id={clientId}");
            requestParameter.Add("response_type=code");
            requestParameter.Add($"state={state}");
            requestParameter.Add($"redirect_uri={Uri.EscapeDataString(redirectUri)}");
            requestParameter.Add($"code_challenge={codeChallenge}");
            requestParameter.Add($"code_challenge_method={codeChallengeMethod}");
            var authorizationRequest = string.Join('&', requestParameter);
            Logger.Info($"AuthorizationRequest URI: {authorizationRequest}");

            // Opens request in the browser.
            var escapedUrl = authorizationRequest.Replace("&", "^&");
            Process.Start(new ProcessStartInfo("cmd", $"/c start {escapedUrl}")
            {
                CreateNoWindow = true,
            });

            // Waits for the OAuth authorization response.
            var context = await http.GetContextAsync();

            // Brings the Console to Focus.
            BringConsoleToFront();

            // Sends an HTTP response to the browser.
            var response = context.Response;
            var responseString = "<html><body>Please return to the app.</body></html>";
            var buffer = Encoding.UTF8.GetBytes(responseString);
            response.ContentLength64 = buffer.Length;
            var responseOutput = response.OutputStream;
            await responseOutput.WriteAsync(buffer, 0, buffer.Length);
            responseOutput.Close();
            http.Stop();
            Logger.Info("HTTP server stopped.");

            // Checks for errors.
            var error = context.Request.QueryString.Get("error");
            if (error is object)
            {
                Logger.Error($"OAuth authorization error: {error}.");
                return;
            }

            var code = context.Request.QueryString.Get("code");
            if (code is null || context.Request.QueryString.Get("state") is null)
            {
                Logger.Error($"Malformed authorization response. {context.Request.QueryString}");
                return;
            }
            
            // extracts the code
            var incomingState = context.Request.QueryString.Get("state");

            // Compares the receieved state to the expected value, to ensure that
            // this app made the request which resulted in authorization.
            if (incomingState != state)
            {
                Logger.Error($"Received request with invalid state ({incomingState})");
                return;
            }
            
            Logger.Info($"Authorization code: {code}");

            // Starts the code exchange at the Token Endpoint.
            await ExchangeCodeForTokensAsync(code, codeVerifier, redirectUri, clientId, clientSecret);
        }

        private static async Task ExchangeCodeForTokensAsync(string code, string codeVerifier, string redirectUri, string clientId, string clientSecret)
        {
            Logger.Info("Exchanging code for tokens...");

            // builds the request
            string tokenRequestUri = "http://127.0.0.1:8080/realms/dotnet-sample/protocol/openid-connect/token";
            var requestParameter = new List<string>();
            requestParameter.Add("grant_type=authorization_code");
            requestParameter.Add($"client_id={clientId}");
            requestParameter.Add($"code={code}");
            requestParameter.Add($"code_verifier={codeVerifier}");
            requestParameter.Add($"redirect_uri={redirectUri}");
            var tokenRequestBody = string.Join('&', requestParameter);

            // sends the request
            HttpWebRequest tokenRequest = (HttpWebRequest)WebRequest.Create(tokenRequestUri);
            tokenRequest.Method = "POST";
            tokenRequest.ContentType = "application/x-www-form-urlencoded";
            var tokenRequestBodyBytes = Encoding.ASCII.GetBytes(tokenRequestBody);
            tokenRequest.ContentLength = tokenRequestBodyBytes.Length;
            using var requestStream = tokenRequest.GetRequestStream();
            await requestStream.WriteAsync(tokenRequestBodyBytes, 0, tokenRequestBodyBytes.Length);

            try
            {
                // gets the response
                WebResponse tokenResponse = await tokenRequest.GetResponseAsync();
                using var reader = new StreamReader(tokenResponse.GetResponseStream());
                // reads response body
                var responseText = await reader.ReadToEndAsync();
                Logger.Info($"{responseText}");
            }
            catch (WebException ex)
            {
                if (ex.Status == WebExceptionStatus.ProtocolError)
                {
                    var response = ex.Response as HttpWebResponse;
                    if (response != null)
                    {
                        Logger.Info($"HTTP: {response.StatusCode}");
                        using var reader = new StreamReader(response.GetResponseStream());
                        // reads response body
                        var responseText = await reader.ReadToEndAsync();
                        Logger.Info(responseText);
                    }
                }
            }
        }

        private static string GenerateRandomDataBase64url(uint length)
        {
            var generator = RandomNumberGenerator.Create();
            var bytes = new byte[length];
            generator.GetBytes(bytes);
            return Base64UrlEncodeNoPadding(bytes);
        }

        private static int GetRandomUnusedPort()
        {
            var listener = new TcpListener(IPAddress.Loopback, 0);
            listener.Start();
            var port = ((IPEndPoint)listener.LocalEndpoint).Port;
            listener.Stop();
            return port;
        }

        private static byte[] Sha256Ascii(string text)
        {
            var bytes = Encoding.ASCII.GetBytes(text);
            using var sha256 = SHA256.Create();
            return sha256.ComputeHash(bytes);
        }

        #endregion

        #endregion

    }

}
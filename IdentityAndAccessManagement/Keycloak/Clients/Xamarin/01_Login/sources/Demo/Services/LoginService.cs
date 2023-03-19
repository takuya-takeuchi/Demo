using System;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Text;
using System.Threading.Tasks;

using Xamarin.Essentials;
using Xamarin.Forms;

using IdentityModel.Client;
using IdentityModel.OidcClient;
using IdentityModel.OidcClient.Browser;
using NLog;

using Demo.Services.Interfaces;

namespace Demo.Services
{

    public sealed class LoginService : ILoginService
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        private readonly Logger _Logger = LogManager.GetCurrentClassLogger();

        private readonly Lazy<HttpClient> _ApiClient = new Lazy<HttpClient>(() => new HttpClient());

        private readonly OidcClient _Client;

        private LoginResult _Result;

        #endregion

        #region Constructors

        public LoginService(ILoggingService loggingService)
        {
            this._LoggingService = loggingService;

            var browser = DependencyService.Get<IBrowser>();
            var options = new OidcClientOptions
            {
                Authority = "http://192.168.11.21:8080/realms/dotnet-sample",
                ClientId = "console-pkce",
                Scope = "openid",
                RedirectUri = "xamarinformsclients://callback",
                PostLogoutRedirectUri = "xamarinformsclients://callback",
                Browser = browser,
                Policy = new Policy
                {
                    Discovery = new DiscoveryPolicy
                    {
                        RequireHttps = false
                    }
                }
            };

            this._Client = new OidcClient(options);
            this._ApiClient.Value.BaseAddress = new Uri("http://192.168.11.21:8080/");
        }

        #endregion

        #region ILoginService Members

        public async Task Login()
        {
            //this.NavigationService.NavigateAsync("ShowLogs");
            var outputText = "";

            try
            {
                //accessToken = await SecureStorage.GetAsync("accessToken");
                string accessToken = null;
                if (accessToken == null)
                {
                    this._Result = await _Client.LoginAsync(new LoginRequest()).ConfigureAwait(true);
                    if (this._Result.IsError)
                    {
                        outputText = this._Result.Error;
                        return;
                    }

                    var sb = new StringBuilder(128);
                    foreach (var claim in this._Result.User.Claims)
                        sb.AppendFormat("{0}: {1}\n", claim.Type, claim.Value);

                    sb.AppendFormat("\n{0}: {1}\n", "refresh token", this._Result?.RefreshToken ?? "none");
                    sb.AppendFormat("\n{0}: {1}\n", "access token", this._Result?.AccessToken);
                    accessToken = this._Result.AccessToken;

                    await SecureStorage.SetAsync("accessToken", accessToken);

                    outputText = sb.ToString();
                }

                //this.IsLoggedIn = true;

                if (this._ApiClient.Value.DefaultRequestHeaders.Authorization == null)
                {
                    this._ApiClient.Value.DefaultRequestHeaders.Authorization = new AuthenticationHeaderValue("Bearer", accessToken ?? "");
                }
            }
            catch (Exception ex)
            {
                outputText = ex.ToString();
            }
        }

        public async Task Logout()
        {
            //SecureStorage.Remove("accessToken");
            try
            {
                await this._Client.LogoutAsync();
            }
            catch (Exception ex)
            {
                throw;
            }
        }

        #endregion

    }

}
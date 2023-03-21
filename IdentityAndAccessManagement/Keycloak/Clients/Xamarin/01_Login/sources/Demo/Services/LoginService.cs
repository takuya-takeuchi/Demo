using System;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Threading.Tasks;

using Xamarin.Essentials;
using Xamarin.Forms;

using IdentityModel.Client;
using IdentityModel.OidcClient;
using IdentityModel.OidcClient.Browser;

using Demo.Services.Interfaces;

namespace Demo.Services
{

    public sealed class LoginService : ILoginService
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        private readonly Lazy<HttpClient> _ApiClient = new Lazy<HttpClient>(() => new HttpClient());

        private readonly OidcClient _Client;

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

        public async Task<LoginResult> Login()
        {
            LoginResult result = null;

            try
            {
                var accessToken = await SecureStorage.GetAsync("accessToken");
                if (accessToken == null)
                {
                    result = await _Client.LoginAsync();
                    if (result.IsError)
                        return result;

                    await SecureStorage.SetAsync("accessToken", result.AccessToken);
                }

                if (this._ApiClient.Value.DefaultRequestHeaders.Authorization == null) 
                    this._ApiClient.Value.DefaultRequestHeaders.Authorization = new AuthenticationHeaderValue("Bearer", accessToken ?? "");
            }
            catch (Exception ex)
            {
                this._LoggingService.Error(ex, null, "Failed to login");
            }

            return result;
        }

        public async Task Logout()
        {
            SecureStorage.Remove("accessToken");

            try
            {
                await this._Client.LogoutAsync();
            }
            catch (Exception ex)
            {
                this._LoggingService.Error(ex, null, "Failed to logout");
            }
        }

        #endregion

    }

}
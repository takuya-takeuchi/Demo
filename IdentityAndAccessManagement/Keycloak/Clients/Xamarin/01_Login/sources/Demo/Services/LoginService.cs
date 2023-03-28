using System;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Threading.Tasks;

using Xamarin.Forms;

using IdentityModel.Client;
using IdentityModel.OidcClient;
using IdentityModel.OidcClient.Browser;

using Demo.Services.Interfaces;
using Demo.Models;

namespace Demo.Services
{

    public sealed class LoginService : ILoginService
    {

        #region Fields

        private readonly IStorageService _StorageService;

        private readonly ILoggingService _LoggingService;

        private readonly Lazy<HttpClient> _ApiClient = new Lazy<HttpClient>(() => new HttpClient());

        private readonly OidcClient _Client;

        #endregion

        #region Constructors

        public LoginService(IStorageService storageService,
                            ILoggingService loggingService)
        {
            this._StorageService = storageService;
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

        public DateTimeOffset? AccessTokenExpiration
        {
            get;
            private set;
        }

        public async Task<AuthenticationResult> Login()
        {
            AuthenticationResult authenticationResult = null;

            try
            {
                this._LoggingService.Info("Restore authentication result from storage");

                authenticationResult = await this._StorageService.GetAuthenticationResult();
                if (!string.IsNullOrWhiteSpace(authenticationResult?.RefreshToken))
                {
                    this._LoggingService.Info("Check Refresh Token is available");

                    var refreshTokenResult = await this._Client.RefreshTokenAsync(authenticationResult.RefreshToken);
                    if (!refreshTokenResult.IsError)
                    {
                        this._LoggingService.Info("Refresh Token is available");
                        authenticationResult = new AuthenticationResult(refreshTokenResult, DateTimeOffset.UtcNow);
                    }
                    else
                    {
                        this._LoggingService.Info("Refresh Token is not available");
                        authenticationResult = null;
                    }
                }

                if (string.IsNullOrWhiteSpace(authenticationResult?.RefreshToken))
                {
                    this._LoggingService.Info("Requires to login");

                    var result = await this._Client.LoginAsync();
                    if (result.IsError)
                        return null;

                    this._LoggingService.Info("Store  authentication result to storage");
                    authenticationResult = new AuthenticationResult(result);
                }

                await this._StorageService.SetAuthenticationResult(authenticationResult);

                this.AccessTokenExpiration = authenticationResult.AccessTokenExpiration;

                if (this._ApiClient.Value.DefaultRequestHeaders.Authorization == null)
                    this._ApiClient.Value.DefaultRequestHeaders.Authorization = new AuthenticationHeaderValue("Bearer", authenticationResult.AccessToken ?? "");
            }
            catch (Exception ex)
            {
                this._LoggingService.Error(ex, null, "Failed to login");
            }

            return authenticationResult;
        }

        public async Task Logout()
        {
            try
            {
                await this._StorageService.ClearAuthenticationResult();

                await this._Client.LogoutAsync();
            }
            catch (Exception ex)
            {
                this._LoggingService.Error(ex, null, "Failed to logout");
            }
        }

        public async Task<AuthenticationResult> RefreshToken()
        {
            AuthenticationResult authenticationResult = null;

            try
            {
                this._LoggingService.Info("Restore authentication result from storage");

                authenticationResult = await this._StorageService.GetAuthenticationResult();
                if (!string.IsNullOrWhiteSpace(authenticationResult?.RefreshToken))
                {
                    this._LoggingService.Info("Check Refresh Token is available");

                    var refreshTokenResult = await this._Client.RefreshTokenAsync(authenticationResult.RefreshToken);
                    if (!refreshTokenResult.IsError)
                    {
                        this._LoggingService.Info("Refresh Token is available");
                        authenticationResult = new AuthenticationResult(refreshTokenResult, DateTimeOffset.UtcNow);
                        await this._StorageService.SetAuthenticationResult(authenticationResult);

                        this.AccessTokenExpiration = authenticationResult.AccessTokenExpiration;
                    }
                    else
                    {
                        this._LoggingService.Info("Refresh Token is not available");
                        authenticationResult = null;
                    }
                }
            }
            catch (Exception ex)
            {
                this._LoggingService.Error(ex, null, "Failed to refresh token");
            }

            return authenticationResult;
        }

        #endregion

    }

}
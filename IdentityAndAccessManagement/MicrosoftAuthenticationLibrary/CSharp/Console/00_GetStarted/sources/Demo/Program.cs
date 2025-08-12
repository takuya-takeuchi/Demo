using System.Threading.Tasks;

using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Configuration.EnvironmentVariables;
using Microsoft.Extensions.Configuration.Json;
using Microsoft.Extensions.Configuration.UserSecrets;
using Microsoft.Identity.Client;
using Microsoft.Identity.Client.Extensions.Msal;
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
            var config = new ConfigurationBuilder()
                .SetBasePath(Directory.GetCurrentDirectory())
                .AddJsonFile(path: "appsettings.json")
                .AddEnvironmentVariables(prefix: "DEMO_")
                .AddUserSecrets<Program>(optional: true)
                .Build();

            var authSettings = config.GetSection("Auth").Get<AuthSettings>();
            if (authSettings == null)
            {
                Logger.Error("Auth setting is missing");
                return;
            }

            // specify app name to avoid conflict other app
            var schemaName = "jp.taktak.msal";

            var clientId = authSettings.ClientId;
            var tenantId = authSettings.TenantId;
            var authority = $"{authSettings.AuthorityInstance}{tenantId}/v2.0";
            var redirectUri = authSettings.RedirectUri;
            var scopes = authSettings.Scopes;

            // create pca
            var pca = PublicClientApplicationBuilder
                .Create(clientId)
                .WithAuthority(authority)
                .WithRedirectUri(redirectUri)
                .Build();

            // persistance token cache
            // Where is MSAL's cahce?
            // - Windows: DPAPI
            // - OSX: KeyChain
            // - Linux: Secret Service API (libsecret / GNOME Keyring etc)
            var storageProps = new StorageCreationPropertiesBuilder(
                    cacheFileName: "msal.cache",
                    cacheDirectory: AppContext.BaseDirectory)
                .WithLinuxKeyring(
                    schemaName: schemaName,
                    collection: "default",
                    secretLabel: "MSAL Token Cache",
                    attribute1: new KeyValuePair<string, string>("Version", "1"),
                    attribute2: new KeyValuePair<string, string>("Product", "MyApp"))
                .WithMacKeyChain(schemaName, "MSAL Token Cache")
                .Build();

            var cacheHelper = await MsalCacheHelper.CreateAsync(storageProps);
            cacheHelper.RegisterCache(pca.UserTokenCache);

            // First, get token silent
            AuthenticationResult? result = null;
            var accounts = await pca.GetAccountsAsync();
            try
            {
                result = await pca.AcquireTokenSilent(scopes, accounts.FirstOrDefault())
                                  .ExecuteAsync();

                Logger.Info("AccessToken or RefreshToken is valid");
            }
            catch (MsalUiRequiredException)
            {
                Logger.Info("MsalUiRequiredException is thrown. You need to proceed authentication");
            }

            if (result == null)
            {
                using var cancellationToken = new CancellationTokenSource(TimeSpan.FromSeconds(30));

                try
                {
                    // Use UI if required (MFA may be used)
                    result = await pca.AcquireTokenInteractive(scopes)
                                      .WithPrompt(Prompt.SelectAccount)
                                      .ExecuteAsync(cancellationToken.Token);
                }
                catch (MsalClientException mce)
                {
                    // Errors caused by client/browser-related issues
                    switch (mce.ErrorCode)
                    {
                        case "authentication_canceled":
                            // User closed or aborted the sign-in flow
                            // Prompt the user to retry or ignore and continue without authentication
                            break;
                        case "browser_error":
                        // Failure to launch system browser or communication issues
                        case "state_mismatch":
                            // Security check failure due to mismatched state parameter
                            // Log and suggest the user retry
                            break;
                        default:
                            // Any other client-side error
                            break;
                    }
                    Logger.Error($"MsalClientException is thrown. ErrorCode: {mce.ErrorCode}");
                }
                catch (MsalServiceException mse)
                {
                    // Errors returned from Azure AD (redirected error from the browser)
                    // ex.ErrorCode may be "access_denied", "invalid_client", etc.
                    // Body/Status may contain an AADSTSxxxx error code with more details
                    var code = mse.ErrorCode;        // Example: "access_denied"
                    var http = mse.StatusCode;       // Example: 400
                    var body = mse.ResponseBody;     // Raw AADSTS error response (should be logged for troubleshooting)

                    // Common examples:
                    // AADSTS500113: No reply address → Redirect URI missing or mismatch
                    // AADSTS700016: Invalid client (incorrect clientId)
                    // AADSTS50158 / 50076: MFA or Conditional Access requirement not met → Retry after satisfying policy
                    Logger.Error($"MsalServiceException is thrown. code: {code}, http: {http}, body: {body}");
                }
                catch (OperationCanceledException oce)
                {
                    // Thrown when the operation is canceled via CancellationToken
                    Logger.Error(oce, $"OperationCanceledException is thrown");
                }
                catch (MsalException me)
                {
                    // General fallback for other MSAL-related exceptions
                    Logger.Error(me, $"MsalException is thrown");
                }
            }

            if (result != null)
            {
                Logger.Info("Authentication is complete");
                
                var remaining = result.ExpiresOn - DateTimeOffset.UtcNow;
                Logger.Info($"Token valid for {remaining.TotalMinutes} more minutes.");
            }
            else
            {
                Logger.Error("Authentication is failed");
            }
        }

        #endregion

    }

}

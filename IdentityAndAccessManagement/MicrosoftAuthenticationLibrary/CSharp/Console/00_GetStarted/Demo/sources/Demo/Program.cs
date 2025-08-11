using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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
            var authority = $"${authSettings.AuthorityInstance}{tenantId}/v2.0";
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
            AuthenticationResult result;
            var accounts = await pca.GetAccountsAsync();
            try
            {
                result = await pca.AcquireTokenSilent(scopes, accounts.FirstOrDefault())
                                  .ExecuteAsync();
            }
            catch (MsalUiRequiredException)
            {
                Logger.Info("MsalUiRequiredException is thrown");

                // Use UI if required (MFA may be used)
                result = await pca.AcquireTokenInteractive(scopes)
                                  .WithPrompt(Prompt.SelectAccount)
                                  .ExecuteAsync();
            }
        }

        #endregion

    }

}

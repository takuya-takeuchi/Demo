using Microsoft.IdentityModel.Protocols;
using Microsoft.IdentityModel.Protocols.OpenIdConnect;
using Microsoft.IdentityModel.Tokens;

namespace Demo.Auth
{

    /// <summary>
    /// Hanko does not publish an OIDC discovery document (/.well-known/openid-configuration),
    /// so the standard OpenIdConnectConfigurationRetriever cannot be used.
    /// Instead, read the JWKS (/.well-known/jwks.json) directly, build an OpenIdConnectConfiguration
    /// that carries only the signing keys, and hand it to the ConfigurationManager.
    ///
    /// The ConfigurationManager caches and refreshes it automatically, so key rotation on the
    /// Hanko side is picked up without any extra work here.
    /// </summary>
    public sealed class HankoJwksRetriever : IConfigurationRetriever<OpenIdConnectConfiguration>
    {

        #region Methods 

        public async Task<OpenIdConnectConfiguration> GetConfigurationAsync(string address, IDocumentRetriever retriever, CancellationToken cancel)
        {
            var json = await retriever.GetDocumentAsync(address, cancel);

            var config = new OpenIdConnectConfiguration { JwksUri = address };
            var keySet = JsonWebKeySet.Create(json);
            config.JsonWebKeySet = keySet;

            foreach (var key in keySet.GetSigningKeys())
                config.SigningKeys.Add(key);

            return config;
        }

        #endregion

    }

}

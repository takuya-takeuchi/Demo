using Microsoft.IdentityModel.Protocols;
using Microsoft.IdentityModel.Protocols.OpenIdConnect;
using Microsoft.IdentityModel.Tokens;

namespace Demo.Auth
{

    /// <summary>
    /// Hanko は OIDC ディスカバリ文書（/.well-known/openid-configuration）を公開していないため、
    /// 標準の OpenIdConnectConfigurationRetriever は使えない。
    /// 代わりに JWKS（/.well-known/jwks.json）を直接読み、署名鍵だけを持つ
    /// OpenIdConnectConfiguration を組み立てて ConfigurationManager に渡す。
    ///
    /// ConfigurationManager 側が自動でキャッシュ・定期更新してくれるので、
    /// Hanko 側で鍵がローテーションされても追従できる。
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

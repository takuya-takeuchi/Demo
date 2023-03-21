using System;

using IdentityModel.OidcClient;
using IdentityModel.OidcClient.Results;

namespace Demo.Models
{

    public sealed class AuthenticationResult
    {

        public AuthenticationResult()
        {
        }

        public AuthenticationResult(LoginResult loginResult)
        {
            this.AccessToken = loginResult.AccessToken;
            this.IdentityToken = loginResult.IdentityToken;
            this.RefreshToken = loginResult.RefreshToken;
            this.AccessTokenExpiration = loginResult.AccessTokenExpiration;
            this.AuthenticationTime = loginResult.AuthenticationTime;
        }

        public AuthenticationResult(RefreshTokenResult refreshTokenResult, DateTimeOffset authenticationTime)
        {
            this.AccessToken = refreshTokenResult.AccessToken;
            this.IdentityToken = refreshTokenResult.IdentityToken;
            this.RefreshToken = refreshTokenResult.RefreshToken;
            this.AccessTokenExpiration = refreshTokenResult.AccessTokenExpiration;
            this.AuthenticationTime = authenticationTime;
        }

        public string AccessToken { get; set; }
        
        public string IdentityToken { get; set; }
        
        public string RefreshToken { get; set; }
        
        public DateTimeOffset AccessTokenExpiration { get; set; }
        
        public DateTimeOffset? AuthenticationTime { get; set; }

    }

}
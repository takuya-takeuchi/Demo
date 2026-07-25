using System;
using System.Security.Cryptography;
using System.Diagnostics;
using System.IdentityModel.Tokens.Jwt;
using System.IO;
using System.Security.Claims;
using System.Text;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Microsoft.IdentityModel.Tokens;

using Demo.Models;
using Demo.Options;

namespace Demo.Services
{

    public sealed class TokenService
    {

        #region Fields

        private readonly JwtOptions _Options;

        private readonly SigningCredentials _SigningCredentials;

        #endregion

        #region Constructors

        public TokenService(IOptions<JwtOptions> options)
        {
            _Options = options.Value;
            var securityKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(_Options.SigningKey));
            _SigningCredentials = new SigningCredentials(securityKey, SecurityAlgorithms.HmacSha256);
        }

        #endregion

        #region Methods

        public TokenResponse CreateTokenResponse(string userId)
        {
            DateTimeOffset now = DateTimeOffset.UtcNow;
            DateTimeOffset accessTokenExpiresAt = now.AddSeconds(_Options.AccessTokenLifetimeSeconds);
            DateTimeOffset refreshTokenExpiresAt = now.AddMinutes(_Options.RefreshTokenLifetimeMinutes);

            var claims = new[]
            {
                new Claim(JwtRegisteredClaimNames.Sub, userId),
                new Claim(JwtRegisteredClaimNames.Jti, Guid.NewGuid().ToString("N")),
                new Claim(JwtRegisteredClaimNames.Iat, now.ToUnixTimeSeconds().ToString(), ClaimValueTypes.Integer64)
            };

            var jwt = new JwtSecurityToken(issuer: _Options.Issuer,
                                           audience: _Options.Audience,
                                           claims: claims,
                                           notBefore: now.UtcDateTime,
                                           expires: accessTokenExpiresAt.UtcDateTime,
                                           signingCredentials: _SigningCredentials);

            string accessToken = new JwtSecurityTokenHandler().WriteToken(jwt);
            string refreshToken = CreateRefreshToken();
            return new TokenResponse(AccessToken: accessToken,
                                     AccessTokenExpiresAt: accessTokenExpiresAt,
                                     RefreshToken: refreshToken,
                                     RefreshTokenExpiresAt: refreshTokenExpiresAt);
        }

        #region Helpers

        private static string CreateRefreshToken()
        {
            byte[] bytes = RandomNumberGenerator.GetBytes(32);
            return Convert.ToBase64String(bytes);
        }

        #endregion

        #endregion

    }

}
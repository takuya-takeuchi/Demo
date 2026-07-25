using System;

namespace Demo.Models
{

    public sealed record TokenResponse(string AccessToken,
                                       DateTimeOffset AccessTokenExpiresAt,
                                       string RefreshToken,
                                       DateTimeOffset RefreshTokenExpiresAt);

}
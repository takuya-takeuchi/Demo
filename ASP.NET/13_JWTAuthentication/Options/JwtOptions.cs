

namespace Demo.Options
{

    public sealed class JwtOptions
    {
        
        public string Issuer { get; init; } = string.Empty;

        public string Audience { get; init; } = string.Empty;

        public string SigningKey { get; init; } = string.Empty;

        public int AccessTokenLifetimeSeconds { get; init; } = 10;

        public int RefreshTokenLifetimeMinutes { get; init; } = 10;

        public int RefreshDelayMilliseconds { get; init; } = 1000;
    }

}
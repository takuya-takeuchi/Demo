using System.Text.Json;

using Microsoft.Extensions.Options;
using StackExchange.Redis;

using Demo.Options;

namespace Demo.Caching
{

    internal sealed class TokenCache
    {

        #region Fields

        private static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web);

        private readonly IDatabase _Database;

        private readonly RedisOptions _Options;

        #endregion

        #region Constructors

        public TokenCache(IConnectionMultiplexer connection,
                          IOptions<RedisOptions> options)
        {
            this._Database = connection.GetDatabase();
            this._Options = options.Value;
        }

        #endregion

        #region Methods

        public async Task<TokenResponse?> GetAsync(CancellationToken cancellationToken)
        {
            RedisValue value = await this._Database.StringGetAsync(this._Options.TokenKey)
                                                   .WaitAsync(cancellationToken);

            if (value.IsNullOrEmpty)
                return null;

            try
            {
                return JsonSerializer.Deserialize<TokenResponse>(value.ToString(), JsonOptions);
            }
            catch (JsonException)
            {
                await this._Database.KeyDeleteAsync(this._Options.TokenKey)
                                    .WaitAsync(cancellationToken);
                return null;
            }
        }

        public async Task SetAsync(TokenResponse token, CancellationToken cancellationToken)
        {
            string json = JsonSerializer.Serialize(token, JsonOptions);
            TimeSpan lifetime = CalculateCacheLifetime(token);
            await this._Database.StringSetAsync(this._Options.TokenKey, json, lifetime).WaitAsync(cancellationToken);
        }

        public async Task<bool> SetIfRefreshTokenMatchesAsync(string expectedRefreshToken,
                                                              TokenResponse replacementToken,
                                                              CancellationToken cancellationToken)
        {
            RedisValue currentValue = await this._Database.StringGetAsync(this._Options.TokenKey)
                                                          .WaitAsync(cancellationToken);

            if (currentValue.IsNullOrEmpty)
                return false;

            TokenResponse? currentToken;

            try
            {
                currentToken = JsonSerializer.Deserialize<TokenResponse>(currentValue.ToString(), JsonOptions);
            }
            catch (JsonException)
            {
                return false;
            }

            if (currentToken is null || !string.Equals(currentToken.RefreshToken, expectedRefreshToken, StringComparison.Ordinal))
            {
                return false;
            }

            string replacementJson = JsonSerializer.Serialize(replacementToken, JsonOptions);
            TimeSpan lifetime = CalculateCacheLifetime(replacementToken);
            ITransaction transaction = this._Database.CreateTransaction();
            transaction.AddCondition(Condition.StringEqual(this._Options.TokenKey, currentValue));
            _ = transaction.StringSetAsync(this._Options.TokenKey, replacementJson, lifetime);

            return await transaction.ExecuteAsync().WaitAsync(cancellationToken);
        }

        public async Task<bool> DeleteIfRefreshTokenMatchesAsync(string expectedRefreshToken,
                                                                 CancellationToken cancellationToken)
        {
            RedisValue currentValue = await this._Database.StringGetAsync(this._Options.TokenKey)
                                                          .WaitAsync(cancellationToken);

            if (currentValue.IsNullOrEmpty)
                return false;

            TokenResponse? currentToken;

            try
            {
                currentToken = JsonSerializer.Deserialize<TokenResponse>(currentValue.ToString(), JsonOptions);
            }
            catch (JsonException)
            {
                return false;
            }

            if (currentToken is null || !string.Equals(currentToken.RefreshToken, expectedRefreshToken, StringComparison.Ordinal))
            {
                return false;
            }

            ITransaction transaction = this._Database.CreateTransaction();

            transaction.AddCondition(Condition.StringEqual(this._Options.TokenKey, currentValue));

            _ = transaction.KeyDeleteAsync(this._Options.TokenKey);

            return await transaction.ExecuteAsync().WaitAsync(cancellationToken);
        }

        #region Helpers

        private static TimeSpan CalculateCacheLifetime(TokenResponse token)
        {
            TimeSpan lifetime = token.RefreshTokenExpiresAt - DateTimeOffset.UtcNow;
            if (lifetime <= TimeSpan.Zero)
                return TimeSpan.FromSeconds(1);
            return lifetime;
        }

        #endregion

        #endregion

    }

}
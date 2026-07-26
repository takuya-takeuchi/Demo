using Microsoft.Extensions.Options;
using StackExchange.Redis;

using Demo.Options;

namespace Demo.Coordination
{

    internal sealed class LeaderLock
    {

        #region Fields

        private readonly IDatabase _Database;

        private readonly RedisOptions _RedisOptions;

        private readonly ClientOptions _ClientOptions;

        #endregion

        #region Constructors

        public LeaderLock(IConnectionMultiplexer connection,
                          IOptions<RedisOptions> redisOptions,
                          IOptions<ClientOptions> clientOptions)
        {
            this._Database =
                connection.GetDatabase();

            this._RedisOptions =
                redisOptions.Value;

            this._ClientOptions =
                clientOptions.Value;
        }

        #endregion

        #region Methods

        public async Task<bool> TryAcquireAsync(
            string owner,
            CancellationToken cancellationToken)
        {
            return await this._Database
                .LockTakeAsync(
                    this._RedisOptions.LeaderLockKey,
                    owner,
                    TimeSpan.FromSeconds(
                        this._ClientOptions.LeaderLockSeconds))
                .WaitAsync(cancellationToken);
        }

        public async Task<bool> RenewAsync(
            string owner,
            CancellationToken cancellationToken)
        {
            return await this._Database
                .LockExtendAsync(
                    this._RedisOptions.LeaderLockKey,
                    owner,
                    TimeSpan.FromSeconds(
                        this._ClientOptions.LeaderLockSeconds))
                .WaitAsync(cancellationToken);
        }

        public async Task<bool> ReleaseAsync(
            string owner,
            CancellationToken cancellationToken)
        {
            return await this._Database
                .LockReleaseAsync(
                    this._RedisOptions.LeaderLockKey,
                    owner)
                .WaitAsync(cancellationToken);
        }

        #endregion

    }

}
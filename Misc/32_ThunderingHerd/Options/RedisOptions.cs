namespace Demo.Options
{

    internal sealed class RedisOptions
    {
        public string Configuration { get; init; } = "localhost:6379";

        public string TokenKey { get; init; } = "thundering-herd:token";

        public string LeaderLockKey { get; init; } = "thundering-herd:leader";

        public string TokenUpdateLockKey { get; init; } = "thundering-herd:token-update-lock";
    }

}
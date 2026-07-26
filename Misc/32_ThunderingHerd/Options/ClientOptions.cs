namespace Demo.Options
{

    internal sealed class ClientOptions
    {
        public string BaseAddress { get; init; } = "http://localhost:5234/";

        public string UserName { get; init; } = "test";

        public string Password { get; init; } = "password";

        public int ApiTimeoutSeconds { get; init; } = 10;

        public int RequestIntervalMilliseconds { get; init; } = 1000;

        public int TokenWaitTimeoutMilliseconds { get; init; } = 1000;

        public int TokenWaitRetryCount { get; init; } = 10;

        public int ProactiveRefreshSeconds { get; init; } = 3;

        public int LeaderLockSeconds { get; init; } = 15;

        public int LeaderRenewIntervalSeconds { get; init; } = 5;

        public int LeaderPollingIntervalMilliseconds { get; init; } = 200;

        public int TokenUpdateLockSeconds { get; init; } = 10;
    }

}
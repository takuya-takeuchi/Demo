using System.CommandLine;
using System.Net;

using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using StackExchange.Redis;

using Demo.Caching;
using Demo.Coordination;
using Demo.Options;

namespace Demo
{

    internal sealed class Program
    {

        #region Methods

        public static Task<int> Main(string[] args)
        {
            var modeOption = new Option<ClientMode>("--mode", "-m")
            {
                Description = "Execution mode: SharedLeader or Independent.",
                Required = true
            };

            var rootCommand = new RootCommand("Thundering Herd authentication test client.");
            rootCommand.Options.Add(modeOption);
            rootCommand.SetAction(async (parseResult, cancellationToken) =>
            {
                ClientMode mode = parseResult.GetValue(modeOption);
                await RunAsync(mode, args, cancellationToken);
            });

            ParseResult result = rootCommand.Parse(args);
            return result.InvokeAsync();
        }

        private static async Task RunAsync(ClientMode mode, string[] args, CancellationToken commandCancellationToken)
        {
            HostApplicationBuilder builder = Host.CreateApplicationBuilder(args);
            builder.Configuration.SetBasePath(AppContext.BaseDirectory)
                                 .AddJsonFile("appsettings.json", optional: false, reloadOnChange: false)
                                 .AddEnvironmentVariables();

            builder.Logging.ClearProviders();
            builder.Logging.AddConsole();

            builder.Services.AddOptions<ClientOptions>()
                            .Bind(builder.Configuration.GetSection("Client"))
                            .ValidateOnStart();
            builder.Services.AddOptions<RedisOptions>()
                            .Bind(builder.Configuration.GetSection("Redis"))
                            .ValidateOnStart();

            ClientOptions clientOptions = builder.Configuration.GetSection("Client").Get<ClientOptions>()
                ?? throw new InvalidOperationException("Client configuration was not found.");
            RedisOptions redisOptions = builder.Configuration.GetSection("Redis").Get<RedisOptions>()
                ?? throw new InvalidOperationException("Redis configuration was not found.");

            builder.Services.AddHttpClient<ApiClient>(client =>
            {
                client.BaseAddress = new Uri(clientOptions.BaseAddress);
                client.Timeout = TimeSpan.FromSeconds(clientOptions.ApiTimeoutSeconds);
            });

            builder.Services.AddSingleton<IConnectionMultiplexer>(_ =>
            {
                ConfigurationOptions configuration = ConfigurationOptions.Parse(redisOptions.Configuration);
                configuration.AbortOnConnectFail = false;
                return ConnectionMultiplexer.Connect(configuration);
            });

            builder.Services.AddSingleton<TokenCache>();
            builder.Services.AddSingleton<LeaderLock>();
            builder.Services.AddSingleton<TokenUpdateLock>();

            using IHost host = builder.Build();

            ApiClient apiClient = host.Services.GetRequiredService<ApiClient>();
            TokenCache tokenCache = host.Services.GetRequiredService<TokenCache>();
            LeaderLock leaderLock = host.Services.GetRequiredService<LeaderLock>();
            TokenUpdateLock tokenUpdateLock = host.Services.GetRequiredService<TokenUpdateLock>();

            ILogger<Program> logger = host.Services.GetRequiredService<ILogger<Program>>();

            using var consoleCancellationSource = new CancellationTokenSource();

            Console.CancelKeyPress += (_, eventArgs) =>
            {
                eventArgs.Cancel = true;
                consoleCancellationSource.Cancel();
            };

            using CancellationTokenSource linkedCancellationSource = CancellationTokenSource.CreateLinkedTokenSource(commandCancellationToken, consoleCancellationSource.Token);

            CancellationToken cancellationToken = linkedCancellationSource.Token;
            string processName = $"{Environment.MachineName}:{Environment.ProcessId}";
            string lockOwner = $"{processName}:{Guid.NewGuid():N}";

            logger.LogInformation($"Client started. Process={processName}, Mode={mode}");

            try
            {
                switch (mode)
                {
                    case ClientMode.SharedLeader:
                        await RunSharedLeaderModeAsync(apiClient,
                                                       tokenCache,
                                                       leaderLock,
                                                       tokenUpdateLock,
                                                       clientOptions,
                                                       logger,
                                                       processName,
                                                       lockOwner,
                                                       cancellationToken);
                        break;
                    case ClientMode.Independent:
                        await RunIndependentModeAsync(apiClient,
                                                      clientOptions,
                                                      logger,
                                                      processName,
                                                      cancellationToken);
                        break;
                    default:
                        throw new ArgumentOutOfRangeException(nameof(mode), mode, "Unsupported client mode.");
                }
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                logger.LogInformation($"Client stopped. Process={processName}");
            }
        }
        
        private static async Task RunSharedLeaderModeAsync(ApiClient apiClient,
                                                           TokenCache tokenCache,
                                                           LeaderLock leaderLock,
                                                           TokenUpdateLock tokenUpdateLock,
                                                           ClientOptions options,
                                                           ILogger logger,
                                                           string processName,
                                                           string lockOwner,
                                                           CancellationToken cancellationToken)
        {
            bool isLeader = await leaderLock.TryAcquireAsync(lockOwner, cancellationToken);
            logger.LogWarning($"Leader election completed. Process={processName}, IsLeader={isLeader}");

            Task? leaderTask = null;

            if (isLeader)
            {
                leaderTask = RunLeaderLoopAsync(apiClient,
                                                tokenCache,
                                                leaderLock,
                                                tokenUpdateLock,
                                                options,
                                                logger,
                                                processName,
                                                lockOwner,
                                                cancellationToken);
            }

            try
            {
                // All foreground requests only read the shared token.
                // They never execute login or refresh operations.
                await RunSharedRequestLoopAsync(apiClient,
                                                tokenCache,
                                                options,
                                                logger,
                                                processName,
                                                cancellationToken);
            }
            finally
            {
                if (isLeader)
                {
                    bool released = await leaderLock.ReleaseAsync(lockOwner, CancellationToken.None);
                    if (!released)
                    {
                        logger.LogWarning($"Leader lock could not be released. Process={processName}");
                    }
                }

                if (leaderTask is not null)
                {
                    try
                    {
                        await leaderTask;
                    }
                    catch (OperationCanceledException)
                    {
                        // Normal shutdown.
                    }
                }
            }
        }

        private static async Task RunLeaderLoopAsync(ApiClient apiClient,
                                                     TokenCache tokenCache,
                                                     LeaderLock leaderLock,
                                                     TokenUpdateLock tokenUpdateLock,
                                                     ClientOptions options,
                                                     ILogger logger,
                                                     string processName,
                                                     string lockOwner,
                                                     CancellationToken cancellationToken)
        {
            DateTimeOffset nextLeaderRenewal = DateTimeOffset.UtcNow;

            while (!cancellationToken.IsCancellationRequested)
            {
                DateTimeOffset now = DateTimeOffset.UtcNow;
                if (now >= nextLeaderRenewal)
                {
                    bool renewed = await leaderLock.RenewAsync(lockOwner,  cancellationToken);
                    if (!renewed)
                    {
                        logger.LogError("Leader lock was lost. Process={Process}", processName);
                        return;
                    }

                    nextLeaderRenewal = now.AddSeconds(options.LeaderRenewIntervalSeconds);
                }

                TokenResponse? token = await tokenCache.GetAsync(cancellationToken);
                bool updateRequired = token is null || token.AccessTokenExpiresAt - DateTimeOffset.UtcNow <= TimeSpan.FromSeconds(options.ProactiveRefreshSeconds);
                if (updateRequired)
                {
                    await TryUpdateSharedTokenAsync(apiClient,
                                                    tokenCache,
                                                    tokenUpdateLock,
                                                    options,
                                                    logger,
                                                    processName,
                                                    lockOwner,
                                                    cancellationToken);
                }

                await Task.Delay(options.LeaderPollingIntervalMilliseconds, cancellationToken);
            }
        }

        private static async Task TryUpdateSharedTokenAsync(ApiClient apiClient,
                                                            TokenCache tokenCache,
                                                            TokenUpdateLock tokenUpdateLock,
                                                            ClientOptions options,
                                                            ILogger logger,
                                                            string processName,
                                                            string lockOwner,
                                                            CancellationToken cancellationToken)
        {
            bool acquired = await tokenUpdateLock.TryAcquireAsync(lockOwner, cancellationToken);
            if (!acquired)
            {
                logger.LogDebug($"Another process owns the token update lock. Process={processName}");
                return;
            }

            try
            {
                // Always reload the token after acquiring the update lock.
                // Another process may have updated it while this process
                // was waiting for the lock.
                TokenResponse? latestToken = await tokenCache.GetAsync(cancellationToken);
                if (latestToken is null)
                {
                    await LoginAndStoreAsync(apiClient,
                                             tokenCache,
                                             tokenUpdateLock,
                                             options,
                                             logger,
                                             processName,
                                             lockOwner,
                                             cancellationToken);
                    return;
                }

                TimeSpan remaining = latestToken.AccessTokenExpiresAt - DateTimeOffset.UtcNow;
                if (remaining > TimeSpan.FromSeconds(options.ProactiveRefreshSeconds))
                {
                    logger.LogDebug($"The token was already updated. Process={processName}, Remaining={remaining.TotalSeconds:F1}s");
                    return;
                }

                await RefreshAndStoreAsync(apiClient,
                                           tokenCache,
                                           tokenUpdateLock,
                                           latestToken,
                                           logger,
                                           processName,
                                           lockOwner,
                                           cancellationToken);
            }
            finally
            {
                bool released = await tokenUpdateLock.ReleaseAsync(lockOwner, CancellationToken.None);
                if (!released)
                {
                    logger.LogWarning($"Token update lock could not be released. The lock may have expired or ownership may have been lost. Process={processName}");
                }
            }
        }

        private static async Task LoginAndStoreAsync(ApiClient apiClient,
                                                     TokenCache tokenCache,
                                                     TokenUpdateLock tokenUpdateLock,
                                                     ClientOptions options,
                                                     ILogger logger,
                                                     string processName,
                                                     string lockOwner,
                                                     CancellationToken cancellationToken)
        {
            bool renewed = await tokenUpdateLock.RenewAsync(lockOwner, cancellationToken);
            if (!renewed)
            {
                logger.LogError($"Token update lock was lost before login. Process={processName}");
                return;
            }

            logger.LogWarning($"No shared token exists. Login will be performed. Process={processName}");
            TokenResponse? loginToken = await apiClient.LoginAsync(new LoginRequest
            {
                UserName = options.UserName,
                Password = options.Password
            }, cancellationToken);

            if (loginToken is null)
            {
                logger.LogError($"Login failed. Process={processName}");
                return;
            }

            //Verify that this process still owns the update lock before
            //storing the token returned by the remote server.
            renewed = await tokenUpdateLock.RenewAsync(lockOwner, cancellationToken);
            if (!renewed)
            {
                logger.LogError($"Token update lock was lost after login. The returned token will not be stored. Process={processName}");
                return;
            }

            // Do not overwrite a token that may have been stored by another
            // process while the login operation was in progress.
            TokenResponse? cachedToken = await tokenCache.GetAsync(cancellationToken);
            if (cachedToken is not null && cachedToken.AccessTokenExpiresAt > loginToken.AccessTokenExpiresAt)
            {
                logger.LogWarning($"A newer shared token already exists. The login result will not overwrite it. Process={processName}");
                return;
            }

            await tokenCache.SetAsync(loginToken, cancellationToken);
            logger.LogWarning($"Login token stored. Process={processName}, AccessExpiresAt={loginToken.AccessTokenExpiresAt:O}");
        }

        private static async Task RefreshAndStoreAsync(ApiClient apiClient,
                                                       TokenCache tokenCache,
                                                       TokenUpdateLock tokenUpdateLock,
                                                       TokenResponse currentToken,
                                                       ILogger logger,
                                                       string processName,
                                                       string lockOwner,
                                                       CancellationToken cancellationToken)
        {
            bool renewed = await tokenUpdateLock.RenewAsync(lockOwner, cancellationToken);
            if (!renewed)
            {
                logger.LogError($"Token update lock was lost before refresh. Process={processName}");
                return;
            }

            TimeSpan remaining = currentToken.AccessTokenExpiresAt - DateTimeOffset.UtcNow;
            logger.LogWarning($"Proactive refresh will be performed. Process={processName}, Remaining={remaining.TotalSeconds:F1}s");

            TokenResponse? refreshedToken = await apiClient.RefreshAsync(new RefreshRequest
            {
                RefreshToken = currentToken.RefreshToken
            }, cancellationToken);

            if (refreshedToken is null)
            {
                await HandleRefreshFailureAsync(tokenCache,
                                                currentToken,
                                                logger,
                                                processName,
                                                cancellationToken);
                return;
            }

            // Verify ownership again because the remote API call may take
            // longer than expected.
            renewed = await tokenUpdateLock.RenewAsync(lockOwner, cancellationToken);
            if (!renewed)
            {
                logger.LogError($"Token update lock was lost after refresh. The returned token will not be stored. Process={processName}");
                return;
            }

            // Reload the cached token before writing. If the refresh token
            // changed, another process has already replaced the shared token.
            TokenResponse? cachedToken = await tokenCache.GetAsync(cancellationToken);
            if (cachedToken is not null && !string.Equals(cachedToken.RefreshToken, currentToken.RefreshToken, StringComparison.Ordinal))
            {
                logger.LogWarning($"The shared token changed during refresh. The refresh result will not overwrite the newer token. Process={processName}");
                return;
            }

            bool stored = await tokenCache.SetIfRefreshTokenMatchesAsync(currentToken.RefreshToken, refreshedToken, cancellationToken);
            if (!stored)
            {
                logger.LogWarning($"The refreshed token was not stored because the shared token changed. Process={processName}");
                return;
            }

            logger.LogWarning($"Refreshed token stored. Process={processName}, AccessExpiresAt={refreshedToken.AccessTokenExpiresAt:O}");
        }

        private static async Task HandleRefreshFailureAsync(TokenCache tokenCache,
                                                            TokenResponse attemptedToken,
                                                            ILogger logger,
                                                            string processName,
                                                            CancellationToken cancellationToken)
        {
            // Do not delete the shared token unconditionally. Another process
            // may have already replaced it with a valid token.
            bool deleted = await tokenCache.DeleteIfRefreshTokenMatchesAsync(attemptedToken.RefreshToken, cancellationToken);
            logger.LogWarning($"Refresh failed. Conditional token deletion result={deleted}. Process={processName}");
        }

        private static async Task RunSharedRequestLoopAsync(ApiClient apiClient,
                                                            TokenCache tokenCache,
                                                            ClientOptions options,
                                                            ILogger logger,
                                                            string processName,
                                                            CancellationToken cancellationToken)
        {
            long requestNumber = 0;

            while (!cancellationToken.IsCancellationRequested)
            {
                requestNumber++;
                TokenResponse? token = await GetSharedTokenWithRetryAsync(tokenCache,
                                                                          options,
                                                                          logger,
                                                                          cancellationToken);
                if (token is null)
                {
                    logger.LogError($"The shared token could not be obtained. Process={processName}, Request={requestNumber}");
                }
                else
                {
                    int statusCode;

                    try
                    {
                        statusCode = await CallTestAsync(apiClient, token.AccessToken, cancellationToken);
                    }
                    catch (ApiException<ProblemDetails> exception)
                    {
                        statusCode = exception.StatusCode;
                    }
                    catch (ApiException exception)
                    {
                        statusCode = exception.StatusCode;
                    }

                    logger.LogInformation($"GET /api/Test. Process={processName}, Request={requestNumber}, Status={statusCode}");

                    // Foreground processing intentionally does not perform
                    // login or refresh, even when the API returns 401.
                }

                await Task.Delay(options.RequestIntervalMilliseconds, cancellationToken);
            }
        }

        private static async Task<TokenResponse?> GetSharedTokenWithRetryAsync(TokenCache tokenCache,
                                                                               ClientOptions options,
                                                                               ILogger logger,
                                                                               CancellationToken cancellationToken)
        {
            for (int attempt = 1; attempt <= options.TokenWaitRetryCount; attempt++)
            {
                TokenResponse? token = await tokenCache.GetAsync(cancellationToken);
                if (token is not null)
                    return token;

                logger.LogWarning($"The shared token is unavailable. Attempt={attempt}/{options.TokenWaitRetryCount}");
                await Task.Delay(options.TokenWaitTimeoutMilliseconds, cancellationToken);
            }

            return null;
        }

        private static async Task RunIndependentModeAsync(ApiClient apiClient,
                                                          ClientOptions options,
                                                          ILogger logger,
                                                          string processName,
                                                          CancellationToken cancellationToken)
        {
            TokenResponse? localToken = null;
            long requestNumber = 0;

            while (!cancellationToken.IsCancellationRequested)
            {
                requestNumber++;

                if (localToken is null)
                {
                    logger.LogWarning($"This process will perform login. Process={processName}");
                    localToken = await apiClient.LoginAsync(new LoginRequest
                    {
                        UserName = options.UserName,
                        Password = options.Password
                    }, cancellationToken);

                    if (localToken is null)
                    {
                        await Task.Delay(options.RequestIntervalMilliseconds, cancellationToken);
                        continue;
                    }
                }

                var statusCode = await CallTestAsync(apiClient, localToken.AccessToken, cancellationToken);
                logger.LogInformation($"GET /api/Test. Process={processName}, Request={requestNumber}, Status={statusCode}");

                if (statusCode == (int)HttpStatusCode.Unauthorized)
                {
                    logger.LogWarning($"The access token was rejected. This process will perform refresh. Process={processName}");
                    TokenResponse? refreshedToken =  await apiClient.RefreshAsync(new RefreshRequest
                    {
                        RefreshToken = localToken.RefreshToken
                    }, cancellationToken);

                    if (refreshedToken is not null)
                    {
                        localToken = refreshedToken;
                        logger.LogWarning($"This process refreshed its own token. Process={processName}");
                    }
                    else
                    {
                        // A failed refresh causes the process to perform login
                        // during the next iteration.
                        localToken = null;
                        logger.LogWarning($"Refresh failed. This process will perform login again. Process={processName}");
                    }
                }

                await Task.Delay(options.RequestIntervalMilliseconds, cancellationToken);
            }
        }

        private static async Task<int> CallTestAsync(ApiClient apiClient,
                                                     string accessToken,
                                                     CancellationToken cancellationToken)
        {
            apiClient.AccessToken = accessToken;

            try
            {
                await apiClient.TestAsync(cancellationToken);

                return (int)HttpStatusCode.OK;
            }
            catch (ApiException<ProblemDetails> exception)
            {
                return exception.StatusCode;
            }
            catch (ApiException exception)
            {
                return exception.StatusCode;
            }
        }

        #endregion

    }

}
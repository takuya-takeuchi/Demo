using System.Net.Http.Json;
using System.Text.Json.Serialization;

using Microsoft.Extensions.Options;

namespace Demo.Auth
{

    /// <summary>
    /// Checks that a session is still alive by calling Hanko's POST /sessions/validate.
    ///
    /// Verifying the JWT signature alone lets a token keep working until exp even after the user
    /// logs out or an administrator revokes the session. Enable this only when immediate revocation
    /// is required (Hanko:ValidateSessionRemotely = true). It costs one extra HTTP call per request,
    /// so calling it by hand for sensitive operations only is a reasonable alternative.
    /// </summary>
    public class HankoSessionValidator(HttpClient http, IOptions<HankoOptions> options, ILogger<HankoSessionValidator> logger)
    {

        #region Fields

        private readonly HankoOptions _options = options.Value;

        #endregion

        #region Methods

        public async Task<bool> IsSessionValidAsync(string sessionToken, CancellationToken ct = default)
        {
            try
            {
                var url = $"{_options.InternalApiUrl.TrimEnd('/')}/sessions/validate";
                using var res = await http.PostAsJsonAsync(url, new ValidateRequest(sessionToken), ct);

                if (!res.IsSuccessStatusCode)
                {
                    logger.LogWarning("Hanko session validation returned {Status}", res.StatusCode);
                    return false;
                }

                var body = await res.Content.ReadFromJsonAsync<ValidateResponse>(ct);
                return body?.IsValid == true;
            }
            catch (Exception ex)
            {
                // Treat an unreachable Hanko as "not valid" (fail closed)
                logger.LogError(ex, "Failed to reach Hanko session validation endpoint");
                return false;
            }
        }

        #endregion

        #region Records

        private sealed record ValidateRequest([property: JsonPropertyName("session_token")] string SessionToken);

        private sealed record ValidateResponse([property: JsonPropertyName("is_valid")] bool IsValid,
                                               [property: JsonPropertyName("expiration_time")] DateTimeOffset? ExpirationTime);

        #endregion

    }

}

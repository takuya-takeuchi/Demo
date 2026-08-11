using System.Net.Http.Json;
using System.Text.Json.Serialization;

using Microsoft.Extensions.Options;

namespace Demo.Auth
{

    /// <summary>
    /// Hanko の POST /sessions/validate を叩いてセッションの生存を確認する。
    ///
    /// JWT の署名検証だけだと、ユーザーがログアウトしたり管理者がセッションを失効させても
    /// exp まではトークンが通ってしまう。即時失効が必要な場合のみ有効にする
    /// （Hanko:ValidateSessionRemotely = true）。毎リクエスト HTTP が1本増えるので、
    /// 重要な操作のみ手動で呼ぶ運用でもよい。
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
                // Hanko に到達できない場合は「無効」として扱う（fail closed）
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

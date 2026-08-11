using System.Security.Claims;

using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.IdentityModel.Protocols;
using Microsoft.IdentityModel.Protocols.OpenIdConnect;
using Microsoft.IdentityModel.Tokens;

namespace Demo.Auth
{

    public static class HankoAuthenticationExtensions
    {

        #region Methods

        public static IServiceCollection AddHankoAuthentication(this IServiceCollection services, IConfiguration configuration)
        {
            services.Configure<HankoOptions>(configuration.GetSection(HankoOptions.SectionName));
            var hanko = configuration.GetSection(HankoOptions.SectionName).Get<HankoOptions>() ?? new HankoOptions();

            services.AddHttpClient<HankoSessionValidator>();

            var jwksUri = $"{hanko.InternalApiUrl.TrimEnd('/')}/.well-known/jwks.json";

            services.AddAuthentication(JwtBearerDefaults.AuthenticationScheme)
                    .AddJwtBearer(options =>
            {
                // 開発時は http の Hanko を見に行くので RequireHttps を落とす
                var documentRetriever = new HttpDocumentRetriever
                {
                    RequireHttps = jwksUri.StartsWith("https://", StringComparison.OrdinalIgnoreCase)
                };

                options.ConfigurationManager = new ConfigurationManager<OpenIdConnectConfiguration>(
                    jwksUri, new HankoJwksRetriever(), documentRetriever)
                {
                    AutomaticRefreshInterval = TimeSpan.FromHours(12),
                    RefreshInterval = TimeSpan.FromMinutes(5),
                };

                options.MapInboundClaims = false; // sub を ClaimTypes.NameIdentifier に化けさせない

                options.TokenValidationParameters = new TokenValidationParameters
                {
                    // Hanko の JWT は iss を含まないため検証しない
                    ValidateIssuer = false,

                    ValidateAudience = hanko.ValidateAudience,
                    ValidAudiences = hanko.ValidAudiences,

                    ValidateLifetime = true,
                    ClockSkew = TimeSpan.FromSeconds(30),

                    ValidateIssuerSigningKey = true,
                    NameClaimType = "email",
                    RoleClaimType = "roles",
                };

                options.Events = new JwtBearerEvents
                {
                    // Authorization ヘッダが無ければ Hanko の Cookie からトークンを拾う
                    OnMessageReceived = context =>
                    {
                        if (string.IsNullOrEmpty(context.Token) &&
                            context.Request.Cookies.TryGetValue(hanko.CookieName, out var cookieToken) &&
                            !string.IsNullOrWhiteSpace(cookieToken))
                        {
                            context.Token = cookieToken;
                        }

                        return Task.CompletedTask;
                    },

                    // 署名検証を通った後、必要ならサーバー側のセッション生存も確認する
                    OnTokenValidated = async context =>
                    {
                        if (!hanko.ValidateSessionRemotely)
                        {
                            return;
                        }

                        var rawToken = context.Request.Headers.Authorization.ToString()
                            .Replace("Bearer ", string.Empty, StringComparison.OrdinalIgnoreCase);

                        if (string.IsNullOrWhiteSpace(rawToken))
                        {
                            context.Request.Cookies.TryGetValue(hanko.CookieName, out rawToken);
                        }

                        if (string.IsNullOrWhiteSpace(rawToken))
                        {
                            context.Fail("No session token present.");
                            return;
                        }

                        var validator = context.HttpContext.RequestServices
                            .GetRequiredService<HankoSessionValidator>();

                        if (!await validator.IsSessionValidAsync(rawToken, context.HttpContext.RequestAborted))
                        {
                            context.Fail("Session has been revoked.");
                        }
                    },
                };
            });

            services.AddAuthorization();
            return services;
        }

        public static string GetHankoUserId(this ClaimsPrincipal principal)
        {
            return principal.FindFirstValue("sub") ?? principal.FindFirstValue(ClaimTypes.NameIdentifier)
                                                   ?? throw new InvalidOperationException("JWT に sub クレームがありません。");
        }

        public static string? GetEmail(this ClaimsPrincipal principal)
        {
            return principal.FindFirstValue("email") ?? principal.FindFirstValue(ClaimTypes.Email);
        }

        #endregion

    }

}

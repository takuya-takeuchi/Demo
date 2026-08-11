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
                // In development Hanko is reached over http, so RequireHttps has to be relaxed
                var documentRetriever = new HttpDocumentRetriever
                {
                    RequireHttps = jwksUri.StartsWith("https://", StringComparison.OrdinalIgnoreCase)
                };

                options.ConfigurationManager = new ConfigurationManager<OpenIdConnectConfiguration>(jwksUri, new HankoJwksRetriever(), documentRetriever)
                {
                    AutomaticRefreshInterval = TimeSpan.FromHours(12),
                    RefreshInterval = TimeSpan.FromMinutes(5),
                };

                options.MapInboundClaims = false; // keep sub as-is instead of renaming it to ClaimTypes.NameIdentifier

                options.TokenValidationParameters = new TokenValidationParameters
                {
                    // Hanko's JWT carries no iss claim, so there is nothing to validate
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
                    // Fall back to Hanko's cookie when there is no Authorization header
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

                    // Once the signature checks out, optionally confirm the session is still alive server-side
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
                                                   ?? throw new InvalidOperationException("The JWT has no sub claim.");
        }

        public static string? GetEmail(this ClaimsPrincipal principal)
        {
            return principal.FindFirstValue("email") ?? principal.FindFirstValue(ClaimTypes.Email);
        }

        #endregion

    }

}

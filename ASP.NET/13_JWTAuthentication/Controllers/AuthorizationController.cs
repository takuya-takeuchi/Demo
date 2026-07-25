using System;
using System.Threading;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Microsoft.IdentityModel.Tokens;

using Demo.Models;
using Demo.Options;
using Demo.Services;

namespace Demo.Controllers
{

    /// <summary>
    /// Provide authorization.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    [Produces("application/json")]
    public sealed class AuthorizationController : ControllerBase
    {

        #region Fields

        private readonly TokenService _TokenService;

        private readonly RefreshTokenStore _RefreshTokenStore;

        private readonly RefreshMetrics _Metrics;

        private readonly JwtOptions _JwtOptions;

        private readonly ILogger<AuthorizationController> _Logger;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="AuthorizationController"/> class with the specified logger.
        /// </summary>
        public AuthorizationController(TokenService tokenService,
                                       RefreshTokenStore refreshTokenStore,
                                       RefreshMetrics metrics,
                                       IOptions<JwtOptions> jwtOptions,
                                       ILogger<AuthorizationController> logger)
        {
            _TokenService = tokenService;
            _RefreshTokenStore = refreshTokenStore;
            _Metrics = metrics;
            _JwtOptions = jwtOptions.Value;
            _Logger = logger;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Login.
        /// </summary>
        [AllowAnonymous]
        [HttpPost("login")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        public IActionResult Login([FromBody] LoginRequest request)
        {
            // This is for demo!!!!!!
            if (request.UserName != "test" ||
                request.Password != "password")
                return Unauthorized();

            const string userId = "user-001";

            TokenResponse response = _TokenService.CreateTokenResponse(userId);
            _RefreshTokenStore.Store(response.RefreshToken,
                                     userId,
                                     response.RefreshTokenExpiresAt);

            return Ok(response);
        }

        /// <summary>
        /// Refresh token.
        /// </summary>
        [AllowAnonymous]
        [HttpPost("refresh")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        public async Task<IActionResult> Refresh([FromBody] RefreshRequest request,
                                                 CancellationToken cancellationToken)
        {
            int activeRequests = _Metrics.Enter();
            _Logger.LogWarning("Refresh started. Active={Active}, Peak={Peak}, Token={Token}",
                               activeRequests,
                               _Metrics.Peak,
                               Abbreviate(request.RefreshToken));

            try
            {
                // Simulate access delays to authentication servers, databases, Redis, external IdPs, and other systems
                await Task.Delay(_JwtOptions.RefreshDelayMilliseconds, cancellationToken);

                if (!_RefreshTokenStore.TryConsume(request.RefreshToken, out RefreshTokenEntry? entry))
                    return Unauthorized();

                if (entry.ExpiresAt <= DateTimeOffset.UtcNow)
                {
                    _RefreshTokenStore.Remove(request.RefreshToken);
                    _Logger.LogWarning("Refresh token has expired.");
                    return Unauthorized();
                }

                TokenResponse response = _TokenService.CreateTokenResponse(entry.UserId);
                _RefreshTokenStore.Store(response.RefreshToken,
                                         entry.UserId,
                                         response.RefreshTokenExpiresAt);

                return Ok(response);
            }
            finally
            {
                int remaining = _Metrics.Exit();
                _Logger.LogWarning("Refresh completed. Active={Active}, Peak={Peak}",
                                   remaining,
                                   _Metrics.Peak);
            }
        }

        #region Helpers        

        private static string Abbreviate(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "(empty)";
            return value.Length <= 12 ? value : $"{value[..8]}...";
        }
        
        #endregion

        #endregion

    }

}
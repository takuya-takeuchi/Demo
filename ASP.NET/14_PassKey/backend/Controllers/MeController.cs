using System.Threading;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;

using Demo.Auth;
using Demo.Models;

namespace Demo.Controllers
{

    /// <summary>
    /// Provides the signed-in user with their own account information.
    /// </summary>
    [ApiController]
    [Authorize]
    // Spelled out instead of "api/[controller]": that token keeps the class casing, which would
    // publish the route as /api/Me and show up capitalised in the OpenAPI document.
    [Route("api/me")]
    [Produces("application/json")]
    public sealed class MeController(CurrentUserService currentUser) : ControllerBase
    {

        #region Methods

        /// <summary>
        /// Returns the current user. The row is created on first access.
        /// </summary>
        [HttpGet]
        [ProducesResponseType(typeof(MeDto), StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        public async Task<ActionResult<MeDto>> Get(CancellationToken ct)
        {
            var user = await currentUser.GetOrCreateAsync(ct);

            return this.Ok(new MeDto(user.Id, user.Email, user.DisplayName, user.CreatedAt));
        }

        #endregion

    }

}

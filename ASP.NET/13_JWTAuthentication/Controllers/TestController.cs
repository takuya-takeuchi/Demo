using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Logging;

namespace Demo.Controllers
{

    /// <summary>
    /// Provide simple functions.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    [Produces("application/json")]
    public sealed class TestController : ControllerBase
    {

        #region Fields

        private readonly ILogger<TestController> _Logger;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="TestController"/> class with the specified logger.
        /// </summary>
        public TestController(ILogger<TestController> logger)
        {
            this._Logger = logger;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Ping.
        /// </summary>
        [Authorize]
        [HttpGet]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        public IActionResult Get()
        {
            return this.Ok();
        }

        #endregion

    }

}
using System;

using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Logging;

using Demo.Models;

namespace Demo.Controllers
{

    /// <summary>
    /// Manage message.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    [Produces("application/json")]
    public sealed class GreetingController : ControllerBase
    {

        #region Fields

        private readonly ILogger<GreetingController> _Logger;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="GreetingController"/> class with the specified logger.
        /// </summary>
        public GreetingController(ILogger<GreetingController> logger)
        {
            this._Logger = logger;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Get message.
        /// </summary>
        [Route("~/api/greet")]
        [HttpGet]
        [ProducesResponseType(StatusCodes.Status200OK)]
        public ActionResult<Message> Get()
        {
            return this.Ok(new Message
            {
                Date = DateTime.Now,
                Text = "Hello!!"
            });
        }

        #endregion

    }

}

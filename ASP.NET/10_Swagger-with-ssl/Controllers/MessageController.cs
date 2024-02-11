using Microsoft.AspNetCore.Mvc;

using SwaggerApiDemo.Models;

namespace SwaggerApiDemo
{

    /// <summary>
    /// Manage message.
    /// </summary>
    [ApiController]
    [Route("api/[controller]")]
    [Produces("application/json")]
    public sealed class MessageController : ControllerBase
    {

        #region Fields

        private readonly ILogger<MessageController> _Logger;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="MessageController"/> class with the specified logger.
        /// </summary>
        public MessageController(ILogger<MessageController> logger)
        {
            this._Logger = logger;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Get message.
        /// </summary>
        [Route("~/api/getMessage")]
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

        /// <summary>
        /// Post message.
        /// </summary>
        [Route("~/api/postMessage")]
        [HttpPost]
        [ProducesResponseType(StatusCodes.Status200OK)]
        public ActionResult<Result> Post(Message message)
        {
            return this.Ok(new Result { StatusCode = StatusCodes.Status200OK});
        }

        #endregion

    }

}

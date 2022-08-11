using System.Threading.Tasks;
using Demo;
using Grpc.Core;
using Microsoft.Extensions.Logging;

namespace Server.Services
{

    public sealed class TestService : Test.TestBase
    {

        #region Fields

        private readonly ILogger<TestService> _Logger;

        #endregion

        #region Constructors

        public TestService(ILogger<TestService> logger)
        {
            this._Logger = logger;
        }

        #endregion

        #region Methods

        public override Task<SendReply> Send(SendRequest request, ServerCallContext context)
        {
            this._Logger.LogInformation("Send");
            return Task.FromResult(new SendReply
            {
                Message = "OK"
            });
        }

        #endregion

    }

}
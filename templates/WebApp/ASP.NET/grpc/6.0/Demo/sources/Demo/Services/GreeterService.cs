using System.Threading.Tasks;
using Grpc.Core;
using Microsoft.Extensions.Logging;

namespace Demo.Services
{

    public sealed class GreeterService : Greeter.GreeterBase
    {

        #region Fields

        private readonly ILogger<GreeterService> _Logger;

        #endregion

        #region Constructors

        public GreeterService(ILogger<GreeterService> logger)
        {
            this._Logger = logger;
        }

        #endregion

        #region Methods

        public override Task<HelloReply> SayHello(HelloRequest request, ServerCallContext context)
        {
            this._Logger.LogInformation("SayHello");
            return Task.FromResult(new HelloReply
            {
                Message = "Hello " + request.Name
            });
        }

        #endregion

    }

}
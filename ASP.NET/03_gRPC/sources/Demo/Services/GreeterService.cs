using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

using Microsoft.Extensions.Logging;

using Grpc.Core;

namespace Demo
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
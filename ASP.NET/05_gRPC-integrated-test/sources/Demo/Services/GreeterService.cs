using System;
using System.Threading.Tasks;
using Grpc.Core;
using Microsoft.Extensions.Logging;

using Demo.Grpc;
using TimeZone = Demo.Grpc.TimeZone;

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
            switch (request.TimeZone)
            {
                case TimeZone.Morning:
                    return Task.FromResult(new HelloReply
                    {
                        Message = "Good Morning, " + request.Name
                    });
                case TimeZone.Afternoon:
                    return Task.FromResult(new HelloReply
                    {
                        Message = "Hello " + request.Name
                    });
                case TimeZone.Evening:
                    return Task.FromResult(new HelloReply
                    {
                        Message = "Good Evening, " + request.Name
                    });
                case TimeZone.Night:
                    return Task.FromResult(new HelloReply
                    {
                        Message = "Good Night, " + request.Name
                    });
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        #endregion

    }

}
using System.Threading.Tasks;

using Google.Protobuf;
using Grpc.Core;
using NLog;

public sealed class ExampleService : Example.ExampleBase
{

    #region Fields

    private readonly Logger _Logger = LogManager.GetCurrentClassLogger();

    #endregion

    public ExampleService(Logger logger)
    {
        this._Logger = logger;

    }

    public override Task<ExampleResponse> UnaryCall(ExampleRequest request, ServerCallContext context)
    {
        this._Logger.Info("Receive message on UnaryCall");

        var response = new ExampleResponse
        {
            Date = ByteString.CopyFrom(new byte[request.ResponseSize])
        };

        return Task.FromResult(response);
    }

}
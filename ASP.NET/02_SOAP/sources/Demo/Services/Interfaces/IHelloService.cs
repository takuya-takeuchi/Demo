using System.ServiceModel;

namespace Demo.Services.Interfaces
{

    [ServiceContract]
    public interface IHelloService
    {

        [OperationContract]
        string Hello(string menamessage);

    }

}
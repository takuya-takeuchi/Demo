using System.ServiceModel;

namespace Demo.Services.Interfaces
{

    [ServiceContract(Namespace = "http://demo.com/helloservice/")]
    public interface IHelloService
    {

        [OperationContract]
        string Hello(string menamessage);

    }

}
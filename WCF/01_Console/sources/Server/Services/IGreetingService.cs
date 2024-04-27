using System.ServiceModel;

using Server.Models;

namespace Server.Services
{

    [ServiceContract]
    public interface IGreetingService
    {

        [OperationContract]
        Greeting Greet(Greeting greeting);

    }

}

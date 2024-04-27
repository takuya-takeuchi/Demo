using Server.Models;

namespace Server.Services
{

    public sealed class GreetingService : IGreetingService
    {

        public Greeting Greet(Greeting greeting)
        {
            return new Greeting()
            {
                Message = "Hello, client"
            };
        }

    }

}

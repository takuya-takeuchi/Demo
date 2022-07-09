using Demo.Services.Interfaces;

namespace Demo.Services
{

    public sealed class HelloService : IHelloService
    {

        public string Hello(string name)
        {
            return $"Hello, {name}";
        }

    }

}
using Demo.Grpc;
using Microsoft.Extensions.Logging;
using Moq;

using Demo.Services;
using Demo.UnitTest.Helpers;
using TimeZone = Demo.Grpc.TimeZone;

namespace Demo.UnitTest
{

    public sealed class GreeterServiceTest
    {

        [Fact]
        public void SayHelloMorning()
        {
            var mock = new Mock<ILogger<GreeterService>>();
            var logger = mock.Object;

            const string name = "John";

            var service = new GreeterService(logger);
            var response = service.SayHello(new HelloRequest
            {
                Name = name,
                TimeZone = TimeZone.Morning
            }, TestServerCallContext.Create());

            Assert.Equal($"Good Morning, {name}", response.Result.Message);
        }

        [Fact]
        public void SayHelloAfternoon()
        {
            var mock = new Mock<ILogger<GreeterService>>();
            var logger = mock.Object;

            const string name = "John";

            var service = new GreeterService(logger);
            var response = service.SayHello(new HelloRequest
            {
                Name = name,
                TimeZone = TimeZone.Afternoon
            }, TestServerCallContext.Create());

            Assert.Equal($"Hello {name}", response.Result.Message);
        }

        [Fact]
        public void SayHelloEvening()
        {
            var mock = new Mock<ILogger<GreeterService>>();
            var logger = mock.Object;

            const string name = "John";

            var service = new GreeterService(logger);
            var response = service.SayHello(new HelloRequest
            {
                Name = name,
                TimeZone = TimeZone.Evening
            }, TestServerCallContext.Create());

            Assert.Equal($"Good Evening, {name}", response.Result.Message);
        }

        [Fact]
        public void SayHelloNight()
        {
            var mock = new Mock<ILogger<GreeterService>>();
            var logger = mock.Object;

            const string name = "John";

            var service = new GreeterService(logger);
            var response = service.SayHello(new HelloRequest
            {
                Name = name,
                TimeZone = TimeZone.Night
            }, TestServerCallContext.Create());

            Assert.Equal($"Good Night, {name}", response.Result.Message);
        }

    }

}
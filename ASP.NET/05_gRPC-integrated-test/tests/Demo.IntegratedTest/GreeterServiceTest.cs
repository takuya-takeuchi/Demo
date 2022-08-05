using Xunit.Abstractions;

using Demo.IntegratedTest.Helpers;

namespace Demo.IntegratedTest
{

    public sealed class GreeterServiceTest : IntegrationTestBase
    {

        #region Constructors

        public GreeterServiceTest(GrpcTestFixture<Startup> fixture, ITestOutputHelper outputHelper)
            : base(fixture, outputHelper)
        {
        }

        #endregion

        [Fact]
        public async void SayHello()
        {
            // Arrange
            var client = new Greeter.GreeterClient(this.Channel);

            // Act
            var response = await client.SayHelloAsync(new HelloRequest
            {
                Name = "Joe",
                TimeZone = TimeZone.Afternoon
            });

            // Assert
            Assert.Equal("Hello Joe", response.Message);
        }

    }

}
using Grpc.Net.Client;

namespace Demo.Test
{

    [TestClass]
    public sealed class GreeterServiceTest : TestBase<Greeter.GreeterClient>
    {

        #region Fields

        private GrpcChannel _GrpcChannel;

        #endregion

        #region Methods

        #region Helpers

        [TestInitialize]
        public void Initialize()
        {
            var httpHandler = new HttpClientHandler();
            httpHandler.ServerCertificateCustomValidationCallback = HttpClientHandler.DangerousAcceptAnyServerCertificateValidator;

            Environment.SetEnvironmentVariable("NO_PROXY", "localhost");
            this._GrpcChannel = GrpcChannel.ForAddress("https://localhost:5001", new GrpcChannelOptions { HttpHandler = httpHandler });
            this.Client = new Greeter.GreeterClient(this._GrpcChannel);
        }

        [TestCleanup]
        public void Cleanup()
        {
            this.Client = null!;
            this._GrpcChannel?.Dispose();
            this._GrpcChannel = null;
        }

        #endregion

        #endregion

        [TestMethod]
        public void SayHelloMorning()
        {
            const string name = "John";
            const string greet = "Good Morning,";
            var ret = this.Client.SayHello(new HelloRequest
            {
                Name = name, TimeZone = TimeZone.Morning
            });

            Assert.AreEqual($"{greet} {name}", ret.Message);
        }

        [TestMethod]
        public void SayHelloAfternoon()
        {
            const string name = "John";
            const string greet = "Hello";
            var ret = this.Client.SayHello(new HelloRequest
            {
                Name = name,
                TimeZone = TimeZone.Afternoon
            });

            Assert.AreEqual($"{greet} {name}", ret.Message);
        }

        [TestMethod]
        public void SayHelloEvening()
        {
            const string name = "John";
            const string greet = "Good Evening,";
            var ret = this.Client.SayHello(new HelloRequest
            {
                Name = name,
                TimeZone = TimeZone.Evening
            });

            Assert.AreEqual($"{greet} {name}", ret.Message);
        }

        [TestMethod]
        public void SayHelloNight()
        {
            const string name = "John";
            const string greet = "Good Night,";
            var ret = this.Client.SayHello(new HelloRequest
            {
                Name = name,
                TimeZone = TimeZone.Night
            });

            Assert.AreEqual($"{greet} {name}", ret.Message);
        }

    }

}
using Grpc.Net.Client;
using Microsoft.Extensions.Logging;
using Xunit.Abstractions;

using Demo.IntegratedTest.Helpers;

namespace Demo.IntegratedTest
{

    public abstract class IntegrationTestBase : IClassFixture<GrpcTestFixture<Startup>>, IDisposable
    {

        #region Fields

        private GrpcChannel? _Channel;

        private readonly IDisposable? _TestContext;

        #endregion

        #region Constructors

        public IntegrationTestBase(GrpcTestFixture<Startup> fixture, ITestOutputHelper outputHelper)
        {
            this.Fixture = fixture;
            this._TestContext = this.Fixture.GetTestContext(outputHelper);
        }

        #endregion

        #region Properties

        protected GrpcChannel Channel => this._Channel ??= this.CreateChannel();

        protected GrpcTestFixture<Startup> Fixture
        {
            get;
            set;
        }

        protected ILoggerFactory LoggerFactory => this.Fixture.LoggerFactory;

        #endregion

        #region Methods

        protected GrpcChannel CreateChannel()
        {
            return GrpcChannel.ForAddress("http://localhost", new GrpcChannelOptions
            {
                LoggerFactory = this.LoggerFactory,
                HttpHandler = this.Fixture.Handler
            });
        }

        #endregion

        #region IDisposable Members

        public void Dispose()
        {
            this._TestContext?.Dispose();
            this._Channel = null;
        }

        #endregion

    }

}
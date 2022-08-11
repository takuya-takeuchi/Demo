using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using Xunit.Abstractions;

namespace Demo.IntegratedTest.Helpers
{

    public sealed class GrpcTestFixture<TStartup> : IDisposable 
        where TStartup : class
    {

        #region Events
        
        public event LogMessage? LoggedMessage;

        #endregion

        #region Fields

        private TestServer? _Server;

        private IHost? _Host;

        private HttpMessageHandler? _Handler;

        private Action<IWebHostBuilder>? _ConfigureWebHost;

        #endregion

        #region Constructors

        public GrpcTestFixture()
        {
            this.LoggerFactory = new LoggerFactory();
            this.LoggerFactory.AddProvider(new ForwardingLoggerProvider((logLevel, category, eventId, message, exception) =>
            {
                this.LoggedMessage?.Invoke(logLevel, category, eventId, message, exception);
            }));
        }

        #endregion

        #region Properties

        public LoggerFactory LoggerFactory
        {
            get;
        }

        public HttpMessageHandler Handler
        {
            get
            {
                this.EnsureServer();
                return this._Handler!;
            }
        }
        #endregion

        #region Methods

        public void ConfigureWebHost(Action<IWebHostBuilder> configure)
        {
            this._ConfigureWebHost = configure;
        }

        public IDisposable GetTestContext(ITestOutputHelper outputHelper)
        {
            return new GrpcTestContext<TStartup>(this, outputHelper);
        }

        #region Helpers

        private void EnsureServer()
        {
            if (this._Host != null)
                return;
            var builder = new HostBuilder()
                .ConfigureServices(services =>
                {
                    services.AddSingleton<ILoggerFactory>(this.LoggerFactory);
                })
                .ConfigureWebHostDefaults(webHost =>
                {
                    webHost
                        .UseTestServer()
                        .UseStartup<TStartup>();

                    this._ConfigureWebHost?.Invoke(webHost);
                });
            this._Host = builder.Start();
            this._Server = this._Host.GetTestServer();
            this._Handler = this._Server.CreateHandler();
        }

        #endregion

        #endregion

        #region IDisposable Members
        
        public void Dispose()
        {
            this._Handler?.Dispose();
            this._Host?.Dispose();
            this._Server?.Dispose();
        }

        #endregion

    }

}
using System.Diagnostics;

using Microsoft.Extensions.Logging;
using Xunit.Abstractions;

namespace Demo.IntegratedTest.Helpers
{

    internal sealed class GrpcTestContext<TStartup> : IDisposable
        where TStartup : class
    {

        #region Fields

        private readonly Stopwatch _Stopwatch;

        private readonly GrpcTestFixture<TStartup> _Fixture;

        private readonly ITestOutputHelper _OutputHelper;

        #endregion

        #region Constructors

        public GrpcTestContext(GrpcTestFixture<TStartup> fixture, ITestOutputHelper outputHelper)
        {
            this._Stopwatch = Stopwatch.StartNew();
            this._Fixture = fixture;
            this._OutputHelper = outputHelper;
            this._Fixture.LoggedMessage += this.WriteMessage;
        }

        #endregion

        #region Methods

        #region Helpers

        private void WriteMessage(LogLevel logLevel, string category, EventId eventId, string message, Exception? exception)
        {
            this._OutputHelper.WriteLine($"{this._Stopwatch.Elapsed.TotalSeconds:N3}s {category} - {logLevel}: {message}");
        }

        #endregion

        #endregion

        #region IDisposable Members
        
        public void Dispose()
        {
            this._Fixture.LoggedMessage -= this.WriteMessage;
        }

        #endregion

    }

}
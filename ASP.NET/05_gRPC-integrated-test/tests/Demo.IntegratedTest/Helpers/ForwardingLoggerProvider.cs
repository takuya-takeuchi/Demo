using Microsoft.Extensions.Logging;

namespace Demo.IntegratedTest.Helpers
{

    internal sealed class ForwardingLoggerProvider : ILoggerProvider
    {

        #region Fields

        private readonly LogMessage _LogAction;

        #endregion

        #region Constructors

        public ForwardingLoggerProvider(LogMessage logAction)
        {
            this._LogAction = logAction;
        }

        #endregion

        #region Methods

        public ILogger CreateLogger(string categoryName)
        {
            return new ForwardingLogger(categoryName, this._LogAction);
        }

        #endregion

        #region IDisposable Members

        public void Dispose()
        {
        }

        #endregion

        internal sealed class ForwardingLogger : ILogger
        {

            #region Fields

            private readonly string _CategoryName;

            private readonly LogMessage _LogAction;

            #endregion

            #region Constructors

            public ForwardingLogger(string categoryName, LogMessage logAction)
            {
                this._CategoryName = categoryName;
                this._LogAction = logAction;
            }

            #endregion

            #region Methods

            public IDisposable BeginScope<TState>(TState state)
            {
                return null!;
            }

            public bool IsEnabled(LogLevel logLevel)
            {
                return true;
            }

            public void Log<TState>(LogLevel logLevel, EventId eventId, TState state, Exception? exception, Func<TState, Exception?, string> formatter)
            {
                this._LogAction(logLevel, this._CategoryName, eventId, formatter(state, exception), exception);
            }

            #endregion

        }

    }

}
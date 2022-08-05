using Microsoft.Extensions.Logging;

namespace Demo.IntegratedTest.Helpers
{

    public delegate void LogMessage(LogLevel logLevel, string categoryName, EventId eventId, string message, Exception? exception);

}

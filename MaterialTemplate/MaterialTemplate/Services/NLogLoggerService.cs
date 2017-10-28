using MaterialTemplate.Services.Interfaces;

namespace MaterialTemplate.Services
{

    public sealed class NLogLoggerService : ILoggerService
    {

        #region Fields

        private static readonly NLog.Logger Logger = NLog.LogManager.GetLogger("MainLogger");

        #endregion

        #region ILoggerService Members

        public void Info(string message, params object[] args)
        {
            Logger.Info(message, args);
        }

        public void Fatal(string message, params object[] args)
        {
            Logger.Fatal(message, args);
        }

        public void Warn(string message, params object[] args)
        {
            Logger.Warn(message, args);
        }

        public void Debug(string message, params object[] args)
        {
            Logger.Debug(message, args);
        }

        public void Trace(string message, params object[] args)
        {
            Logger.Trace(message, args);
        }

        public void Error(string message, params object[] args)
        {
            Logger.Error(message, args);
        }

        #endregion

    }

}

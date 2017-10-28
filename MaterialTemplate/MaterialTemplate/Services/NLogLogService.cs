using System;
using System.Collections.Generic;
using MaterialTemplate.Services.Interfaces;
using NLog;

namespace MaterialTemplate.Services
{

    public sealed class NLogLogService : ILogService
    {

        #region Fields

        private readonly Logger _Logger;

        private readonly LogFactory _Factory;

        #endregion

        #region Constructors

        internal NLogLogService(Logger logger)
        {
            this._Logger = logger;
        }

        internal NLogLogService(string name, IDictionary<string, string> variables)
        {
            this._Factory = new LogFactory();
            var logger = this._Factory.GetLogger(name);
            foreach (var variable in variables)
                logger.Factory.Configuration.Variables.Add(variable.Key, variable.Value);

            this._Logger = logger;
            this._Factory.ReconfigExistingLoggers();
        }

        #endregion

        #region Properties
        #endregion

        #region Methods

        #region Overrids
        #endregion

        #region Event Handlers
        #endregion

        #region Helpers

        private void Write(LogLevel logLevel, string message)
        {
            this._Logger.Log(typeof(NLogLogService), LogEventInfo.Create(logLevel, this._Logger.Name, message));
        }

        private void Write(LogLevel logLevel, Exception exception, IFormatProvider formatProvider, string message)
        {
            this._Logger.Log(typeof(NLogLogService), LogEventInfo.Create(logLevel, this._Logger.Name, exception, formatProvider, message));
        }

        #endregion

        #endregion

        #region ILogService Members

        public void Debug(string message)
        {
            this.Write(LogLevel.Debug, message);
        }

        public void Debug(Exception exception, IFormatProvider formatProvider, string message)
        {
            this.Write(LogLevel.Debug, exception, formatProvider, message);
        }

        public void Error(string message)
        {
            this.Write(LogLevel.Error, message);
        }

        public void Error(Exception exception, IFormatProvider formatProvider, string message)
        {
            this.Write(LogLevel.Error, exception, formatProvider, message);
        }

        public void Fatal(string message)
        {
            this.Write(LogLevel.Fatal, message);
        }

        public void Fatal(Exception exception, IFormatProvider formatProvider, string message)
        {
            this.Write(LogLevel.Fatal, exception, formatProvider, message);
        }

        public void Info(string message)
        {
            this.Write(LogLevel.Info, message);
        }

        public void Info(Exception exception, IFormatProvider formatProvider, string message)
        {
            this.Write(LogLevel.Info, exception, formatProvider, message);
        }

        public void Warn(string message)
        {
            this.Write(LogLevel.Warn, message);
        }

        public void Warn(Exception exception, IFormatProvider formatProvider, string message)
        {
            this.Write(LogLevel.Warn, exception, formatProvider, message);
        }

        #endregion

    }

}

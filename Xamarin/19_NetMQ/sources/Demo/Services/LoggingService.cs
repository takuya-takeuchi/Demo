using System;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using NLog;
using NLog.Config;
using Xamarin.Forms;

using Demo.Services.Interfaces;
using NLog.Layouts;

namespace Demo.Services
{

    public sealed class LoggingService : ILoggingService
    {

        #region Fields

        private readonly Logger _Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region ILoggingService Members

        public void Initialize(Assembly assembly)
        {
            var names = assembly.GetManifestResourceNames();
            var resource = names.FirstOrDefault(s => s.EndsWith(".NLog.config"));
            if (string.IsNullOrEmpty(resource))
                throw new Exception("NLog.config is not embedded");
            var stream = assembly.GetManifestResourceStream(resource);
            if (stream == null)
                throw new Exception($"The resource '{resource}' was not loaded properly.");
            LogManager.Configuration = new XmlLoggingConfiguration(System.Xml.XmlReader.Create(stream), null);
            stream.Dispose();
        }

        public string GetCurrentLogFilePath()
        {
            if (this._Logger.Factory.Configuration.AllTargets.Count <= 0) 
                return null;

            foreach (var target in LogManager.Configuration.AllTargets)
            {
                if (!(target is NLog.Targets.FileTarget fileTarget))
                    continue;
                if (fileTarget.FileName is SimpleLayout simpleLayout)
                    return simpleLayout.FixedText;
                else
                    return fileTarget.FileName.ToString();
            }

            return null;
        }

        public void Trace(string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Trace, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Trace(Exception exception,
                          IFormatProvider formatProvider,
                          string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Trace, exception, formatProvider, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Debug(string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Debug, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Debug(Exception exception,
                          IFormatProvider formatProvider,
                          string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Debug, exception, formatProvider, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Info(string message,
                         [CallerMemberName] string callerMemberName = "",
                         [CallerFilePath] string callerFilePath = "",
                         [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Info, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Info(Exception exception,
                         IFormatProvider formatProvider,
                         string message,
                         [CallerMemberName] string callerMemberName = "",
                         [CallerFilePath] string callerFilePath = "",
                         [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Info, exception, formatProvider, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Warn(string message,
                         [CallerMemberName] string callerMemberName = "",
                         [CallerFilePath] string callerFilePath = "",
                         [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Warn, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Warn(Exception exception,
                         IFormatProvider formatProvider,
                         string message,
                         [CallerMemberName] string callerMemberName = "",
                         [CallerFilePath] string callerFilePath = "",
                         [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Warn, exception, formatProvider, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Error(string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Error, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Error(Exception exception,
                          IFormatProvider formatProvider,
                          string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Error, exception, formatProvider, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Fatal(string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Fatal, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        public void Fatal(Exception exception,
                          IFormatProvider formatProvider,
                          string message,
                          [CallerMemberName] string callerMemberName = "",
                          [CallerFilePath] string callerFilePath = "",
                          [CallerLineNumber] int callerLineNumber = 0)
        {
            this.Write(LogLevel.Fatal, exception, formatProvider, message, callerMemberName, callerFilePath, callerLineNumber);
        }

        #region Helpers
        
        private void Write(LogLevel logLevel, string message,
                           string callerMemberName,
                           string callerFilePath,
                           int callerLineNumber)
        {
            var logEventInfo = NLog.LogEventInfo.Create(logLevel, this._Logger.Name, message);
            logEventInfo.SetCallerInfo(null, callerMemberName, callerFilePath, callerLineNumber);
            this._Logger.Log(typeof(LoggingService), logEventInfo);
        }

        private void Write(LogLevel logLevel,
                           Exception exception,
                           IFormatProvider formatProvider,
                           string message,
                           string callerMemberName,
                           string callerFilePath,
                           int callerLineNumber)
        {
            var logEventInfo = NLog.LogEventInfo.Create(logLevel, this._Logger.Name, exception, formatProvider, message);
            logEventInfo.SetCallerInfo(null, callerMemberName, callerFilePath, callerLineNumber);
            this._Logger.Log(typeof(LoggingService), logEventInfo);
        }

        #endregion

        #endregion

    }

}

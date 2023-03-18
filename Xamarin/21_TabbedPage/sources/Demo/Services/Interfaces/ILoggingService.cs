using System;
using System.Reflection;
using System.Runtime.CompilerServices;

namespace Demo.Services.Interfaces
{

    public interface ILoggingService
    {

        void Initialize(Assembly assembly);

        string GetCurrentLogFilePath();

        void Trace(string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

        void Trace(Exception exception,
                   IFormatProvider formatProvider,
                   string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

        void Debug(string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

        void Debug(Exception exception,
                   IFormatProvider formatProvider,
                   string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

        void Info(string message,
                  [CallerMemberName] string callerMemberName = "",
                  [CallerFilePath] string callerFilePath = "",
                  [CallerLineNumber] int callerLineNumber = 0);

        void Info(Exception exception,
                  IFormatProvider formatProvider,
                  string message,
                  [CallerMemberName] string callerMemberName = "",
                  [CallerFilePath] string callerFilePath = "",
                  [CallerLineNumber] int callerLineNumber = 0);

        void Warn(string message,
                  [CallerMemberName] string callerMemberName = "",
                  [CallerFilePath] string callerFilePath = "",
                  [CallerLineNumber] int callerLineNumber = 0);

        void Warn(Exception exception,
                  IFormatProvider formatProvider,
                  string message,
                  [CallerMemberName] string callerMemberName = "",
                  [CallerFilePath] string callerFilePath = "",
                  [CallerLineNumber] int callerLineNumber = 0);

        void Error(string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

        void Error(Exception exception,
                   IFormatProvider formatProvider,
                   string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

        void Fatal(string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

        void Fatal(Exception exception,
                   IFormatProvider formatProvider, 
                   string message,
                   [CallerMemberName] string callerMemberName = "",
                   [CallerFilePath] string callerFilePath = "",
                   [CallerLineNumber] int callerLineNumber = 0);

    }

}

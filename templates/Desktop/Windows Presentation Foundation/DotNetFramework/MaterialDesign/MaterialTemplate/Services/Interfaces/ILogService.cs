using System;

namespace MaterialTemplate.Services.Interfaces
{

    public interface ILogService
    {


        void Debug(string message);

        void Debug(Exception exception, IFormatProvider formatProvider, string message);

        void Error(string message);

        void Error(Exception exception, IFormatProvider formatProvider, string message);

        void Fatal(string message);

        void Fatal(Exception exception, IFormatProvider formatProvider, string message);

        void Info(string message);

        void Info(Exception exception, IFormatProvider formatProvider, string message);

        void Warn(string message);

        void Warn(Exception exception, IFormatProvider formatProvider, string message);

    }

}

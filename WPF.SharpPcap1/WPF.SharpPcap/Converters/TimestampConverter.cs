using System;
using System.Globalization;
using System.Windows.Data;

namespace WPF.PacketCapture.Converters
{

    public sealed class TimestampConverter : IValueConverter
    {

        #region Methods

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return ((DateTime) value).ToString("yyyy/MM/dd hh:mm:ss.fff");
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotSupportedException();
        }

        #endregion

    }

}
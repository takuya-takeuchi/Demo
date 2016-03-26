using System;
using System.Windows.Data;

namespace OpenCV3.Converters
{
    public sealed class Int32ToStringConverter : IValueConverter
    {

        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value is int)
            {
                return ((int)value).ToString();
            }

            return null;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            var s = value as string;
            int result;
            if (s != null && int.TryParse(s, out result))
            {
                return result;
            }

            return null;
        }

    }
}

using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
    public class VoltageToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ushort voltage)
                return voltage.ToString();

            return "0";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
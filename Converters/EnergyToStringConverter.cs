using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
    public class EnergyToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ushort energy)
                return $"{energy / 10.0:F1}";

            return "0.0";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
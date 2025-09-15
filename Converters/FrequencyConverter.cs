// FrequencyConverter.cs - Modified
using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
    public class FrequencyConverter : IValueConverter
    {
        public string Format { get; set; } = "F1"; // Default format for frequency

        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value == null) return "0.0";

            // Handle different numeric types
            if (value is ushort ushortValue)
                return (ushortValue / 10.0).ToString(Format);

            if (value is double doubleValue)
                return doubleValue.ToString(Format);

            if (value is float floatValue)
                return floatValue.ToString(Format);

            if (value is int intValue)
                return intValue.ToString(Format);

            return value.ToString();
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            if (value is string strValue && double.TryParse(strValue, out double result))
            {
                return (ushort)(result * 10);
            }
            return (ushort)0;
        }
    }
}
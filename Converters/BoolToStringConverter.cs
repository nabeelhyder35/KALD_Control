// BoolToStringConverter.cs
using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
    public class BoolToStringConverter : IValueConverter
    {
        public string TrueText { get; set; } = "OK";
        public string FalseText { get; set; } = "FAULT";

        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool boolValue)
                return boolValue ? TrueText : FalseText;

            return FalseText;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
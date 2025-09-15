using System;
using Microsoft.UI.Xaml.Data;

namespace KALD_Control.Converters
{
    public class WaveformButtonTextConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool isEnabled)
            {
                return isEnabled ? "WAVE OFF" : "WAVE ON";
            }
            return "WAVE ON";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
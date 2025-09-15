using System;
using Microsoft.UI.Xaml.Data;
using Microsoft.UI.Xaml.Media;

namespace KALD_Control.Converters
{
    public class InterlockStatusToBrushConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool status)
            {
                return status ? new SolidColorBrush(Windows.UI.Color.FromArgb(255, 16, 124, 16)) : // Green for OK
                               new SolidColorBrush(Windows.UI.Color.FromArgb(255, 209, 52, 56));   // Red for fault
            }
            return new SolidColorBrush(Windows.UI.Color.FromArgb(255, 255, 140, 0)); // Orange for unknown
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
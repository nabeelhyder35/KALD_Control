using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
    public class InverseBoolToVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool flag)
                return flag ? Visibility.Collapsed : Visibility.Visible;

            return Visibility.Visible;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            if (value is Visibility visibility)
                return visibility != Visibility.Visible;

            return true;
        }
    }
}
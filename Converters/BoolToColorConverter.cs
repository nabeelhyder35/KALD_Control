using Microsoft.UI.Xaml.Data;
using Microsoft.UI.Xaml.Media;
using System;

namespace KALD_Control.Converters
{
    public class BoolToColorConverter : IValueConverter
    {
        public SolidColorBrush TrueBrush { get; set; } = new SolidColorBrush(Microsoft.UI.Colors.Green);
        public SolidColorBrush FalseBrush { get; set; } = new SolidColorBrush(Microsoft.UI.Colors.Red);

        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool boolValue)
                return boolValue ? TrueBrush : FalseBrush;

            return FalseBrush;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
            => throw new NotImplementedException();
    }
}
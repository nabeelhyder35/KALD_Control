// ConnectionButtonTextConverter.cs
using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
    public class ConnectionButtonTextConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool isConnected)
            {
                return isConnected ? "Disconnect" : "Connect";
            }
            return "Connect";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
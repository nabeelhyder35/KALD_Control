// LaserStateToColorConverter.cs
using KALD_Control.Models;
using Microsoft.UI;
using Microsoft.UI.Xaml.Data;
using Microsoft.UI.Xaml.Media;
using System;

namespace KALD_Control.Converters
{
    public class LaserStateToColorConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is not LaserStateType state) return new SolidColorBrush(Colors.Gray);

            return state switch
            {
                LaserStateType.lsrIdle => new SolidColorBrush(Colors.Gray),
                LaserStateType.lsrArming => new SolidColorBrush(Colors.Blue),
                LaserStateType.lsrArmed => new SolidColorBrush(Colors.Green),
                LaserStateType.lsrRunning => new SolidColorBrush(Colors.Red),
                LaserStateType.lsrPausing => new SolidColorBrush(Colors.Orange),
                LaserStateType.lsrPaused => new SolidColorBrush(Colors.Orange),
                LaserStateType.lsrDisarming => new SolidColorBrush(Colors.Purple),
                LaserStateType.lsrFault => new SolidColorBrush(Colors.DarkRed),
                _ => new SolidColorBrush(Colors.Gray)
            };
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
            => throw new NotImplementedException();
    }
}
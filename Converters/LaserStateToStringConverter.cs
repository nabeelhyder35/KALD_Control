// LaserStateToStringConverter.cs
using KALD_Control.Models;
using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
    public class LaserStateToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is not LaserStateType state) return "UNKNOWN";

            return state switch
            {
                LaserStateType.lsrIdle => "IDLE",
                LaserStateType.lsrArming => "ARMING",
                LaserStateType.lsrArmed => "ARMED",
                LaserStateType.lsrRunning => "FIRING",
                LaserStateType.lsrPausing => "PAUSING",
                LaserStateType.lsrPaused => "PAUSED",
                LaserStateType.lsrDisarming => "DISARMING",
                LaserStateType.lsrFault => "FAULT",
                _ => "UNKNOWN"
            };
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
            => throw new NotImplementedException();
    }
}
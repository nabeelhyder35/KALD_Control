// Converters.cs - A single file containing all value converter classes.
using KALD_Control.Models;
using Microsoft.UI;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Data;
using Microsoft.UI.Xaml.Media;
using System;

namespace KALD_Control.Converters
{
    /// <summary>
    /// Converts a boolean value to a SolidColorBrush.
    /// </summary>
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

    /// <summary>
    /// Converts a boolean value to a Brush.
    /// </summary>
    public class BoolToBrushConverter : IValueConverter
    {
        public Brush TrueBrush { get; set; } = new SolidColorBrush(Microsoft.UI.Colors.Green);
        public Brush FalseBrush { get; set; } = new SolidColorBrush(Microsoft.UI.Colors.Red);

        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return value is bool boolValue && boolValue ? TrueBrush : FalseBrush;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean value to a string.
    /// </summary>
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

    /// <summary>
    /// Converts a boolean value to Visibility.
    /// </summary>
    public class BoolToVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool flag)
                return flag ? Visibility.Visible : Visibility.Collapsed;

            return Visibility.Collapsed;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            if (value is Visibility visibility)
                return visibility == Visibility.Visible;

            return false;
        }
    }

    /// <summary>
    /// Converts a boolean value to GridLength.
    /// </summary>
    public class BoolToGridLengthConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return (value is bool boolValue && boolValue) ? new GridLength(1, GridUnitType.Star) : new GridLength(0);
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean value to an expand/collapse icon.
    /// </summary>
    public class ExpandCollapseIconConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return (value is bool boolValue && boolValue) ? "▼" : "▲";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean connection state to a button's text.
    /// </summary>
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

    /// <summary>
    /// Converts a boolean connection state to status text.
    /// </summary>
    public class ConnectionStatusConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool isConnected)
            {
                return isConnected ? "Connected" : "Disconnected";
            }
            return "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a numeric energy value to a formatted string.
    /// </summary>
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

    /// <summary>
    /// Converts a numeric frequency value to a formatted string.
    /// </summary>
    public class FrequencyConverter : IValueConverter
    {
        public string Format { get; set; } = "F1";

        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value == null) return "0.0";

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

    /// <summary>
    /// Converts a boolean value to a brush based on interlock status.
    /// </summary>
    public class InterlockStatusToBrushConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool status)
            {
                return status ? new SolidColorBrush(Windows.UI.Color.FromArgb(255, 16, 124, 16)) :
                               new SolidColorBrush(Windows.UI.Color.FromArgb(255, 209, 52, 56));
            }
            return new SolidColorBrush(Windows.UI.Color.FromArgb(255, 255, 140, 0));
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean value to "Yes" or "No".
    /// </summary>
    public class BoolToYesNoConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return value is bool isDone && isDone ? "Yes" : "No";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean value to the inverse visibility.
    /// </summary>
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

    /// <summary>
    /// Converts a LaserStateType enum to a color.
    /// </summary>
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

    /// <summary>
    /// Converts a LaserStateType enum to a string.
    /// </summary>
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

    /// <summary>
    /// Converts a TimeSpan to a user-friendly string (e.g., "10m ago").
    /// </summary>
    public class TimeSpanToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is DateTime dateTime)
            {
                var timeSpan = DateTime.Now - dateTime;
                if (timeSpan.TotalMinutes < 1)
                {
                    return "Just now";
                }
                else if (timeSpan.TotalMinutes < 60)
                {
                    return $"{timeSpan.Minutes}m ago";
                }
                else if (timeSpan.TotalHours < 24)
                {
                    return $"{timeSpan.Hours}h ago";
                }
                else
                {
                    return dateTime.ToString("MMM dd, HH:mm");
                }
            }
            return string.Empty;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a numeric value to a percentage.
    /// </summary>
    public class ValueToPercentageConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is double doubleValue && parameter is string maxValueStr)
            {
                if (double.TryParse(maxValueStr, out double maxValue) && maxValue > 0)
                {
                    return doubleValue / maxValue;
                }
            }
            return 0.0;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a numeric voltage value to a string.
    /// </summary>
    public class VoltageToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ushort voltage)
                return voltage.ToString();

            return "0";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean value to a waveform button's text.
    /// </summary>
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

    /// <summary>
    /// Converts a waveform samples array to a formatted string.
    /// </summary>
    public class WaveformToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ushort[] samples)
            {
                return string.Join(", ", samples);
            }
            return "No data";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a TriggerModeType enum to a string.
    /// </summary>
    public class TriggerModeToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is TriggerModeType triggerMode)
            {
                return triggerMode switch
                {
                    TriggerModeType.intTrigMode => "Internal Trigger",
                    TriggerModeType.extTrigMode => "External Trigger",
                    _ => triggerMode.ToString()
                };
            }
            return value?.ToString() ?? "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a ShotModeType enum to a string.
    /// </summary>
    public class ShotModeToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ShotModeType shotMode)
            {
                return shotMode switch
                {
                    ShotModeType.burstMode => "Burst Mode",
                    ShotModeType.contMode => "Continuous Mode",
                    _ => shotMode.ToString()
                };
            }
            return value?.ToString() ?? "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a ShutterModeType enum to a string.
    /// </summary>
    public class ShutterModeToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ShutterModeType shutterMode)
            {
                return shutterMode switch
                {
                    ShutterModeType.autoMode => "Automatic Mode",
                    ShutterModeType.manualMode => "Manual Mode",
                    _ => shutterMode.ToString()
                };
            }
            return value?.ToString() ?? "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a ShutterStateType enum to a string.
    /// </summary>
    public class ShutterStateToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ShutterStateType shutterState)
            {
                return shutterState switch
                {
                    ShutterStateType.shutterClosed => "Shutter Closed",
                    ShutterStateType.shutterOpened => "Shutter Opened",
                    _ => shutterState.ToString()
                };
            }
            return value?.ToString() ?? "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
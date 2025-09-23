using KALD_Control.Models;
using Microsoft.UI;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Data;
using Microsoft.UI.Xaml.Media;
using System;

namespace KALD_Control.Converters
{
    /// <summary>
    /// Converts a LaserStateType enum to a SolidColorBrush for UI display.
    /// </summary>
    public class LaserStateToColorConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is LaserStateType state)
            {
                return state switch
                {
                    LaserStateType.lsrIdle => new SolidColorBrush(Colors.Gray),
                    LaserStateType.lsrArming or LaserStateType.lsrCharging => new SolidColorBrush(Colors.Orange),
                    LaserStateType.lsrArmed => new SolidColorBrush(Colors.Yellow),
                    LaserStateType.lsrRunning => Application.Current.Resources["SystemFillColorSuccessBrush"] as SolidColorBrush ?? new SolidColorBrush(Colors.Green),
                    LaserStateType.lsrFinished or LaserStateType.lsrPaused => new SolidColorBrush(Colors.LightGreen),
                    LaserStateType.lsrPausing or LaserStateType.lsrDisarming or LaserStateType.lsrDischarging => new SolidColorBrush(Colors.Orange),
                    LaserStateType.lsrFireError => Application.Current.Resources["SystemFillColorCriticalBrush"] as SolidColorBrush ?? new SolidColorBrush(Colors.Red),
                    _ => new SolidColorBrush(Colors.Gray)
                };
            }
            return new SolidColorBrush(Colors.Gray);
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a LaserStateType enum to a human-readable string.
    /// </summary>
    public class LaserStateToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is LaserStateType state)
            {
                return state switch
                {
                    LaserStateType.lsrIdle => "Idle",
                    LaserStateType.lsrArming => "Arming",
                    LaserStateType.lsrCharging => "Charging",
                    LaserStateType.lsrArmed => "Armed",
                    LaserStateType.lsrRunning => "Running",
                    LaserStateType.lsrFinished => "Finished",
                    LaserStateType.lsrPausing => "Pausing",
                    LaserStateType.lsrPaused => "Paused",
                    LaserStateType.lsrDisarming => "Disarming",
                    LaserStateType.lsrDischarging => "Discharging",
                    LaserStateType.lsrFireError => "Fire Error",
                    _ => "Unknown"
                };
            }
            return "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a double energy value to a formatted string.
    /// </summary>
    public class EnergyToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is double energy)
            {
                return $"{energy:F2}";
            }
            return "0.00";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a TriggerModeType enum to a human-readable string.
    /// </summary>
    public class TriggerModeToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is TriggerModeType mode)
            {
                return mode switch
                {
                    TriggerModeType.intTrigMode => "Internal Trigger",
                    TriggerModeType.extTrigMode => "External Trigger",
                    _ => "Unknown"
                };
            }
            return "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a ShotModeType enum to a human-readable string.
    /// </summary>
    public class ShotModeToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ShotModeType mode)
            {
                return mode switch
                {
                    ShotModeType.burstMode => "Burst Mode",
                    ShotModeType.contMode => "Continuous Mode",
                    _ => "Unknown"
                };
            }
            return "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a ShutterModeType enum to a human-readable string.
    /// </summary>
    public class ShutterModeToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ShutterModeType mode)
            {
                return mode switch
                {
                    ShutterModeType.autoMode => "Automatic",
                    ShutterModeType.manualMode => "Manual",
                    _ => "Unknown"
                };
            }
            return "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a ShutterStateType enum to a human-readable string.
    /// </summary>
    public class ShutterStateToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ShutterStateType state)
            {
                return state switch
                {
                    ShutterStateType.shutterClosed => "Closed",
                    ShutterStateType.shutterOpened => "Opened",
                    _ => "Unknown"
                };
            }
            return "Unknown";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean value to a GridLength for UI layout.
    /// </summary>
    public class BoolToGridLengthConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool isExpanded)
            {
                return isExpanded ? new GridLength(1, GridUnitType.Star) : new GridLength(0);
            }
            return new GridLength(0);
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
            if (value is bool isExpanded)
            {
                return isExpanded ? "▲" : "▼";
            }
            return "▼";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a boolean value to a SolidColorBrush using system-defined colors.
    /// </summary>
    public class BoolToColorConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool isOk)
            {
                return isOk
                    ? Application.Current.Resources["SystemFillColorSuccessBrush"] as SolidColorBrush ?? new SolidColorBrush(Colors.Green)
                    : Application.Current.Resources["SystemFillColorCriticalBrush"] as SolidColorBrush ?? new SolidColorBrush(Colors.Red);
            }
            return new SolidColorBrush(Colors.Gray);
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
            return (value is bool isVisible && isVisible) ? Visibility.Visible : Visibility.Collapsed;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts an InterlockStatus property to a boolean value based on the property name.
    /// </summary>
    public class InterlockStatusConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is InterlockStatus interlockStatus && parameter is string interlockName)
            {
                return interlockName switch
                {
                    "ArmOK" => interlockStatus.ArmOK,
                    "RunOK" => interlockStatus.RunOK,
                    "FlowOK" => interlockStatus.FlowOK,
                    "TempOK" => interlockStatus.TempOK,
                    "DoorOK" => interlockStatus.DoorOK,
                    "CoverOK" => interlockStatus.CoverOK,
                    "DischargeTempOK" => interlockStatus.DischargeTempOK,
                    "OverVoltageOK" => interlockStatus.OverVoltageOK,
                    "OverTempOK" => interlockStatus.OverTempOK,
                    "EndOfChargeOK" => interlockStatus.EndOfChargeOK,
                    "ExternalTriggerOK" => interlockStatus.ExternalTriggerOK,
                    _ => false
                };
            }
            return false;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Inverts a boolean value.
    /// </summary>
    public class InverseBoolConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is bool b)
            {
                return !b;
            }
            return false;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
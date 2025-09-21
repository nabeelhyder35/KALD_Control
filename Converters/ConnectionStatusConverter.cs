    using global::KALD_Control.Models;
    using Microsoft.UI;
    using Microsoft.UI.Xaml;
    using Microsoft.UI.Xaml.Data;
    using Microsoft.UI.Xaml.Media;
    using System;

    namespace KALD_Control.Converters
    {      
        /// <summary>
        /// Converts connection state to status text with additional context
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
        /// Converts numeric values to progress bar percentages
        /// </summary>
        public class ValueToPercentageConverter : IValueConverter
        {
            public object Convert(object value, Type targetType, object parameter, string language)
            {
                if (value is double doubleValue && parameter is string maxValueStr)
                {
                    if (double.TryParse(maxValueStr, out double maxValue) && maxValue > 0)
                    {
                        return (doubleValue / maxValue) * 100;
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
        /// Converts timestamp to relative time string
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
    }

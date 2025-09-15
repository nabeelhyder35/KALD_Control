﻿using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Data;
using System;

namespace KALD_Control.Converters
{
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
}
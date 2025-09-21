using KALD_Control.Models;
using KALD_Control.ViewModels;
using Microsoft.UI;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;
using Microsoft.UI.Xaml.Navigation;
using System;

namespace KALD_Control
{
    public sealed partial class MainPage : Page
    {
        public MainViewModel ViewModel { get; private set; }

        public MainPage()
        {
            this.InitializeComponent();
        }

        protected override void OnNavigatedTo(NavigationEventArgs e)
        {
            base.OnNavigatedTo(e);
            if (e.Parameter is MainViewModel vm)
            {
                ViewModel = vm;
                this.DataContext = ViewModel;
            }
        }

        // Helper method for interlock status binding in XAML (temporary, consider using InterlockStatusConverter)
        public bool GetInterlockStatus(string interlockName)
        {
            return interlockName switch
            {
                "Power OK" => ViewModel.InterlockStatus.PowerOK,
                "Temperature OK" => ViewModel.InterlockStatus.TempOK,
                "Door OK" => ViewModel.InterlockStatus.DoorOK,
                "Water OK" => ViewModel.InterlockStatus.WaterOK,
                "Cover OK" => ViewModel.InterlockStatus.CoverOK,
                "Discharge Temp OK" => ViewModel.InterlockStatus.DischargeTempOK,
                "Over Voltage OK" => ViewModel.InterlockStatus.OverVoltageOK,
                "Over Temp OK" => ViewModel.InterlockStatus.OverTempOK,
                _ => false
            };
        }
    }

    // Value converter for binding InterlockStatus properties directly in XAML
    public class InterlockStatusConverter : Microsoft.UI.Xaml.Data.IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is InterlockStatus interlockStatus && parameter is string interlockName)
            {
                return interlockName switch
                {
                    "PowerOK" => interlockStatus.PowerOK,
                    "TempOK" => interlockStatus.TempOK,
                    "DoorOK" => interlockStatus.DoorOK,
                    "WaterOK" => interlockStatus.WaterOK,
                    "CoverOK" => interlockStatus.CoverOK,
                    "DischargeTempOK" => interlockStatus.DischargeTempOK,
                    "OverVoltageOK" => interlockStatus.OverVoltageOK,
                    "OverTempOK" => interlockStatus.OverTempOK,
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

    // Converter for boolean to color (used in Interlock Status bindings)
    public class BoolToColorConverter : Microsoft.UI.Xaml.Data.IValueConverter
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

    // Converter for boolean to visibility (used in Emergency Controls Strip)
    public class BoolToVisibilityConverter : Microsoft.UI.Xaml.Data.IValueConverter
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
}
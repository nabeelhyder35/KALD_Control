using KALD_Control.Models;
using KALD_Control.ViewModels;
using Microsoft.UI;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;
using Microsoft.UI.Xaml.Navigation;
using System;
using System.Linq;

namespace KALD_Control
{
    public sealed partial class MainPage : Page
    {
        public MainViewModel ViewModel { get; private set; }

        public MainPage()
        {
            this.InitializeComponent();
            // If DataContext is set here, subscribe immediately
            if (DataContext is MainViewModel vm)
                vm.PropertyChanged += ViewModel_PropertyChanged;
        }

        protected override void OnNavigatedTo(NavigationEventArgs e)
        {
            base.OnNavigatedTo(e);
            if (e.Parameter is MainViewModel vm)
            {
                ViewModel = vm;
                this.DataContext = ViewModel;
                ViewModel.PropertyChanged += ViewModel_PropertyChanged;
            }
        }

        private void ViewModel_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(MainViewModel.LogText))
            {
                LogScrollViewer?.ChangeView(null, LogScrollViewer.ScrollableHeight, null);
            }
        }

        // Helper method for interlock status binding in XAML
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

        // Helper method to convert WaveformData to a string for display
        public string GetWaveformDataString()
        {
            if (ViewModel?.DeviceData?.Waveform?.Samples == null)
                return "No waveform data";
            // Convert ushort[] to byte[] (each ushort becomes 2 bytes)
            byte[] bytes = new byte[ViewModel.DeviceData.Waveform.Samples.Length * 2];
            for (int i = 0; i < ViewModel.DeviceData.Waveform.Samples.Length; i++)
            {
                byte[] ushortBytes = BitConverter.GetBytes(ViewModel.DeviceData.Waveform.Samples[i]);
                bytes[i * 2] = ushortBytes[0];
                bytes[i * 2 + 1] = ushortBytes[1];
            }
            // Take first 20 bytes (equivalent to 10 ushorts) for display
            return BitConverter.ToString(bytes.Take(20).ToArray()).Replace("-", " ") + (bytes.Length > 20 ? " ..." : "");
        }
    }

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

    public class ChargeStateConverter : Microsoft.UI.Xaml.Data.IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            if (value is ChargeStateData chargeState && parameter is string property)
            {
                return property switch
                {
                    "MeasuredVolts" => chargeState.MeasuredVolts.ToString(),
                    "ChargeDone" => chargeState.ChargeDone,
                    _ => string.Empty
                };
            }
            return string.Empty;
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            throw new NotImplementedException();
        }
    }
}
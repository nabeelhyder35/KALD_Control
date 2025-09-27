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

        private void ToggleSwitch_Toggled(object sender, RoutedEventArgs e)
        {

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
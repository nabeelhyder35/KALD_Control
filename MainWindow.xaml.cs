using KALD_Control.Services;
using KALD_Control.ViewModels;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.UI;
using Microsoft.UI.Windowing;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;
using System;
using System.Diagnostics;
using WinRT.Interop;

namespace KALD_Control
{
    public sealed partial class MainWindow : Window
    {
        public MainViewModel ViewModel { get; }

        public MainWindow()
        {
            try
            {
                Debug.WriteLine("MainWindow constructor started");

                // Initialize XAML
                try
                {
                    this.InitializeComponent();
                    Debug.WriteLine("InitializeComponent completed");
                }
                catch (Exception initEx)
                {
                    Debug.WriteLine($"InitializeComponent failed: {initEx}");
                    throw;
                }

                // Resolve ViewModel from DI container
                ViewModel = App.Services.GetRequiredService<MainViewModel>();
                Debug.WriteLine("ViewModel resolved from DI");

                // Set DataContext
                SetDataContext();

                this.Title = "KALD Laser Control System";

                // Subscribe to closed for cleanup
                this.Closed += MainWindow_Closed;

                // Enforce minimum/initial window size
                InitWindowSize(1400, 900);

                // Check window state
                CheckWindowState();

                Debug.WriteLine("MainWindow initialization finished");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"MainWindow initialization failed: {ex}");
                InitializeManualFallback();
            }
        }

        private void SetDataContext()
        {
            if (this.Content is FrameworkElement content)
            {
                content.DataContext = ViewModel;
                Debug.WriteLine("DataContext set on root content element");
            }
            else
            {
                Debug.WriteLine("Content is not a FrameworkElement, cannot set DataContext");
            }
        }

        private void InitWindowSize(int width, int height)
        {
            try
            {
                var hwnd = WindowNative.GetWindowHandle(this);
                var windowId = Win32Interop.GetWindowIdFromWindow(hwnd);
                var appWindow = AppWindow.GetFromWindowId(windowId);

                if (appWindow != null)
                {
                    appWindow.Resize(new Windows.Graphics.SizeInt32(width, height));
                    appWindow.SetPresenter(AppWindowPresenterKind.Default);
                    Debug.WriteLine($"Window size set to {width}x{height}");
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"InitWindowSize failed: {ex}");
            }
        }

        private void MainWindow_Closed(object sender, WindowEventArgs args)
        {
            Debug.WriteLine("MainWindow closing");
            ViewModel?.Dispose();
        }

        private void ClearLog_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (ViewModel != null)
                {
                    ViewModel.LogText = string.Empty;
                    Debug.WriteLine("Log cleared");
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Error clearing log: {ex.Message}");
            }
        }

        private void InitializeManualFallback()
        {
            try
            {
                Debug.WriteLine("Initializing manual fallback UI");

                var grid = new Grid();

                grid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
                grid.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });

                var statusText = new TextBlock
                {
                    Text = "WinUI XAML failed to load. Using fallback UI.",
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                    FontSize = 16,
                    Margin = new Thickness(10)
                };

                Grid.SetRow(statusText, 0);
                grid.Children.Add(statusText);

                var contentText = new TextBlock
                {
                    Text = "Application is running in fallback mode.\nPlease check the debug output for errors.",
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                    FontSize = 14,
                    TextAlignment = TextAlignment.Center
                };

                Grid.SetRow(contentText, 1);
                grid.Children.Add(contentText);

                this.Content = grid;

                if (ViewModel != null)
                    grid.DataContext = ViewModel;

                Debug.WriteLine("Manual fallback UI created successfully");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Manual fallback also failed: {ex}");

                var fallbackText = new TextBlock
                {
                    Text = "KALD Control - Critical Error\nCheck debug output",
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                    TextAlignment = TextAlignment.Center
                };

                this.Content = fallbackText;
            }
        }

        public void CheckWindowState()
        {
            try
            {
                Debug.WriteLine("Window state check:");

                var hwnd = WindowNative.GetWindowHandle(this);
                var windowId = Win32Interop.GetWindowIdFromWindow(hwnd);
                var appWindow = AppWindow.GetFromWindowId(windowId);

                Debug.WriteLine($"- Window size: {appWindow.Size}");
                Debug.WriteLine($"- IsVisible: {appWindow.IsVisible}");

                if (this.Content is FrameworkElement fe)
                {
                    Debug.WriteLine($"- Content size: {fe.ActualWidth}x{fe.ActualHeight}");
                }

                if (ViewModel != null)
                {
                    Debug.WriteLine($"- IsConnected: {ViewModel.IsConnected}");
                    Debug.WriteLine($"- SelectedPort: {ViewModel.SelectedPort ?? "null"}");
                    Debug.WriteLine($"- Ports: {ViewModel.AvailablePorts?.Count ?? 0}");
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Window state check failed: {ex.Message}");
            }
        }
    }
}
using KALD_Control.ViewModels;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.UI;
using Microsoft.UI.Windowing;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;
using Microsoft.UI.Xaml.Shapes;
using Microsoft.VisualBasic;
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

                // Initialize XAML first - use try/catch in case generated code is missing
                try
                {
                    this.InitializeComponent();
                    Debug.WriteLine("InitializeComponent completed");
                }
                catch (Exception initEx)
                {
                    Debug.WriteLine($"InitializeComponent failed: {initEx}");
                    // We'll handle this in the main catch block
                    throw;
                }

                // Resolve ViewModel from DI container
                ViewModel = App.Services.GetRequiredService<MainViewModel>();
                Debug.WriteLine("ViewModel resolved from DI");

                // Set DataContext - use safe access pattern
                SetDataContext();

                this.Title = "KALD Laser Control System";

                // subscribe to closed for cleanup
                this.Closed += MainWindow_Closed;

                // enforce minimum/initial window size
                InitWindowSize(1400, 900);

                // existing helpers
                TestCommandBinding();
                SetupFallbackClickHandlers();
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
            // Safe way to set DataContext - Window doesn't have DataContext in WinUI 3
            // Instead, set it on the root content element
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

        // Add this event handler for the Clear Log button
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

                var connectionPanel = new StackPanel
                {
                    Orientation = Orientation.Horizontal,
                    Padding = new Thickness(10),
                    Background = new SolidColorBrush(Colors.LightGray)
                };

                var statusText = new TextBlock
                {
                    Text = "WinUI XAML failed to load. Using fallback UI.",
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                    FontSize = 16,
                    Margin = new Thickness(10)
                };

                connectionPanel.Children.Add(statusText);
                Grid.SetRow(connectionPanel, 0);
                grid.Children.Add(connectionPanel);

                var contentText = new TextBlock
                {
                    Text = "Application is running in fallback mode.\nPlease check the debug output for errors.",
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                    FontSize = 14,
                    TextAlignment = TextAlignment.Center
                };

                var contentGrid = new Grid();
                contentGrid.Children.Add(contentText);
                Grid.SetRow(contentGrid, 1);
                grid.Children.Add(contentGrid);

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

        #region Test / helper methods

        public void TestUIElements()
        {
            try
            {
                Debug.WriteLine("Testing UI elements...");

                var testButton = new Button
                {
                    Content = "Test Button",
                    Width = 120,
                    Height = 40,
                    Margin = new Thickness(10)
                };

                testButton.Click += (s, e) =>
                {
                    Debug.WriteLine("Test button clicked successfully!");
                    ShowStatusMessage("Test button clicked!");
                };

                if (this.Content is Grid mainGrid)
                {
                    mainGrid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
                    Grid.SetRow(testButton, mainGrid.RowDefinitions.Count - 1);

                    var testPanel = new StackPanel
                    {
                        Orientation = Orientation.Horizontal,
                        HorizontalAlignment = HorizontalAlignment.Center,
                        VerticalAlignment = VerticalAlignment.Bottom,
                        Margin = new Thickness(10)
                    };
                    testPanel.Children.Add(testButton);
                    Grid.SetRow(testPanel, mainGrid.RowDefinitions.Count - 1);
                    mainGrid.Children.Add(testPanel);
                }
                else if (this.Content is Panel panel)
                {
                    panel.Children.Add(testButton);
                }

                Debug.WriteLine("UI test completed successfully");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"UI test failed: {ex.Message}");
            }
        }

        public void TestCommandBinding()
        {
            try
            {
                Debug.WriteLine("Testing command binding...");

                if (ViewModel == null || ViewModel.ConnectCommand == null)
                {
                    Debug.WriteLine("ViewModel or ConnectCommand is null!");
                    return;
                }

                Debug.WriteLine($"CanExecute: {ViewModel.ConnectCommand.CanExecute(null)}");

                if (this.Content is FrameworkElement content)
                {
                    var connectButton = FindDescendant<Button>(content);
                    if (connectButton != null)
                    {
                        Debug.WriteLine($"Found button: {connectButton.Content}");
                        Debug.WriteLine($"Button command: {connectButton.Command != null}");
                        Debug.WriteLine($"Button enabled: {connectButton.IsEnabled}");

                        if (connectButton.Content?.ToString() == "Connect" &&
                            connectButton.Command?.CanExecute(null) == true)
                        {
                            Debug.WriteLine("Executing ConnectCommand...");
                            connectButton.Command.Execute(null);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Command binding test failed: {ex.Message}");
            }
        }

        private void SetupFallbackClickHandlers()
        {
            try
            {
                Debug.WriteLine("Setting up fallback click handlers...");

                if (this.Content is FrameworkElement content)
                {
                    var connectButton = FindDescendant<Button>(content);
                    if (connectButton != null && connectButton.Content?.ToString() == "Connect")
                    {
                        if (connectButton.Command == null && ViewModel?.ConnectCommand != null)
                        {
                            connectButton.Click += (s, e) =>
                            {
                                Debug.WriteLine("Connect button clicked (fallback handler)");
                                if (ViewModel.ConnectCommand.CanExecute(null))
                                    ViewModel.ConnectCommand.Execute(null);
                            };

                            Debug.WriteLine("Fallback click handler setup completed");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Fallback click handler setup failed: {ex.Message}");
            }
        }

        public void CheckWindowState()
        {
            try
            {
                Debug.WriteLine($"Window state check:");

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

        // Find child controls
        public T FindDescendant<T>(DependencyObject element) where T : DependencyObject
        {
            if (element == null) return null;

            if (element is T match) return match;

            int count = VisualTreeHelper.GetChildrenCount(element);
            for (int i = 0; i < count; i++)
            {
                var child = VisualTreeHelper.GetChild(element, i);
                var result = FindDescendant<T>(child);
                if (result != null) return result;
            }
            return null;
        }

        public void ShowStatusMessage(string message)
        {
            Debug.WriteLine($"Status: {message}");

            if (this.Content is Grid grid)
            {
                foreach (var child in grid.Children)
                {
                    if (child is TextBlock tb && tb.Text.Contains("Status"))
                    {
                        tb.Text = message;
                        return;
                    }
                }

                var statusText = new TextBlock
                {
                    Text = message,
                    Margin = new Thickness(10),
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Bottom
                };

                if (grid.RowDefinitions.Count > 0)
                {
                    Grid.SetRow(statusText, grid.RowDefinitions.Count - 1);
                    grid.Children.Add(statusText);
                }
            }
        }

        #endregion

        private void MenuButton_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Menu button clicked");
        }
    }
}
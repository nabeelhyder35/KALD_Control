using KALD_Control.Services;
using KALD_Control.ViewModels;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.UI;
using Microsoft.UI.Windowing;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using System;
using WinRT.Interop;

namespace KALD_Control
{
    public sealed partial class MainWindow : Window
    {
        public MainViewModel ViewModel { get; }
        private AppWindow _appWindow;
        private Frame _rootFrame;

        public MainWindow()
        {
            this.InitializeComponent();

            try
            {
                // Get ViewModel from DI container with fallback
                ViewModel = App.Services?.GetService<MainViewModel>();
                if (ViewModel == null)
                {
                    // Fallback creation if DI fails
                    var deviceManager = new DeviceManager(null);
                    var dispatcherQueue = Microsoft.UI.Dispatching.DispatcherQueue.GetForCurrentThread();
                    ViewModel = new MainViewModel(deviceManager, null, dispatcherQueue);
                }
            }
            catch (Exception ex)
            {
                // Emergency fallback
                var deviceManager = new DeviceManager(null);
                var dispatcherQueue = Microsoft.UI.Dispatching.DispatcherQueue.GetForCurrentThread();
                ViewModel = new MainViewModel(deviceManager, null, dispatcherQueue);
                System.Diagnostics.Debug.WriteLine($"DI failed: {ex.Message}");
            }

            // Create and set up the frame
            _rootFrame = new Frame();
            this.Content = _rootFrame;

            // Configure window
            ConfigureWindow();

            // Set up event handlers
            this.Closed += MainWindow_Closed;
            this.Activated += MainWindow_Activated;
        }

        private void MainWindow_Activated(object sender, WindowActivatedEventArgs args)
        {
            // Only perform initialization when the window is fully active
            if (args.WindowActivationState == WindowActivationState.Deactivated)
            {
                return;
            }

            // Navigate to the main page if not already done
            if (_rootFrame.Content == null)
            {
                _rootFrame.Navigate(typeof(MainPage), ViewModel);
            }

            // Set up keyboard event handling
            if (this.CoreWindow != null)
            {
                this.CoreWindow.KeyDown += MainWindow_KeyDown;
            }

            // Unsubscribe from this event to prevent multiple navigations
            this.Activated -= MainWindow_Activated;
        }

        private void ConfigureWindow()
        {
            try
            {
                // Get window handle and AppWindow object
                var hWnd = WindowNative.GetWindowHandle(this);
                var windowId = Win32Interop.GetWindowIdFromWindow(hWnd);
                _appWindow = AppWindow.GetFromWindowId(windowId);

                // Set window properties
                if (_appWindow is not null)
                {
                    // Set minimum and default size
                    _appWindow.Resize(new Windows.Graphics.SizeInt32(1200, 800));

                    // Set title bar
                    var titleBar = _appWindow.TitleBar;
                    titleBar.ExtendsContentIntoTitleBar = true;
                    titleBar.ButtonBackgroundColor = Colors.Transparent;
                    titleBar.ButtonInactiveBackgroundColor = Colors.Transparent;

                    // Set custom title bar colors to match theme
                    titleBar.BackgroundColor = Colors.Transparent;
                    titleBar.ForegroundColor = Colors.White;
                    titleBar.InactiveBackgroundColor = Colors.Transparent;
                    titleBar.InactiveForegroundColor = Colors.Gray;

                    // Title bar button colors
                    titleBar.ButtonForegroundColor = Colors.White;
                    titleBar.ButtonHoverBackgroundColor = Microsoft.UI.Colors.DarkGray;
                    titleBar.ButtonHoverForegroundColor = Colors.White;
                    titleBar.ButtonPressedBackgroundColor = Microsoft.UI.Colors.Gray;
                    titleBar.ButtonPressedForegroundColor = Colors.White;

                    // Apply initial theme colors
                    UpdateTitleBarColors();
                }

                // Set window icon if available
                try
                {
                    // AppWindow.SetIcon("Assets/Square44x44Logo.scale-200.png");
                }
                catch
                {
                    // Icon file not found, continue without it
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Window configuration failed: {ex.Message}");
            }
        }

        private void UpdateTitleBarColors()
        {
            if (_appWindow?.TitleBar is not null)
            {
                var titleBar = _appWindow.TitleBar;

                // Update title bar colors based on current theme
                if (Application.Current.RequestedTheme == ApplicationTheme.Dark)
                {
                    titleBar.ButtonForegroundColor = Colors.White;
                    titleBar.ButtonHoverBackgroundColor = Microsoft.UI.Colors.DarkGray;
                }
                else
                {
                    titleBar.ButtonForegroundColor = Colors.Black;
                    titleBar.ButtonHoverBackgroundColor = Microsoft.UI.Colors.LightGray;
                }
            }
        }

        private void MainWindow_Closed(object sender, WindowEventArgs e)
        {
            // Dispose of ViewModel to clean up resources
            ViewModel?.Dispose();
        }

        // Handle keyboard shortcuts for power users
        private void MainWindow_KeyDown(Windows.UI.Core.CoreWindow sender, Windows.UI.Core.KeyEventArgs e)
        {
            // Get the current page to access UI elements if needed
            if (_rootFrame.Content is MainPage page)
            {
                // Handle keyboard shortcuts
                var ctrlPressed = sender.GetKeyState(Windows.System.VirtualKey.Control).HasFlag(Windows.UI.Core.CoreVirtualKeyStates.Down);

                if (ctrlPressed)
                {
                    switch (e.VirtualKey)
                    {
                        case Windows.System.VirtualKey.L:
                            // Ctrl+L - Show log (if you still want this functionality)
                            // page.ShowLog(); // Commented out since log is always visible now
                            e.Handled = true;
                            break;

                        case Windows.System.VirtualKey.R:
                            // Ctrl+R - Refresh ports
                            if (ViewModel.RefreshPortsCommand.CanExecute(null))
                            {
                                ViewModel.RefreshPortsCommand.Execute(null);
                            }
                            e.Handled = true;
                            break;

                        case Windows.System.VirtualKey.D:
                            // Ctrl+D - Debug info
                            if (ViewModel.DebugTestCommand.CanExecute(null))
                            {
                                ViewModel.DebugTestCommand.Execute(null);
                            }
                            e.Handled = true;
                            break;
                    }
                }
                else
                {
                    // Emergency shortcuts without Ctrl modifier for safety
                    switch (e.VirtualKey)
                    {
                        case Windows.System.VirtualKey.Escape:
                            // ESC - Emergency stop
                            if (ViewModel.StopCommand.CanExecute(null))
                            {
                                ViewModel.StopCommand.Execute(null);
                            }
                            e.Handled = true;
                            break;

                        case Windows.System.VirtualKey.F1:
                            // F1 - Show system status
                            if (ViewModel.RequestStatusCommand.CanExecute(null))
                            {
                                ViewModel.RequestStatusCommand.Execute(null);
                            }
                            e.Handled = true;
                            break;
                    }
                }
            }
        }

        // Show confirmation dialog for critical operations
        private async System.Threading.Tasks.Task<bool> ShowConfirmationDialog(string title, string message, string primaryButtonText = "Confirm", string secondaryButtonText = "Cancel")
        {
            var dialog = new ContentDialog
            {
                Title = title,
                Content = message,
                PrimaryButtonText = primaryButtonText,
                SecondaryButtonText = secondaryButtonText,
                DefaultButton = ContentDialogButton.Secondary,
                XamlRoot = _rootFrame.XamlRoot
            };

            var result = await dialog.ShowAsync();
            return result == ContentDialogResult.Primary;
        }

        // Enhanced error handling with user-friendly messages
        private async void ShowErrorDialog(string title, string message, Exception ex = null)
        {
            var detailedMessage = message;
            if (ex != null)
            {
                detailedMessage += $"\n\nTechnical details: {ex.Message}";
            }

            var dialog = new ContentDialog
            {
                Title = title,
                Content = detailedMessage,
                CloseButtonText = "OK",
                XamlRoot = _rootFrame.XamlRoot
            };

            await dialog.ShowAsync();
        }

        // Method to handle system notifications
        private void ShowNotification(string title, string message, InfoBarSeverity severity = InfoBarSeverity.Informational)
        {
            // In a full implementation, this would show a toast notification or info bar
            // For now, we'll add it to the log
            var timestamp = DateTime.Now.ToString("HH:mm:ss");
            ViewModel.LogText += $"[{timestamp}] {severity}: {title} - {message}{Environment.NewLine}";
        }
    }
}
using KALD_Control.Services;
using KALD_Control.ViewModels;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Microsoft.UI;
using Microsoft.UI.Windowing;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using System;
using System.Diagnostics;
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
                ViewModel = App.Services?.GetService<MainViewModel>();
                if (ViewModel == null)
                {
                    var loggerFactory = App.Services?.GetService<ILoggerFactory>() ?? new LoggerFactory();
                    var logger = loggerFactory.CreateLogger<DeviceManager>();
                    var deviceManager = new DeviceManager(logger);
                    var dispatcherQueue = Microsoft.UI.Dispatching.DispatcherQueue.GetForCurrentThread();
                    ViewModel = new MainViewModel(deviceManager, loggerFactory, dispatcherQueue);
                    Debug.WriteLine("MainViewModel initialized via fallback.");
                }
                else
                {
                    Debug.WriteLine("MainViewModel initialized via DI.");
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Failed to initialize MainViewModel: {ex.Message}");
                var loggerFactory = new LoggerFactory();
                var logger = loggerFactory.CreateLogger<DeviceManager>();
                var deviceManager = new DeviceManager(logger);
                var dispatcherQueue = Microsoft.UI.Dispatching.DispatcherQueue.GetForCurrentThread();
                ViewModel = new MainViewModel(deviceManager, loggerFactory, dispatcherQueue);
            }

            _rootFrame = new Frame();
            this.Content = _rootFrame;

            ConfigureWindow();

            this.Closed += MainWindow_Closed;
            this.Activated += MainWindow_Activated;


        }

        private void MainWindow_Activated(object sender, WindowActivatedEventArgs args)
        {
            try
            {
                if (args.WindowActivationState == WindowActivationState.Deactivated)
                {
                    Debug.WriteLine("Window deactivated, skipping navigation.");
                    return;
                }

                if (_rootFrame.Content == null)
                {
                    Debug.WriteLine("Navigating to MainPage.");
                    _rootFrame.Navigate(typeof(MainPage), ViewModel);
                }
                else
                {
                    Debug.WriteLine("Frame content already set.");
                }

                if (this.CoreWindow != null)
                {
                    this.CoreWindow.KeyDown += MainWindow_KeyDown;
                    Debug.WriteLine("KeyDown event handler attached.");
                }

                this.Activated -= MainWindow_Activated;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"MainWindow_Activated failed: {ex.Message}");
                LogError("Initialization Error", "Failed to load the main page.", ex);
            }
        }

        private void ConfigureWindow()
        {
            try
            {
                var hWnd = WindowNative.GetWindowHandle(this);
                var windowId = Win32Interop.GetWindowIdFromWindow(hWnd);
                _appWindow = AppWindow.GetFromWindowId(windowId);

                if (_appWindow != null)
                {
                    // Replace this line:
                    // var displayArea = DisplayArea.GetFromWindowId(windowId);

                    // With the following line, specifying a fallback value (e.g., DisplayAreaFallback.Primary):
                    var displayArea = DisplayArea.GetFromWindowId(windowId, DisplayAreaFallback.Primary);
                    if (displayArea != null)
                    {
                        // Set to full screen or maximized
                        this.AppWindow.SetPresenter(AppWindowPresenterKind.Default);
                        this.AppWindow.MoveAndResize(displayArea.WorkArea);

                        // Alternatively, you can maximize the window:
                        // this.AppWindow.SetPresenter(AppWindowPresenterKind.Maximized);
                    }
                    else
                    {
                        // Fallback to a large size
                        _appWindow.Resize(new Windows.Graphics.SizeInt32(1600, 1000));
                    }

                    // Rest of your title bar configuration...
                    var titleBar = _appWindow.TitleBar;
                    titleBar.ExtendsContentIntoTitleBar = true;
                    titleBar.ButtonBackgroundColor = Colors.Transparent;
                    titleBar.ButtonInactiveBackgroundColor = Colors.Transparent;
                    titleBar.BackgroundColor = Colors.Transparent;
                    titleBar.ForegroundColor = Colors.White;
                    titleBar.InactiveBackgroundColor = Colors.Transparent;
                    titleBar.InactiveForegroundColor = Colors.Gray;
                    titleBar.ButtonForegroundColor = Colors.White;
                    titleBar.ButtonHoverBackgroundColor = Microsoft.UI.Colors.DarkGray;
                    titleBar.ButtonHoverForegroundColor = Colors.White;
                    titleBar.ButtonPressedBackgroundColor = Microsoft.UI.Colors.Gray;
                    titleBar.ButtonPressedForegroundColor = Colors.White;

                    UpdateTitleBarColors();
                    Debug.WriteLine("Window configured for full screen.");
                }
                else
                {
                    Debug.WriteLine("AppWindow is null.");
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Window configuration failed: {ex.Message}");
                LogError("Window Configuration", "Failed to configure window.", ex);

                // Fallback: just maximize the window
                this.AppWindow.SetPresenter(AppWindowPresenterKind.FullScreen);
            }
        }
        private void UpdateTitleBarColors()
        {
            if (_appWindow?.TitleBar is not null)
            {
                var titleBar = _appWindow.TitleBar;

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
                Debug.WriteLine("Title bar colors updated.");
            }
        }

        private void MainWindow_Closed(object sender, WindowEventArgs e)
        {
            try
            {
                ViewModel?.Dispose();
                Debug.WriteLine("ViewModel disposed.");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Error disposing ViewModel: {ex.Message}");
                LogError("Disposal Error", "Error disposing ViewModel.", ex);
            }
        }

        private void MainWindow_KeyDown(Windows.UI.Core.CoreWindow sender, Windows.UI.Core.KeyEventArgs e)
        {
            if (_rootFrame.Content is MainPage page)
            {
                var ctrlPressed = sender.GetKeyState(Windows.System.VirtualKey.Control).HasFlag(Windows.UI.Core.CoreVirtualKeyStates.Down);

                if (ctrlPressed)
                {
                    switch (e.VirtualKey)
                    {
                        case Windows.System.VirtualKey.L:
                            if (ViewModel.ClearLogsCommand.CanExecute(null))
                            {
                                ViewModel.ClearLogsCommand.Execute(null);
                                Debug.WriteLine("ClearLogsCommand executed.");
                            }
                            e.Handled = true;
                            break;

                        case Windows.System.VirtualKey.R:
                            if (ViewModel.RefreshPortsCommand.CanExecute(null))
                            {
                                ViewModel.RefreshPortsCommand.Execute(null);
                                Debug.WriteLine("RefreshPortsCommand executed.");
                            }
                            e.Handled = true;
                            break;

                        case Windows.System.VirtualKey.W:
                            ViewModel.WaveformEnabled = !ViewModel.WaveformEnabled;
                            Debug.WriteLine($"WaveformEnabled toggled to {ViewModel.WaveformEnabled}.");
                            e.Handled = true;
                            break;
                    }
                }
                else
                {
                    switch (e.VirtualKey)
                    {
                        case Windows.System.VirtualKey.Escape:
                            if (ViewModel.StopCommand.CanExecute(null))
                            {
                                ViewModel.StopCommand.Execute(null);
                                Debug.WriteLine("StopCommand executed.");
                            }
                            e.Handled = true;
                            break;

                        case Windows.System.VirtualKey.F5:
                            if (ViewModel.IsConnected)
                            {
                                if (ViewModel.DisconnectCommand.CanExecute(null))
                                {
                                    ViewModel.DisconnectCommand.Execute(null);
                                    Debug.WriteLine("DisconnectCommand executed.");
                                }
                            }
                            else
                            {
                                if (ViewModel.ConnectCommand.CanExecute(null))
                                {
                                    ViewModel.ConnectCommand.Execute(null);
                                    Debug.WriteLine("ConnectCommand executed.");
                                }
                            }
                            e.Handled = true;
                            break;
                    }
                }
            }
        }

        private void LogError(string title, string message, Exception ex = null)
        {
            var timestamp = DateTime.Now.ToString("HH:mm:ss");
            var errorMessage = $"[{timestamp}] ERROR: {title} - {message}";

            if (ex != null)
            {
                errorMessage += $"\nTechnical details: {ex.Message}";
                if (ex.StackTrace != null)
                {
                    errorMessage += $"\nStack trace: {ex.StackTrace}";
                }
            }

            Debug.WriteLine(errorMessage);

            // Also log to the ViewModel's log if available
            if (ViewModel != null)
            {
                ViewModel.LogText += errorMessage + Environment.NewLine;
            }
        }

        private void ShowNotification(string title, string message, InfoBarSeverity severity = InfoBarSeverity.Informational)
        {
            var timestamp = DateTime.Now.ToString("HH:mm:ss");
            if (ViewModel != null)
            {
                ViewModel.LogText += $"[{timestamp}] {severity}: {title} - {message}{Environment.NewLine}";
            }
        }
    }
}
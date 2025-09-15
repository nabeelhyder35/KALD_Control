using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using KALD_Control.ViewModels;
using KALD_Control.Services;
using KALD_Control.Models;
using System;
using System.Diagnostics;

namespace KALD_Control
{
    public partial class App : Application
    {
        public static IServiceProvider Services { get; private set; }
        public static new App Current => (App)Application.Current;
        private Window _mainWindow;

        public App()
        {
            // InitializeComponent() is called automatically by WinUI
            this.UnhandledException += OnUnhandledException;

            // Configure services
            Services = ConfigureServices();
        }

        private static IServiceProvider ConfigureServices()
        {
            var services = new ServiceCollection();

            // Add Logging
            services.AddLogging(configure =>
            {
#if DEBUG
                configure.AddDebug();
                configure.SetMinimumLevel(LogLevel.Debug);
#else
                configure.SetMinimumLevel(LogLevel.Information);
#endif
            });

            // Add Services
            services.AddSingleton<DeviceManager>();

            // Add ViewModels
            services.AddTransient<MainViewModel>();

            // Add Models
            services.AddTransient<DeviceData>();

            return services.BuildServiceProvider();
        }

        protected override void OnLaunched(LaunchActivatedEventArgs args)
        {
            // Create and activate the main window
            _mainWindow = new MainWindow();
            _mainWindow.Activate();
        }

        private void OnUnhandledException(object sender, Microsoft.UI.Xaml.UnhandledExceptionEventArgs e)
        {
            try
            {
                var logger = Services?.GetService<ILogger<App>>();
                logger?.LogError(e.Exception, $"Unhandled exception: {e.Exception.Message}");
            }
            catch
            {
                // Fallback logging if DI container is not available
                Debug.WriteLine($"Unhandled exception: {e.Exception}");
            }

            e.Handled = true;

            // Only show message dialog if we have a valid window
            if (_mainWindow != null)
            {
                ShowMessage("Error", $"An error occurred: {e.Exception.Message}");
            }
        }

        public void ShowMessage(string title, string message, string buttonText = "OK")
        {
            // Get the XamlRoot from the main window
            var xamlRoot = _mainWindow?.Content?.XamlRoot;

            if (xamlRoot != null)
            {
                var dialog = new ContentDialog
                {
                    Title = title,
                    Content = message,
                    PrimaryButtonText = buttonText,
                    XamlRoot = xamlRoot
                };
                _ = dialog.ShowAsync();
            }
            else
            {
                // Fallback if XamlRoot is not available
                Debug.WriteLine($"Message dialog could not be shown: {title} - {message}");
            }
        }
    }
}
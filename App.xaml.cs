using KALD_Control.Services;
using KALD_Control.ViewModels;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Microsoft.UI.Dispatching;
using Microsoft.UI.Xaml;
using System;

namespace KALD_Control
{
    public partial class App : Application
    {
        public static IServiceProvider Services { get; private set; }
        private Window m_window;

        public App()
        {
            this.InitializeComponent();
            this.UnhandledException += App_UnhandledException;
        }

        private void App_UnhandledException(object sender, Microsoft.UI.Xaml.UnhandledExceptionEventArgs e)
        {
            // Log unhandled exceptions
            System.Diagnostics.Debug.WriteLine($"Unhandled exception: {e.Exception}");
            e.Handled = true;
        }

        protected override void OnLaunched(LaunchActivatedEventArgs args)
        {
            // Configure DI here, after the app is launched and DispatcherQueue is available
            ConfigureServices();

            m_window = new MainWindow();
            m_window.Activate();
        }

        private void ConfigureServices()
        {
            var serviceCollection = new ServiceCollection();

            // Add logging
            serviceCollection.AddLogging(builder =>
            {
                builder.AddDebug();
                builder.AddConsole();
                builder.SetMinimumLevel(LogLevel.Information);
            });

            // Add services - use proper factory methods to handle dependencies
            serviceCollection.AddSingleton<DeviceManager>(provider =>
            {
                var logger = provider.GetService<ILogger<DeviceManager>>();
                return new DeviceManager(logger);
            });

            serviceCollection.AddSingleton<MainViewModel>(provider =>
            {
                var deviceManager = provider.GetService<DeviceManager>();
                var logger = provider.GetService<ILogger<MainViewModel>>();
                var dispatcherQueue = DispatcherQueue.GetForCurrentThread();
                return new MainViewModel(deviceManager, logger, dispatcherQueue);
            });

            // Build the service provider
            Services = serviceCollection.BuildServiceProvider();
        }
    }
}
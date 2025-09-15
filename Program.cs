using Microsoft.UI.Dispatching;
using Microsoft.UI.Xaml;
using Microsoft.Windows.ApplicationModel.DynamicDependency;
using System;
using System.Threading;

namespace KALD_Control
{
    internal static class Program
    {
        [System.Runtime.InteropServices.DllImport("Microsoft.ui.xaml.dll")]
        private static extern void XamlCheckProcessRequirements();

        [System.STAThreadAttribute]
        static void Main(string[] args)
        {
            XamlCheckProcessRequirements();

            WinRT.ComWrappersSupport.InitializeComWrappers();

            // Bootstrap Windows App SDK (for version 1.7.x)
            bool success = Bootstrap.TryInitialize(0x00010007, out int hr);
            if (!success)
            {
                // Handle failure (e.g., runtime not installed or version mismatch)
                Console.WriteLine($"Bootstrap failed with HRESULT: 0x{hr:X8}"); // Or log to file/debug
                Environment.Exit(hr);
                return;
            }

            try
            {
                // Start the WinUI 3 application
                Application.Start((p) =>
                {
                    var context = new DispatcherQueueSynchronizationContext(
                        DispatcherQueue.GetForCurrentThread());
                    SynchronizationContext.SetSynchronizationContext(context);

                    new App();
                });
            }
            finally
            {
                // Clean up bootstrap resources
                Bootstrap.Shutdown();
            }
        }
    }
}
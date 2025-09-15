using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KALD_Control
{
    public static class Constants
    {
        // Window settings
        public const int DefaultWindowWidth = 1400;
        public const int DefaultWindowHeight = 900;
        public const int MinimumWindowWidth = 1200;
        public const int MinimumWindowHeight = 800;

        // Voltage settings
        public const int MaxVoltage = 1000;
        public const int DacResolution = 4095;
        public const float DacReferenceVoltage = 3.3f;

        // Serial settings
        public const int DefaultBaudRate = 115200;
        public const int ReadTimeoutMs = 1000;
        public const int WriteTimeoutMs = 1000;

        // Pulse settings
        public const double MinFrequency = 0.1;
        public const double MaxFrequency = 100.0;
        public const int MinPulseWidth = 10;
        public const int MaxPulseWidth = 5000;
        public const uint MinTotalShots = 1;
        public const uint MaxTotalShots = 1000000;

        // Digital output channels
        public const int HvSupplyChannel = 16;
    }
}
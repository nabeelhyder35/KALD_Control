using System;

// Enums.cs - Fixed version
namespace KALD_Control.Models
{
    public enum LaserStateType : byte
    {
        lsrIdle = 0,
        lsrArming = 1,
        lsrArmed = 2,
        lsrRunning = 3,
        lsrPausing = 4,
        lsrPaused = 5,
        lsrDisarming = 6,
        lsrFault = 7,
        lsrCalibrating = 8,
        lsrTesting = 9
    }

    public enum ShotModeType : byte
    {
        Single = 0,
        Burst = 1,
        Continuous = 2
    }

    public enum TriggerModeType : byte
    {
        Internal = 0,
        External = 1
    }

    public enum ShutterModeType : byte
    {
        Manual = 0,
        Auto = 1
    }

    public enum ShutterStateType : byte
    {
        Closed = 0,
        Open = 1
    }

    // Rx commands - From LCD to FPGA (matches CSV exactly)
    public enum LcdRxCommand : byte
    {
        lcdRxBadCmd = 0xFE,           // -2 as byte
        lcdRxNoCmd = 0xFF,            // -1 as byte
        lcdRxFPGABadCmd = 0x00,       // 0
        lcdRxLsrPulseConfig = 0x01,   // 1
        lcdRxLsrState = 0x02,         // 2
        lcdRxIntMask = 0x03,          // 3
        lcdRxWaveState = 0x04,        // 4
        lcdRxLsrDelays = 0x05,        // 5
        lcdRxLsrCal = 0x06,           // 6
        lcdRxLsrVolts = 0x07,         // 7
        lcdRxLsrChargeCancel = 0x08,  // 8
        lcdRxShutterConfig = 0x09,    // 9
        lcdRxSoftStartConfig = 0x0A,  // 10
        lcdRxLsrShots = 0x0B,         // 11
        lcdRxRunStatus = 0x0C,        // 12
        lcdRxIntStatus = 0x0D,        // 13
        lcdRxWaveform = 0x0E,         // 14
        lcdRxDiscovery = 0x0F,        // 15
        lcdRxLsrChargeState = 0x10,   // 16
        lcdRxLsrChargeVolt = 0x11,    // 17
        lcdRxDigitalIO = 0x12,        // 18 (added for missing reference)
        lcdRxSystemReset = 0x13       // 19 (added for missing reference)
    }

    // Tx commands - From FPGA to LCD (matches CSV exactly)
    public enum LcdTxCommand : byte
    {
        lcdTxBadCmd = 0x00,           // 0
        lcdTxLsrPulseConfig = 0x01,   // 1
        lcdTxLsrState = 0x02,         // 2
        lcdTxLsrCount = 0x03,         // 3
        lcdTxLsrRunStatus = 0x04,     // 4
        lcdTxLsrIntStatus = 0x05,     // 5
        lcdTxLsrIntMask = 0x06,       // 6
        lcdTxLsrWaveform = 0x07,      // 7
        lcdTxDiscovery = 0x08,        // 8
        lcdTxLsrVolts = 0x09,         // 9
        lcdTxLsrChargeState = 0x0A,   // 10
        lcdTxLsrChargeVolts = 0x0B,   // 11
        lcdTxShutterConfig = 0x0C,    // 12
        lcdTxSoftStartConfig = 0x0D,  // 13
        lcdTxLsrCal = 0x0E,           // 14 (added for missing reference)
        lcdTxLsrDelays = 0x0F,        // 15 (added for missing reference)
        lcdTxDigitalIO = 0x10         // 16 (added for missing reference)
    }

    // FPGA command data length constants (updated to match CSV)
    public static class FpgaCommandLengths
    {
        public const int LSR_PULSE_CONFIG = 14;    // lcdRxLsrPulseConfig
        public const int LSR_STATE = 1;            // lcdRxLsrState
        public const int INT_MASK = 1;             // lcdRxIntMask
        public const int WAVE_STATE = 1;           // lcdRxWaveState
        public const int LSR_DELAYS = 4;           // lcdRxLsrDelays
        public const int LSR_CAL = 2;              // lcdRxLsrCal
        public const int LSR_VOLTS = 2;            // lcdRxLsrVolts
        public const int LSR_CHARGE_CANCEL = 1;    // lcdRxLsrChargeCancel
        public const int SHUTTER_CONFIG = 2;       // lcdRxShutterConfig
        public const int SOFT_START_CONFIG = 7;    // lcdRxSoftStartConfig
        public const int LSR_SHOTS = 4;            // lcdRxLsrShots
        public const int RUN_STATUS = 14;          // lcdRxRunStatus
        public const int INT_STATUS = 1;           // lcdRxIntStatus
        public const int WAVEFORM = 64;            // lcdRxWaveform (32 samples * 2 bytes)
        public const int DISCOVERY = 1;            // lcdRxDiscovery
        public const int CHARGE_STATE = 3;         // lcdRxLsrChargeState
        public const int CHARGE_VOLT = 2;          // lcdRxLsrChargeVolt

        // Added for missing references (based on code usage)
        public const int DIGITAL_IO = 8;           // For lcdRxDigitalIO / lcdTxDigitalIO (8 bytes as per code)
        public const int SYSTEM_RESET = 0;         // For lcdRxSystemReset (assuming no data payload, adjust if needed)
    }
}
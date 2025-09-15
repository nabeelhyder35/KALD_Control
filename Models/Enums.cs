using System;

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

    // FPGA Command Enums - Use sbyte for Rx commands to allow negative values
    public enum LcdRxCommand : sbyte
    {
        lcdRxBadCmd = -2,
        lcdRxNoCmd = -1,
        lcdFPGABadCmd = 0,
        lcdRxLsrPulseConfig = 1,
        lcdRxLsrState = 2,
        lcdRxIntMask = 3,
        lcdRxWaveState = 4,
        lcdRxLsrDelays = 5,
        lcdRxLsrCal = 6,
        lcdRxLsrVolts = 7,
        lcdRxLsrChargeCancel = 8,
        lcdRxShutterConfig = 9,
        lcdRxSoftStartConfig = 10,
        lcdRxDiscovery = 11,
        lcdRxDigitalIO = 12,
        lcdRxSystemReset = 13
    }

    // Tx commands don't need negative values, so keep as byte
    public enum LcdTxCommand : byte
    {
        lcdTxTestResp = 0,
        lcdTxBadCmd = 1,
        lcdTxLsrState = 2,
        lcdTxLsrPulseConfig = 3,
        lcdTxLsrCount = 4,
        lcdTxLsrRunStatus = 5,
        lcdTxLsrIntStatus = 6,
        lcdTxLsrIntMask = 7,
        lcdTxLsrWaveform = 8,
        lcdTxDiscovery = 9,
        lcdTxLsrVolts = 10,
        lcdTxLsrChargeState = 11,
        lcdTxLsrChargeVolts = 12,
        lcdTxShutterConfig = 13,
        lcdTxSoftStartConfig = 14,
        lcdTxLsrCal = 15,
        lcdTxLsrDelays = 16,
        lcdTxDigitalIO = 17
    }

    // Add a helper class for special command handling
    public static class SpecialCommands
    {
        public const byte LCD_RX_BAD_CMD = 0xFE;    // -2 as byte
        public const byte LCD_RX_NO_CMD = 0xFF;     // -1 as byte
    }
}
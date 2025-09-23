namespace KALD_Control.Models
{
    /// <summary>
    /// Defines communication protocol constants for device communication.
    /// </summary>
    public static class ProtocolConstants
    {
        public const byte STX = 0x2A;    // Start of Text
        public const byte ETX = 0x3A;    // End of Text
        public const int HEADER_LENGTH = 5; // STX(1) + Length(2) + Command(1) + Checksum(1)
        public const int MIN_PACKET_LENGTH = HEADER_LENGTH + 1; // Minimum packet length (including ETX)
    }

    /// <summary>
    /// Defines possible laser state types for the system.
    /// </summary>
    public enum LaserStateType : byte
    {
        lsrIdle = 0,
        lsrArming = 1,
        lsrArmed = 2,
        lsrRunning = 3,
        lsrPausing = 4,
        lsrDisarming = 5,
        lsrPaused = 6,
        lsrFault = 7
    }

    /// <summary>
    /// Defines shot modes for laser pulse configuration.
    /// </summary>
    public enum ShotModeType : byte
    {
        burstMode = 0,
        contMode = 1
    }

    /// <summary>
    /// Defines trigger modes for laser pulse configuration.
    /// </summary>
    public enum TriggerModeType : byte
    {
        intTrigMode = 0,
        extTrigMode = 1
    }

    /// <summary>
    /// Defines shutter modes for the laser system.
    /// </summary>
    public enum ShutterModeType : byte
    {
        autoMode = 0,
        manualMode = 1
    }

    /// <summary>
    /// Defines shutter states for the laser system.
    /// </summary>
    public enum ShutterStateType : byte
    {
        shutterClosed = 0,
        shutterOpened = 1
    }

    /// <summary>
    /// Defines analog input channel types for calibration.
    /// </summary>
    public enum AiChanType : byte
    {
        CapVoltage = 0,
        LampCurrent = 1,
        LampVoltage = 2,
        Reserved3 = 3,
        Reserved4 = 4,
        Reserved5 = 5,
        Reserved6 = 6,
        Reserved7 = 7
    }

    /// <summary>
    /// Defines analog output channel types for calibration.
    /// </summary>
    public enum AoChanType : byte
    {
        CapVoltageSet = 0,
        Reserved1 = 1
    }

    /// <summary>
    /// Defines commands sent to the device (receive commands).
    /// </summary>
    public enum uiTxCommand : byte
    {
        uiTxBadCmd = 0xFE,
        uiTxNoCmd = 0xFF,
        uiTxFPGABadCmd = 0x00,
        uiTxLsrPulseConfig = 0x01,
        uiTxLsrState = 0x02,
        uiTxIntMask = 0x03,
        uiTxWaveState = 0x04,
        uiTxLsrDelays = 0x05,
        uiTxLsrCal = 0x06,
        uiTxLsrVolts = 0x07,
        uiTxLsrChargeCancel = 0x08,
        uiTxShutterConfig = 0x09,
        uiTxSoftStartConfig = 0x0A
    }

    /// <summary>
    /// Defines commands received from the device (transmit commands).
    /// </summary>
    public enum uiRxCommand : byte
    {
        uiRxTestResp = 0x00,
        uiRxFPGABadCmd = 0x01,
        uiRxLsrState = 0x02,
        uiRxLsrPulseConfig = 0x03,
        uiRxLsrCount = 0x04,
        uiRxLsrRunStatus = 0x05,
        uiRxLsrIntStatus = 0x06,
        uiRxLsrIntMask = 0x07,
        uiRxLsrWaveform = 0x08,
        uiRxDiscovery = 0x09,
        uiRxLsrVolts = 0x0A,
        uiRxLsrChargeState = 0x0B,
        uiRxLsrChargeVolts = 0x0C,
        uiRxShutterConfig = 0x0D,
        uiRxSoftStartConfig = 0x0E
    }

    /// <summary>
    /// Defines packet validation results for communication protocol.
    /// </summary>
    public enum PacketValidationResult : byte
    {
        Valid = 0,
        InvalidSTX = 1,
        InvalidETX = 2,
        InvalidLength = 3,
        ChecksumMismatch = 4,
        InsufficientData = 5
    }
}
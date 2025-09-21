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
    public enum LcdRxCommand : byte
    {
        lcdRxBadCmd = 0xFE,
        lcdRxNoCmd = 0xFF,
        lcdRxFPGABadCmd = 0x00,
        lcdRxLsrPulseConfig = 0x01,
        lcdRxLsrState = 0x02,
        lcdRxLsrCount = 0x03,
        lcdRxLsrRunStatus = 0x04,
        lcdRxLsrIntStatus = 0x05,
        lcdRxLsrIntMask = 0x06,
        lcdRxLsrWaveform = 0x07,
        lcdRxLsrChargeCancel = 0x08,
        lcdRxDiscovery = 0x09,
        lcdRxDiscoveryAck = 0x0A,
        lcdRxLsrVolts = 0x0B,
        lcdRxLsrChargeState = 0x0C,
        lcdRxLsrChargeVolts = 0x0D,
        lcdRxShutterConfig = 0x0E,
        lcdRxSoftStartConfig = 0x0F,
        lcdRxReadEnergy = 0x10,
        lcdRxReadTemperature = 0x11,
        lcdRxSystemInfo = 0x12,
        lcdRxSystemReset = 0x13,
        lcdRxLsrCal = 0x14,
        lcdRxLsrDelays = 0x15,
        lcdRxDigitalIO = 0x16
    }

    /// <summary>
    /// Defines commands received from the device (transmit commands).
    /// </summary>
    public enum LcdTxCommand : byte
    {
        lcdTxFPGABadCmd = 0x00,
        lcdTxLsrPulseConfig = 0x01,
        lcdTxLsrState = 0x02,
        lcdTxLsrCount = 0x03,
        lcdTxLsrRunStatus = 0x04,
        lcdTxLsrIntStatus = 0x05,
        lcdTxLsrIntMask = 0x06,
        lcdTxLsrWaveform = 0x07,
        lcdTxDiscovery = 0x08,
        lcdTxLsrVolts = 0x09,
        lcdTxLsrChargeState = 0x0A,
        lcdTxLsrChargeVolts = 0x0B,
        lcdTxShutterConfig = 0x0C,
        lcdTxSoftStartConfig = 0x0D,
        lcdTxReadEnergy = 0x0E,
        lcdTxReadTemperature = 0x0F,
        lcdTxSystemInfo = 0x10,
        lcdTxFactorySettings = 0x11,
        lcdTxInterlockStatus = 0x12,
        lcdTxLsrCal = 0x13,
        lcdTxLsrDelays = 0x14,
        lcdTxDigitalIO = 0x15
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
using System;

namespace KALD_Control.Models
{
    /// <summary>
    /// Represents calibration data for the laser system, matching the VpdData structure in vpd.h.
    /// Handles cap voltage range, measured voltage, and analog input/output channel calibrations.
    /// </summary>
    public class CalibrationData : ObservableObject
    {
        // Cap voltage range from hardware.c/lcd.c
        private ushort _capVoltRange;
        public ushort CapVoltRange
        {
            get => _capVoltRange;
            set => SetProperty(ref _capVoltRange, value);
        }

        // Measured voltage from the system
        private ushort _measuredVoltage;
        public ushort MeasuredVoltage
        {
            get => _measuredVoltage;
            set => SetProperty(ref _measuredVoltage, value);
        }

        // Calibration status flags
        private byte _calibrationStatus;
        public byte CalibrationStatus
        {
            get => _calibrationStatus;
            set => SetProperty(ref _calibrationStatus, value);
        }

        // Product information matching VpdData structure from vpd.h
        private string _productName = "";
        public string ProductName
        {
            get => _productName;
            set => SetProperty(ref _productName, value);
        }

        private string _serialNumber = "";
        public string SerialNumber
        {
            get => _serialNumber;
            set => SetProperty(ref _serialNumber, value);
        }

        // AI Channel calibrations (matching VPD_AI_CAL_ENTRIES = 8 from vpd.h)
        private AiChannelCalibration[] _aiChannelCals = new AiChannelCalibration[8];
        public AiChannelCalibration[] AiChannelCals
        {
            get => _aiChannelCals;
            set => SetProperty(ref _aiChannelCals, value);
        }

        // AO Channel calibrations (matching VPD_AO_CAL_ENTRIES = 2 from vpd.h)
        private AoChannelCalibration[] _aoChannelCals = new AoChannelCalibration[2];
        public AoChannelCalibration[] AoChannelCals
        {
            get => _aoChannelCals;
            set => SetProperty(ref _aoChannelCals, value);
        }

        // CRC validation from VPD data
        private ushort _crc;
        public ushort Crc
        {
            get => _crc;
            set => SetProperty(ref _crc, value);
        }

        private bool _isValid;
        public bool IsValid
        {
            get => _isValid;
            set => SetProperty(ref _isValid, value);
        }

        /// <summary>
        /// Initializes a new instance of CalibrationData with default AI and AO channel arrays.
        /// </summary>
        public CalibrationData()
        {
            for (int i = 0; i < 8; i++)
                _aiChannelCals[i] = new AiChannelCalibration();
            for (int i = 0; i < 2; i++)
                _aoChannelCals[i] = new AoChannelCalibration();
        }

        /// <summary>
        /// Converts raw capacitor voltage value to physical units.
        /// </summary>
        public double ConvertCapVoltage(int rawValue)
        {
            return ((double)(rawValue - 0x125) * HardwareConstants.CAPV_IN_SLOPE) / HardwareConstants.CAPV_IN_COUNT + HardwareConstants.CAPV_IN_OFFSET;
        }

        /// <summary>
        /// Converts raw lamp current value to physical units.
        /// </summary>
        public double ConvertLampAmps(int rawValue)
        {
            return ((double)(rawValue - 0x125) * HardwareConstants.LAMPC_IN_SLOPE) / HardwareConstants.LAMPC_IN_COUNT + HardwareConstants.LAMPC_IN_OFFSET;
        }

        /// <summary>
        /// Converts raw lamp voltage value to physical units.
        /// </summary>
        public double ConvertLampVolts(int rawValue)
        {
            return ((double)(rawValue - 0x125) * HardwareConstants.LAMPV_IN_SLOPE) / HardwareConstants.LAMPV_IN_COUNT + HardwareConstants.LAMPV_IN_OFFSET;
        }
    }

    /// <summary>
    /// Represents calibration data for an analog input channel.
    /// </summary>
    public class AiChannelCalibration : ObservableObject
    {
        private AiChanType _channel;
        public AiChanType Channel
        {
            get => _channel;
            set => SetProperty(ref _channel, value);
        }

        private ushort _in0Vdc;
        public ushort In0Vdc
        {
            get => _in0Vdc;
            set => SetProperty(ref _in0Vdc, value);
        }

        private ushort _in10Vdc;
        public ushort In10Vdc
        {
            get => _in10Vdc;
            set => SetProperty(ref _in10Vdc, value);
        }
    }

    /// <summary>
    /// Represents calibration data for an analog output channel.
    /// </summary>
    public class AoChannelCalibration : ObservableObject
    {
        private AoChanType _channel;
        public AoChanType Channel
        {
            get => _channel;
            set => SetProperty(ref _channel, value);
        }

        private ushort _out0Vdc;
        public ushort Out0Vdc
        {
            get => _out0Vdc;
            set => SetProperty(ref _out0Vdc, value);
        }

        private ushort _out10Vdc;
        public ushort Out10Vdc
        {
            get => _out10Vdc;
            set => SetProperty(ref _out10Vdc, value);
        }
    }
}
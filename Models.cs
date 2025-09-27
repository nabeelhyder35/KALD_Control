using KALD_Control.ViewModels;
using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace KALD_Control.Models
{
    /// <summary>
    /// Base class for observable objects implementing INotifyPropertyChanged.
    /// </summary>
    public abstract class ObservableObject : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler? PropertyChanged;

        protected bool SetProperty<T>(ref T field, T value, [CallerMemberName] string? propertyName = null)
        {
            if (EqualityComparer<T>.Default.Equals(field, value))
                return false;

            field = value;
            OnPropertyChanged(propertyName);
            return true;
        }

        protected void OnPropertyChanged([CallerMemberName] string? propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }

    /// <summary>
    /// Represents the interlock status and mask flags for the laser system.
    /// Each bit in the status/mask byte represents a specific interlock condition.
    /// A bit value of 0 indicates the condition is OK, while 1 indicates a fault.
    /// </summary>
    public class InterlockClass : ObservableObject
    {
        private byte _status = 0x00;

        // Status properties: true if the interlock condition is OK (bit is 0), false if faulted (bit is 1)
        public bool coolantFlowStatus
        {
            get => (_status & 0x01) == 0;
        }
        public bool coolantTempStatus
        {
            get => (_status & 0x02) == 0;
        }
        public bool DoorStatus
        {
            get => (_status & 0x04) == 0;
        }
        public bool CoverStatus
        {
            get => (_status & 0x08) == 0;
        }
        public bool DumpTempStatus
        {
            get => (_status & 0x10) == 0;
        }
        public bool ChargerOverVoltageStatus
        {
            get => (_status & 0x20) == 0;
        }
        public bool ChargerOverTempStatus { 
            get => (_status & 0x40) == 0;
        }
        public byte Status
        {
            get => _status;
            set
            {
                _status = value;
                OnPropertyChanged();
                OnPropertyChanged(nameof(coolantFlowStatus));
                OnPropertyChanged(nameof(coolantTempStatus));
                OnPropertyChanged(nameof(DoorStatus));
                OnPropertyChanged(nameof(CoverStatus));
                OnPropertyChanged(nameof(DumpTempStatus));
                OnPropertyChanged(nameof(ChargerOverVoltageStatus));
                OnPropertyChanged(nameof(ChargerOverTempStatus));

                // IMPORTANT: Notify combined properties when status changes
                OnPropertyChanged(nameof(CoolantFlowCombined));
                OnPropertyChanged(nameof(CoolantTempCombined));
                OnPropertyChanged(nameof(DoorCombined));
                OnPropertyChanged(nameof(CoverCombined));
            }
        }

        private readonly MainViewModel _viewModel;
        public InterlockClass(MainViewModel viewModel)
        {
            _viewModel = viewModel;
        }

        private bool _coolantFlowEnabled = true;
        private bool _coolantTempEnabled = true;
        private bool _doorEnabled = true;
        private bool _coverEnabled = true;

        // Event or Action to notify when mask changes
        public event Action<byte> MaskChanged;
        // Or if you prefer a direct method call:
        private Action<byte> _onMaskChanged;


        // Mask properties: true if the interlock condition is enabled (bit is 1), false if disabled (bit is 0)
        public bool CoolantFlowEnabled
        {
            get => _coolantFlowEnabled;
            set
            {
                if (_coolantFlowEnabled != value)
                {
                    _coolantFlowEnabled = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(CoolantFlowCombined)); // Notify combined property
                    OnMaskChanged();
                }
            }
        }
        public bool CoolantTempEnabled
        {
            get => _coolantTempEnabled;
            set
            {
                if (_coolantTempEnabled != value)
                {
                    _coolantTempEnabled = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(CoolantTempCombined)); // Notify combined property
                    OnMaskChanged();
                }
            }
        }
        public bool DoorEnabled
        {
            get => _doorEnabled;
            set
            {
                if (_doorEnabled != value)
                {
                    _doorEnabled = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(DoorCombined)); // Notify combined property
                    OnMaskChanged();
                }
            }
        }

        public bool CoverEnabled
        {
            get => _coverEnabled;
            set
            {
                if (_coverEnabled != value)
                {
                    _coverEnabled = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(CoverCombined)); // Notify combined property
                    OnMaskChanged();
                }
            }
        }
        public byte Mask
        {
            get
            {
                byte mask = 0;
                if (_coolantFlowEnabled) mask |= 0x01;
                if (_coolantTempEnabled) mask |= 0x02;
                if (_doorEnabled) mask |= 0x04;
                if (_coverEnabled) mask |= 0x08;
                return (byte)(mask | 0x70);
            }
        }

        private void OnMaskChanged()
        {
            _viewModel?.ExecuteIntMaskUpdated();
        }


        // Combined property that returns array for converter
        public bool[] CoolantFlowCombined => new bool[] { CoolantFlowEnabled, coolantFlowStatus };
        public bool[] CoolantTempCombined => new bool[] { CoolantTempEnabled, coolantTempStatus };
        public bool[] DoorCombined => new bool[] { DoorEnabled, DoorStatus };
        public bool[] CoverCombined => new bool[] { CoverEnabled, CoverStatus };
    }

    /// <summary>
    /// Stores waveform samples and capture time for the laser system.
    /// </summary>
    public class WaveformData
    {
        public ushort[] Samples { get; } = new ushort[32];
        public DateTime CaptureTime { get; set; } = DateTime.MinValue;
    }

    /// <summary>
    /// Constants used in CalibrationData conversion formulas.
    /// Values are placeholders; adjust to match real hardware characteristics if available.
    /// </summary>
    public static class HardwareConstants
    {
        // Cap voltage conversion
        public const double CAPV_IN_SLOPE = 1.0;
        public const double CAPV_IN_COUNT = 4096.0;
        public const double CAPV_IN_OFFSET = 0.0;

        // Lamp current conversion
        public const double LAMPC_IN_SLOPE = 1.0;
        public const double LAMPC_IN_COUNT = 4096.0;
        public const double LAMPC_IN_OFFSET = 0.0;

        // Lamp voltage conversion
        public const double LAMPV_IN_SLOPE = 1.0;
        public const double LAMPV_IN_COUNT = 4096.0;
        public const double LAMPV_IN_OFFSET = 0.0;
    }

    /// <summary>
    /// Represents the pulse configuration for the laser system.
    /// </summary>
    public class PulseConfig : ObservableObject
    {
        private ushort _frequency;
        public ushort Frequency
        {
            get => _frequency;
            set => SetProperty(ref _frequency, value);
        }

        private ushort _pulseWidth;
        public ushort PulseWidth
        {
            get => _pulseWidth;
            set => SetProperty(ref _pulseWidth, value);
        }

        private uint _shotTotal;
        public uint ShotTotal
        {
            get => _shotTotal;
            set => SetProperty(ref _shotTotal, value);
        }

        private ushort _delay1;
        public ushort Delay1
        {
            get => _delay1;
            set => SetProperty(ref _delay1, value);
        }

        private ushort _delay2;
        public ushort Delay2
        {
            get => _delay2;
            set => SetProperty(ref _delay2, value);
        }

        private ShotModeType _shotMode;
        public ShotModeType ShotMode
        {
            get => _shotMode;
            set => SetProperty(ref _shotMode, value);
        }

        private TriggerModeType _trigMode;
        public TriggerModeType TrigMode
        {
            get => _trigMode;
            set => SetProperty(ref _trigMode, value);
        }
    }

    /// <summary>
    /// Tracks runtime data such as shot count, laser state, energy, and power.
    /// </summary>
    public class RunStatusData : ObservableObject
    {
        private uint _shotCount;
        public uint ShotCount
        {
            get => _shotCount;
            set => SetProperty(ref _shotCount, value);
        }

        private LaserStateType _state;
        public LaserStateType State
        {
            get => _state;
            set => SetProperty(ref _state, value);
        }

        private ushort _energy;
        public ushort Energy
        {
            get => _energy;
            set => SetProperty(ref _energy, value);
        }

        private ushort _power;
        public ushort Power
        {
            get => _power;
            set => SetProperty(ref _power, value);
        }

        private ushort _current;
        public ushort Current
        {
            get => _current;
            set => SetProperty(ref _current, value);
        }

        private ushort _vDroop;
        public ushort VDroop
        {
            get => _vDroop;
            set => SetProperty(ref _vDroop, value);
        }
    }

    /// <summary>
    /// Represents the shutter configuration for the laser system.
    /// </summary>
    public class ShutterConfig : ObservableObject
    {
        private ShutterModeType _shutterMode;
        public ShutterModeType ShutterMode
        {
            get => _shutterMode;
            set => SetProperty(ref _shutterMode, value);
        }

        private ShutterStateType _shutterState;
        public ShutterStateType ShutterState
        {
            get => _shutterState;
            set => SetProperty(ref _shutterState, value);
        }
    }

    /// <summary>
    /// Configures soft start settings, including enable flag, idle setpoint, and ramp count.
    /// </summary>
    public class SoftStartConfig : ObservableObject
    {
        private bool _enable;
        public bool Enable
        {
            get => _enable;
            set => SetProperty(ref _enable, value);
        }

        private ushort _idleSetpoint;
        public ushort IdleSetpoint
        {
            get => _idleSetpoint;
            set => SetProperty(ref _idleSetpoint, value);
        }

        private uint _rampCount;
        public uint RampCount
        {
            get => _rampCount;
            set => SetProperty(ref _rampCount, value);
        }
    }

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
    /// Commands RECEIVED from FPGA (FPGA → GUI)
    /// These match what's actually defined in your code
    /// </summary>
    public enum uiRxCommand : byte
    {
        uiRxTestResp = 0x00,         // Test response
        uiRxFPGABadCmd = 0x01,       // FPGA sent bad command (NOT uiRxBadCmd)
        uiRxLsrState = 0x02,         // Laser state
        uiRxLsrPulseConfig = 0x03,   // Pulse configuration
        uiRxLsrCount = 0x04,         // Shot count
        uiRxLsrRunStatus = 0x05,     // Run status
        uiRxLsrIntStatus = 0x06,     // Interlock status
        uiRxLsrIntMask = 0x07,       // Interlock mask
        uiRxLsrWaveform = 0x08,      // Waveform data
        uiRxDiscovery = 0x09,        // Discovery message
        uiRxLsrVolts = 0x0A,         // Voltage data
        uiRxLsrChargeState = 0x0B,   // Charge state
        uiRxLsrChargeVolts = 0x0C,   // Charge voltage
        uiRxShutterConfig = 0x0D,    // Shutter configuration
        uiRxSoftStartConfig = 0x0E   // Soft start configuration
                                     // Note: uiRxBadCmd doesn't exist in your original enum
    }

    /// <summary>
    /// Commands TRANSMITTED to FPGA (GUI → FPGA)  
    /// These match what's actually defined in your code
    /// </summary>
    public enum uiTxCommand : byte
    {
        uiTxBadCmd = 0xFE,           // Bad command (checksum error)
        uiTxNoCmd = 0xFF,            // No command received yet
        uiTxFPGABadCmd = 0x00,       // FPGA sent bad command
        uiTxLsrPulseConfig = 0x01,   // Laser Pulse Configuration
        uiTxLsrState = 0x02,         // Laser State change
        uiTxIntMask = 0x03,          // Change Interlock Mask
        uiTxWaveState = 0x04,        // Enable/Disable Waveform Messaging
        uiTxLsrDelays = 0x05,        // Laser Fire Setup (delays)
        uiTxLsrCal = 0x06,           // Laser Calibration
        uiTxLsrVolts = 0x07,         // Laser voltage setting
        uiTxLsrChargeCancel = 0x08,  // Cancel a (dis)charge
        uiTxShutterConfig = 0x09,    // Shutter State Config
        uiTxSoftStartConfig = 0x0A   // Soft Start Configuration
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

    /// <summary>
    /// Provides helper methods for communication protocol handling.
    /// </summary>
    public static class CommunicationHelper
    {
        private static readonly Action<string> Log = message => System.Diagnostics.Debug.WriteLine($"[CommunicationHelper] {message}");

        /// <summary>
        /// Creates a packet according to the specified protocol structure.
        /// </summary>
        /// <param name="command">The command byte.</param>
        /// <param name="data">The data payload.</param>
        /// <returns>The complete packet as a byte array.</returns>
        public static byte[] CreatePacket(byte command, byte[] data = null)
        {
            data = data ?? Array.Empty<byte>();
            int dataLength = data.Length;
            int packetLength = 1 + 2 + 1 + dataLength + 1 + 1; // STX + LEN(2) + CMD + DATA + CHK + ETX

            // Length field represents only the data length
            ushort lengthField = (ushort)dataLength;

            byte[] packet = new byte[packetLength];
            int index = 0;

            // STX
            packet[index++] = ProtocolConstants.STX;

            // Length (2 bytes, big-endian)
            packet[index++] = (byte)(lengthField >> 8);
            packet[index++] = (byte)(lengthField & 0xFF);

            // Command
            packet[index++] = command;

            // Data
            if (dataLength > 0)
            {
                Array.Copy(data, 0, packet, index, dataLength);
                index += dataLength;
            }

            // Calculate checksum (0 - sum of command and data bytes)
            byte checksum = command;
            for (int i = 0; i < dataLength; i++)
            {
                checksum += data[i];
            }
            packet[index++] = (byte)((0 - checksum) & 0xFF);

            // ETX
            packet[index] = ProtocolConstants.ETX;

            Log($"Created packet: {BitConverter.ToString(packet).Replace("-", " ")}");
            return packet;
        }

        /// <summary>
        /// Validates a received packet according to the protocol structure.
        /// </summary>
        /// <param name="packet">The received packet.</param>
        /// <returns>The validation result.</returns>
        public static PacketValidationResult ValidatePacket(byte[] packet)
        {
            if (packet == null || packet.Length < ProtocolConstants.MIN_PACKET_LENGTH)
            {
                Log($"Validation failed: Insufficient data, length={packet?.Length ?? 0}");
                return PacketValidationResult.InsufficientData;
            }

            // Check STX
            if (packet[0] != ProtocolConstants.STX)
            {
                Log($"Validation failed: Invalid STX, got 0x{packet[0]:X2}");
                return PacketValidationResult.InvalidSTX;
            }

            // Extract length (big-endian)
            ushort lengthField = (ushort)((packet[1] << 8) | packet[2]);

            // Verify packet length
            int expectedPacketLength = 1 + 2 + 1 + lengthField + 1 + 1; // STX + LEN(2) + CMD + DATA + CHK + ETX
            if (packet.Length < expectedPacketLength)
            {
                Log($"Validation failed: Insufficient data, expected {expectedPacketLength}, got {packet.Length}");
                return PacketValidationResult.InsufficientData;
            }

            // Check ETX
            if (packet[expectedPacketLength - 1] != ProtocolConstants.ETX)
            {
                Log($"Validation failed: Invalid ETX, got 0x{packet[expectedPacketLength - 1]:X2}");
                return PacketValidationResult.InvalidETX;
            }

            // Verify checksum
            int dataEnd = expectedPacketLength - 2; // Position before checksum
            byte sum = packet[3]; // Command
            for (int i = 4; i < 4 + lengthField; i++)
            {
                sum += packet[i];
            }
            byte calculatedChecksum = (byte)((0 - sum) & 0xFF);
            byte receivedChecksum = packet[expectedPacketLength - 2];

            if (calculatedChecksum != receivedChecksum)
            {
                Log($"Validation failed: Checksum mismatch, expected 0x{calculatedChecksum:X2}, got 0x{receivedChecksum:X2}");
                return PacketValidationResult.ChecksumMismatch;
            }

            Log($"Packet validated: Length={lengthField}, Checksum=0x{receivedChecksum:X2}");
            return PacketValidationResult.Valid;
        }

        /// <summary>
        /// Extracts command and data from a valid packet.
        /// </summary>
        /// <param name="packet">The validated packet.</param>
        /// <param name="command">The extracted command.</param>
        /// <param name="data">The extracted data.</param>
        /// <returns>True if extraction was successful.</returns>
        public static bool ExtractPacketData(byte[] packet, out byte command, out byte[] data)
        {
            command = 0;
            data = null;

            if (ValidatePacket(packet) != PacketValidationResult.Valid)
            {
                Log("Extraction failed: Invalid packet");
                return false;
            }

            // Extract length (big-endian)
            ushort lengthField = (ushort)((packet[1] << 8) | packet[2]);

            // Command is at position 3
            command = packet[3];

            // Data length is lengthField
            int dataLength = lengthField;

            if (dataLength > 0)
            {
                data = new byte[dataLength];
                Array.Copy(packet, 4, data, 0, dataLength);
            }
            else
            {
                data = Array.Empty<byte>();
            }

            Log($"Extracted: Command=0x{command:X2}, Data=[{BitConverter.ToString(data).Replace("-", " ")}]");
            return true;
        }
    }

    /// <summary>
    /// Represents the charge state data of the laser system, including measured voltage and charge completion status.
    /// </summary>
    public class ChargeStateData : ObservableObject
    {
        private ushort _measuredVolts;
        public ushort MeasuredVolts
        {
            get => _measuredVolts;
            set => SetProperty(ref _measuredVolts, value);
        }

        private bool _chargeDone;
        public bool ChargeDone
        {
            get => _chargeDone;
            set => SetProperty(ref _chargeDone, value);
        }
    }

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

    /// <summary>
    /// Represents the main device data for the laser system, matching variables in nios_app.h.
    /// </summary>
    public class DeviceData : ObservableObject
    {
        private LaserStateType _systemState;
        public LaserStateType SystemState
        {
            get => _systemState;
            set => SetProperty(ref _systemState, value);
        }

        private LaserStateType _nextState;
        public LaserStateType NextState
        {
            get => _nextState;
            set => SetProperty(ref _nextState, value);
        }

        private LaserStateType _lastState;
        public LaserStateType LastState
        {
            get => _lastState;
            set => SetProperty(ref _lastState, value);
        }

        private LaserStateType _laserState;
        public LaserStateType LaserState
        {
            get => _laserState;
            set => SetProperty(ref _laserState, value);
        }

        private ushort _setVoltage;
        public ushort SetVoltage
        {
            get => _setVoltage;
            set => SetProperty(ref _setVoltage, value);
        }

        private ushort _nextVoltage;
        public ushort NextVoltage
        {
            get => _nextVoltage;
            set => SetProperty(ref _nextVoltage, value);
        }

        private ushort _actualVoltage;
        public ushort ActualVoltage
        {
            get => _actualVoltage;
            set => SetProperty(ref _actualVoltage, value);
        }

        private ushort _voltageSetpoint;
        public ushort VoltageSetpoint
        {
            get => _voltageSetpoint;
            set => SetProperty(ref _voltageSetpoint, value);
        }

        private ushort _idleSetpoint;
        public ushort IdleSetpoint
        {
            get => _idleSetpoint;
            set => SetProperty(ref _idleSetpoint, value);
        }

        private ushort _rampVoltage;
        public ushort RampVoltage
        {
            get => _rampVoltage;
            set => SetProperty(ref _rampVoltage, value);
        }

        private ushort _pulseWidth;
        public ushort PulseWidth
        {
            get => _pulseWidth;
            set => SetProperty(ref _pulseWidth, value);
        }

        private ushort _frequency;
        public ushort Frequency
        {
            get => _frequency;
            set => SetProperty(ref _frequency, value);
        }

        private ushort _delay1;
        public ushort Delay1
        {
            get => _delay1;
            set => SetProperty(ref _delay1, value);
        }

        private ushort _delay2;
        public ushort Delay2
        {
            get => _delay2;
            set => SetProperty(ref _delay2, value);
        }

        private uint _shotTotal;
        public uint ShotTotal
        {
            get => _shotTotal;
            set => SetProperty(ref _shotTotal, value);
        }

        private uint _shotCount;
        public uint ShotCount
        {
            get => _shotCount;
            set => SetProperty(ref _shotCount, value);
        }

        private uint _shotStart;
        public uint ShotStart
        {
            get => _shotStart;
            set => SetProperty(ref _shotStart, value);
        }

        private uint _rampCount;
        public uint RampCount
        {
            get => _rampCount;
            set => SetProperty(ref _rampCount, value);
        }

        private ushort _energy;
        public ushort Energy
        {
            get => _energy;
            set => SetProperty(ref _energy, value);
        }

        private ushort _power;
        public ushort Power
        {
            get => _power;
            set => SetProperty(ref _power, value);
        }

        private ushort _current;
        public ushort Current
        {
            get => _current;
            set => SetProperty(ref _current, value);
        }

        private ushort _vDroop;
        public ushort VDroop
        {
            get => _vDroop;
            set => SetProperty(ref _vDroop, value);
        }

        private ushort _voltage;
        public ushort Voltage
        {
            get => _voltage;
            set => SetProperty(ref _voltage, value);
        }

        private CalibrationData _calibration;
        public CalibrationData Calibration
        {
            get => _calibration;
            set => SetProperty(ref _calibration, value);
        }

        private RunStatusData _runStatus;
        public RunStatusData RunStatus
        {
            get => _runStatus;
            set => SetProperty(ref _runStatus, value);
        }

        private ChargeStateData _chargeState;
        public ChargeStateData ChargeState
        {
            get => _chargeState;
            set => SetProperty(ref _chargeState, value);
        }

        private ShutterConfig _shutterConfig;
        public ShutterConfig ShutterConfig
        {
            get => _shutterConfig;
            set => SetProperty(ref _shutterConfig, value);
        }

        private SoftStartConfig _softStartConfig;
        public SoftStartConfig SoftStartConfig
        {
            get => _softStartConfig;
            set => SetProperty(ref _softStartConfig, value);
        }

        private WaveformData _waveform;
        public WaveformData Waveform
        {
            get => _waveform;
            set => SetProperty(ref _waveform, value);
        }

        private PulseConfig _pulseConfig;
        public PulseConfig PulseConfig
        {
            get => _pulseConfig;
            set => SetProperty(ref _pulseConfig, value);
        }
    }

    /// <summary>
    /// Represents the digital I/O state of the laser system, including interlock status and mask.
    /// </summary>
    public class DigitalIOState : ObservableObject
    {
        private uint _outputStates;
        public uint OutputStates
        {
            get => _outputStates;
            set
            {
                if (SetProperty(ref _outputStates, value))
                {
                    //OnPropertyChanged(nameof(Output0));
                    //OnPropertyChanged(nameof(Output1));
                    //OnPropertyChanged(nameof(Output2));
                    //OnPropertyChanged(nameof(Output3));
                    //OnPropertyChanged(nameof(Output4));
                    //OnPropertyChanged(nameof(Output5));
                    //OnPropertyChanged(nameof(Output6));
                    //OnPropertyChanged(nameof(Output7));
                    OnPropertyChanged(nameof(ShutterOpen));
                    OnPropertyChanged(nameof(CoolerOn));
                    OnPropertyChanged(nameof(HVEnabled));
                    OnPropertyChanged(nameof(SimmerOn));
                    OnPropertyChanged(nameof(DumpOn));
                }
            }
        }

        private DateTime _timestamp;
        public DateTime Timestamp
        {
            get => _timestamp;
            set => SetProperty(ref _timestamp, value);
        }

        // Output controls
        public bool ShutterOpen
        {
            get => (_outputStates & 0x01) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x01;
                else
                    OutputStates &= ~0x01u;
            }
        }

        public bool CoolerOn
        {
            get => (_outputStates & 0x02) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x02;
                else
                    OutputStates &= ~0x02u;
            }
        }

        public bool HVEnabled
        {
            get => (_outputStates & 0x04) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x04;
                else
                    OutputStates &= ~0x04u;
            }
        }

        public bool SimmerOn
        {
            get => (_outputStates & 0x08) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x08;
                else
                    OutputStates &= ~0x08u;
            }
        }

        public bool DumpOn
        {
            get => (_outputStates & 0x10) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x10;
                else
                    OutputStates &= ~0x10u;
            }
        }
    }
}
// DeviceManager.cs - CLEANED AND CORRECTED

using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using KALD_Control.Models;

namespace KALD_Control.Services
{
    /// <summary>
    /// Manages serial communication between WinUI3 GUI and FPGA laser controller
    /// Handles command transmission, response parsing, and device state management
    /// </summary>
    public class DeviceManager : IDisposable
    {
        private readonly SerialPort _serialPort;
        private readonly ILogger<DeviceManager> _logger;
        private readonly object _serialLock = new object();
        private bool _disposed = false;
        private readonly List<byte> _incomingBuffer = new List<byte>();

        // Packet parsing state machine
        private enum ParserState { LookingForStart, ReadingLength, ReadingCommand, ReadingData, ReadingChecksum, ReadingEnd }
        private ParserState _currentState = ParserState.LookingForStart;
        private int _expectedDataLength = 0;
        private byte _currentCommand = 0;
        private DateTime _lastByteReceived = DateTime.MinValue;
        private readonly TimeSpan _parserTimeout = TimeSpan.FromSeconds(5);

        private CancellationTokenSource _cts;
        private int _discoveryRetries = 0;
        private const int MAX_DISCOVERY_RETRIES = 10;
        private bool _deviceReady = false;
        private DeviceData _deviceData = new DeviceData();

        // Event declarations for GUI notification
        public event EventHandler<LaserStateType> StateUpdated;
        public event EventHandler<PulseConfig> PulseConfigUpdated;
        public event EventHandler<uint> ShotCountUpdated;
        public event EventHandler<RunStatusData> RunStatusUpdated;
        public event EventHandler<byte> IntStatusUpdated;
        public event EventHandler<byte> IntMaskUpdated;
        public event EventHandler<WaveformData> WaveformUpdated;
        public event EventHandler<ushort> VoltsUpdated;
        public event EventHandler<ChargeStateData> ChargeStateUpdated;
        public event EventHandler<ShutterConfig> ShutterConfigUpdated;
        public event EventHandler<SoftStartConfig> SoftStartConfigUpdated;
        public event EventHandler<CalibrationData> CalibrationUpdated;
        public event EventHandler<DigitalIOState> DigitalIOUpdated;
        public event EventHandler<string> LogMessage;
        public event EventHandler<(string Command, Exception Exception)> CommandError;
        public event EventHandler<bool> DiscoveryResponseReceived;
        public event EventHandler<DeviceData> DeviceDataUpdated;

        public bool IsConnected => _serialPort?.IsOpen ?? false;
        public bool DeviceReady => _deviceReady;
        public int BaudRate => _serialPort?.BaudRate ?? 115200;

        public DeviceManager(ILogger<DeviceManager> logger)
        {
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));

            // Configure serial port for laser controller communication
            _serialPort = new SerialPort
            {
                BaudRate = 115200,
                Parity = Parity.None,
                DataBits = 8,
                StopBits = StopBits.One,
                Handshake = Handshake.None,
                ReadTimeout = 2000,
                WriteTimeout = 2000,
                ReadBufferSize = 4096,
                WriteBufferSize = 4096
            };

            _serialPort.ErrorReceived += OnErrorReceived;
            _cts = new CancellationTokenSource();
            StartParserTimeoutCheck();
        }

        /// <summary>
        /// Handles serial port errors and recovers communication
        /// </summary>
        private void OnErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            Log($"Serial port error: {e.EventType}");
            switch (e.EventType)
            {
                case SerialError.RXOver:
                    Log("Input buffer overflow; discarding input buffer");
                    _serialPort.DiscardInBuffer();
                    break;
                case SerialError.Overrun:
                    Log("Buffer overrun; possible data loss");
                    break;
                case SerialError.RXParity:
                    Log("Parity error; verify serial settings");
                    break;
                case SerialError.Frame:
                    Log("Framing error; check baud rate/stop bits");
                    break;
                case SerialError.TXFull:
                    Log("Output buffer full; discarding output buffer");
                    _serialPort.DiscardOutBuffer();
                    break;
            }
        }

        /// <summary>
        /// Establishes connection to laser controller and initiates discovery
        /// </summary>
        public void Connect(string portName, int baudRate = 115200)
        {
            if (IsConnected)
            {
                Log("Already connected; disconnecting first");
                Disconnect();
            }

            try
            {
                _serialPort.PortName = portName;
                _serialPort.BaudRate = baudRate;
                _serialPort.Open();
                _serialPort.DataReceived += OnDataReceived;
                Log($"Connected to {portName} at {baudRate} baud");

                _deviceReady = false;
                _discoveryRetries = 0;
                SendDiscovery(); // Start device discovery process
            }
            catch (Exception ex)
            {
                Log($"Failed to connect to {portName}: {ex.Message}");
                throw;
            }
        }

        /// <summary>
        /// Closes connection to laser controller and cleans up resources
        /// </summary>
        public void Disconnect()
        {
            if (!IsConnected)
            {
                Log("Already disconnected");
                return;
            }

            try
            {
                _serialPort.DataReceived -= OnDataReceived;
                _serialPort.Close();
                Log("Disconnected from serial port");
                _deviceReady = false;
                _discoveryRetries = 0;
                ResetParser();
            }
            catch (Exception ex)
            {
                Log($"Error disconnecting: {ex.Message}");
            }
        }

        /// <summary>
        /// Sends discovery packet to identify laser controller device
        /// </summary>
        private void SendDiscovery()
        {
            if (_discoveryRetries >= MAX_DISCOVERY_RETRIES)
            {
                Log("Max discovery retries reached; device not responding");
                DiscoveryResponseReceived?.Invoke(this, false);
                return;
            }

            _discoveryRetries++;
            Log($"Sending discovery packet (attempt {_discoveryRetries}/{MAX_DISCOVERY_RETRIES})");

            // Send discovery command (uiTxNoCmd with zero data)
            SendCommand(uiTxCommand.uiTxNoCmd, new byte[] { 0x00 });

            // Retry discovery if no response
            Task.Delay(1000, _cts.Token).ContinueWith(t =>
            {
                if (!t.IsCanceled && !_deviceReady)
                {
                    SendDiscovery();
                }
            }, _cts.Token);
        }

        /// <summary>
        /// Processes incoming data from laser controller using state machine parser
        /// Implements protocol: STX + Length(2) + Command + Data + Checksum + ETX
        /// </summary>
        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                lock (_serialLock)
                {
                    while (_serialPort.BytesToRead > 0)
                    {
                        byte b = (byte)_serialPort.ReadByte();
                        _lastByteReceived = DateTime.Now;
                        _incomingBuffer.Add(b);

                        switch (_currentState)
                        {
                            case ParserState.LookingForStart:
                                if (b == ProtocolConstants.STX) // Start of packet
                                {
                                    _incomingBuffer.Clear();
                                    _incomingBuffer.Add(b);
                                    _currentState = ParserState.ReadingLength;
                                }
                                break;

                            case ParserState.ReadingLength:
                                if (_incomingBuffer.Count == 3) // Got both length bytes
                                {
                                    _expectedDataLength = (_incomingBuffer[1] << 8) | _incomingBuffer[2];
                                    _currentState = ParserState.ReadingCommand;
                                }
                                break;

                            case ParserState.ReadingCommand:
                                if (_incomingBuffer.Count == 4) // Got command byte
                                {
                                    _currentCommand = b;
                                    _currentState = ParserState.ReadingData;
                                }
                                break;

                            case ParserState.ReadingData:
                                if (_incomingBuffer.Count == 4 + _expectedDataLength) // Got all data
                                {
                                    _currentState = ParserState.ReadingChecksum;
                                }
                                break;

                            case ParserState.ReadingChecksum:
                                _currentState = ParserState.ReadingEnd;
                                break;

                            case ParserState.ReadingEnd:
                                if (b == ProtocolConstants.ETX) // End of packet
                                {
                                    byte[] packet = _incomingBuffer.ToArray();
                                    var validationResult = CommunicationHelper.ValidatePacket(packet);

                                    if (validationResult == PacketValidationResult.Valid)
                                    {
                                        if (CommunicationHelper.ExtractPacketData(packet, out byte command, out byte[] data))
                                        {
                                            ProcessPacket((uiRxCommand)command, data); // Process valid packet
                                        }
                                    }
                                    else
                                    {
                                        Log($"Invalid packet: {validationResult}");
                                    }
                                }
                                else
                                {
                                    Log($"Invalid ETX: 0x{b:X2}");
                                }
                                ResetParser(); // Ready for next packet
                                break;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Log($"Error in OnDataReceived: {ex.Message}");
                ResetParser();
            }
        }

        /// <summary>
        /// Processes valid packets received from laser controller (FPGA → GUI)
        /// Updates device state and notifies GUI through events
        /// </summary>
        private void ProcessPacket(uiRxCommand command, byte[] data)
        {
            Log($"Processing command {command} (0x{(byte)command:X2}), DataLen={data.Length}");

            int expectedLength = GetCommandLength(command);
            if (data.Length != expectedLength)
            {
                Log($"Invalid data length for {command}: expected {expectedLength}, got {data.Length}");
                return;
            }

            try
            {
                switch (command)
                {
                    case uiRxCommand.uiRxDiscovery:
                        _deviceReady = true;
                        _discoveryRetries = 0;
                        Log("Device discovered successfully");
                        DiscoveryResponseReceived?.Invoke(this, true);
                        break;

                    case uiRxCommand.uiRxLsrState:
                        _deviceData.LaserState = (LaserStateType)data[0];
                        StateUpdated?.Invoke(this, _deviceData.LaserState);
                        Log($"Laser state updated: {_deviceData.LaserState}");
                        break;

                    case uiRxCommand.uiRxLsrPulseConfig:
                        _deviceData.PulseConfig = new PulseConfig
                        {
                            Frequency = (ushort)((data[0] << 8) | data[1]),
                            PulseWidth = (ushort)((data[2] << 8) | data[3]),
                            ShotTotal = (uint)((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]),
                            Delay1 = (ushort)((data[8] << 8) | data[9]),
                            Delay2 = (ushort)((data[10] << 8) | data[11]),
                            ShotMode = (ShotModeType)data[12],
                            TrigMode = (TriggerModeType)data[13]
                        };
                        PulseConfigUpdated?.Invoke(this, _deviceData.PulseConfig);
                        Log($"Pulse config updated: Freq={_deviceData.PulseConfig.Frequency}, Width={_deviceData.PulseConfig.PulseWidth}, Total={_deviceData.PulseConfig.ShotTotal}");
                        break;

                    case uiRxCommand.uiRxLsrCount:
                        _deviceData.ShotCount = (uint)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
                        ShotCountUpdated?.Invoke(this, _deviceData.ShotCount);
                        Log($"Shot count updated: {_deviceData.ShotCount}");
                        break;

                    case uiRxCommand.uiRxLsrRunStatus:
                        // 13 bytes: ShotCount(4) + State(1) + Energy(2) + Power(2) + Current(2) + VDroop(2)
                        _deviceData.RunStatus = new RunStatusData
                        {
                            ShotCount = (uint)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]),
                            State = (LaserStateType)data[4],
                            Energy = (ushort)((data[5] << 8) | data[6]),
                            Power = (ushort)((data[7] << 8) | data[8]),
                            Current = (ushort)((data[9] << 8) | data[10]),
                            VDroop = (ushort)((data[11] << 8) | data[12])
                        };
                        RunStatusUpdated?.Invoke(this, _deviceData.RunStatus);
                        Log($"Run status: Shots={_deviceData.RunStatus.ShotCount}, State={_deviceData.RunStatus.State}, Energy={_deviceData.RunStatus.Energy}J");
                        break;

                    case uiRxCommand.uiRxLsrIntStatus:
                        {
                            var digitalIO = new DigitalIOState();
                            // Single byte for interlock status
                            digitalIO.InputStates = data[0];
                            IntStatusUpdated?.Invoke(this, digitalIO.GetInterlockStatusForLCD());
                            DigitalIOUpdated?.Invoke(this, digitalIO);
                            Log($"Interlock status: 0x{data[0]:X2}");
                        }
                        break;

                    case uiRxCommand.uiRxLsrIntMask:
                        {
                            var digitalIO = new DigitalIOState();
                            // Single byte for interlock mask
                            digitalIO.SetInterlockMaskFromLCD(data[0]);
                            IntMaskUpdated?.Invoke(this, data[0]);
                            DigitalIOUpdated?.Invoke(this, digitalIO);
                            Log($"Interlock mask: 0x{data[0]:X2}");
                        }
                        break;

                    case uiRxCommand.uiRxLsrWaveform:
                        _deviceData.Waveform = new WaveformData
                        {
                            CaptureTime = DateTime.Now
                        };
                        // 64 bytes: 32 samples × 2 bytes each
                        for (int i = 0; i < 32; i++)
                        {
                            _deviceData.Waveform.Samples[i] = (ushort)((data[i * 2] << 8) | data[i * 2 + 1]);
                        }
                        WaveformUpdated?.Invoke(this, _deviceData.Waveform);
                        Log($"Waveform received: {_deviceData.Waveform.Samples.Length} samples at {_deviceData.Waveform.CaptureTime:HH:mm:ss.fff}");
                        break;

                    case uiRxCommand.uiRxLsrVolts:
                        _deviceData.ActualVoltage = (ushort)((data[0] << 8) | data[1]);
                        VoltsUpdated?.Invoke(this, _deviceData.ActualVoltage);
                        Log($"Voltage updated: {_deviceData.ActualVoltage}V");
                        break;

                    case uiRxCommand.uiRxLsrChargeState:
                        // 3 bytes: MeasuredVolts(2) + ChargeDone(1)
                        _deviceData.ChargeState = new ChargeStateData
                        {
                            MeasuredVolts = (ushort)((data[0] << 8) | data[1]),
                            ChargeDone = data[2] != 0
                        };
                        ChargeStateUpdated?.Invoke(this, _deviceData.ChargeState);
                        Log($"Charge state: {_deviceData.ChargeState.MeasuredVolts}V, Done={_deviceData.ChargeState.ChargeDone}");
                        break;

                    case uiRxCommand.uiRxLsrChargeVolts:
                        _deviceData.VoltageSetpoint = (ushort)((data[0] << 8) | data[1]);
                        Log($"Charge voltage setpoint: {_deviceData.VoltageSetpoint}V");
                        break;

                    case uiRxCommand.uiRxShutterConfig:
                        // 2 bytes: ShutterMode(1) + ShutterState(1)
                        _deviceData.ShutterConfig = new ShutterConfig
                        {
                            ShutterMode = (ShutterModeType)data[0],
                            ShutterState = (ShutterStateType)data[1]
                        };
                        ShutterConfigUpdated?.Invoke(this, _deviceData.ShutterConfig);
                        Log($"Shutter config: Mode={_deviceData.ShutterConfig.ShutterMode}, State={_deviceData.ShutterConfig.ShutterState}");
                        break;

                    case uiRxCommand.uiRxSoftStartConfig:
                        // 7 bytes: Enable(1) + IdleSetpoint(2) + RampCount(4)
                        _deviceData.SoftStartConfig = new SoftStartConfig
                        {
                            Enable = data[0] != 0,
                            IdleSetpoint = (ushort)((data[1] << 8) | data[2]),
                            RampCount = (uint)((data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6])
                        };
                        SoftStartConfigUpdated?.Invoke(this, _deviceData.SoftStartConfig);
                        Log($"Soft start: Enable={_deviceData.SoftStartConfig.Enable}, Idle={_deviceData.SoftStartConfig.IdleSetpoint}V, Ramp={_deviceData.SoftStartConfig.RampCount} shots");
                        break;

                    case uiRxCommand.uiRxTestResp:
                        Log($"Test response received: 0x{data[0]:X2}");
                        break;

                    case uiRxCommand.uiRxFPGABadCmd:
                        Log($"FPGA reported bad command: 0x{data[0]:X2}");
                        break;

                    default:
                        Log($"Unhandled command: {command} (0x{(byte)command:X2})");
                        break;
                }

                DeviceDataUpdated?.Invoke(this, _deviceData);
            }
            catch (Exception ex)
            {
                Log($"Error processing {command}: {ex.Message}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }
        /// <summary>
        /// Sends commands from GUI to laser controller with proper packet formatting
        /// </summary>
        public void SendCommand(uiTxCommand command, byte[] data)
        {
            if (!IsConnected)
            {
                Log("Cannot send command: Serial port not open");
                return;
            }

            try
            {
                lock (_serialLock)
                {
                    byte[] packet = CommunicationHelper.CreatePacket((byte)command, data);
                    _serialPort.Write(packet, 0, packet.Length);

                    string hexData = data?.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                    Log($"SENT: {command} (0x{(byte)command:X2}), DataLen={data?.Length ?? 0}, Data=[{hexData}]");
                }
            }
            catch (Exception ex)
            {
                Log($"Error sending command {command}: {ex.Message}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }

        // Utility methods for big-endian conversion (FPGA uses big-endian)
        public byte[] ToBigEndian(ushort value) => new byte[] { (byte)(value >> 8), (byte)(value & 0xFF) };
        public byte[] ToBigEndian(uint value) => new byte[] { (byte)(value >> 24), (byte)(value >> 16), (byte)(value >> 8), (byte)(value & 0xFF) };

        private void ResetParser()
        {
            _incomingBuffer.Clear();
            _currentState = ParserState.LookingForStart;
            _expectedDataLength = 0;
            _currentCommand = 0;
        }

        private void StartParserTimeoutCheck()
        {
            Task.Run(async () =>
            {
                while (!_cts.Token.IsCancellationRequested)
                {
                    await Task.Delay(1000, _cts.Token);
                    if (_lastByteReceived != DateTime.MinValue && (DateTime.Now - _lastByteReceived) > _parserTimeout)
                    {
                        lock (_serialLock)
                        {
                            if (_incomingBuffer.Count > 0)
                            {
                                Log("Parser timeout; resetting parser");
                                ResetParser();
                            }
                        }
                    }
                }
            }, _cts.Token);
        }

        private void Log(string message)
        {
            string formattedMessage = $"{DateTime.Now:HH:mm:ss.fff} - {message}";
            _logger?.LogInformation(formattedMessage);
            LogMessage?.Invoke(this, formattedMessage + Environment.NewLine);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            try
            {
                _cts?.Cancel();
                if (_serialPort.IsOpen)
                {
                    _serialPort.DataReceived -= OnDataReceived;
                    _serialPort.ErrorReceived -= OnErrorReceived;
                    _serialPort.Close();
                }
                _serialPort.Dispose();
                Log("DeviceManager disposed");
            }
            catch (Exception ex)
            {
                Log($"Error disposing DeviceManager: {ex.Message}");
            }
            finally
            {
                _cts?.Dispose();
                _cts = null;
            }
        }

        /// <summary>
        /// Returns expected data length for commands RECEIVED from FPGA (FPGA → GUI)
        /// </summary>
        public static int GetCommandLength(uiRxCommand command)
        {
            return command switch
            {
                uiRxCommand.uiRxTestResp => 1,           // Simple response
                uiRxCommand.uiRxFPGABadCmd => 1,         // Error code
                uiRxCommand.uiRxLsrState => 1,           // LaserStateType (byte)
                uiRxCommand.uiRxLsrPulseConfig => 14,    // Frequency(2) + PulseWidth(2) + ShotTotal(4) + Delay1(2) + Delay2(2) + ShotMode(1) + TrigMode(1)
                uiRxCommand.uiRxLsrCount => 4,           // uint shot count
                uiRxCommand.uiRxLsrRunStatus => 14,      // 14 bytes
                uiRxCommand.uiRxLsrIntStatus => 1,       // byte status
                uiRxCommand.uiRxLsrIntMask => 1,         // byte mask  
                uiRxCommand.uiRxLsrWaveform => 64,       // 32 samples × 2 bytes
                uiRxCommand.uiRxDiscovery => 1,          // Simple response
                uiRxCommand.uiRxLsrVolts => 2,           // ushort voltage
                uiRxCommand.uiRxLsrChargeState => 3,     // MeasuredVolts(2) + ChargeDone(1)
                uiRxCommand.uiRxLsrChargeVolts => 2,     // ushort voltage
                uiRxCommand.uiRxShutterConfig => 2,      // ShutterMode(1) + ShutterState(1)
                uiRxCommand.uiRxSoftStartConfig => 7,    // Enable(1) + IdleSetpoint(2) + RampCount(4)
                _ => 0
            };
        }

        /// <summary>
        /// Returns expected data length for each command type (GUI → FPGA)
        /// </summary>
        public static int GetCommandLength(uiTxCommand command)
        {
            return command switch
            {
                uiTxCommand.uiTxBadCmd => 1,
                uiTxCommand.uiTxNoCmd => 1,
                uiTxCommand.uiTxFPGABadCmd => 1,
                uiTxCommand.uiTxLsrPulseConfig => 14,
                uiTxCommand.uiTxLsrState => 1,
                uiTxCommand.uiTxIntMask => 1,
                uiTxCommand.uiTxWaveState => 1,
                uiTxCommand.uiTxLsrDelays => 4,
                uiTxCommand.uiTxLsrCal => 2,
                uiTxCommand.uiTxLsrVolts => 2,
                uiTxCommand.uiTxLsrChargeCancel => 1,
                uiTxCommand.uiTxShutterConfig => 2,
                uiTxCommand.uiTxSoftStartConfig => 7,
                _ => 0
            };
        }
    }
}
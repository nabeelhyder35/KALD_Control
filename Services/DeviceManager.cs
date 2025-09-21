using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using KALD_Control.Models;
using Microsoft.Extensions.Logging;

namespace KALD_Control.Services
{
    public class DeviceManager : IDisposable
    {
        private readonly SerialPort _serialPort;
        private readonly ILogger<DeviceManager> _logger;
        private readonly object _serialLock = new object();
        private bool _disposed = false;
        private readonly List<byte> _incomingBuffer = new List<byte>();
        private enum ParserState { LookingForStart, ReadingLength, ReadingCommand, ReadingData, ReadingChecksum, ReadingEnd }
        private ParserState _currentState = ParserState.LookingForStart;
        private int _expectedDataLength = 0;
        private byte _currentCommand = 0;
        private DateTime _lastByteReceived = DateTime.MinValue;
        private readonly TimeSpan _parserTimeout = TimeSpan.FromSeconds(5);
        private CancellationTokenSource _cts;
        private int _discoveryRetries = 0;
        private const int MAX_DISCOVERY_RETRIES = 10; // Increased retries
        private bool _deviceReady = false;

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

        public bool IsConnected => _serialPort?.IsOpen ?? false;
        public bool DeviceReady => _deviceReady;
        public int BaudRate => _serialPort?.BaudRate ?? 115200;

        public DeviceManager(ILogger<DeviceManager> logger)
        {
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
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
            StartParserTimeoutCheck();
        }

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

        public void Connect(string portName, int baudRate = 115200)
        {
            if (IsConnected)
            {
                Log("Disconnecting existing connection");
                Disconnect();
            }

            try
            {
                _serialPort.PortName = portName;
                _serialPort.BaudRate = baudRate;
                _serialPort.Parity = Parity.None;
                _serialPort.DataBits = 8;
                _serialPort.StopBits = StopBits.One;
                _serialPort.Handshake = Handshake.None;
                _serialPort.ReadTimeout = 2000;
                _serialPort.WriteTimeout = 2000;
                _serialPort.DtrEnable = false;
                _serialPort.RtsEnable = false;
                _serialPort.DiscardNull = false;

                _serialPort.Open();
                _cts = new CancellationTokenSource();
                _serialPort.DataReceived += OnDataReceived;
                Log($"Connected to {portName} at {baudRate} baud");
                LogSerialPortSettings();
                _serialPort.DiscardInBuffer();
                _serialPort.DiscardOutBuffer();
                Thread.Sleep(100);
                ResetParser();
                _discoveryRetries = 0;
                _deviceReady = false;
                SendDiscoveryCommand();
            }
            catch (UnauthorizedAccessException ex)
            {
                Log($"Access denied to {portName}: {ex.Message}");
                CommandError?.Invoke(this, ("Connect", ex));
                throw;
            }
            catch (Exception ex)
            {
                Log($"Connection failed to {portName}: {ex.Message}");
                CommandError?.Invoke(this, ("Connect", ex));
                throw new InvalidOperationException($"Could not connect to {portName}.", ex);
            }
        }

        public void LogSerialPortSettings()
        {
            if (IsConnected)
            {
                Log($"Serial Port Settings: {_serialPort.PortName}, {_serialPort.BaudRate} baud, {_serialPort.DataBits} data bits, {_serialPort.StopBits} stop bits, {_serialPort.Parity} parity, Handshake: {_serialPort.Handshake}, DTR: {_serialPort.DtrEnable}, RTS: {_serialPort.RtsEnable}");
            }
            else
            {
                Log("Serial port not open");
            }
        }

        public void Disconnect()
        {
            if (!IsConnected)
            {
                Log("Disconnect requested but no active connection");
                return;
            }

            try
            {
                _cts?.Cancel();
                _serialPort.DataReceived -= OnDataReceived;
                _serialPort.Close();
                ResetParser();
                _deviceReady = false;
                DiscoveryResponseReceived?.Invoke(this, false);
                Log("Disconnected from serial port");
            }
            catch (Exception ex)
            {
                Log($"Disconnection error: {ex.Message}");
                CommandError?.Invoke(this, ("Disconnect", ex));
            }
            finally
            {
                _cts?.Dispose();
                _cts = null;
            }
        }

        private void SendDiscoveryCommand()
        {
            if (_discoveryRetries >= MAX_DISCOVERY_RETRIES)
            {
                Log("Max discovery retries reached; aborting");
                _deviceReady = false;
                DiscoveryResponseReceived?.Invoke(this, false);
                return;
            }

            _discoveryRetries++;
            SendCommand(LcdRxCommand.lcdRxDiscovery, new byte[] { 0x03, 0xE8 });
            Log($"Sent discovery command (attempt {_discoveryRetries}/{MAX_DISCOVERY_RETRIES}): 2A 00 02 09 03 E8 0C 3A");
        }

        public void SendVoltage(ushort voltage) => SendCommand(LcdRxCommand.lcdRxLsrVolts, ToBigEndian(voltage));
        public void SendReadEnergy() => SendCommand(LcdRxCommand.lcdRxReadEnergy, new byte[0]);
        public void SendReadTemperature() => SendCommand(LcdRxCommand.lcdRxReadTemperature, new byte[0]);
        public void SendSystemInfoRequest() => SendCommand(LcdRxCommand.lcdRxSystemInfo, new byte[0]);

        public void SendCalibration(CalibrationData calibration)
        {
            byte[] data = ToBigEndian(calibration.CapVoltRange);
            SendCommand(LcdRxCommand.lcdRxLsrCal, data);
            Log($"Sent calibration: CapVoltRange={calibration.CapVoltRange}V");
        }

        public void SendPulseConfig(PulseConfig config)
        {
            var data = new List<byte>();
            if (config.Frequency < ushort.MinValue || config.Frequency > ushort.MaxValue)
            {
                Log($"Frequency out of range: {config.Frequency}");
                return;
            }
            data.AddRange(ToBigEndian(config.Frequency));
            data.AddRange(ToBigEndian(config.PulseWidth));
            data.AddRange(ToBigEndian(config.ShotTotal));
            data.AddRange(ToBigEndian(config.Delay1));
            data.AddRange(ToBigEndian(config.Delay2));
            data.Add((byte)config.ShotMode);
            data.Add((byte)config.TrigMode);

            if (data.Count != FpgaCommandLengths.LSR_PULSE_CONFIG)
            {
                Log($"Pulse config length error: Got {data.Count}, expected {FpgaCommandLengths.LSR_PULSE_CONFIG}");
                return;
            }

            SendCommand(LcdRxCommand.lcdRxLsrPulseConfig, data.ToArray());
            Log($"Sent pulse config: Frequency={config.Frequency / 10.0:F1}Hz, PulseWidth={config.PulseWidth}us, Shots={config.ShotTotal}, Delay1={config.Delay1}us, Delay2={config.Delay2}us, ShotMode={config.ShotMode}, TrigMode={config.TrigMode}");
        }

        public void SendStateCommand(LaserStateType state)
        {
            SendCommand(LcdRxCommand.lcdRxLsrState, new[] { (byte)state });
            Log($"Sent state command: State={state}");
        }

        public void SendInterlockMask(byte mask)
        {
            SendCommand(LcdRxCommand.lcdRxLsrIntMask, new[] { mask });
            Log($"Sent interlock mask: 0x{mask:X2}");
        }

        public void SendWaveformState(bool enable)
        {
            SendCommand(LcdRxCommand.lcdRxLsrWaveform, new[] { (byte)(enable ? 1 : 0) });
            Log($"Sent waveform state: Enable={enable}");
        }

        public void SendLaserDelays(ushort delay1, ushort delay2)
        {
            var data = new List<byte>();
            data.AddRange(ToBigEndian(delay1));
            data.AddRange(ToBigEndian(delay2));

            if (data.Count != FpgaCommandLengths.LSR_DELAYS)
            {
                Log($"Laser delays length error: Got {data.Count}, expected {FpgaCommandLengths.LSR_DELAYS}");
                return;
            }

            SendCommand(LcdRxCommand.lcdRxLsrDelays, data.ToArray());
            Log($"Sent delays: Delay1={delay1}us, Delay2={delay2}us");
        }

        public void SendDigitalIOCommand()
        {
            SendCommand(LcdRxCommand.lcdRxDigitalIO, new byte[0]);
            Log("Sent digital IO command");
        }

        public void SendSystemReset()
        {
            SendCommand(LcdRxCommand.lcdRxSystemReset, new byte[0]);
            Log("Sent system reset");
        }

        public void RequestSystemStatus()
        {
            SendCommand(LcdRxCommand.lcdRxLsrRunStatus, new byte[0]);
            Log("Requested system status");
        }

        public void RequestWaveformData()
        {
            SendCommand(LcdRxCommand.lcdRxLsrWaveform, new byte[0]);
            Log("Requested waveform data");
        }

        public void SendChargeCancel()
        {
            SendCommand(LcdRxCommand.lcdRxLsrChargeCancel, new byte[] { 1 });
            Log("Sent charge cancel");
        }

        public void SendShutterConfig(ShutterConfig config)
        {
            SendCommand(LcdRxCommand.lcdRxShutterConfig, new byte[] { (byte)config.ShutterMode, (byte)config.ShutterState });
            Log($"Sent shutter config: Mode={config.ShutterMode}, State={config.ShutterState}");
        }

        public void SendSoftStartConfig(SoftStartConfig config)
        {
            var data = new List<byte>();
            data.Add((byte)(config.Enable ? 1 : 0));
            data.AddRange(ToBigEndian(config.IdleSetpoint));
            data.AddRange(ToBigEndian(config.RampCount));

            SendCommand(LcdRxCommand.lcdRxSoftStartConfig, data.ToArray());
            Log($"Sent soft start config: Enable={config.Enable}, Idle={config.IdleSetpoint}V, Ramp={config.RampCount}");
        }

        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            lock (_serialLock)
            {
                try
                {
                    while (_serialPort.BytesToRead > 0 && !(_cts?.IsCancellationRequested ?? true))
                    {
                        byte b = (byte)_serialPort.ReadByte();
                        ProcessByte(b);
                    }
                }
                catch (Exception ex)
                {
                    Log($"Error in OnDataReceived: {ex.Message}");
                    CommandError?.Invoke(this, ("OnDataReceived", ex));
                    SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                }
            }
        }

        private void ProcessByte(byte b)
        {
            _lastByteReceived = DateTime.Now;
            _incomingBuffer.Add(b);
            Log($"Processing byte 0x{b:X2}, State={_currentState}, Buffer=[{BitConverter.ToString(_incomingBuffer.ToArray()).Replace("-", " ")}]");

            switch (_currentState)
            {
                case ParserState.LookingForStart:
                    if (b == ProtocolConstants.STX)
                    {
                        _incomingBuffer.Clear();
                        _incomingBuffer.Add(b);
                        _currentState = ParserState.ReadingLength;
                    }
                    else
                    {
                        _incomingBuffer.Clear();
                    }
                    break;

                case ParserState.ReadingLength:
                    if (_incomingBuffer.Count == 3)
                    {
                        _expectedDataLength = (_incomingBuffer[1] << 8) | _incomingBuffer[2];
                        if (_expectedDataLength < 0)
                        {
                            Log($"Invalid data length: {_expectedDataLength}");
                            ResetParser();
                        }
                        else
                        {
                            _currentState = ParserState.ReadingCommand;
                        }
                    }
                    break;

                case ParserState.ReadingCommand:
                    _currentCommand = b;
                    _currentState = _expectedDataLength > 0 ? ParserState.ReadingData : ParserState.ReadingChecksum;
                    break;

                case ParserState.ReadingData:
                    if (_incomingBuffer.Count == 4 + _expectedDataLength)
                    {
                        _currentState = ParserState.ReadingChecksum;
                    }
                    break;

                case ParserState.ReadingChecksum:
                    _currentState = ParserState.ReadingEnd;
                    break;

                case ParserState.ReadingEnd:
                    if (b == ProtocolConstants.ETX)
                    {
                        ProcessPacket();
                    }
                    else
                    {
                        Log($"Invalid packet: Expected ETX (0x3A), got 0x{b:X2}");
                        SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                    }
                    ResetParser();
                    break;
            }
        }

        private void ProcessPacket()
        {
            try
            {
                byte[] packet = _incomingBuffer.ToArray();
                var validationResult = CommunicationHelper.ValidatePacket(packet);
                if (validationResult != PacketValidationResult.Valid)
                {
                    Log($"Packet validation failed: {validationResult}, Packet=[{BitConverter.ToString(packet).Replace("-", " ")}]");
                    SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                    ResetParser();
                    return;
                }

                if (!CommunicationHelper.ExtractPacketData(packet, out byte commandByte, out byte[] data))
                {
                    Log($"Failed to extract packet data: Packet=[{BitConverter.ToString(packet).Replace("-", " ")}]");
                    SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                    ResetParser();
                    return;
                }

                string hexData = data.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                Log($"RECV: 0x{commandByte:X2}, DataLen={data.Length}, Data=[{hexData}], Packet=[{BitConverter.ToString(packet).Replace("-", " ")}]");

                if (Enum.IsDefined(typeof(LcdRxCommand), commandByte))
                {
                    LcdRxCommand rxCommand = (LcdRxCommand)commandByte;
                    Log($"Received LcdRxCommand: {rxCommand} (0x{commandByte:X2})");
                    switch (rxCommand)
                    {
                        case LcdRxCommand.lcdRxDiscovery:
                            Log($"Received discovery response: Data=[{hexData}]");
                            if (data.Length == 1 && data[0] == 0x00)
                            {
                                Log($"Device returned lcdRxDiscovery with data 0x00; retrying discovery (attempt {_discoveryRetries}/{MAX_DISCOVERY_RETRIES})");
                                SendDiscoveryCommand();
                            }
                            else
                            {
                                Log($"Unexpected discovery data; expected LcdTxCommand.lcdTxDiscovery (0x08), got {hexData}");
                                SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                                SendDiscoveryCommand();
                            }
                            break;

                        case LcdRxCommand.lcdRxLsrState:
                            if (data.Length >= 1)
                            {
                                StateUpdated?.Invoke(this, (LaserStateType)data[0]);
                                Log($"Received state: {(LaserStateType)data[0]}");
                            }
                            else
                            {
                                Log($"Invalid state data length: {data.Length}");
                                SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                            }
                            break;

                        case LcdRxCommand.lcdRxLsrIntMask:
                            if (data.Length >= 1)
                            {
                                IntMaskUpdated?.Invoke(this, data[0]);
                                Log($"Received interlock mask: 0x{data[0]:X2}");
                            }
                            else
                            {
                                Log($"Invalid interlock mask data length: {data.Length}");
                                SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                            }
                            break;

                        default:
                            Log($"Unhandled LcdRxCommand: {rxCommand} (0x{commandByte:X2})");
                            SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                            break;
                    }
                    ResetParser();
                    return;
                }
                else if (Enum.IsDefined(typeof(LcdTxCommand), commandByte))
                {
                    LcdTxCommand command = (LcdTxCommand)commandByte;
                    int expectedLength = FpgaCommandLengths.GetCommandLength(command);

                    if (expectedLength >= 0 && data.Length != expectedLength)
                    {
                        Log($"Invalid data length for {command}: Expected {expectedLength}, got {data.Length}");
                        SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                        ResetParser();
                        return;
                    }

                    Log($"Processing LcdTxCommand: {command} (0x{commandByte:X2})");

                    switch (command)
                    {
                        case LcdTxCommand.lcdTxDiscovery:
                            Log("Received discovery response");
                            if (data.Length >= 2 && FromBigEndianUshort(data, 0) == 0x03E8)
                            {
                                Log("Discovery successful: NIOS system detected");
                                _deviceReady = true;
                                DiscoveryResponseReceived?.Invoke(this, true);
                                _discoveryRetries = 0;
                                // Request initial state and interlock mask to sync UI
                                SendCommand(LcdRxCommand.lcdRxLsrState, new byte[0]);
                                SendCommand(LcdRxCommand.lcdRxLsrIntMask, new byte[0]);
                            }
                            else
                            {
                                Log($"Unexpected discovery data: [{hexData}]");
                                _deviceReady = false;
                                DiscoveryResponseReceived?.Invoke(this, false);
                                SendDiscoveryCommand();
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrPulseConfig:
                            if (data.Length >= 14)
                            {
                                var config = new PulseConfig
                                {
                                    Frequency = FromBigEndianUshort(data, 0),
                                    PulseWidth = FromBigEndianUshort(data, 2),
                                    ShotTotal = FromBigEndianUshort(data, 4),
                                    Delay1 = FromBigEndianUshort(data, 6),
                                    Delay2 = FromBigEndianUshort(data, 8),
                                    ShotMode = (ShotModeType)data[10],
                                    TrigMode = (TriggerModeType)data[11]
                                };
                                PulseConfigUpdated?.Invoke(this, config);
                                Log($"Received pulse config: Frequency={config.Frequency / 10.0:F1}Hz, PulseWidth={config.PulseWidth}us, Shots={config.ShotTotal}, Delay1={config.Delay1}us, Delay2={config.Delay2}us, ShotMode={config.ShotMode}, TrigMode={config.TrigMode}");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrState:
                            if (data.Length >= 1)
                            {
                                StateUpdated?.Invoke(this, (LaserStateType)data[0]);
                                Log($"Received state: {(LaserStateType)data[0]}");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrCount:
                            if (data.Length >= 4)
                            {
                                uint shotCount = FromBigEndianUint(data, 0);
                                ShotCountUpdated?.Invoke(this, shotCount);
                                Log($"Received shot count: {shotCount}");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrRunStatus:
                            if (data.Length >= 14)
                            {
                                var status = new RunStatusData
                                {
                                    Energy = FromBigEndianUshort(data, 0),
                                    Power = FromBigEndianUshort(data, 2),
                                    ShotCount = FromBigEndianUint(data, 4),
                                    Current = FromBigEndianUshort(data, 8),
                                    VDroop = FromBigEndianUshort(data, 10),
                                    State = (LaserStateType)data[12]
                                };
                                RunStatusUpdated?.Invoke(this, status);
                                Log($"Received run status: Energy={status.Energy}, Power={status.Power}, Shots={status.ShotCount}, Current={status.Current}, VDroop={status.VDroop}, State={status.State}");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrIntStatus:
                            if (data.Length >= 1)
                            {
                                IntStatusUpdated?.Invoke(this, data[0]);
                                Log($"Received interlock status: 0x{data[0]:X2}");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrIntMask:
                            if (data.Length >= 1)
                            {
                                IntMaskUpdated?.Invoke(this, data[0]);
                                Log($"Received interlock mask: 0x{data[0]:X2}");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrWaveform:
                            if (data.Length >= 64)
                            {
                                ushort[] samples = new ushort[32];
                                for (int i = 0; i < 32; i++)
                                {
                                    samples[i] = FromBigEndianUshort(data, i * 2);
                                }
                                var waveform = new WaveformData
                                {
                                    CaptureTime = DateTime.Now
                                };
                                Array.Copy(samples, waveform.Samples, 32);
                                WaveformUpdated?.Invoke(this, waveform);
                                Log($"Received waveform: {waveform.Samples.Length} samples");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrVolts:
                            if (data.Length >= 2)
                            {
                                ushort volts = FromBigEndianUshort(data, 0);
                                VoltsUpdated?.Invoke(this, volts);
                                Log($"Received voltage: {volts}V");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrChargeState:
                            if (data.Length >= 3)
                            {
                                var chargeState = new ChargeStateData
                                {
                                    MeasuredVolts = FromBigEndianUshort(data, 0),
                                    ChargeDone = data[2] != 0
                                };
                                ChargeStateUpdated?.Invoke(this, chargeState);
                                Log($"Received charge state: Volts={chargeState.MeasuredVolts}V, Done={chargeState.ChargeDone}");
                            }
                            break;

                        case LcdTxCommand.lcdTxShutterConfig:
                            if (data.Length >= 2)
                            {
                                var config = new ShutterConfig
                                {
                                    ShutterMode = (ShutterModeType)(data[0] <= 1 ? data[0] : 0), // Validate enum
                                    ShutterState = (ShutterStateType)(data[1] <= 1 ? data[1] : 0) // Validate enum
                                };
                                ShutterConfigUpdated?.Invoke(this, config);
                                Log($"Received shutter config: Mode={config.ShutterMode}, State={config.ShutterState}");
                            }
                            else
                            {
                                Log($"Invalid shutter config data length: {data.Length}");
                                SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                            }
                            break;

                        case LcdTxCommand.lcdTxSoftStartConfig:
                            if (data.Length >= 7)
                            {
                                var config = new SoftStartConfig
                                {
                                    Enable = data[0] != 0,
                                    IdleSetpoint = FromBigEndianUshort(data, 1),
                                    RampCount = FromBigEndianUint(data, 3)
                                };
                                SoftStartConfigUpdated?.Invoke(this, config);
                                Log($"Received soft start config: Enable={config.Enable}, Idle={config.IdleSetpoint}V, Ramp={config.RampCount}");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrCal:
                            if (data.Length >= 2)
                            {
                                var calibration = new CalibrationData
                                {
                                    CapVoltRange = FromBigEndianUshort(data, 0),
                                    MeasuredVoltage = 0
                                };
                                CalibrationUpdated?.Invoke(this, calibration);
                                Log($"Received calibration: CapVoltRange={calibration.CapVoltRange}V");
                            }
                            break;

                        case LcdTxCommand.lcdTxLsrDelays:
                            if (data.Length >= 4)
                            {
                                ushort delay1 = FromBigEndianUshort(data, 0);
                                ushort delay2 = FromBigEndianUshort(data, 2);
                                Log($"Received delays: Delay1={delay1}us, Delay2={delay2}us");
                            }
                            break;

                        case LcdTxCommand.lcdTxDigitalIO:
                            if (data.Length >= 8)
                            {
                                var dioState = new DigitalIOState
                                {
                                    InputStates = FromBigEndianUint(data, 0),
                                    OutputStates = FromBigEndianUint(data, 4),
                                    Timestamp = DateTime.Now
                                };
                                DigitalIOUpdated?.Invoke(this, dioState);
                                Log($"Received digital IO: InputStates=0x{dioState.InputStates:X8}, OutputStates=0x{dioState.OutputStates:X8}");
                            }
                            break;

                        default:
                            Log($"Unhandled command: {command} (0x{commandByte:X2})");
                            SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                            break;
                    }
                }
                else
                {
                    Log($"Invalid command byte: 0x{commandByte:X2}");
                    SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
                }
            }
            catch (Exception ex)
            {
                Log($"Error processing packet: {ex.Message}");
                CommandError?.Invoke(this, ("ProcessPacket", ex));
                SendCommand(LcdRxCommand.lcdRxBadCmd, new byte[] { 0 });
            }
            finally
            {
                ResetParser();
            }
        }

        private void ResetParser()
        {
            _incomingBuffer.Clear();
            _currentState = ParserState.LookingForStart;
            _expectedDataLength = 0;
            _currentCommand = 0;
            Log("Parser reset");
        }

        private void StartParserTimeoutCheck()
        {
            Task.Run(async () =>
            {
                while (!_disposed)
                {
                    await Task.Delay(100, _cts?.Token ?? CancellationToken.None);
                    if (_currentState != ParserState.LookingForStart &&
                        DateTime.Now - _lastByteReceived > _parserTimeout)
                    {
                        Log($"Parser timeout; resetting. Buffer=[{BitConverter.ToString(_incomingBuffer.ToArray()).Replace("-", " ")}]");
                        ResetParser();
                    }
                }
            });
        }

        public void SendCommand(LcdRxCommand command, byte[] data)
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
                    string hexPacket = BitConverter.ToString(packet).Replace("-", " ");
                    Log($"SENT: {hexPacket}");
                    string hexData = data?.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                    Log($"SENT: {command} (0x{(byte)command:X2}), DataLen={data?.Length ?? 0}, Data=[{hexData}], Checksum=0x{packet[packet.Length - 2]:X2}");
                }
            }
            catch (Exception ex)
            {
                Log($"Error sending command {command}: {ex.Message}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }

        public void SendCommand(LcdTxCommand command, byte[] data)
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
                    string hexPacket = BitConverter.ToString(packet).Replace("-", " ");
                    Log($"SENT: {hexPacket}");
                    string hexData = data?.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                    Log($"SENT: {command} (0x{(byte)command:X2}), DataLen={data?.Length ?? 0}, Data=[{hexData}], Checksum=0x{packet[packet.Length - 2]:X2}");
                }
            }
            catch (Exception ex)
            {
                Log($"Error sending command {command}: {ex.Message}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }

        private byte[] ToBigEndian(ushort value) => new byte[] { (byte)(value >> 8), (byte)(value & 0xFF) };
        private byte[] ToBigEndian(uint value) => new byte[] { (byte)(value >> 24), (byte)(value >> 16), (byte)(value >> 8), (byte)(value & 0xFF) };

        private ushort FromBigEndianUshort(byte[] data, int startIndex = 0)
        {
            if (data == null || startIndex < 0 || startIndex + 2 > data.Length)
                return 0;
            return (ushort)((data[startIndex] << 8) | data[startIndex + 1]);
        }

        private uint FromBigEndianUint(byte[] data, int startIndex = 0)
        {
            if (data == null || startIndex < 0 || startIndex + 4 > data.Length)
                return 0;
            return (uint)((data[startIndex] << 24) | (data[startIndex + 1] << 16) |
                         (data[startIndex + 2] << 8) | data[startIndex + 3]);
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
    }

    public static class FpgaCommandLengths
    {
        public const int LSR_PULSE_CONFIG = 14;
        public const int LSR_STATE = 1;
        public const int LSR_SHOTS = 4;
        public const int RUN_STATUS = 14;
        public const int INT_STATUS = 1;
        public const int INT_MASK = 1;
        public const int WAVEFORM = 64;
        public const int DISCOVERY = 2;
        public const int LSR_VOLTS = 2;
        public const int CHARGE_STATE = 3;
        public const int CHARGE_VOLT = 2;
        public const int SHUTTER_CONFIG = 2;
        public const int SOFT_START_CONFIG = 7;
        public const int LSR_CAL = 2;
        public const int LSR_DELAYS = 4;
        public const int DIGITAL_IO = 8;

        public static int GetCommandLength(LcdTxCommand command)
        {
            return command switch
            {
                LcdTxCommand.lcdTxLsrPulseConfig => LSR_PULSE_CONFIG,
                LcdTxCommand.lcdTxLsrState => LSR_STATE,
                LcdTxCommand.lcdTxLsrCount => LSR_SHOTS,
                LcdTxCommand.lcdTxLsrRunStatus => RUN_STATUS,
                LcdTxCommand.lcdTxLsrIntStatus => INT_STATUS,
                LcdTxCommand.lcdTxLsrIntMask => INT_MASK,
                LcdTxCommand.lcdTxLsrWaveform => WAVEFORM,
                LcdTxCommand.lcdTxDiscovery => DISCOVERY,
                LcdTxCommand.lcdTxLsrVolts => LSR_VOLTS,
                LcdTxCommand.lcdTxLsrChargeState => CHARGE_STATE,
                LcdTxCommand.lcdTxLsrChargeVolts => CHARGE_VOLT,
                LcdTxCommand.lcdTxShutterConfig => SHUTTER_CONFIG,
                LcdTxCommand.lcdTxSoftStartConfig => SOFT_START_CONFIG,
                LcdTxCommand.lcdTxLsrCal => LSR_CAL,
                LcdTxCommand.lcdTxLsrDelays => LSR_DELAYS,
                LcdTxCommand.lcdTxDigitalIO => DIGITAL_IO,
                _ => -1
            };
        }
    }
}
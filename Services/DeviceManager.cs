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
    public enum uiRxCommand : byte
    {
        uiTxDiscovery = 0x09,
        uiTxLsrState = 0x02,
        uiTxLsrIntMask = 0x04,
        uiTxLsrVolts = 0x0A,
        uiTxLsrPulseConfig = 0x01,
        uiTxShutterConfig = 0x0B,
        uiTxSoftStartConfig = 0x0C,
        uiTxLsrCal = 0x0D,
        uiTxLsrDelays = 0x0E,
        uiTxDigitalIO = 0x0F,
        uiTxSystemReset = 0x10,
        uiTxLsrRunStatus = 0x11,
        uiTxLsrWaveform = 0x12,
        uiTxLsrChargeCancel = 0x13,
        uiTxReadEnergy = 0x14,
        uiTxReadTemperature = 0x15,
        uiTxSystemInfo = 0x16,
        uiTxLsrBadCmd = 0xFF
    }

    public enum uiTxCommand : byte
    {
        uiRxDiscovery = 0x08,
        uiRxLsrPulseConfig = 0x01,
        uiRxLsrState = 0x02,
        uiRxLsrCount = 0x03,
        uiRxLsrIntStatus = 0x04,
        uiRxLsrIntMask = 0x05,
        uiRxLsrWaveform = 0x06,
        uiRxLsrVolts = 0x07,
        uiRxLsrChargeState = 0x09,
        uiRxLsrChargeVolts = 0x0A,
        uiRxShutterConfig = 0x0B,
        uiRxSoftStartConfig = 0x0C,
        uiRxLsrCal = 0x0D,
        uiRxLsrDelays = 0x0E,
        uiRxDigitalIO = 0x0F,
        uiRxLsrRunStatus = 0x11
    }

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
        private const int MAX_DISCOVERY_RETRIES = 10;
        private bool _deviceReady = false;

        public event EventHandler<LaserStateType> StateUpdated;
        public event EventHandler<PulseConfigData> PulseConfigUpdated;
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
        public event EventHandler<string> LogMessage;
        public event EventHandler<(string Command, Exception Exception)> CommandError;
        public event EventHandler<bool> DiscoveryResponseReceived;
        public event EventHandler<DeviceData> DeviceDataUpdated;
        public event EventHandler<InterlockStatus> InterlockStatusUpdated;
        public event EventHandler<ushort> ChargeVoltsUpdated;
        public event EventHandler<bool> DeviceReadyChanged;

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
                ReadTimeout = 5000,
                WriteTimeout = 5000,
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

        public async Task ConnectAsync(string portName, int baudRate = 115200)
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
                _serialPort.ReadTimeout = 5000;
                _serialPort.WriteTimeout = 5000;
                _serialPort.DtrEnable = false;
                _serialPort.RtsEnable = false;
                _serialPort.DiscardNull = false;

                await Task.Run(() => _serialPort.Open());
                _cts = new CancellationTokenSource();
                _serialPort.DataReceived += OnDataReceived;
                Log($"Connected to {portName} at {baudRate} baud");
                LogSerialPortSettings();
                _serialPort.DiscardInBuffer();
                _serialPort.DiscardOutBuffer();
                await Task.Delay(100);
                ResetParser();
                _discoveryRetries = 0;
                _deviceReady = false;
                await SendDiscoveryCommandAsync();
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
                DeviceReadyChanged?.Invoke(this, false);
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

        private async Task SendDiscoveryCommandAsync()
        {
            if (_discoveryRetries >= MAX_DISCOVERY_RETRIES)
            {
                Log("Discovery failed: Max retries reached or timeout");
                _deviceReady = false;
                DiscoveryResponseReceived?.Invoke(this, false);
                DeviceReadyChanged?.Invoke(this, false);
                return;
            }

            _discoveryRetries++;
            await SendCommandAsync(uiRxCommand.uiTxDiscovery, new byte[] { 0x03, 0xE8 });
            Log($"Sent discovery command (attempt {_discoveryRetries}/{MAX_DISCOVERY_RETRIES}): 2A 00 02 09 03 E8 0C 3A");
        }

        public async Task SendVoltageAsync(ushort voltage)
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrVolts, ToBigEndian(voltage));
        }

        public async Task SendReadEnergyAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxReadEnergy, new byte[0]);
        }

        public async Task SendReadTemperatureAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxReadTemperature, new byte[0]);
        }

        public async Task SendSystemInfoRequestAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxSystemInfo, new byte[0]);
        }

        public async Task SendCalibrationAsync(ushort capVoltRange)
        {
            byte[] data = ToBigEndian(capVoltRange);
            await SendCommandAsync(uiRxCommand.uiTxLsrCal, data);
            Log($"Sent calibration: CapVoltRange={capVoltRange}V");
        }

        public async Task SendPulseConfigAsync(uint shotTotal, ushort pulseWidth, ushort frequency, ushort delay1, ushort delay2, TriggerModeType triggerMode, ShotModeType shotMode)
        {
            var data = new List<byte>();
            data.AddRange(ToBigEndian(frequency));
            data.AddRange(ToBigEndian(pulseWidth));
            data.AddRange(ToBigEndian(shotTotal));
            data.AddRange(ToBigEndian(delay1));
            data.AddRange(ToBigEndian(delay2));
            data.Add((byte)shotMode);
            data.Add((byte)triggerMode);

            if (data.Count != FpgaCommandLengths.LSR_PULSE_CONFIG)
            {
                Log($"Pulse config length error: Got {data.Count}, expected {FpgaCommandLengths.LSR_PULSE_CONFIG}");
                return;
            }

            await SendCommandAsync(uiRxCommand.uiTxLsrPulseConfig, data.ToArray());
            Log($"Sent pulse config: Frequency={frequency / 10.0:F1}Hz, PulseWidth={pulseWidth}us, Shots={shotTotal}, Delay1={delay1}us, Delay2={delay2}us, ShotMode={shotMode}, TrigMode={triggerMode}");
        }

        public async Task SendStateCommandAsync(LaserStateType state)
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrState, new[] { (byte)state });
            Log($"Sent state command: State={state}");
        }

        public async Task SendInterlockMaskAsync(byte mask)
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrIntMask, new[] { mask });
            Log($"Sent interlock mask: 0x{mask:X2}");
        }

        public async Task SendWaveformStateAsync(bool enable)
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrWaveform, new[] { (byte)(enable ? 1 : 0) });
            Log($"Sent waveform state: Enable={enable}");
        }

        public async Task SendDelaysAsync(ushort delay1, ushort delay2)
        {
            var data = new List<byte>();
            data.AddRange(ToBigEndian(delay1));
            data.AddRange(ToBigEndian(delay2));

            if (data.Count != FpgaCommandLengths.LSR_DELAYS)
            {
                Log($"Laser delays length error: Got {data.Count}, expected {FpgaCommandLengths.LSR_DELAYS}");
                return;
            }

            await SendCommandAsync(uiRxCommand.uiTxLsrDelays, data.ToArray());
            Log($"Sent delays: Delay1={delay1}us, Delay2={delay2}us");
        }

        public async Task SendDigitalIOCommandAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxDigitalIO, new byte[0]);
            Log("Sent digital IO command");
        }

        public async Task SendSystemResetAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxSystemReset, new byte[0]);
            Log("Sent system reset");
        }

        public async Task RequestSystemStatusAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrRunStatus, new byte[0]);
            Log("Requested system status");
        }

        public async Task RequestWaveformAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrWaveform, new byte[0]);
            Log("Requested waveform data");
        }

        public async Task SendChargeCancelAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrChargeCancel, new byte[] { 1 });
            Log("Sent charge cancel");
        }

        public async Task SendShutterConfigAsync(ShutterModeType mode, ShutterStateType state)
        {
            if (!Enum.IsDefined(typeof(ShutterModeType), mode) || !Enum.IsDefined(typeof(ShutterStateType), state))
            {
                Log($"Invalid shutter config: Mode={mode}, State={state}");
                return;
            }
            await SendCommandAsync(uiRxCommand.uiTxShutterConfig, new byte[] { (byte)mode, (byte)state });
            Log($"Sent shutter config: Mode={mode}, State={state}");
        }

        public async Task SendSoftStartConfigAsync(bool enable, ushort idleSetpoint, uint rampCount)
        {
            var data = new List<byte>();
            data.Add((byte)(enable ? 1 : 0));
            data.AddRange(ToBigEndian(idleSetpoint));
            data.AddRange(ToBigEndian(rampCount));

            await SendCommandAsync(uiRxCommand.uiTxSoftStartConfig, data.ToArray());
            Log($"Sent soft start config: Enable={enable}, Idle={idleSetpoint}V, Ramp={rampCount}");
        }

        public async Task RequestChargeStateAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrChargeCancel, new byte[0]);
            Log("Requested charge state");
        }

        public async Task RequestChargeVoltsAsync()
        {
            await SendCommandAsync(uiRxCommand.uiTxLsrVolts, new byte[0]);
            Log("Requested charge volts");
        }

        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            lock (_serialLock)
            {
                try
                {
                    int bytesToRead = _serialPort.BytesToRead;
                    byte[] buffer = new byte[bytesToRead];
                    int bytesRead = _serialPort.Read(buffer, 0, bytesToRead);
                    Log($"Received {bytesRead} bytes: [{BitConverter.ToString(buffer, 0, bytesRead).Replace("-", " ")}]");
                    for (int i = 0; i < bytesRead; i++)
                    {
                        ProcessByte(buffer[i]);
                    }
                }
                catch (Exception ex)
                {
                    Log($"Error in OnDataReceived: {ex.Message}");
                    CommandError?.Invoke(this, ("OnDataReceived", ex));
                    SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { _currentCommand });
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
                        if (_expectedDataLength < 0 || _expectedDataLength > 256)
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
                        SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { _currentCommand });
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
                    SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { _currentCommand });
                    ResetParser();
                    return;
                }

                if (!CommunicationHelper.ExtractPacketData(packet, out byte commandByte, out byte[] data))
                {
                    Log($"Failed to extract packet data: Packet=[{BitConverter.ToString(packet).Replace("-", " ")}]");
                    SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { _currentCommand });
                    ResetParser();
                    return;
                }

                string hexData = data.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                Log($"RECV: Command=0x{commandByte:X2}, DataLen={data.Length}, Data=[{hexData}], Packet=[{BitConverter.ToString(packet).Replace("-", " ")}]");

                if (commandByte == (byte)uiRxCommand.uiTxDiscovery)
                {
                    Log("Received discovery response");
                    if (data.Length == 1 && data[0] == 0x00)
                    {
                        Log("Discovery successful: NIOS system detected");
                        _deviceReady = true;
                        _discoveryRetries = 0;
                        DiscoveryResponseReceived?.Invoke(this, true);
                        DeviceReadyChanged?.Invoke(this, true);
                        SendCommand(uiRxCommand.uiTxLsrState, new byte[0]);
                        SendCommand(uiRxCommand.uiTxLsrIntMask, new byte[0]);
                    }
                    else
                    {
                        Log($"Unexpected discovery data: [{hexData}]");
                        SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { commandByte });
                        SendDiscoveryCommandAsync().GetAwaiter().GetResult();
                    }
                    ResetParser();
                    return;
                }

                if (Enum.IsDefined(typeof(uiTxCommand), commandByte))
                {
                    uiTxCommand rxCommand = (uiTxCommand)commandByte;
                    Log($"Received uiTxCommand: {rxCommand} (0x{commandByte:X2})");
                    int expectedLength = FpgaCommandLengths.GetCommandLength(rxCommand);
                    if (expectedLength >= 0 && data.Length != expectedLength)
                    {
                        Log($"Invalid data length for {rxCommand}: Expected {expectedLength}, got {data.Length}");
                        SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { commandByte });
                        ResetParser();
                        return;
                    }

                    switch (rxCommand)
                    {
                        case uiTxCommand.uiRxLsrPulseConfig:
                            if (data.Length >= 14)
                            {
                                var config = new PulseConfigData
                                {
                                    Frequency = FromBigEndianUshort(data, 0),
                                    PulseWidth = FromBigEndianUshort(data, 2),
                                    ShotCount = FromBigEndianUint(data, 4),
                                    Delay1 = FromBigEndianUshort(data, 6),
                                    Delay2 = FromBigEndianUshort(data, 8),
                                    ShotMode = (ShotModeType)data[10],
                                    TriggerMode = (TriggerModeType)data[11]
                                };
                                PulseConfigUpdated?.Invoke(this, config);
                                Log($"Received pulse config: Frequency={config.Frequency / 10.0:F1}Hz, PulseWidth={config.PulseWidth}us, Shots={config.ShotCount}, Delay1={config.Delay1}us, Delay2={config.Delay2}us, ShotMode={config.ShotMode}, TrigMode={config.TriggerMode}");
                            }
                            break;

                        case uiTxCommand.uiRxLsrState:
                            if (data.Length >= 1)
                            {
                                StateUpdated?.Invoke(this, (LaserStateType)data[0]);
                                Log($"Received state: {(LaserStateType)data[0]}");
                            }
                            break;

                        case uiTxCommand.uiRxLsrCount:
                            if (data.Length >= 4)
                            {
                                uint shotCount = FromBigEndianUint(data, 0);
                                ShotCountUpdated?.Invoke(this, shotCount);
                                Log($"Received shot count: {shotCount}");
                            }
                            break;

                        case uiTxCommand.uiRxLsrRunStatus:
                            if (data.Length >= 14)
                            {
                                var status = new RunStatusData
                                {
                                    Energy = FromBigEndianUshort(data, 0),
                                    Power = FromBigEndianUshort(data, 2),
                                    ShotCount = FromBigEndianUint(data, 4),
                                    Current = FromBigEndianUshort(data, 8),
                                    VoltageDroop = FromBigEndianUshort(data, 10),
                                    State = (LaserStateType)data[12]
                                };
                                RunStatusUpdated?.Invoke(this, status);
                                DeviceDataUpdated?.Invoke(this, new DeviceData
                                {
                                    SystemState = status.State,
                                    ShotCount = status.ShotCount,
                                    RunStatus = status
                                });
                                Log($"Received run status: Energy={status.Energy}, Power={status.Power}, Shots={status.ShotCount}, Current={status.Current}, VoltageDroop={status.VoltageDroop}, State={status.State}");
                            }
                            break;

                        case uiTxCommand.uiRxLsrIntStatus:
                            if (data.Length >= 1)
                            {
                                IntStatusUpdated?.Invoke(this, data[0]);
                                var status = new InterlockStatus
                                {
                                    ArmOK = (data[0] & 0x01) != 0,
                                    RunOK = (data[0] & 0x02) != 0,
                                    FlowOK = (data[0] & 0x04) != 0,
                                    TempOK = (data[0] & 0x08) != 0,
                                    DoorOK = (data[0] & 0x10) != 0,
                                    CoverOK = (data[0] & 0x20) != 0,
                                    DischargeTempOK = (data[0] & 0x40) != 0,
                                    OverVoltageOK = (data[0] & 0x80) != 0
                                };
                                InterlockStatusUpdated?.Invoke(this, status);
                                Log($"Received interlock status: 0x{data[0]:X2}");
                            }
                            break;

                        case uiTxCommand.uiRxLsrIntMask:
                            if (data.Length >= 1)
                            {
                                IntMaskUpdated?.Invoke(this, data[0]);
                                Log($"Received interlock mask: 0x{data[0]:X2}");
                            }
                            break;

                        case uiTxCommand.uiRxLsrWaveform:
                            if (data.Length >= 64)
                            {
                                ushort[] samples = new ushort[32];
                                for (int i = 0; i < 32; i++)
                                {
                                    samples[i] = FromBigEndianUshort(data, i * 2);
                                }
                                var waveform = new WaveformData
                                {
                                    Waveform = samples
                                };
                                WaveformUpdated?.Invoke(this, waveform);
                                Log($"Received waveform: {waveform.Waveform.Length} samples");
                            }
                            break;

                        case uiTxCommand.uiRxLsrVolts:
                            if (data.Length >= 2)
                            {
                                ushort volts = FromBigEndianUshort(data, 0);
                                VoltsUpdated?.Invoke(this, volts);
                                Log($"Received voltage: {volts}V");
                            }
                            break;

                        case uiTxCommand.uiRxLsrChargeState:
                            if (data.Length >= 3)
                            {
                                var chargeState = new ChargeStateData
                                {
                                    MeasuredVoltage = FromBigEndianUshort(data, 0),
                                    IsCharging = data[2] != 0
                                };
                                ChargeStateUpdated?.Invoke(this, chargeState);
                                Log($"Received charge state: Voltage={chargeState.MeasuredVoltage}V, IsCharging={chargeState.IsCharging}");
                            }
                            break;

                        case uiTxCommand.uiRxLsrChargeVolts:
                            if (data.Length >= 2)
                            {
                                ushort volts = FromBigEndianUshort(data, 0);
                                ChargeVoltsUpdated?.Invoke(this, volts);
                                Log($"Received charge volts: {volts}V");
                            }
                            break;

                        case uiTxCommand.uiRxShutterConfig:
                            if (data.Length >= 2)
                            {
                                var config = new ShutterConfig
                                {
                                    ShutterMode = (ShutterModeType)data[0],
                                    ShutterState = (ShutterStateType)data[1]
                                };
                                if (!Enum.IsDefined(typeof(ShutterModeType), config.ShutterMode) || !Enum.IsDefined(typeof(ShutterStateType), config.ShutterState))
                                {
                                    Log($"Invalid shutter config received: Mode={data[0]}, State={data[1]}");
                                    SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { commandByte });
                                }
                                else
                                {
                                    ShutterConfigUpdated?.Invoke(this, config);
                                    Log($"Received shutter config: Mode={config.ShutterMode}, State={config.ShutterState}");
                                }
                            }
                            else
                            {
                                Log($"Invalid shutter config data length: {data.Length}");
                                SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { commandByte });
                            }
                            break;

                        case uiTxCommand.uiRxSoftStartConfig:
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

                        case uiTxCommand.uiRxLsrCal:
                            if (data.Length >= 2)
                            {
                                var calibration = new CalibrationData
                                {
                                    CapVoltRange = FromBigEndianUshort(data, 0)
                                };
                                CalibrationUpdated?.Invoke(this, calibration);
                                DeviceDataUpdated?.Invoke(this, new DeviceData { Calibration = calibration });
                                Log($"Received calibration: CapVoltRange={calibration.CapVoltRange}V");
                            }
                            break;

                        case uiTxCommand.uiRxLsrDelays:
                            if (data.Length >= 4)
                            {
                                ushort delay1 = FromBigEndianUshort(data, 0);
                                ushort delay2 = FromBigEndianUshort(data, 2);
                                Log($"Received delays: Delay1={delay1}us, Delay2={delay2}us");
                            }
                            break;

                        case uiTxCommand.uiRxDigitalIO:
                            Log("Received digital IO data");
                            break;

                        default:
                            Log($"Unhandled command: {rxCommand} (0x{commandByte:X2})");
                            SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { commandByte });
                            break;
                    }
                }
                else
                {
                    Log($"Invalid command byte: 0x{commandByte:X2}");
                    SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { commandByte });
                }
            }
            catch (Exception ex)
            {
                Log($"Error processing packet: {ex.Message}");
                CommandError?.Invoke(this, ("ProcessPacket", ex));
                SendCommand(uiRxCommand.uiTxLsrBadCmd, new byte[] { _currentCommand });
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

        public async Task SendCommandAsync(uiRxCommand command, byte[] data)
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

        public void SendCommand(uiRxCommand command, byte[] data)
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

        public async Task SendCommandAsync(uiTxCommand command, byte[] data)
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
            if (data == null || startIndex < 0 || startIndex + 1 >= data.Length)
                return 0;
            return (ushort)((data[startIndex] << 8) | data[startIndex + 1]);
        }

        private uint FromBigEndianUint(byte[] data, int startIndex = 0)
        {
            if (data == null || startIndex < 0 || startIndex + 3 >= data.Length)
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
        public const int DISCOVERY = 1;
        public const int LSR_VOLTS = 2;
        public const int CHARGE_STATE = 3;
        public const int CHARGE_VOLT = 2;
        public const int SHUTTER_CONFIG = 2;
        public const int SOFT_START_CONFIG = 7;
        public const int LSR_CAL = 2;
        public const int LSR_DELAYS = 4;
        public const int DIGITAL_IO = 8;

        public static int GetCommandLength(uiTxCommand command)
        {
            return command switch
            {
                uiTxCommand.uiRxLsrPulseConfig => LSR_PULSE_CONFIG,
                uiTxCommand.uiRxLsrState => LSR_STATE,
                uiTxCommand.uiRxLsrCount => LSR_SHOTS,
                uiTxCommand.uiRxLsrRunStatus => RUN_STATUS,
                uiTxCommand.uiRxLsrIntStatus => INT_STATUS,
                uiTxCommand.uiRxLsrIntMask => INT_MASK,
                uiTxCommand.uiRxLsrWaveform => WAVEFORM,
                uiTxCommand.uiRxLsrVolts => LSR_VOLTS,
                uiTxCommand.uiRxLsrChargeState => CHARGE_STATE,
                uiTxCommand.uiRxLsrChargeVolts => CHARGE_VOLT,
                uiTxCommand.uiRxShutterConfig => SHUTTER_CONFIG,
                uiTxCommand.uiRxSoftStartConfig => SOFT_START_CONFIG,
                uiTxCommand.uiRxLsrCal => LSR_CAL,
                uiTxCommand.uiRxLsrDelays => LSR_DELAYS,
                uiTxCommand.uiRxDigitalIO => DIGITAL_IO,
                _ => -1
            };
        }
    }
}
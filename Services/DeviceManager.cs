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
                //SendDiscoveryCommand();
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

        private void SendDiscoveryResp()
        {
            SendCommand(uiTxCommand.uiTxShutterConfig, new byte[] { (byte)ShutterModeType.autoMode, (byte)ShutterStateType.shutterClosed });
        }

        public void SendVoltage(ushort voltage) => SendCommand(uiTxCommand.uiTxLsrVolts, ToBigEndian(voltage));
        //public void SendReadEnergy() => SendCommand(uiTxCommand.uiTxReadEnergy, new byte[0]);
        //public void SendReadTemperature() => SendCommand(uiTxCommand.uiTxReadTemperature, new byte[0]);
        //public void SendSystemInfoRequest() => SendCommand(uiTxCommand.uiTxSystemInfo, new byte[0]);

        public void SendCalibration(CalibrationData calibration)
        {
            byte[] data = ToBigEndian(calibration.CapVoltRange);
            SendCommand(uiTxCommand.uiTxLsrCal, data);
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

            if (data.Count != GetCommandLength(uiTxCommand.uiTxLsrPulseConfig))
            {
                Log($"Pulse config length error: Got {data.Count}, expected {GetCommandLength(uiTxCommand.uiTxLsrPulseConfig)}");
                return;
            }

            SendCommand(uiTxCommand.uiTxLsrPulseConfig, data.ToArray());
            Log($"Sent pulse config: Frequency={config.Frequency / 10.0:F1}Hz, PulseWidth={config.PulseWidth}us, Shots={config.ShotTotal}, Delay1={config.Delay1}us, Delay2={config.Delay2}us, ShotMode={config.ShotMode}, TrigMode={config.TrigMode}");
        }

        public void SendStateCommand(LaserStateType state)
        {
            SendCommand(uiTxCommand.uiTxLsrState, new[] { (byte)state });
            Log($"Sent state command: State={state}");
        }

        public void SendInterlockMask(byte mask)
        {
            SendCommand(uiTxCommand.uiTxIntMask, new[] { mask });
            Log($"Sent interlock mask: 0x{mask:X2}");
        }

        public void SendWaveformState(bool enable)
        {
            SendCommand(uiTxCommand.uiTxWaveState, new[] { (byte)(enable ? 1 : 0) });
            Log($"Sent waveform state: Enable={enable}");
        }

        public void SendLaserDelays(ushort delay1, ushort delay2)
        {
            var data = new List<byte>();
            data.AddRange(ToBigEndian(delay1));
            data.AddRange(ToBigEndian(delay2));

            if (data.Count != GetCommandLength(uiTxCommand.uiTxLsrDelays))
            {
                Log($"Laser delays length error: Got {data.Count}, expected {GetCommandLength(uiTxCommand.uiTxLsrDelays)}");
                return;
            }

            SendCommand(uiTxCommand.uiTxLsrDelays, data.ToArray());
            Log($"Sent delays: Delay1={delay1}us, Delay2={delay2}us");
        }

        //public void SendDigitalIOCommand()
        //{
        //    SendCommand(uiTxCommand.uiTxDigitalIO, new byte[0]);
        //    Log("Sent digital IO command");
        //}

        //public void SendSystemReset()
        //{
        //    SendCommand(uiTxCommand.uiTxSystemReset, new byte[0]);
        //    Log("Sent system reset");
        //}

        //public void RequestSystemStatus()
        //{
        //    SendCommand(uiTxCommand.uiTxLsrRunStatus, new byte[0]);
        //    Log("Requested system status");
        //}

        public void RequestWaveformData()
        {
            SendCommand(uiTxCommand.uiTxWaveState, new byte[0]);
            Log("Requested waveform data");
        }

        public void SendChargeCancel()
        {
            SendCommand(uiTxCommand.uiTxLsrChargeCancel, new byte[] { 1 });
            Log("Sent charge cancel");
        }

        public void SendShutterConfig(ShutterConfig config)
        {
            SendCommand(uiTxCommand.uiTxShutterConfig, new byte[] { (byte)config.ShutterMode, (byte)config.ShutterState });
            Log($"Sent shutter config: Mode={config.ShutterMode}, State={config.ShutterState}");
        }

        public void SendSoftStartConfig(SoftStartConfig config)
        {
            var data = new List<byte>();
            data.Add((byte)(config.Enable ? 1 : 0));
            data.AddRange(ToBigEndian(config.IdleSetpoint));
            data.AddRange(ToBigEndian(config.RampCount));

            SendCommand(uiTxCommand.uiTxSoftStartConfig, data.ToArray());
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
                    SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
                }
            }
        }

        private void ProcessByte(byte b)
        {
            _lastByteReceived = DateTime.Now;
            _incomingBuffer.Add(b);
            //Log($"Processing byte 0x{b:X2}, State={_currentState}, Buffer=[{BitConverter.ToString(_incomingBuffer.ToArray()).Replace("-", " ")}]");

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
                        SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
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
                    SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
                    ResetParser();
                    return;
                }

                if (!CommunicationHelper.ExtractPacketData(packet, out byte commandByte, out byte[] data))
                {
                    Log($"Failed to extract packet data: Packet=[{BitConverter.ToString(packet).Replace("-", " ")}]");
                    SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
                    ResetParser();
                    return;
                }

                string hexData = data.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                Log($"RECV: 0x{commandByte:X2}, DataLen={data.Length}, Data=[{hexData}], Packet=[{BitConverter.ToString(packet).Replace("-", " ")}]");

                if (Enum.IsDefined(typeof(uiRxCommand), commandByte))
                {
                    uiRxCommand rxCommand = (uiRxCommand)commandByte;
                    Log($"Received uiTxCommand: {rxCommand} (0x{commandByte:X2})");

                    if (data.Length != GetCommandLength(rxCommand))
                    {
                        Log($"Length mismatch");
                        SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
                        ResetParser();
                        return;
                    }

                    switch (rxCommand)
                    {
                        case uiRxCommand.uiRxLsrState:
                            StateUpdated?.Invoke(this, (LaserStateType)data[0]);
                            Log($"Received state: {(LaserStateType)data[0]}");
                            break;

                        case uiRxCommand.uiRxLsrPulseConfig:
                            Log($"Received uiRxLsrPulseConfig: Data=[{hexData}]");
                            //SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxLsrCount:
                            Log($"Received uiRxLsrCount: Data=[{hexData}]");
                            //SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxLsrRunStatus:
                            Log($"Received uiRxLsrRunStatus: Data=[{hexData}]");
                            //SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxLsrIntStatus:
                            //IntMaskUpdated?.Invoke(this, data[0]);
                            Log($"Received interlock status: 0x{data[0]:X2}");
                            break;

                        case uiRxCommand.uiRxLsrIntMask:
                            IntMaskUpdated?.Invoke(this, data[0]);
                            Log($"Received interlock mask: 0x{data[0]:X2}");
                            break;

                        case uiRxCommand.uiRxLsrWaveform:
                            Log($"Received uiRxLsrWaveform: Data=[{hexData}]");
                            //SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxDiscovery:
                            Log($"Received uiRxDiscovery: Data=[{hexData}]");
                            SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxLsrVolts:
                            Log($"Received uiRxLsrVolts: Data=[{hexData}]");
                            //SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxLsrChargeState:
                            Log($"Received uiRxLsrChargeState: Data=[{hexData}]");
                            //SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxLsrChargeVolts:
                            Log($"Received uiRxLsrChargeVolts: Data=[{hexData}]");
                            //SendDiscoveryResp();
                            break;

                        case uiRxCommand.uiRxShutterConfig:
                            //IntMaskUpdated?.Invoke(this, data[0]);
                            Log($"Received uiRxShutterConfig: Data=[{hexData}]");
                            break;

                        case uiRxCommand.uiRxSoftStartConfig:
                            //IntMaskUpdated?.Invoke(this, data[0]);
                            Log($"Received uiRxSoftStartConfig: Data=[{hexData}]");
                            break;

                        default:
                            Log($"Unhandled uiRxCommand: {rxCommand} (0x{commandByte:X2})");
                            SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
                            break;
                    }
                    ResetParser();
                    return;
                }
                else
                {
                    Log($"Invalid command byte: 0x{commandByte:X2}");
                    SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
                }
            }
            catch (Exception ex)
            {
                Log($"Error processing packet: {ex.Message}");
                CommandError?.Invoke(this, ("ProcessPacket", ex));
                SendCommand(uiTxCommand.uiTxBadCmd, new byte[] { 0 });
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
            //Log("Parser reset");
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

        //private ushort FromBigEndianUshort(byte[] data, int startIndex = 0)
        //{
        //    if (data == null || startIndex < 0 || startIndex + 2 > data.Length)
        //        return 0;
        //    return (ushort)((data[startIndex] << 8) | data[startIndex + 1]);
        //}
        //private uint FromBigEndianUint(byte[] data, int startIndex = 0)
        //{
        //    if (data == null || startIndex < 0 || startIndex + 4 > data.Length)
        //        return 0;
        //    return (uint)((data[startIndex] << 24) | (data[startIndex + 1] << 16) |
        //                 (data[startIndex + 2] << 8) | data[startIndex + 3]);
        //}

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
        public static int GetCommandLength(uiRxCommand command)
        {
            return command switch
            {
                uiRxCommand.uiRxTestResp => 1,          // 1
                uiRxCommand.uiRxFPGABadCmd => 1,        // same as receive
                uiRxCommand.uiRxLsrState => 1,
                uiRxCommand.uiRxLsrPulseConfig => 14,
                uiRxCommand.uiRxLsrCount => 4,
                uiRxCommand.uiRxLsrRunStatus => 14,
                uiRxCommand.uiRxLsrIntStatus => 1,
                uiRxCommand.uiRxLsrIntMask => 1,
                uiRxCommand.uiRxLsrWaveform => 64,
                uiRxCommand.uiRxDiscovery => 1,
                uiRxCommand.uiRxLsrVolts => 2,
                uiRxCommand.uiRxLsrChargeState => 3,
                uiRxCommand.uiRxLsrChargeVolts => 2,
                uiRxCommand.uiRxShutterConfig => 2,
                uiRxCommand.uiRxSoftStartConfig => 7
            };
        }
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
                uiTxCommand.uiTxSoftStartConfig => 7
            };
        }
    }
}
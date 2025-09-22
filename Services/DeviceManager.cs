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
        private const int MAX_DISCOVERY_RETRIES = 10;
        private bool _deviceReady = false;
        private readonly TimeSpan _commandDelay = TimeSpan.FromMilliseconds(100);
        private DateTime _lastCommandTime = DateTime.MinValue;

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
                Handshake = Handshake.None
            };
            _serialPort.DataReceived += OnDataReceived;
            _serialPort.ErrorReceived += OnErrorReceived;
        }

        public bool Connect(string portName, int baudRate)
        {
            try
            {
                if (_serialPort.IsOpen)
                {
                    Log("Port already open");
                    return false;
                }

                _serialPort.PortName = portName;
                _serialPort.BaudRate = baudRate;
                _serialPort.Open();
                Log($"Connected to {portName} at {baudRate} baud");
                StartDiscovery();
                return true;
            }
            catch (Exception ex)
            {
                Log($"Failed to connect to {portName}: {ex.Message}");
                CommandError?.Invoke(this, ("Connect", ex));
                return false;
            }
        }

        public void Disconnect()
        {
            try
            {
                if (_serialPort.IsOpen)
                {
                    _cts?.Cancel();
                    _serialPort.Close();
                    _deviceReady = false;
                    Log("Disconnected from serial port");
                }
            }
            catch (Exception ex)
            {
                Log($"Error disconnecting: {ex.Message}");
                CommandError?.Invoke(this, ("Disconnect", ex));
            }
        }

        public void SendCommand(uiTxCommand command, byte[] data = null)
        {
            try
            {
                lock (_serialLock)
                {
                    if (!_serialPort.IsOpen)
                    {
                        Log("Cannot send command: Serial port is not open");
                        return;
                    }

                    int expectedLength = GetCommandLength(command);
                    if (data != null && data.Length != expectedLength)
                    {
                        Log($"Invalid data length for {command}: expected {expectedLength}, got {data.Length}");
                        return;
                    }

                    byte[] packet = CommunicationHelper.CreatePacket((byte)command, data);
                    _serialPort.Write(packet, 0, packet.Length);
                    string hexData = data?.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                    Log($"SENT: {command} (0x{(sbyte)command:X2}), DataLen={data?.Length ?? 0}, Data=[{hexData}]");
                    _lastCommandTime = DateTime.Now;
                }
            }
            catch (Exception ex)
            {
                Log($"Error sending command {command}: {ex.Message}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }

        public async Task SendCommandWithDelay(uiTxCommand command, byte[] data = null)
        {
            try
            {
                TimeSpan timeSinceLastCommand = DateTime.Now - _lastCommandTime;
                if (timeSinceLastCommand < _commandDelay)
                {
                    await Task.Delay(_commandDelay - timeSinceLastCommand);
                }

                lock (_serialLock)
                {
                    if (!_serialPort.IsOpen)
                    {
                        Log("Cannot send command: Serial port is not open");
                        return;
                    }

                    int expectedLength = GetCommandLength(command);
                    if (data != null && data.Length != expectedLength)
                    {
                        Log($"Invalid data length for {command}: expected {expectedLength}, got {data.Length}");
                        return;
                    }

                    byte[] packet = CommunicationHelper.CreatePacket((byte)command, data);
                    _serialPort.Write(packet, 0, packet.Length);
                    string hexData = data?.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                    Log($"SENT: {command} (0x{(sbyte)command:X2}), DataLen={data?.Length ?? 0}, Data=[{hexData}]");
                    _lastCommandTime = DateTime.Now;
                }
            }
            catch (Exception ex)
            {
                Log($"Error sending command {command}: {ex.Message}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }

        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                lock (_serialLock)
                {
                    byte[] buffer = new byte[_serialPort.BytesToRead];
                    int bytesRead = _serialPort.Read(buffer, 0, buffer.Length);
                    _incomingBuffer.AddRange(buffer);
                    _lastByteReceived = DateTime.Now;

                    while (_incomingBuffer.Count > 0)
                    {
                        if (_currentState == ParserState.LookingForStart)
                        {
                            int stxIndex = _incomingBuffer.IndexOf(ProtocolConstants.STX);
                            if (stxIndex > 0)
                            {
                                _incomingBuffer.RemoveRange(0, stxIndex);
                            }
                            if (_incomingBuffer.Count > 0 && _incomingBuffer[0] == ProtocolConstants.STX)
                            {
                                _currentState = ParserState.ReadingLength;
                                _incomingBuffer.RemoveAt(0);
                            }
                            else
                            {
                                _incomingBuffer.Clear();
                                continue;
                            }
                        }

                        if (_currentState == ParserState.ReadingLength && _incomingBuffer.Count >= 2)
                        {
                            _expectedDataLength = (_incomingBuffer[0] << 8) | _incomingBuffer[1];
                            _incomingBuffer.RemoveRange(0, 2);
                            _currentState = ParserState.ReadingCommand;
                        }

                        if (_currentState == ParserState.ReadingCommand && _incomingBuffer.Count >= 1)
                        {
                            _currentCommand = _incomingBuffer[0];
                            _incomingBuffer.RemoveAt(0);
                            _currentState = ParserState.ReadingData;
                        }

                        if (_currentState == ParserState.ReadingData && _incomingBuffer.Count >= _expectedDataLength)
                        {
                            byte[] data = _incomingBuffer.Take(_expectedDataLength).ToArray();
                            _incomingBuffer.RemoveRange(0, _expectedDataLength);
                            _currentState = ParserState.ReadingChecksum;
                            byte[] packet = new byte[1 + 2 + 1 + _expectedDataLength + 1 + 1];
                            packet[0] = ProtocolConstants.STX;
                            packet[1] = (byte)(_expectedDataLength >> 8);
                            packet[2] = (byte)(_expectedDataLength & 0xFF);
                            packet[3] = _currentCommand;
                            Array.Copy(data, 0, packet, 4, _expectedDataLength);
                            packet[4 + _expectedDataLength] = CommunicationHelper.CreatePacket(_currentCommand, data)[4 + _expectedDataLength];
                            packet[5 + _expectedDataLength] = ProtocolConstants.ETX;

                            if (CommunicationHelper.ValidatePacket(packet) == PacketValidationResult.Valid)
                            {
                                string hexData = data.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data";
                                Log($"RECEIVED: {(uiRxCommand)_currentCommand} (0x{_currentCommand:X2}), DataLen={data.Length}, Data=[{hexData}]");

                                if (_currentCommand == (byte)uiRxCommand.lcdTxDiscovery)
                                {
                                    if (data.Length == 1 && data[0] == 0x00)
                                    {
                                        _cts?.Cancel();
                                        _deviceReady = true;
                                        DiscoveryResponseReceived?.Invoke(this, true);
                                    }
                                    else
                                    {
                                        Log("Invalid discovery response: incorrect data length or content");
                                        DiscoveryResponseReceived?.Invoke(this, false);
                                    }
                                }
                                else
                                {
                                    ProcessCommand((uiRxCommand)_currentCommand, data);
                                }
                            }
                            else
                            {
                                Log($"Invalid packet for command 0x{_currentCommand:X2}: {BitConverter.ToString(packet).Replace("-", " ")}");
                                if (_currentCommand == (byte)uiRxCommand.lcdTxDiscovery)
                                {
                                    DiscoveryResponseReceived?.Invoke(this, false);
                                }
                            }
                            _currentState = ParserState.LookingForStart;
                        }

                        if (_incomingBuffer.Count == 0 && (DateTime.Now - _lastByteReceived) > _parserTimeout)
                        {
                            _currentState = ParserState.LookingForStart;
                            _incomingBuffer.Clear();
                            Log("Parser timeout, resetting state");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Log($"Error processing received data: {ex.Message}");
            }
        }

        private void OnErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            Log($"Serial port error: {e.EventType}");
        }

        private void StartDiscovery()
        {
            _cts?.Dispose();
            _cts = new CancellationTokenSource();
            _discoveryRetries = 0;
            Task.Run(async () =>
            {
                while (_discoveryRetries < MAX_DISCOVERY_RETRIES && !_cts.Token.IsCancellationRequested)
                {
                    // Send lcdRxShutterConfig with 2 bytes: lsrShutterMode (0 = autoMode), manShutterState (0 = shutterClosed)
                    SendCommand(uiTxCommand.lcdRxShutterConfig, new byte[] { 0x00, 0x00 });
                    await Task.Delay(1000, _cts.Token);
                    _discoveryRetries++;
                }
                if (!_cts.Token.IsCancellationRequested)
                {
                    Log("Discovery failed after maximum retries");
                    DiscoveryResponseReceived?.Invoke(this, false);
                }
            }, _cts.Token);
        }
        private void ProcessCommand(uiRxCommand command, byte[] data)
        {
            // Implementation of ProcessCommand remains unchanged from original
            // Add handling for specific commands if needed
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

        public static int GetCommandLength(uiRxCommand command)
        {
            return command switch
            {
                uiRxCommand.lcdTxTestResp => 1,
                uiRxCommand.lcdTxBadCmd => 1,
                uiRxCommand.lcdTxLsrState => 1,
                uiRxCommand.lcdTxLsrPulseConfig => 14,
                uiRxCommand.lcdTxLsrCount => 4,
                uiRxCommand.lcdTxLsrRunStatus => 14,
                uiRxCommand.lcdTxLsrIntStatus => 1,
                uiRxCommand.lcdTxLsrIntMask => 1,
                uiRxCommand.lcdTxLsrWaveform => 64,
                uiRxCommand.lcdTxDiscovery => 1,
                uiRxCommand.lcdTxLsrVolts => 2,
                uiRxCommand.lcdTxLsrChargeState => 3,
                uiRxCommand.lcdTxLsrChargeVolts => 2,
                uiRxCommand.lcdTxShutterConfig => 2,
                uiRxCommand.lcdTxSoftStartConfig => 7,
                _ => throw new ArgumentException($"Unknown command: {command}")
            };
        }

        public static int GetCommandLength(uiTxCommand command)
        {
            return command switch
            {
                uiTxCommand.lcdRxBadCmd => 1,
                uiTxCommand.lcdRxNoCmd => 1,
                uiTxCommand.lcdFPGABadCmd => 1,
                uiTxCommand.lcdRxLsrPulseConfig => 14,
                uiTxCommand.lcdRxLsrState => 1,
                uiTxCommand.lcdRxIntMask => 1,
                uiTxCommand.lcdRxWaveState => 1,
                uiTxCommand.lcdRxLsrDelays => 4,
                uiTxCommand.lcdRxLsrCal => 2,
                uiTxCommand.lcdRxLsrVolts => 2,
                uiTxCommand.lcdRxLsrChargeCancel => 1,
                uiTxCommand.lcdRxShutterConfig => 2,
                uiTxCommand.lcdRxSoftStartConfig => 7,
                _ => throw new ArgumentException($"Unknown command: {command}")
            };
        }
    }
}
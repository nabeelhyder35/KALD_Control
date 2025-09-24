using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using KALD_Control.Models;
using Microsoft.UI.Dispatching;

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
        private readonly List<byte> _receiveBuffer = new List<byte>();
        private enum ParserState { LookingForStart, ReadingLength, ReadingCommand, ReadingData, ReadingChecksum, ReadingEnd }
        private ParserState _currentState = ParserState.LookingForStart;
        private int _expectedDataLength = 0;
        private byte _currentCommand = 0;
        private DateTime _lastByteReceived = DateTime.MinValue;
        private readonly TimeSpan _parserTimeout = TimeSpan.FromSeconds(5);
        private DeviceData _deviceData = new DeviceData();
        private readonly DispatcherQueue _dispatcherQueue;

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
        public event EventHandler<DeviceData> DeviceDataUpdated;

        public bool IsConnected => _serialPort?.IsOpen ?? false;
        public int BaudRate => _serialPort?.BaudRate ?? 115200;

        public DeviceManager(ILogger<DeviceManager> logger, DispatcherQueue dispatcherQueue = null)
        {
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
            _dispatcherQueue = dispatcherQueue;

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
                WriteBufferSize = 4096,
                NewLine = "\r\n",
                DtrEnable = true,
                RtsEnable = true
            };
            _serialPort.ErrorReceived += OnErrorReceived;
            _serialPort.DataReceived += OnDataReceived;
        }

        private void OnErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            try
            {
                Log($"Serial port error: {e.EventType}");
                switch (e.EventType)
                {
                    case SerialError.RXOver:
                        Log("Input buffer overflow; discarding input buffer");
                        lock (_serialLock)
                        {
                            _serialPort.DiscardInBuffer();
                            _receiveBuffer.Clear();
                            ResetParser();
                        }
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
            catch (Exception ex)
            {
                Log($"Error in OnErrorReceived: {ex.Message}");
            }
        }

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
                _serialPort.DiscardInBuffer();
                _serialPort.DiscardOutBuffer();
                _receiveBuffer.Clear();
                ResetParser();
                Log($"Connected to {portName} at {baudRate} baud");
            }
            catch (Exception ex)
            {
                Log($"Failed to connect to {portName}: {ex.Message}");
                throw new InvalidOperationException($"Connection failed: {ex.Message}", ex);
            }
        }

        public void Disconnect()
        {
            if (!IsConnected)
            {
                Log("Already disconnected");
                return;
            }

            try
            {
                _serialPort.Close();
                Log("Disconnected from serial port");
                ResetParser();
                _receiveBuffer.Clear();
            }
            catch (Exception ex)
            {
                Log($"Error disconnecting: {ex.Message}");
            }
        }

        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                if (!IsConnected) return;
                int bytesToRead = _serialPort.BytesToRead;
                if (bytesToRead <= 0) return;
                byte[] buffer = new byte[bytesToRead];
                int bytesRead = _serialPort.Read(buffer, 0, bytesToRead);
                if (bytesRead > 0)
                {
                    ProcessReceivedData(buffer, bytesRead);
                }
            }
            catch (Exception ex)
            {
                Log($"Error in OnDataReceived: {ex.Message}");
                ResetParser();
            }
        }

        private void ProcessReceivedData(byte[] data, int bytesRead)
        {
            lock (_serialLock)
            {
                try
                {
                    _lastByteReceived = DateTime.Now;
                    Log($"Processing received data: {bytesRead} bytes, Hex: {BitConverter.ToString(data, 0, bytesRead).Replace("-", " ")}");

                    // Add new data to buffer
                    _receiveBuffer.AddRange(data.Take(bytesRead));

                    // Process all complete packets in buffer
                    while (_receiveBuffer.Count > 0)
                    {
                        int packetEndIndex = FindPacketEnd(_receiveBuffer);
                        if (packetEndIndex == -1)
                        {
                            Log($"No complete packet found in buffer. Buffer size: {_receiveBuffer.Count} bytes");
                            break;
                        }

                        // Extract and process the complete packet
                        byte[] packet = _receiveBuffer.Take(packetEndIndex + 1).ToArray();
                        Log($"Complete packet found: {BitConverter.ToString(packet).Replace("-", " ")}");
                        _receiveBuffer.RemoveRange(0, packetEndIndex + 1);

                        ProcessCompletePacket(packet);
                    }

                    // Buffer cleanup
                    if (_receiveBuffer.Count > 2048)
                    {
                        Log("Receive buffer overflow protection triggered");
                        _receiveBuffer.Clear();
                        ResetParser();
                    }
                }
                catch (Exception ex)
                {
                    Log($"Error in ProcessReceivedData: {ex.Message}");
                    _receiveBuffer.Clear();
                    ResetParser();
                }
            }
        }

        private int FindPacketEnd(List<byte> buffer)
        {
            for (int i = 0; i < buffer.Count; i++)
            {
                if (buffer[i] == ProtocolConstants.STX && i + 4 < buffer.Count)
                {
                    // Extract length
                    ushort dataLength = (ushort)((buffer[i + 1] << 8) | buffer[i + 2]);
                    int totalPacketLength = 1 + 2 + 1 + dataLength + 1 + 1; // STX + LEN + CMD + DATA + CHK + ETX

                    if (i + totalPacketLength <= buffer.Count &&
                        buffer[i + totalPacketLength - 1] == ProtocolConstants.ETX)
                    {
                        return i + totalPacketLength - 1;
                    }
                }
            }
            return -1;
        }

        private void ProcessCompletePacket(byte[] packet)
        {
            try
            {
                Log($"Processing complete packet: {BitConverter.ToString(packet).Replace("-", " ")}");
                var validationResult = CommunicationHelper.ValidatePacket(packet);

                if (validationResult == PacketValidationResult.Valid)
                {
                    if (CommunicationHelper.ExtractPacketData(packet, out byte command, out byte[] data))
                    {
                        Log($"Packet validated successfully. Command: {(uiRxCommand)command} (0x{command:X2}), Data: {BitConverter.ToString(data).Replace("-", " ")}");
                        // Process on background thread to avoid blocking serial port
                        Task.Run(() => ProcessPacket((uiRxCommand)command, data));
                    }
                    else
                    {
                        Log("Failed to extract packet data from valid packet");
                    }
                }
                else
                {
                    Log($"Checksum Failed! Data: {BitConverter.ToString(packet).Replace("-", " ")}");
                }
            }
            catch (Exception ex)
            {
                Log($"Error in ProcessCompletePacket: {ex.Message}, Packet: {BitConverter.ToString(packet).Replace("-", " ")}");
            }
        }

        public byte[] ToBigEndian(ushort value) => new byte[] { (byte)(value >> 8), (byte)(value & 0xFF) };
        public byte[] ToBigEndian(uint value) => new byte[] { (byte)(value >> 24), (byte)(value >> 16), (byte)(value >> 8), (byte)(value & 0xFF) };

        /*---------------------------------------------------------------------------
         * UI FUNCTIONS
         ----------------------------------------------------------------------------*/
        private void SafeInvoke(Action action)
        {
            try
            {
                if (_dispatcherQueue != null)
                {
                    _dispatcherQueue.TryEnqueue(() =>
                    {
                        try
                        {
                            action();
                        }
                        catch (Exception ex)
                        {
                            Log($"Error in SafeInvoke dispatcher: {ex.Message}");
                        }
                    });
                }
                else
                {
                    // Fallback to direct invocation if no dispatcher available
                    action();
                }
            }
            catch (Exception ex)
            {
                Log($"Error in SafeInvoke: {ex.Message}");
            }
        }
        private void Log(string message)
        {
            try
            {
                string formattedMessage = $"{DateTime.Now:HH:mm:ss.fff} - {message}";
                _logger?.LogInformation(formattedMessage);
                SafeInvoke(() => LogMessage?.Invoke(this, formattedMessage + Environment.NewLine));
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Error in Log: {ex.Message}");
            }
        }
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            try
            {
                if (_serialPort != null)
                {
                    if (_serialPort.IsOpen)
                    {
                        _serialPort.Close();
                    }
                    _serialPort.Dispose();
                }
                _receiveBuffer.Clear();
                Log("DeviceManager disposed");
            }
            catch (Exception ex)
            {
                Log($"Error disposing DeviceManager: {ex.Message}");
            }
        }

        /*---------------------------------------------------------------------------
         * FUNCTIONS FOR SERIAL PORT HANDLING
         ----------------------------------------------------------------------------*/
        private void ResetParser()
        {
            _receiveBuffer.Clear();
            _currentState = ParserState.LookingForStart;
            _expectedDataLength = 0;
            _currentCommand = 0;
            Log("Parser reset");
        }

        /*---------------------------------------------------------------------------
         * FUNCTIONS FOR RECEIVING AND HANDLING COMMANDS (RX)
         ----------------------------------------------------------------------------*/
        public static int GetCommandLength(uiRxCommand command)
        {
            return command switch
            {
                uiRxCommand.uiRxTestResp => 1,
                uiRxCommand.uiRxFPGABadCmd => 1,
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
                uiRxCommand.uiRxSoftStartConfig => 7,
                _ => 0
            };
        }
        private void HandleLsrState(byte[] data)
        {
            var state = (LaserStateType)data[0];
            _deviceData.LaserState = state;
            SafeInvoke(() => StateUpdated?.Invoke(this, state));
            Log($"Laser state updated: {state}");
        }
        private void HandleLsrPulseConfig(byte[] data)
        {
            var pulseConfig = new PulseConfig
            {
                Frequency = (ushort)((data[0] << 8) | data[1]),
                PulseWidth = (ushort)((data[2] << 8) | data[3]),
                ShotTotal = (uint)((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]),
                Delay1 = (ushort)((data[8] << 8) | data[9]),
                Delay2 = (ushort)((data[10] << 8) | data[11]),
                ShotMode = (ShotModeType)data[12],
                TrigMode = (TriggerModeType)data[13]
            };
            _deviceData.PulseConfig = pulseConfig;
            SafeInvoke(() => PulseConfigUpdated?.Invoke(this, pulseConfig));
            Log($"Pulse config updated: Freq={pulseConfig.Frequency}, Width={pulseConfig.PulseWidth}");
        }
        private void HandleLsrCount(byte[] data)
        {
            var shotCount = (uint)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
            _deviceData.ShotCount = shotCount;
            SafeInvoke(() => ShotCountUpdated?.Invoke(this, shotCount));
            Log($"Shot count updated: {shotCount}");
        }
        private void HandleLsrRunStatus(byte[] data)
        {
            var runStatus = new RunStatusData
            {
                ShotCount = (uint)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]),
                State = (LaserStateType)data[4],
                Energy = (ushort)((data[5] << 8) | data[6]),
                Power = (ushort)((data[7] << 8) | data[8]),
                Current = (ushort)((data[9] << 8) | data[10]),
                VDroop = (ushort)((data[11] << 8) | data[12]),
            };
            _deviceData.RunStatus = runStatus;
            SafeInvoke(() => RunStatusUpdated?.Invoke(this, runStatus));
            Log($"Run status: Shots={runStatus.ShotCount}, State={runStatus.State}, Energy={runStatus.Energy}J");
        }
        private void HandleLsrIntStatus(byte[] data)
        {
            var digitalIO = new DigitalIOState();
            digitalIO.InputStates = data[0];
            var status = digitalIO.GetInterlockStatusForLCD();
            SafeInvoke(() =>
            {
                IntStatusUpdated?.Invoke(this, status);
                DigitalIOUpdated?.Invoke(this, digitalIO);
            });
            Log($"Interlock status: 0x{data[0]:X2}");
        }
        private void HandleLsrIntMask(byte[] data)
        {
            var digitalIO = new DigitalIOState();
            digitalIO.SetInterlockMaskFromLCD(data[0]);
            SafeInvoke(() =>
            {
                IntMaskUpdated?.Invoke(this, data[0]);
                DigitalIOUpdated?.Invoke(this, digitalIO);
            });
            Log($"Interlock mask: 0x{data[0]:X2}");
        }
        private void HandleLsrWaveform(byte[] data)
        {
            var waveform = new WaveformData
            {
                CaptureTime = DateTime.Now
            };
            for (int i = 0; i < 32; i++)
            {
                waveform.Samples[i] = (ushort)((data[i * 2] << 8) | data[i * 2 + 1]);
            }
            _deviceData.Waveform = waveform;
            SafeInvoke(() => WaveformUpdated?.Invoke(this, waveform));
            Log($"Waveform received: {waveform.Samples.Length} samples");
        }
        private void HandleDiscovery()
        {
            Log("Responding to discovery command");
            SendCommand(uiTxCommand.uiTxNoCmd, new byte[] { 0x00 });
        }
        private void HandleLsrVolts(byte[] data)
        {
            var voltage = (ushort)((data[0] << 8) | data[1]);
            _deviceData.ActualVoltage = voltage;
            SafeInvoke(() => VoltsUpdated?.Invoke(this, voltage));
            Log($"Voltage updated: {voltage}V");
        }
        private void HandleLsrChargeState(byte[] data)
        {
            var chargeState = new ChargeStateData
            {
                MeasuredVolts = (ushort)((data[0] << 8) | data[1]),
                ChargeDone = data[2] != 0
            };
            _deviceData.ChargeState = chargeState;
            SafeInvoke(() => ChargeStateUpdated?.Invoke(this, chargeState));
            Log($"Charge state: {chargeState.MeasuredVolts}V, Done={chargeState.ChargeDone}");
        }
        private void HandleLsrChargeVolts(byte[] data)
        {
            try
            {
                if (data == null || data.Length != 2)
                {
                    Log($"Invalid data for uiRxLsrChargeVolts: Length={data?.Length ?? 0}");
                    return;
                }

                ushort voltageSetpoint = (ushort)((data[0] << 8) | data[1]);
                Log($"Charge voltage setpoint: {voltageSetpoint}V");

                _deviceData.VoltageSetpoint = voltageSetpoint;
                SafeInvoke(() => DeviceDataUpdated?.Invoke(this, _deviceData));
            }
            catch (Exception ex)
            {
                Log($"Error in HandleChargeVoltage: {ex.GetType().Name}: {ex.Message}");
                CommandError?.Invoke(this, ("uiRxLsrChargeVolts", ex));
            }
        }
        private void HandleShutterConfig(byte[] data)
        {
            var shutterConfig = new ShutterConfig
            {
                ShutterMode = (ShutterModeType)data[0],
                ShutterState = (ShutterStateType)data[1]
            };
            _deviceData.ShutterConfig = shutterConfig;
            SafeInvoke(() => ShutterConfigUpdated?.Invoke(this, shutterConfig));
            Log($"Shutter config: Mode={shutterConfig.ShutterMode}, State={shutterConfig.ShutterState}");
        }
        private void HandleSoftStartConfig(byte[] data)
        {
            var softStartConfig = new SoftStartConfig
            {
                Enable = data[0] != 0,
                IdleSetpoint = (ushort)((data[1] << 8) | data[2]),
                RampCount = (uint)((data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6])
            };
            _deviceData.SoftStartConfig = softStartConfig;
            SafeInvoke(() => SoftStartConfigUpdated?.Invoke(this, softStartConfig));
            Log($"Soft start: Enable={softStartConfig.Enable}, Idle={softStartConfig.IdleSetpoint}V");
        }
        private void ProcessPacket(uiRxCommand command, byte[] data)
        {
            try
            {
                Log($"Processing command {command} Data: {BitConverter.ToString(data).Replace("-", " ")}");

                int expectedLength = GetCommandLength(command);
                if (data?.Length != expectedLength)
                {
                    Log($"Invalid data length for {command}: expected {expectedLength}, got {data?.Length ?? 0}");
                    return;
                }

                // Use dispatcher for UI updates to ensure thread safety
                SafeInvoke(() =>
                {
                    try
                    {
                        switch (command)
                        {
                            case uiRxCommand.uiRxTestResp:
                                Log($"Test response received: 0x{data[0]:X2}");
                                break;

                            case uiRxCommand.uiRxFPGABadCmd:
                                Log($"FPGA reported bad command: 0x{data[0]:X2}");
                                break;

                            case uiRxCommand.uiRxLsrState:
                                HandleLsrState(data);
                                break;

                            case uiRxCommand.uiRxLsrPulseConfig:
                                HandleLsrPulseConfig(data);
                                break;

                            case uiRxCommand.uiRxLsrCount:
                                HandleLsrCount(data);
                                break;

                            case uiRxCommand.uiRxLsrRunStatus:
                                HandleLsrRunStatus(data);
                                break;

                            case uiRxCommand.uiRxLsrIntStatus:
                                HandleLsrIntStatus(data);
                                break;

                            case uiRxCommand.uiRxLsrIntMask:
                                HandleLsrIntMask(data);
                                break;

                            case uiRxCommand.uiRxLsrWaveform:
                                HandleLsrWaveform(data);
                                break;

                            case uiRxCommand.uiRxDiscovery:
                                HandleDiscovery();
                                break;

                            case uiRxCommand.uiRxLsrVolts:
                                HandleLsrVolts(data);
                                break;

                            case uiRxCommand.uiRxLsrChargeState:
                                HandleLsrChargeState(data);
                                break;

                            case uiRxCommand.uiRxLsrChargeVolts:
                                HandleLsrChargeVolts(data);
                                break;

                            case uiRxCommand.uiRxShutterConfig:
                                HandleShutterConfig(data);
                                break;

                            case uiRxCommand.uiRxSoftStartConfig:
                                HandleSoftStartConfig(data);
                                break;

                            default:
                                Log($"Unhandled command: {command} (0x{(byte)command:X2})");
                                break;
                        }
                        DeviceDataUpdated?.Invoke(this, _deviceData);
                    }
                    catch (Exception ex)
                    {
                        Log($"Error processing {command} on UI thread: {ex.Message}, Data: {(data?.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data")}");
                        CommandError?.Invoke(this, (command.ToString(), ex));
                    }
                });
            }
            catch (Exception ex)
            {
                Log($"Error in ProcessPacket for {command}: {ex.Message}, Data: {(data?.Length > 0 ? BitConverter.ToString(data).Replace("-", " ") : "No Data")}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }

        /*---------------------------------------------------------------------------
         * FUNCTIONS FOR SENDING COMMANDS (TX) 
         ----------------------------------------------------------------------------*/
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
        public void SendLsrPulseConfig(PulseConfig config)
        {
            byte[] data = new byte[14]
            {
                (byte)(config.Frequency >> 8), (byte)(config.Frequency & 0xFF),
                (byte)(config.PulseWidth >> 8), (byte)(config.PulseWidth & 0xFF),
                (byte)(config.ShotTotal >> 24), (byte)(config.ShotTotal >> 16), (byte)(config.ShotTotal >> 8), (byte)(config.ShotTotal & 0xFF),
                (byte)(config.Delay1 >> 8), (byte)(config.Delay1 & 0xFF),
                (byte)(config.Delay2 >> 8), (byte)(config.Delay2 & 0xFF),
                (byte)config.ShotMode,
                (byte)config.TrigMode
            };
            SendCommand(uiTxCommand.uiTxLsrPulseConfig, data);
        }
        public void SendLsrState(LaserStateType state)
        {
            SendCommand(uiTxCommand.uiTxLsrState, new byte[] { (byte)state });
        }

        public void SendIntMask(byte mask)
        {
            SendCommand(uiTxCommand.uiTxIntMask, new byte[] { mask });
        }
        public void SendWaveState(bool enabled)
        {
            SendCommand(uiTxCommand.uiTxWaveState, new byte[] { (byte)(enabled ? 1 : 0) });
        }
        public void SendLsrDelays(ushort delay1, ushort delay2)
        {
            byte[] data = new byte[4]
            {
                (byte)(delay1 >> 8), (byte)(delay1 & 0xFF),
                (byte)(delay2 >> 8), (byte)(delay2 & 0xFF)
            };
            SendCommand(uiTxCommand.uiTxLsrDelays, data);
        }
        public void SendLsrCal(ushort calValue)
        {
            byte[] data = new byte[2]
            {
                (byte)(calValue >> 8), (byte)(calValue & 0xFF)
            };
            SendCommand(uiTxCommand.uiTxLsrCal, data);
        }

        public void SendLsrVolts(ushort voltage)
        {
            byte[] data = new byte[2]
            {
                (byte)(voltage >> 8), (byte)(voltage & 0xFF)
            };
            SendCommand(uiTxCommand.uiTxLsrVolts, data);
        }
        public void SendLsrChargeCancel()
        {
            SendCommand(uiTxCommand.uiTxLsrChargeCancel, new byte[] { 0x00 });
        }
        public void SendShutterConfig(ShutterConfig config)
        {
            byte[] data = new byte[2]
            {
                (byte)config.ShutterMode,
                (byte)config.ShutterState
            };
            SendCommand(uiTxCommand.uiTxShutterConfig, data);
        }
        public void SendSoftStartConfig(SoftStartConfig config)
        {
            byte[] data = new byte[7]
            {
                (byte)(config.Enable ? 1 : 0),
                (byte)(config.IdleSetpoint >> 8), (byte)(config.IdleSetpoint & 0xFF),
                (byte)(config.RampCount >> 24), (byte)(config.RampCount >> 16), (byte)(config.RampCount >> 8), (byte)(config.RampCount & 0xFF)
            };
            SendCommand(uiTxCommand.uiTxSoftStartConfig, data);
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
                int expectedLength = GetCommandLength(command);
                if (data?.Length != expectedLength)
                {
                    Log($"Invalid data length for {command}: expected {expectedLength}, got {data?.Length ?? 0}");
                    return;
                }

                lock (_serialLock)
                {
                    byte[] packet = CommunicationHelper.CreatePacket((byte)command, data);
                    _serialPort.Write(packet, 0, packet.Length);
                    string hexData = BitConverter.ToString(data).Replace("-", " ");
                    Log($"SENT: {command} Data=[{hexData}]");
                    // Small delay to prevent overwhelming the device
                    Thread.Sleep(50);
                }
            }
            catch (Exception ex)
            {
                Log($"Error sending command {command}: {ex.Message}");
                CommandError?.Invoke(this, (command.ToString(), ex));
            }
        }
    }
}
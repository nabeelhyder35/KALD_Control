using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using Microsoft.Extensions.Logging;
using KALD_Control.Models;

namespace KALD_Control.Services
{
    public class DeviceManager : IDisposable
    {
        private readonly SerialPort _serialPort;
        private readonly ILogger<DeviceManager> _logger;
        private readonly object _serialLock = new object();
        private bool _disposed = false;

        // Packet framing constants
        private const byte STX = 0x2A; // Start of text
        private const byte ETX = 0x3A; // End of text

        // Buffer for assembling incoming packets
        private readonly List<byte> _incomingBuffer = new List<byte>();
        private enum ParserState { LookingForStart, ReadingHeader, ReadingData }
        private ParserState _currentState = ParserState.LookingForStart;
        private int _expectedDataLength = 0;

        // Events for UI updates
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

        public bool IsConnected => _serialPort.IsOpen;
        public int BaudRate => _serialPort.BaudRate;

        public DeviceManager(ILogger<DeviceManager> logger)
        {
            _logger = logger;
            _serialPort = new SerialPort
            {
                BaudRate = 115200,
                Parity = Parity.None,
                DataBits = 8,
                StopBits = StopBits.One,
                Handshake = Handshake.None,
                ReadTimeout = 1000,
                WriteTimeout = 1000,
                ReadBufferSize = 4096,
                WriteBufferSize = 4096
            };

            // Set error handling
            _serialPort.ErrorReceived += OnErrorReceived;
        }

        private void OnErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            Log($"Serial port error: {e.EventType}");

            // Handle specific error types
            switch (e.EventType)
            {
                case SerialError.RXOver:
                    Log("Input buffer overflow - data may be lost");
                    _serialPort.DiscardInBuffer();
                    break;
                case SerialError.Overrun:
                    Log("Character buffer overrun - data may be lost");
                    break;
                case SerialError.RXParity:
                    Log("Parity error - data corruption detected");
                    break;
                case SerialError.Frame:
                    Log("Framing error - check baud rate and settings");
                    break;
                case SerialError.TXFull:
                    Log("Output buffer full - cannot send more data");
                    _serialPort.DiscardOutBuffer();
                    break;
            }
        }

        public void Connect(string portName, int baudRate = 115200)
        {
            if (_serialPort.IsOpen)
            {
                Disconnect();
            }

            _serialPort.PortName = portName;
            _serialPort.BaudRate = baudRate;

            try
            {
                _serialPort.Open();

                // Subscribe to data received AFTER opening the port
                _serialPort.DataReceived += OnDataReceived;

                Log($"Connected to {portName} at {baudRate} baud successfully.");

                // Clear any existing data in the buffer
                _serialPort.DiscardInBuffer();
                _serialPort.DiscardOutBuffer();

                // Reset parser state
                ResetParser();

                // Send discovery command to initialize communication
                SendDiscoveryCommand();
            }
            catch (UnauthorizedAccessException ex)
            {
                Log($"Access denied to {portName}: {ex.Message}");
                throw new InvalidOperationException($"Access denied to {portName}. The port may be in use by another application.", ex);
            }
            catch (IOException ex)
            {
                Log($"I/O error connecting to {portName}: {ex.Message}");
                throw new InvalidOperationException($"I/O error connecting to {portName}. Check cable connection.", ex);
            }
            catch (Exception ex)
            {
                Log($"Connection error to {portName}: {ex.Message}");
                throw new InvalidOperationException($"Could not connect to {portName}.", ex);
            }
        }

        public void Disconnect()
        {
            if (!_serialPort.IsOpen)
            {
                Log("Not connected.");
                return;
            }

            try
            {
                // Unsubscribe first to prevent events during disconnect
                _serialPort.DataReceived -= OnDataReceived;
                _serialPort.Close();
                _incomingBuffer.Clear();
                _currentState = ParserState.LookingForStart;
                _expectedDataLength = 0;
                Log("Disconnected successfully.");
            }
            catch (Exception ex)
            {
                Log($"Disconnection error: {ex.Message}");
            }
        }

        public void SetBaudRate(int baudRate)
        {
            if (_serialPort.IsOpen)
            {
                _serialPort.Close();
                _serialPort.BaudRate = baudRate;
                _serialPort.Open();
                Log($"Baud rate changed to {baudRate}");
            }
            else
            {
                _serialPort.BaudRate = baudRate;
            }
        }

        // --- Command Sending Methods ---

        public void SendDiscoveryCommand()
        {
            SendCommand(LcdRxCommand.lcdRxDiscovery, new byte[0]);
            Log("Sent discovery command");
        }

        public void SendPulseConfig(PulseConfig config)
        {
            var data = new List<byte>();
            data.AddRange(ToBigEndian(config.Frequency));
            data.AddRange(ToBigEndian(config.PulseWidth));
            data.AddRange(ToBigEndian(config.ShotTotal));
            data.AddRange(ToBigEndian(config.Delay1));
            data.AddRange(ToBigEndian(config.Delay2));
            data.Add((byte)config.ShotMode);
            data.Add((byte)config.TrigMode);

            SendCommand(LcdRxCommand.lcdRxLsrPulseConfig, data.ToArray());
            Log($"Sent Pulse Config: F{config.Frequency / 10.0:F1}Hz, W{config.PulseWidth}us, {config.ShotTotal} shots");
        }

        public void SendStateCommand(LaserStateType state)
        {
            SendCommand(LcdRxCommand.lcdRxLsrState, new[] { (byte)state });
            Log($"Sent State Command: {state}");
        }

        public void SendInterlockMask(byte mask)
        {
            SendCommand(LcdRxCommand.lcdRxIntMask, new[] { mask });
            Log($"Sent Interlock Mask: 0x{mask:X2}");
        }

        public void SendWaveformState(bool enable)
        {
            SendCommand(LcdRxCommand.lcdRxWaveState, new[] { (byte)(enable ? 1 : 0) });
            Log($"Sent Waveform State: {enable}");
        }

        public void SendLaserDelays(ushort delay1, ushort delay2)
        {
            var data = new List<byte>();
            data.AddRange(ToBigEndian(delay1));
            data.AddRange(ToBigEndian(delay2));

            SendCommand(LcdRxCommand.lcdRxLsrDelays, data.ToArray());
            Log($"Sent Laser Delays: Delay1={delay1}us, Delay2={delay2}us");
        }

        public void SendLaserCalibration(ushort capVoltRange)
        {
            SendCommand(LcdRxCommand.lcdRxLsrCal, ToBigEndian(capVoltRange));
            Log($"Sent Laser Calibration: CapVoltRange={capVoltRange}");
        }

        public void SendVoltageSetpoint(ushort voltage)
        {
            if (voltage > 1500)
                throw new ArgumentOutOfRangeException(nameof(voltage), "Voltage cannot exceed 1500V.");

            SendCommand(LcdRxCommand.lcdRxLsrVolts, ToBigEndian(voltage));
            Log($"Sent Voltage Setpoint: {voltage}V");
        }

        public void SendChargeCancel()
        {
            SendCommand(LcdRxCommand.lcdRxLsrChargeCancel, new byte[] { 0x01 });
            Log("Sent Charge Cancel command.");
        }

        public void SendShutterConfig(ShutterConfig config)
        {
            var data = new byte[] { (byte)config.ShutterMode, (byte)config.ShutterState };
            SendCommand(LcdRxCommand.lcdRxShutterConfig, data);
            Log($"Sent Shutter Config: Mode={config.ShutterMode}, State={config.ShutterState}");
        }

        public void SendSoftStartConfig(SoftStartConfig config)
        {
            var data = new List<byte>();
            data.Add((byte)(config.Enable ? 1 : 0));
            data.AddRange(ToBigEndian(config.IdleSetpoint));
            data.AddRange(ToBigEndian((uint)config.RampCount));

            SendCommand(LcdRxCommand.lcdRxSoftStartConfig, data.ToArray());
            Log($"Sent SoftStart Config: Enable={config.Enable}, IdleV={config.IdleSetpoint}, RampShots={config.RampCount}");
        }

        public void SendDigitalOutput(int channel, bool state)
        {
            if (channel < 0 || channel > 31)
                throw new ArgumentOutOfRangeException(nameof(channel), "Channel must be between 0-31");

            byte[] data = new byte[2];
            data[0] = (byte)channel;
            data[1] = (byte)(state ? 1 : 0);

            SendCommand(LcdRxCommand.lcdRxDigitalIO, data);
            Log($"Sent Digital Output: Channel={channel}, State={state}");
        }

        public void RequestSystemStatus()
        {
            SendDiscoveryCommand();
        }

        public void RequestInterlockStatus()
        {
            SendCommand(LcdRxCommand.lcdRxIntMask, new byte[] { 0x00 });
        }

        public void RequestWaveformData()
        {
            SendWaveformState(true);
        }

        public void SendSystemReset()
        {
            SendCommand(LcdRxCommand.lcdRxSystemReset, new byte[] { 0x01 });
            Log("System reset command sent");
        }

        // --- Core Packet Handling ---

        private void SendCommand(LcdRxCommand command, byte[] data)
        {
            lock (_serialLock)
            {
                try
                {
                    if (!_serialPort.IsOpen)
                    {
                        Log("Cannot send packet - serial port not open");
                        return;
                    }

                    // Additional connection validation
                    if (_serialPort.BytesToWrite > 1000)
                    {
                        Log("Output buffer full, clearing before send");
                        _serialPort.DiscardOutBuffer();
                    }

                    // Packet structure: STX(1) + LEN(1) + CMD(1) + DATA(n) + CHKSUM(1) + ETX(1)
                    // LEN = data.Length + 2 (CMD + CHKSUM)
                    byte length = (byte)(data.Length + 2);
                    byte[] packet = new byte[4 + data.Length + 1]; // STX + LEN + CMD + DATA + CHKSUM + ETX

                    packet[0] = STX;
                    packet[1] = length;
                    packet[2] = (byte)command;

                    Array.Copy(data, 0, packet, 3, data.Length);

                    // Calculate checksum: CMD + all data bytes
                    byte checksum = (byte)command;
                    foreach (byte b in data)
                    {
                        checksum += b;
                    }
                    checksum = (byte)(0 - checksum);

                    packet[3 + data.Length] = checksum;
                    packet[4 + data.Length] = ETX;

                    // Send with timeout protection
                    _serialPort.WriteTimeout = 1000;
                    _serialPort.Write(packet, 0, packet.Length);

                    Log($"TX Cmd: {command} (0x{(byte)command:X2}), Data: {BitConverter.ToString(data)}, Checksum: 0x{checksum:X2}");
                }
                catch (TimeoutException)
                {
                    Log("Write timeout - serial port may be disconnected");
                    HandleCOMException(new System.Runtime.InteropServices.COMException("Write timeout", -2147417842));
                }
                catch (System.Runtime.InteropServices.COMException comEx)
                {
                    HandleCOMException(comEx);
                }
                catch (Exception ex)
                {
                    Log($"Error sending packet: {ex.Message}");
                }
            }
        }

        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // Use a try-catch at the very top level to catch any COM exceptions
            try
            {
                ProcessIncomingData();
            }
            catch (System.Runtime.InteropServices.COMException comEx)
            {
                HandleCOMException(comEx);
            }
            catch (Exception ex)
            {
                Log($"Unexpected error in data received: {ex.Message}");
                ResetParser();
            }
        }

        private void ProcessIncomingData()
        {
            if (!_serialPort.IsOpen)
                return;

            // Use a buffer to read all available data at once
            int bytesToRead = _serialPort.BytesToRead;
            if (bytesToRead <= 0)
                return;

            byte[] buffer = new byte[bytesToRead];
            int totalBytesRead = 0;

            // Read all available data with timeout protection
            DateTime startTime = DateTime.Now;
            while (totalBytesRead < bytesToRead && (DateTime.Now - startTime).TotalMilliseconds < 1000)
            {
                try
                {
                    int bytesRead = _serialPort.Read(buffer, totalBytesRead, bytesToRead - totalBytesRead);
                    if (bytesRead == 0) break;
                    totalBytesRead += bytesRead;
                }
                catch (TimeoutException)
                {
                    Log("Read timeout occurred");
                    break;
                }
                catch (InvalidOperationException ioEx)
                {
                    Log($"Serial port operation invalid: {ioEx.Message}");
                    break;
                }
            }

            if (totalBytesRead == 0)
            {
                Log("No data read from serial port");
                return;
            }

            Log($"Received {totalBytesRead} bytes: {BitConverter.ToString(buffer, 0, Math.Min(totalBytesRead, 32))}");

            // Process each byte
            for (int i = 0; i < totalBytesRead; i++)
            {
                byte currentByte = buffer[i];

                switch (_currentState)
                {
                    case ParserState.LookingForStart:
                        if (currentByte == STX)
                        {
                            _incomingBuffer.Clear();
                            _incomingBuffer.Add(currentByte);
                            _currentState = ParserState.ReadingHeader;
                        }
                        break;

                    case ParserState.ReadingHeader:
                        _incomingBuffer.Add(currentByte);

                        if (_incomingBuffer.Count == 3) // STX + LEN(1) + CMD(1)
                        {
                            // Extract length from byte 1
                            _expectedDataLength = _incomingBuffer[1] - 2; // Subtract CMD and CHKSUM

                            // Validate length
                            if (_expectedDataLength < 0 || _expectedDataLength > 1024)
                            {
                                Log($"Invalid expected length: {_expectedDataLength}. Resetting parser.");
                                ResetParser();
                                break;
                            }

                            _currentState = ParserState.ReadingData;
                        }
                        break;

                    case ParserState.ReadingData:
                        _incomingBuffer.Add(currentByte);

                        // Total packet should be: STX(1) + LEN(1) + CMD(1) + DATA(n) + CHKSUM(1) + ETX(1)
                        int expectedTotalLength = 3 + _expectedDataLength + 2;

                        if (_incomingBuffer.Count >= expectedTotalLength)
                        {
                            if (_incomingBuffer[_incomingBuffer.Count - 1] == ETX)
                            {
                                ProcessPacket(_incomingBuffer.ToArray());
                            }
                            else
                            {
                                Log($"Missing ETX byte. Last byte: 0x{_incomingBuffer[_incomingBuffer.Count - 1]:X2}");
                            }
                            ResetParser();
                        }
                        break;
                }
            }

            // Check if parser state is stuck and reset if necessary
            if (_currentState != ParserState.LookingForStart && _incomingBuffer.Count > 100)
            {
                Log($"Parser state stuck. Current state: {_currentState}, Buffer count: {_incomingBuffer.Count}");
                ResetParser();
            }
        }

        private void ProcessPacket(byte[] packet)
        {
            try
            {
                if (packet == null || packet.Length < 6)
                {
                    Log($"Packet too short: {packet?.Length ?? 0} bytes");
                    return;
                }

                // Verify STX
                if (packet[0] != STX)
                {
                    Log($"Invalid STX: 0x{packet[0]:X2}");
                    return;
                }

                // Extract length
                byte length = packet[1];

                // Verify packet length matches expected
                if (packet.Length != length + 3) // STX + LEN + DATA + CHKSUM + ETX
                {
                    Log($"Length mismatch. Expected {length + 3} bytes, got {packet.Length}");
                    return;
                }

                // Verify ETX
                if (packet[packet.Length - 1] != ETX)
                {
                    Log($"Invalid ETX: 0x{packet[packet.Length - 1]:X2}");
                    return;
                }

                // Extract command and data
                byte commandByte = packet[2];

                // Handle special negative commands first
                if (commandByte == SpecialCommands.LCD_RX_BAD_CMD)
                {
                    Log("Received bad command response from FPGA - checksum error");
                    return;
                }
                else if (commandByte == SpecialCommands.LCD_RX_NO_CMD)
                {
                    Log("No command received from FPGA");
                    return;
                }

                LcdTxCommand command = (LcdTxCommand)commandByte;

                // Calculate data length: length includes CMD(1) + DATA(n) + CHKSUM(1)
                int dataLength = length - 2;

                if (dataLength < 0)
                {
                    Log($"Invalid data length: {dataLength}");
                    return;
                }

                byte[] data = new byte[dataLength];
                Array.Copy(packet, 3, data, 0, dataLength);

                byte receivedChecksum = packet[3 + dataLength];

                // Calculate checksum: CMD + all data bytes
                byte calculatedChecksum = commandByte;
                for (int i = 0; i < dataLength; i++)
                {
                    calculatedChecksum += data[i];
                }
                calculatedChecksum = (byte)(0 - calculatedChecksum);

                if (receivedChecksum != calculatedChecksum)
                {
                    Log($"Checksum error for command {command}. Expected 0x{calculatedChecksum:X2}, got 0x{receivedChecksum:X2}");
                    return;
                }

                // Handle the command
                switch (command)
                {
                    case LcdTxCommand.lcdTxDiscovery:
                        if (data.Length >= 1)
                        {
                            StateUpdated?.Invoke(this, (LaserStateType)data[0]);
                            Log($"Discovery response: State = {(LaserStateType)data[0]}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrState:
                        if (data.Length >= 1)
                        {
                            StateUpdated?.Invoke(this, (LaserStateType)data[0]);
                            Log($"Laser state updated: {(LaserStateType)data[0]}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrRunStatus:
                        if (data.Length >= 14)
                        {
                            var runStatus = new RunStatusData
                            {
                                ShotCount = FromBigEndianUint(data, 0),
                                State = (LaserStateType)data[4],
                                Energy = FromBigEndianUshort(data, 5),
                                Power = FromBigEndianUshort(data, 7),
                                Current = FromBigEndianUshort(data, 9),
                                VDroop = FromBigEndianUshort(data, 11)
                            };
                            RunStatusUpdated?.Invoke(this, runStatus);
                            Log($"Run status: {runStatus.ShotCount} shots, {runStatus.Energy}mJ");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrIntStatus:
                        if (data.Length >= 1)
                        {
                            IntStatusUpdated?.Invoke(this, data[0]);
                            Log($"Interlock status: 0x{data[0]:X2}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrIntMask:
                        if (data.Length >= 1)
                        {
                            IntMaskUpdated?.Invoke(this, data[0]);
                            Log($"Interlock mask: 0x{data[0]:X2}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrWaveform:
                        if (data.Length >= 64) // 32 samples * 2 bytes each
                        {
                            var waveform = new WaveformData
                            {
                                Samples = new ushort[32],
                                CaptureTime = DateTime.Now
                            };

                            for (int i = 0; i < 32; i++)
                            {
                                waveform.Samples[i] = FromBigEndianUshort(data, i * 2);
                            }

                            WaveformUpdated?.Invoke(this, waveform);
                            Log($"Waveform data received: {waveform.Samples.Length} samples");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrVolts:
                        if (data.Length >= 2)
                        {
                            ushort voltage = FromBigEndianUshort(data, 0);
                            VoltsUpdated?.Invoke(this, voltage);
                            Log($"Voltage updated: {voltage}");
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
                            Log($"Charge state: Measured={chargeState.MeasuredVolts}, Done={chargeState.ChargeDone}");
                        }
                        break;

                    case LcdTxCommand.lcdTxShutterConfig:
                        if (data.Length >= 2)
                        {
                            var config = new ShutterConfig
                            {
                                ShutterMode = (ShutterModeType)data[0],
                                ShutterState = (ShutterStateType)data[1]
                            };
                            ShutterConfigUpdated?.Invoke(this, config);
                            Log($"Shutter config: Mode={config.ShutterMode}, State={config.ShutterState}");
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
                            Log($"SoftStart config: Enable={config.Enable}, IdleV={config.IdleSetpoint}, RampShots={config.RampCount}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrCal:
                        if (data.Length >= 4)
                        {
                            var calibration = new CalibrationData
                            {
                                CapVoltRange = FromBigEndianUshort(data, 0),
                                MeasuredVoltage = FromBigEndianUshort(data, 2)
                            };
                            CalibrationUpdated?.Invoke(this, calibration);
                            Log($"Calibration: Range={calibration.CapVoltRange}, Measured={calibration.MeasuredVoltage}");
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
                            Log($"Digital I/O: Inputs=0x{dioState.InputStates:X8}, Outputs=0x{dioState.OutputStates:X8}");
                        }
                        break;

                    default:
                        Log($"Unhandled command received: {command} (0x{commandByte:X2})");
                        break;
                }

                Log($"RX Cmd: {command} (0x{commandByte:X2}), Data: {BitConverter.ToString(data)}");
            }
            catch (Exception ex)
            {
                Log($"Error processing packet: {ex.Message}");
            }
        }

        private void ResetParser()
        {
            _incomingBuffer.Clear();
            _currentState = ParserState.LookingForStart;
            _expectedDataLength = 0;
        }

        private void HandleCOMException(System.Runtime.InteropServices.COMException comEx)
        {
            Log($"COM Exception (Error Code: {comEx.ErrorCode}): {comEx.Message}");

            switch (comEx.ErrorCode)
            {
                case -2147417842: // RPC_E_DISCONNECTED
                    Log("COM port disconnected unexpectedly - the connection was lost");
                    break;
                case -2147467259: // Generic failure
                    Log("Generic COM failure - check cable connection and port settings");
                    break;
                case -2147024891: // Access denied
                    Log("Access denied to COM port - may be in use by another application");
                    break;
                case -2147024809: // Invalid parameter
                    Log("Invalid parameter - check serial port configuration");
                    break;
                default:
                    Log($"Unknown COM error: {comEx.ErrorCode}");
                    break;
            }

            // Try to recover by resetting the connection
            if (IsConnected)
            {
                Log("Attempting to recover from COM error by resetting connection...");
                try
                {
                    Disconnect();
                }
                catch (Exception disconnectEx)
                {
                    Log($"Error during disconnect: {disconnectEx.Message}");
                }
            }
        }

        // --- Utility Methods ---

        private byte[] ToBigEndian(ushort value) => BitConverter.GetBytes(value).Reverse().ToArray();
        private byte[] ToBigEndian(uint value) => BitConverter.GetBytes(value).Reverse().ToArray();

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
            LogMessage?.Invoke(this, formattedMessage);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            try
            {
                if (_serialPort.IsOpen)
                {
                    _serialPort.DataReceived -= OnDataReceived;
                    _serialPort.ErrorReceived -= OnErrorReceived;
                    _serialPort.Close();
                }
                _serialPort.Dispose();
                Log("DeviceManager disposed.");
            }
            catch (Exception ex)
            {
                Log($"Error during dispose: {ex.Message}");
            }
        }
    }
}
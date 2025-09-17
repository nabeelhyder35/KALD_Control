using KALD_Control.Models;
using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Data;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;

namespace KALD_Control.Services
{
    public class DeviceManager : IDisposable
    {
        private readonly SerialPort _serialPort;
        private readonly ILogger<DeviceManager> _logger;
        private readonly object _serialLock = new object();
        private bool _disposed = false;

        // Packet framing constants - matches firmware: FPGA_START=0x2A, FPGA_END=0x3A
        private const byte STX = 0x2A; // Start of text
        private const byte ETX = 0x3A; // End of text

        // Buffer and parser state for assembling incoming packets
        private readonly List<byte> _incomingBuffer = new List<byte>();
        private enum ParserState { LookingForStart, ReadingHeader, ReadingData }
        private ParserState _currentState = ParserState.LookingForStart;
        private int _expectedDataLength = 0;

        // Events for UI updates based on received packets
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
                Parity = Parity.None,
                DataBits = 8,
                StopBits = StopBits.One,
                Handshake = Handshake.None,
                ReadTimeout = 1000,
                WriteTimeout = 1000,
                ReadBufferSize = 4096,
                WriteBufferSize = 4096
            };
            _serialPort.ErrorReceived += OnErrorReceived;
        }

        private void OnErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            Log($"Serial port error detected: {e.EventType}");
            switch (e.EventType)
            {
                case SerialError.RXOver:
                    Log("Input buffer overflow detected - discarding input buffer to recover");
                    _serialPort.DiscardInBuffer();
                    break;
                case SerialError.Overrun:
                    Log("Character buffer overrun - potential data loss, check baud rate and timing");
                    break;
                case SerialError.RXParity:
                    Log("Parity error - data corruption detected, verify serial settings");
                    break;
                case SerialError.Frame:
                    Log("Framing error - incorrect baud rate or stop bits, verify configuration");
                    break;
                case SerialError.TXFull:
                    Log("Output buffer full - discarding output buffer to recover");
                    _serialPort.DiscardOutBuffer();
                    break;
            }
        }

        public void Connect(string portName, int baudRate)
        {
            if (_serialPort.IsOpen)
                Disconnect();

            _serialPort.PortName = portName;
            _serialPort.BaudRate = baudRate;

            try
            {
                _serialPort.Open();
                _serialPort.DataReceived += OnDataReceived;
                Log($"Successfully connected to {portName} at {baudRate} baud");
                _serialPort.DiscardInBuffer();
                _serialPort.DiscardOutBuffer();
                ResetParser();
                SendDiscoveryCommand();
            }
            catch (UnauthorizedAccessException ex)
            {
                Log($"Failed to connect to {portName}: Access denied - {ex.Message}");
                throw new InvalidOperationException($"Access denied to {portName}.", ex);
            }
            catch (IOException ex)
            {
                Log($"Failed to connect to {portName}: I/O error - {ex.Message}");
                throw new InvalidOperationException($"I/O error connecting to {portName}.", ex);
            }
            catch (Exception ex)
            {
                Log($"Failed to connect to {portName}: Unexpected error - {ex.Message}");
                throw new InvalidOperationException($"Could not connect to {portName}.", ex);
            }
        }

        public void Disconnect()
        {
            if (!_serialPort.IsOpen)
            {
                Log("Disconnect requested but no active connection exists");
                return;
            }
            try
            {
                _serialPort.DataReceived -= OnDataReceived;
                _serialPort.Close();
                _incomingBuffer.Clear();
                _currentState = ParserState.LookingForStart;
                _expectedDataLength = 0;
                Log("Successfully disconnected from serial port");
            }
            catch (Exception ex)
            {
                Log($"Error during disconnection: {ex.Message}");
            }
        }

        public void SetBaudRate(int baudRate)
        {
            if (_serialPort.IsOpen)
            {
                _serialPort.Close();
                _serialPort.BaudRate = baudRate;
                _serialPort.Open();
                Log($"Baud rate updated to {baudRate}");
            }
            else
            {
                _serialPort.BaudRate = baudRate;
                Log($"Baud rate set to {baudRate} for next connection");
            }
        }

        public void SendDiscoveryCommand()
        {
            SendCommand(LcdRxCommand.lcdRxDiscovery, new byte[0]);
            Log("Sent discovery command to request laser state");
        }

        public void SendPulseConfig(PulseConfig config)
        {
            var data = new List<byte>();

            // Fix for CS0266: Explicit cast int to ushort with range check
            if (config.Frequency < ushort.MinValue || config.Frequency > ushort.MaxValue)
            {
                Log($"Frequency value out of range for ushort: {config.Frequency} (must be 0 to 65535)");
                return;
            }
            data.AddRange(ToBigEndian((ushort)config.Frequency));

            data.AddRange(ToBigEndian((ushort)config.PulseWidth));
            data.AddRange(ToBigEndian(config.ShotTotal));
            data.AddRange(ToBigEndian((ushort)config.Delay1));
            data.AddRange(ToBigEndian((ushort)config.Delay2));
            ushort modes = (ushort)(((ushort)config.ShotMode << 8) | (ushort)config.TrigMode);
            data.AddRange(ToBigEndian(modes));

            if (data.Count != FpgaCommandLengths.LSR_PULSE_CONFIG)
            {
                Log($"Pulse config data length error: Got {data.Count} bytes, expected {FpgaCommandLengths.LSR_PULSE_CONFIG} bytes");
                return;
            }

            SendCommand(LcdRxCommand.lcdRxLsrPulseConfig, data.ToArray());
            Log($"Sent laser pulse configuration: Frequency={config.Frequency / 10.0:F1}Hz, PulseWidth={config.PulseWidth}us, Shots={config.ShotTotal}, Delay1={config.Delay1}us, Delay2={config.Delay2}us, ShotMode={config.ShotMode}, TrigMode={config.TrigMode}");
        }

        public void SendStateCommand(LaserStateType state)
        {
            SendCommand(LcdRxCommand.lcdRxLsrState, new[] { (byte)state });
            Log($"Sent laser state command: State={state}");
        }

        public void SendInterlockMask(byte mask)
        {
            SendCommand(LcdRxCommand.lcdRxIntMask, new[] { mask });
            Log($"Sent interlock mask command: Mask=0x{mask:X2}");
        }

        public void SendWaveformState(bool enable)
        {
            SendCommand(LcdRxCommand.lcdRxWaveState, new[] { (byte)(enable ? 1 : 0) });
            Log($"Sent waveform messaging state: Enable={enable}");
        }

        public void SendLaserDelays(ushort delay1, ushort delay2)
        {
            var data = new List<byte>();
            data.AddRange(ToBigEndian(delay1));
            data.AddRange(ToBigEndian(delay2));

            if (data.Count != FpgaCommandLengths.LSR_DELAYS)
            {
                Log($"Laser delays data length error: Got {data.Count} bytes, expected {FpgaCommandLengths.LSR_DELAYS} bytes");
                return;
            }

            SendCommand(LcdRxCommand.lcdRxLsrDelays, data.ToArray());
            Log($"Sent laser delays: Delay1={delay1}us, Delay2={delay2}us");
        }

        public void SendDigitalIOCommand()
        {
            // Assuming this sends a command for digital IO, adjust as per your logic
            SendCommand(LcdRxCommand.lcdRxDigitalIO, new byte[0]); // Or with data if needed
            Log("Sent digital IO command");
        }

        public void SendSystemReset()
        {
            SendCommand(LcdRxCommand.lcdRxSystemReset, new byte[0]);
            Log("Sent system reset command");
        }
        public void RequestSystemStatus()
        {
            SendCommand(LcdRxCommand.lcdRxRunStatus, new byte[0]);
            Log("Requested system status");
        }

        public void RequestWaveformData()
        {
            SendCommand(LcdRxCommand.lcdRxWaveform, new byte[0]);
            Log("Requested waveform data");
        }

        public void SendLaserCalibration(ushort capVoltRange)
        {
            var data = ToBigEndian(capVoltRange);
            SendCommand(LcdRxCommand.lcdRxLsrCal, data);
            Log($"Sent laser calibration: CapVoltRange={capVoltRange}V");
        }

        public void SendChargeCancel()
        {
            SendCommand(LcdRxCommand.lcdRxLsrChargeCancel, new byte[] { 1 });
            Log("Sent charge cancel command");
        }
        // Add these methods to DeviceManager.cs
        public void SendVolts(ushort voltage)
        {
            SendCommand(LcdRxCommand.lcdRxLsrVolts, ToBigEndian(voltage));
            Log($"Sent voltage setpoint: {voltage}V");
        }

        public void SendShutterConfig(ShutterConfig config)
        {
            ushort packed = (ushort)(((ushort)config.ShutterMode << 8) | (ushort)config.ShutterState);
            SendCommand(LcdRxCommand.lcdRxShutterConfig, ToBigEndian(packed));
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


        public void SendCommand(LcdRxCommand command, byte[] data)
        {
            lock (_serialLock)
            {
                if (!_serialPort.IsOpen) return;

                byte checksum = (byte)command;
                foreach (byte b in data)
                {
                    checksum = (byte)(checksum + b);
                }
                checksum = (byte)(0 - checksum);

                var packet = new List<byte> { STX };
                packet.Add((byte)((data.Length >> 8) & 0xFF));
                packet.Add((byte)(data.Length & 0xFF));
                packet.Add((byte)command);
                packet.AddRange(data);
                packet.Add(checksum);
                packet.Add(ETX);

                _serialPort.Write(packet.ToArray(), 0, packet.Count);
            }
        }

        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            lock (_serialLock)
            {
                try
                {
                    int bytesToRead = _serialPort.BytesToRead;
                    byte[] buffer = new byte[bytesToRead];
                    _serialPort.Read(buffer, 0, bytesToRead);
                    _incomingBuffer.AddRange(buffer);

                    while (_incomingBuffer.Count > 0)
                    {
                        switch (_currentState)
                        {
                            case ParserState.LookingForStart:
                                int stxIndex = _incomingBuffer.IndexOf(STX);
                                if (stxIndex >= 0)
                                {
                                    _incomingBuffer.RemoveRange(0, stxIndex);
                                    _currentState = ParserState.ReadingHeader;
                                    _incomingBuffer.RemoveAt(0); // Remove STX
                                }
                                else
                                {
                                    _incomingBuffer.Clear();
                                    return;
                                }
                                break;

                            case ParserState.ReadingHeader:
                                if (_incomingBuffer.Count < 3)
                                    return;

                                int dataLength = (_incomingBuffer[0] << 8) | _incomingBuffer[1];
                                byte commandByte = _incomingBuffer[2];

                                _expectedDataLength = dataLength + 1; // Data + Checksum
                                _incomingBuffer.RemoveRange(0, 3); // Remove length and command

                                if (_incomingBuffer.Count >= _expectedDataLength + 1) // +1 for ETX
                                {
                                    ProcessPacket(commandByte, dataLength);
                                }
                                else
                                {
                                    _currentState = ParserState.ReadingData;
                                }
                                break;

                            case ParserState.ReadingData:
                                if (_incomingBuffer.Count < _expectedDataLength + 1) // +1 for ETX
                                    return;

                                // Process the packet (commandByte from header)
                                ProcessPacket(_incomingBuffer[-(_expectedDataLength + 3) + 2], _expectedDataLength - 1); // Adjust as needed
                                break;
                        }
                    }
                }
                catch (Exception ex)
                {
                    Log($"Error receiving data: {ex.Message}");
                    ResetParser();
                }
            }
        }

        private void ProcessPacket(byte commandByte, int dataLength)
        {
            try
            {
                if (_incomingBuffer[_expectedDataLength] != ETX)
                {
                    Log("Invalid ETX - discarding packet");
                    ResetParser();
                    return;
                }

                byte[] data = _incomingBuffer.GetRange(0, dataLength).ToArray();
                byte receivedChecksum = _incomingBuffer[dataLength];

                byte calculatedChecksum = commandByte;
                foreach (byte b in data)
                {
                    calculatedChecksum = (byte)(calculatedChecksum + b);
                }
                calculatedChecksum = (byte)(0 - calculatedChecksum);

                if (receivedChecksum != calculatedChecksum)
                {
                    Log($"Checksum mismatch: Received=0x{receivedChecksum:X2}, Calculated=0x{calculatedChecksum:X2}");
                    ResetParser();
                    return;
                }

                _incomingBuffer.RemoveRange(0, _expectedDataLength + 1); // Data + Checksum + ETX
                _currentState = ParserState.LookingForStart;

                // Handle command
                LcdTxCommand command = (LcdTxCommand)commandByte;
                switch (command)
                {
                    case LcdTxCommand.lcdTxBadCmd:
                        Log("Received bad command notification from device");
                        break;

                    // In DeviceManager.cs, replace the relevant case blocks in ProcessPacket
                    case LcdTxCommand.lcdTxLsrPulseConfig:
                        if (data.Length != FpgaCommandLengths.LSR_PULSE_CONFIG)
                        {
                            Log($"Pulse config response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.LSR_PULSE_CONFIG} bytes");
                            break;
                        }
                        var pulseConfig = new PulseConfig // Renamed from 'config' to 'pulseConfig'
                        {
                            Frequency = FromBigEndianUshort(data, 0),
                            PulseWidth = FromBigEndianUshort(data, 2),
                            ShotTotal = FromBigEndianUint(data, 4),
                            Delay1 = FromBigEndianUshort(data, 8),
                            Delay2 = FromBigEndianUshort(data, 10),
                            ShotMode = (ShotModeType)(data[12]),
                            TrigMode = (TriggerModeType)(data[13])
                        };
                        PulseConfigUpdated?.Invoke(this, pulseConfig);
                        Log($"Received pulse config: Freq={pulseConfig.Frequency / 10.0:F1}Hz, PW={pulseConfig.PulseWidth}us, Shots={pulseConfig.ShotTotal}, D1={pulseConfig.Delay1}, D2={pulseConfig.Delay2}, Mode={pulseConfig.ShotMode}/{pulseConfig.TrigMode}");
                        break;

                  

                    case LcdTxCommand.lcdTxShutterConfig:
                        if (data.Length != FpgaCommandLengths.SHUTTER_CONFIG)
                        {
                            Log($"Shutter config response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.SHUTTER_CONFIG} bytes");
                            break;
                        }
                        if (data.Length >= 2)
                        {
                            ushort packed = FromBigEndianUshort(data, 0);
                            var config = new ShutterConfig
                            {
                                ShutterMode = (ShutterModeType)(packed >> 8),
                                ShutterState = (ShutterStateType)(packed & 0xFF)
                            };
                            ShutterConfigUpdated?.Invoke(this, config);
                            Log($"Received shutter configuration: lsrShutterMode={config.ShutterMode}, lsrShutterState={config.ShutterState}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrState:
                        if (data.Length != FpgaCommandLengths.LSR_STATE)
                        {
                            Log($"State response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.LSR_STATE} bytes");
                            break;
                        }
                        if (data.Length >= 1)
                        {
                            LaserStateType state = (LaserStateType)data[0];
                            StateUpdated?.Invoke(this, state);
                            Log($"Received state: {state}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrCount:
                        if (data.Length != FpgaCommandLengths.LSR_SHOTS)
                        {
                            Log($"Shot count response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.LSR_SHOTS} bytes");
                            break;
                        }
                        if (data.Length >= 4)
                        {
                            uint shotCount = FromBigEndianUint(data, 0);
                            ShotCountUpdated?.Invoke(this, shotCount);
                            Log($"Received shot count: {shotCount}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrRunStatus:
                        if (data.Length != FpgaCommandLengths.RUN_STATUS)
                        {
                            Log($"Run status response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.RUN_STATUS} bytes");
                            break;
                        }
                        if (data.Length >= 14)
                        {
                            var status = new RunStatusData
                            {
                                ShotCount = FromBigEndianUint(data, 0),
                                State = (LaserStateType)data[4],
                                Energy = FromBigEndianUshort(data, 5),
                                Power = FromBigEndianUshort(data, 7),
                                Current = FromBigEndianUshort(data, 9),
                                VDroop = FromBigEndianUshort(data, 11)
                            };
                            RunStatusUpdated?.Invoke(this, status);
                            Log($"Received run status: Shots={status.ShotCount}, State={status.State}, E={status.Energy}, P={status.Power}, C={status.Current}, VD={status.VDroop}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrIntStatus:
                        if (data.Length != FpgaCommandLengths.INT_STATUS)
                        {
                            Log($"Int status response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.INT_STATUS} bytes");
                            break;
                        }
                        if (data.Length >= 1)
                        {
                            byte intStatus = data[0];
                            IntStatusUpdated?.Invoke(this, intStatus);
                            Log($"Received interlock status: 0x{intStatus:X2}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrIntMask:
                        if (data.Length != FpgaCommandLengths.INT_MASK)
                        {
                            Log($"Int mask response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.INT_MASK} bytes");
                            break;
                        }
                        if (data.Length >= 1)
                        {
                            byte intMask = data[0];
                            IntMaskUpdated?.Invoke(this, intMask);
                            Log($"Received interlock mask: 0x{intMask:X2}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrWaveform:
                        if (data.Length != FpgaCommandLengths.WAVEFORM)
                        {
                            Log($"Waveform response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.WAVEFORM} bytes");
                            break;
                        }
                        var samples = new ushort[data.Length / 2];
                        for (int i = 0; i < samples.Length; i++)
                        {
                            samples[i] = FromBigEndianUshort(data, i * 2);
                        }
                        var waveform = new WaveformData { Samples = samples, CaptureTime = DateTime.Now };
                        WaveformUpdated?.Invoke(this, waveform);
                        Log($"Received waveform: {samples.Length} samples");
                        break;

                    case LcdTxCommand.lcdTxDiscovery:
                        if (data.Length != FpgaCommandLengths.DISCOVERY)
                        {
                            Log($"Discovery response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.DISCOVERY} bytes");
                            break;
                        }
                        Log("Received discovery response");
                        break;

                    case LcdTxCommand.lcdTxLsrVolts:
                        if (data.Length != FpgaCommandLengths.LSR_VOLTS)
                        {
                            Log($"Volts response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.LSR_VOLTS} bytes");
                            break;
                        }
                        if (data.Length >= 2)
                        {
                            ushort volts = FromBigEndianUshort(data, 0);
                            VoltsUpdated?.Invoke(this, volts);
                            Log($"Received voltage: {volts}V");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrChargeState:
                        if (data.Length != FpgaCommandLengths.CHARGE_STATE)
                        {
                            Log($"Charge state response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.CHARGE_STATE} bytes");
                            break;
                        }
                        if (data.Length >= 3)
                        {
                            var state = new ChargeStateData
                            {
                                MeasuredVolts = FromBigEndianUshort(data, 0),
                                ChargeDone = data[2] != 0
                            };
                            ChargeStateUpdated?.Invoke(this, state);
                            Log($"Received charge state: MeasuredVolts={state.MeasuredVolts}V, ChargeDone={state.ChargeDone}");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrChargeVolts:
                        if (data.Length != FpgaCommandLengths.CHARGE_VOLT)
                        {
                            Log($"Charge volt response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.CHARGE_VOLT} bytes");
                            break;
                        }
                        if (data.Length >= 2)
                        {
                            ushort chargeVolt = FromBigEndianUshort(data, 0);
                            Log($"Received charge voltage setpoint: chargeVolts={chargeVolt}V");
                        }
                        break;


                    case LcdTxCommand.lcdTxSoftStartConfig:
                        if (data.Length != FpgaCommandLengths.SOFT_START_CONFIG)
                        {
                            Log($"Soft start config response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.SOFT_START_CONFIG} bytes");
                            break;
                        }
                        if (data.Length >= 7)
                        {
                            byte enable = data[0];
                            ushort idle = FromBigEndianUshort(data, 1);
                            uint rampCount = FromBigEndianUint(data, 3);
                            var config = new SoftStartConfig
                            {
                                Enable = enable != 0,
                                IdleSetpoint = idle,
                                RampCount = rampCount
                            };
                            SoftStartConfigUpdated?.Invoke(this, config);
                            Log($"Received soft start configuration: lsrSoftStart={config.Enable}, lsrIdleSetpoint={config.IdleSetpoint}V, lsrRampCount={config.RampCount} shots");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrCal:
                        if (data.Length != FpgaCommandLengths.LSR_CAL)
                        {
                            Log($"Calibration response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.LSR_CAL} bytes");
                            break;
                        }
                        if (data.Length >= 2)
                        {
                            var calibration = new CalibrationData
                            {
                                CapVoltRange = FromBigEndianUshort(data, 0),
                                MeasuredVoltage = 0
                            };
                            CalibrationUpdated?.Invoke(this, calibration);
                            Log($"Received laser calibration: capVoltRange={calibration.CapVoltRange}V");
                        }
                        break;

                    case LcdTxCommand.lcdTxLsrDelays:
                        if (data.Length != FpgaCommandLengths.LSR_DELAYS)
                        {
                            Log($"Delays response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.LSR_DELAYS} bytes");
                            break;
                        }
                        if (data.Length >= 4)
                        {
                            ushort delay1 = FromBigEndianUshort(data, 0);
                            ushort delay2 = FromBigEndianUshort(data, 2);
                            Log($"Received laser fire setup delays: lsrDelay1={delay1}us, lsrDelay2={delay2}us");
                        }
                        break;

                    case LcdTxCommand.lcdTxDigitalIO:
                        if (data.Length != FpgaCommandLengths.DIGITAL_IO)
                        {
                            Log($"Digital I/O response invalid: Got {data.Length} bytes, expected {FpgaCommandLengths.DIGITAL_IO} bytes");
                            break;
                        }
                        if (data.Length >= 8)
                        {
                            var dioState = new DigitalIOState
                            {
                                InputStates = FromBigEndianUint(data, 0),
                                OutputStates = FromBigEndianUint(data, 4),
                                Timestamp = DateTime.Now
                            };
                            DigitalIOUpdated?.Invoke(this, dioState);
                            Log($"Received digital I/O state: InputStates=0x{dioState.InputStates:X8}, OutputStates=0x{dioState.OutputStates:X8}");
                        }
                        break;

                    default:
                        Log($"Received unhandled command: CMD=0x{commandByte:X2}, DataLen={dataLength}");
                        break;
                }
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
            Log("Reset packet parser to initial state");
        }

        private void HandleCOMException(System.Runtime.InteropServices.COMException comEx)
        {
            Log($"COM Exception (Error Code: {comEx.ErrorCode}): {comEx.Message}");
            switch (comEx.ErrorCode)
            {
                case -2147417842: Log("COM port disconnected unexpectedly: Attempting recovery"); break;
                case -2147467259: Log("Generic COM failure: Check cable and port configuration"); break;
                case -2147024891: Log("Access denied to COM port: Verify permissions"); break;
                case -2147024809: Log("Invalid COM parameter: Check serial port settings"); break;
                default: Log($"Unknown COM error: {comEx.ErrorCode}"); break;
            }
            if (IsConnected)
            {
                Log("Attempting recovery by resetting connection...");
                try { Disconnect(); }
                catch (Exception disconnectEx) { Log($"Disconnect error: {disconnectEx.Message}"); }
            }
        }

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
            string formattedMessage = $"{DateTime.Now:HH:mm:ss.fff} - {message}\n";
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
                Log("DeviceManager disposed successfully");
            }
            catch (Exception ex)
            {
                Log($"Error disposing DeviceManager: {ex.Message}");
            }
        }
    }
}
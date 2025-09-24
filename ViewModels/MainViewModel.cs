using KALD_Control.Models;
using KALD_Control.Services;
using Microsoft.Extensions.Logging;
using Microsoft.UI.Dispatching;
using Microsoft.UI.Xaml;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace KALD_Control.ViewModels
{
    public class MainViewModel : ObservableObject, IDisposable
    {
        private readonly DeviceManager _deviceManager;
        private readonly ILogger<MainViewModel> _logger;
        private readonly DispatcherQueue _dispatcherQueue;
        private bool _disposed = false;
        private string _connectionStatus = "Disconnected";
        private bool _isConnected = false;
        private bool _deviceReady = false;
        private string _selectedPort;
        private int _selectedBaudRate = Constants.DefaultBaudRate;
        private ushort _voltageSetpoint = 0;
        private StringBuilder _logText = new StringBuilder();
        private InterlockStatus _interlockStatus = new InterlockStatus();
        private DigitalIOState _digitalIO = new DigitalIOState();
        private bool _waveformEnabled = false;
        private DateTime _lastCommandTime = DateTime.MinValue;
        private readonly TimeSpan _commandDebounce = TimeSpan.FromMilliseconds(500);

        // Add missing properties
        private ushort _frequencySetpoint = 100;
        private ushort _pulseWidth = 100;
        private uint _totalShots = 100;
        private TriggerModeType _selectedTriggerMode = TriggerModeType.intTrigMode;
        private ShotModeType _selectedShotMode = ShotModeType.burstMode;
        private ShutterModeType _selectedShutterMode = ShutterModeType.autoMode;
        private ShutterStateType _selectedShutterState = ShutterStateType.shutterClosed;
        private bool _softStartEnabled = false;
        private ushort _idleSetpoint = 100;
        private uint _rampCount = 10;

        public ObservableCollection<string> AvailablePorts { get; } = new ObservableCollection<string>();
        public ObservableCollection<int> AvailableBaudRates { get; } = new ObservableCollection<int> { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
        public ObservableCollection<TriggerModeType> TriggerModes { get; } = new ObservableCollection<TriggerModeType>(Enum.GetValues(typeof(TriggerModeType)).Cast<TriggerModeType>());
        public ObservableCollection<ShotModeType> ShotModes { get; } = new ObservableCollection<ShotModeType>(Enum.GetValues(typeof(ShotModeType)).Cast<ShotModeType>());
        public ObservableCollection<ShutterModeType> ShutterModes { get; } = new ObservableCollection<ShutterModeType>(Enum.GetValues(typeof(ShutterModeType)).Cast<ShutterModeType>());
        public ObservableCollection<ShutterStateType> ShutterStates { get; } = new ObservableCollection<ShutterStateType>(Enum.GetValues(typeof(ShutterStateType)).Cast<ShutterStateType>());

        public DeviceData DeviceData
        {
            get => _deviceData;
            set => SetProperty(ref _deviceData, value);
        }
        private DeviceData _deviceData = new DeviceData();

        // Add missing properties with proper getters/setters
        public ushort FrequencySetpoint
        {
            get => _frequencySetpoint;
            set => SetProperty(ref _frequencySetpoint, value);
        }

        public ushort PulseWidth
        {
            get => _pulseWidth;
            set => SetProperty(ref _pulseWidth, value);
        }

        public uint TotalShots
        {
            get => _totalShots;
            set => SetProperty(ref _totalShots, value);
        }

        public TriggerModeType SelectedTriggerMode
        {
            get => _selectedTriggerMode;
            set => SetProperty(ref _selectedTriggerMode, value);
        }

        public ShotModeType SelectedShotMode
        {
            get => _selectedShotMode;
            set => SetProperty(ref _selectedShotMode, value);
        }

        public ShutterModeType SelectedShutterMode
        {
            get => _selectedShutterMode;
            set => SetProperty(ref _selectedShutterMode, value);
        }

        public ShutterStateType SelectedShutterState
        {
            get => _selectedShutterState;
            set => SetProperty(ref _selectedShutterState, value);
        }

        public bool SoftStartEnabled
        {
            get => _softStartEnabled;
            set => SetProperty(ref _softStartEnabled, value);
        }

        public ushort IdleSetpoint
        {
            get => _idleSetpoint;
            set => SetProperty(ref _idleSetpoint, value);
        }

        public uint RampCount
        {
            get => _rampCount;
            set => SetProperty(ref _rampCount, value);
        }

        public string ConnectionStatus
        {
            get => _connectionStatus;
            set => SetProperty(ref _connectionStatus, value);
        }

        public bool IsConnected
        {
            get => _isConnected;
            set => SetProperty(ref _isConnected, value);
        }

        public bool DeviceReady
        {
            get => _deviceReady;
            set => SetProperty(ref _deviceReady, value);
        }

        public string SelectedPort
        {
            get => _selectedPort;
            set => SetProperty(ref _selectedPort, value);
        }

        public int SelectedBaudRate
        {
            get => _selectedBaudRate;
            set => SetProperty(ref _selectedBaudRate, value);
        }

        public ushort VoltageSetpoint
        {
            get => _voltageSetpoint;
            set => SetProperty(ref _voltageSetpoint, value);
        }

        public string LogText
        {
            get => _logText.ToString();
            set
            {
                _logText.Clear();
                _logText.Append(value);
                OnPropertyChanged(nameof(LogText));
            }
        }

        public InterlockStatus InterlockStatus
        {
            get => _interlockStatus;
            set => SetProperty(ref _interlockStatus, value);
        }

        public DigitalIOState DigitalIO
        {
            get => _digitalIO;
            set => SetProperty(ref _digitalIO, value);
        }

        public bool WaveformEnabled
        {
            get => _waveformEnabled;
            set => SetProperty(ref _waveformEnabled, value);
        }

        // Commands - Add missing commands
        public ICommand ConnectCommand { get; }
        public ICommand DisconnectCommand { get; }
        public ICommand RefreshPortsCommand { get; }
        public ICommand ApplyPulseSettingsCommand { get; }
        public ICommand SendInterlockMaskCommand { get; }
        public ICommand SendLaserDelaysCommand { get; }
        public ICommand ApplyShutterSettingsCommand { get; }
        public ICommand ApplySoftStartCommand { get; }
        public ICommand RequestStatusCommand { get; }
        public ICommand ReadEnergyCommand { get; }
        public ICommand ReadTemperatureCommand { get; }
        public ICommand SystemInfoCommand { get; }
        public ICommand SystemResetCommand { get; }
        public ICommand RequestWaveformCommand { get; }
        public ICommand ClearLogsCommand { get; }
        public ICommand DebugTestCommand { get; }
        public ICommand StopCommand { get; }
        public ICommand ArmCommand { get; }
        public ICommand DisarmCommand { get; }
        public ICommand FireCommand { get; }
        public ICommand ApplySettingsCommand { get; }

        public MainViewModel(DeviceManager deviceManager, ILogger<MainViewModel> logger, DispatcherQueue dispatcherQueue)
        {
            _deviceManager = deviceManager ?? throw new ArgumentNullException(nameof(deviceManager));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
            _dispatcherQueue = dispatcherQueue ?? throw new ArgumentNullException(nameof(dispatcherQueue));

            // Initialize commands with all missing commands
            ConnectCommand = new RelayCommand(ExecuteConnect, () => !IsConnected && !string.IsNullOrEmpty(SelectedPort));
            DisconnectCommand = new RelayCommand(ExecuteDisconnect, () => IsConnected);
            RefreshPortsCommand = new RelayCommand(ExecuteRefreshPorts);
            ApplyPulseSettingsCommand = new RelayCommand(ExecuteApplyPulseSettings, () => DeviceReady && CanSendCommand());
            SendInterlockMaskCommand = new RelayCommand(ExecuteSendInterlockMask, () => DeviceReady && CanSendCommand());
            SendLaserDelaysCommand = new RelayCommand(ExecuteSendLaserDelays, () => DeviceReady && CanSendCommand());
            ApplyShutterSettingsCommand = new RelayCommand(ExecuteApplyShutterSettings, () => DeviceReady && CanSendCommand());
            ApplySoftStartCommand = new RelayCommand(ExecuteApplySoftStart, () => DeviceReady && CanSendCommand());
            RequestStatusCommand = new RelayCommand(ExecuteRequestStatus, () => DeviceReady && CanSendCommand());
            ReadEnergyCommand = new RelayCommand(ExecuteReadEnergy, () => DeviceReady && CanSendCommand());
            ReadTemperatureCommand = new RelayCommand(ExecuteReadTemperature, () => DeviceReady && CanSendCommand());
            SystemInfoCommand = new RelayCommand(ExecuteSystemInfo, () => DeviceReady && CanSendCommand());
            SystemResetCommand = new RelayCommand(ExecuteSystemReset, () => DeviceReady && CanSendCommand());
            RequestWaveformCommand = new RelayCommand(ExecuteRequestWaveform, () => DeviceReady && CanSendCommand());
            ClearLogsCommand = new RelayCommand(ExecuteClearLogs);
            DebugTestCommand = new RelayCommand(ExecuteDebugTest, () => DeviceReady && CanSendCommand());
            StopCommand = new RelayCommand(ExecuteStop, () => DeviceReady && CanSendCommand());

            // Add missing commands
            ArmCommand = new RelayCommand(ExecuteArm, () => DeviceReady && CanSendCommand());
            DisarmCommand = new RelayCommand(ExecuteDisarm, () => DeviceReady && CanSendCommand());
            FireCommand = new RelayCommand(ExecuteFire, () => DeviceReady && CanSendCommand());
            ApplySettingsCommand = new RelayCommand(ExecuteApplySettings, () => DeviceReady && CanSendCommand());

            // Subscribe to DeviceManager events
            _deviceManager.StateUpdated += OnStateUpdated;
            _deviceManager.RunStatusUpdated += OnRunStatusUpdated;
            _deviceManager.LogMessage += OnLogMessage;
            _deviceManager.IntStatusUpdated += OnIntStatusUpdated;
            _deviceManager.VoltsUpdated += OnVoltsUpdated;
            _deviceManager.ChargeStateUpdated += OnChargeStateUpdated;
            _deviceManager.ShutterConfigUpdated += OnShutterConfigUpdated;
            _deviceManager.SoftStartConfigUpdated += OnSoftStartConfigUpdated;
            _deviceManager.WaveformUpdated += OnWaveformUpdated;
            _deviceManager.CalibrationUpdated += OnCalibrationUpdated;
            _deviceManager.DigitalIOUpdated += OnDigitalIOUpdated;
            _deviceManager.PulseConfigUpdated += OnPulseConfigUpdated;
            _deviceManager.CommandError += OnCommandError;
            _deviceManager.DiscoveryResponseReceived += OnDiscoveryResponseReceived;
            _deviceManager.ShotCountUpdated += OnShotCountUpdated;
            _deviceManager.DeviceDataUpdated += OnDeviceDataUpdated;

            // Initialize available ports
            ExecuteRefreshPorts();
        }

        // Add missing command implementations
        private void ExecuteArm()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { (byte)LaserStateType.lsrArming });
            _logger.LogInformation("Sent arm command");
        }

        private void ExecuteDisarm()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { (byte)LaserStateType.lsrDisarming });
            _logger.LogInformation("Sent disarm command");
        }

        private void ExecuteFire()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { (byte)LaserStateType.lsrRunning });
            _logger.LogInformation("Sent fire command");
        }

        private void ExecuteApplySettings()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            var data = _deviceManager.ToBigEndian(VoltageSetpoint);
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrVolts, data);
            _logger.LogInformation($"Applied voltage setting: {VoltageSetpoint}V");
        }

        // Rest of the existing methods remain the same...
        private bool CanSendCommand()
        {
            return DateTime.Now - _lastCommandTime >= _commandDebounce;
        }

        private void ExecuteConnect()
        {
            try
            {
                _deviceManager.Connect(SelectedPort, SelectedBaudRate);
                IsConnected = _deviceManager.IsConnected;
                ConnectionStatus = IsConnected ? "Connected" : "Disconnected";
                _logger.LogInformation($"Connected to {SelectedPort} at {SelectedBaudRate} baud");
            }
            catch (Exception ex)
            {
                _logger.LogError($"Connection failed: {ex.Message}");
                ConnectionStatus = $"Connection failed: {ex.Message}";
            }
            UpdateCommandStates();
        }

        private void ExecuteDisconnect()
        {
            _deviceManager.Disconnect();
            IsConnected = false;
            DeviceReady = false;
            ConnectionStatus = "Disconnected";
            _logger.LogInformation("Disconnected from device");
            UpdateCommandStates();
        }

        private void ExecuteRefreshPorts()
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                AvailablePorts.Clear();
                foreach (var port in SerialPort.GetPortNames())
                {
                    AvailablePorts.Add(port);
                }
                if (AvailablePorts.Any() && string.IsNullOrEmpty(SelectedPort))
                {
                    SelectedPort = AvailablePorts.First();
                }
                _logger.LogInformation($"Refreshed ports: {string.Join(", ", AvailablePorts)}");
            });
        }

        private void ExecuteApplyPulseSettings()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;

            // Update DeviceData first
            if (DeviceData.PulseConfig == null)
                DeviceData.PulseConfig = new PulseConfig();

            DeviceData.PulseConfig.Frequency = FrequencySetpoint;
            DeviceData.PulseConfig.PulseWidth = PulseWidth;
            DeviceData.PulseConfig.ShotTotal = TotalShots;
            DeviceData.PulseConfig.ShotMode = SelectedShotMode;
            DeviceData.PulseConfig.TrigMode = SelectedTriggerMode;

            var pulseConfig = DeviceData.PulseConfig;
            var data = new List<byte>();
            data.AddRange(_deviceManager.ToBigEndian(pulseConfig.Frequency));
            data.AddRange(_deviceManager.ToBigEndian(pulseConfig.PulseWidth));
            data.AddRange(_deviceManager.ToBigEndian(pulseConfig.ShotTotal));
            data.AddRange(_deviceManager.ToBigEndian(pulseConfig.Delay1));
            data.AddRange(_deviceManager.ToBigEndian(pulseConfig.Delay2));
            data.Add((byte)pulseConfig.ShotMode);
            data.Add((byte)pulseConfig.TrigMode);
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrPulseConfig, data.ToArray());
            _logger.LogInformation("Sent pulse configuration");
        }

        private void ExecuteSendInterlockMask()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxIntMask, new byte[] { DigitalIO.GetInterlockMaskForLCD() });
            _logger.LogInformation($"Sent interlock mask: 0x{DigitalIO.GetInterlockMaskForLCD():X2}");
        }

        private void ExecuteSendLaserDelays()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            var pulseConfig = DeviceData.PulseConfig ?? new PulseConfig();
            var data = new List<byte>();
            data.AddRange(_deviceManager.ToBigEndian(pulseConfig.Delay1));
            data.AddRange(_deviceManager.ToBigEndian(pulseConfig.Delay2));
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrDelays, data.ToArray());
            _logger.LogInformation("Sent laser delays");
        }

        private void ExecuteApplyShutterSettings()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;

            // Update DeviceData first
            if (DeviceData.ShutterConfig == null)
                DeviceData.ShutterConfig = new ShutterConfig();

            DeviceData.ShutterConfig.ShutterMode = SelectedShutterMode;
            DeviceData.ShutterConfig.ShutterState = SelectedShutterState;

            var shutterConfig = DeviceData.ShutterConfig;
            var data = new byte[] { (byte)shutterConfig.ShutterMode, (byte)shutterConfig.ShutterState };
            _deviceManager.SendCommand(uiTxCommand.uiTxShutterConfig, data);
            _logger.LogInformation("Sent shutter configuration");
        }

        private void ExecuteApplySoftStart()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;

            // Update DeviceData first
            if (DeviceData.SoftStartConfig == null)
                DeviceData.SoftStartConfig = new SoftStartConfig();

            DeviceData.SoftStartConfig.Enable = SoftStartEnabled;
            DeviceData.SoftStartConfig.IdleSetpoint = IdleSetpoint;
            DeviceData.SoftStartConfig.RampCount = RampCount;

            var softStartConfig = DeviceData.SoftStartConfig;
            var data = new List<byte>();
            data.Add((byte)(softStartConfig.Enable ? 1 : 0));
            data.AddRange(_deviceManager.ToBigEndian(softStartConfig.IdleSetpoint));
            data.AddRange(_deviceManager.ToBigEndian(softStartConfig.RampCount));
            _deviceManager.SendCommand(uiTxCommand.uiTxSoftStartConfig, data.ToArray());
            _logger.LogInformation("Sent soft start configuration");
        }

        private void ExecuteRequestStatus()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { 0x00 });
            _logger.LogInformation("Requested laser status");
        }

        private void ExecuteReadEnergy()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { 0x00 });
            _logger.LogInformation("Requested energy reading");
        }

        private void ExecuteReadTemperature()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { 0x00 });
            _logger.LogInformation("Requested temperature reading");
        }

        private void ExecuteSystemInfo()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrCal, new byte[] { 0x00, 0x00 });
            _logger.LogInformation("Requested system info");
        }

        private void ExecuteSystemReset()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrChargeCancel, new byte[] { 0x00 });
            _logger.LogInformation("Sent system reset command");
        }

        private void ExecuteRequestWaveform()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxWaveState, new byte[] { (byte)(WaveformEnabled ? 1 : 0) });
            _logger.LogInformation("Requested waveform data");
        }

        private void ExecuteClearLogs()
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                LogText = "";
                _logger.LogInformation("Cleared logs");
            });
        }

        private void ExecuteDebugTest()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxNoCmd, new byte[] { 0x00 });
            _logger.LogInformation("Sent debug test command");
        }

        private void ExecuteStop()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrChargeCancel, new byte[] { 0x00 });
            _logger.LogInformation("Sent stop command");
        }

        // Event handlers remain the same...
        private void OnStateUpdated(object sender, LaserStateType state)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.LaserState = state;
                _logger.LogInformation($"Laser state updated: {state}");
            });
        }

        private void OnRunStatusUpdated(object sender, RunStatusData runStatus)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.RunStatus = runStatus;
                _logger.LogInformation($"Run status updated: ShotCount={runStatus.ShotCount}, State={runStatus.State}");
            });
        }

        private void OnLogMessage(object sender, string message)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                _logText.Append(message);
                OnPropertyChanged(nameof(LogText));
            });
        }

        private void OnIntStatusUpdated(object sender, byte status)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                InterlockStatus.UpdateFromByte(status);
                _logger.LogInformation($"Interlock status updated: 0x{status:X2}");
            });
        }

        private void OnVoltsUpdated(object sender, ushort volts)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ActualVoltage = volts;
                _logger.LogInformation($"Voltage updated: {volts}V");
            });
        }

        private void OnChargeStateUpdated(object sender, ChargeStateData chargeState)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ChargeState = chargeState;
                _logger.LogInformation($"Charge state updated: MeasuredVolts={chargeState.MeasuredVolts}, ChargeDone={chargeState.ChargeDone}");
            });
        }

        private void OnShutterConfigUpdated(object sender, ShutterConfig shutterConfig)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ShutterConfig = shutterConfig;
                _logger.LogInformation($"Shutter config updated: Mode={shutterConfig.ShutterMode}, State={shutterConfig.ShutterState}");
            });
        }

        private void OnSoftStartConfigUpdated(object sender, SoftStartConfig softStartConfig)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.SoftStartConfig = softStartConfig;
                _logger.LogInformation($"Soft start config updated: Enable={softStartConfig.Enable}, IdleSetpoint={softStartConfig.IdleSetpoint}");
            });
        }

        private void OnWaveformUpdated(object sender, WaveformData waveform)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.Waveform = waveform;
                _logger.LogInformation($"Waveform updated: CaptureTime={waveform.CaptureTime}");
            });
        }

        private void OnCalibrationUpdated(object sender, CalibrationData calibration)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.Calibration = calibration;
                _logger.LogInformation($"Calibration updated: ProductName={calibration.ProductName}, SerialNumber={calibration.SerialNumber}");
            });
        }

        private void OnDigitalIOUpdated(object sender, DigitalIOState digitalIO)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DigitalIO = digitalIO;
                _logger.LogInformation($"Digital I/O updated: InputStates=0x{digitalIO.InputStates:X8}, OutputStates=0x{digitalIO.OutputStates:X8}");
            });
        }

        private void OnPulseConfigUpdated(object sender, PulseConfig pulseConfig)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.PulseConfig = pulseConfig;
                _logger.LogInformation($"Pulse config updated: Frequency={pulseConfig.Frequency}, PulseWidth={pulseConfig.PulseWidth}");
            });
        }

        private void OnCommandError(object sender, (string Command, Exception Exception) error)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                _logger.LogError($"Command error: {error.Command}, {error.Exception.Message}");
                _logText.AppendLine($"ERROR: {error.Command} failed - {error.Exception.Message}");
                OnPropertyChanged(nameof(LogText));
            });
        }

        private void OnDiscoveryResponseReceived(object sender, bool success)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceReady = success;
                ConnectionStatus = success ? "Device Ready" : "Device Not Responding";
                _logger.LogInformation($"Discovery response: {success}");
                UpdateCommandStates();
            });
        }

        private void OnShotCountUpdated(object sender, uint shotCount)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ShotCount = shotCount;
                _logger.LogInformation($"Shot count updated: {shotCount}");
            });
        }

        private void OnDeviceDataUpdated(object sender, DeviceData deviceData)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData = deviceData;
                _logger.LogInformation("Device data updated");
            });
        }

        private void UpdateCommandStates()
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                ((RelayCommand)ConnectCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)DisconnectCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ApplyPulseSettingsCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)SendInterlockMaskCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)SendLaserDelaysCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ApplyShutterSettingsCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ApplySoftStartCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)RequestStatusCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ReadEnergyCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ReadTemperatureCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)SystemInfoCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)SystemResetCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)RequestWaveformCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ClearLogsCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)DebugTestCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)StopCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ArmCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)DisarmCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)FireCommand)?.RaiseCanExecuteChanged();
                ((RelayCommand)ApplySettingsCommand)?.RaiseCanExecuteChanged();
            });
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            _deviceManager.StateUpdated -= OnStateUpdated;
            _deviceManager.RunStatusUpdated -= OnRunStatusUpdated;
            _deviceManager.LogMessage -= OnLogMessage;
            _deviceManager.IntStatusUpdated -= OnIntStatusUpdated;
            _deviceManager.VoltsUpdated -= OnVoltsUpdated;
            _deviceManager.ChargeStateUpdated -= OnChargeStateUpdated;
            _deviceManager.ShutterConfigUpdated -= OnShutterConfigUpdated;
            _deviceManager.SoftStartConfigUpdated -= OnSoftStartConfigUpdated;
            _deviceManager.WaveformUpdated -= OnWaveformUpdated;
            _deviceManager.CalibrationUpdated -= OnCalibrationUpdated;
            _deviceManager.DigitalIOUpdated -= OnDigitalIOUpdated;
            _deviceManager.PulseConfigUpdated -= OnPulseConfigUpdated;
            _deviceManager.CommandError -= OnCommandError;
            _deviceManager.DiscoveryResponseReceived -= OnDiscoveryResponseReceived;
            _deviceManager.ShotCountUpdated -= OnShotCountUpdated;
            _deviceManager.DeviceDataUpdated -= OnDeviceDataUpdated;

            if (_deviceManager.IsConnected)
            {
                _deviceManager.Disconnect();
            }
            _logger.LogInformation("MainViewModel disposed");
        }
    }

    public class RelayCommand : ICommand
    {
        private readonly Action _execute;
        private readonly Func<bool> _canExecute;

        public event EventHandler CanExecuteChanged;

        public RelayCommand(Action execute, Func<bool> canExecute = null)
        {
            _execute = execute ?? throw new ArgumentNullException(nameof(execute));
            _canExecute = canExecute;
        }

        public bool CanExecute(object parameter) => _canExecute?.Invoke() ?? true;

        public void Execute(object parameter) => _execute();

        public void RaiseCanExecuteChanged()
        {
            CanExecuteChanged?.Invoke(this, EventArgs.Empty);
        }
    }
}
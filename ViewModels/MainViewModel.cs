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
        private string _selectedPort;
        private int _selectedBaudRate = Constants.DefaultBaudRate;
        private ushort _voltageSetpoint = 0;
        private StringBuilder _logText = new StringBuilder();
        private bool _waveformEnabled = false;
        private DateTime _lastCommandTime = DateTime.MinValue;
        private readonly TimeSpan _commandDebounce = TimeSpan.FromMilliseconds(500);

        // Interlock properties - properly exposed with property change notifications
        private InterlockClass _interlockObj;

        public InterlockClass InterlockObj
        {
            get => _interlockObj;
            set => SetProperty(ref _interlockObj, value);
        }

        private ushort _frequencySetpoint = 100;
        private ushort _pulseWidth = 100;
        private uint _totalShots = 100;
        private TriggerModeType _selectedTriggerMode = TriggerModeType.intTrigMode;
        private ShotModeType _selectedShotMode = ShotModeType.burstMode;
        private ShutterModeType _selectedShutterMode = ShutterModeType.autoMode;
        private ShutterStateType _selectedShutterState = ShutterStateType.shutterClosed;
        private ushort _setDelay1 = 0;
        private ushort _setDelay2 = 0;
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

        public ushort SetDelay1
        {
            get => _setDelay1;
            set => SetProperty(ref _setDelay1, value);
        }

        public ushort SetDelay2
        {
            get => _setDelay2;
            set => SetProperty(ref _setDelay2, value);
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

        public bool WaveformEnabled
        {
            get => _waveformEnabled;
            set => SetProperty(ref _waveformEnabled, value);
        }

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

            // Initialize interlock mask with reference to this viewmodel
            _interlockObj = new InterlockClass(this);

            ConnectCommand = new RelayCommand(ExecuteConnect, () => !IsConnected && !string.IsNullOrEmpty(SelectedPort));
            DisconnectCommand = new RelayCommand(ExecuteDisconnect, () => IsConnected);
            RefreshPortsCommand = new RelayCommand(ExecuteRefreshPorts);
            ApplyPulseSettingsCommand = new RelayCommand(ExecuteApplyPulseSettings, () => CanSendCommand());
            SendInterlockMaskCommand = new RelayCommand(ExecuteSendInterlockMask, () => CanSendCommand());
            SendLaserDelaysCommand = new RelayCommand(ExecuteSendLaserDelays, () => CanSendCommand());
            ApplyShutterSettingsCommand = new RelayCommand(ExecuteApplyShutterSettings, () => CanSendCommand());
            ApplySoftStartCommand = new RelayCommand(ExecuteApplySoftStart, () => CanSendCommand());
            RequestStatusCommand = new RelayCommand(ExecuteRequestStatus, () => CanSendCommand());
            ReadEnergyCommand = new RelayCommand(ExecuteReadEnergy, () => CanSendCommand());
            ReadTemperatureCommand = new RelayCommand(ExecuteReadTemperature, () => CanSendCommand());
            SystemInfoCommand = new RelayCommand(ExecuteSystemInfo, () => CanSendCommand());
            SystemResetCommand = new RelayCommand(ExecuteSystemReset, () => CanSendCommand());
            RequestWaveformCommand = new RelayCommand(ExecuteRequestWaveform, () => CanSendCommand());
            ClearLogsCommand = new RelayCommand(ExecuteClearLogs);
            DebugTestCommand = new RelayCommand(ExecuteDebugTest, () => CanSendCommand());
            StopCommand = new RelayCommand(ExecuteStop, () => CanSendCommand());
            ArmCommand = new RelayCommand(ExecuteArm, () => CanSendCommand());
            DisarmCommand = new RelayCommand(ExecuteDisarm, () => CanSendCommand());
            FireCommand = new RelayCommand(ExecuteFire, () => CanSendCommand());
            ApplySettingsCommand = new RelayCommand(ExecuteApplySettings, () => CanSendCommand());

            _deviceManager.StateUpdated += OnStateUpdated;
            _deviceManager.RunStatusUpdated += OnRunStatusUpdated;
            _deviceManager.LogMessage += OnLogMessage;
            _deviceManager.VoltsUpdated += OnVoltsUpdated;
            _deviceManager.ChargeStateUpdated += OnChargeStateUpdated;
            _deviceManager.ShutterConfigUpdated += OnShutterConfigUpdated;
            _deviceManager.SoftStartConfigUpdated += OnSoftStartConfigUpdated;
            _deviceManager.WaveformUpdated += OnWaveformUpdated;
            _deviceManager.CalibrationUpdated += OnCalibrationUpdated;
            _deviceManager.DigitalIOUpdated += OnDigitalIOUpdated;
            _deviceManager.PulseConfigUpdated += OnPulseConfigUpdated;
            _deviceManager.CommandError += OnCommandError;
            _deviceManager.ShotCountUpdated += OnShotCountUpdated;
            _deviceManager.DeviceDataUpdated += OnDeviceDataUpdated;

            _deviceManager.IntStatusUpdated += OnIntStatusUpdated;

            ExecuteRefreshPorts();
        }

        private void ExecuteArm()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendLsrState(LaserStateType.lsrArming);
            _logger.LogInformation("Sent arm command");
        }

        private void ExecuteDisarm()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendLsrState(LaserStateType.lsrDisarming);
            _logger.LogInformation("Sent disarm command");
        }

        private void ExecuteFire()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendLsrState(LaserStateType.lsrRunning);
            _logger.LogInformation("Sent fire command");
        }

        private void ExecuteApplySettings()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendLsrVolts(VoltageSetpoint);
            _logger.LogInformation($"Applied voltage setting: {VoltageSetpoint}V");
        }

        private void ExecuteSendInterlockMask()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendIntMask(_interlockObj.Mask);
            _logger.LogInformation($"Sent interlock mask: 0x{_interlockObj.Mask:X2}");
        }

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

            var pulseConfig = new PulseConfig
            {
                Frequency = FrequencySetpoint,
                PulseWidth = PulseWidth,
                ShotTotal = TotalShots,
                ShotMode = SelectedShotMode,
                TrigMode = SelectedTriggerMode,
                Delay1 = SetDelay1,
                Delay2 = SetDelay2
            };

            _deviceManager.SendLsrPulseConfig(pulseConfig);
            _logger.LogInformation("Sent pulse configuration");
        }

        private void ExecuteSendLaserDelays()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            ushort Delay1 = SetDelay1;
            ushort Delay2 = SetDelay2;
            _deviceManager.SendLsrDelays(Delay1, Delay2);
            _logger.LogInformation("Sent laser delays");
        }

        private void ExecuteApplyShutterSettings()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;

            var shutterConfig = new ShutterConfig
            {
                ShutterMode = SelectedShutterMode,
                ShutterState = SelectedShutterState
            };

            _deviceManager.SendShutterConfig(shutterConfig);
            _logger.LogInformation("Sent shutter configuration");
        }

        private void ExecuteApplySoftStart()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;

            var softStartConfig = new SoftStartConfig
            {
                Enable = SoftStartEnabled,
                IdleSetpoint = IdleSetpoint,
                RampCount = RampCount
            };

            _deviceManager.SendSoftStartConfig(softStartConfig);
            _logger.LogInformation("Sent soft start configuration");
        }

        private void ExecuteRequestStatus()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendLsrState(LaserStateType.lsrIdle);
            _logger.LogInformation("Requested laser status");
        }

        private void ExecuteReadEnergy()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            // Placeholder: Adjust to actual command if available
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { 0x00 });
            _logger.LogInformation("Requested energy reading");
        }

        private void ExecuteReadTemperature()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            // Placeholder: Adjust to actual command if available
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrState, new byte[] { 0x00 });
            _logger.LogInformation("Requested temperature reading");
        }

        private void ExecuteSystemInfo()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            // Placeholder: Adjust to actual command if available
            _deviceManager.SendCommand(uiTxCommand.uiTxLsrCal, new byte[] { 0x00, 0x00 });
            _logger.LogInformation("Requested system info");
        }

        private void ExecuteSystemReset()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            // Placeholder: Adjust to actual command if available
            _deviceManager.SendLsrChargeCancel();
            _logger.LogInformation("Sent system reset command");
        }

        private void ExecuteRequestWaveform()
        {
            if (!CanSendCommand()) return;
            _lastCommandTime = DateTime.Now;
            _deviceManager.SendWaveState(WaveformEnabled);
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

        public void ExecuteIntMaskUpdated()
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                _deviceManager.SendIntMask(_interlockObj.Mask);
                _logger.LogInformation($"Interlock mask updated: 0x{_interlockObj.Mask:X2}");
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
            _deviceManager.SendLsrChargeCancel();
            _logger.LogInformation("Sent stop command");
        }

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
                _interlockObj.Status = status;
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
            //_dispatcherQueue.TryEnqueue(() =>
            //{
            //    DigitalIO = digitalIO;
            //    _logger.LogInformation($"Digital I/O updated: InputStates=0x{digitalIO.InputStates:X8}, OutputStates=0x{digitalIO.OutputStates:X8}");
            //});
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
                try
                {
                    DeviceData = deviceData;
                    _logger.LogInformation("Device data updated");
                }
                catch (Exception ex)
                {
                    _logger.LogError($"Error updating device data: {ex.Message}");
                }
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
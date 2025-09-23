using System;
using System.Collections.ObjectModel;
using System.IO.Ports;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;
using KALD_Control.Models;
using KALD_Control.Services;
using Microsoft.Extensions.Logging;
using Microsoft.UI.Dispatching;

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
        private double _frequencySetpoint = 10.0;
        private ushort _pulseWidth = 200;
        private uint _totalShots = 100;
        private string _logText = string.Empty;
        private readonly StringBuilder _logBuilder = new StringBuilder();
        private InterlockStatus _interlockStatus = new InterlockStatus();
        private double _energy = 0;
        private TriggerModeType _selectedTriggerMode = TriggerModeType.intTrigMode;
        private ShotModeType _selectedShotMode = ShotModeType.burstMode;
        private ShutterModeType _selectedShutterMode = ShutterModeType.autoMode;
        private ShutterStateType _selectedShutterState = ShutterStateType.shutterClosed;
        private bool _softStartEnabled = true;
        private ushort _idleSetpoint = 100;
        private uint _rampCount = 10;
        private ushort _capVoltRange = 1000;
        private bool _waveformEnabled = false;
        private ushort _delay1 = 100;
        private ushort _delay2 = 200;
        private bool _isLogPanelExpanded = false;
        private DateTime _lastCommandTime = DateTime.MinValue;
        private readonly TimeSpan _commandDebounce = TimeSpan.FromMilliseconds(200);
        private ushort _measuredVoltage = 0;
        private bool _isCharging = false;
        private ushort _chargeVolts = 0;
        private ushort[] _waveform = new ushort[32];

        public ObservableCollection<string> AvailablePorts { get; } = new ObservableCollection<string>();
        public ObservableCollection<int> AvailableBaudRates { get; } = new ObservableCollection<int> { 9600, 19200, 38400, 57600, 115200, 230400, 460800 };
        public ObservableCollection<TriggerModeType> TriggerModes { get; } = new ObservableCollection<TriggerModeType> { TriggerModeType.intTrigMode, TriggerModeType.extTrigMode };
        public ObservableCollection<ShotModeType> ShotModes { get; } = new ObservableCollection<ShotModeType> { ShotModeType.burstMode, ShotModeType.contMode };
        public ObservableCollection<ShutterModeType> ShutterModes { get; } = new ObservableCollection<ShutterModeType> { ShutterModeType.autoMode, ShutterModeType.manualMode };
        public ObservableCollection<ShutterStateType> ShutterStates { get; } = new ObservableCollection<ShutterStateType> { ShutterStateType.shutterClosed, ShutterStateType.shutterOpened };
        public DeviceData DeviceData { get; private set; } = new DeviceData();

        public ICommand ConnectCommand { get; }
        public ICommand DisconnectCommand { get; }
        public ICommand SendPulseConfigCommand { get; }
        public ICommand SendShutterConfigCommand { get; }
        public ICommand SendSoftStartConfigCommand { get; }
        public ICommand SendInterlockMaskCommand { get; }
        public ICommand SendVoltageCommand { get; }
        public ICommand SendCalibrationCommand { get; }
        public ICommand ApplyDelaysCommand { get; }
        public ICommand SendWaveformStateCommand { get; }
        public ICommand SendChargeCancelCommand { get; }
        public ICommand RefreshPortsCommand { get; }
        public ICommand ClearLogsCommand { get; }
        public ICommand StopCommand { get; }
        public ICommand ArmCommand { get; }
        public ICommand DisarmCommand { get; }
        public ICommand FireCommand { get; }
        public ICommand ApplySettingsCommand { get; }
        public ICommand ApplyPulseSettingsCommand { get; }
        public ICommand ApplyShutterSettingsCommand { get; }
        public ICommand ApplySoftStartCommand { get; }
        public ICommand RequestWaveformCommand { get; }
        public ICommand ToggleLogPanelCommand { get; }

        public string ConnectionStatus
        {
            get => _connectionStatus;
            set => SetProperty(ref _connectionStatus, value);
        }

        public bool IsConnected
        {
            get => _isConnected;
            set
            {
                if (SetProperty(ref _isConnected, value))
                {
                    (ConnectCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (DisconnectCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (ArmCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (DisarmCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (FireCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (StopCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (ApplySettingsCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (ApplyPulseSettingsCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (ApplyShutterSettingsCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (ApplySoftStartCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (ApplyDelaysCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (RequestWaveformCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (SendInterlockMaskCommand as RelayCommand)?.RaiseCanExecuteChanged();
                }
            }
        }

        public bool DeviceReady
        {
            get => _deviceReady;
            set
            {
                if (SetProperty(ref _deviceReady, value))
                {
                    (ArmCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (FireCommand as RelayCommand)?.RaiseCanExecuteChanged();
                }
            }
        }

        public string SelectedPort
        {
            get => _selectedPort;
            set
            {
                if (SetProperty(ref _selectedPort, value))
                {
                    (ConnectCommand as RelayCommand)?.RaiseCanExecuteChanged();
                    (RefreshPortsCommand as RelayCommand)?.RaiseCanExecuteChanged();
                }
            }
        }

        public int SelectedBaudRate
        {
            get => _selectedBaudRate;
            set => SetProperty(ref _selectedBaudRate, value);
        }

        public ushort VoltageSetpoint
        {
            get => _voltageSetpoint;
            set
            {
                if (SetProperty(ref _voltageSetpoint, value))
                {
                    Task.Run(() => SendVoltageAsync());
                }
            }
        }

        public double FrequencySetpoint
        {
            get => _frequencySetpoint;
            set
            {
                if (SetProperty(ref _frequencySetpoint, value))
                {
                    Task.Run(() => SendPulseConfigAsync());
                }
            }
        }

        public ushort PulseWidth
        {
            get => _pulseWidth;
            set
            {
                if (SetProperty(ref _pulseWidth, value))
                {
                    Task.Run(() => SendPulseConfigAsync());
                }
            }
        }

        public uint TotalShots
        {
            get => _totalShots;
            set
            {
                if (SetProperty(ref _totalShots, value))
                {
                    Task.Run(() => SendPulseConfigAsync());
                }
            }
        }

        public string LogText
        {
            get => _logText;
            set => _dispatcherQueue.TryEnqueue(() => SetProperty(ref _logText, value));
        }

        public InterlockStatus InterlockStatus
        {
            get => _interlockStatus;
            set
            {
                if (SetProperty(ref _interlockStatus, value))
                {
                    Task.Run(() => SendInterlockMaskAsync());
                }
            }
        }

        public double Energy
        {
            get => _energy;
            set => SetProperty(ref _energy, value);
        }

        public TriggerModeType SelectedTriggerMode
        {
            get => _selectedTriggerMode;
            set
            {
                if (SetProperty(ref _selectedTriggerMode, value))
                {
                    Task.Run(() => SendPulseConfigAsync());
                }
            }
        }

        public ShotModeType SelectedShotMode
        {
            get => _selectedShotMode;
            set
            {
                if (SetProperty(ref _selectedShotMode, value))
                {
                    Task.Run(() => SendPulseConfigAsync());
                }
            }
        }

        public ShutterModeType SelectedShutterMode
        {
            get => _selectedShutterMode;
            set
            {
                if (SetProperty(ref _selectedShutterMode, value))
                {
                    Task.Run(() => SendShutterConfigAsync());
                }
            }
        }

        public ShutterStateType SelectedShutterState
        {
            get => _selectedShutterState;
            set
            {
                if (SetProperty(ref _selectedShutterState, value))
                {
                    Task.Run(() => SendShutterConfigAsync());
                }
            }
        }

        public bool SoftStartEnabled
        {
            get => _softStartEnabled;
            set
            {
                if (SetProperty(ref _softStartEnabled, value))
                {
                    Task.Run(() => SendSoftStartConfigAsync());
                }
            }
        }

        public ushort IdleSetpoint
        {
            get => _idleSetpoint;
            set
            {
                if (SetProperty(ref _idleSetpoint, value))
                {
                    Task.Run(() => SendSoftStartConfigAsync());
                }
            }
        }

        public uint RampCount
        {
            get => _rampCount;
            set
            {
                if (SetProperty(ref _rampCount, value))
                {
                    Task.Run(() => SendSoftStartConfigAsync());
                }
            }
        }

        public bool WaveformEnabled
        {
            get => _waveformEnabled;
            set
            {
                if (SetProperty(ref _waveformEnabled, value))
                {
                    Task.Run(() => SendWaveformStateAsync());
                }
            }
        }

        public ushort Delay1
        {
            get => _delay1;
            set
            {
                if (SetProperty(ref _delay1, value))
                {
                    Task.Run(() => SendPulseConfigAsync());
                }
            }
        }

        public ushort Delay2
        {
            get => _delay2;
            set
            {
                if (SetProperty(ref _delay2, value))
                {
                    Task.Run(() => SendPulseConfigAsync());
                }
            }
        }

        public bool IsLogPanelExpanded
        {
            get => _isLogPanelExpanded;
            set => SetProperty(ref _isLogPanelExpanded, value);
        }

        // Constructor
        public MainViewModel(DeviceManager deviceManager, ILoggerFactory loggerFactory, DispatcherQueue dispatcherQueue)
        {
            _deviceManager = deviceManager;
            _logger = loggerFactory.CreateLogger<MainViewModel>();
            _dispatcherQueue = dispatcherQueue;

            // Wire up events
            _deviceManager.StateUpdated += DeviceManager_StateUpdated;
            _deviceManager.LogMessage += DeviceManager_LogMessage;
            _deviceManager.CommandError += DeviceManager_CommandError;
            _deviceManager.DeviceDataUpdated += DeviceManager_DeviceDataUpdated;
            _deviceManager.InterlockStatusUpdated += DeviceManager_InterlockStatusUpdated;

            // Commands with CanExecute
            ConnectCommand = new RelayCommand(Connect, () => !IsConnected && !string.IsNullOrEmpty(SelectedPort));
            DisconnectCommand = new RelayCommand(Disconnect, () => IsConnected);
            StopCommand = new RelayCommand(ExecuteStopAsync, () => IsConnected);
            ArmCommand = new RelayCommand(() => SendStateAsync(LaserStateType.lsrArming), () => IsConnected && DeviceReady);
            DisarmCommand = new RelayCommand(() => SendStateAsync(LaserStateType.lsrDisarming), () => IsConnected);
            FireCommand = new RelayCommand(() => SendStateAsync(LaserStateType.lsrRunning), () => IsConnected && DeviceReady);
            RefreshPortsCommand = new RelayCommand(RefreshPorts, () => !IsConnected);
            ClearLogsCommand = new RelayCommand(ClearLogs);
            ToggleLogPanelCommand = new RelayCommand(() => IsLogPanelExpanded = !IsLogPanelExpanded);
            ApplySettingsCommand = new RelayCommand(() => Task.Run(() => SendVoltageAsync()), () => IsConnected);
            ApplyPulseSettingsCommand = new RelayCommand(() => Task.Run(() => SendPulseConfigAsync()), () => IsConnected);
            ApplyShutterSettingsCommand = new RelayCommand(() => Task.Run(() => SendShutterConfigAsync()), () => IsConnected);
            ApplySoftStartCommand = new RelayCommand(() => Task.Run(() => SendSoftStartConfigAsync()), () => IsConnected);
            ApplyDelaysCommand = new RelayCommand(() => Task.Run(() => SendDelaysAsync()), () => IsConnected);
            RequestWaveformCommand = new RelayCommand(() => Task.Run(() => _deviceManager.RequestWaveformAsync()), () => IsConnected);
            SendInterlockMaskCommand = new RelayCommand(() => Task.Run(() => SendInterlockMaskAsync()), () => IsConnected);
            SendVoltageCommand = new RelayCommand(() => Task.Run(() => SendVoltageAsync()), () => IsConnected);
            SendCalibrationCommand = new RelayCommand(() => Task.Run(() => SendCalibrationAsync()), () => IsConnected);
            SendWaveformStateCommand = new RelayCommand(() => Task.Run(() => SendWaveformStateAsync()), () => IsConnected);
            SendChargeCancelCommand = new RelayCommand(() => Task.Run(() => _deviceManager.SendChargeCancelAsync()), () => IsConnected);

            RefreshPorts();
        }

        // Methods
        public async void Connect()
        {
            try
            {
                await _deviceManager.ConnectAsync(SelectedPort, SelectedBaudRate);
                ConnectionStatus = "Connected";
                IsConnected = true;
                DeviceReady = true; // Update based on actual device state
                AppendToLog($"Connected to {SelectedPort} at {SelectedBaudRate} baud");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Connection failed");
                AppendToLog($"ERROR: Connection failed: {ex.Message}");
                ConnectionStatus = "Disconnected";
                IsConnected = false;
            }
        }

        public void Disconnect()
        {
            if (!IsConnected)
                return;
            try
            {
                _deviceManager.Disconnect();
                ConnectionStatus = "Disconnected";
                IsConnected = false;
                DeviceReady = false;
                AppendToLog("Disconnected from device");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Disconnection failed");
                AppendToLog($"ERROR: Disconnection failed: {ex.Message}");
            }
        }

        private async Task SendStateAsync(LaserStateType state)
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendStateCommandAsync(state);
                AppendToLog($"Sent state: {state}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, $"Failed to send state {state}");
                AppendToLog($"ERROR: Failed to send state {state}: {ex.Message}");
            }
        }

        private void ExecuteStopAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            Task.Run(async () =>
            {
                try
                {
                    await _deviceManager.SendStateCommandAsync(LaserStateType.lsrIdle);
                    await _deviceManager.SendChargeCancelAsync();
                    AppendToLog("Stop command executed");
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "Stop command failed");
                    AppendToLog($"ERROR: Stop command failed: {ex.Message}");
                }
            });
        }

        private async Task SendPulseConfigAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendPulseConfigAsync(TotalShots, PulseWidth, (ushort)(FrequencySetpoint * 10), Delay1, Delay2, SelectedTriggerMode, SelectedShotMode);
                AppendToLog($"Pulse config sent: Shots={TotalShots}, Width={PulseWidth}, Freq={FrequencySetpoint}, Delay1={Delay1}, Delay2={Delay2}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send pulse config");
                AppendToLog($"ERROR: Failed to send pulse config: {ex.Message}");
            }
        }

        private async Task SendVoltageAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendVoltageAsync(VoltageSetpoint);
                AppendToLog($"Voltage set to {VoltageSetpoint}V");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send voltage");
                AppendToLog($"ERROR: Failed to send voltage: {ex.Message}");
            }
        }

        private async Task SendInterlockMaskAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                // Convert InterlockStatus to a byte bitmask
                byte interlockMask = 0;
                interlockMask |= (byte)(InterlockStatus.ArmOK ? 1 << 0 : 0);
                interlockMask |= (byte)(InterlockStatus.RunOK ? 1 << 1 : 0);
                interlockMask |= (byte)(InterlockStatus.FlowOK ? 1 << 2 : 0);
                interlockMask |= (byte)(InterlockStatus.TempOK ? 1 << 3 : 0);
                interlockMask |= (byte)(InterlockStatus.DoorOK ? 1 << 4 : 0);
                interlockMask |= (byte)(InterlockStatus.CoverOK ? 1 << 5 : 0);
                interlockMask |= (byte)(InterlockStatus.DischargeTempOK ? 1 << 6 : 0);
                interlockMask |= (byte)(InterlockStatus.OverVoltageOK ? 1 << 7 : 0);

                await _deviceManager.SendInterlockMaskAsync(interlockMask);
                AppendToLog($"Interlock mask sent: {Convert.ToString(interlockMask, 2).PadLeft(8, '0')}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send interlock mask");
                AppendToLog($"ERROR: Failed to send interlock mask: {ex.Message}");
            }
        }

        private async Task SendShutterConfigAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendShutterConfigAsync(SelectedShutterMode, SelectedShutterState);
                AppendToLog($"Shutter config sent: Mode={SelectedShutterMode}, State={SelectedShutterState}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send shutter config");
                AppendToLog($"ERROR: Failed to send shutter config: {ex.Message}");
            }
        }

        private async Task SendSoftStartConfigAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendSoftStartConfigAsync(SoftStartEnabled, IdleSetpoint, RampCount);
                AppendToLog($"Soft start config sent: Enabled={SoftStartEnabled}, Idle={IdleSetpoint}, Ramp={RampCount}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send soft start config");
                AppendToLog($"ERROR: Failed to send soft start config: {ex.Message}");
            }
        }

        private async Task SendDelaysAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendDelaysAsync(Delay1, Delay2);
                AppendToLog($"Delays sent: Delay1={Delay1}, Delay2={Delay2}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send delays");
                AppendToLog($"ERROR: Failed to send delays: {ex.Message}");
            }
        }

        private async Task SendCalibrationAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendCalibrationAsync(_capVoltRange);
                AppendToLog($"Calibration sent with capVoltRange={_capVoltRange}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send calibration");
                AppendToLog($"ERROR: Failed to send calibration: {ex.Message}");
            }
        }

        private async Task SendWaveformStateAsync()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
                return;
            _lastCommandTime = DateTime.Now;
            try
            {
                await _deviceManager.SendWaveformStateAsync(WaveformEnabled);
                AppendToLog($"Waveform state sent: {WaveformEnabled}");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to send waveform state");
                AppendToLog($"ERROR: Failed to send waveform state: {ex.Message}");
            }
        }

        private void RefreshPorts()
        {
            AvailablePorts.Clear();
            foreach (var port in SerialPort.GetPortNames())
                AvailablePorts.Add(port);
            AppendToLog($"Refreshed available ports: {AvailablePorts.Count} found");
        }

        private void ClearLogs()
        {
            _logBuilder.Clear();
            LogText = string.Empty;
            AppendToLog("Logs cleared");
        }

        // Helper method to safely append to log from any thread
        private void AppendToLog(string message)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                _logBuilder.AppendLine($"[{DateTime.Now:HH:mm:ss}] {message}");
                LogText = _logBuilder.ToString();
            });
        }

        // Event handlers
        private void DeviceManager_StateUpdated(object sender, LaserStateType e)
        {
            AppendToLog($"Device state updated: {e}");
        }

        private void DeviceManager_LogMessage(object sender, string e)
        {
            AppendToLog($"Device: {e.Trim()}");
        }

        private void DeviceManager_CommandError(object sender, (string Command, Exception Exception) e)
        {
            AppendToLog($"ERROR in {e.Command}: {e.Exception.Message}");
        }

        private void DeviceManager_DeviceDataUpdated(object sender, DeviceData e)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData = e;
                OnPropertyChanged(nameof(DeviceData));
            });
        }

        private void DeviceManager_InterlockStatusUpdated(object sender, InterlockStatus e)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                InterlockStatus = e;
                OnPropertyChanged(nameof(InterlockStatus));
            });
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                _deviceManager.StateUpdated -= DeviceManager_StateUpdated;
                _deviceManager.LogMessage -= DeviceManager_LogMessage;
                _deviceManager.CommandError -= DeviceManager_CommandError;
                _deviceManager.DeviceDataUpdated -= DeviceManager_DeviceDataUpdated;
                _deviceManager.InterlockStatusUpdated -= DeviceManager_InterlockStatusUpdated;

                _deviceManager?.Dispose();
                _disposed = true;
                AppendToLog("MainViewModel disposed");
            }
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
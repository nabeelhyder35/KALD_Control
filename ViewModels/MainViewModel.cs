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
        private double _frequencySetpoint = 10.0;
        private ushort _pulseWidth = 200;
        private uint _totalShots = 100;
        private StringBuilder _logText = new StringBuilder();
        private InterlockStatus _interlockStatus = new InterlockStatus();
        private DigitalIOState _digitalIO = new DigitalIOState();
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
        private DateTime _lastCommandTime = DateTime.MinValue;
        private readonly TimeSpan _commandDebounce = TimeSpan.FromMilliseconds(500);

        public ObservableCollection<string> AvailablePorts { get; } = new ObservableCollection<string>();
        public ObservableCollection<int> AvailableBaudRates { get; } = new ObservableCollection<int> { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
        public ObservableCollection<TriggerModeType> TriggerModes { get; } = new ObservableCollection<TriggerModeType>(Enum.GetValues(typeof(TriggerModeType)).Cast<TriggerModeType>());
        public ObservableCollection<ShotModeType> ShotModes { get; } = new ObservableCollection<ShotModeType>(Enum.GetValues(typeof(ShotModeType)).Cast<ShotModeType>());
        public ObservableCollection<ShutterModeType> ShutterModes { get; } = new ObservableCollection<ShutterModeType>(Enum.GetValues(typeof(ShutterModeType)).Cast<ShutterModeType>());
        public ObservableCollection<ShutterStateType> ShutterStates { get; } = new ObservableCollection<ShutterStateType>(Enum.GetValues(typeof(ShutterStateType)).Cast<ShutterStateType>());

        public DeviceData DeviceData { get; private set; } = new DeviceData();

        public MainViewModel(DeviceManager deviceManager, ILogger<MainViewModel> logger, DispatcherQueue dispatcherQueue)
        {
            _deviceManager = deviceManager ?? throw new ArgumentNullException(nameof(deviceManager));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
            _dispatcherQueue = dispatcherQueue ?? throw new ArgumentNullException(nameof(dispatcherQueue));

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

            ConnectCommand = new RelayCommand(Connect, () => !_isConnected);
            DisconnectCommand = new RelayCommand(Disconnect, () => _isConnected);
            RefreshPortsCommand = new RelayCommand(RefreshPorts, () => true);
            ArmCommand = new RelayCommand(() => _deviceManager.SendStateCommand(LaserStateType.lsrArming), CanExecuteDeviceCommand);
            DisarmCommand = new RelayCommand(() => _deviceManager.SendStateCommand(LaserStateType.lsrDisarming), CanExecuteDeviceCommand);
            FireCommand = new RelayCommand(() => _deviceManager.SendStateCommand(LaserStateType.lsrRunning), () => CanExecuteDeviceCommand() && DeviceData.SystemState == LaserStateType.lsrArmed);
            StopCommand = new RelayCommand(() => _deviceManager.SendStateCommand(LaserStateType.lsrPausing), CanExecuteDeviceCommand);
            ApplySettingsCommand = new RelayCommand(ApplyVoltage, CanExecuteDeviceCommand);
            ApplyPulseSettingsCommand = new RelayCommand(ApplyPulseSettings, CanExecuteDeviceCommand);
            SendInterlockMaskCommand = new RelayCommand(SendInterlockMask, CanExecuteDeviceCommand);
            SendLaserDelaysCommand = new RelayCommand(SendLaserDelays, CanExecuteDeviceCommand);
            ApplyShutterSettingsCommand = new RelayCommand(ApplyShutterSettings, CanExecuteDeviceCommand);
            ApplySoftStartCommand = new RelayCommand(ApplySoftStart, CanExecuteDeviceCommand);
            RequestWaveformCommand = new RelayCommand(() => _deviceManager.RequestWaveformData(), () => CanExecuteDeviceCommand() && _waveformEnabled);
            ClearLogsCommand = new RelayCommand(() => LogText = "", () => true);
            DebugTestCommand = new RelayCommand(() => _logger.LogInformation("Debug test executed"), CanExecuteDeviceCommand); // 
            RefreshPorts();
        }

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
                    ConnectionStatus = value ? (_deviceReady ? "Connected" : "Connecting...") : "Disconnected";
                    NotifyCommandsCanExecuteChanged();
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
                    ConnectionStatus = value ? "Connected" : (_isConnected ? "Discovery Failed" : "Disconnected");
                    NotifyCommandsCanExecuteChanged();
                }
            }
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

        public double FrequencySetpoint
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

        public string LogText
        {
            get => _logText.ToString();
            set => SetProperty(ref _logText, new StringBuilder(value));
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

        public double Energy
        {
            get => _energy;
            set => SetProperty(ref _energy, value);
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

        public ushort CapVoltRange
        {
            get => _capVoltRange;
            set => SetProperty(ref _capVoltRange, value);
        }

        public bool WaveformEnabled
        {
            get => _waveformEnabled;
            set
            {
                if (SetProperty(ref _waveformEnabled, value))
                {
                    _deviceManager.SendWaveformState(value);
                }
            }
        }

        public ushort Delay1
        {
            get => _delay1;
            set => SetProperty(ref _delay1, value);
        }

        public ushort Delay2
        {
            get => _delay2;
            set => SetProperty(ref _delay2, value);
        }

        public ICommand ConnectCommand { get; }
        public ICommand DisconnectCommand { get; }
        public ICommand RefreshPortsCommand { get; }
        public ICommand ArmCommand { get; }
        public ICommand DisarmCommand { get; }
        public ICommand FireCommand { get; }
        public ICommand StopCommand { get; }
        public ICommand ApplySettingsCommand { get; }
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

        private void Connect()
        {
            try
            {
                _deviceManager.Connect(_selectedPort, _selectedBaudRate);
                IsConnected = true;
            }
            catch (Exception ex)
            {
                IsConnected = false;
                LogText += $"Connection failed: {ex.Message}\n";
                _logger.LogError(ex, "Connection failed");
            }
        }

        private void Disconnect()
        {
            _deviceManager.Disconnect();
            IsConnected = false;
            DeviceReady = false;
        }

        private void RefreshPorts()
        {
            AvailablePorts.Clear();
            foreach (var port in SerialPort.GetPortNames())
            {
                AvailablePorts.Add(port);
            }
            if (AvailablePorts.Any() && string.IsNullOrEmpty(_selectedPort))
            {
                SelectedPort = AvailablePorts.First();
            }
        }

        private void ApplyVoltage()
        {
            if (!CanSendCommand()) return;
            _deviceManager.SendVoltage(_voltageSetpoint);
            _lastCommandTime = DateTime.Now;
        }

        private void ApplyPulseSettings()
        {
            if (!CanSendCommand()) return;
            var config = new PulseConfig
            {
                Frequency = (ushort)_frequencySetpoint,
                PulseWidth = _pulseWidth,
                ShotTotal = _totalShots,
                Delay1 = _delay1,
                Delay2 = _delay2,
                ShotMode = _selectedShotMode,
                TrigMode = _selectedTriggerMode
            };
            _deviceManager.SendPulseConfig(config);
            _lastCommandTime = DateTime.Now;
        }

        private void SendInterlockMask()
        {
            if (!CanSendCommand()) return;
            _deviceManager.SendInterlockMask(_interlockStatus.Mask);
            _lastCommandTime = DateTime.Now;
        }

        private void SendLaserDelays()
        {
            if (!CanSendCommand()) return;
            _deviceManager.SendLaserDelays(_delay1, _delay2);
            _lastCommandTime = DateTime.Now;
        }

        private void ApplyShutterSettings()
        {
            if (!CanSendCommand()) return;
            var config = new ShutterConfig
            {
                ShutterMode = _selectedShutterMode,
                ShutterState = _selectedShutterState
            };
            _deviceManager.SendShutterConfig(config);
            _lastCommandTime = DateTime.Now;
        }

        private void ApplySoftStart()
        {
            if (!CanSendCommand()) return;
            var config = new SoftStartConfig
            {
                Enable = _softStartEnabled,
                IdleSetpoint = _idleSetpoint,
                RampCount = _rampCount
            };
            _deviceManager.SendSoftStartConfig(config);
            _lastCommandTime = DateTime.Now;
        }

        private bool CanSendCommand()
        {
            if (DateTime.Now - _lastCommandTime < _commandDebounce)
            {
                _logger.LogWarning("Command debounced");
                return false;
            }
            return _isConnected && _deviceReady;
        }

        private bool CanExecuteDeviceCommand() => _isConnected && _deviceReady;

        private void OnStateUpdated(object sender, LaserStateType state)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.SystemState = state;
                NotifyCommandsCanExecuteChanged();
            });
        }

        private void OnRunStatusUpdated(object sender, RunStatusData status)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.RunStatus = status;
                Energy = status.Energy;
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
                _interlockStatus.UpdateFromByte(status);
                OnPropertyChanged(nameof(InterlockStatus));
            });
        }

        private void OnVoltsUpdated(object sender, ushort volts)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ActualVoltage = volts;
            });
        }

        private void OnChargeStateUpdated(object sender, ChargeStateData state)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ChargeState = state;
            });
        }

        private void OnShutterConfigUpdated(object sender, ShutterConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                SelectedShutterMode = config.ShutterMode;
                SelectedShutterState = config.ShutterState;
            });
        }

        private void OnSoftStartConfigUpdated(object sender, SoftStartConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                SoftStartEnabled = config.Enable;
                IdleSetpoint = config.IdleSetpoint;
                RampCount = config.RampCount;
            });
        }

        private void OnWaveformUpdated(object sender, WaveformData waveform)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.Waveform = waveform;
            });
        }

        private void OnCalibrationUpdated(object sender, CalibrationData calibration)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.Calibration = calibration;
                CapVoltRange = calibration.CapVoltRange;
            });
        }

        private void OnDigitalIOUpdated(object sender, DigitalIOState dio)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DigitalIO = dio;
            });
        }

        private void OnPulseConfigUpdated(object sender, PulseConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                FrequencySetpoint = config.Frequency;
                PulseWidth = config.PulseWidth;
                TotalShots = config.ShotTotal;
                Delay1 = config.Delay1;
                Delay2 = config.Delay2;
                SelectedTriggerMode = config.TrigMode;
                SelectedShotMode = config.ShotMode;
            });
        }

        private void OnCommandError(object sender, (string Command, Exception Exception) e)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                _logText.AppendLine($"Error in {e.Command}: {e.Exception.Message}");
                OnPropertyChanged(nameof(LogText));
            });
        }

        private void OnDiscoveryResponseReceived(object sender, bool success)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceReady = success;
                ConnectionStatus = success ? "Connected" : (_isConnected ? "Discovery Failed" : "Disconnected");
                NotifyCommandsCanExecuteChanged();
            });
        }

        private void OnShotCountUpdated(object sender, uint shotCount)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ShotCount = shotCount;
            });
        }

        private void NotifyCommandsCanExecuteChanged()
        {
            ((RelayCommand)ConnectCommand)?.RaiseCanExecuteChanged();
            ((RelayCommand)DisconnectCommand)?.RaiseCanExecuteChanged();
            ((RelayCommand)RefreshPortsCommand)?.RaiseCanExecuteChanged();
            ((RelayCommand)ArmCommand)?.RaiseCanExecuteChanged();
            ((RelayCommand)DisarmCommand)?.RaiseCanExecuteChanged();
            ((RelayCommand)FireCommand)?.RaiseCanExecuteChanged();
            ((RelayCommand)StopCommand)?.RaiseCanExecuteChanged();
            ((RelayCommand)ApplySettingsCommand)?.RaiseCanExecuteChanged();
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

            if (_deviceManager.IsConnected)
            {
                _deviceManager.Disconnect();
            }
        }
    }

    public class ObservableObject : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        protected void OnPropertyChanged([System.Runtime.CompilerServices.CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        protected bool SetProperty<T>(ref T field, T value, [System.Runtime.CompilerServices.CallerMemberName] string propertyName = null)
        {
            if (EqualityComparer<T>.Default.Equals(field, value)) return false;
            field = value;
            OnPropertyChanged(propertyName);
            return true;
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
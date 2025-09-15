using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
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
        private string _selectedPort;
        private int _selectedBaudRate = 115200;
        private ushort _voltageSetpoint = 0;
        private double _frequencySetpoint = 10.0;
        private ushort _pulseWidth = 200;
        private uint _totalShots = 100;
        private string _logText = "";
        private InterlockStatus _interlockStatus = new InterlockStatus();
        private double _energy = 0;
        private string _selectedTriggerMode = "Internal";
        private string _selectedShotMode = "Single";
        private string _selectedShutterMode = "Auto";
        private string _selectedShutterState = "Closed";
        private bool _softStartEnabled = true;
        private ushort _idleSetpoint = 100;
        private uint _rampCount = 10;
        private ushort _capVoltRange = 1000;
        private bool _waveformEnabled = false;
        private ushort _delay1 = 100;
        private ushort _delay2 = 200;

        public ObservableCollection<string> AvailablePorts { get; } = new ObservableCollection<string>();
        public ObservableCollection<int> AvailableBaudRates { get; } = new ObservableCollection<int>
        {
            9600, 19200, 38400, 57600, 115200
        };
        public ObservableCollection<string> TriggerModes { get; } = new ObservableCollection<string> { "Internal", "External" };
        public ObservableCollection<string> ShotModes { get; } = new ObservableCollection<string> { "Single", "Burst", "Continuous" };
        public ObservableCollection<string> ShutterModes { get; } = new ObservableCollection<string> { "Manual", "Auto" };
        public ObservableCollection<string> ShutterStates { get; } = new ObservableCollection<string> { "Closed", "Open" };

        public DeviceData DeviceData { get; } = new DeviceData();

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
                    RefreshCommands();
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
                    RefreshCommands();
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

        public double Energy
        {
            get => _energy;
            set => SetProperty(ref _energy, value);
        }

        public string SelectedTriggerMode
        {
            get => _selectedTriggerMode;
            set => SetProperty(ref _selectedTriggerMode, value);
        }

        public string SelectedShotMode
        {
            get => _selectedShotMode;
            set => SetProperty(ref _selectedShotMode, value);
        }

        public string SelectedShutterMode
        {
            get => _selectedShutterMode;
            set => SetProperty(ref _selectedShutterMode, value);
        }

        public string SelectedShutterState
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
            set => SetProperty(ref _waveformEnabled, value);
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

        public string LogText
        {
            get => _logText;
            set => SetProperty(ref _logText, value);
        }

        public InterlockStatus InterlockStatus
        {
            get => _interlockStatus;
            set => SetProperty(ref _interlockStatus, value);
        }

        // Commands
        public ICommand ConnectCommand { get; private set; }
        public ICommand DisconnectCommand { get; private set; }
        public ICommand RefreshPortsCommand { get; private set; }
        public ICommand ArmCommand { get; private set; }
        public ICommand DisarmCommand { get; private set; }
        public ICommand FireCommand { get; private set; }
        public ICommand StopCommand { get; private set; }
        public ICommand ApplySettingsCommand { get; private set; }
        public ICommand ApplyPulseSettingsCommand { get; private set; }
        public ICommand ApplyShutterSettingsCommand { get; private set; }
        public ICommand ApplySoftStartCommand { get; private set; }
        public ICommand ApplyCalibrationCommand { get; private set; }
        public ICommand SendInterlockMaskCommand { get; private set; }
        public ICommand RequestStatusCommand { get; private set; }
        public ICommand RequestWaveformCommand { get; private set; }
        public ICommand SendChargeCancelCommand { get; private set; }
        public ICommand ToggleWaveformCommand { get; private set; }
        public ICommand SendLaserDelaysCommand { get; private set; }
        public ICommand SystemResetCommand { get; private set; }

        public MainViewModel(DeviceManager deviceManager, ILogger<MainViewModel> logger)
        {
            _deviceManager = deviceManager;
            _logger = logger;
            _dispatcherQueue = DispatcherQueue.GetForCurrentThread();

            VoltageSetpoint = 0;

            InitializeCommands();
            SubscribeToDeviceEvents();
            RefreshAvailablePorts();
        }

        private void InitializeCommands()
        {
            ConnectCommand = new RelayCommand(Connect, CanConnect);
            DisconnectCommand = new RelayCommand(Disconnect, CanDisconnect);
            RefreshPortsCommand = new RelayCommand(RefreshAvailablePorts);
            ArmCommand = new RelayCommand(Arm, CanArm);
            DisarmCommand = new RelayCommand(Disarm, CanDisarm);
            FireCommand = new RelayCommand(Fire, CanFire);
            StopCommand = new RelayCommand(Stop, CanStop);
            ApplySettingsCommand = new RelayCommand(ApplySettings, CanApplySettings);
            ApplyPulseSettingsCommand = new RelayCommand(ApplyPulseSettings, CanApplyPulseSettings);
            ApplyShutterSettingsCommand = new RelayCommand(ApplyShutterSettings, CanApplyShutterSettings);
            ApplySoftStartCommand = new RelayCommand(ApplySoftStart, CanApplySoftStart);
            ApplyCalibrationCommand = new RelayCommand(ApplyCalibration, CanApplyCalibration);
            SendInterlockMaskCommand = new RelayCommand(SendInterlockMask, CanSendInterlockMask);
            RequestStatusCommand = new RelayCommand(RequestStatus, CanRequestStatus);
            RequestWaveformCommand = new RelayCommand(RequestWaveform, CanRequestWaveform);
            SendChargeCancelCommand = new RelayCommand(SendChargeCancel, CanSendChargeCancel);
            ToggleWaveformCommand = new RelayCommand(ToggleWaveform, CanToggleWaveform);
            SendLaserDelaysCommand = new RelayCommand(SendLaserDelays, CanSendLaserDelays);
            SystemResetCommand = new RelayCommand(SystemReset, CanSystemReset);
        }

        private void SubscribeToDeviceEvents()
        {
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
        }

        private void OnLogMessage(object sender, string message)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                LogText += message + Environment.NewLine;
            });
        }

        private void OnStateUpdated(object sender, LaserStateType state)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.SystemState = state;
                RefreshCommands();
            });
        }

        private void OnRunStatusUpdated(object sender, RunStatusData status)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.RunStatus = status;
                DeviceData.ShotCount = status.ShotCount;
                DeviceData.SystemState = status.State;
                DeviceData.Energy = status.Energy;
                DeviceData.Power = status.Power;
                DeviceData.Current = status.Current;
                DeviceData.VDroop = status.VDroop;

                Energy = status.Energy;
                RefreshCommands();
            });
        }

        private void OnIntStatusUpdated(object sender, byte status)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.RawStatus = status;
                InterlockStatus.RawStatus = status;
            });
        }

        private void OnVoltsUpdated(object sender, ushort voltage)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ActualVoltage = voltage;
            });
        }

        private void OnChargeStateUpdated(object sender, ChargeStateData chargeState)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ChargeState = chargeState;
                DeviceData.ActualVoltage = chargeState.MeasuredVolts;
            });
        }

        private void OnShutterConfigUpdated(object sender, ShutterConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ShutterConfig = config;
            });
        }

        private void OnSoftStartConfigUpdated(object sender, SoftStartConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.SoftStartConfig = config;
            });
        }

        private void OnWaveformUpdated(object sender, WaveformData waveform)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.WaveformData = waveform;
                LogText += $"Waveform data received: {waveform.Samples.Length} samples{Environment.NewLine}";
            });
        }

        private void OnCalibrationUpdated(object sender, CalibrationData calibration)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                LogText += $"Calibration updated: Range={calibration.CapVoltRange}, Measured={calibration.MeasuredVoltage}{Environment.NewLine}";
            });
        }

        private void OnDigitalIOUpdated(object sender, DigitalIOState dioState)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                LogText += $"Digital I/O updated: Inputs=0x{dioState.InputStates:X8}, Outputs=0x{dioState.OutputStates:X8}{Environment.NewLine}";
            });
        }

        private void OnPulseConfigUpdated(object sender, PulseConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.CurrentPulseConfig = config;
            });
        }

        private void RefreshCommands()
        {
            _dispatcherQueue.TryEnqueue(() =>
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
                (ApplyCalibrationCommand as RelayCommand)?.RaiseCanExecuteChanged();
                (SendInterlockMaskCommand as RelayCommand)?.RaiseCanExecuteChanged();
                (RequestStatusCommand as RelayCommand)?.RaiseCanExecuteChanged();
                (RequestWaveformCommand as RelayCommand)?.RaiseCanExecuteChanged();
                (SendChargeCancelCommand as RelayCommand)?.RaiseCanExecuteChanged();
                (ToggleWaveformCommand as RelayCommand)?.RaiseCanExecuteChanged();
                (SendLaserDelaysCommand as RelayCommand)?.RaiseCanExecuteChanged();
                (SystemResetCommand as RelayCommand)?.RaiseCanExecuteChanged();
            });
        }

        private void RefreshAvailablePorts()
        {
            try
            {
                var ports = SerialPort.GetPortNames().OrderBy(p => p).ToArray();

                _dispatcherQueue.TryEnqueue(() =>
                {
                    AvailablePorts.Clear();
                    foreach (var port in ports)
                    {
                        AvailablePorts.Add(port);
                    }

                    if (string.IsNullOrEmpty(SelectedPort) && AvailablePorts.Count > 0)
                    {
                        SelectedPort = AvailablePorts[0];
                    }

                    RefreshCommands();
                });
            }
            catch (Exception ex)
            {
                _dispatcherQueue.TryEnqueue(() =>
                {
                    LogText += $"Error refreshing ports: {ex.Message}{Environment.NewLine}";
                });
            }
        }

        private bool CanConnect() => !IsConnected && !string.IsNullOrEmpty(SelectedPort);
        private void Connect()
        {
            try
            {
                LogText += $"Attempting to connect to {SelectedPort} at {SelectedBaudRate} baud...{Environment.NewLine}";

                if (_deviceManager.IsConnected)
                {
                    _deviceManager.Disconnect();
                }

                _deviceManager.Connect(SelectedPort, SelectedBaudRate);
                IsConnected = true;
                ConnectionStatus = $"Connected to {SelectedPort}";
                LogText += $"Connected to {SelectedPort} at {SelectedBaudRate} baud{Environment.NewLine}";

                // Request initial status
                System.Threading.Thread.Sleep(100);
                _deviceManager.RequestSystemStatus();
            }
            catch (Exception ex)
            {
                ConnectionStatus = $"Connection failed: {ex.Message}";
                LogText += $"Connection failed: {ex.Message}{Environment.NewLine}";
                IsConnected = false;
            }
        }

        private bool CanDisconnect() => IsConnected;
        private void Disconnect()
        {
            try
            {
                _deviceManager.Disconnect();
                IsConnected = false;
                ConnectionStatus = "Disconnected";
                LogText += $"Disconnected{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                ConnectionStatus = $"Disconnection failed: {ex.Message}";
                LogText += $"Disconnection failed: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanArm() => IsConnected && DeviceData.SystemState == LaserStateType.lsrIdle;
        private void Arm()
        {
            try
            {
                _deviceManager.SendVoltageSetpoint(VoltageSetpoint);
                _deviceManager.SendStateCommand(LaserStateType.lsrArming);
                LogText += $"Arming command sent, voltage set to {VoltageSetpoint}V{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Arming failed: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanDisarm() => IsConnected && (DeviceData.SystemState == LaserStateType.lsrArmed || DeviceData.SystemState == LaserStateType.lsrPaused);
        private void Disarm()
        {
            try
            {
                _deviceManager.SendStateCommand(LaserStateType.lsrDisarming);
                LogText += "Disarming command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Disarming failed: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanFire() => IsConnected && DeviceData.SystemState == LaserStateType.lsrArmed;
        private void Fire()
        {
            try
            {
                _deviceManager.SendStateCommand(LaserStateType.lsrRunning);
                LogText += "Fire command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Fire command failed: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanStop() => IsConnected && (DeviceData.SystemState == LaserStateType.lsrRunning || DeviceData.SystemState == LaserStateType.lsrArming);
        private void Stop()
        {
            try
            {
                _deviceManager.SendStateCommand(LaserStateType.lsrPausing);
                LogText += "Stop command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Stop command failed: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplySettings() => IsConnected;
        private void ApplySettings()
        {
            try
            {
                _deviceManager.SendVoltageSetpoint(VoltageSetpoint);
                LogText += $"Voltage set to {VoltageSetpoint}V{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply settings: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplyPulseSettings() => IsConnected;
        private void ApplyPulseSettings()
        {
            try
            {
                var config = new PulseConfig
                {
                    Frequency = (ushort)(FrequencySetpoint * 10),
                    PulseWidth = PulseWidth,
                    ShotTotal = TotalShots,
                    Delay1 = Delay1,
                    Delay2 = Delay2,
                    ShotMode = SelectedShotMode switch
                    {
                        "Single" => ShotModeType.Single,
                        "Burst" => ShotModeType.Burst,
                        "Continuous" => ShotModeType.Continuous,
                        _ => ShotModeType.Single
                    },
                    TrigMode = SelectedTriggerMode switch
                    {
                        "Internal" => TriggerModeType.Internal,
                        "External" => TriggerModeType.External,
                        _ => TriggerModeType.Internal
                    }
                };

                _deviceManager.SendPulseConfig(config);
                LogText += $"Pulse settings applied: {FrequencySetpoint}Hz, {PulseWidth}μs, {TotalShots} shots{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply pulse settings: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplyShutterSettings() => IsConnected;
        private void ApplyShutterSettings()
        {
            try
            {
                var config = new ShutterConfig
                {
                    ShutterMode = SelectedShutterMode switch
                    {
                        "Manual" => ShutterModeType.Manual,
                        "Auto" => ShutterModeType.Auto,
                        _ => ShutterModeType.Auto
                    },
                    ShutterState = SelectedShutterState switch
                    {
                        "Closed" => ShutterStateType.Closed,
                        "Open" => ShutterStateType.Open,
                        _ => ShutterStateType.Closed
                    }
                };

                _deviceManager.SendShutterConfig(config);
                LogText += $"Shutter settings applied: Mode={SelectedShutterMode}, State={SelectedShutterState}{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply shutter settings: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplySoftStart() => IsConnected;
        private void ApplySoftStart()
        {
            try
            {
                var config = new SoftStartConfig
                {
                    Enable = SoftStartEnabled,
                    IdleSetpoint = IdleSetpoint,
                    RampCount = RampCount
                };

                _deviceManager.SendSoftStartConfig(config);
                LogText += $"SoftStart applied: Enable={SoftStartEnabled}, IdleV={IdleSetpoint}, RampShots={RampCount}{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply SoftStart: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplyCalibration() => IsConnected;
        private void ApplyCalibration()
        {
            try
            {
                _deviceManager.SendLaserCalibration(CapVoltRange);
                LogText += $"Calibration applied: CapVoltRange={CapVoltRange}{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply calibration: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanSendInterlockMask() => IsConnected;
        private void SendInterlockMask()
        {
            try
            {
                byte mask = 0xFF;
                _deviceManager.SendInterlockMask(mask);
                LogText += $"Interlock mask sent: 0x{mask:X2}{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to send interlock mask: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanRequestStatus() => IsConnected;
        private void RequestStatus()
        {
            try
            {
                _deviceManager.RequestSystemStatus();
                _deviceManager.RequestInterlockStatus();
                LogText += "Status requested" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to request status: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanRequestWaveform() => IsConnected;
        private void RequestWaveform()
        {
            try
            {
                _deviceManager.RequestWaveformData();
                LogText += "Waveform data requested" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to request waveform: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanSendChargeCancel() => IsConnected;
        private void SendChargeCancel()
        {
            try
            {
                _deviceManager.SendChargeCancel();
                LogText += "Charge cancel sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to send charge cancel: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanToggleWaveform() => IsConnected;
        private void ToggleWaveform()
        {
            try
            {
                _deviceManager.SendWaveformState(WaveformEnabled);
                LogText += $"Waveform {(WaveformEnabled ? "enabled" : "disabled")}{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to toggle waveform: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanSendLaserDelays() => IsConnected;
        private void SendLaserDelays()
        {
            try
            {
                _deviceManager.SendLaserDelays(Delay1, Delay2);
                LogText += $"Laser delays sent: {Delay1}μs, {Delay2}μs" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to send laser delays: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanSystemReset() => IsConnected;
        private void SystemReset()
        {
            try
            {
                _deviceManager.SendSystemReset();
                LogText += "System reset command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to send system reset: {ex.Message}{Environment.NewLine}";
            }
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

            if (_deviceManager.IsConnected)
            {
                _deviceManager.Disconnect();
            }
        }
    }

    public class RelayCommand : ICommand
    {
        private readonly Action _execute;
        private readonly Func<bool> _canExecute;

        public RelayCommand(Action execute, Func<bool> canExecute = null)
        {
            _execute = execute ?? throw new ArgumentNullException(nameof(execute));
            _canExecute = canExecute;
        }

        public bool CanExecute(object parameter) => _canExecute?.Invoke() ?? true;

        public void Execute(object parameter) => _execute();

        public event EventHandler CanExecuteChanged;

        public void RaiseCanExecuteChanged()
        {
            CanExecuteChanged?.Invoke(this, EventArgs.Empty);
        }
    }
}
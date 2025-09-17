using KALD_Control.Models;
using KALD_Control.Services;
using Microsoft.Extensions.Logging;
using Microsoft.UI.Dispatching;
using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
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
        private int _selectedBaudRate = 9600; // Match FPGA firmware
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
        private DateTime _lastCommandTime = DateTime.MinValue;
        private readonly TimeSpan _commandDebounce = TimeSpan.FromMilliseconds(500);

        public ObservableCollection<string> AvailablePorts { get; } = new ObservableCollection<string>();
        public ObservableCollection<int> AvailableBaudRates { get; } = new ObservableCollection<int> { 9600 }; // Only FPGA-compatible baud rate
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
        public ICommand TestScreenCommand { get; private set; }

        public MainViewModel(DeviceManager deviceManager, ILogger<MainViewModel> logger)
        {
            _deviceManager = deviceManager;
            _logger = logger;
            _dispatcherQueue = DispatcherQueue.GetForCurrentThread();

            // Initialize commands
            ConnectCommand = new RelayCommand(Connect, CanConnect);
            DisconnectCommand = new RelayCommand(Disconnect, CanDisconnect);
            RefreshPortsCommand = new RelayCommand(RefreshPorts);
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
            TestScreenCommand = new RelayCommand(TestScreenUpdates, CanTestScreen);

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

            // Initialize available ports
            RefreshPorts();
        }

        // Add these methods for testing
        private bool CanTestScreen() => true; // Always enabled for testing

        private void TestScreenUpdates()
        {
            try
            {
                // Test basic state updates
                DeviceData.SystemState = LaserStateType.lsrArmed;

                // Test metrics
                DeviceData.ActualVoltage = 1250;
                DeviceData.ShotCount = 25;
                DeviceData.Energy = 500;
                DeviceData.Power = 1000;
                DeviceData.Current = 50;
                DeviceData.VDroop = 20;

                // Test interlock status
                DeviceData.InterlockStatus.RawStatus = 0x00; // All OK
                InterlockStatus = DeviceData.InterlockStatus;

                // Test different state
                DeviceData.SystemState = LaserStateType.lsrRunning;

                LogText += "Test data injected successfully" + Environment.NewLine;

                // Refresh UI
                RefreshCommands();
                OnPropertyChanged(nameof(DeviceData));
            }
            catch (Exception ex)
            {
                LogText += $"Test failed: {ex.Message}{Environment.NewLine}";
            }
        }

        private void RefreshPorts()
        {
            AvailablePorts.Clear();
            foreach (var port in SerialPort.GetPortNames())
            {
                AvailablePorts.Add(port);
            }
            SelectedPort = AvailablePorts.FirstOrDefault();
        }

        private bool CanConnect() => !IsConnected && !string.IsNullOrEmpty(SelectedPort);
        private void Connect()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.Connect(SelectedPort, SelectedBaudRate);
                IsConnected = true;
                ConnectionStatus = "Connected";
                LogText += $"Connected to {SelectedPort} at {SelectedBaudRate} baud{Environment.NewLine}";

                // Request initial data on connect
                _deviceManager.SendDiscoveryCommand();
                _deviceManager.RequestSystemStatus();
            }
            catch (Exception ex)
            {
                LogText += $"Failed to connect: {ex.Message}{Environment.NewLine}";
            }
            RefreshCommands();
        }

        private bool CanDisconnect() => IsConnected;
        private void Disconnect()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.Disconnect();
                IsConnected = false;
                ConnectionStatus = "Disconnected";
                LogText += "Disconnected" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to disconnect: {ex.Message}{Environment.NewLine}";
            }
            RefreshCommands();
        }

        private bool CanArm() => IsConnected;
        private void Arm()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.SendStateCommand(LaserStateType.lsrArmed);
                LogText += "Arm command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to arm: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanDisarm() => IsConnected;
        private void Disarm()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.SendStateCommand(LaserStateType.lsrDisarming);
                LogText += "Disarm command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to disarm: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanFire() => IsConnected;
        private void Fire()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.SendStateCommand(LaserStateType.lsrRunning);
                LogText += "Fire command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to fire: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanStop() => IsConnected;
        private void Stop()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.SendStateCommand(LaserStateType.lsrPausing);
                LogText += "Stop command sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to stop: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplySettings() => IsConnected;
        private void ApplySettings()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.SendVolts(VoltageSetpoint);
                LogText += $"Voltage setpoint sent: {VoltageSetpoint}V{Environment.NewLine}";
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply settings: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplyPulseSettings() => IsConnected;
        private void ApplyPulseSettings()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                var config = new PulseConfig
                {
                    Frequency = (ushort)(FrequencySetpoint * 10),
                    PulseWidth = PulseWidth,
                    ShotTotal = TotalShots,
                    Delay1 = Delay1,
                    Delay2 = Delay2,
                    ShotMode = SelectedShotMode switch { "Single" => ShotModeType.Single, "Burst" => ShotModeType.Burst, _ => ShotModeType.Continuous },
                    TrigMode = SelectedTriggerMode == "Internal" ? TriggerModeType.Internal : TriggerModeType.External
                };
                _deviceManager.SendPulseConfig(config);
                LogText += "Pulse settings applied" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply pulse settings: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplyShutterSettings() => IsConnected;
        private void ApplyShutterSettings()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                var config = new ShutterConfig
                {
                    ShutterMode = SelectedShutterMode == "Manual" ? ShutterModeType.Manual : ShutterModeType.Auto,
                    ShutterState = SelectedShutterState == "Closed" ? ShutterStateType.Closed : ShutterStateType.Open
                };
                _deviceManager.SendShutterConfig(config);
                LogText += "Shutter settings applied" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply shutter settings: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplySoftStart() => IsConnected;
        private void ApplySoftStart()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                var config = new SoftStartConfig
                {
                    Enable = SoftStartEnabled,
                    IdleSetpoint = IdleSetpoint,
                    RampCount = RampCount
                };
                _deviceManager.SendSoftStartConfig(config);
                LogText += "Soft start settings applied" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply soft start: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanApplyCalibration() => IsConnected;
        private void ApplyCalibration()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.SendLaserCalibration(CapVoltRange);
                LogText += "Calibration applied" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to apply calibration: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanSendInterlockMask() => IsConnected;
        private void SendInterlockMask()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.SendInterlockMask(0x00); // Example: reset mask
                LogText += "Interlock mask sent" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to send interlock mask: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanRequestStatus() => IsConnected;
        private void RequestStatus()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                _deviceManager.RequestSystemStatus();
                LogText += "System status requested" + Environment.NewLine;
            }
            catch (Exception ex)
            {
                LogText += $"Failed to request status: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanRequestWaveform() => IsConnected;
        private void RequestWaveform()
        {
            if (!CanExecuteCommand()) return;
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
            if (!CanExecuteCommand()) return;
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
            if (!CanExecuteCommand()) return;
            try
            {
                if (DeviceData.WaveformData == null || WaveformEnabled != DeviceData.WaveformEnabled)
                {
                    _deviceManager.SendWaveformState(WaveformEnabled);
                    DeviceData.WaveformEnabled = WaveformEnabled;
                    LogText += $"Waveform {(WaveformEnabled ? "enabled" : "disabled")}{Environment.NewLine}";
                }
                else
                {
                    LogText += $"Waveform state unchanged: {(WaveformEnabled ? "enabled" : "disabled")}{Environment.NewLine}";
                }
            }
            catch (Exception ex)
            {
                LogText += $"Failed to toggle waveform: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanSendLaserDelays() => IsConnected;
        private void SendLaserDelays()
        {
            if (!CanExecuteCommand()) return;
            try
            {
                if (DeviceData.CurrentPulseConfig == null ||
                    Delay1 != DeviceData.CurrentPulseConfig.Delay1 ||
                    Delay2 != DeviceData.CurrentPulseConfig.Delay2)
                {
                    _deviceManager.SendLaserDelays(Delay1, Delay2);
                    DeviceData.CurrentPulseConfig.Delay1 = Delay1;
                    DeviceData.CurrentPulseConfig.Delay2 = Delay2;
                    LogText += $"Laser delays sent: Delay1={Delay1}us, Delay2={Delay2}us{Environment.NewLine}";
                }
                else
                {
                    LogText += $"Laser delays unchanged: Delay1={Delay1}us, Delay2={Delay2}us{Environment.NewLine}";
                }
            }
            catch (Exception ex)
            {
                LogText += $"Failed to send laser delays: {ex.Message}{Environment.NewLine}";
            }
        }

        private bool CanSystemReset() => IsConnected;
        private void SystemReset()
        {
            if (!CanExecuteCommand()) return;
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

        private bool CanExecuteCommand()
        {
            var now = DateTime.Now;
            if ((now - _lastCommandTime) < _commandDebounce)
            {
                LogText += "Command ignored: Too soon after previous command" + Environment.NewLine;
                return false;
            }
            _lastCommandTime = now;
            return true;
        }

        private void OnStateUpdated(object sender, LaserStateType state)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.SystemState = state;
                LogText += $"System state updated: {state}{Environment.NewLine}";
                RefreshCommands();
            });
        }

        private void OnRunStatusUpdated(object sender, RunStatusData status)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.RunStatus = status;
                Energy = status.Energy / 1000.0; // Assuming energy in mJ
                LogText += $"Run status updated: ShotCount={status.ShotCount}, State={status.State}, Energy={Energy:F3}J{Environment.NewLine}";
                RefreshCommands();
            });
        }

        private void OnLogMessage(object sender, string message)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                LogText += message;
            });
        }

        private void OnIntStatusUpdated(object sender, byte intStatus)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.InterlockStatus.RawStatus = intStatus;
                InterlockStatus = DeviceData.InterlockStatus;
                LogText += $"Interlock status updated: 0x{intStatus:X2}{Environment.NewLine}";
            });
        }

        private void OnVoltsUpdated(object sender, ushort volts)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.SetVoltage = volts;
                VoltageSetpoint = volts;  // Sync to UI-bound property
                LogText += $"Voltage updated: {volts}V{Environment.NewLine}";
            });
        }

        private void OnChargeStateUpdated(object sender, ChargeStateData state)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ChargeState.MeasuredVolts = state.MeasuredVolts;
                DeviceData.ChargeState.ChargeDone = state.ChargeDone;
                DeviceData.ActualVoltage = state.MeasuredVolts;  // Sync actual voltage
                LogText += $"Charge state updated: MeasuredVolts={state.MeasuredVolts}V, ChargeDone={state.ChargeDone}{Environment.NewLine}";
            });
        }

        private void OnShutterConfigUpdated(object sender, ShutterConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.ShutterConfig = config;
                SelectedShutterMode = config.ShutterMode == ShutterModeType.Manual ? "Manual" : "Auto";
                SelectedShutterState = config.ShutterState == ShutterStateType.Closed ? "Closed" : "Open";
                LogText += $"Shutter config updated: Mode={config.ShutterMode}, State={config.ShutterState}{Environment.NewLine}";
            });
        }

        private void OnSoftStartConfigUpdated(object sender, SoftStartConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.SoftStartConfig = config;
                SoftStartEnabled = config.Enable;
                IdleSetpoint = config.IdleSetpoint;
                RampCount = config.RampCount;
                LogText += $"Soft start config updated: Enable={config.Enable}, IdleSetpoint={config.IdleSetpoint}V, RampCount={config.RampCount}{Environment.NewLine}";
            });
        }

        private void OnWaveformUpdated(object sender, WaveformData waveform)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.WaveformData = waveform;
                WaveformEnabled = true;  // Assume receipt implies enabled
                LogText += $"Waveform updated: {waveform.Samples.Length} samples at {waveform.CaptureTime}{Environment.NewLine}";
            });
        }

        private void OnCalibrationUpdated(object sender, CalibrationData calibration)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.CalibrationData = calibration;
                CapVoltRange = calibration.CapVoltRange;
                LogText += $"Calibration updated: CapVoltRange={calibration.CapVoltRange}V{Environment.NewLine}";
            });
        }

        private void OnDigitalIOUpdated(object sender, DigitalIOState dio)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.DigitalIO = dio;
                LogText += $"Digital I/O updated: InputStates=0x{dio.InputStates:X8}, OutputStates=0x{dio.OutputStates:X8}{Environment.NewLine}";
            });
        }

        private void OnPulseConfigUpdated(object sender, PulseConfig config)
        {
            _dispatcherQueue.TryEnqueue(() =>
            {
                DeviceData.CurrentPulseConfig = config;
                FrequencySetpoint = config.Frequency / 10.0;
                PulseWidth = config.PulseWidth;
                TotalShots = config.ShotTotal;
                SelectedTriggerMode = config.TrigMode == TriggerModeType.Internal ? "Internal" : "External";
                SelectedShotMode = config.ShotMode == ShotModeType.Single ? "Single" : (config.ShotMode == ShotModeType.Burst ? "Burst" : "Continuous");
                Delay1 = config.Delay1;
                Delay2 = config.Delay2;
                LogText += $"Pulse config updated: Frequency={config.Frequency / 10.0:F1}Hz, PulseWidth={config.PulseWidth}us, Shots={config.ShotTotal}, Delay1={config.Delay1}us, Delay2={config.Delay2}us, ShotMode={config.ShotMode}, TrigMode={config.TrigMode}{Environment.NewLine}";
            });
        }

        private void RefreshCommands()
        {
            ((RelayCommand)ConnectCommand).RaiseCanExecuteChanged();
            ((RelayCommand)DisconnectCommand).RaiseCanExecuteChanged();
            ((RelayCommand)ArmCommand).RaiseCanExecuteChanged();
            ((RelayCommand)DisarmCommand).RaiseCanExecuteChanged();
            ((RelayCommand)FireCommand).RaiseCanExecuteChanged();
            ((RelayCommand)StopCommand).RaiseCanExecuteChanged();
            ((RelayCommand)ApplySettingsCommand).RaiseCanExecuteChanged();
            ((RelayCommand)ApplyPulseSettingsCommand).RaiseCanExecuteChanged();
            ((RelayCommand)ApplyShutterSettingsCommand).RaiseCanExecuteChanged();
            ((RelayCommand)ApplySoftStartCommand).RaiseCanExecuteChanged();
            ((RelayCommand)ApplyCalibrationCommand).RaiseCanExecuteChanged();
            ((RelayCommand)SendInterlockMaskCommand).RaiseCanExecuteChanged();
            ((RelayCommand)RequestStatusCommand).RaiseCanExecuteChanged();
            ((RelayCommand)RequestWaveformCommand).RaiseCanExecuteChanged();
            ((RelayCommand)SendChargeCancelCommand).RaiseCanExecuteChanged();
            ((RelayCommand)ToggleWaveformCommand).RaiseCanExecuteChanged();
            ((RelayCommand)SendLaserDelaysCommand).RaiseCanExecuteChanged();
            ((RelayCommand)SystemResetCommand).RaiseCanExecuteChanged();
            ((RelayCommand)TestScreenCommand).RaiseCanExecuteChanged();
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
using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace KALD_Control.Models
{
    public class DeviceData : ObservableObject
    {
        private LaserStateType _systemState;
        public LaserStateType SystemState
        {
            get => _systemState;
            set => SetProperty(ref _systemState, value);
        }

        private ushort _setVoltage;
        public ushort SetVoltage
        {
            get => _setVoltage;
            set => SetProperty(ref _setVoltage, value);
        }

        private ushort _actualVoltage;
        public ushort ActualVoltage
        {
            get => _actualVoltage;
            set => SetProperty(ref _actualVoltage, value);
        }

        private ushort _pulseEnergy;
        public ushort PulseEnergy
        {
            get => _pulseEnergy;
            set => SetProperty(ref _pulseEnergy, value);
        }

        private ushort _power;
        public ushort Power
        {
            get => _power;
            set => SetProperty(ref _power, value);
        }

        private ushort _current;
        public ushort Current
        {
            get => _current;
            set => SetProperty(ref _current, value);
        }

        private ushort _vDroop;
        public ushort VDroop
        {
            get => _vDroop;
            set => SetProperty(ref _vDroop, value);
        }

        private uint _shotCount;
        public uint ShotCount
        {
            get => _shotCount;
            set => SetProperty(ref _shotCount, value);
        }

        private ushort _energy;
        public ushort Energy
        {
            get => _energy;
            set => SetProperty(ref _energy, value);
        }

        private ushort _frequency;
        public ushort Frequency
        {
            get => _frequency;
            set => SetProperty(ref _frequency, value);
        }

        private ushort _pulseWidth;
        public ushort PulseWidth
        {
            get => _pulseWidth;
            set => SetProperty(ref _pulseWidth, value);
        }

        private InterlockStatus _interlockStatus = new InterlockStatus();
        public InterlockStatus InterlockStatus
        {
            get => _interlockStatus;
            set => SetProperty(ref _interlockStatus, value);
        }

        private PulseConfig _currentPulseConfig = new PulseConfig();
        public PulseConfig CurrentPulseConfig
        {
            get => _currentPulseConfig;
            set => SetProperty(ref _currentPulseConfig, value);
        }

        private RunStatusData _runStatus = new RunStatusData();
        public RunStatusData RunStatus
        {
            get => _runStatus;
            set => SetProperty(ref _runStatus, value);
        }

        private ChargeStateData _chargeState = new ChargeStateData();
        public ChargeStateData ChargeState
        {
            get => _chargeState;
            set => SetProperty(ref _chargeState, value);
        }

        private ShutterConfig _shutterConfig = new ShutterConfig();
        public ShutterConfig ShutterConfig
        {
            get => _shutterConfig;
            set => SetProperty(ref _shutterConfig, value);
        }

        private SoftStartConfig _softStartConfig = new SoftStartConfig();
        public SoftStartConfig SoftStartConfig
        {
            get => _softStartConfig;
            set => SetProperty(ref _softStartConfig, value);
        }

        private WaveformData _waveformData = new WaveformData();
        public WaveformData WaveformData
        {
            get => _waveformData;
            set => SetProperty(ref _waveformData, value);
        }

        private CalibrationData _calibrationData = new CalibrationData();
        public CalibrationData CalibrationData
        {
            get => _calibrationData;
            set => SetProperty(ref _calibrationData, value);
        }

        private DigitalIOState _digitalIO = new DigitalIOState();
        public DigitalIOState DigitalIO
        {
            get => _digitalIO;
            set => SetProperty(ref _digitalIO, value);
        }

        private bool _waveformEnabled;
        public bool WaveformEnabled
        {
            get => _waveformEnabled;
            set => SetProperty(ref _waveformEnabled, value);
        }

        private byte _rawStatus;
        public byte RawStatus
        {
            get => _rawStatus;
            set
            {
                if (SetProperty(ref _rawStatus, value))
                {
                    InterlockStatus.RawStatus = value;
                }
            }
        }
    }

    public class PulseConfig : ObservableObject
    {
        private ushort _frequency;
        public ushort Frequency
        {
            get => _frequency;
            set => SetProperty(ref _frequency, value);
        }

        private ushort _pulseWidth;
        public ushort PulseWidth
        {
            get => _pulseWidth;
            set => SetProperty(ref _pulseWidth, value);
        }

        private uint _shotTotal;
        public uint ShotTotal
        {
            get => _shotTotal;
            set => SetProperty(ref _shotTotal, value);
        }

        private ushort _delay1;
        public ushort Delay1
        {
            get => _delay1;
            set => SetProperty(ref _delay1, value);
        }

        private ushort _delay2;
        public ushort Delay2
        {
            get => _delay2;
            set => SetProperty(ref _delay2, value);
        }

        private ShotModeType _shotMode;
        public ShotModeType ShotMode
        {
            get => _shotMode;
            set => SetProperty(ref _shotMode, value);
        }

        private TriggerModeType _trigMode;
        public TriggerModeType TrigMode
        {
            get => _trigMode;
            set => SetProperty(ref _trigMode, value);
        }
    }

    public class ShutterConfig : ObservableObject
    {
        private ShutterModeType _shutterMode;
        public ShutterModeType ShutterMode
        {
            get => _shutterMode;
            set => SetProperty(ref _shutterMode, value);
        }

        private ShutterStateType _shutterState;
        public ShutterStateType ShutterState
        {
            get => _shutterState;
            set => SetProperty(ref _shutterState, value);
        }
    }

    public class SoftStartConfig : ObservableObject
    {
        private bool _enable;
        public bool Enable
        {
            get => _enable;
            set => SetProperty(ref _enable, value);
        }

        private ushort _idleSetpoint;
        public ushort IdleSetpoint
        {
            get => _idleSetpoint;
            set => SetProperty(ref _idleSetpoint, value);
        }

        private uint _rampCount;
        public uint RampCount
        {
            get => _rampCount;
            set => SetProperty(ref _rampCount, value);
        }
    }

    public class RunStatusData : ObservableObject
    {
        private uint _shotCount;
        public uint ShotCount
        {
            get => _shotCount;
            set => SetProperty(ref _shotCount, value);
        }

        private LaserStateType _state;
        public LaserStateType State
        {
            get => _state;
            set => SetProperty(ref _state, value);
        }

        private ushort _energy;
        public ushort Energy
        {
            get => _energy;
            set => SetProperty(ref _energy, value);
        }

        private ushort _power;
        public ushort Power
        {
            get => _power;
            set => SetProperty(ref _power, value);
        }

        private ushort _current;
        public ushort Current
        {
            get => _current;
            set => SetProperty(ref _current, value);
        }

        private ushort _vDroop;
        public ushort VDroop
        {
            get => _vDroop;
            set => SetProperty(ref _vDroop, value);
        }
    }

    public class ChargeStateData : ObservableObject
    {
        private ushort _measuredVolts;
        public ushort MeasuredVolts
        {
            get => _measuredVolts;
            set => SetProperty(ref _measuredVolts, value);
        }

        private bool _chargeDone;
        public bool ChargeDone
        {
            get => _chargeDone;
            set => SetProperty(ref _chargeDone, value);
        }
    }

    public class WaveformData : ObservableObject
    {
        private ushort[] _samples = new ushort[32];
        public ushort[] Samples
        {
            get => _samples;
            set => SetProperty(ref _samples, value);
        }

        private DateTime _captureTime;
        public DateTime CaptureTime
        {
            get => _captureTime;
            set => SetProperty(ref _captureTime, value);
        }
    }

    public class InterlockStatus : ObservableObject
    {
        private bool _powerOK = true;
        public bool PowerOK
        {
            get => _powerOK;
            set => SetProperty(ref _powerOK, value);
        }

        private bool _tempOK = true;
        public bool TempOK
        {
            get => _tempOK;
            set => SetProperty(ref _tempOK, value);
        }

        private bool _doorOK = true;
        public bool DoorOK
        {
            get => _doorOK;
            set => SetProperty(ref _doorOK, value);
        }

        private bool _waterOK = true;
        public bool WaterOK
        {
            get => _waterOK;
            set => SetProperty(ref _waterOK, value);
        }

        private bool _coverOK = true;
        public bool CoverOK
        {
            get => _coverOK;
            set => SetProperty(ref _coverOK, value);
        }

        private bool _dischargeTempOK = true;
        public bool DischargeTempOK
        {
            get => _dischargeTempOK;
            set => SetProperty(ref _dischargeTempOK, value);
        }

        private bool _overVoltageOK = true;
        public bool OverVoltageOK
        {
            get => _overVoltageOK;
            set => SetProperty(ref _overVoltageOK, value);
        }

        private bool _overTempOK = true;
        public bool OverTempOK
        {
            get => _overTempOK;
            set => SetProperty(ref _overTempOK, value);
        }

        private bool _externalTriggerOK = true;
        public bool ExternalTriggerOK
        {
            get => _externalTriggerOK;
            set => SetProperty(ref _externalTriggerOK, value);
        }

        private byte _rawStatus;
        public byte RawStatus
        {
            get => _rawStatus;
            set
            {
                if (SetProperty(ref _rawStatus, value))
                {
                    PowerOK = (value & 0x01) == 0;
                    TempOK = (value & 0x02) == 0;
                    DoorOK = (value & 0x04) == 0;
                    WaterOK = (value & 0x08) == 0;
                    CoverOK = (value & 0x10) == 0;
                    DischargeTempOK = (value & 0x20) == 0;
                    OverVoltageOK = (value & 0x40) == 0;
                    OverTempOK = (value & 0x80) == 0;
                }
            }
        }
    }
}
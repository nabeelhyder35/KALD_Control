
using System;

namespace KALD_Control.Models
{
    /// <summary>
    /// Represents the main device data for the laser system, matching variables in nios_app.h.
    /// </summary>
    public class DeviceData : ObservableObject
    {
        private LaserStateType _systemState;
        public LaserStateType SystemState
        {
            get => _systemState;
            set => SetProperty(ref _systemState, value);
        }

        private LaserStateType _nextState;
        public LaserStateType NextState
        {
            get => _nextState;
            set => SetProperty(ref _nextState, value);
        }

        private LaserStateType _lastState;
        public LaserStateType LastState
        {
            get => _lastState;
            set => SetProperty(ref _lastState, value);
        }

        private LaserStateType _laserState;
        public LaserStateType LaserState
        {
            get => _laserState;
            set => SetProperty(ref _laserState, value);
        }

        private ushort _setVoltage;
        public ushort SetVoltage
        {
            get => _setVoltage;
            set => SetProperty(ref _setVoltage, value);
        }

        private ushort _nextVoltage;
        public ushort NextVoltage
        {
            get => _nextVoltage;
            set => SetProperty(ref _nextVoltage, value);
        }

        private ushort _actualVoltage;
        public ushort ActualVoltage
        {
            get => _actualVoltage;
            set => SetProperty(ref _actualVoltage, value);
        }

        private ushort _voltageSetpoint;
        public ushort VoltageSetpoint
        {
            get => _voltageSetpoint;
            set => SetProperty(ref _voltageSetpoint, value);
        }

        private ushort _idleSetpoint;
        public ushort IdleSetpoint
        {
            get => _idleSetpoint;
            set => SetProperty(ref _idleSetpoint, value);
        }

        private ushort _rampVoltage;
        public ushort RampVoltage
        {
            get => _rampVoltage;
            set => SetProperty(ref _rampVoltage, value);
        }

        private ushort _pulseWidth;
        public ushort PulseWidth
        {
            get => _pulseWidth;
            set => SetProperty(ref _pulseWidth, value);
        }

        private ushort _frequency;
        public ushort Frequency
        {
            get => _frequency;
            set => SetProperty(ref _frequency, value);
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

        private uint _shotTotal;
        public uint ShotTotal
        {
            get => _shotTotal;
            set => SetProperty(ref _shotTotal, value);
        }

        private uint _shotCount;
        public uint ShotCount
        {
            get => _shotCount;
            set => SetProperty(ref _shotCount, value);
        }

        private uint _shotStart;
        public uint ShotStart
        {
            get => _shotStart;
            set => SetProperty(ref _shotStart, value);
        }

        private uint _rampCount;
        public uint RampCount
        {
            get => _rampCount;
            set => SetProperty(ref _rampCount, value);
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

        private ushort _voltage;
        public ushort Voltage
        {
            get => _voltage;
            set => SetProperty(ref _voltage, value);
        }

        private CalibrationData _calibration;
        public CalibrationData Calibration
        {
            get => _calibration;
            set => SetProperty(ref _calibration, value);
        }

        private RunStatusData _runStatus;
        public RunStatusData RunStatus
        {
            get => _runStatus;
            set => SetProperty(ref _runStatus, value);
        }

        private ChargeStateData _chargeState;
        public ChargeStateData ChargeState
        {
            get => _chargeState;
            set => SetProperty(ref _chargeState, value);
        }

        private ShutterConfig _shutterConfig;
        public ShutterConfig ShutterConfig
        {
            get => _shutterConfig;
            set => SetProperty(ref _shutterConfig, value);
        }

        private SoftStartConfig _softStartConfig;
        public SoftStartConfig SoftStartConfig
        {
            get => _softStartConfig;
            set => SetProperty(ref _softStartConfig, value);
        }

        private WaveformData _waveform;
        public WaveformData Waveform
        {
            get => _waveform;
            set => SetProperty(ref _waveform, value);
        }

        private PulseConfig _pulseConfig;
        public PulseConfig PulseConfig
        {
            get => _pulseConfig;
            set => SetProperty(ref _pulseConfig, value);
        }
    }
}
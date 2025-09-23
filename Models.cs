using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;

namespace KALD_Control.Models
{
    public abstract class ObservableObject : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler? PropertyChanged;

        protected bool SetProperty<T>(ref T field, T value, [CallerMemberName] string? propertyName = null)
        {
            if (EqualityComparer<T>.Default.Equals(field, value))
                return false;
            field = value;
            OnPropertyChanged(propertyName);
            return true;
        }

        protected void OnPropertyChanged([CallerMemberName] string? propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }

    public static class ProtocolConstants
    {
        public const byte STX = 0x2A;
        public const byte ETX = 0x3A;
        public const int HEADER_LENGTH = 5;
        public const int MIN_PACKET_LENGTH = HEADER_LENGTH + 1;
    }

    public enum LaserStateType : byte
    {
        lsrIdle = 0,
        lsrArming = 1,
        lsrCharging = 2,
        lsrArmed = 3,
        lsrRunning = 4,
        lsrFinished = 5,
        lsrPausing = 6,
        lsrPaused = 7,
        lsrDisarming = 8,
        lsrDischarging = 9,
        lsrFireError = 10
    }

    public enum TriggerModeType : byte
    {
        intTrigMode = 0,
        extTrigMode = 1
    }

    public enum ShotModeType : byte
    {
        burstMode = 0,
        contMode = 1
    }

    public enum ShutterModeType : byte
    {
        autoMode = 0,
        manualMode = 1
    }

    public enum ShutterStateType : byte
    {
        shutterClosed = 0,
        shutterOpened = 1
    }

    public class InterlockStatus
    {
        public bool ArmOK { get; set; }
        public bool RunOK { get; set; }
        public bool FlowOK { get; set; }
        public bool TempOK { get; set; }
        public bool DoorOK { get; set; }
        public bool CoverOK { get; set; }
        public bool DischargeTempOK { get; set; }
        public bool OverVoltageOK { get; set; }
        public bool OverTempOK { get; set; }
        public bool EndOfChargeOK { get; set; }
        public bool ExternalTriggerOK { get; set; }
    }

    public class DeviceData
    {
        public LaserStateType SystemState { get; set; }
        public uint ShotCount { get; set; }
        public PulseConfigData PulseConfig { get; set; }
        public RunStatusData RunStatus { get; set; }
        public CalibrationData Calibration { get; set; }
        public ushort Voltage { get; set; }
    }

    public class PulseConfigData
    {
        public uint ShotCount { get; set; }
        public ushort PulseWidth { get; set; }
        public ushort Frequency { get; set; }
        public ushort Delay1 { get; set; }
        public ushort Delay2 { get; set; }
        public TriggerModeType TriggerMode { get; set; }
        public ShotModeType ShotMode { get; set; }
    }

    public class RunStatusData
    {
        public uint ShotCount { get; set; }
        public ushort Energy { get; set; }
        public ushort Power { get; set; }
        public ushort Current { get; set; }
        public ushort VoltageDroop { get; set; }
        public LaserStateType State { get; set; }
    }

    public class WaveformData
    {
        public ushort[] Waveform { get; set; }
    }

    public class ChargeStateData
    {
        public ushort MeasuredVoltage { get; set; }
        public bool IsCharging { get; set; }
    }

    public class CalibrationData
    {
        public ushort CapVoltRange { get; set; }
    }

    public class ShutterConfig
    {
        public ShutterModeType ShutterMode { get; set; }
        public ShutterStateType ShutterState { get; set; }
    }

    public class SoftStartConfig
    {
        public bool Enable { get; set; }
        public ushort IdleSetpoint { get; set; }
        public uint RampCount { get; set; }
    }

    public static class ProtocolUtilities
    {
        public static byte[] UshortToBytes(ushort value)
        {
            return new byte[] { (byte)(value >> 8), (byte)(value & 0xFF) };
        }

        public static ushort BytesToUshort(byte[] bytes, int startIndex = 0)
        {
            if (bytes == null || startIndex + 1 >= bytes.Length)
                return 0;
            return (ushort)((bytes[startIndex] << 8) | bytes[startIndex + 1]);
        }

        public static byte[] UintToBytes(uint value)
        {
            return new byte[] { (byte)(value >> 24), (byte)(value >> 16), (byte)(value >> 8), (byte)(value & 0xFF) };
        }

        public static uint BytesToUint(byte[] bytes, int startIndex = 0)
        {
            if (bytes == null || startIndex + 3 >= bytes.Length)
                return 0;
            return (uint)((bytes[startIndex] << 24) | (bytes[startIndex + 1] << 16) |
                          (bytes[startIndex + 2] << 8) | bytes[startIndex + 3]);
        }
    }

    public static class Constants
    {
        public const int DefaultBaudRate = 115200;
    }
}
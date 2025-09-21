using System;

namespace KALD_Control.Models
{
    /// <summary>
    /// Represents the digital I/O state of the laser system, including interlock status and mask.
    /// </summary>
    public class DigitalIOState : ObservableObject
    {
        private uint _inputStates;
        public uint InputStates
        {
            get => _inputStates;
            set
            {
                if (SetProperty(ref _inputStates, value))
                {
                    UpdateInterlockStatus();
                    OnPropertyChanged(nameof(InterlockStatus));
                    OnPropertyChanged(nameof(Input0));
                    OnPropertyChanged(nameof(Input1));
                    OnPropertyChanged(nameof(Input2));
                    OnPropertyChanged(nameof(Input3));
                    OnPropertyChanged(nameof(Input4));
                    OnPropertyChanged(nameof(Input5));
                    OnPropertyChanged(nameof(Input6));
                    OnPropertyChanged(nameof(Input7));
                    OnPropertyChanged(nameof(PowerInterlockOK));
                    OnPropertyChanged(nameof(TempInterlockOK));
                    OnPropertyChanged(nameof(DoorInterlockOK));
                    OnPropertyChanged(nameof(WaterInterlockOK));
                    OnPropertyChanged(nameof(CoverInterlockOK));
                    OnPropertyChanged(nameof(DischargeInterlockOK));
                }
            }
        }

        private uint _outputStates;
        public uint OutputStates
        {
            get => _outputStates;
            set
            {
                if (SetProperty(ref _outputStates, value))
                {
                    OnPropertyChanged(nameof(Output0));
                    OnPropertyChanged(nameof(Output1));
                    OnPropertyChanged(nameof(Output2));
                    OnPropertyChanged(nameof(Output3));
                    OnPropertyChanged(nameof(Output4));
                    OnPropertyChanged(nameof(Output5));
                    OnPropertyChanged(nameof(Output6));
                    OnPropertyChanged(nameof(Output7));
                    OnPropertyChanged(nameof(ShutterOpen));
                    OnPropertyChanged(nameof(CoolerOn));
                    OnPropertyChanged(nameof(HVEnabled));
                    OnPropertyChanged(nameof(SimmerOn));
                    OnPropertyChanged(nameof(DumpOn));
                }
            }
        }

        private DateTime _timestamp;
        public DateTime Timestamp
        {
            get => _timestamp;
            set => SetProperty(ref _timestamp, value);
        }

        private byte _interlockMask = 0x3F; // Bits 0–5 (matching InterlockStatus.cs)
        public byte InterlockMask
        {
            get => _interlockMask;
            set
            {
                if (SetProperty(ref _interlockMask, value))
                {
                    UpdateInterlockStatus();
                    OnPropertyChanged(nameof(InterlockStatus));
                    OnPropertyChanged(nameof(PowerInterlockOK));
                    OnPropertyChanged(nameof(TempInterlockOK));
                    OnPropertyChanged(nameof(DoorInterlockOK));
                    OnPropertyChanged(nameof(WaterInterlockOK));
                    OnPropertyChanged(nameof(CoverInterlockOK));
                    OnPropertyChanged(nameof(DischargeInterlockOK));
                }
            }
        }

        private byte _interlockStatus;
        public byte InterlockStatus
        {
            get => _interlockStatus;
            private set => SetProperty(ref _interlockStatus, value);
        }

        private bool _interlockGood = true;
        public bool InterlockGood
        {
            get => _interlockGood;
            private set => SetProperty(ref _interlockGood, value);
        }

        // Individual input bit properties
        public bool Input0 => (_inputStates & 0x01) != 0;
        public bool Input1 => (_inputStates & 0x02) != 0;
        public bool Input2 => (_inputStates & 0x04) != 0; // Power interlock
        public bool Input3 => (_inputStates & 0x08) != 0; // Temperature interlock
        public bool Input4 => (_inputStates & 0x10) != 0; // Door interlock
        public bool Input5 => (_inputStates & 0x20) != 0; // Water interlock
        public bool Input6 => (_inputStates & 0x40) != 0; // Cover interlock
        public bool Input7 => (_inputStates & 0x80) != 0; // Discharge interlock

        // Individual output bit properties
        public bool Output0 => (_outputStates & 0x01) != 0;
        public bool Output1 => (_outputStates & 0x02) != 0;
        public bool Output2 => (_outputStates & 0x04) != 0;
        public bool Output3 => (_outputStates & 0x08) != 0;
        public bool Output4 => (_outputStates & 0x10) != 0;
        public bool Output5 => (_outputStates & 0x20) != 0;
        public bool Output6 => (_outputStates & 0x40) != 0;
        public bool Output7 => (_outputStates & 0x80) != 0;

        // Interlock properties (bits 2–7, matching InterlockStatus.cs)
        public bool PowerInterlockOK => ((_inputStates & 0x04) == 0) || ((_interlockMask & 0x01) == 0);
        public bool TempInterlockOK => ((_inputStates & 0x08) == 0) || ((_interlockMask & 0x02) == 0);
        public bool DoorInterlockOK => ((_inputStates & 0x10) == 0) || ((_interlockMask & 0x04) == 0);
        public bool WaterInterlockOK => ((_inputStates & 0x20) == 0) || ((_interlockMask & 0x08) == 0);
        public bool CoverInterlockOK => ((_inputStates & 0x40) == 0) || ((_interlockMask & 0x10) == 0);
        public bool DischargeInterlockOK => ((_inputStates & 0x80) == 0) || ((_interlockMask & 0x20) == 0);

        // Output controls
        public bool ShutterOpen
        {
            get => (_outputStates & 0x01) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x01;
                else
                    OutputStates &= ~0x01u;
            }
        }

        public bool CoolerOn
        {
            get => (_outputStates & 0x02) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x02;
                else
                    OutputStates &= ~0x02u;
            }
        }

        public bool HVEnabled
        {
            get => (_outputStates & 0x04) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x04;
                else
                    OutputStates &= ~0x04u;
            }
        }

        public bool SimmerOn
        {
            get => (_outputStates & 0x08) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x08;
                else
                    OutputStates &= ~0x08u;
            }
        }

        public bool DumpOn
        {
            get => (_outputStates & 0x10) != 0;
            set
            {
                if (value)
                    OutputStates |= 0x10;
                else
                    OutputStates &= ~0x10u;
            }
        }

        private void UpdateInterlockStatus()
        {
            // Map input bits 2–7 to interlock status bits 0–5
            byte inputBits = (byte)((_inputStates >> 2) & 0x3F);
            // Interlocks are active low; invert and mask
            InterlockStatus = (byte)((~inputBits) & _interlockMask & 0x3F);
            InterlockGood = InterlockStatus == (_interlockMask & 0x3F);
            Timestamp = DateTime.Now;
        }

        /// <summary>
        /// Checks interlock status and returns true if all are satisfied.
        /// </summary>
        public bool CheckInterlocks()
        {
            UpdateInterlockStatus();
            return InterlockGood;
        }

        /// <summary>
        /// Gets the interlock status for LCD transmission.
        /// </summary>
        public byte GetInterlockStatusForLCD()
        {
            return InterlockStatus;
        }

        /// <summary>
        /// Gets the interlock mask for LCD transmission.
        /// </summary>
        public byte GetInterlockMaskForLCD()
        {
            return _interlockMask;
        }

        /// <summary>
        /// Sets the interlock mask from an LCD command.
        /// </summary>
        public void SetInterlockMaskFromLCD(byte maskByte)
        {
            InterlockMask = (byte)(maskByte & 0x3F);
        }
    }
}
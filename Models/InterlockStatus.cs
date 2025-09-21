namespace KALD_Control.Models
{
    /// <summary>
    /// Represents the interlock status and mask flags for the laser system.
    /// Each bit in the status/mask byte represents a specific interlock condition.
    /// A bit value of 0 indicates the condition is OK, while 1 indicates a fault.
    /// </summary>
    public class InterlockStatus : ObservableObject
    {
        private byte _status;
        private byte _mask = 0x3F; // Default mask: bits 0-5 enabled, 6-7 reserved

        /// <summary>
        /// Gets or sets the interlock status byte. Updates all related properties when changed.
        /// </summary>
        public byte Status
        {
            get => _status;
            set
            {
                if (SetProperty(ref _status, value))
                {
                    OnPropertyChanged(nameof(PowerOK));
                    OnPropertyChanged(nameof(TempOK));
                    OnPropertyChanged(nameof(DoorOK));
                    OnPropertyChanged(nameof(WaterOK));
                    OnPropertyChanged(nameof(CoverOK));
                    OnPropertyChanged(nameof(DischargeTempOK));
                    OnPropertyChanged(nameof(OverVoltageOK));
                    OnPropertyChanged(nameof(OverTempOK));
                    OnPropertyChanged(nameof(AllOK));
                }
            }
        }

        /// <summary>
        /// Gets or sets the interlock mask byte. Updates all related masked properties when changed.
        /// </summary>
        public byte Mask
        {
            get => _mask;
            set
            {
                if (SetProperty(ref _mask, value))
                {
                    OnPropertyChanged(nameof(PowerMasked));
                    OnPropertyChanged(nameof(TempMasked));
                    OnPropertyChanged(nameof(DoorMasked));
                    OnPropertyChanged(nameof(WaterMasked));
                    OnPropertyChanged(nameof(CoverMasked));
                    OnPropertyChanged(nameof(DischargeTempMasked));
                    OnPropertyChanged(nameof(OverVoltageMasked));
                    OnPropertyChanged(nameof(OverTempMasked));
                    OnPropertyChanged(nameof(AllMasked));
                }
            }
        }

        // Status properties: true if the interlock condition is OK (bit is 0), false if faulted (bit is 1)
        public bool PowerOK => (_status & 0x01) == 0;
        public bool TempOK => (_status & 0x02) == 0;
        public bool DoorOK => (_status & 0x04) == 0;
        public bool WaterOK => (_status & 0x08) == 0;
        public bool CoverOK => (_status & 0x10) == 0;
        public bool DischargeTempOK => (_status & 0x20) == 0;
        public bool OverVoltageOK => (_status & 0x40) == 0; // Reserved, may be used in UI
        public bool OverTempOK => (_status & 0x80) == 0;   // Reserved, may be used in UI

        /// <summary>
        /// Gets a value indicating whether all enabled interlock conditions are OK.
        /// </summary>
        public bool AllOK => (PowerOK || !PowerMasked) &&
                             (TempOK || !TempMasked) &&
                             (DoorOK || !DoorMasked) &&
                             (WaterOK || !WaterMasked) &&
                             (CoverOK || !CoverMasked) &&
                             (DischargeTempOK || !DischargeTempMasked) &&
                             (OverVoltageOK || !OverVoltageMasked) &&
                             (OverTempOK || !OverTempMasked);

        // Mask properties: true if the interlock condition is enabled (bit is 1), false if disabled (bit is 0)
        public bool PowerMasked => (_mask & 0x01) != 0;
        public bool TempMasked => (_mask & 0x02) != 0;
        public bool DoorMasked => (_mask & 0x04) != 0;
        public bool WaterMasked => (_mask & 0x08) != 0;
        public bool CoverMasked => (_mask & 0x10) != 0;
        public bool DischargeTempMasked => (_mask & 0x20) != 0;
        public bool OverVoltageMasked => (_mask & 0x40) != 0; // Reserved, may be used in UI
        public bool OverTempMasked => (_mask & 0x80) != 0;   // Reserved, may be used in UI

        /// <summary>
        /// Gets a value indicating whether all interlock conditions are enabled.
        /// </summary>
        public bool AllMasked => PowerMasked && TempMasked && DoorMasked && WaterMasked &&
                                 CoverMasked && DischargeTempMasked && OverVoltageMasked && OverTempMasked;

        /// <summary>
        /// Updates the interlock status from a received byte.
        /// </summary>
        /// <param name="newStatus">The new status byte received from the device.</param>
        public void UpdateFromByte(byte newStatus)
        {
            Status = (byte)(newStatus & 0x3F); // Mask to 6 bits for NIOS compatibility
        }

        /// <summary>
        /// Returns the interlock status as a byte for transmission.
        /// </summary>
        /// <returns>The status byte.</returns>
        public byte ToByte()
        {
            return (byte)(_status & 0x3F); // Mask to 6 bits for NIOS compatibility
        }

        /// <summary>
        /// Updates the interlock mask from a received byte.
        /// </summary>
        /// <param name="newMask">The new mask byte received from the device.</param>
        public void UpdateMask(byte newMask)
        {
            Mask = newMask; // Allow 8 bits for UI, NIOS uses 6 bits
        }
    }
}
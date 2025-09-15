using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KALD_Control.Models
{
    public class CalibrationData : ObservableObject
    {
        private ushort _capVoltRange;
        public ushort CapVoltRange
        {
            get => _capVoltRange;
            set => SetProperty(ref _capVoltRange, value);
        }

        private ushort _measuredVoltage;
        public ushort MeasuredVoltage
        {
            get => _measuredVoltage;
            set => SetProperty(ref _measuredVoltage, value);
        }

        private byte _calibrationStatus;
        public byte CalibrationStatus
        {
            get => _calibrationStatus;
            set => SetProperty(ref _calibrationStatus, value);
        }
    }
}
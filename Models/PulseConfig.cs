namespace KALD_Control.Models
{
    /// <summary>
    /// Represents the pulse configuration for the laser system.
    /// </summary>
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
}
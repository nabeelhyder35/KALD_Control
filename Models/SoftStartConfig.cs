namespace KALD_Control.Models
{
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
}
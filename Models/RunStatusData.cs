namespace KALD_Control.Models
{
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
}
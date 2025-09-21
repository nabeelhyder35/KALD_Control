namespace KALD_Control.Models
{
    /// <summary>
    /// Represents the charge state data of the laser system, including measured voltage and charge completion status.
    /// </summary>
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
}
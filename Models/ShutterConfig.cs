namespace KALD_Control.Models
{
    /// <summary>
    /// Represents the shutter configuration for the laser system.
    /// </summary>
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
}
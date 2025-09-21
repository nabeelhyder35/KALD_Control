namespace KALD_Control.Models
{
    // Constants used in CalibrationData conversion formulas.
    // Values are placeholders; adjust to match real hardware characteristics if available.
    public static class HardwareConstants
    {
        // Cap voltage conversion
        public const double CAPV_IN_SLOPE = 1.0;
        public const double CAPV_IN_COUNT = 4096.0;
        public const double CAPV_IN_OFFSET = 0.0;

        // Lamp current conversion
        public const double LAMPC_IN_SLOPE = 1.0;
        public const double LAMPC_IN_COUNT = 4096.0;
        public const double LAMPC_IN_OFFSET = 0.0;

        // Lamp voltage conversion
        public const double LAMPV_IN_SLOPE = 1.0;
        public const double LAMPV_IN_COUNT = 4096.0;
        public const double LAMPV_IN_OFFSET = 0.0;
    }
}
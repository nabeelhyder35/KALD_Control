using System;

namespace KALD_Control.Models
{
    public class WaveformData
    {
        public ushort[] Samples { get; } = new ushort[32];
        public DateTime CaptureTime { get; set; } = DateTime.MinValue;
    }
}
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KALD_Control.Models
{
    public class DigitalIOState : ObservableObject
    {
        private uint _inputStates;
        public uint InputStates
        {
            get => _inputStates;
            set => SetProperty(ref _inputStates, value);
        }

        private uint _outputStates;
        public uint OutputStates
        {
            get => _outputStates;
            set => SetProperty(ref _outputStates, value);
        }

        private DateTime _timestamp;
        public DateTime Timestamp
        {
            get => _timestamp;
            set => SetProperty(ref _timestamp, value);
        }
    }
}
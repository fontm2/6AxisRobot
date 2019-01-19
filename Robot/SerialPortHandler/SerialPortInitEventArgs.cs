using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SerialPortHandler
{
    public class SerialPortInitEventArgs : EventArgs
    {
        public string InitializedString;
        public SerialPortInitEventArgs(string str)
        {
            InitializedString = str;
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;

namespace SerialPortHandler
{
    public class MotorController : SerialPort, IDisposable
    {
        private bool disposed = false;
        public MotorController(string COMName, int BaudRate)
        {
            this.PortName = COMName;
            this.BaudRate = BaudRate;
            this.Parity = Parity.None;
            this.DataBits = 8;
            this.StopBits = StopBits.One;
            this.Handshake = Handshake.None;
            this.ReadTimeout = 500;
            this.WriteTimeout = 500;

            if (!(this.IsOpen == true))
            {
                this.Open();
            }
        }
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }
        protected override void Dispose(bool disposing)
        {
            if (!disposed)
            {
                if (disposing)
                {
                    //this.Dispose();
                }
                disposed = true;
            }
            base.Dispose(disposing);
        }
        public void SendCommand(byte Adress, byte Command, byte Typ, byte Motor, int Value)
        {
            int i;
            byte[] valueHandle = BitConverter.GetBytes(Value);
            byte[] buffer = new byte[9];
            buffer[0] = Adress;
            buffer[1] = Command;
            buffer[2] = Typ;
            buffer[3] = Motor;
            buffer[4] = valueHandle[3];
            buffer[5] = valueHandle[2];
            buffer[6] = valueHandle[1];
            buffer[7] = valueHandle[0];
            buffer[8] = 0;
            for (i = 0; i < 8; i++)
                buffer[8] += buffer[i];

            this.Write(buffer, 0, buffer.Length);
        }
    }
}

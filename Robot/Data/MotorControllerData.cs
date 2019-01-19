using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Data
{
    public class MotorControllerData: ProcessData
    {
        private int pulseDiv;
        public int PulseDiv
        {
            get { return pulseDiv; }
            set { pulseDiv = value; OnValueChange("PulseDiv"); }
        }
        private int rampDiv;
        public int RampDiv
        {
            get { return rampDiv; }
            set { rampDiv = value; OnValueChange("RampDiv"); }
        }
        private int adress;
        public int Adress
        {
            get { return adress; }
            set { adress = value; OnValueChange("Adress"); }
        }
        private int speed;
        public int Speed
        {
            get { return speed; }
            set { speed = value; OnValueChange("Speed"); }
        }
        private int acceleration;
        public int Acceleration
        {
            get { return acceleration; }
            set { acceleration = value; OnValueChange("Acceleration"); }
        }
        private int position;
        public int Position
        {
            get { return position; }
            set { position = value; OnValueChange("Position"); }
        }
        private double current;
        public double Current
        {
            get { return current; }
            set { current = value; OnValueChange("Current"); }
        }
        private int _mfrequ;
        public int mFrequency
        {
            get { return _mfrequ; }
            set { _mfrequ = value; OnValueChange("mFrequency"); }
        }
        private int status;
        public int Status
        {
            get { return status; }
            set { status = value; OnValueChange("Status"); }
        }
        private int baud;
        public int Baudrate
        {
            get { return baud; }
            set { baud = value; OnValueChange("Baudrate"); }
        }  
    }
}

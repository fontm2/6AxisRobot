using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.ComponentModel;

namespace Kinematics
{
    public class DHParameter : INotifyPropertyChanged 
    {
        public event PropertyChangedEventHandler PropertyChanged;
        private double[] alpha;
        private double[] a;
        private double[] d;
        public DHParameter(RobotType robottype)
        {
            switch (robottype)
            {
                //case RobotType.FiveAxis:
                //    {
                //        alpha = new double[5];
                //        a = new double[5];
                //        d = new double[5];
                //        break;
                //    }
                case RobotType.SixAxis:
                    {
                        alpha = new double[6];
                        a = new double[6];
                        d = new double[6];
                        break;
                    }
            }
        }
        public void OnValueChange(String propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
        public double[] Alpha
        {
            get { return alpha; }
            set { alpha = value; OnValueChange("Alpha"); }
        }
        public double[] D
        {
            get { return d; }
            set { d = value; OnValueChange("D"); }
        }
        public double[] A
        {
            get { return a; }
            set { a = value; OnValueChange("A"); }
        }
    }   
}

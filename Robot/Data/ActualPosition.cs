using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Data
{
    public class ActualPosition : ProcessData
    {
        private int schritte1;
        public int Schritte1
        {
            get { return schritte1; }
            set { schritte1 = value; OnValueChange("Schritte1"); }
        }
        private int schritte2;
        public int Schritte2
        {
            get { return schritte2; }
            set { schritte2 = value; OnValueChange("Schritte2"); }
        }
        private int schritte3;
        public int Schritte3
        {
            get { return schritte3; }
            set { schritte3 = value; OnValueChange("Schritte3"); }
        }
        private int schritte4;
        public int Schritte4
        {
            get { return schritte4; }
            set { schritte4 = value; OnValueChange("Schritte4"); }
        }
        private int schritte5;
        public int Schritte5
        {
            get { return schritte5; }
            set { schritte5 = value; OnValueChange("Schritte5"); }
        }
        private int schritte6;
        public int Schritte6
        {
            get { return schritte6; }
            set { schritte6 = value; OnValueChange("Schritte6"); }
        }
        private double winkel1;
        public double Winkel1
        {
            get { return winkel1; }
            set { winkel1 = value; OnValueChange("Winkel1"); }
        }
        private double winkel2;
        public double Winkel2
        {
            get { return winkel2; }
            set { winkel2 = value; OnValueChange("Winkel2"); }
        }
        private double winkel3;
        public double Winkel3
        {
            get { return winkel3; }
            set { winkel3 = value; OnValueChange("Winkel3"); }
        }
        private double winkel4;
        public double Winkel4
        {
            get { return winkel4; }
            set { winkel4 = value; OnValueChange("Winkel4"); }
        }
        private double winkel5;
        public double Winkel5
        {
            get { return winkel5; }
            set { winkel5 = value; OnValueChange("Winkel5"); }
        }
        private double winkel6;
        public double Winkel6
        {
            get { return winkel6; }
            set { winkel6 = value; OnValueChange("Winkel6"); }
        }
        private double posX;
        public double PosX
        {
            get { return posX; }
            set { posX = value; OnValueChange("PosX"); }
        }
        private double posY;
        public double PosY
        {
            get { return posY; }
            set { posY = value; OnValueChange("PosY"); }
        }
        private double posZ;
        public double PosZ
        {
            get { return posZ; }
            set { posZ = value; OnValueChange("PosZ"); }
        }
        private double alpha;
        public double Alpha
        {
            get { return alpha; }
            set { alpha = value; OnValueChange("Alpha"); }
        }
        private double beta;
        public double Beta
        {
            get { return beta; }
            set { beta = value; OnValueChange("Beta"); }
        }
        private double gamma;
        public double Gamma
        {
            get { return gamma; }
            set { gamma = value; OnValueChange("Gamma"); }
        }
    }
}

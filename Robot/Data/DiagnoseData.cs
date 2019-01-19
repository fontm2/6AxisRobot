using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Data
{
    public class DiagnoseData : NCData
    {
        private int pos;
        public int diagPos
        {
            get { return pos; }
            set { pos = value; OnValueChange("diagPos"); }
        }
        private double gx;
        public double diagX
        {
            get { return gx; }
            set { gx = value; OnValueChange("diagX"); }
        }
        private double gy;
        public double diagY
        {
            get { return gy; }
            set { gy = value; OnValueChange("diagY"); }
        }
        private double gz;
        public double diagZ
        {
            get { return gz; }
            set { gz = value; OnValueChange("diagZ"); }
        }
        private double ga;
        public double diagA
        {
            get { return ga; }
            set { ga = value; OnValueChange("diagA"); }
        }
        private double gb;
        public double diagB
        {
            get { return gb; }
            set { gb = value; OnValueChange("diagB"); }
        }
        private double gc;
        public double diagC
        {
            get { return gc; }
            set { gc = value; OnValueChange("diagC"); }
        }
        private string diagstr;
        public string DiagStr
        {
            get { return diagstr; }
            set { diagstr = value; OnValueChange("DiagStr"); }
        }
        private string diagstrTreadNr;
        public string DiagStrTreadNr
        {
            get { return diagstrTreadNr; }
            set { diagstrTreadNr = value; OnValueChange("DiagStrTreadNr"); }
        }
    }
}

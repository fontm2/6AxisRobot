using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinematics
{
    public class KinematicEventArgs : EventArgs
    {
        public string KinematicEventMessage;
        public KinematicEventArgs(double teta1, double teta2, double teta3, double teta4, double teta5, double teta6)
        {
            this.KinematicEventMessage = String.Format("Sie liegene ausserhalb des Arbeitsbereiches\n{0,-10}{1,7}<{2,9}>{3,7}\n{4,-10}{5,7}<{6,9}>{7,7}\n" +
                "{8,-10}{9,7}<{10,9}>{11,7}\n{12,-10}{13,7}<{14,9}>{15,7}\n{16,-10}{17,7}<{18,9}>{19,7}\n{20,-10}{21,7}<{22,9}>{23,7}", "Teta1 :"
                , "-200°", Math.Round(teta2, 2).ToString() + "°", "20°", "Teta2 :", "-70°", Math.Round(teta3, 2).ToString() + "°", "250°", "Teta3 :",
                "-100°", Math.Round(teta5, 2).ToString() + "°", "100°", "Teta4 :", "-20°", Math.Round(teta1, 2).ToString() + "°", " 200°", "Teta5 :",
                "-200°", Math.Round(teta4, 2).ToString() + "°", " 200°", "Teta6 :", "-360°", Math.Round(teta6, 2).ToString() + "°", "360°");
        }
        public KinematicEventArgs()
        {
            this.KinematicEventMessage = "All calculation succesfully completed";
        }
        //The "\t" character will probably correctly align your colons.
    }
}

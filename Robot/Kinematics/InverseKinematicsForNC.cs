using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinematics
{
    public class InverseKinematicsForNC : KinematicBase
    {
        public InverseKinematicsForNC(DHParameter dhparam)
            : base(dhparam)
        {

        }
        public double[] NextSpeedForNC(float PositionToReach1, float PositionToReach2, float PositionToReach3, float PositionToReach4, float PositionToReach5, float PositionToReach6, int milis)
        {

            double[] NewSpeed = new double[6];
            //anzahl microsteps(da angle1 die anzahl mikroschritte zurückgibt) pro sekunde =  (16000000 * NewSpeed / (2^6 * 2048 * 32)); pulse_div ist auf 6
            // (16'000'000 * NewSpeed / (2^6 * 2048 * 32) = PositionToReach * 1/sek ---> NewSpeed = (PositionToReach *1/sek * (2^6*2048*32)) / 16'000'000 = PositionToReach * 64 * 2048 * 32 * 1000 / milis / 16'000'000;
            NewSpeed[0] = PositionToReach1 * 64 * 2048 * 32 * 1000 / milis / 16000000;
            NewSpeed[1] = PositionToReach2 * 64 * 2048 * 32 * 1000 / milis / 16000000;
            NewSpeed[2] = PositionToReach3 * 64 * 2048 * 32 * 1000 / milis / 16000000;
            NewSpeed[3] = PositionToReach4 * 64 * 2048 * 32 * 1000 / milis / 16000000;
            NewSpeed[4] = PositionToReach5 * 64 * 2048 * 32 * 1000 / milis / 16000000;
            NewSpeed[5] = PositionToReach6 * 64 * 2048 * 32 * 1000 / milis / 16000000;


            return NewSpeed;
        }
        public double[] InverseKinematicForNC(double vp, double up, double y, double eulerZ, double eulerX, double eulerZ_)
        {
            double[] Angles8 = new double[12];
            double eulerz = eulerZ / 180 * Math.PI;
            double eulerx = eulerX / 180 * Math.PI;
            double eulerz_ = eulerZ_ / 180 * Math.PI;

            double[,] RotZ = new double[,] { { Math.Cos(eulerz), -Math.Sin(eulerz), 0, 0 }, { Math.Sin(eulerz), Math.Cos(eulerz), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
            double[,] RotX = new double[,] { { 1, 0, 0, 0 }, { 0, Math.Cos(eulerx), -Math.Sin(eulerx), 0 }, { 0, Math.Sin(eulerx), Math.Cos(eulerx), 0 }, { 0, 0, 0, 1 } };
            double[,] RotZ_ = new double[,] { { Math.Cos(eulerz_), -Math.Sin(eulerz_), 0, 0 }, { Math.Sin(eulerz_), Math.Cos(eulerz_), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

            double[,] res1 = MultiplyMatricesSeriell(RotZ, RotX);
            double[,] res2 = MultiplyMatricesSeriell(res1, RotZ_);
            double[] vector = new double[3];
            vector[0] = 0;
            vector[1] = 0;
            vector[2] = 1;
            double[] NewVector = MultiplyMatrixVektor(res2, vector);
            double vp_ = vp - base.DHParam.D[5] * NewVector[0]; //l3 = d6 =DHParam.D[5]
            double up_ = up - base.DHParam.D[5] * NewVector[1];//l3 = d6 =DHParam.D[5]
            double y_ = y - base.DHParam.D[5] * NewVector[2];//l3 = d6 =DHParam.D[5]
            //teta1    
            double teta1 = Math.Atan2(up_, vp_);
            double x = Math.Sqrt(vp_ * vp_ + up_ * up_);
            //teta3
            double L_sq = x * x + y_ * y_;
            double L = Math.Sqrt(L_sq);
            double Atan2x2 = -(L_sq - base.DHParam.A[1] * base.DHParam.A[1] - base.DHParam.D[3] * base.DHParam.D[3]) / (2 * base.DHParam.A[1] * base.DHParam.D[3]);//l2 = d4 =DHParam.D[3]; l1 = a2 = DHParam.A[1]
            double Atan2x2_sq = Atan2x2 * Atan2x2;
            double Atan2y2 = Math.Sqrt(1 - Atan2x2_sq);
            double teta3 = Math.Atan2(Atan2y2, -Atan2x2) - Math.PI / 2; // __| ausgangstellung 90° cw = __ __ teta3Original =  Math.Atan2(Atan2y2, -Atan2x2)
            //teta2
            double Atan2x1 = base.DHParam.A[1] + base.DHParam.D[3] * Math.Cos(teta3 + Math.PI / 2);//zum rechnen des ersten winkels muss die tatsächliche position genommen werden also __| ohne die geradestellung
            double Atan2y1 = base.DHParam.D[3] * Math.Sin(teta3 + Math.PI / 2);
            double teta2 = -Math.Atan2(y_, x) - Math.Atan2(Atan2y1, Atan2x1);
            teta3 = teta3 + Math.PI;// der roboter arm wird von der anderen seite angeschaut, darum ändern sich die vorzeichen (cw/ccw) also statt __| sehen wir |__ und nun stellen wir diesen gerade und wir erhalten __ __ indem wir 90° ccw(+) drehen
            //zum rechnen der vorwärts transformation geht man von der ausgangsstellung __ __ und nicht von |__ aus.
            // hier steht +180, weil bei der berechnung: teta3 = Math.Atan2(Atan2y2, -Atan2x2) - Math.PI / 2 bereit einmal 90° in die andere richtung gedreht wurde 
            double[,] DH_1 = DH(teta1, base.DHParam.D[0], base.DHParam.A[0], base.DHParam.Alpha[0]);
            double[,] DH_2 = DH(teta2, base.DHParam.D[1], base.DHParam.A[1], base.DHParam.Alpha[1]);
            double[,] DH_3 = DH(teta3, base.DHParam.D[2], base.DHParam.A[2], base.DHParam.Alpha[2]);
            double[,] DH_123 = DH123(DH_1, DH_2, DH_3);
            double[,] R_123 = ExtractRotationMatrix(DH_123);
            double[,] R_123_4x4 = Get4x4(R_123);
            double[,] R_123_Inv_4x4 = GetInverse(R_123_4x4);
            double[,] res2_4x4 = Get4x4(res2);
            double[,] MatrixFuerTeta4Teta5Teta6 = MultiplyMatricesSeriell(R_123_Inv_4x4, res2_4x4);
            int Flip = -1;

            double teta5 = Math.Atan2(Flip * Math.Sqrt(1 - MatrixFuerTeta4Teta5Teta6[2, 2] * MatrixFuerTeta4Teta5Teta6[2, 2]), MatrixFuerTeta4Teta5Teta6[2, 2]);
            double teta4 = Math.Atan2(Flip * MatrixFuerTeta4Teta5Teta6[1, 2], Flip * MatrixFuerTeta4Teta5Teta6[0, 2]);
            double teta6 = Math.Atan2(Flip * MatrixFuerTeta4Teta5Teta6[2, 1], -MatrixFuerTeta4Teta5Teta6[2, 0] * Flip);

            Angles8[0] = teta1 / Math.PI * 180;
            Angles8[1] = teta2 / Math.PI * 180;
            Angles8[2] = teta3 / Math.PI * 180;
            Angles8[3] = teta4 / Math.PI * 180;
            Angles8[4] = teta5 / Math.PI * 180;
            Angles8[5] = teta6 / Math.PI * 180;
            //Schritte = new int[6];
            //Schritte[0] = (-(int)Math.Round(Angles8[0] / (1.8 / 8 / 32), 0));
            //Schritte[1] = (int)Math.Round(Angles8[1] / (1.8 / 25 / 32), 0);
            //Schritte[2] = (-(int)Math.Round(Angles8[2] / (1.8 / 16 / 32), 0));
            //Schritte[3] = (-(int)Math.Round(Angles8[3] / (1.8 / 8 / 32), 0));
            //Schritte[4] = (int)Math.Round(Angles8[4] / (1.8 / 8 / 32), 0);
            //Schritte[5] = (-(int)Math.Round(Angles8[5] / (1.8 / 32), 0));//über Microsteps übersetzung regeln
            Angles8[6] = (double)(-(int)Math.Round(Angles8[0] / (1.8 / 8 / 32), 0));
            Angles8[7] = (double)(int)Math.Round(Angles8[1] / (1.8 / 25 / 32), 0);
            Angles8[8] = (double)(-(int)Math.Round(Angles8[2] / (1.8 / 16 / 32), 0));
            Angles8[9] = (double)(-(int)Math.Round(Angles8[3] / (1.8 / 8 / 32), 0));
            Angles8[10] = (double)(int)Math.Round(Angles8[4] / (1.8 / 8 / 32), 0);
            Angles8[11] = (-(int)Math.Round(Angles8[5] / (1.8 / 32), 0));
            return Angles8;

        }
    }
}

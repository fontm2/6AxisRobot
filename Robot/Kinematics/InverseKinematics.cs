using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Data;

namespace Kinematics
{
    public sealed class InverseKinematics : KinematicBase
    {      
        public delegate void InverseKineamaticDelegate(InverseKinematics wk, KinematicEventArgs e);
        public event InverseKineamaticDelegate InverseKinematicEvent;
        private int out_of_range_counter;
        private double[] InverseKinematicsDataArray;
        private int[] InverseKinematicsSpeedArray;
        private int[] InverseKinematicsDeltaStepsArray;
        private ActualPosition ActualPosition;

        //static variables are shared between all class instances.
        private static volatile InverseKinematics instance;//because it's static, its for all the instances of InverseKinematics Class tha same Instance. You Can not create a new one, because the Constructor is private
        private static object LockObj = new Object();//this object can only be used from one Thread at the same time (UI-Thread OR Backgroundthread, but not both at a time, can use the same Instance of this Class)
        //Note that instead of locking on typeof(Singleton) as some versions of this implementation do, 
        //Lock on the value of a static variable which is private to the class. Locking on objects
        //which other classes can access and lock on (such as the type of this Class) risks performance issues 
        //and even deadlocks. Wherever possible, only lock on objects specifically created for the purpose of locking, or which document that they are 
        //to be locked on for specific purposes (e.g. for waiting/pulsing a queue). 
        //Usually such objects should be private to the class they are used in. 
        //This helps to make writing thread-safe applications significantly easier.
        
        
        
        ///////////////////////////////DHParam belegung
        //int d1 = 0;
        //double d2 = 0;
        //double d3 = 0;
        //int d4 = 23;
        //double d5 = 0;
        //int d6 = 10;
        //double a1 = 0;
        //int a2 = 25;
        //int a3 = 0;
        //double a4 = 0;
        //int a5 = 0;
        //double a6 = 0;
        //double alpha1 = -Math.PI / 2;
        //double alpha2 = 0;
        //double alpha3 = Math.PI / 2;
        //double alpha4 = -Math.PI / 2;
        //double alpha5 = Math.PI / 2;
        //double alpha6 = 0;

        private InverseKinematics(DHParameter dhparam, ActualPosition _ActualPosition)
            : base(dhparam)
        {
            this.ActualPosition = _ActualPosition;
        }
        /// <summary>
        /// IF the instance has allready been created, the instance created before will be given back, when calling this method. Ohterwise a new instance will be given back
        /// </summary>
        /// <param name="dhparam"></param>
        /// <returns></returns>
        public static InverseKinematics Instantiate(DHParameter dhparam, ActualPosition _ActualPosition) //static variables are shared between all class instances.
        {
            if (instance == null)//Prevents of Locking if the instance allready has been created. IF the instance has allready been created, this instance created before will be given back, when calling this method
            {
                lock (LockObj)//no new instance can be createt from any thread as long as the first one onlocks the object
                {
                    if (instance == null)
                    {
                        instance = new InverseKinematics(dhparam, _ActualPosition);
                    }
                }
            }
            return instance;
        }

        public void InverseKinematic(double vp, double up, double y, double eulerZ, double eulerX, double eulerZ_, int _speed)
        {
            InverseKinematicsDataArray = new double[12];
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
            double vp_ = vp - DHParam.D[5] * NewVector[0]; //l3 = d6 =DHParam.D[5]
            double up_ = up - DHParam.D[5] * NewVector[1];//l3 = d6 =DHParam.D[5]
            double y_ = y - DHParam.D[5] * NewVector[2];//l3 = d6 =DHParam.D[5]
            //teta1    
            double teta1 = Math.Atan2(up_, vp_);
            double x = Math.Sqrt(vp_ * vp_ + up_ * up_);
            //teta3
            double L_sq = x * x + y_ * y_;
            double L = Math.Sqrt(L_sq);
            double Atan2x2 = -(L_sq - DHParam.A[1] * DHParam.A[1] - DHParam.D[3] * DHParam.D[3]) / (2 * DHParam.A[1] * DHParam.D[3]);//l2 = d4 =DHParam.D[3]; l1 = a2 = DHParam.A[1]
            double Atan2x2_sq = Atan2x2 * Atan2x2;
            double Atan2y2 = Math.Sqrt(1 - Atan2x2_sq);
            double teta3 = Math.Atan2(Atan2y2, -Atan2x2) - Math.PI / 2; // __| ausgangstellung 90° cw = __ __ teta3Original =  Math.Atan2(Atan2y2, -Atan2x2)
            //teta2
            double Atan2x1 = DHParam.A[1] + DHParam.D[3] * Math.Cos(teta3 + Math.PI / 2);//zum rechnen des ersten winkels muss die tatsächliche position genommen werden also __| ohne die geradestellung
            double Atan2y1 = DHParam.D[3] * Math.Sin(teta3 + Math.PI / 2);
            double teta2 = -Math.Atan2(y_, x) - Math.Atan2(Atan2y1, Atan2x1);
            teta3 = teta3 + Math.PI;// der roboter arm wird von der anderen seite angeschaut, darum ändern sich die vorzeichen (cw/ccw) also statt __| sehen wir |__ und nun stellen wir diesen gerade und wir erhalten __ __ indem wir 90° ccw(+) drehen
            //zum rechnen der vorwärts transformation geht man von der ausgangsstellung __ __ und nicht von |__ aus.
            // hier steht +180, weil bei der berechnung: teta3 = Math.Atan2(Atan2y2, -Atan2x2) - Math.PI / 2 bereit einmal 90° in die andere richtung gedreht wurde 
            double[,] DH_1 = DH(teta1, DHParam.D[0], DHParam.A[0], DHParam.Alpha[0]);
            double[,] DH_2 = DH(teta2, DHParam.D[1], DHParam.A[1], DHParam.Alpha[1]);
            double[,] DH_3 = DH(teta3, DHParam.D[2], DHParam.A[2], DHParam.Alpha[2]);
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

            InverseKinematicsDataArray[0] = teta1 / Math.PI * 180;
            InverseKinematicsDataArray[1] = teta2 / Math.PI * 180;
            InverseKinematicsDataArray[2] = teta3 / Math.PI * 180;
            InverseKinematicsDataArray[3] = teta4 / Math.PI * 180;
            InverseKinematicsDataArray[4] = teta5 / Math.PI * 180;
            InverseKinematicsDataArray[5] = teta6 / Math.PI * 180;
            int[] Schritte = new int[6];
            Schritte[0] = -(int)Math.Round(InverseKinematicsDataArray[0] / (1.8 / 8 / 32), 0);
            Schritte[1] = (int)Math.Round(InverseKinematicsDataArray[1] / (1.8 / 25 / 32), 0);
            Schritte[2] = -(int)Math.Round(InverseKinematicsDataArray[2] / (1.8 / 16 / 32), 0);
            Schritte[3] = -(int)Math.Round(InverseKinematicsDataArray[3] / (1.8 / 8 / 32), 0);
            Schritte[4] = (int)Math.Round(InverseKinematicsDataArray[4] / (1.8 / 8 / 32), 0);
            Schritte[5] = -(int)Math.Round(InverseKinematicsDataArray[5] / (1.8 / 32), 0);//über Microsteps übersetzung regeln
            InverseKinematicsDataArray[6] = (double)Schritte[0];
            InverseKinematicsDataArray[7] = (double)Schritte[1];
            InverseKinematicsDataArray[8] = (double)Schritte[2];
            InverseKinematicsDataArray[9] = (double)Schritte[3];
            InverseKinematicsDataArray[10] = (double)Schritte[4];
            InverseKinematicsDataArray[11] = (double)Schritte[5];
            out_of_range_counter = 0;
            for (int b = 0; b < 6; b++)
            {
                switch (b)
                {
                    case 0:
                        {
                            if (InverseKinematicsDataArray[b] <= -20 || InverseKinematicsDataArray[b] >= 200)
                            {
                                out_of_range_counter++;
                            }
                            break;
                        }
                    case 1:
                        {
                            if (InverseKinematicsDataArray[b] <= -200 || InverseKinematicsDataArray[b] >= 20)
                            {
                                out_of_range_counter++;
                            }
                            break;
                        }
                    case 2:
                        {
                            if (InverseKinematicsDataArray[b] <= -70 || InverseKinematicsDataArray[b] >= 250)
                            {
                                out_of_range_counter++;
                            }
                            break;
                        }
                    case 3:
                        {
                            if (InverseKinematicsDataArray[b] <= -200 || InverseKinematicsDataArray[b] >= 200)
                            {
                                out_of_range_counter++;
                            }
                            break;
                        }
                    case 4:
                        {
                            if (InverseKinematicsDataArray[b] <= -100 || InverseKinematicsDataArray[b] >= 100)
                            {
                                out_of_range_counter++;
                            }
                            break;
                        }
                    case 5:
                        {
                            if (InverseKinematicsDataArray[b] <= -360 || InverseKinematicsDataArray[b] >= 360)
                            {
                                out_of_range_counter++;
                            }
                            break;
                        }
                }
            }
            //if ((Angles8[0] <= -20 || Angles8[0] >= 200) || (Angles8[1] <= -200 || Angles8[1] >= 20) || (Angles8[2] <= -70 || Angles8[2] >= 250) || (Angles8[3] <= -200 || Angles8[3] >= 200) || (Angles8[4] <= -100 || Angles8[4] >= 100) || (Angles8[5] <= -360 || Angles8[5] >= 360))//Amgles8[0]=teta4,Amgles8[1]=teta1,Amgles8[2]=teta2,Amgles8[3]=teta5,Amgles8[4]=teta3,Amgles8[5]=teta6
            if (out_of_range_counter > 0)
            {              
                OnKinematicEvent(new KinematicEventArgs(InverseKinematicsDataArray[0], InverseKinematicsDataArray[1], InverseKinematicsDataArray[2], InverseKinematicsDataArray[3], InverseKinematicsDataArray[4], InverseKinematicsDataArray[5]));
            }
            else
            {
                //Calcs the Speed of each axis
                InverseKinematicsDeltaStepsArray = new int[6];
                InverseKinematicsDeltaStepsArray[0] = (int)(InverseKinematicsData[6] - ActualPosition.Schritte1);
                InverseKinematicsDeltaStepsArray[1] = (int)(InverseKinematicsData[7] - ActualPosition.Schritte2);
                InverseKinematicsDeltaStepsArray[2] = (int)(InverseKinematicsData[8] - ActualPosition.Schritte3);
                InverseKinematicsDeltaStepsArray[3] = (int)(InverseKinematicsData[9] - ActualPosition.Schritte4);
                InverseKinematicsDeltaStepsArray[4] = (int)(InverseKinematicsData[10] - ActualPosition.Schritte5);
                InverseKinematicsDeltaStepsArray[5] = (int)(InverseKinematicsData[11] - ActualPosition.Schritte6);
                int deltaSchritte1_2 = Math.Abs(InverseKinematicsDeltaStepsArray[0]);
                int deltaSchritte2_2 = Math.Abs(InverseKinematicsDeltaStepsArray[1]);
                int deltaSchritte3_2 = Math.Abs(InverseKinematicsDeltaStepsArray[2]);
                int deltaSchritte4_2 = Math.Abs(InverseKinematicsDeltaStepsArray[3]);
                int deltaSchritte5_2 = Math.Abs(InverseKinematicsDeltaStepsArray[4]);
                int deltaSchritte6_2 = Math.Abs(InverseKinematicsDeltaStepsArray[5]);
                double[] IKSpeed2 = NextSpeed2(deltaSchritte1_2, deltaSchritte2_2, deltaSchritte3_2, deltaSchritte4_2, deltaSchritte5_2, deltaSchritte6_2, _speed);//hier den maxspeed lassen
                InverseKinematicsSpeedArray = new int[6];
                InverseKinematicsSpeedArray[0] = (int)Math.Abs(Math.Round(IKSpeed2[0], 0));
                InverseKinematicsSpeedArray[1] = (int)Math.Abs(Math.Round(IKSpeed2[1], 0));
                InverseKinematicsSpeedArray[2] = (int)Math.Abs(Math.Round(IKSpeed2[2], 0));
                InverseKinematicsSpeedArray[3] = (int)Math.Abs(Math.Round(IKSpeed2[3], 0));
                InverseKinematicsSpeedArray[4] = (int)Math.Abs(Math.Round(IKSpeed2[4], 0));
                InverseKinematicsSpeedArray[5] = (int)Math.Abs(Math.Round(IKSpeed2[5], 0));
                //Calcs the Speed of each axis
                OnKinematicEvent(new KinematicEventArgs());//Raises OK-Event
            }
        }
        private double[] NextSpeed2(double PositionToReach1, double PositionToReach2, double PositionToReach3, double PositionToReach4, double PositionToReach5, double PositionToReach6, int ActualSpeed)
        {
            //try
            //{
            double StepsPerSecond1 = (16000000 * ActualSpeed / (64 * 2048 * 32 * 32));//64=2^Puls_Div der auf 6 gesetzt ist auf eeprom||32=2^microstep die auf 5 sind
            double StepsPerSecond2 = (16000000 * ActualSpeed / (64 * 2048 * 32 * 32));
            double StepsPerSecond3 = (16000000 * ActualSpeed / (64 * 2048 * 32 * 32));
            double StepsPerSecond4 = (16000000 * ActualSpeed / (64 * 2048 * 32 * 32));
            double StepsPerSecond5 = (16000000 * ActualSpeed / (64 * 2048 * 32 * 32));
            double StepsPerSecond6 = (16000000 * ActualSpeed / (64 * 2048 * 32 * 32));

            double[] TimeToReachPosition = new double[6];//wichtig für Skalierung von NewSpeed
            TimeToReachPosition[0] = Math.Abs(PositionToReach1 / StepsPerSecond1);
            TimeToReachPosition[1] = Math.Abs(PositionToReach2 / StepsPerSecond2);
            TimeToReachPosition[2] = Math.Abs(PositionToReach3 / StepsPerSecond3);
            TimeToReachPosition[3] = Math.Abs(PositionToReach4 / StepsPerSecond4);
            TimeToReachPosition[4] = Math.Abs(PositionToReach5 / StepsPerSecond5);
            TimeToReachPosition[5] = Math.Abs(PositionToReach6 / StepsPerSecond6);


            double[] timearray = new double[6];//zu sortierendes Array. wichtig für Skalierung von NewSpeed
            timearray[0] = Math.Abs(PositionToReach1 / StepsPerSecond1);
            timearray[1] = Math.Abs(PositionToReach2 / StepsPerSecond2);
            timearray[2] = Math.Abs(PositionToReach3 / StepsPerSecond3);
            timearray[3] = Math.Abs(PositionToReach4 / StepsPerSecond4);
            timearray[4] = Math.Abs(PositionToReach5 / StepsPerSecond5);
            timearray[5] = Math.Abs(PositionToReach6 / StepsPerSecond6);


            Array.Sort(timearray);//timearray sortieren, bis der höchste werte zuunterst ist; HöchsterWert=timearray[Letztes Element aus timearray]
            double[] NewSpeed = new double[6];
            for (int c = 0; c < TimeToReachPosition.Length; c++)
            {
                if (TimeToReachPosition[c] != 0)
                {
                    NewSpeed[c] = (ActualSpeed * TimeToReachPosition[c] / timearray[5]);
                }
                else
                {
                    NewSpeed[c] = 0;
                }
                //NewSpeed[0] = (ActualSpeed * TimeToReachPosition[0] / timearray[5]); //timearray[Letztes Element aus timearray] da dies nach sortierung der höchste wert ist
                //NewSpeed[1] = (ActualSpeed * TimeToReachPosition[1] / timearray[5]); //timearray[5] da dies nach sortierung der höchste wert ist
                //NewSpeed[2] = (ActualSpeed * TimeToReachPosition[2] / timearray[5]);
                //NewSpeed[3] = (ActualSpeed * TimeToReachPosition[3] / timearray[5]);
                //NewSpeed[4] = (ActualSpeed * TimeToReachPosition[4] / timearray[5]);
                //NewSpeed[5] = (ActualSpeed * TimeToReachPosition[5] / timearray[5]);
            }
            return NewSpeed;
        }       
        public void OnKinematicEvent(KinematicEventArgs KinEventArgs)
        {
            if (InverseKinematicEvent != null)//checks weather the event has subscribers
            {
                InverseKinematicEvent(this, KinEventArgs);                
                out_of_range_counter = 0;
            }
        }

        public double[] InverseKinematicsData
        {
            get { return InverseKinematicsDataArray; }
        }
        public int[] InverseKinematicsSpeed
        {
            get { return InverseKinematicsSpeedArray; }
        }
        public int[] InverseKinematicsDeltaSteps
        {
            get { return InverseKinematicsDeltaStepsArray; }
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinematics
{
    public sealed class ForewardKinematics : KinematicBase
    {
        public delegate void ForewardKineamaticDelegate(ForewardKinematics wk, KinematicEventArgs e);
        public event ForewardKineamaticDelegate ForewardKinematicEvent;

        //static variables are shared between all class instances.
        private static volatile ForewardKinematics instance;//because it's static, its for all the instances of InverseKinematics tha same Instance. You Can not create a new one, because the Constructor is private
        private static object LockObj = new Object();//this object can only be used from one Thread at the same time (UI-Thread OR Backgroundthread, but not both at a time, can use the same Instance of this Class)
        //Note that instead of locking on typeof(Singleton) as some versions of this implementation do, 
        //I lock on the value of a static variable which is private to the class. Locking on objects
        //which other classes can access and lock on (such as the type of this Class) risks performance issues 
        //and even deadlocks. Wherever possible, only lock on objects specifically created for the purpose of locking, or which document that they are 
        //to be locked on for specific purposes (e.g. for waiting/pulsing a queue). 
        //Usually such objects should be private to the class they are used in. 
        //This helps to make writing thread-safe applications significantly easier.
        public double[] ForewardKinematicsDataArray = new double[6];
        private ForewardKinematics(DHParameter dhparam)
            : base(dhparam)
        {

        }
        /// <summary>
        /// IF the instance has allready been created, the instance created before will be given back, when calling this method. Ohterwise a new instance will be given back
        /// </summary>
        /// <param name="dhparam"></param>
        /// <returns></returns>
        public static ForewardKinematics Instantiate(DHParameter dhparam) //static variables are shared between all class instances.
        {
            if (instance == null)//Prevents of Locking if the instance allready has been created
            {
                lock (LockObj)//no new instance can be createt from any thread as long as the first one onlocks the object
                {
                    if (instance == null)
                    {
                        instance = new ForewardKinematics(dhparam);
                    }
                }
            }
            return instance;// when using this methode, allways the same and only instance will be returned
        }

        public void ForewardKinematic(double t1, double t2, double t3, double t4, double t5, double t6, double beta_)
        {
            double[,] DH_1 = DH(t1, DHParam.D[0], DHParam.A[0], DHParam.Alpha[0]);
            double[,] DH_2 = DH(t2, DHParam.D[1], DHParam.A[1], DHParam.Alpha[1]);
            double[,] DH_3 = DH(t3, DHParam.D[2], DHParam.A[2], DHParam.Alpha[2]);
            double[,] DH_4 = DH(t4, DHParam.D[3], DHParam.A[3], DHParam.Alpha[3]);
            double[,] DH_5 = DH(t5, DHParam.D[4], DHParam.A[4], DHParam.Alpha[4]);
            double[,] DH_6 = DH(t6, DHParam.D[5], DHParam.A[5], DHParam.Alpha[5]);
            double[,] DH_123 = DH123(DH_1, DH_2, DH_3);
            double[,] DH_456 = DH123(DH_4, DH_5, DH_6);
            double[,] DH_123456 = MultiplyMatricesSeriell(DH_123, DH_456);
            ForewardKinematicsDataArray[0] = DH_123456[0, 3];
            ForewardKinematicsDataArray[1] = DH_123456[1, 3];
            ForewardKinematicsDataArray[2] = DH_123456[2, 3];
            if (beta_ <= 0)
            {
                ForewardKinematicsDataArray[3] = Math.Atan2(-DH_123456[0, 2], DH_123456[1, 2]) * 180 / Math.PI;
                //berechnen von beta    
                //gerundet!!!!!!!!!
                ForewardKinematicsDataArray[4] = Math.Atan2(-Math.Sqrt(Math.Round(1 - DH_123456[2, 2] * DH_123456[2, 2], 6)), DH_123456[2, 2]) * 180 / Math.PI;
                ForewardKinematicsDataArray[5] = Math.Atan2(DH_123456[2, 0], -DH_123456[2, 1]) * 180 / Math.PI;
            }
            else
            {
                ForewardKinematicsDataArray[3] = Math.Atan2(DH_123456[0, 2], -DH_123456[1, 2]) * 180 / Math.PI;
                //berechnen von beta    
                //gerundet!!!!!!!!!
                ForewardKinematicsDataArray[4] = Math.Atan2(Math.Sqrt(Math.Round(1 - DH_123456[2, 2] * DH_123456[2, 2], 6)), DH_123456[2, 2]) * 180 / Math.PI;
                ForewardKinematicsDataArray[5] = Math.Atan2(DH_123456[2, 0], DH_123456[2, 1]) * 180 / Math.PI;
            }
            OnKinematicEvent(new KinematicEventArgs());//Raises OK-Event
        }

        public double[] ForewardKinematicsData
        {
            get { return ForewardKinematicsDataArray; }
        }
        public void OnKinematicEvent(KinematicEventArgs KinEventArgs)
        {
            if (ForewardKinematicEvent != null)//checks weather the event has subscribers
            {
                ForewardKinematicEvent(this, KinEventArgs);
            }
        }
    }
}

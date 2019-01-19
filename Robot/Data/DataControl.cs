using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Data
{
    public class DataControl
    {
        //static variables are shared between all class instances.
        private static volatile DataControl instance;//because it's static, its for all the instances of DataControl tha same Instance. You Can not create a new one, because the Constructor is private
        private static object LockObj = new Object();//this object can only be used from one Thread at the same time (UI-Thread OR Backgroundthread, but not both at a time, can use the same Instance of this Class)
        //Note that instead of locking on typeof(Singleton) as some versions of this implementation do, 
        //I lock on the value of a static variable which is private to the class. Locking on objects
        //which other classes can access and lock on (such as the type of this Class) risks performance issues 
        //and even deadlocks. Wherever possible, only lock on objects specifically created for the purpose of locking, or which document that they are 
        //to be locked on for specific purposes (e.g. for waiting/pulsing a queue). 
        //Usually such objects should be private to the class they are used in. 
        //This helps to make writing thread-safe applications significantly easier.
        private NCData ncdata;
        private List<MotorControllerData> motorcontrollerdata;
        private ActualPosition actualposition;
        private DiagnoseData diagnosedata;
        private DataControl()
        {
            ncdata = new NCData();
            motorcontrollerdata = new List<MotorControllerData>();
            motorcontrollerdata.Add(new MotorControllerData());
            motorcontrollerdata.Add(new MotorControllerData());
            motorcontrollerdata.Add(new MotorControllerData());
            motorcontrollerdata.Add(new MotorControllerData());
            motorcontrollerdata.Add(new MotorControllerData());
            motorcontrollerdata.Add(new MotorControllerData());
            actualposition = new ActualPosition();
            diagnosedata = new DiagnoseData();
        }
        /// <summary>
        /// IF the instance has allready been created, the instance created before will be given back, when calling this method. Ohterwise a new instance will be given back
        /// </summary>
        /// <param name="dhparam"></param>
        /// <returns></returns>
        public static DataControl Instantiate()
        {
            if (instance == null)//Prevents of Locking if the instance allready has been created
            {
                lock (LockObj)//no new instance can be createt from any thread as long as the first one onlocks the object
                {
                    if (instance == null)
                    {
                        instance = new DataControl();
                    }
                }
            }
            return instance;
        }
        public NCData NCData
        {
            get
            { 
                return ncdata; 
            }
            set
            {
                ncdata = value;
            }

        }
        public List<MotorControllerData> MotorControllerData
        {
            get
            {
                return motorcontrollerdata;
            }
            set
            {
                motorcontrollerdata = value;
            }

        }
        public ActualPosition ActualPosition
        {
            get
            {
                return actualposition;
            }
            set
            {
                actualposition = value;
            }

        }
        public DiagnoseData DiagnoseData
        {
            get
            {
                return diagnosedata;
            }
            set
            {
                diagnosedata = value;
            }

        }
    }
}

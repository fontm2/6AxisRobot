using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.IO.Ports;

namespace SerialPortHandler
{
    public class SerialPortHandle : IDisposable
    {
        //static variables are shared between all class instances.
        private static volatile SerialPortHandle instance = null;//because it's static, its for all the instances of InverseKinematics tha same Instance. You Can not create a new one, because the Constructor is private
        private static object LockObj = new Object();//this object can only be used from one Thread at the same time (UI-Thread OR Backgroundthread, but not both at a time, can use the same Instance of this Class)
        //Note that instead of locking on typeof(Singleton) as some versions of this implementation do, 
        //I lock on the value of a static variable which is private to the class. Locking on objects
        //which other classes can access and lock on (such as the type of this Class) risks performance issues 
        //and even deadlocks. Wherever possible, only lock on objects specifically created for the purpose of locking, or which document that they are 
        //to be locked on for specific purposes (e.g. for waiting/pulsing a queue). 
        //Usually such objects should be private to the class they are used in. 
        //This helps to make writing thread-safe applications significantly easier.
        public delegate void SerialPortIsInitioalized(SerialPortHandle SerialPortHandle, SerialPortInitEventArgs e);
        public event SerialPortIsInitioalized InitializedEvent;
        public SerialPortInitEventArgs SeriealPortInitEventArgs;
        private int k;
        private string[] COM_Name = new string[6];
        private List<MotorController> MotorControllers = new List<MotorController>();
        private string[] datarecieved = new string[6];
        private bool disposed = false;

        private SerialPortHandle()
        {

        }
        /// <summary>
        /// IF the instance has allready been created, the instance created before will be given back, when calling this method. Ohterwise a new instance will be given back
        /// </summary>
        /// <param name="dhparam"></param>
        /// <returns></returns>
        public static SerialPortHandle Instantiate()
        {
            if (instance == null)//Prevents of Locking if the instance allready has been created
            {
                lock (LockObj)//no new instance can be createt from any thread as long as the first one onlocks the object
                {
                    if (instance == null)
                    {
                        instance = new SerialPortHandle();
                    }
                }
            }
            return instance;
        }
        ~SerialPortHandle()
        {
            Dispose(false);
        }
        public void Dispose()
        {
            Dispose(true);         
            GC.SuppressFinalize(this);
        }
        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                if (disposing)
                {                    
                    foreach(MotorController controller in MotorControllers)
                    {
                        instance = null;
                        controller.Close();
                        controller.Dispose();
                    }
                    MotorControllers.Clear();
                    MotorControllers = null;
                }             
                disposed = true;
            }
        }
        /// <summary>
        /// Creates a new List of MotorControllers
        /// </summary>
        public void CreateNew()
        {
            k = 0;
            string[] COM_Names = SerialPort.GetPortNames();
            for (int i = 0; i < COM_Names.Length; i++)
            {
                MotorControllers.Add(new MotorController(COM_Names[i], 115400));
            }
            if (COM_Names.Length > 0)
            {
                Initialize();
            }
            else
            {
                OnInitializedEvent("COM-Ports womöglich nicht angeschlossen \n" + "angeschlossene Ports: " + MotorControllers.Count.ToString());
                foreach (MotorController controller in MotorControllers)
                {
                    controller.Close();
                }
                MotorControllers.Clear();
            }
        }       
        private void Initialize()
        {
            if (k < MotorControllers.Count)
            {
                MotorControllers[k].ReadExisting();//clear buffer
                MotorControllers[k].Write(GetCommandByteArray(0x01, 0x03, 0x00, 0x00, 0), 0, GetCommandByteArray(0x01, 0x03, 0x00, 0x00, 0).Length);
                Thread.Sleep(10);
                dataread();
            }
            else
            {
                foreach (MotorController port in MotorControllers)
                {
                    port.Close();
                }
                if (k == MotorControllers.Count - 1)
                {
                    MotorControllers.Clear();
                    MotorControllers = null;
                    CreateNewMotorControllers();
                    OnInitializedEvent("Alle COM-Ports gefunden \nCOM-MotorControllerse bereit unter Port-Liste");
                }
                else
                {
                    OnInitializedEvent("COM-Ports nur unvollständig gefunden");
                    MotorControllers.Clear();
                    MotorControllers = null;
                }
            }         
        }
        private void dataread()
        {
            byte[] inbuffer = new byte[9];          
            MotorControllers[k].Read(inbuffer, 0, 9);      
            if (inbuffer[1] == 0x01)
            {
                COM_Name[0] = MotorControllers[k].PortName;
                datarecieved[0] = "HostAdress: " + inbuffer[0].ToString() + "; SlaveAdress:" + inbuffer[1].ToString() + "; Status:" + inbuffer[2].ToString();
                Thread.Sleep(500);
                k++;
            }
            else if (inbuffer[1] == 0x02)
            {
                COM_Name[1] = MotorControllers[k].PortName;
                datarecieved[1] = "HostAdress: " + inbuffer[0].ToString() + "; SlaveAdress:" + inbuffer[1].ToString() + "; Status:" + inbuffer[2].ToString();
                Thread.Sleep(500);
                k++;
            }
            else if (inbuffer[1] == 0x03)
            {
                COM_Name[2] = MotorControllers[k].PortName;
                datarecieved[2] = "HostAdress: " + inbuffer[0].ToString() + "; SlaveAdress:" + inbuffer[1].ToString() + "; Status:" + inbuffer[2].ToString();
                Thread.Sleep(500);
                k++;
            }
            else if (inbuffer[1] == 0x04)
            {
                COM_Name[3] = MotorControllers[k].PortName;
                datarecieved[3] = "HostAdress: " + inbuffer[0].ToString() + "; SlaveAdress:" + inbuffer[1].ToString() + "; Status:" + inbuffer[2].ToString();
                Thread.Sleep(500);
                k++;
            }
            else if (inbuffer[1] == 0x05)
            {
                COM_Name[4] = MotorControllers[k].PortName;
                datarecieved[4] = "HostAdress: " + inbuffer[0].ToString() + "; SlaveAdress:" + inbuffer[1].ToString() + "; Status:" + inbuffer[2].ToString();
                Thread.Sleep(500);
                k++;
            }
            else if (inbuffer[1] == 0x06)
            {
                COM_Name[5] = MotorControllers[k].PortName;
                datarecieved[5] = "HostAdress: " + inbuffer[0].ToString() + "; SlaveAdress:" + inbuffer[1].ToString() + "; Status:" + inbuffer[2].ToString();
                Thread.Sleep(500);
                k++;
            }
            Initialize();           
        }
        private byte[] GetCommandByteArray(byte Adress, byte Command, byte Typ, byte Motor, int Value)
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

            return buffer;
        }
        private void CreateNewMotorControllers()
        {
            for (int i = 0; i < COM_NAMES.Length; i++)
            {
                MotorControllers.Add(new MotorController(COM_NAMES[i], 115400));
            }          
        }
        public virtual void OnInitializedEvent(string str)
        {
            if (InitializedEvent != null)//if there are subscribers
            {
                InitializedEvent(this, new SerialPortInitEventArgs(str));
            }
        }
        //property
        public string[] COM_NAMES
        {
            get { return COM_Name; }
        }
        public string[] RecievedData
        {
            get { return datarecieved; }
        }
        public List<MotorController> MotorController
        {
            get 
            { return MotorControllers; }
        }
    }   
}

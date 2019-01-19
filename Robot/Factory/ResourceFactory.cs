using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SerialPortHandler;
using Kinematics;
using Data;
using NC;
using Simulation;

namespace Factory
{
    public static class ResourceFactory
    {
        /// <summary>
        /// Creates an Instanze of the SerialPortHanlde Class which can handle MotorControllers as SerialPorts
        /// </summary>
        /// <returns>MotorController: SerialPort</returns>
        public static SerialPortHandle GetMotorControllerMaster()
        {
            return SerialPortHandle.Instantiate();
        }
        /// <summary>
        /// denavit hartenberg are Robot-related Geometrical Data
        /// </summary>
        /// <param name="a">Lenght in x-Axis</param>
        /// <param name="alpha">Angle around x-Axis</param>
        /// <param name="d">Lenght in z-Axis</param>
        /// <param name="robottype">Type of the Robot</param>
        /// <returns>Returns a Instanze of DHParamter with Property A, Alpha, D as DoubleArrays</returns>
        public static DHParameter SetDHParameter(double[] a, double[] alpha, double[] d, RobotType robottype)
        {
            return new DHParameter(robottype) { A = a, Alpha = alpha, D = d };
        }
        /// <summary>
        /// The KinematicControl Instnze will Handle the InverseKinematics and the ForewardKinematics
        /// </summary>
        /// <param name="dhparam"></param>
        /// <returns></returns>
        public static KinematicControl GetKinematicControl(DHParameter dhparam, ActualPosition _ActualPosition)
        {
            return KinematicControl.Instantiate(dhparam, _ActualPosition);
        }
        /// <summary>
        /// The DataControl Instante will Handle all the Data needed
        /// </summary>
        /// <returns></returns>
        public static DataControl GetDataControl()
        {
            return DataControl.Instantiate();
        }
        /// <summary>
        /// Creates an empty ObservableCollection with one Sub-Collection for SubNodes
        /// </summary>
        /// <returns></returns>
        public static TreeViewCollection GetTreeViewCollection()
        {
            return new TreeViewCollection();
        }
        /// <summary>
        /// The NumericalControl Instanze handles everything for the Numerical Control Part
        /// </summary>
        /// <param name="indata"></param>
        /// <param name="millis"></param>
        /// <param name="ende"></param>
        /// <param name="DHParam"></param>
        /// <returns></returns>
        public static List<NumericalControl> AddNumericalControlSemaphor(List<string> indata, int millis, int ende, DHParameter DHParam)
        {
            return NumericalControl.InstantiateSemaphor(indata, millis, ende, DHParam);
        }
        /// <summary>
        /// Returns a Model of the Robot
        /// </summary>
        /// <param name="dhparam"></param>
        /// <returns></returns>
        public static Simulation3D GetModel(string FileName1, string FileName2, string FileName3, string FileName4, string FileName5, string FileName6)
        {
            return Simulation3D.Instantiate(FileName1, FileName2, FileName3, FileName4, FileName5, FileName6);
        }
    }
}

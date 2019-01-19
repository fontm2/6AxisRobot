using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;//add a reference to PresentationCore.dll (for imageprocessing to)
using System.Windows.Media.Media3D;
using System.Windows.Controls;//add a reference to PresentationFramework.dll 
using System.Drawing;// add a reference to System.Drawing.dll
//Wichtig für XamlReader
using System.Windows.Markup;
using System.Xml;
using System.Xml.Serialization;
using System.IO;

namespace Simulation
{
    public class Simulation3D
    {

        #region Simulation_Members
        //allgemeine einstellungen
        private PerspectiveCamera Cam;
        private Point3D CamPosition = new Point3D(1, 0, 0);
        private Vector3D CamLookDirection = new Vector3D(-1, 0, 0);
        private int CamFieldOfView;
        private DirectionalLight LightSource1;
        private Vector3D LightDirection1 = new Vector3D(-1, 0, 0);
        //Drawn Line
        public Point3D[] Linepoint3D_Array;
        public ModelVisual3D _3DModel_Line = new ModelVisual3D();
        public Model3DGroup Geometry3DContainer_Line = new Model3DGroup();
        public List<GeometryModel3D> GeometryModel3d_Line = new List<GeometryModel3D>();

        //GCodeSimulation
        int GCodeSimulationsCounter = 0;
        int GCodeSimulationsCounter2 = 0;

        private ModelVisual3D HighestModel;
        private ModelVisual3D LightConteiner;
        private Model3DGroup Lightsource;
        private ModelVisual3D model1;
        private ModelVisual3D model2;
        private ModelVisual3D model3;
        private ModelVisual3D model4;
        private ModelVisual3D model5;
        private ModelVisual3D model6;
        private Point3D RotationCenter2;
        private Point3D RotationCenter3;
        private Point3D RotationCenter4;
        private Point3D RotationCenter5;
        private Point3D RotationCenter6;
        private Point3D RotationCenter7;

        //Slider
        private double SliderValue;
        private double SliderCamRotationAngle;
        private Vector3D y_achse = new Vector3D(0, 1, 0);
        private AxisAngleRotation3D SliderAxisRotation;
        private RotateTransform3D SliderAxisTransform;
        private Point3D CamPosition_rotated = new Point3D(0, 0, 0);
        private Vector3D CamLookDirection_rotated = new Vector3D();
        private Vector3D LightDirection1_rotated = new Vector3D();

        private double SliderValue_Over;
        private double SliderCamRotationAngle_Over;
        private AxisAngleRotation3D SliderAxisRotation_Over;
        private RotateTransform3D SliderAxisTransform_Over;
        private Point3D CamPosition_rotated_Over = new Point3D(0, 0, 0);
        private Vector3D CamLookDirection_rotated_Over = new Vector3D();
        private Vector3D LightDirection1_rotated_Over = new Vector3D();

        private double SliderValue_Zoom;
        #endregion

        //static variables are shared between all class instances.
        private static volatile Simulation3D instance;//because it's static, its for all the instances of InverseKinematics tha same Instance. You Can not create a new one, because the Constructor is private
        private static object LockObj = new Object();//this object can only be used from one Thread at the same time (UI-Thread OR Backgroundthread, but not both at a time, can use the same Instance of this Class)
        //Note that instead of locking on typeof(Singleton) as some versions of this implementation do, 
        //I lock on the value of a static variable which is private to the class. Locking on objects
        //which other classes can access and lock on (such as the type of this Class) risks performance issues 
        //and even deadlocks. Wherever possible, only lock on objects specifically created for the purpose of locking, or which document that they are 
        //to be locked on for specific purposes (e.g. for waiting/pulsing a queue). 
        //Usually such objects should be private to the class they are used in. 
        //This helps to make writing thread-safe applications significantly easier.
        private Simulation3D(string FileName1, string FileName2, string FileName3, string FileName4, string FileName5, string FileName6)        
        {
            LightSource1 = new DirectionalLight(Colors.WhiteSmoke, new Vector3D(-1, -0.5, -1));
            HighestModel = new ModelVisual3D();

            RotationCenter2 = new Point3D(0, 0.365, 0);
            RotationCenter3 = new Point3D(0.07, 0.365, 0);
            RotationCenter4 = new Point3D(-0.03, 0.365, 0.250);
            RotationCenter5 = new Point3D(0, 0.365, 0.405);
            RotationCenter6 = new Point3D(-0.0475, 0.365, 0.475);
            RotationCenter7 = new Point3D(0, 0.365, 0.620);


            Transform3DGroup TrafoGroup2 = new Transform3DGroup();
            AxisAngleRotation3D axisAngle2 = new AxisAngleRotation3D(new Vector3D(0, 1, 0), 40);
            RotateTransform3D myRotateTransform2 = new RotateTransform3D(axisAngle2, RotationCenter2);
            TrafoGroup2.Children.Add(myRotateTransform2);

            Transform3DGroup TrafoGroup3 = new Transform3DGroup();
            Vector3D axis3_rotated = TrafoGroup2.Transform(new Vector3D(1, 0, 0));
            AxisAngleRotation3D axisAngle3 = new AxisAngleRotation3D(axis3_rotated, 0);
            Point3D RotationCenter3_Transformed = new Point3D();
            RotationCenter3_Transformed = TrafoGroup2.Transform(RotationCenter3);
            RotateTransform3D myRotateTransform3 = new RotateTransform3D(axisAngle3, RotationCenter3_Transformed);
            TrafoGroup3.Children.Add(myRotateTransform2);
            TrafoGroup3.Children.Add(myRotateTransform3);


            Transform3DGroup TrafoGroup4 = new Transform3DGroup();
            Vector3D axis4_rotated = TrafoGroup3.Transform(new Vector3D(1, 0, 0));
            AxisAngleRotation3D axisAngle4 = new AxisAngleRotation3D(axis4_rotated, -90);
            Point3D RotationCenter4_Transformed = new Point3D();
            RotationCenter4_Transformed = TrafoGroup3.Transform(RotationCenter4);
            RotateTransform3D myRotateTransform4 = new RotateTransform3D(axisAngle4, RotationCenter4_Transformed);
            TrafoGroup4.Children.Add(myRotateTransform2);
            TrafoGroup4.Children.Add(myRotateTransform3);
            TrafoGroup4.Children.Add(myRotateTransform4);


            Transform3DGroup TrafoGroup5 = new Transform3DGroup();
            Vector3D axis5_rotated = TrafoGroup4.Transform(new Vector3D(0, 0, 1));
            AxisAngleRotation3D axisAngle5 = new AxisAngleRotation3D(axis5_rotated, 0);
            Point3D RotationCenter5_Transformed = new Point3D();
            RotationCenter5_Transformed = TrafoGroup4.Transform(RotationCenter5);
            RotateTransform3D myRotateTransform5 = new RotateTransform3D(axisAngle5, RotationCenter5_Transformed);
            TrafoGroup5.Children.Add(myRotateTransform2);
            TrafoGroup5.Children.Add(myRotateTransform3);
            TrafoGroup5.Children.Add(myRotateTransform4);
            TrafoGroup5.Children.Add(myRotateTransform5);


            Transform3DGroup TrafoGroup6 = new Transform3DGroup();
            Vector3D axis6_rotated = TrafoGroup5.Transform(new Vector3D(1, 0, 0));
            AxisAngleRotation3D axisAngle6 = new AxisAngleRotation3D(axis6_rotated, 0);
            Point3D RotationCenter6_Transformed = new Point3D();
            RotationCenter6_Transformed = TrafoGroup5.Transform(RotationCenter6);
            RotateTransform3D myRotateTransform6 = new RotateTransform3D(axisAngle6, RotationCenter6_Transformed);
            TrafoGroup6.Children.Add(myRotateTransform2);
            TrafoGroup6.Children.Add(myRotateTransform3);
            TrafoGroup6.Children.Add(myRotateTransform4);
            TrafoGroup6.Children.Add(myRotateTransform5);
            TrafoGroup6.Children.Add(myRotateTransform6);

            Cam = new PerspectiveCamera();
            Cam.Position = CamPosition;
            Cam.LookDirection = CamLookDirection;
            //lichquelle manuell erzeugen in den übergeordnetsten ModelViusal3D, wo alle Teile (1 bis 6) dirn sind
            LightConteiner = new ModelVisual3D();
            Lightsource = new Model3DGroup();
            Lightsource.Children.Add(LightSource1);
            LightConteiner.Content = Lightsource;

            //importieren der Modele von Teil 1 bis 6 und Transformieren
            model1 = Load_XAML_ModelVisual3D(FileName1);
            model2 = Load_XAML_ModelVisual3D(FileName2);
            model2.Transform = TrafoGroup2;
            model3 = Load_XAML_ModelVisual3D(FileName3);
            model3.Transform = TrafoGroup3;
            model4 = Load_XAML_ModelVisual3D(FileName4);
            model4.Transform = TrafoGroup4;
            model5 = Load_XAML_ModelVisual3D(FileName5);
            model5.Transform = TrafoGroup5;
            model6 = Load_XAML_ModelVisual3D(FileName6);
            model6.Transform = TrafoGroup6;

            //alle Modelle und die Lichtquelle in den Haupt-Modell-Container Packen
            HighestModel.Children.Add(model1);
            HighestModel.Children.Add(model2);
            HighestModel.Children.Add(model3);
            HighestModel.Children.Add(model4);
            HighestModel.Children.Add(model5);
            HighestModel.Children.Add(model6);
            HighestModel.Children.Add(LightConteiner);

            //initialize Slyder_CamRotationen
            SliderAxisRotation = new AxisAngleRotation3D(y_achse, 0);
            SliderAxisTransform = new RotateTransform3D(SliderAxisRotation);
            SliderAxisRotation_Over = new AxisAngleRotation3D(y_achse, 0);
            SliderAxisTransform_Over = new RotateTransform3D(SliderAxisRotation_Over);
        }
        public static Simulation3D Instantiate(string FileName1, string FileName2, string FileName3, string FileName4, string FileName5, string FileName6) //static variables are shared between all class instances.
        {
            if (instance == null)//Prevents of Locking if the instance allready has been created
            {
                lock (LockObj)//no new instance can be createt from any thread as long as the first one onlocks the object
                {
                    if (instance == null)
                    {
                        instance = new Simulation3D(FileName1, FileName2, FileName3, FileName4, FileName5, FileName6);
                    }
                }
            }
            return instance;// when using this methode, allways the same and only instance will be returned
        }
        public void MoveSimulation(double angle_simulation0, double angle_simulation1, double angle_simulation2, double angle_simulation3, double angle_simulation4, double angle_simulation5)
        {
            HighestModel.Children.Clear();


            Transform3DGroup TrafoGroup2 = new Transform3DGroup();
            AxisAngleRotation3D axisAngle2 = new AxisAngleRotation3D(new Vector3D(0, 1, 0), angle_simulation0);
            RotateTransform3D myRotateTransform2 = new RotateTransform3D(axisAngle2, RotationCenter2);
            TrafoGroup2.Children.Add(myRotateTransform2);

            Transform3DGroup TrafoGroup3 = new Transform3DGroup();
            Vector3D axis3_rotated = TrafoGroup2.Transform(new Vector3D(1, 0, 0));
            AxisAngleRotation3D axisAngle3 = new AxisAngleRotation3D(axis3_rotated, angle_simulation1);
            Point3D RotationCenter3_Transformed = new Point3D();
            RotationCenter3_Transformed = TrafoGroup2.Transform(RotationCenter3);
            RotateTransform3D myRotateTransform3 = new RotateTransform3D(axisAngle3, RotationCenter3_Transformed);
            TrafoGroup3.Children.Add(myRotateTransform2);
            TrafoGroup3.Children.Add(myRotateTransform3);


            Transform3DGroup TrafoGroup4 = new Transform3DGroup();
            Vector3D axis4_rotated = TrafoGroup3.Transform(new Vector3D(1, 0, 0));
            AxisAngleRotation3D axisAngle4 = new AxisAngleRotation3D(axis4_rotated, -90 + angle_simulation2);
            Point3D RotationCenter4_Transformed = new Point3D();
            RotationCenter4_Transformed = TrafoGroup3.Transform(RotationCenter4);
            RotateTransform3D myRotateTransform4 = new RotateTransform3D(axisAngle4, RotationCenter4_Transformed);
            TrafoGroup4.Children.Add(myRotateTransform2);
            TrafoGroup4.Children.Add(myRotateTransform3);
            TrafoGroup4.Children.Add(myRotateTransform4);


            Transform3DGroup TrafoGroup5 = new Transform3DGroup();
            Vector3D axis5_rotated = TrafoGroup4.Transform(new Vector3D(0, 0, 1));
            AxisAngleRotation3D axisAngle5 = new AxisAngleRotation3D(axis5_rotated, angle_simulation3);
            Point3D RotationCenter5_Transformed = new Point3D();
            RotationCenter5_Transformed = TrafoGroup4.Transform(RotationCenter5);
            RotateTransform3D myRotateTransform5 = new RotateTransform3D(axisAngle5, RotationCenter5_Transformed);
            TrafoGroup5.Children.Add(myRotateTransform2);
            TrafoGroup5.Children.Add(myRotateTransform3);
            TrafoGroup5.Children.Add(myRotateTransform4);
            TrafoGroup5.Children.Add(myRotateTransform5);


            Transform3DGroup TrafoGroup6 = new Transform3DGroup();
            Vector3D axis6_rotated = TrafoGroup5.Transform(new Vector3D(1, 0, 0));
            AxisAngleRotation3D axisAngle6 = new AxisAngleRotation3D(axis6_rotated, angle_simulation4);
            Point3D RotationCenter6_Transformed = new Point3D();
            RotationCenter6_Transformed = TrafoGroup5.Transform(RotationCenter6);
            RotateTransform3D myRotateTransform6 = new RotateTransform3D(axisAngle6, RotationCenter6_Transformed);
            TrafoGroup6.Children.Add(myRotateTransform2);
            TrafoGroup6.Children.Add(myRotateTransform3);
            TrafoGroup6.Children.Add(myRotateTransform4);
            TrafoGroup6.Children.Add(myRotateTransform5);
            TrafoGroup6.Children.Add(myRotateTransform6);


            //importieren der Modele von Teil 1 bis 6 und Transformieren

            model2.Transform = TrafoGroup2;

            model3.Transform = TrafoGroup3;

            model4.Transform = TrafoGroup4;

            model5.Transform = TrafoGroup5;

            model6.Transform = TrafoGroup6;

            //alle Modelle und die Lichtquelle in den Haupt-Modell-Container Packen
            HighestModel.Children.Add(model1);
            HighestModel.Children.Add(model2);
            HighestModel.Children.Add(model3);
            HighestModel.Children.Add(model4);
            HighestModel.Children.Add(model5);
            HighestModel.Children.Add(model6);
            HighestModel.Children.Add(LightConteiner);

        }
        private ModelVisual3D Load_XAML_ModelVisual3D(string filename__)
        {
            ModelVisual3D LoadedModel;
            StreamReader textReader = new StreamReader(filename__);
            XmlReader xmlReader = XmlReader.Create(textReader);
            LoadedModel = (ModelVisual3D)XamlReader.Load(xmlReader);
            textReader.Close();
            return LoadedModel;
        }
        public void SliderZoomChange(double slidervalue)
        {
            //Slider_Zoom
            SliderValue_Zoom = slidervalue;
            SliderValue_Zoom = SliderValue_Zoom / 4;
            CamPosition = new Point3D(SliderValue_Zoom, 0, 0);
            //Slider_XZ
            SliderAxisRotation = new AxisAngleRotation3D(y_achse, SliderCamRotationAngle);
            SliderAxisTransform = new RotateTransform3D(SliderAxisRotation);
            CamPosition_rotated = SliderAxisTransform.Transform(CamPosition);
            CamLookDirection_rotated = SliderAxisTransform.Transform(CamLookDirection);
            LightDirection1_rotated = SliderAxisTransform.Transform(LightDirection1);
            //Slider_Over
            Vector3D CamPositionVector = new Vector3D(CamPosition_rotated.X, CamPosition_rotated.Y, CamPosition_rotated.Z);
            Vector3D RotationaxisofSlider_Over = MathForSimulation.CrossP(CamPositionVector, y_achse);
            SliderAxisRotation_Over = new AxisAngleRotation3D(RotationaxisofSlider_Over, SliderCamRotationAngle_Over);
            SliderAxisTransform_Over = new RotateTransform3D(SliderAxisRotation_Over);
            CamPosition_rotated_Over = SliderAxisTransform_Over.Transform(CamPosition_rotated);
            Cam.Position = CamPosition_rotated_Over;
            CamLookDirection_rotated_Over = SliderAxisTransform_Over.Transform(CamLookDirection_rotated);
            Cam.LookDirection = CamLookDirection_rotated_Over;
            LightDirection1_rotated_Over = SliderAxisTransform_Over.Transform(LightDirection1_rotated);
            LightSource1.Direction = LightDirection1_rotated_Over;
        }
        public void SliderXZCahnge(double slidervalue)
        {
            //Slider_Zoom
            CamPosition = new Point3D(SliderValue_Zoom, 0, 0);

            //Slider_XZ
            SliderValue = slidervalue;
            SliderCamRotationAngle = 36 * SliderValue;
            SliderAxisRotation = new AxisAngleRotation3D(y_achse, SliderCamRotationAngle);
            SliderAxisTransform = new RotateTransform3D(SliderAxisRotation);
            CamPosition_rotated = SliderAxisTransform.Transform(CamPosition);
            CamLookDirection_rotated = SliderAxisTransform.Transform(CamLookDirection);
            LightDirection1_rotated = SliderAxisTransform.Transform(LightDirection1);

            //Slider_Over
            Vector3D CamPositionVector = new Vector3D(CamPosition_rotated.X, CamPosition_rotated.Y, CamPosition_rotated.Z);
            Vector3D RotationaxisofSlider_Over = MathForSimulation.CrossP(CamPositionVector, y_achse);
            SliderAxisRotation_Over = new AxisAngleRotation3D(RotationaxisofSlider_Over, SliderCamRotationAngle_Over);
            SliderAxisTransform_Over = new RotateTransform3D(SliderAxisRotation_Over);
            CamPosition_rotated_Over = SliderAxisTransform_Over.Transform(CamPosition_rotated);
            Cam.Position = CamPosition_rotated_Over;
            CamLookDirection_rotated_Over = SliderAxisTransform_Over.Transform(CamLookDirection_rotated);
            Cam.LookDirection = CamLookDirection_rotated_Over;
            LightDirection1_rotated_Over = SliderAxisTransform_Over.Transform(LightDirection1_rotated);
            LightSource1.Direction = LightDirection1_rotated_Over;
        }
        public void SliderOverChange(double slidervalue)
        {
            //Slider_Zoom
            CamPosition = new Point3D(SliderValue_Zoom, 0, 0);

            //Slider_Over
            SliderValue_Over = slidervalue;
            SliderCamRotationAngle_Over = 9 * SliderValue_Over;
            Vector3D CamPositionVector = new Vector3D(CamPosition.X, CamPosition.Y, CamPosition.Z);
            Vector3D RotationaxisofSlider_Over = MathForSimulation.CrossP(CamPositionVector, y_achse);
            SliderAxisRotation_Over = new AxisAngleRotation3D(RotationaxisofSlider_Over, SliderCamRotationAngle_Over);
            SliderAxisTransform_Over = new RotateTransform3D(SliderAxisRotation_Over);
            CamPosition_rotated_Over = SliderAxisTransform_Over.Transform(CamPosition);
            CamLookDirection_rotated_Over = SliderAxisTransform_Over.Transform(CamLookDirection);
            LightDirection1_rotated_Over = SliderAxisTransform_Over.Transform(LightDirection1);

            //Slider_XZ Anwendung
            SliderAxisRotation = new AxisAngleRotation3D(y_achse, SliderCamRotationAngle);
            SliderAxisTransform = new RotateTransform3D(SliderAxisRotation);
            CamPosition_rotated = SliderAxisTransform.Transform(CamPosition_rotated_Over);
            Cam.Position = CamPosition_rotated;
            CamLookDirection_rotated = SliderAxisTransform.Transform(CamLookDirection_rotated_Over);
            Cam.LookDirection = CamLookDirection_rotated;
            LightDirection1_rotated = SliderAxisTransform.Transform(LightDirection1_rotated_Over);
            LightSource1.Direction = LightDirection1_rotated;
        }
        public ModelVisual3D Model
        {
            get { return HighestModel; }
        }
        public Vector3D LightDirection
        {
            get { return LightDirection1; }
            set { LightDirection1 = value; }
        }
        public DirectionalLight LightSource
        {
            get { return LightSource1; }
            set { LightSource1 = value; }
        }
        public PerspectiveCamera Camera
        {
            get { return Cam; }
            set { Cam = value; }
        }
        public Point3D CameraPosition
        {
            get { return CamPosition; }
            set { CamPosition = value; }
        }
        public Vector3D CameraLookDirection
        {
            get { return CamLookDirection; }
            set { CamLookDirection = value; }
        }
    }
}

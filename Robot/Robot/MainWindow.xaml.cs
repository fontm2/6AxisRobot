using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
//using Microsoft.Kinect;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Threading;
using System.Xml.Serialization;
using System.IO;
using System.Windows.Threading;
using System.Collections.ObjectModel;
using System.Windows.Controls.Primitives;
using System.Collections;
using System.Diagnostics;
using System.Windows.Media.Media3D;

//Important for XamlReader
using System.Windows.Markup;
using System.Xml;

using SerialPortHandler;
using Factory;
using Kinematics;
using Data;
using NC;
using FileHandler;
using Simulation;

namespace Robot
{
    public partial class MainWindow : Window
    {     
        #region Members
        public static RoutedCommand TicTacToeCommand;
        public static RoutedCommand NormalCommand;
        public static RoutedCommand SimulationCommand;
        Mode_ mode;

        private BitmapImage BitMapImage; //Ein BitmapSource
        private static string KeinBild_Picture =  @"F:\VisualStudio\Robot_Aufgeraeumt_4\KeinBild.bmp";

        int[] aktuelleposition = new int[6];
        ///////Datenverwaltung
        private ObservableCollection<NCData> NCDataCollection = new ObservableCollection<NCData>();
        private ObservableCollection<DiagnoseData> DiagnoseDataCollection = new ObservableCollection<DiagnoseData>();

        TreeViewCollection TreeViewCollection;
        string SelectedNode;
        int defaultTreeViewLabeFontSize = 15;
        string Modul_statusstring; //statusanzeige beim obersten treeviewknoten "Module". Wird geeändert auf online wenn Module initialisisert
        UniformGrid row1 = new UniformGrid();
        UniformGrid row2 = new UniformGrid();
        UniformGrid row3 = new UniformGrid();
        UniformGrid row4 = new UniformGrid();
        UniformGrid row5 = new UniformGrid();
        UniformGrid row6 = new UniformGrid();
        UniformGrid row7 = new UniformGrid();
        UniformGrid row8 = new UniformGrid();
        UniformGrid row9 = new UniformGrid();
        UniformGrid row10 = new UniformGrid();
        UniformGrid row11 = new UniformGrid();
        UniformGrid row12 = new UniformGrid();      
        List<Label> TreeViewLables = new List<Label>();
        SolidColorBrush defaultLabelColor = new SolidColorBrush(Colors.White);

        ///////Datagrid; G-Code
        //G-Code
        public bool G_CodeOn;
        string line;
        DiagnoseTable DiagnoseTable;
        bool Start = false;
        List<string> list = new List<string>();
        ArrayList GCodeListe = new ArrayList();
        ArrayList GCodeDeltaPos = new ArrayList();
        ArrayList GCodeListeSpeed = new ArrayList();
        ArrayList GCodeWinkel = new ArrayList();
        ArrayList DataToSend = new ArrayList();
        ArrayList GXD = new ArrayList();
        ArrayList GYD = new ArrayList();
        ArrayList GZD = new ArrayList();
        ArrayList GAD = new ArrayList();
        ArrayList GBD = new ArrayList();
        ArrayList GCD = new ArrayList();
        ArrayList GDiagStr = new ArrayList();
        ArrayList GDiagStrThreadNr = new ArrayList();
        int GCodeListCounter = 0;//zähler zum code-senden
        int GCodeListSpeedCounter = 0;//zähler zum code-senden
        int p = 0; //zähler für GCodeListe in xml zu schreiben
        int pos = 0; //zähler für GCodeListe in xml zu schreiben
        Stopwatch stopwatch = new Stopwatch();

        bool v_activ = false;
        bool u_activ = false;
        bool y_activ = false;
        bool alpha_activ = false;
        bool beta_activ = false;
        bool gamma_activ = false;
        DispatcherTimer timerIK = new DispatcherTimer();
        DispatcherTimer GCodeAblaufTimer = new DispatcherTimer();//tick2 aboniert
        //eventhandler der timer im MainWindow() aboniert
        //SerialPortRobot port1;
        //SerialMot1 Port1;
        //SerialMot2 Port2;
        //SerialMot3 Port3;
        //SerialMot4 Port4;
        //SerialMot5 Port5;
        //SerialMot6 Port6;
        //SerialPortInit PortInit;
        ////Klassen initialisieren--->weiter in MainWindow()
        //WinkelRechner WK;
        //AktuellePosition aktuelle_position;
        //ModulData modul_data1;
        //ModulData modul_data2;
        //ModulData modul_data3;
        //ModulData modul_data4;
        //ModulData modul_data5;
        //ModulData modul_data6;
        //ZeitGeschwindigkeit ZG;
        //GCodeRechner GCalc1;
        //GCodeRechner2 GCalc2;
        private SerialPortHandle MotorControllerMaster = null;
        private DHParameter DHParam = null;
        private KinematicControl KinematicControl = null;
        private List<NumericalControl> NCControls = null;
        private DataControl DataControl = null;   
        public string filename;// String filename = "D:\\RoboterPositionen\\Daten.xml";
        private byte[] Addresses = new byte[] { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
        public byte Motor0 = 0x00;
        public byte Motor1 = 0x01;
        public byte Motor2 = 0x02;
        //TMCL Befehle
        public byte TMCL_ROR = 0x01;
        public byte TMCL_ROL = 0x02;
        public byte TMCL_MST = 0x03;
        public byte TMCL_MVP = 0x04;
        public byte TMCL_SAP = 0x05;
        public byte TMCL_GAP = 0x06;
        public byte TMCL_STAP = 0x07;
        public byte TMCL_RSAP = 0x08;
        public byte TMCL_SGP = 0x09;
        public byte TMCL_GGP = 0x0a;
        public byte TMCL_STGP = 0x0b;
        public byte TMCL_RSGP = 0x0c;
        public byte TMCL_RFS = 0x0d;
        public byte TMCL_SIO = 0x0e;
        public byte TMCL_GIO = 0x0f;
        public byte TMCL_SCO = 0x1e;
        public byte TMCL_GCO = 0x1f;
        public byte TMCL_CCO = 0x20;
        public byte TMCL_WAIT = 0x1b;
        //options für MVP und WAIT und SAP
        public byte MVP_ABS = 0x00;
        public byte MVP_REL = 0x01;
        public byte MVP_COORD = 0x02;
        public byte WAIT_POS = 0x01;
        public byte ACTUAL_POS = 0x01;
        public byte MAX_SPEED = 0x04;
        public byte MAX_ACC = 0x05;
        public byte MICROSTEP = 0x8c; 
        public byte RAMP_DIV = 0x99;
        public byte PULSE_DIV = 0x9a;
        public byte STROM_MAX = 0x06; 
        public byte STROM_RUHE = 0x07; 
        public byte FULLSTEP = 0x8c;
     
        public static byte EMPTY = 0x00;
        ////inverse Kinematik
        //public int Laenge1 = 25;
        //public int Laenge2 = 24;
        //public int Laenge3 = 14;





        ////////////////////////////////////////////////////////////double[] IKAngle2 = new double[12];






        //int[] IKSPeedInt2;               
        //////////winkelrechner
        //int d4 = 23; 
        //int d6 = 10;  
        //int a2 = 25;
        //int[] deltaschritte;
        ////////winkelrechner

        Binding binding1;
        Binding binding2;
        Binding binding3;
        Binding binding4;
        Binding binding5;
        Binding binding6;
        Binding bindingA;
        Binding bindingB;
        Binding bindingC;
        Binding bindingD;
        Binding bindingE;
        Binding bindingF;

        Binding x;
        Binding y;
        Binding z;
        Binding a;
        Binding b;
        Binding c;
        List<Binding> ActualPositionBindings = new List<Binding>();     
        List<List<Binding>> ModuleBindings = new List<List<Binding>>();     

        private Viewport3D viewport3D = new Viewport3D();
        private Simulation3D Simulation3D;
        private int GCodeSimulationsCounter = 0;
        private int GCodeSimulationsCounter2 = 0;


#endregion

        #region Star up
        public MainWindow()
        {
            InitializeComponent();

            //MenuItem
            MenuItem normal_Mode = new MenuItem();
            normal_Mode.Header = "Normal";
            normal_Mode.Background = Brushes.Black;
            normal_Mode.Foreground = Brushes.White;
            NormalCommand = new RoutedCommand();
            CommandBinding Mode_Normal = new CommandBinding(NormalCommand, NormalCommand_execute, NormalCommand_activ);
            this.CommandBindings.Add(Mode_Normal);
            normal_Mode.Command = NormalCommand;
            Mode.Items.Add(normal_Mode);

            MenuItem TicTacToe = new MenuItem();
            TicTacToe.Header = "TicTacToe";
            TicTacToe.Background = Brushes.Black;
            TicTacToe.Foreground = Brushes.White;
            TicTacToeCommand = new RoutedCommand();
            CommandBinding Mode_TicTacToe = new CommandBinding(TicTacToeCommand, TicTacToeCommand_execute, TicTacToeCommand_activ);
            this.CommandBindings.Add(Mode_TicTacToe);
            TicTacToe.Command = TicTacToeCommand;
            Mode.Items.Add(TicTacToe);

            MenuItem Simulation = new MenuItem();
            Simulation.Header = "Simulation";
            Simulation.Background = Brushes.Black;
            Simulation.Foreground = Brushes.White;
            SimulationCommand = new RoutedCommand();
            CommandBinding Mode_Kinect = new CommandBinding(SimulationCommand, SimulationCommand_execute, SimulationCommand_activ);
            this.CommandBindings.Add(Mode_Kinect);
            Simulation.Command = SimulationCommand;
            Mode.Items.Add(Simulation);
            mode = Mode_.Normal;

            double[] A = new double[6] { 0, 25, 0, 0, 0, 0 };
            double[] D = new double[6] { 0, 0, 0, 23, 0, 10 };
            double[] Alpha = new double[6] { -Math.PI / 2, 0, Math.PI / 2, -Math.PI / 2, Math.PI / 2, 0 };
            DHParam = ResourceFactory.SetDHParameter(A, Alpha, D, RobotType.SixAxis);

            DataControl = ResourceFactory.GetDataControl();
            initBindings();

            string p1 = System.IO.Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location);
            string[] s = p1.Split(new[] { "bin" }, StringSplitOptions.None); //die Pattern sind unter dem aktuellen Projekt gespeichert, dort wo auch der Ordner "bin" ist.
            Simulation3D = ResourceFactory.GetModel(s[0] + @"robot\1.xaml", s[0] + @"robot\2.xaml", s[0] + @"robot\3.xaml", s[0] + @"robot\4.xaml", s[0] + @"robot\5.xaml", s[0] + @"robot\6.xaml");
            viewport3D.Children.Add(Simulation3D.Model);
            viewport3D.Camera = Simulation3D.Camera;
            Animation.Children.Add(viewport3D);
            Grid.SetColumn(viewport3D, 0);
            Grid.SetRow(viewport3D, 0);
            Slider_Zoom.Value = 7;
            Slider_Up.Value = 5;


            MotorControllerMaster = ResourceFactory.GetMotorControllerMaster();
            MotorControllerMaster.InitializedEvent += SerialPortHanldeEventHandler;
            //MotorControllerMaster.CreateNew();
            //MotorControllerMaster.Dispose();
            ////MotorControllerMaster.MotorController[1].SendCommand(0x00, 0x00, 0x00, 0x00, 0);      
            
            KinematicControl = ResourceFactory.GetKinematicControl(DHParam, DataControl.ActualPosition);
            KinematicControl.InverseKinematics.InverseKinematicEvent += InversKinematicEventHandler;
            KinematicControl.ForewardKinematics.ForewardKinematicEvent += ForewardKinematicEventHandler;
            //KinematicControl.ForewardKinematics.ForewardKinematic(3, 3, 3, 3, 3, 3, 3);
            //KinematicControl.InverseKinematics.InverseKinematic(20, 20, 35,0, 0, 0, int.Parse(TextBoxGlobalMaxSpeed.Text));
            timerIK.Tick += new EventHandler(timerIK_Tick);

            //DataControl = ResourceFactory.GetDataControl();
            //DataControl.NCData.gDeltaSteps4 = 12000;
            //MessageBox.Show(DataControl.NCData.gDeltaSteps4.ToString());

            //TreeViewCollection = ResourceFactory.GetTreeViewCollection();
            //TreeViewCollection.CreateNode("Module");
            //TreeViewCollection.AddSubNode("Module", "Modul1");
            //TreeViewCollection.AddSubNode("Module", "Modul2");
            //TreeViewCollection.AddSubNode("Module", "Modul3");
            //TreeViewCollection.AddSubNode("Module", "Modul4");
            //TreeViewCollection.AddSubNode("Module", "Modul5");
            //TreeViewCollection.AddSubNode("Module", "Modul6");
            //TreeViewCollection.CreateNode("Position Data");
            //TreeViewCollection.AddSubNode("Position Data", "Motor1");
            //TreeViewCollection.AddSubNode("Position Data", "Motor2");
            //TreeViewCollection.AddSubNode("Position Data", "Motor3");
            //TreeViewCollection.AddSubNode("Position Data", "Motor4");
            //TreeViewCollection.AddSubNode("Position Data", "Motor5");
            //TreeViewCollection.AddSubNode("Position Data", "Motor6");
            //DataTreeView.DataContext = TreeViewCollection;
            initTreeView();
            InitComboBoxes();
            //the List of NCControls can only Contain 2 Objects of it's type. it's a semaphor
            //NCControls = ResourceFactory.AddNumericalControlSemaphor(new List<string>(), 24, 55, DHParam);
            //NCControls = ResourceFactory.AddNumericalControlSemaphor(new List<string>(), 24, 55, DHParam);
            //NCControls[0].GetStartPos(2);
            //NCControls[1].GetStartPos(5);
            NCControls = new List<NumericalControl>();
            GCodeAblaufTimer.Tick += new EventHandler(gcodeablauf_Tick2);

            //adress1 = new Binding("Adress");
            //adress1.Source = DataControl.MotorControllerData[0];//modul_data1
            //adress1.Mode = BindingMode.OneWay;
            //current1 = new Binding("Current");
            //current1.Source = DataControl.MotorControllerData[0];//modul_data1
            //current1.Mode = BindingMode.OneWay;
        }
        void Window_Closing(object sender, CancelEventArgs e)
        {
            if (timerIK.IsEnabled == true)
            {
                timerIK.Stop();
            }
            if (GCodeAblaufTimer.IsEnabled == true)
            {
                GCodeAblaufTimer.Stop();
            }
            Start = false;
            try
            {
                for (int q = 0; q < 6; q++)
                {
                    MotorControllerMaster.MotorController[0].SendCommand(Addresses[q], TMCL_MST, EMPTY, Motor0, 0);               
                }
                if (MotorControllerMaster.MotorController[0].IsOpen == true && MotorControllerMaster.MotorController[1].IsOpen == true 
                    && MotorControllerMaster.MotorController[2].IsOpen == true && MotorControllerMaster.MotorController[3].IsOpen == true 
                    && MotorControllerMaster.MotorController[4].IsOpen == true && MotorControllerMaster.MotorController[5].IsOpen == true)
                {
                    MotorControllerMaster.Dispose();
                }
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen");
            }
            list.Clear();
            GCodeListe.Clear();
            GCodeListeSpeed.Clear();
            GCodeDeltaPos.Clear();
            GCodeWinkel.Clear();
            GXD.Clear();
            GYD.Clear();
            GZD.Clear();
            GAD.Clear();
            GBD.Clear();
            GCD.Clear();
            GDiagStr.Clear();
            NCDataCollection.Clear();
            dataGrid.DataContext = list.Select(x => new { Value = x }).ToList();
            dataGrid.ItemsSource = list.Select(x => new { Value = x }).ToList();
            tabelle1.ItemsSource = null;
            GCodeListCounter = 0;
            GCodeListSpeedCounter = 0;
            p = 0;
            pos = 0;
            MessageBox.Show("Window_Closing has been called");
        }
        #endregion

        #region MENU und Commands
        private void Open_execute(object sender, ExecutedRoutedEventArgs e)
        {
            G_Code_Diagnose.IsEnabled = false;
            G_Code_Converter.IsEnabled = false;
            G_Code_Start.IsEnabled = false;
            StartNCCalc.IsEnabled = false;
            G_Code_Clear.IsEnabled = false;
            G_Code_Stop.IsEnabled = false;
            G_Code_Continue.IsEnabled = false;
            list.Clear();
            GCodeListe.Clear();
            GCodeListeSpeed.Clear();
            GCodeDeltaPos.Clear();
            GCodeWinkel.Clear();
            GXD.Clear();
            GYD.Clear();
            GZD.Clear();
            GAD.Clear();
            GBD.Clear();
            GCD.Clear();
            GDiagStr.Clear();
            NCDataCollection.Clear();
            dataGrid.DataContext = list.Select(x => new { Value = x }).ToList();
            dataGrid.ItemsSource = list.Select(x => new { Value = x }).ToList();
            tabelle1.ItemsSource = null;
            GCodeListCounter = 0;
            GCodeListSpeedCounter = 0;
            GCodeSimulationsCounter2 = 0;
            GCodeSimulationsCounter = 0;
            Simulation3D.Linepoint3D_Array = null;
            Simulation3D.GeometryModel3d_Line.Clear();
            Simulation3D.Geometry3DContainer_Line.Children.Clear();
            Start = false;//GCode-Timer start
            p = 0;
            pos = 0;
            Microsoft.Win32.OpenFileDialog dlg = new Microsoft.Win32.OpenFileDialog();
            dlg.DefaultExt = ".txt";
            dlg.Filter = "Text documents (.txt)|*.txt|" + "Text documents (.xml)|*.xml|" + "All Files|*.*";
            Nullable<bool> result = dlg.ShowDialog();

            if (result == true)
            {
                filename = dlg.FileName;
                if (filename.EndsWith("txt"))
                {
                    list.Clear();
                    stopwatch.Start();
                    list = CustomFile.ReadText(filename);
                    dataGrid.DataContext = list.Select(x => new { Value = x }).ToList();
                    dataGrid.ItemsSource = list.Select(x => new { Value = x }).ToList();
                    StartNCCalc.IsEnabled = true;
                    G_Code_Clear.IsEnabled = true;
                }
                else if (filename.EndsWith("xml"))
                {
                    tabelle1.ItemsSource = CustomFile.ReadXMLListGCode(filename);

                }
            }         
        }
        private void Open_activ(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void Save_execute(object sender, ExecutedRoutedEventArgs e)
        {
            try
            {
                CustomFile.WriteXMLListGCode(NCDataCollection, filename);
            }
            catch
            {
                Microsoft.Win32.SaveFileDialog dlg = new Microsoft.Win32.SaveFileDialog();
                dlg.FileName = "Document";
                dlg.DefaultExt = ".xml";
                dlg.Filter = "Text documents (.xml)|*.xml";

                // Show save file dialog box
                Nullable<bool> result = dlg.ShowDialog();

                if (result == true)
                {
                    // Save document 
                    filename = dlg.FileName;
                    CustomFile.WriteXMLListGCode(NCDataCollection, filename);
                    MessageBox.Show("Postion gespeichert in " + filename);
                }
            }
        }
        private void Save_activ(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void SaveAs_execute(object sender, ExecutedRoutedEventArgs e)
        {
            Microsoft.Win32.SaveFileDialog dlg = new Microsoft.Win32.SaveFileDialog();
            dlg.FileName = "Document";
            dlg.DefaultExt = ".xml";
            dlg.Filter = "Text documents (.xml)|*.xml";

            // Show save file dialog box
            Nullable<bool> result = dlg.ShowDialog();

            if (result == true)
            {
                // Save document 
                filename = dlg.FileName;
                CustomFile.WriteXMLListGCode(NCDataCollection, filename);
            }
        }
        private void SaveAs_activ(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void Close_execute(object sender, ExecutedRoutedEventArgs e)
        {
            
            MessageBoxResult CloseBox = MessageBox.Show("Sollen alle Prozesse beendet werden?", "Question", MessageBoxButton.YesNoCancel, MessageBoxImage.Question);
            if (CloseBox == MessageBoxResult.Yes)
            {
                this.Close();//diese Methode ruft "Window_Closing" auf
            }
        }
        private void Close_activ(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void TicTacToeCommand_execute(object sender, ExecutedRoutedEventArgs e)
        {
            MessageBox.Show("TicTacToe-Modus noch nicht eingerichtet");
        }
        private void TicTacToeCommand_activ(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void NormalCommand_execute(object sender, ExecutedRoutedEventArgs e)
        {
            mode = Mode_.Normal;
            Mode_Lable.Content = "Normal-Mode";
            ManualPosMotorButton.IsEnabled = true;
            GetCOMNamesBotton.IsEnabled = true;
            MoveButton.IsEnabled = true;
            GlobalMaxSpeedButton.IsEnabled = true;
            GlobalMaxAccelerationButton.IsEnabled = true;
            MoverKinematicsButton.IsEnabled = true;
            CurrentButton.IsEnabled = true;
            MotorStopButton.IsEnabled = true;
            StandyButton.IsEnabled = true;
            HomeButton.IsEnabled = true;
            Tabs1.Background = Brushes.White;

        }
        private void NormalCommand_activ(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void SimulationCommand_execute(object sender, ExecutedRoutedEventArgs e)
        {
            mode = Mode_.Simulation;
            Mode_Lable.Content = "Simulation-Mode"; 
            ManualPosMotorButton.IsEnabled = false;
            GetCOMNamesBotton.IsEnabled = false;
            MoveButton.IsEnabled = false;
            GlobalMaxSpeedButton.IsEnabled = false;
            GlobalMaxAccelerationButton.IsEnabled = false;
            MoverKinematicsButton.IsEnabled = false;
            CurrentButton.IsEnabled = false;
            MotorStopButton.IsEnabled = false;
            StandyButton.IsEnabled = false;
            HomeButton.IsEnabled = false;
            Tabs1.Background = Brushes.Red;
        }
        private void SimulationCommand_activ(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        #endregion

        #region INITIALISIEREN
        private void GetCOMNames_Main(object sender, RoutedEventArgs e)
        {
            try
            {
                MotorControllerMaster.CreateNew();
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen 1");
            }
        }

        public void SerialPortHanldeEventHandler(object sender, SerialPortInitEventArgs e)
        {
            MessageBoxResult resultat = MessageBox.Show(e.InitializedString + ":", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
            if (resultat == MessageBoxResult.OK)
            {
                if (MotorControllerMaster.COM_NAMES[0] != null && MotorControllerMaster.COM_NAMES[1] != null && MotorControllerMaster.COM_NAMES[2] != null && MotorControllerMaster.COM_NAMES[3] != null && MotorControllerMaster.COM_NAMES[4] != null && MotorControllerMaster.COM_NAMES[5] != null)
                {
                    comboBox3.IsEnabled = true;
                    comboBox3.Items.Add(MotorControllerMaster.COM_NAMES[0] + "//" + MotorControllerMaster.RecievedData[0]);
                    comboBox3.Items.Add(MotorControllerMaster.COM_NAMES[1] + "//" + MotorControllerMaster.RecievedData[1]);
                    comboBox3.Items.Add(MotorControllerMaster.COM_NAMES[2] + "//" + MotorControllerMaster.RecievedData[2]);
                    comboBox3.Items.Add(MotorControllerMaster.COM_NAMES[3] + "//" + MotorControllerMaster.RecievedData[3]);
                    comboBox3.Items.Add(MotorControllerMaster.COM_NAMES[4] + "//" + MotorControllerMaster.RecievedData[4]);
                    comboBox3.Items.Add(MotorControllerMaster.COM_NAMES[5] + "//" + MotorControllerMaster.RecievedData[5]);
                    GetCOMNamesBotton.IsEnabled = false;
                    initialize();
                }
            }
        }
        private void initialize()
        {
            try
            {
                //MotorControllerMaster.CreateNew(); ICH DENKE DAS IST NICHT MEHR NOTWENDIG (geschrieben am 22.12.2015)
                for (int q = 0; q < 6; q++)
                {
                    DataControl.MotorControllerData[q].Baudrate = 115200;
                    DataControl.MotorControllerData[q].Adress = q + 1;
                    DataControl.MotorControllerData[q].PulseDiv = 6;
                    DataControl.MotorControllerData[q].RampDiv = 7;
                    DataControl.MotorControllerData[q].Acceleration = 500;
                    DataControl.MotorControllerData[q].mFrequency = 32;
                    DataControl.MotorControllerData[q].Current = 0.3;
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, RAMP_DIV, Motor0, DataControl.MotorControllerData[q].RampDiv);
                    Thread.Sleep(10);
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, PULSE_DIV, Motor0, DataControl.MotorControllerData[q].PulseDiv);
                }
                Modul_statusstring = "online";
                V.IsEnabled = true;
                U.IsEnabled = true;
                Y.IsEnabled = true;
                ALPHA.IsEnabled = true;
                BETA.IsEnabled = true;
                GAMMA.IsEnabled = true;
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen");
            }
        }
        private void SetMaxAcceleration(object sender, RoutedEventArgs e)
        {
            try
            {
                int Acc = int.Parse(TextBoxGlobalMaxAcceleration.Text);
                for (int q = 0; q < 6; q++)
                {
                    DataControl.MotorControllerData[q].Acceleration = Acc;
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, MAX_ACC, Motor0, Acc);
                }
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen");
            }
        }
        private void SetCurrent(object sender, RoutedEventArgs e)
        {
            try
            {
                int current = int.Parse(tbCurent.Text);
                for (int q = 0; q < 5; q++)////////////////Strom des letzten motores sehr gering belassen bis Motor getestet. darum q<5
                {
                    DataControl.MotorControllerData[q].Current = current;
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, STROM_RUHE, Motor0, current);
                }
                //Port6.SendCommand(Adress6, TMCL_SAP, STROM_RUHE, Motor0, curent);
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen");
            }
        }
        private void SetMaxSpeed(object sender, RoutedEventArgs e)
        {
            try
            {
                int Speed = int.Parse(TextBoxGlobalMaxSpeed.Text);
                for (int q = 0; q < 6; q++)
                {
                    DataControl.MotorControllerData[q].Speed = Speed;
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, MAX_SPEED, Motor0, Speed);
                }
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen");
            }
        }
        private void InitComboBoxes()
        {
            comboBox1.Items.Add("Motor1");
            comboBox1.Items.Add("Motor2");
            comboBox1.Items.Add("Motor3");
            comboBox1.Items.Add("Motor4");
            comboBox1.Items.Add("Motor5");
            comboBox1.Items.Add("Motor6");
            comboBox2.Items.Add("Position in Grad");
            comboBox2.Items.Add("Position in Schritte");
            comboBox2.SelectedIndex = 0;
        }
        #endregion

        #region MANUAL POSITIONING
        private void ManualPositioning(object sender, RoutedEventArgs e)
        {
            switch (comboBox1.SelectedIndex)
            {
                case 0:
                    try
                    {
                        MotorControllerMaster.MotorController[0].SendCommand(Addresses[0], TMCL_MVP, MVP_REL, Motor0, int.Parse(AnzufahrendePosition.Text));
                        DataControl.ActualPosition.Schritte1 = DataControl.ActualPosition.Schritte1 + int.Parse(AnzufahrendePosition.Text);
                    }
                    catch
                    {
                        MessageBox.Show("COM-Ports nicht angeschlossen");
                    }
                    break;
                case 1:
                    try
                    {
                        MotorControllerMaster.MotorController[1].SendCommand(Addresses[1], TMCL_MVP, MVP_REL, Motor0, int.Parse(AnzufahrendePosition.Text));
                        DataControl.ActualPosition.Schritte2 = DataControl.ActualPosition.Schritte2 + int.Parse(AnzufahrendePosition.Text);
                    }
                    catch
                    {
                        MessageBox.Show("COM-Ports nicht angeschlossen");
                    }
                    break;
                case 2:
                    try
                    {
                        MotorControllerMaster.MotorController[2].SendCommand(Addresses[2], TMCL_MVP, MVP_REL, Motor0, int.Parse(AnzufahrendePosition.Text));
                        DataControl.ActualPosition.Schritte3 = DataControl.ActualPosition.Schritte3 + int.Parse(AnzufahrendePosition.Text);
                    }
                    catch
                    {
                        MessageBox.Show("COM-Ports nicht angeschlossen");
                    }
                    break;
                case 3:
                    try
                    {
                        MotorControllerMaster.MotorController[3].SendCommand(Addresses[3], TMCL_MVP, MVP_REL, Motor0, int.Parse(AnzufahrendePosition.Text));
                        DataControl.ActualPosition.Schritte4 = DataControl.ActualPosition.Schritte4 + int.Parse(AnzufahrendePosition.Text);
                    }
                    catch
                    {
                        MessageBox.Show("COM-Ports nicht angeschlossen");
                    }
                    break;
                case 4:
                    try
                    {
                        MotorControllerMaster.MotorController[4].SendCommand(Addresses[4], TMCL_MVP, MVP_REL, Motor0, int.Parse(AnzufahrendePosition.Text));
                        DataControl.ActualPosition.Schritte5 = DataControl.ActualPosition.Schritte5 + int.Parse(AnzufahrendePosition.Text);
                    }
                    catch
                    {
                        MessageBox.Show("COM-Ports nicht angeschlossen");
                    }
                    break;
                case 5:
                    try
                    {
                        MotorControllerMaster.MotorController[5].SendCommand(Addresses[5], TMCL_MVP, MVP_REL, Motor0, int.Parse(AnzufahrendePosition.Text));
                        DataControl.ActualPosition.Schritte6 = DataControl.ActualPosition.Schritte6 + int.Parse(AnzufahrendePosition.Text);
                    }
                    catch
                    {
                        MessageBox.Show("COM-Ports nicht angeschlossen");
                    }
                    break;
            }
        }
        private void MoveManualStart(object sender, MouseButtonEventArgs e)
        {
            if ((cb1.IsChecked == true) && (cb2.IsChecked == false))
            {
                switch (comboBox1.SelectedIndex)
                {
                    case 0:
                        try
                        {
                            MotorControllerMaster.MotorController[0].SendCommand(Addresses[0], TMCL_ROR, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 1:
                        try
                        {
                            MotorControllerMaster.MotorController[1].SendCommand(Addresses[1], TMCL_ROR, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 2:
                        try
                        {
                            MotorControllerMaster.MotorController[2].SendCommand(Addresses[2], TMCL_ROR, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 3:
                        try
                        {
                            MotorControllerMaster.MotorController[3].SendCommand(Addresses[3], TMCL_ROR, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 4:
                        try
                        {
                            MotorControllerMaster.MotorController[4].SendCommand(Addresses[4], TMCL_ROR, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 5:
                        try
                        {
                            MotorControllerMaster.MotorController[5].SendCommand(Addresses[5], TMCL_ROR, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                }
            }
            if ((cb1.IsChecked == false) && (cb2.IsChecked == true))
            {
                switch (comboBox1.SelectedIndex)
                {
                    case 0:
                        try
                        {
                            MotorControllerMaster.MotorController[0].SendCommand(Addresses[0], TMCL_ROL, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 1:
                        try
                        {
                            MotorControllerMaster.MotorController[1].SendCommand(Addresses[1], TMCL_ROL, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 2:
                        try
                        {
                            MotorControllerMaster.MotorController[2].SendCommand(Addresses[2], TMCL_ROL, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 3:
                        try
                        {
                            MotorControllerMaster.MotorController[3].SendCommand(Addresses[3], TMCL_ROL, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 4:
                        try
                        {
                            MotorControllerMaster.MotorController[4].SendCommand(Addresses[4], TMCL_ROL, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                    case 5:
                        try
                        {
                            MotorControllerMaster.MotorController[5].SendCommand(Addresses[5], TMCL_ROL, 0, Motor0, 50);
                        }
                        catch
                        {
                            MessageBox.Show("COM-Ports nicht angeschlossen");
                        }
                        break;
                }
            }
            else
            {
                MessageBox.Show("keine oder beide Richtung/en angewählt");
            }
        }
        private void MoveManualStop(object sender, MouseButtonEventArgs e)
        {
            try
            {
                for (int q = 0; q < 6; q++)
                {
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_MST, EMPTY, Motor0, 0);
                }
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen");
            }
        }
        #endregion

        #region KINEMATIK
        private void CalculateIK(object sender, RoutedEventArgs e)
        {
            try
            {
                KinematicControl.InverseKinematics.InverseKinematic(double.Parse(TextBoxV.Text), double.Parse(TextBoxU.Text), double.Parse(TextBoxY.Text), double.Parse(TextBoxAlpha.Text), double.Parse(TextBoxBeta.Text), double.Parse(TextBoxGamma.Text), int.Parse(TextBoxGlobalMaxSpeed.Text));
                Foreward_Kinematic.IsEnabled = true;
            }
            catch
            {
                MessageBox.Show("ungültige Werte eingegeben");        
            }
        }
        private void InversKinematicEventHandler(InverseKinematics sender, KinematicEventArgs e)
        {
            if (e.KinematicEventMessage.Contains("succesfully"))
            {
                MessageBox.Show(e.KinematicEventMessage);
                //IKAngle2 = KinematicControl.InverseKinematics.InverseKinematicsData;
                TextBoxTeta1.Text = KinematicControl.InverseKinematics.InverseKinematicsData[0].ToString();
                TextBoxTeta2.Text = KinematicControl.InverseKinematics.InverseKinematicsData[1].ToString();
                TextBoxTeta3.Text = KinematicControl.InverseKinematics.InverseKinematicsData[2].ToString();
                TextBoxTeta4.Text = KinematicControl.InverseKinematics.InverseKinematicsData[3].ToString();
                TextBoxTeta5.Text = KinematicControl.InverseKinematics.InverseKinematicsData[4].ToString();
                TextBoxTeta6.Text = KinematicControl.InverseKinematics.InverseKinematicsData[5].ToString();
                //////////Speed berechnen               
                //deltaschritte = new int[6];
                //deltaschritte[0] = (int)(KinematicControl.InverseKinematics.InverseKinematicsData[6] - DataControl.ActualPosition.Schritte1);
                //deltaschritte[1] = (int)(KinematicControl.InverseKinematics.InverseKinematicsData[7] - DataControl.ActualPosition.Schritte2);
                //deltaschritte[2] = (int)(KinematicControl.InverseKinematics.InverseKinematicsData[8] - DataControl.ActualPosition.Schritte3);
                //deltaschritte[3] = (int)(KinematicControl.InverseKinematics.InverseKinematicsData[9] - DataControl.ActualPosition.Schritte4);
                //deltaschritte[4] = (int)(KinematicControl.InverseKinematics.InverseKinematicsData[10] - DataControl.ActualPosition.Schritte5);
                //deltaschritte[5] = (int)(KinematicControl.InverseKinematics.InverseKinematicsData[11] - DataControl.ActualPosition.Schritte6);
                //int deltaSchritte1_2 = Math.Abs(deltaschritte[0]);
                //int deltaSchritte2_2 = Math.Abs(deltaschritte[1]);
                //int deltaSchritte3_2 = Math.Abs(deltaschritte[2]);
                //int deltaSchritte4_2 = Math.Abs(deltaschritte[3]);
                //int deltaSchritte5_2 = Math.Abs(deltaschritte[4]);
                //int deltaSchritte6_2 = Math.Abs(deltaschritte[5]);
                //float[] IKSpeed2 = KinematicControl.InverseKinematics.NextSpeed2(deltaSchritte1_2, deltaSchritte2_2, deltaSchritte3_2, deltaSchritte4_2, deltaSchritte5_2, deltaSchritte6_2, int.Parse(TextBoxGlobalMaxSpeed.Text));//hier den maxspeed lassen
                //IKSPeedInt2 = new int[6];
                //IKSPeedInt2[0] = (int)Math.Abs(Math.Round(IKSpeed2[0], 0));
                //IKSPeedInt2[1] = (int)Math.Abs(Math.Round(IKSpeed2[1], 0));
                //IKSPeedInt2[2] = (int)Math.Abs(Math.Round(IKSpeed2[2], 0));
                //IKSPeedInt2[3] = (int)Math.Abs(Math.Round(IKSpeed2[3], 0));
                //IKSPeedInt2[4] = (int)Math.Abs(Math.Round(IKSpeed2[4], 0));
                //IKSPeedInt2[5] = (int)Math.Abs(Math.Round(IKSpeed2[5], 0));
                TextBoxTeta1Speed.Text = KinematicControl.InverseKinematics.InverseKinematicsSpeed[0].ToString();
                TextBoxTeta2Speed.Text = KinematicControl.InverseKinematics.InverseKinematicsSpeed[1].ToString();
                TextBoxTeta3Speed.Text = KinematicControl.InverseKinematics.InverseKinematicsSpeed[2].ToString();
                TextBoxTeta4Speed.Text = KinematicControl.InverseKinematics.InverseKinematicsSpeed[3].ToString();
                TextBoxTeta5Speed.Text = KinematicControl.InverseKinematics.InverseKinematicsSpeed[4].ToString();
                TextBoxTeta6Speed.Text = KinematicControl.InverseKinematics.InverseKinematicsSpeed[5].ToString();
                ///////////Speed berechnen
                TextBoxTeta1Steps.Text = KinematicControl.InverseKinematics.InverseKinematicsDeltaSteps[0].ToString();
                TextBoxTeta2Steps.Text = KinematicControl.InverseKinematics.InverseKinematicsDeltaSteps[1].ToString();
                TextBoxTeta3Steps.Text = KinematicControl.InverseKinematics.InverseKinematicsDeltaSteps[2].ToString();
                TextBoxTeta4Steps.Text = KinematicControl.InverseKinematics.InverseKinematicsDeltaSteps[3].ToString();
                TextBoxTeta5Steps.Text = KinematicControl.InverseKinematics.InverseKinematicsDeltaSteps[4].ToString();
                TextBoxTeta6Steps.Text = KinematicControl.InverseKinematics.InverseKinematicsDeltaSteps[5].ToString();

                if (mode == Mode_.Simulation)
                {
                    KinematicControl.ForewardKinematics.ForewardKinematic(KinematicControl.InverseKinematics.InverseKinematicsData[0] / 180 * Math.PI, KinematicControl.InverseKinematics.InverseKinematicsData[1] / 180 * Math.PI, KinematicControl.InverseKinematics.InverseKinematicsData[2] / 180 * Math.PI, KinematicControl.InverseKinematics.InverseKinematicsData[3] / 180 * Math.PI, KinematicControl.InverseKinematics.InverseKinematicsData[4] / 180 * Math.PI, KinematicControl.InverseKinematics.InverseKinematicsData[5] / 180 * Math.PI, double.Parse(TextBoxBeta.Text) / 180 * Math.PI);
                    Simulation3D.MoveSimulation(Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[0], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[1], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[2], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[3], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[4], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[5], 3));
                }
            }
            else
            {
                MessageBoxResult resultat = MessageBox.Show(e.ToString() + ":\n" + e.KinematicEventMessage, "Warunung", MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }
        private void ForewardKinematicEventHandler(ForewardKinematics sender, KinematicEventArgs e)
        {
            switch(mode)
            {
                case Mode_.Simulation:
                    BoxV.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[0].ToString();
                    BoxU.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[1].ToString();
                    BoxY.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[2].ToString(); ;
                    BoxAlpha.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[3].ToString();
                    BoxBeta.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[4].ToString();
                    BoxGamma.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[5].ToString();
                    //DataControl.ActualPosition.PosX = KinematicControl.ForewardKinematics.ForewardKinematicsData[0];
                    //DataControl.ActualPosition.PosY = KinematicControl.ForewardKinematics.ForewardKinematicsData[1];
                    //DataControl.ActualPosition.PosZ = KinematicControl.ForewardKinematics.ForewardKinematicsData[2];
                    //DataControl.ActualPosition.Alpha = KinematicControl.ForewardKinematics.ForewardKinematicsData[3];
                    //DataControl.ActualPosition.Beta = KinematicControl.ForewardKinematics.ForewardKinematicsData[4];
                    //DataControl.ActualPosition.Gamma = KinematicControl.ForewardKinematics.ForewardKinematicsData[5];
                    break;
                case Mode_.Normal:
                    DataControl.ActualPosition.PosX = KinematicControl.ForewardKinematics.ForewardKinematicsData[0];
                    DataControl.ActualPosition.PosY = KinematicControl.ForewardKinematics.ForewardKinematicsData[1];
                    DataControl.ActualPosition.PosZ = KinematicControl.ForewardKinematics.ForewardKinematicsData[2];
                    DataControl.ActualPosition.Alpha = KinematicControl.ForewardKinematics.ForewardKinematicsData[3];
                    DataControl.ActualPosition.Beta = KinematicControl.ForewardKinematics.ForewardKinematicsData[4];
                    DataControl.ActualPosition.Gamma = KinematicControl.ForewardKinematics.ForewardKinematicsData[5];
                    BoxV.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[0].ToString();
                    BoxU.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[1].ToString();
                    BoxY.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[2].ToString(); ;
                    BoxAlpha.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[3].ToString();
                    BoxBeta.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[4].ToString();
                    BoxGamma.Text = KinematicControl.ForewardKinematics.ForewardKinematicsData[5].ToString();
                    break;
            }
            //MessageBox.Show(e.KinematicEventMessage);          
        }
        private void Move_IK(object sender, RoutedEventArgs e)
        {
            try
            {
                IKCommands();
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet");
            }
        }
        private void IKCommands()
        {
            for (int q = 0; q < 6; q++)
            {
                //Port1.SendCommand(Adress1, TMCL_SAP, MAX_SPEED, Motor0, IKSPeedInt2[0]);
                //Port2.SendCommand(Adress2, TMCL_SAP, MAX_SPEED, Motor0, IKSPeedInt2[1]);
                //Port3.SendCommand(Adress3, TMCL_SAP, MAX_SPEED, Motor0, IKSPeedInt2[2]);
                //Port4.SendCommand(Adress4, TMCL_SAP, MAX_SPEED, Motor0, IKSPeedInt2[3]);
                //Port5.SendCommand(Adress5, TMCL_SAP, MAX_SPEED, Motor0, IKSPeedInt2[4]);
                //Port6.SendCommand(Adress6, TMCL_SAP, MAX_SPEED, Motor0, IKSPeedInt2[5]);
                MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, MAX_SPEED, Motor0, (int)KinematicControl.InverseKinematics.InverseKinematicsSpeed[q]);
            }
            for (int q = 0; q < 6; q++)
            {
                //Port1.SendCommand(Adress1, TMCL_MVP, MVP_ABS, Motor0, (int)IKAngle2[6]);//7
                //Port2.SendCommand(Adress2, TMCL_MVP, MVP_ABS, Motor0, (int)IKAngle2[7]);//8
                //Port3.SendCommand(Adress3, TMCL_MVP, MVP_ABS, Motor0, (int)IKAngle2[8]);//10
                //Port4.SendCommand(Adress4, TMCL_MVP, MVP_ABS, Motor0, (int)IKAngle2[9]);//6
                //Port5.SendCommand(Adress5, TMCL_MVP, MVP_ABS, Motor0, (int)IKAngle2[10]);//9
                //Port6.SendCommand(Adress6, TMCL_MVP, MVP_ABS, Motor0, (int)IKAngle2[11]); //11      
                MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_MVP, MVP_ABS, Motor0, (int)KinematicControl.InverseKinematics.InverseKinematicsData[q + 6]);
            }
            for (int q = 0; q < 6; q++)
            {
                //modul_data1.Speed = IKSPeedInt2[0];
                //modul_data2.Speed = IKSPeedInt2[1];
                //modul_data3.Speed = IKSPeedInt2[2];
                //modul_data4.Speed = IKSPeedInt2[3];
                //modul_data5.Speed = IKSPeedInt2[4];
                //modul_data6.Speed = IKSPeedInt2[5];
                //modul_data1.Position = (int)IKAngle2[6];
                //modul_data2.Position = (int)IKAngle2[7];
                //modul_data3.Position = (int)IKAngle2[8];
                //modul_data4.Position = (int)IKAngle2[9];
                //modul_data5.Position = (int)IKAngle2[10];
                //modul_data6.Position = (int)IKAngle2[11];
                DataControl.MotorControllerData[q].Speed = KinematicControl.InverseKinematics.InverseKinematicsSpeed[q];
                DataControl.MotorControllerData[q].Position = (int)KinematicControl.InverseKinematics.InverseKinematicsData[q + 6];
            }
            /////Fals probleme mit sendetiming, den Rest der Methode verschieben auf "Move_IK" und "Standby_Click"
            DataControl.ActualPosition.Winkel1 = Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[0], 3);
            DataControl.ActualPosition.Winkel2 = Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[1], 3);
            DataControl.ActualPosition.Winkel3 = Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[2], 3);
            DataControl.ActualPosition.Winkel4 = Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[3], 3);
            DataControl.ActualPosition.Winkel5 = Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[4], 3);
            DataControl.ActualPosition.Winkel6 = Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[5], 3);
            DataControl.ActualPosition.Schritte1 = (int)KinematicControl.InverseKinematics.InverseKinematicsData[6];//aktuelle_position.Schritte1 + deltaschritte[0];
            DataControl.ActualPosition.Schritte2 = (int)KinematicControl.InverseKinematics.InverseKinematicsData[7];//aktuelle_position.Schritte2 + deltaschritte[1];
            DataControl.ActualPosition.Schritte3 = (int)KinematicControl.InverseKinematics.InverseKinematicsData[8];//aktuelle_position.Schritte3 + deltaschritte[2];
            DataControl.ActualPosition.Schritte4 = (int)KinematicControl.InverseKinematics.InverseKinematicsData[9];//aktuelle_position.Schritte4 + deltaschritte[3];
            DataControl.ActualPosition.Schritte5 = (int)KinematicControl.InverseKinematics.InverseKinematicsData[10];//aktuelle_position.Schritte5 + deltaschritte[4];
            DataControl.ActualPosition.Schritte6 = (int)KinematicControl.InverseKinematics.InverseKinematicsData[11];//aktuelle_position.Schritte6 + deltaschritte[5];
            KinematicControl.ForewardKinematics.ForewardKinematic(DataControl.ActualPosition.Winkel1 / 180 * Math.PI, DataControl.ActualPosition.Winkel2 / 180 * Math.PI, DataControl.ActualPosition.Winkel3 / 180 * Math.PI, DataControl.ActualPosition.Winkel4 / 180 * Math.PI, DataControl.ActualPosition.Winkel5 / 180 * Math.PI, DataControl.ActualPosition.Winkel6 / 180 * Math.PI, double.Parse(TextBoxBeta.Text) / 180 * Math.PI);
            Simulation3D.MoveSimulation(Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[0], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[1], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[2], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[3], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[4], 3), Math.Round(KinematicControl.InverseKinematics.InverseKinematicsData[5], 3));
        }
        private void FK(object sender, RoutedEventArgs e)
        {
            KinematicControl.ForewardKinematics.ForewardKinematic(double.Parse(TextBoxTeta1.Text) / 180 * Math.PI, double.Parse(TextBoxTeta2.Text) / 180 * Math.PI, double.Parse(TextBoxTeta3.Text) / 180 * Math.PI, double.Parse(TextBoxTeta4.Text) / 180 * Math.PI, double.Parse(TextBoxTeta5.Text) / 180 * Math.PI, double.Parse(TextBoxTeta6.Text) / 180 * Math.PI, double.Parse(TextBoxBeta.Text) / 180 * Math.PI);          
        }
        private void Position_Nullen(object sender, RoutedEventArgs e)
        {
            TextBoxTeta1.Text = "0";
            TextBoxTeta2.Text = "0";
            TextBoxTeta3.Text = "0";
            TextBoxTeta4.Text = "0";
            TextBoxTeta5.Text = "0";
            TextBoxTeta6.Text = "0";
            TextBoxTeta1Steps.Text = "0";
            TextBoxTeta2Steps.Text = "0";
            TextBoxTeta3Steps.Text = "0";
            TextBoxTeta4Steps.Text = "0";
            TextBoxTeta5Steps.Text = "0";
            TextBoxTeta6Steps.Text = "0";
            TextBoxTeta1Speed.Text = "0";
            TextBoxTeta2Speed.Text = "0";
            TextBoxTeta3Speed.Text = "0";
            TextBoxTeta4Speed.Text = "0";
            TextBoxTeta5Speed.Text = "0";
            TextBoxTeta6Speed.Text = "0";
            DataControl.ActualPosition.Winkel1 = 0;
            DataControl.ActualPosition.Winkel2 = 0;
            DataControl.ActualPosition.Winkel3 = 0;
            DataControl.ActualPosition.Winkel4 = 0;
            DataControl.ActualPosition.Winkel5 = 0;
            DataControl.ActualPosition.Winkel6 = 0;
            DataControl.ActualPosition.Schritte1 = 0;
            DataControl.ActualPosition.Schritte2 = 0;
            DataControl.ActualPosition.Schritte3 = 0;
            DataControl.ActualPosition.Schritte4 = 0;
            DataControl.ActualPosition.Schritte5 = 0;
            DataControl.ActualPosition.Schritte6 = 0;
            AktualisiereComboBox2();
        }
        //Kartesisch
        private void V_DOWN(object sender, MouseButtonEventArgs e)
        {
            v_activ = true;
            timerIK.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerIK.Start();
        }
        private void V_UP(object sender, MouseButtonEventArgs e)
        {
            timerIK.Stop();
            v_activ = false;
        }
        private void timerIK_Tick(object sender, EventArgs e)
        {
            if (v_activ == true)
            {
                if (MotorControllerMaster.MotorController[0] != null)//da man schon eine Meldung bekommen hat, wenn nicht alle Ports initialisisert werden konnten, reich es hier wenn man nur den Ersten abcheckt
                {
                    double v__ = double.Parse(TextBoxV.Text);
                    if (cb.IsChecked == true)
                    {
                        v__ = v__ + 0.1;
                    }
                    else
                    {
                        v__ = v__ - 0.1;
                    }
                    TextBoxV.Text = v__.ToString();
                    KinematicControl.InverseKinematics.InverseKinematic(double.Parse(TextBoxV.Text), double.Parse(TextBoxU.Text), double.Parse(TextBoxY.Text), double.Parse(TextBoxAlpha.Text), double.Parse(TextBoxBeta.Text), double.Parse(TextBoxGamma.Text), int.Parse(TextBoxGlobalMaxSpeed.Text));                 
                    try
                    {
                        IKCommands();                    
                    }
                    catch
                    {
                        timerIK.Stop();
                        MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                else
                {
                    timerIK.Stop();
                    MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
            else if (u_activ == true)
            {
                if (MotorControllerMaster.MotorController[0] != null)//da man schon eine Meldung bekommen hat, wenn nicht alle Ports initialisisert werden konnten, reich es hier wenn man nur den Ersten abcheckt
                {
                    double u__ = double.Parse(TextBoxU.Text);
                    if (cb.IsChecked == true)
                    {
                        u__ = u__ + 0.1;
                    }
                    else
                    {
                        u__ = u__ - 0.1;
                    }
                    TextBoxU.Text = u__.ToString();
                    KinematicControl.InverseKinematics.InverseKinematic(double.Parse(TextBoxV.Text), double.Parse(TextBoxU.Text), double.Parse(TextBoxY.Text), double.Parse(TextBoxAlpha.Text), double.Parse(TextBoxBeta.Text), double.Parse(TextBoxGamma.Text), int.Parse(TextBoxGlobalMaxSpeed.Text));                 
                    try
                    {
                        IKCommands();
                    }
                    catch
                    {
                        timerIK.Stop();
                        MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                else
                {
                    timerIK.Stop();
                    MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
            else if (y_activ == true)
            {
                if (MotorControllerMaster.MotorController[0] != null)//da man schon eine Meldung bekommen hat, wenn nicht alle Ports initialisisert werden konnten, reich es hier wenn man nur den Ersten abcheckt
                {
                    double y__ = double.Parse(TextBoxY.Text);
                    if (cb.IsChecked == true)
                    {
                        y__ = y__ + 0.1;
                    }
                    else
                    {
                        y__ = y__ - 0.1;
                    }
                    TextBoxY.Text = y__.ToString();
                    KinematicControl.InverseKinematics.InverseKinematic(double.Parse(TextBoxV.Text), double.Parse(TextBoxU.Text), double.Parse(TextBoxY.Text), double.Parse(TextBoxAlpha.Text), double.Parse(TextBoxBeta.Text), double.Parse(TextBoxGamma.Text), int.Parse(TextBoxGlobalMaxSpeed.Text));
                    try
                    {
                        IKCommands();
                    }
                    catch
                    {
                        timerIK.Stop();
                        MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                else
                {
                    timerIK.Stop();
                    MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
            else if (alpha_activ == true)
            {
                if (MotorControllerMaster.MotorController[0] != null)//da man schon eine Meldung bekommen hat, wenn nicht alle Ports initialisisert werden konnten, reich es hier wenn man nur den Ersten abcheckt
                {
                    double alpha__ = double.Parse(TextBoxAlpha.Text);
                    if (cb.IsChecked == true)
                    {
                        alpha__ = alpha__ + 0.1;
                    }
                    else
                    {
                        alpha__ = alpha__ - 0.1;
                    }
                    TextBoxAlpha.Text = alpha__.ToString();
                    KinematicControl.InverseKinematics.InverseKinematic(double.Parse(TextBoxV.Text), double.Parse(TextBoxU.Text), double.Parse(TextBoxY.Text), double.Parse(TextBoxAlpha.Text), double.Parse(TextBoxBeta.Text), double.Parse(TextBoxGamma.Text), int.Parse(TextBoxGlobalMaxSpeed.Text));
                    try
                    {
                        IKCommands();
                    }
                    catch
                    {
                        timerIK.Stop();
                        MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                else
                {
                    timerIK.Stop();
                    MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
            else if (beta_activ == true)
            {
                if (MotorControllerMaster.MotorController[0] != null)//da man schon eine Meldung bekommen hat, wenn nicht alle Ports initialisisert werden konnten, reich es hier wenn man nur den Ersten abcheckt
                {
                    double beta__ = double.Parse(TextBoxBeta.Text);
                    if (cb.IsChecked == true)
                    {
                        beta__ = beta__ + 0.1;
                    }
                    else
                    {
                        beta__ = beta__ - 0.1;
                    }
                    TextBoxBeta.Text = beta__.ToString();
                    KinematicControl.InverseKinematics.InverseKinematic(double.Parse(TextBoxV.Text), double.Parse(TextBoxU.Text), double.Parse(TextBoxY.Text), double.Parse(TextBoxAlpha.Text), double.Parse(TextBoxBeta.Text), double.Parse(TextBoxGamma.Text), int.Parse(TextBoxGlobalMaxSpeed.Text));
                    try
                    {
                        IKCommands();
                    }
                    catch
                    {
                        timerIK.Stop();
                        MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                else
                {
                    timerIK.Stop();
                    MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
            else if (gamma_activ == true)
            {
                if (MotorControllerMaster.MotorController[0] != null)//da man schon eine Meldung bekommen hat, wenn nicht alle Ports initialisisert werden konnten, reich es hier wenn man nur den Ersten abcheckt
                {
                    double gamma__ = double.Parse(TextBoxGamma.Text);
                    if (cb.IsChecked == true)
                    {
                        gamma__ = gamma__ + 0.1;
                    }
                    else
                    {
                        gamma__ = gamma__ - 0.1;
                    }
                    TextBoxGamma.Text = gamma__.ToString();
                    KinematicControl.InverseKinematics.InverseKinematic(double.Parse(TextBoxV.Text), double.Parse(TextBoxU.Text), double.Parse(TextBoxY.Text), double.Parse(TextBoxAlpha.Text), double.Parse(TextBoxBeta.Text), double.Parse(TextBoxGamma.Text), int.Parse(TextBoxGlobalMaxSpeed.Text));
                    try
                    {
                        IKCommands();
                    }
                    catch
                    {
                        timerIK.Stop();
                        MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                else
                {
                    timerIK.Stop();
                    MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
        }
        private void U_DOWN(object sender, MouseButtonEventArgs e)
        {
            u_activ = true;
            timerIK.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerIK.Start();
        }
        private void U_UP(object sender, MouseButtonEventArgs e)
        {
            timerIK.Stop();
            u_activ = false;
        }
        private void Y_DOWN(object sender, MouseButtonEventArgs e)
        {
            y_activ = true;
            timerIK.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerIK.Start();
        }
        private void Y_UP(object sender, MouseButtonEventArgs e)
        {
            timerIK.Stop();
            y_activ = false;
        }
        //Kartesisch
        //winkel
        private void ALPHA_DOWN(object sender, MouseButtonEventArgs e)
        {
            alpha_activ = true;
            timerIK.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerIK.Start();
        }
        private void ALPHA_UP(object sender, MouseButtonEventArgs e)
        {
            timerIK.Stop();
            alpha_activ = false;
        }
        private void BETA_DOWN(object sender, MouseButtonEventArgs e)
        {
            beta_activ = true;
            timerIK.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerIK.Start();
        }
        private void BETA_UP(object sender, MouseButtonEventArgs e)
        {
            timerIK.Stop();
            beta_activ = false;
        }
        private void GAMMA_DOWN(object sender, MouseButtonEventArgs e)
        {
            gamma_activ = true;
            timerIK.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerIK.Start();
        }
        private void GAMMA_UP(object sender, MouseButtonEventArgs e)
        {
            timerIK.Stop();
            gamma_activ = false;
        }
        //winkel
        #endregion

        #region Stop
        private void Stop_Motoren(object sender, RoutedEventArgs e)
        {
            StopAll();
        }
        private void Stop_Motoren_Kinect(object sender, RoutedEventArgs e)
        {
            StopAll();
        }
        private void Standby_Click(object sender, RoutedEventArgs e)
        {
            switch(mode)
            {
                case Mode_.Simulation:
                    KinematicControl.InverseKinematics.InverseKinematic(0, 0, 58, 0, 0, 0, int.Parse(TextBoxGlobalMaxSpeed.Text));                 
                    break;
                case Mode_.Normal:
                    try
                    {
                        KinematicControl.InverseKinematics.InverseKinematic(0, 0, 58, 0, 0, 0, int.Parse(TextBoxGlobalMaxSpeed.Text));
                        IKCommands();
                    }
                    catch
                    {
                        MessageBox.Show(" Ports not initialized");
                    }
                    break;
            }          
        }
        private void Home_Click(object sender, RoutedEventArgs e)
        {
            //Port1.SendCommand(Adress1, TMCL_SAP, MAX_SPEED, Motor0, 200);
            //Port2.SendCommand(Adress2, TMCL_SAP, MAX_SPEED, Motor0, 200);
            //Port3.SendCommand(Adress3, TMCL_SAP, MAX_SPEED, Motor0, 200);
            //Port4.SendCommand(Adress4, TMCL_SAP, MAX_SPEED, Motor0, 200);
            //Port5.SendCommand(Adress5, TMCL_SAP, MAX_SPEED, Motor0, 200);
            //Port6.SendCommand(Adress6, TMCL_SAP, MAX_SPEED, Motor0, 200);
            //Port1.SendCommand(Adress1, TMCL_MVP, MVP_ABS, Motor0, 0);
            //Port2.SendCommand(Adress2, TMCL_MVP, MVP_ABS, Motor0, 0);
            //Port3.SendCommand(Adress3, TMCL_MVP, MVP_ABS, Motor0, 0);
            //Port4.SendCommand(Adress4, TMCL_MVP, MVP_ABS, Motor0, 0);
            //Port5.SendCommand(Adress5, TMCL_MVP, MVP_ABS, Motor0, 0);
            //Port6.SendCommand(Adress6, TMCL_MVP, MVP_ABS, Motor0, 0);
            try
            {
                for (int q = 0; q < 6; q++)
                {
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, MAX_SPEED, Motor0, 200);
                }
                for (int q = 0; q < 6; q++)
                {
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_MVP, MVP_ABS, Motor0, 0);
                }
            }
            catch
            {
                MessageBox.Show("Com-Ports noch nicht gestartet");
            }
        }
        private void StopAll()
        {
            if (timerIK.IsEnabled == true)
            {
                timerIK.Stop();
            }
            if (GCodeAblaufTimer.IsEnabled == true)
            {
                GCodeAblaufTimer.Stop();
            }       
            try
            {
                //Port1.SendCommand(Adress1, TMCL_MST, LEER, Motor0, 0);
                //Port2.SendCommand(Adress2, TMCL_MST, LEER, Motor0, 0);
                //Port3.SendCommand(Adress3, TMCL_MST, LEER, Motor0, 0);
                //Port4.SendCommand(Adress4, TMCL_MST, LEER, Motor0, 0);
                //Port5.SendCommand(Adress5, TMCL_MST, LEER, Motor0, 0);
                //Port6.SendCommand(Adress6, TMCL_MST, LEER, Motor0, 0);
                for (int q = 0; q < 6; q++)
                {
                    MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_MST, EMPTY, Motor0, 0);
                }
            }
            catch
            {
                MessageBox.Show("COM-Ports nicht angeschlossen");
            }
        }
        #endregion

        #region Data and Bindings
        private void initBindings()
        {
            for (int q = 0; q < 6; q++)
            {
                ModuleBindings.Add(new List<Binding>());
                ModuleBindings[q].Add(new Binding("Status") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("Adress") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("Baudrate") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("mFrequency") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("PulseDiv") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("RampDiv") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("Current") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("Acceleration") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("Speed") { Mode = BindingMode.OneWay });
                ModuleBindings[q].Add(new Binding("Position") { Mode = BindingMode.OneWay });
                foreach (Binding b in ModuleBindings[q])
                {
                    b.Source = DataControl.MotorControllerData[q];
                }
            }


            ActualPositionBindings.Add(new Binding("Winkel1") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Winkel2") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Winkel3") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Winkel4") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Winkel5") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Winkel6") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("PosX") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("PosY") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("PosZ") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Alpha") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Beta") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Gamma") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Schritte1") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Schritte2") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Schritte3") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Schritte4") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Schritte5") { Mode = BindingMode.OneWay });
            ActualPositionBindings.Add(new Binding("Schritte6") { Mode = BindingMode.OneWay });        
            foreach(Binding b in ActualPositionBindings)
            {
                b.Source = DataControl.ActualPosition;
            }
        }
        public void AktualisiereComboBox2()
        {
            switch (comboBox2.SelectedIndex)
            {
                case 0:
                    TextBoxPosition1.SetBinding(TextBox.TextProperty, ActualPositionBindings[0]);//Winkel1
                    TextBoxPosition2.SetBinding(TextBox.TextProperty, ActualPositionBindings[1]);//Winkel2
                    TextBoxPosition3.SetBinding(TextBox.TextProperty, ActualPositionBindings[2]);//Winkel3
                    TextBoxPosition4.SetBinding(TextBox.TextProperty, ActualPositionBindings[3]);//Winkel4
                    TextBoxPosition5.SetBinding(TextBox.TextProperty, ActualPositionBindings[4]);//Winkel5
                    TextBoxPosition6.SetBinding(TextBox.TextProperty, ActualPositionBindings[5]);//Winkel6
                    break;
                case 1:
                    TextBoxPosition1.SetBinding(TextBox.TextProperty, ActualPositionBindings[12]);//Schritte1
                    TextBoxPosition2.SetBinding(TextBox.TextProperty, ActualPositionBindings[13]);//Schritte2
                    TextBoxPosition3.SetBinding(TextBox.TextProperty, ActualPositionBindings[14]);//Schritte3
                    TextBoxPosition4.SetBinding(TextBox.TextProperty, ActualPositionBindings[15]);//Schritte4
                    TextBoxPosition5.SetBinding(TextBox.TextProperty, ActualPositionBindings[16]);//Schritte5
                    TextBoxPosition6.SetBinding(TextBox.TextProperty, ActualPositionBindings[17]);//Schritte6
                    break;
            }
        }
        private void selection_changed(object sender, SelectionChangedEventArgs e)
        {
            AktualisiereComboBox2();
        }
        #endregion

        #region TreeView
        private void initTreeView()
        {
            TreeViewCollection = ResourceFactory.GetTreeViewCollection();
            TreeViewCollection.CreateNode("Module");
            TreeViewCollection.AddSubNode("Module", "Modul1");
            TreeViewCollection.AddSubNode("Module", "Modul2");
            TreeViewCollection.AddSubNode("Module", "Modul3");
            TreeViewCollection.AddSubNode("Module", "Modul4");
            TreeViewCollection.AddSubNode("Module", "Modul5");
            TreeViewCollection.AddSubNode("Module", "Modul6");
            TreeViewCollection.CreateNode("Position Data");
            TreeViewCollection.AddSubNode("Position Data", "Motor1");
            TreeViewCollection.AddSubNode("Position Data", "Motor2");
            TreeViewCollection.AddSubNode("Position Data", "Motor3");
            TreeViewCollection.AddSubNode("Position Data", "Motor4");
            TreeViewCollection.AddSubNode("Position Data", "Motor5");
            TreeViewCollection.AddSubNode("Position Data", "Motor6");
            DataTreeView.DataContext = TreeViewCollection;
            initTreeViewDataLabelsAndUniformGrid();
        }

        private void TreeViewSelectedItemChanged(object sender, RoutedEventArgs e)
        {
            //TreeView treeview = sender as TreeView;
            TreeViewNode n1 = DataTreeView.SelectedItem as TreeViewNode;
            if (n1 != null)
            {
                SelectedNode = n1.NodeName;
            }
            else
            {
                SubNode n2 = DataTreeView.SelectedItem as SubNode;
                if (n2 != null)
                {
                    SelectedNode = n2.SubNodeName;
                }
                else
                {
                    MessageBox.Show("NULL");
                }
            }
            switch (SelectedNode)
            {
                case "Position Data":
                    {
                        foreach(Label lable in TreeViewLables)
                        {
                            lable.Foreground = defaultLabelColor;
                        }                    
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        row11.Children.Clear();
                        row12.Children.Clear();
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Angle1"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "angle1"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Angle2"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "angle2"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Angle3"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "angle3"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Angle4"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "angle4"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Angle5"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "angle5"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Angle6"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "angle6"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "PosX"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "posX"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "PosY"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "posY"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "PosZ"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "posZ"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Alpha"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "alpha"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "Beta"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "beta"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "Gamma"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "gamma"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        TreeViewDisplayData.Children.Add(row11);
                        TreeViewDisplayData.Children.Add(row12);
                        break;
                    }
                case "Motor1":
                    {
                        foreach (Label lable in TreeViewLables)
                        {
                            lable.Foreground = defaultLabelColor;
                        }
                        foreach (Label lable in TreeViewLables.Where(s => s.Name.EndsWith("1")))
                        {
                            lable.Foreground = Brushes.Red;
                        }                      
                        TreeViewDisplayData.Children.Clear();                                         
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        row11.Children.Clear();
                        row12.Children.Clear();
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Angle1"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "angle1"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Angle2"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "angle2"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Angle3"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "angle3"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Angle4"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "angle4"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Angle5"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "angle5"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Angle6"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "angle6"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "PosX"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "posX"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "PosY"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "posY"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "PosZ"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "posZ"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Alpha"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "alpha"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "Beta"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "beta"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "Gamma"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "gamma"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        TreeViewDisplayData.Children.Add(row11);
                        TreeViewDisplayData.Children.Add(row12);
                        break;
                    }
                case "Motor2":
                    {
                        foreach (Label lable in TreeViewLables)
                        {
                            lable.Foreground = defaultLabelColor;
                        }
                        foreach (Label lable in TreeViewLables.Where(s => s.Name.EndsWith("2")))
                        {
                            lable.Foreground = Brushes.Red;
                        }
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        row11.Children.Clear();
                        row12.Children.Clear();
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Angle1"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "angle1"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Angle2"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "angle2"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Angle3"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "angle3"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Angle4"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "angle4"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Angle5"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "angle5"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Angle6"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "angle6"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "PosX"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "posX"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "PosY"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "posY"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "PosZ"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "posZ"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Alpha"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "alpha"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "Beta"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "beta"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "Gamma"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "gamma"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        TreeViewDisplayData.Children.Add(row11);
                        TreeViewDisplayData.Children.Add(row12);
                        break;
                    }
                case "Motor3":
                    {
                        foreach (Label lable in TreeViewLables)
                        {
                            lable.Foreground = defaultLabelColor;
                        }
                        foreach (Label lable in TreeViewLables.Where(s => s.Name.EndsWith("3")))
                        {
                            lable.Foreground = Brushes.Red;
                        }
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        row11.Children.Clear();
                        row12.Children.Clear();
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Angle1"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "angle1"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Angle2"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "angle2"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Angle3"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "angle3"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Angle4"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "angle4"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Angle5"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "angle5"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Angle6"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "angle6"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "PosX"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "posX"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "PosY"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "posY"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "PosZ"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "posZ"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Alpha"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "alpha"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "Beta"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "beta"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "Gamma"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "gamma"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        TreeViewDisplayData.Children.Add(row11);
                        TreeViewDisplayData.Children.Add(row12);
                        break;
                    }
                case "Motor4":
                    {
                        foreach (Label lable in TreeViewLables)
                        {
                            lable.Foreground = defaultLabelColor;
                        }
                        foreach (Label lable in TreeViewLables.Where(s => s.Name.EndsWith("4")))
                        {
                            lable.Foreground = Brushes.Red;
                        }
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        row11.Children.Clear();
                        row12.Children.Clear();
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Angle1"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "angle1"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Angle2"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "angle2"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Angle3"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "angle3"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Angle4"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "angle4"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Angle5"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "angle5"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Angle6"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "angle6"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "PosX"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "posX"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "PosY"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "posY"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "PosZ"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "posZ"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Alpha"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "alpha"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "Beta"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "beta"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "Gamma"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "gamma"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        TreeViewDisplayData.Children.Add(row11);
                        TreeViewDisplayData.Children.Add(row12);
                        break;
                    }
                case "Motor5":
                    {
                        foreach (Label lable in TreeViewLables)
                        {
                            lable.Foreground = defaultLabelColor;
                        }
                        foreach (Label lable in TreeViewLables.Where(s => s.Name.EndsWith("5")))
                        {
                            lable.Foreground = Brushes.Red;
                        }
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        row11.Children.Clear();
                        row12.Children.Clear();
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Angle1"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "angle1"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Angle2"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "angle2"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Angle3"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "angle3"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Angle4"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "angle4"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Angle5"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "angle5"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Angle6"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "angle6"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "PosX"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "posX"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "PosY"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "posY"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "PosZ"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "posZ"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Alpha"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "alpha"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "Beta"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "beta"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "Gamma"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "gamma"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        TreeViewDisplayData.Children.Add(row11);
                        TreeViewDisplayData.Children.Add(row12);
                        break;
                    }
                case "Motor6":
                    {
                        foreach (Label lable in TreeViewLables)
                        {
                            lable.Foreground = defaultLabelColor;
                        }
                        foreach (Label lable in TreeViewLables.Where(s => s.Name.EndsWith("6")))
                        {
                            lable.Foreground = Brushes.Red;
                        }
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        row11.Children.Clear();
                        row12.Children.Clear();
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Angle1"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "angle1"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Angle2"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "angle2"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Angle3"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "angle3"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Angle4"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "angle4"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Angle5"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "angle5"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Angle6"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "angle6"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "PosX"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "posX"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "PosY"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "posY"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "PosZ"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "posZ"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Alpha"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "alpha"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "Beta"));
                        row11.Children.Add(TreeViewLables.Find(n => n.Name == "beta"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "Gamma"));
                        row12.Children.Add(TreeViewLables.Find(n => n.Name == "gamma"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        TreeViewDisplayData.Children.Add(row11);
                        TreeViewDisplayData.Children.Add(row12);
                        break;
                    }
                case "Module":
                    {
                        TreeViewDisplayData.Children.Clear();
                        Label module = TreeViewLables.Find(n => n.Name == "module");
                        module.Content = "Module 1 bis 6 Status: " + Modul_statusstring;
                        TreeViewDisplayData.Children.Add(module);
                        break;
                    }
                case "Modul1":
                    {
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        Label l = TreeViewLables.Find(n => n.Name == "module_Status");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][0]);
                        l = TreeViewLables.Find(n => n.Name == "module_Address");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][1]);
                        l = TreeViewLables.Find(n => n.Name == "module_Baud");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][2]);
                        l = TreeViewLables.Find(n => n.Name == "module_mFrequency");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][3]);
                        l = TreeViewLables.Find(n => n.Name == "module_PulseDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][4]);
                        l = TreeViewLables.Find(n => n.Name == "module_RampDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][5]);
                        l = TreeViewLables.Find(n => n.Name == "module_Current");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][6]);
                        l = TreeViewLables.Find(n => n.Name == "module_acc");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][7]);
                        l = TreeViewLables.Find(n => n.Name == "module_speed");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][8]);
                        l = TreeViewLables.Find(n => n.Name == "module_pos");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[0][9]);
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Status"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "module_Status"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Adress"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "module_Address"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Baud"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "module_Baud"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_mFrequency"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "module_mFrequency"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_PulseDiv"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "module_PulseDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_RampDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "module_RampDiv"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Current"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "module_Current"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Acc"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "module_acc"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Speed"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "module_speed"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Position"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "module_pos"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        break;
                    }
                case "Modul2":
                    {
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        Label l = TreeViewLables.Find(n => n.Name == "module_Status");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][0]);
                        l = TreeViewLables.Find(n => n.Name == "module_Address");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][1]);
                        l = TreeViewLables.Find(n => n.Name == "module_Baud");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][2]);
                        l = TreeViewLables.Find(n => n.Name == "module_mFrequency");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][3]);
                        l = TreeViewLables.Find(n => n.Name == "module_PulseDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][4]);
                        l = TreeViewLables.Find(n => n.Name == "module_RampDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][5]);
                        l = TreeViewLables.Find(n => n.Name == "module_Current");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][6]);
                        l = TreeViewLables.Find(n => n.Name == "module_acc");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][7]);
                        l = TreeViewLables.Find(n => n.Name == "module_speed");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][8]);
                        l = TreeViewLables.Find(n => n.Name == "module_pos");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[1][9]);
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Status"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "module_Status"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Adress"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "module_Address"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Baud"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "module_Baud"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_mFrequency"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "module_mFrequency"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_PulseDiv"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "module_PulseDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_RampDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "module_RampDiv"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Current"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "module_Current"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Acc"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "module_acc"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Speed"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "module_speed"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Position"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "module_pos"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        break;
                    }
                case "Modul3":
                    {
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        Label l = TreeViewLables.Find(n => n.Name == "module_Status");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][0]);
                        l = TreeViewLables.Find(n => n.Name == "module_Address");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][1]);
                        l = TreeViewLables.Find(n => n.Name == "module_Baud");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][2]);
                        l = TreeViewLables.Find(n => n.Name == "module_mFrequency");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][3]);
                        l = TreeViewLables.Find(n => n.Name == "module_PulseDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][4]);
                        l = TreeViewLables.Find(n => n.Name == "module_RampDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][5]);
                        l = TreeViewLables.Find(n => n.Name == "module_Current");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][6]);
                        l = TreeViewLables.Find(n => n.Name == "module_acc");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][7]);
                        l = TreeViewLables.Find(n => n.Name == "module_speed");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][8]);
                        l = TreeViewLables.Find(n => n.Name == "module_pos");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[2][9]);
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Status"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "module_Status"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Adress"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "module_Address"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Baud"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "module_Baud"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_mFrequency"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "module_mFrequency"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_PulseDiv"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "module_PulseDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_RampDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "module_RampDiv"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Current"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "module_Current"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Acc"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "module_acc"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Speed"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "module_speed"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Position"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "module_pos"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        break;
                    }
                case "Modul4":
                    {
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        Label l = TreeViewLables.Find(n => n.Name == "module_Status");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][0]);
                        l = TreeViewLables.Find(n => n.Name == "module_Address");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][1]);
                        l = TreeViewLables.Find(n => n.Name == "module_Baud");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][2]);
                        l = TreeViewLables.Find(n => n.Name == "module_mFrequency");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][3]);
                        l = TreeViewLables.Find(n => n.Name == "module_PulseDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][4]);
                        l = TreeViewLables.Find(n => n.Name == "module_RampDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][5]);
                        l = TreeViewLables.Find(n => n.Name == "module_Current");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][6]);
                        l = TreeViewLables.Find(n => n.Name == "module_acc");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][7]);
                        l = TreeViewLables.Find(n => n.Name == "module_speed");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][8]);
                        l = TreeViewLables.Find(n => n.Name == "module_pos");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[3][9]);
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Status"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "module_Status"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Adress"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "module_Address"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Baud"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "module_Baud"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_mFrequency"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "module_mFrequency"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_PulseDiv"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "module_PulseDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_RampDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "module_RampDiv"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Current"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "module_Current"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Acc"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "module_acc"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Speed"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "module_speed"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Position"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "module_pos"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        break;
                    }
                case "Modul5":
                    {
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        Label l = TreeViewLables.Find(n => n.Name == "module_Status");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][0]);
                        l = TreeViewLables.Find(n => n.Name == "module_Address");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][1]);
                        l = TreeViewLables.Find(n => n.Name == "module_Baud");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][2]);
                        l = TreeViewLables.Find(n => n.Name == "module_mFrequency");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][3]);
                        l = TreeViewLables.Find(n => n.Name == "module_PulseDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][4]);
                        l = TreeViewLables.Find(n => n.Name == "module_RampDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][5]);
                        l = TreeViewLables.Find(n => n.Name == "module_Current");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][6]);
                        l = TreeViewLables.Find(n => n.Name == "module_acc");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][7]);
                        l = TreeViewLables.Find(n => n.Name == "module_speed");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][8]);
                        l = TreeViewLables.Find(n => n.Name == "module_pos");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[4][9]);
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Status"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "module_Status"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Adress"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "module_Address"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Baud"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "module_Baud"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_mFrequency"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "module_mFrequency"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_PulseDiv"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "module_PulseDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_RampDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "module_RampDiv"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Current"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "module_Current"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Acc"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "module_acc"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Speed"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "module_speed"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Position"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "module_pos"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        break;
                    }
                case "Modul6":
                    {
                        TreeViewDisplayData.Children.Clear();
                        row1.Children.Clear();
                        row2.Children.Clear();
                        row3.Children.Clear();
                        row4.Children.Clear();
                        row5.Children.Clear();
                        row6.Children.Clear();
                        row7.Children.Clear();
                        row8.Children.Clear();
                        row9.Children.Clear();
                        row10.Children.Clear();
                        Label l = TreeViewLables.Find(n => n.Name == "module_Status");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][0]);
                        l = TreeViewLables.Find(n => n.Name == "module_Address");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][1]);
                        l = TreeViewLables.Find(n => n.Name == "module_Baud");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][2]);
                        l = TreeViewLables.Find(n => n.Name == "module_mFrequency");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][3]);
                        l = TreeViewLables.Find(n => n.Name == "module_PulseDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][4]);
                        l = TreeViewLables.Find(n => n.Name == "module_RampDiv");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][5]);
                        l = TreeViewLables.Find(n => n.Name == "module_Current");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][6]);
                        l = TreeViewLables.Find(n => n.Name == "module_acc");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][7]);
                        l = TreeViewLables.Find(n => n.Name == "module_speed");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][8]);
                        l = TreeViewLables.Find(n => n.Name == "module_pos");
                        l.SetBinding(Label.ContentProperty, ModuleBindings[5][9]);
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Status"));
                        row1.Children.Add(TreeViewLables.Find(n => n.Name == "module_Status"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Adress"));
                        row2.Children.Add(TreeViewLables.Find(n => n.Name == "module_Address"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Baud"));
                        row3.Children.Add(TreeViewLables.Find(n => n.Name == "module_Baud"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_mFrequency"));
                        row4.Children.Add(TreeViewLables.Find(n => n.Name == "module_mFrequency"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_PulseDiv"));
                        row5.Children.Add(TreeViewLables.Find(n => n.Name == "module_PulseDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_RampDiv"));
                        row6.Children.Add(TreeViewLables.Find(n => n.Name == "module_RampDiv"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Current"));
                        row7.Children.Add(TreeViewLables.Find(n => n.Name == "module_Current"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Acc"));
                        row8.Children.Add(TreeViewLables.Find(n => n.Name == "module_acc"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Speed"));
                        row9.Children.Add(TreeViewLables.Find(n => n.Name == "module_speed"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "Modul_Position"));
                        row10.Children.Add(TreeViewLables.Find(n => n.Name == "module_pos"));
                        TreeViewDisplayData.Children.Add(row1);
                        TreeViewDisplayData.Children.Add(row2);
                        TreeViewDisplayData.Children.Add(row3);
                        TreeViewDisplayData.Children.Add(row4);
                        TreeViewDisplayData.Children.Add(row5);
                        TreeViewDisplayData.Children.Add(row6);
                        TreeViewDisplayData.Children.Add(row7);
                        TreeViewDisplayData.Children.Add(row8);
                        TreeViewDisplayData.Children.Add(row9);
                        TreeViewDisplayData.Children.Add(row10);
                        break;
                    }
            }
        }

        private void initTreeViewDataLabelsAndUniformGrid()
        {
            TreeViewLables.Add(new Label() { Name = "angle1" });
            TreeViewLables.Add(new Label() { Name = "angle2" });
            TreeViewLables.Add(new Label() { Name = "angle3" });
            TreeViewLables.Add(new Label() { Name = "angle4" });
            TreeViewLables.Add(new Label() { Name = "angle5" });
            TreeViewLables.Add(new Label() { Name = "angle6" });
            TreeViewLables.Add(new Label() { Name = "Angle1" });
            TreeViewLables.Add(new Label() { Name = "Angle2" });
            TreeViewLables.Add(new Label() { Name = "Angle3" });
            TreeViewLables.Add(new Label() { Name = "Angle4" });
            TreeViewLables.Add(new Label() { Name = "Angle5" });
            TreeViewLables.Add(new Label() { Name = "Angle6" });
            TreeViewLables.Add(new Label() { Name = "posX" });
            TreeViewLables.Add(new Label() { Name = "posY" });
            TreeViewLables.Add(new Label() { Name = "posZ" });
            TreeViewLables.Add(new Label() { Name = "alpha" });
            TreeViewLables.Add(new Label() { Name = "beta" });
            TreeViewLables.Add(new Label() { Name = "gamma" });
            TreeViewLables.Add(new Label() { Name = "PosX" });
            TreeViewLables.Add(new Label() { Name = "PosY" });
            TreeViewLables.Add(new Label() { Name = "PosZ" });
            TreeViewLables.Add(new Label() { Name = "Alpha" });
            TreeViewLables.Add(new Label() { Name = "Beta" });
            TreeViewLables.Add(new Label() { Name = "Gamma" });
            TreeViewLables.Add(new Label() { Name = "Modul_Status" });
            TreeViewLables.Add(new Label() { Name = "Modul_Current" });
            TreeViewLables.Add(new Label() { Name = "Modul_Position" });
            TreeViewLables.Add(new Label() { Name = "Modul_mFrequency" });
            TreeViewLables.Add(new Label() { Name = "Modul_Adress" });
            TreeViewLables.Add(new Label() { Name = "Modul_Speed" });
            TreeViewLables.Add(new Label() { Name = "Modul_Acc" });
            TreeViewLables.Add(new Label() { Name = "Modul_RampDiv" });
            TreeViewLables.Add(new Label() { Name = "Modul_PulseDiv" });
            TreeViewLables.Add(new Label() { Name = "Modul_Baud" });
            TreeViewLables.Add(new Label() { Name = "module" });
            TreeViewLables.Add(new Label() { Name = "module_Address" });
            TreeViewLables.Add(new Label() { Name = "module_pos" });
            TreeViewLables.Add(new Label() { Name = "module_mFrequency" });
            TreeViewLables.Add(new Label() { Name = "module_speed" });
            TreeViewLables.Add(new Label() { Name = "module_acc" });
            TreeViewLables.Add(new Label() { Name = "module_Current" });
            TreeViewLables.Add(new Label() { Name = "module_Baud" });
            TreeViewLables.Add(new Label() { Name = "module_PulseDiv" });
            TreeViewLables.Add(new Label() { Name = "module_RampDiv" });
            TreeViewLables.Add(new Label() { Name = "module_Status" });

            foreach (Label label in TreeViewLables)
            {
                label.Foreground = defaultLabelColor;
                label.FontSize = defaultTreeViewLabeFontSize;
            }

            Label l = TreeViewLables.Find(n => n.Name == "module_Status");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][0]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("Status")));
            l = TreeViewLables.Find(n => n.Name == "module_Address");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][1]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("Adress")));
            l = TreeViewLables.Find(n => n.Name == "module_Baud");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][2]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("Baudrate")));
            l = TreeViewLables.Find(n => n.Name == "module_mFrequency");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][3]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("mFrequency")));
            l = TreeViewLables.Find(n => n.Name == "module_PulseDiv");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][4]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("PulseDiv")));
            l = TreeViewLables.Find(n => n.Name == "module_RampDiv");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][5]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("RampDiv")));
            l = TreeViewLables.Find(n => n.Name == "module_Current");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][6]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("Current")));
            l = TreeViewLables.Find(n => n.Name == "module_acc");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][7]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("Acceleration")));
            l = TreeViewLables.Find(n => n.Name == "module_speed");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][8]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("Speed")));
            l = TreeViewLables.Find(n => n.Name == "module_pos");
            l.SetBinding(Label.ContentProperty, ModuleBindings[0][9]);
            //l.SetBinding(Label.ContentProperty, ModuleBindings.Find(b => b.Path.ToString().Contains("Position")));
            row1.Rows = 1;
            row1.Width = 300;
            row2.Rows = 1;
            row2.Width = 300;
            row3.Rows = 1;
            row3.Width = 300;
            row4.Rows = 1;
            row4.Width = 300;
            row5.Rows = 1;
            row5.Width = 300;
            row6.Rows = 1;
            row6.Width = 300;
            row7.Rows = 1;
            row7.Width = 300;
            row8.Rows = 1;
            row8.Width = 300;
            row9.Rows = 1;
            row9.Width = 300;
            row10.Rows = 1;
            row10.Width = 300;
            row11.Rows = 1;
            row11.Width = 300;
            row12.Rows = 1;
            row12.Width = 300;
            l = TreeViewLables.Find(n => n.Name == "angle1");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[0]);
            l = TreeViewLables.Find(n => n.Name == "angle2");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[1]);
            l = TreeViewLables.Find(n => n.Name == "angle3");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[2]);
            l = TreeViewLables.Find(n => n.Name == "angle4");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[3]);
            l = TreeViewLables.Find(n => n.Name == "angle5");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[4]);
            l = TreeViewLables.Find(n => n.Name == "angle6");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[5]);
            l = TreeViewLables.Find(n => n.Name == "posX");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[6]);
            l = TreeViewLables.Find(n => n.Name == "posY");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[7]);
            l = TreeViewLables.Find(n => n.Name == "posZ");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[8]);
            l = TreeViewLables.Find(n => n.Name == "alpha");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[9]);
            l = TreeViewLables.Find(n => n.Name == "beta");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[10]);
            l = TreeViewLables.Find(n => n.Name == "gamma");
            l.SetBinding(Label.ContentProperty, ActualPositionBindings[11]);
            l = TreeViewLables.Find(n => n.Name == "Angle1");
            l.Content = "Teta1";
            l = TreeViewLables.Find(n => n.Name == "Angle2");
            l.Content = "Teta2";
            l = TreeViewLables.Find(n => n.Name == "Angle3");
            l.Content = "Teta3";
            l = TreeViewLables.Find(n => n.Name == "Angle4");
            l.Content = "Teta4";
            l = TreeViewLables.Find(n => n.Name == "Angle5");
            l.Content = "Teta5";
            l = TreeViewLables.Find(n => n.Name == "Angle6");
            l.Content = "Teta6";
            l = TreeViewLables.Find(n => n.Name == "PosX");
            l.Content = "PosX";
            l = TreeViewLables.Find(n => n.Name == "PosY");
            l.Content = "PosY";
            l = TreeViewLables.Find(n => n.Name == "PosZ");
            l.Content = "PosZ";
            l = TreeViewLables.Find(n => n.Name == "Alpha");
            l.Content = "Alpha";
            l = TreeViewLables.Find(n => n.Name == "Beta");
            l.Content = "Beta";
            l = TreeViewLables.Find(n => n.Name == "Gamma");
            l.Content = "Gamma";
            l = TreeViewLables.Find(n => n.Name == "Modul_Status");
            l.Content = "Modul Status";
            l = TreeViewLables.Find(n => n.Name == "Modul_Current");
            l.Content = "Modul Current";
            l = TreeViewLables.Find(n => n.Name == "Modul_Position");
            l.Content = "Modul Position";
            l = TreeViewLables.Find(n => n.Name == "Modul_mFrequency");
            l.Content = "Modul mFrequency";
            l = TreeViewLables.Find(n => n.Name == "Modul_Adress");
            l.Content = "Modul Adress";
            l = TreeViewLables.Find(n => n.Name == "Modul_Speed");
            l.Content = "Modul Speed";
            l = TreeViewLables.Find(n => n.Name == "Modul_Acc");
            l.Content = "Modul Acc";
            l = TreeViewLables.Find(n => n.Name == "Modul_RampDiv");
            l.Content = "Modul RampDiv";
            l = TreeViewLables.Find(n => n.Name == "Modul_PulseDiv");
            l.Content = "Modul PulseDiv";
            l = TreeViewLables.Find(n => n.Name == "Modul_Baud");
            l.Content = "Modul Baud";


            //PosX.Content = "PosX";
            //PosY.Content = "PosY";
            //PosZ.Content = "PosZ";
            //Alpha.Content = "Alpha";
            //Beta.Content = "Beta";
            //Gamma.Content = "Gamma";
            //Modul_Status.Foreground = defaultLabelColor;
            //Modul_Status.Content = "Stauts:";
            //Modul_Status.FontSize = defaultTreeViewLabeFontSize;
            //Modul_Adress.Foreground = defaultLabelColor;
            //Modul_Adress.Content = "Adress:";
            //Modul_Adress.FontSize = defaultTreeViewLabeFontSize;
            //Modul_Baud.Foreground = defaultLabelColor;
            //Modul_Baud.Content = "BaudRate:";
            //Modul_Baud.FontSize = defaultTreeViewLabeFontSize;
            //Modul_mFrequency.Foreground = defaultLabelColor;
            //Modul_mFrequency.Content = "mFrequency:";
            //Modul_mFrequency.FontSize = defaultTreeViewLabeFontSize;
            //Modul_PulseDiv.Foreground = defaultLabelColor;
            //Modul_PulseDiv.Content = "PulseDiv:";
            //Modul_PulseDiv.FontSize = defaultTreeViewLabeFontSize;
            //Modul_RampDiv.Foreground = defaultLabelColor;
            //Modul_RampDiv.Content = "RampDiv:";
            //Modul_RampDiv.FontSize = defaultTreeViewLabeFontSize;
            //Modul_Current.Foreground = defaultLabelColor;
            //Modul_Current.Content = "Strom:";
            //Modul_Current.FontSize = defaultTreeViewLabeFontSize;
            //Modul_Acc.Foreground = defaultLabelColor;
            //Modul_Acc.Content = "Beschleunigung:";
            //Modul_Acc.FontSize = defaultTreeViewLabeFontSize;
            //Modul_Speed.Foreground = defaultLabelColor;
            //Modul_Speed.Content = "Geschwindigkeit:";
            //Modul_Speed.FontSize = defaultTreeViewLabeFontSize;
            //Modul_Position.Foreground = defaultLabelColor;
            //Modul_Position.Content = "Position:";
            //Modul_Position.FontSize = defaultTreeViewLabeFontSize;
            //Winkel1.Content = "Winkel1:";
            //Winkel1.FontSize = defaultTreeViewLabeFontSize;
            //Winkel2.Content = "Winkel2:";
            //Winkel2.FontSize = defaultTreeViewLabeFontSize;
            //Winkel3.Content = "Winkel3:";
            //Winkel3.FontSize = defaultTreeViewLabeFontSize;
            //Winkel4.Content = "Winkel4:";
            //Winkel4.FontSize = defaultTreeViewLabeFontSize;
            //Winkel5.Content = "Winkel5:";
            //Winkel5.FontSize = defaultTreeViewLabeFontSize;
            //Winkel6.Content = "Winkel6:";
            //Winkel6.FontSize = defaultTreeViewLabeFontSize;
        }
        #endregion

        #region NC
        private void StartNCCalculation(object sender, RoutedEventArgs e)
        {          
            int[] startpos = new int[2];
            startpos[0] = 0;
            startpos[1] = (int)(list.Count) / 2;
            if (list.Count > 4)
            {
                if (NCControls.Count < 1)
                {
                    NCControls = ResourceFactory.AddNumericalControlSemaphor(list, int.Parse(MilisTextBox.Text), startpos[1] + 1, DHParam);// dank dem +1 wird auch noch der startwert des 2.Task gerechnet, sonst ginge der unter.
                    NCControls = ResourceFactory.AddNumericalControlSemaphor(list, int.Parse(MilisTextBox.Text), list.Count, DHParam);
                }
                Task[] TaskArray = new Task[2];
                TaskArray[0] = Task.Factory.StartNew(() => NCControls[0].GetStartPos(startpos[0], "Thread1"));
                TaskArray[1] = Task.Factory.StartNew(() => NCControls[1].GetStartPos(startpos[1], "Thread2"));
                Task.WaitAll(TaskArray);
                foreach (var pos in NCControls[0].GCodePos)
                {
                    GCodeListe.Add(pos);
                }
                foreach (var pos in NCControls[1].GCodePos)
                {
                    GCodeListe.Add(pos);
                }

                foreach (var pos in NCControls[0].GCodeW)
                {
                    GCodeWinkel.Add(pos);
                }
                foreach (var pos in NCControls[1].GCodeW)
                {
                    GCodeWinkel.Add(pos);
                }

                foreach (var speed in NCControls[0].GCodeSpeed)
                {
                    GCodeListeSpeed.Add(speed);
                }
                foreach (var speed in NCControls[1].GCodeSpeed)
                {
                    GCodeListeSpeed.Add(speed);
                }


                foreach (var speed in NCControls[0].GCodeDelta)
                {
                    GCodeDeltaPos.Add(speed);
                }
                foreach (var speed in NCControls[1].GCodeDelta)
                {
                    GCodeDeltaPos.Add(speed);
                }

                ////////diagnose
                foreach (var pos in NCControls[0].GX)
                {
                    GXD.Add(pos);
                }
                foreach (var pos in NCControls[1].GX)
                {
                    GXD.Add(pos);
                }
                foreach (var pos in NCControls[0].GY)
                {
                    GYD.Add(pos);
                }
                foreach (var pos in NCControls[1].GY)
                {
                    GYD.Add(pos);
                }
                foreach (var pos in NCControls[0].GZ)
                {
                    GZD.Add(pos);
                }
                foreach (var pos in NCControls[1].GZ)
                {
                    GZD.Add(pos);
                }
                foreach (var pos in NCControls[0].GA)
                {
                    GAD.Add(pos);
                }
                foreach (var pos in NCControls[1].GA)
                {
                    GAD.Add(pos);
                }
                foreach (var pos in NCControls[0].GB)
                {
                    GBD.Add(pos);
                }
                foreach (var pos in NCControls[1].GB)
                {
                    GBD.Add(pos);
                }
                foreach (var pos in NCControls[0].GC)
                {
                    GCD.Add(pos);
                }
                foreach (var pos in NCControls[1].GC)
                {
                    GCD.Add(pos);
                }
                foreach (var pos in NCControls[0].GStr)
                {
                    GDiagStr.Add(pos);
                }
                foreach (var pos in NCControls[1].GStr)
                {
                    GDiagStr.Add(pos);
                }
                foreach (var pos in NCControls[0].GStrThreadNr)
                {
                    GDiagStrThreadNr.Add(pos);
                }
                foreach (var pos in NCControls[1].GStrThreadNr)
                {
                    GDiagStrThreadNr.Add(pos);
                }
            }
            //für kleine G-Codes nur in einem task rechnen.
            else
            {
                if (NCControls.Count < 1)
                {
                    NCControls = ResourceFactory.AddNumericalControlSemaphor(list, int.Parse(MilisTextBox.Text), list.Count, DHParam);
                }
                Task[] TaskArray = new Task[1];
                TaskArray[0] = Task.Factory.StartNew(() => NCControls[0].GetStartPos(startpos[0], "Thread1"));
                Task.WaitAll(TaskArray);
                foreach (var pos in NCControls[0].GCodePos)
                {
                    GCodeListe.Add(pos);
                }
                foreach (var pos in NCControls[0].GCodeW)
                {
                    GCodeWinkel.Add(pos);
                }
                foreach (var speed in NCControls[0].GCodeSpeed)
                {
                    GCodeListeSpeed.Add(speed);
                }
                foreach (var speed in NCControls[0].GCodeDelta)
                {
                    GCodeDeltaPos.Add(speed);
                }
                foreach (var pos in NCControls[0].GX)
                {
                    GXD.Add(pos);
                }
                foreach (var pos in NCControls[0].GY)
                {
                    GYD.Add(pos);
                }
                foreach (var pos in NCControls[0].GZ)
                {
                    GZD.Add(pos);
                }
                foreach (var pos in NCControls[0].GA)
                {
                    GAD.Add(pos);
                }
                foreach (var pos in NCControls[0].GB)
                {
                    GBD.Add(pos);
                }
                foreach (var pos in NCControls[0].GC)
                {
                    GCD.Add(pos);
                }
                foreach (var pos in NCControls[0].GStr)
                {
                    GDiagStr.Add(pos);
                }
                foreach (var pos in NCControls[0].GStrThreadNr)
                {
                    GDiagStrThreadNr.Add(pos);
                }
            }
            G_Code_Diagnose.IsEnabled = true;
            G_Code_Converter.IsEnabled = true;
            G_Code_Start.IsEnabled = true;
            NumericalControl.DeleteInstanceSemaphorAt(1);
            NumericalControl.DeleteInstanceSemaphorAt(0);
            MessageBox.Show("Anzahl Listenelemente: " + (GCodeListe.Count + GCodeListeSpeed.Count + GCodeDeltaPos.Count).ToString());
        }
        private void StartGCode(object sender, RoutedEventArgs e)
        {
            if (GCodeListe.Count > 0 && int.Parse(MilisTextBox.Text) > 10)
            {
                GCodeSimulationsCounter = 0;
                GCodeSimulationsCounter2 = 0;
                G_Code_Stop.IsEnabled = true;
                G_Code_Continue.IsEnabled = true;
                Start = true;
                GCodeListCounter = 0;
                GCodeListSpeedCounter = 0;
                GCodeAblaufTimer.Interval = new TimeSpan(0, 0, 0, 0, int.Parse(MilisTextBox.Text) - 10);//-10ms weil der tick zum senden in etwa soviel braucht
                GCodeAblaufTimer.Start();
                Daten_Simulation_Tab.IsSelected = true;
                G_Code_Start.IsEnabled = false;
            }
            else
            {
                MessageBox.Show("G-Code noch nicht vorbereitet \nOder Zeit ungültig");
            }
        }
        private void StopGCode(object sender, RoutedEventArgs e)
        {
            GCodeAblaufTimer.Stop();
            Start = false;
        }
        private void ContinueGCode(object sender, RoutedEventArgs e)
        {
            Start = true;
            GCodeAblaufTimer.Interval = new TimeSpan(0, 0, 0, 0, int.Parse(MilisTextBox.Text) - 10);//-10ms weil der tick zum senden in etwa soviel braucht
            GCodeAblaufTimer.Start();
            Daten_Simulation_Tab.IsSelected = true;
        }
        private void ClearGCode(object sender, RoutedEventArgs e)
        {
            G_Code_Diagnose.IsEnabled = false;
            G_Code_Converter.IsEnabled = false;
            G_Code_Start.IsEnabled = false;
            StartNCCalc.IsEnabled = false;
            G_Code_Clear.IsEnabled = false;
            G_Code_Stop.IsEnabled = false;
            G_Code_Continue.IsEnabled = false;
            list.Clear();
            GCodeListe.Clear();
            GCodeListeSpeed.Clear();
            GCodeDeltaPos.Clear();
            GCodeWinkel.Clear();
            GXD.Clear();
            GYD.Clear();
            GZD.Clear();
            GAD.Clear();
            GBD.Clear();
            GCD.Clear();
            GDiagStr.Clear();
            NCDataCollection.Clear();
            dataGrid.DataContext = list.Select(x => new { Value = x }).ToList();
            dataGrid.ItemsSource = list.Select(x => new { Value = x }).ToList();
            tabelle1.ItemsSource = null;
            GCodeListCounter = 0;
            GCodeListSpeedCounter = 0;
            GCodeSimulationsCounter2 = 0;
            GCodeSimulationsCounter = 0;

            //Not implemented yet. see also gecodeablauf_Tick2
            //Linepoint3D_Array = null;
            //GeometryModel3d_Line.Clear();
            //Geometry3DContainer_Line.Children.Clear();

            p = 0;
            pos = 0;
            Start = false;
        }
        private void gcodeablauf_Tick2(object sender, EventArgs e)
        {
            if (Start == true && mode == Mode_.Normal)
            {
                PortCommands();
                //if (GCodeSimulationsCounter < GCodeListe.Count)
                //{
                //    Simulation3D.MoveSimulation((double)GCodeWinkel[GCodeSimulationsCounter], (double)GCodeWinkel[GCodeSimulationsCounter + 1], (double)GCodeWinkel[GCodeSimulationsCounter + 2], (double)GCodeWinkel[GCodeSimulationsCounter + 3], (double)GCodeWinkel[GCodeSimulationsCounter + 4], (double)GCodeWinkel[GCodeSimulationsCounter + 5]);
                //    DataControl.ActualPosition.PosX = Math.Round((double)GXD[GCodeSimulationsCounter2], 4);
                //    DataControl.ActualPosition.PosY = Math.Round((double)GYD[GCodeSimulationsCounter2], 4);
                //    DataControl.ActualPosition.PosZ = Math.Round((double)GZD[GCodeSimulationsCounter2], 4);
                //    DataControl.ActualPosition.Alpha = Math.Round((double)GAD[GCodeSimulationsCounter2], 4);
                //    DataControl.ActualPosition.Beta = Math.Round((double)GBD[GCodeSimulationsCounter2], 4);
                //    DataControl.ActualPosition.Gamma = Math.Round((double)GCD[GCodeSimulationsCounter2], 4);
                //    DataControl.ActualPosition.Winkel1 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter], 3);
                //    DataControl.ActualPosition.Winkel2 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 1], 3);
                //    DataControl.ActualPosition.Winkel3 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 2], 3);
                //    DataControl.ActualPosition.Winkel4 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 3], 3);
                //    DataControl.ActualPosition.Winkel5 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 4], 3);
                //    DataControl.ActualPosition.Winkel6 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 5], 3);
                //    GCodeSimulationsCounter += 6;
                //    #region 3D-line not implemented yet
                //    //if (Linepoint3D_Array == null)
                //    //{
                //    //    Linepoint3D_Array = new Point3D[GXD.Count];//muss hier stehen, da ArrayX erst in der schlaufe oben definiert wird
                //    //}
                //    //Linepoint3D_Array[GCodeSimulationsCounter2] = new Point3D((double)GYD[GCodeSimulationsCounter2], (double)GZD[GCodeSimulationsCounter2], (double)GXD[GCodeSimulationsCounter2]);
                //    //if (GCodeSimulationsCounter2 > 0)
                //    //{
                //    //    GeometryModel3d_Line.Add(Simulation_helfer.Make3DGeometry_Over3Points(Linepoint3D_Array[GCodeSimulationsCounter2 - 1], Linepoint3D_Array[GCodeSimulationsCounter2], 0.1));
                //    //    Geometry3DContainer_Line.Children.Add(GeometryModel3d_Line[GCodeSimulationsCounter2 - 1]);       //jeder Geometry3DModel muss einzeln der Model3DGroup hinzugefügt werden        
                //    //}
                //    #endregion
                //    GCodeSimulationsCounter2++;
                //}
                //else
                //{
                //    if (GCodeAblaufTimer.IsEnabled == true)
                //    {
                //        GCodeAblaufTimer.Stop();
                //    }
                //    Start = false;
                //    G_Code_Start.IsEnabled = true;
                //}
            }
            else if(Start == true && mode == Mode_.Simulation)
            {
                if (GCodeSimulationsCounter < GCodeListe.Count)
                {
                    Simulation3D.MoveSimulation((double)GCodeWinkel[GCodeSimulationsCounter], (double)GCodeWinkel[GCodeSimulationsCounter + 1], (double)GCodeWinkel[GCodeSimulationsCounter + 2], (double)GCodeWinkel[GCodeSimulationsCounter + 3], (double)GCodeWinkel[GCodeSimulationsCounter + 4], (double)GCodeWinkel[GCodeSimulationsCounter + 5]);
                    Winkel0_Simu.Text = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter], 3).ToString();
                    Winkel1_Simu.Text = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 1], 3).ToString();
                    Winkel2_Simu.Text = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 2], 3).ToString();
                    Winkel3_Simu.Text = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 3], 3).ToString();
                    Winkel4_Simu.Text = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 4], 3).ToString();
                    Winkel5_Simu.Text = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 5], 3).ToString();
                    GCodeSimulationsCounter += 6;
                    #region 3D-line not implemented yet
                    //if (Linepoint3D_Array == null)
                    //{
                    //    Linepoint3D_Array = new Point3D[GXD.Count];//muss hier stehen, da ArrayX erst in der schlaufe oben definiert wird
                    //}
                    //Linepoint3D_Array[GCodeSimulationsCounter2] = new Point3D((double)GYD[GCodeSimulationsCounter2], (double)GZD[GCodeSimulationsCounter2], (double)GXD[GCodeSimulationsCounter2]);
                    //if (GCodeSimulationsCounter2 > 0)
                    //{
                    //    GeometryModel3d_Line.Add(Simulation_helfer.Make3DGeometry_Over3Points(Linepoint3D_Array[GCodeSimulationsCounter2 - 1], Linepoint3D_Array[GCodeSimulationsCounter2], 0.1));
                    //    Geometry3DContainer_Line.Children.Add(GeometryModel3d_Line[GCodeSimulationsCounter2 - 1]);       //jeder Geometry3DModel muss einzeln der Model3DGroup hinzugefügt werden        
                    //}
                    #endregion
                    GCodeSimulationsCounter2++;
                }
                else
                {
                    if (GCodeAblaufTimer.IsEnabled == true)
                    {
                        GCodeAblaufTimer.Stop();
                    }
                    Start = false;
                    G_Code_Start.IsEnabled = true;
                }
            }
        }//aboniert in der main methode
        public void PortCommands()
        {
            if (GCodeListCounter < GCodeListe.Count)
            {
                try
                {
                    for (int q = 0; q < 6; q++)
                    {
                        MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_SAP, MAX_SPEED, Motor0, (int)GCodeListeSpeed[GCodeListSpeedCounter]);
                        GCodeListSpeedCounter++;
                    }
                    //MotorControllerMaster.MotorController[0].SendCommand(Adress1, TMCL_SAP, MAX_SPEED, Motor0, (int)GCodeListeSpeed[GCodeListSpeedCounter]);
                    //GCodeListSpeedCounter++;
                    //MotorControllerMaster.MotorController[1].SendCommand(Adress2, TMCL_SAP, MAX_SPEED, Motor0, (int)GCodeListeSpeed[GCodeListSpeedCounter]);
                    //GCodeListSpeedCounter++;
                    //MotorControllerMaster.MotorController[2].SendCommand(Adress3, TMCL_SAP, MAX_SPEED, Motor0, (int)GCodeListeSpeed[GCodeListSpeedCounter]);
                    //GCodeListSpeedCounter++;
                    //MotorControllerMaster.MotorController[3].SendCommand(Adress4, TMCL_SAP, MAX_SPEED, Motor0, (int)GCodeListeSpeed[GCodeListSpeedCounter]);
                    //GCodeListSpeedCounter++;
                    //MotorControllerMaster.MotorController[4].SendCommand(Adress5, TMCL_SAP, MAX_SPEED, Motor0, (int)GCodeListeSpeed[GCodeListSpeedCounter]);
                    //GCodeListSpeedCounter++;
                    //MotorControllerMaster.MotorController[5].SendCommand(Adress6, TMCL_SAP, MAX_SPEED, Motor0, (int)GCodeListeSpeed[GCodeListSpeedCounter]);
                    //GCodeListSpeedCounter++;
                    for (int q = 0; q < 6; q++)
                    {
                        MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_MVP, MVP_ABS, Motor0, (int)GCodeListe[GCodeListCounter]);
                        GCodeListCounter++;
                    }
                    //MotorControllerMaster.MotorController[0].SendCommand(Adress1, TMCL_MVP, MVP_ABS, Motor0, (int)GCodeListe[GCodeListCounter]);
                    //GCodeListCounter++;
                    //MotorControllerMaster.MotorController[1].SendCommand(Adress2, TMCL_MVP, MVP_ABS, Motor0, (int)GCodeListe[GCodeListCounter]);
                    //GCodeListCounter++;
                    //MotorControllerMaster.MotorController[2].SendCommand(Adress3, TMCL_MVP, MVP_ABS, Motor0, (int)GCodeListe[GCodeListCounter]);
                    //GCodeListCounter++;
                    //MotorControllerMaster.MotorController[3].SendCommand(Adress4, TMCL_MVP, MVP_ABS, Motor0, (int)GCodeListe[GCodeListCounter]);
                    //GCodeListCounter++;
                    //MotorControllerMaster.MotorController[4].SendCommand(Adress5, TMCL_MVP, MVP_ABS, Motor0, (int)GCodeListe[GCodeListCounter]);
                    //GCodeListCounter++;
                    //MotorControllerMaster.MotorController[5].SendCommand(Adress6, TMCL_MVP, MVP_ABS, Motor0, (int)GCodeListe[GCodeListCounter]);
                    //GCodeListCounter++;

                    Simulation3D.MoveSimulation((double)GCodeWinkel[GCodeSimulationsCounter], (double)GCodeWinkel[GCodeSimulationsCounter + 1], (double)GCodeWinkel[GCodeSimulationsCounter + 2], (double)GCodeWinkel[GCodeSimulationsCounter + 3], (double)GCodeWinkel[GCodeSimulationsCounter + 4], (double)GCodeWinkel[GCodeSimulationsCounter + 5]);
                    DataControl.ActualPosition.PosX = Math.Round((double)GXD[GCodeSimulationsCounter2], 4);
                    DataControl.ActualPosition.PosY = Math.Round((double)GYD[GCodeSimulationsCounter2], 4);
                    DataControl.ActualPosition.PosZ = Math.Round((double)GZD[GCodeSimulationsCounter2], 4);
                    DataControl.ActualPosition.Alpha = Math.Round((double)GAD[GCodeSimulationsCounter2], 4);
                    DataControl.ActualPosition.Beta = Math.Round((double)GBD[GCodeSimulationsCounter2], 4);
                    DataControl.ActualPosition.Gamma = Math.Round((double)GCD[GCodeSimulationsCounter2], 4);
                    DataControl.ActualPosition.Winkel1 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter], 3);
                    DataControl.ActualPosition.Winkel2 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 1], 3);
                    DataControl.ActualPosition.Winkel3 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 2], 3);
                    DataControl.ActualPosition.Winkel4 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 3], 3);
                    DataControl.ActualPosition.Winkel5 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 4], 3);
                    DataControl.ActualPosition.Winkel6 = Math.Round((double)GCodeWinkel[GCodeSimulationsCounter + 5], 3);
                    GCodeSimulationsCounter += 6;
                    #region 3D-line not implemented yet
                    //if (Linepoint3D_Array == null)
                    //{
                    //    Linepoint3D_Array = new Point3D[GXD.Count];//muss hier stehen, da ArrayX erst in der schlaufe oben definiert wird
                    //}
                    //Linepoint3D_Array[GCodeSimulationsCounter2] = new Point3D((double)GYD[GCodeSimulationsCounter2], (double)GZD[GCodeSimulationsCounter2], (double)GXD[GCodeSimulationsCounter2]);
                    //if (GCodeSimulationsCounter2 > 0)
                    //{
                    //    GeometryModel3d_Line.Add(Simulation_helfer.Make3DGeometry_Over3Points(Linepoint3D_Array[GCodeSimulationsCounter2 - 1], Linepoint3D_Array[GCodeSimulationsCounter2], 0.1));
                    //    Geometry3DContainer_Line.Children.Add(GeometryModel3d_Line[GCodeSimulationsCounter2 - 1]);       //jeder Geometry3DModel muss einzeln der Model3DGroup hinzugefügt werden        
                    //}
                    #endregion
                    GCodeSimulationsCounter2++;
                }
                catch
                {
                    if (GCodeAblaufTimer.IsEnabled == true)
                    {
                        GCodeAblaufTimer.Stop();
                    }
                    MessageBox.Show("COM-Ports nicht initialisiert oder Kinematik noch nicht gerechnet");
                    Start = false;
                    G_Code_Start.IsEnabled = true;
                }
            }
            else
            {
                if (GCodeAblaufTimer.IsEnabled == true)
                {
                    GCodeAblaufTimer.Stop();
                }
                try
                {
                    for (int q = 0; q < 6; q++)
                    {
                        MotorControllerMaster.MotorController[q].SendCommand(Addresses[q], TMCL_MST, EMPTY, Motor0, 0);
                    }
                    MessageBox.Show("GCode-Ablauf Ende");
                }
                catch
                {
                    MessageBox.Show("COM-Ports nicht angeschlossen");
                    Start = false;
                }
                Start = false;
                G_Code_Start.IsEnabled = true;
            }
        }
        private void GCodeXAML(object sender, RoutedEventArgs e)
        {
            p = 0;
            pos = 0;
            while (p < GCodeListe.Count)//egal welche ArrayList vom GCode, da alle exakt gliech lang sind
            {
                this.tabelle1.ItemsSource = GetGCodePos((int)GCodeListeSpeed[p], (int)GCodeListeSpeed[p + 1], (int)GCodeListeSpeed[p + 2], (int)GCodeListeSpeed[p + 3], (int)GCodeListeSpeed[p + 4], (int)GCodeListeSpeed[p + 5], (int)GCodeListe[p], (int)GCodeListe[p + 1], (int)GCodeListe[p + 2], (int)GCodeListe[p + 3], (int)GCodeListe[p + 4], (int)GCodeListe[p + 5], (int)GCodeDeltaPos[p], (int)GCodeDeltaPos[p + 1], (int)GCodeDeltaPos[p + 2], (int)GCodeDeltaPos[p + 3], (int)GCodeDeltaPos[p + 4], (int)GCodeDeltaPos[p + 5], Math.Round((double)GCodeWinkel[p], 1), Math.Round((double)GCodeWinkel[p + 1], 1), Math.Round((double)GCodeWinkel[p + 2], 1), Math.Round((double)GCodeWinkel[p + 3], 1), Math.Round((double)GCodeWinkel[p + 4], 1), Math.Round((double)GCodeWinkel[p + 5], 1));
                p = p + 6;
            }
        }
        private ObservableCollection<NCData> GetGCodePos(int speed1_, int speed2_, int speed3_, int speed4_, int speed5_, int speed6_, int steps1_, int steps2_, int steps3_, int steps4_, int steps5_, int steps6_, int Deltasteps1_, int Deltasteps2_, int Deltasteps3_, int Deltasteps4_, int Deltasteps5_, int Deltasteps6_, double W1, double W2, double W3, double W4, double W5, double W6)
        {
            NCDataCollection.Add(new NCData { gPos = pos, gSpeed1 = speed1_, gSpeed2 = speed2_, gSpeed3 = speed3_, gSpeed4 = speed4_, gSpeed5 = speed5_, gSpeed6 = speed6_, gSteps1 = steps1_, gSteps2 = steps2_, gSteps3 = steps3_, gSteps4 = steps4_, gSteps5 = steps5_, gSteps6 = steps6_, gDeltaSteps1 = Deltasteps1_, gDeltaSteps2 = Deltasteps2_, gDeltaSteps3 = Deltasteps3_, gDeltaSteps4 = Deltasteps4_, gDeltaSteps5 = Deltasteps5_, gDeltaSteps6 = Deltasteps6_, gWinkel1 = W1, gWinkel2 = W2, gWinkel3 = W3, gWinkel4 = W4, gWinkel5 = W5, gWinkel6 = W6 });
            pos++;
            return NCDataCollection;
        }
        private void GCodeDiagnose(object sender, RoutedEventArgs e)
        {
            DiagnoseTable = new DiagnoseTable();
            DiagnoseTable.WindowState = WindowState.Maximized;
            pos = 0;
            p = 0;
            int p_ = 0;
            while (p < GCodeListe.Count)//egal welche ArrayList vom GCode, da alle exakt gliech lang sind
            {
                DiagnoseTable.tabelle1.ItemsSource = GetDiagnose((double)GXD[p_], (double)GYD[p_], (double)GZD[p_], (double)GAD[p_], (double)GBD[p_], (double)GCD[p_], (int)GCodeListeSpeed[p], (int)GCodeListeSpeed[p + 1], (int)GCodeListeSpeed[p + 2], (int)GCodeListeSpeed[p + 3], (int)GCodeListeSpeed[p + 4], (int)GCodeListeSpeed[p + 5], (int)GCodeListe[p], (int)GCodeListe[p + 1], (int)GCodeListe[p + 2], (int)GCodeListe[p + 3], (int)GCodeListe[p + 4], (int)GCodeListe[p + 5], (int)GCodeDeltaPos[p], (int)GCodeDeltaPos[p + 1], (int)GCodeDeltaPos[p + 2], (int)GCodeDeltaPos[p + 3], (int)GCodeDeltaPos[p + 4], (int)GCodeDeltaPos[p + 5], Math.Round((double)GCodeWinkel[p], 1), Math.Round((double)GCodeWinkel[p + 1], 1), Math.Round((double)GCodeWinkel[p + 2], 1), Math.Round((double)GCodeWinkel[p + 3], 1), Math.Round((double)GCodeWinkel[p + 4], 1), Math.Round((double)GCodeWinkel[p + 5], 1), (string)GDiagStr[p_], (string)GDiagStrThreadNr[p_]);
                p_++;
                p = p + 6;
            }
            DiagnoseTable.Show();
        }
        private ObservableCollection<DiagnoseData> GetDiagnose(double gx, double gy, double gz, double ga, double gb, double gc, int speed1_, int speed2_, int speed3_, int speed4_, int speed5_, int speed6_, int steps1_, int steps2_, int steps3_, int steps4_, int steps5_, int steps6_, int Deltasteps1_, int Deltasteps2_, int Deltasteps3_, int Deltasteps4_, int Deltasteps5_, int Deltasteps6_, double W1, double W2, double W3, double W4, double W5, double W6, string diagstr, string diagstrthreadnr)
        {
            DiagnoseDataCollection.Add(new DiagnoseData { diagPos = pos, diagX = gx, diagY = gy, diagZ = gz, diagA = ga, diagB = gb, diagC = gc, gSpeed1 = speed1_, gSpeed2 = speed2_, gSpeed3 = speed3_, gSpeed4 = speed4_, gSpeed5 = speed5_, gSpeed6 = speed6_, gSteps1 = steps1_, gSteps2 = steps2_, gSteps3 = steps3_, gSteps4 = steps4_, gSteps5 = steps5_, gSteps6 = steps6_, gDeltaSteps1 = Deltasteps1_, gDeltaSteps2 = Deltasteps2_, gDeltaSteps3 = Deltasteps3_, gDeltaSteps4 = Deltasteps4_, gDeltaSteps5 = Deltasteps5_, gDeltaSteps6 = Deltasteps6_, gWinkel1 = W1, gWinkel2 = W2, gWinkel3 = W3, gWinkel4 = W4, gWinkel5 = W5, gWinkel6 = W6, DiagStr = diagstr, DiagStrTreadNr = diagstrthreadnr });
            pos++;
            return DiagnoseDataCollection;
        }
        #endregion

        #region Simulation
        private void Slider_ValueChanged_Zoom(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            var slider = sender as Slider;
            Simulation3D.SliderZoomChange(slider.Value);

        }

        private void Slider_ValueChanged_XZ(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            var slider = sender as Slider;
            Simulation3D.SliderXZCahnge(slider.Value);
        }

        private void Slider_ValueChanged_Over(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            var slider = sender as Slider;
            Simulation3D.SliderOverChange(slider.Value);
        }
        #endregion
    }
    enum Mode_
    {
        Normal, TicTacToe, Simulation, Vision
    }
}

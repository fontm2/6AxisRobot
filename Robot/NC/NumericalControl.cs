using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Kinematics;

namespace NC
{
    public class NumericalControl
    {
        //static variables are shared between all class instances.
        private static int CounterSemaphor = 2;
        private static volatile List<NumericalControl> instances;//because it's static, its for all the instances of InverseKinematics tha same Instance. You Can not create a new one, because the Constructor is private
        private static object LockObj = new Object();//this object can only be used from one Thread at the same time (UI-Thread OR Backgroundthread, but not both at a time, can use the same Instance of this Class)
        //Note that instead of locking on typeof(Singleton) as some versions of this implementation do, 
        //I lock on the value of a static variable which is private to the class. Locking on objects
        //which other classes can access and lock on (such as the type of this Class) risks performance issues 
        //and even deadlocks. Wherever possible, only lock on objects specifically created for the purpose of locking, or which document that they are 
        //to be locked on for specific purposes (e.g. for waiting/pulsing a queue). 
        //Usually such objects should be private to the class they are used in. 
        //This helps to make writing thread-safe applications significantly easier.
        List<string> InData = new List<string>();
        InverseKinematicsForNC InvKinematicForNC = null;
        int[] aktuelleposition = new int[6];
        private double inkrement = 0.1;
        private double[] IKAngle2;
        double[] startpos;
        int counter1;
        double GCodeXZeil;
        double GCodeYZeil;
        double GCodeZZeil;
        double GCodeAZeil;
        double GCodeBZeil;
        double GCodeCZeil;
        double GCodeIDouble;
        double GCodeJDouble;
        double GCodeKDouble;
        double GCodeGDouble;
        private double GCodeQDouble;
        double ybeiwert;
        double zbeiwert;
        double startgcodewinkel;
        double stopgcodewinkel;
        double radius;
        double startgcodewinkel_xz;
        double stopgcodewinkel_xz;
        double radius_xz;
        private double radius_yz;
        private double startgcodewinkel_yz;
        private double stopgcodewinkel_yz;
        private double GCodeXMomentan;
        private double GCodeYMomentan;
        private double GCodeZMomentan;
        private double GCodeAMomentan;
        private double GCodeBMomentan;
        private double GCodeCMomentan;
        private string GCodeX;
        private string GCodeY;
        private string GCodeZ;
        private string GCodeA;
        private string GCodeB;
        private string GCodeC;
        private string GCodeI;
        private string GCodeJ;
        private string GCodeK;
        private string GCodeG;
        private string GCodeQ;
        private bool cw;
        private double GCodeDeltaX;
        private double GCodeDeltaY;
        private double GCodeDeltaZ;
        private double GCodeDeltaA;
        private double GCodeDeltaB;
        private double GCodeDeltaC;
        private int[] deltaschritte;
        private int[] IKSPeedInt2;
        int Milisek;
        int Ende;
        ArrayList GCodeListe = new ArrayList();
        ArrayList GCodeDeltaPos = new ArrayList();
        ArrayList GCodeWinkel = new ArrayList();
        ArrayList GCodeListeSpeed = new ArrayList();
        ArrayList X = new ArrayList();
        ArrayList Y = new ArrayList();
        ArrayList Z = new ArrayList();
        ArrayList A = new ArrayList();
        ArrayList B = new ArrayList();
        ArrayList C = new ArrayList();
        ArrayList DiagStr = new ArrayList();
        ArrayList DiagStrThreadNr = new ArrayList();
        private double anzahl_erhoehungen_xy;
        private double anzahl_erhoehungen_xz;
        private double anzahl_erhoehungen_yz;
        private double inkrement_eulerwinkel_alpha;
        private double inkrement_eulerwinkel_beta;
        private double inkrement_eulerwinkel_gamma;
        double mitellwertbildner = 0;
        private double vorzeichenX;
        private double vorzeichenY;
        private double vorzeichenZ;
        private double vorzeichenA;
        private double vorzeichenB;
        private double vorzeichenC;
        private double xbeiwert;
        private double anzahl_erhoehungen_x;
        private double anzahl_erhoehungen_y;
        private double anzahl_erhoehungen_z;
        int[] deltaSchritte2 = new int[6];
        private double xy_KurveBeiwert;
        private double xz_KurveBeiwert;
        private double yz_KurveBeiwert;
        private bool I_On = false;
        private bool J_On = false;
        private bool K_On = false;     
        private bool Q_On;
        private double phi;
        private double radius_strich;
        private double alpha_start;
        private string GCodeP;
        private double GCodePDouble;
        private bool P_On;
        private double nu;
        double inkrement_yz_for_IJK;
        private string ThreadNr;


        private NumericalControl(List<string> indata, int millis, int ende, DHParameter DHParam)
        {
            InvKinematicForNC = new InverseKinematicsForNC(DHParam);
            this.InData = indata;
            this.Milisek = millis;
            this.Ende = ende;
        }
        /// <summary>
        /// IF the instance has allready been created, the instance created before will be given back, when calling this method. Ohterwise a new instance will be given back
        /// </summary>
        /// <param name="dhparam"></param>
        /// <returns></returns>
        public static List<NumericalControl> InstantiateSemaphor(List<string> indata, int millis, int ende, DHParameter dhparam)
        {
            if (CounterSemaphor > 0)
            {
                    lock (LockObj)//no new instance can be createt from any thread as long as the first one onlocks the object
                    {
                        if (CounterSemaphor > 0)
                        {
                            if(instances==null)
                            {
                                instances = new List<NumericalControl>();
                            }                                                 
                            instances.Add(new NumericalControl(indata, millis, ende, dhparam));
                            CounterSemaphor--;
                        }
                    }
            }
            return instances;// when using this methode, allways the same and only instance will be returned
        }
        public static List<NumericalControl> DeleteInstanceSemaphorAt(int index)
        {
            if (index > 1)
            {
                index = 1;
            }
            if (index < 0)
            {
                index = 0;
            }
            if (CounterSemaphor <2)
            {
                lock (LockObj)//no new instance can be createt from any thread as long as the first one onlocks the object
                {
                    if (CounterSemaphor <2)
                    {
                        instances.RemoveAt(index);
                        CounterSemaphor++;
                    }
                }
            }
            return instances;// when using this methode, allways the same and only instance will be returned
        }
        public void GetStartPos(int k1, string threadNr)
        {
            this.ThreadNr = threadNr;
            counter1 = k1;
            string GCodeXHelper = "";
            string GCodeYHelper = "";
            string GCodeZHelper = "";
            string GCodeAHelper = "";
            string GCodeBHelper = "";
            string GCodeCHelper = "";
            if (counter1 < Ende)
            {
                cw = false;
                startpos = new double[6];
                char[] schneidestelleHelper = new char[2];
                schneidestelleHelper[0] = ':';
                schneidestelleHelper[1] = ' ';
                string splitHelperstring = InData.ElementAt<string>(counter1);
                string[] splitHelper = splitHelperstring.Split(schneidestelleHelper[0], schneidestelleHelper[1]);
                for (int i = 0; i < splitHelper.Length; i++)
                {
                    splitHelper[i] = splitHelper[i].Trim(schneidestelleHelper[1]);
                    char[] splitedstringHelper = splitHelper[i].ToCharArray();
                    switch (splitedstringHelper[0])//füllen der string
                    {
                        case 'x':
                        case 'X':
                            GCodeXHelper = splitHelper[i];
                            char[] splitXHelper = GCodeXHelper.ToCharArray();
                            if (GCodeXHelper != "" && GCodeXHelper != null)
                            {
                                char[] zahlenwertXHelper = new char[splitXHelper.Length - 1];
                                for (int k = 0; k < splitXHelper.Length - 1; k++)
                                {
                                    zahlenwertXHelper[k] = splitXHelper[k + 1];
                                }
                                string xHelper = new string(zahlenwertXHelper);
                                startpos[0] = double.Parse(xHelper);
                            }
                            else
                            {
                                string xHelper = "0";
                                startpos[0] = double.Parse(xHelper);
                            }
                            break;
                        case 'y':
                        case 'Y':
                            GCodeYHelper = splitHelper[i];
                            char[] splitYHelper = GCodeYHelper.ToCharArray();
                            if (GCodeYHelper != "" && GCodeYHelper != null)
                            {
                                char[] zahlenwertYHelper = new char[splitYHelper.Length - 1];
                                for (int l = 0; l < splitYHelper.Length - 1; l++)
                                {
                                    zahlenwertYHelper[l] = splitYHelper[l + 1];
                                }
                                string yHelper = new string(zahlenwertYHelper);
                                startpos[1] = double.Parse(yHelper);
                            }
                            else
                            {
                                string yHelper = "0";
                                startpos[1] = double.Parse(yHelper);
                            }
                            break;
                        case 'z':
                        case 'Z':
                            GCodeZHelper = splitHelper[i];
                            char[] splitZHelper = GCodeZHelper.ToCharArray();
                            if (GCodeZHelper != "" && GCodeZHelper != null)
                            {
                                char[] zahlenwertZHelper = new char[splitZHelper.Length - 1];
                                for (int l = 0; l < splitZHelper.Length - 1; l++)
                                {
                                    zahlenwertZHelper[l] = splitZHelper[l + 1];
                                }
                                string zHelper = new string(zahlenwertZHelper);
                                startpos[2] = double.Parse(zHelper);
                            }
                            else
                            {
                                string zHelper = "0";
                                startpos[2] = double.Parse(zHelper);
                            }
                            break;
                        case 'a':
                        case 'A':
                            GCodeAHelper = splitHelper[i];
                            char[] splitAHelper = GCodeAHelper.ToCharArray();
                            if (GCodeAHelper != "" && GCodeAHelper != null)
                            {
                                char[] zahlenwertAHelper = new char[splitAHelper.Length - 1];
                                for (int l = 0; l < splitAHelper.Length - 1; l++)
                                {
                                    zahlenwertAHelper[l] = splitAHelper[l + 1];
                                }
                                string aHelper = new string(zahlenwertAHelper);
                                startpos[3] = double.Parse(aHelper);
                            }
                            else
                            {
                                string aHelper = "0";
                                startpos[3] = double.Parse(aHelper);
                            }
                            break;
                        case 'b':
                        case 'B':
                            GCodeBHelper = splitHelper[i];
                            char[] splitBHelper = GCodeBHelper.ToCharArray();
                            if (GCodeBHelper != "" && GCodeBHelper != null)
                            {
                                char[] zahlenwertBHelper = new char[splitBHelper.Length - 1];
                                for (int l = 0; l < splitBHelper.Length - 1; l++)
                                {
                                    zahlenwertBHelper[l] = splitBHelper[l + 1];
                                }
                                string bHelper = new string(zahlenwertBHelper);
                                startpos[4] = double.Parse(bHelper);
                            }
                            else
                            {
                                string bHelper = "0";
                                startpos[4] = double.Parse(bHelper);
                            }
                            break;
                        case 'c':
                        case 'C':
                            GCodeCHelper = splitHelper[i];
                            char[] splitCHelper = GCodeCHelper.ToCharArray();
                            if (GCodeCHelper != "" && GCodeCHelper != null)
                            {
                                char[] zahlenwertCHelper = new char[splitCHelper.Length - 1];
                                for (int l = 0; l < splitCHelper.Length - 1; l++)
                                {
                                    zahlenwertCHelper[l] = splitCHelper[l + 1];
                                }
                                string CHelper = new string(zahlenwertCHelper);
                                startpos[5] = double.Parse(CHelper);
                            }
                            else
                            {
                                string CHelper = "0";
                                startpos[5] = double.Parse(CHelper);
                            }
                            break;
                    }
                    //bestimmen der aktuellen posiotion in form der schritte jeder einzelner achse

                }
                double[] IK_WK = InvKinematicForNC.InverseKinematicForNC(startpos[0], startpos[1], startpos[2], startpos[3], startpos[4] - 90, startpos[5]);
                aktuelleposition[0] = (int)IK_WK[6];//nicht die globale aktuelleposition
                aktuelleposition[1] = (int)IK_WK[7];
                aktuelleposition[2] = (int)IK_WK[8];
                aktuelleposition[3] = (int)IK_WK[9];
                aktuelleposition[4] = (int)IK_WK[10];
                aktuelleposition[5] = (int)IK_WK[11];
                GCodeXMomentan = startpos[0];//müssen nur zu beginn so initialiserte werden, da momentane positionen in den jeweiligen timern immer aktualisiertwerden bei jedem tick und entsprechen dann den
                GCodeYMomentan = startpos[1];
                GCodeZMomentan = startpos[2];
                GCodeAMomentan = startpos[3];
                GCodeBMomentan = startpos[4] - 90;//-Math.PI/2, weil nullstellung bei 90 grad ist
                GCodeCMomentan = startpos[5];
                counter1++;
                GCodeAblauf();
            }
        }
        private void GCodeAblauf()
        {
            if (counter1 < Ende)
            {
                GCodeX = "";//leeren der string
                GCodeY = "";
                GCodeZ = "";
                GCodeA = "";
                GCodeB = "";
                GCodeC = "";
                GCodeI = "";
                GCodeJ = "";
                GCodeK = "";
                GCodeG = "";
                GCodeQ = "";
                GCodeP = "";
                char[] schneidestelle = new char[2];
                schneidestelle[0] = ':';
                schneidestelle[1] = ' ';
                string splitstring = InData.ElementAt<string>(counter1);
                string[] split = splitstring.Split(schneidestelle[0], schneidestelle[1]);
                for (int i = 0; i < split.Length; i++)
                {
                    split[i] = split[i].Trim(schneidestelle[1]);
                    char[] splitedstring = split[i].ToCharArray();
                    switch (splitedstring[0])//füllen der string
                    {
                        case 'x':
                        case 'X':
                            GCodeX = split[i];
                            break;
                        case 'y':
                        case 'Y':
                            GCodeY = split[i];
                            break;
                        case 'z':
                        case 'Z':
                            GCodeZ = split[i];
                            break;
                        case 'a':
                        case 'A':
                            GCodeA = split[i];
                            break;
                        case 'b':
                        case 'B':
                            GCodeB = split[i];
                            break;
                        case 'c':
                        case 'C':
                            GCodeC = split[i];
                            break;
                        case 'i':
                        case 'I':
                            GCodeI = split[i];
                            break;
                        case 'j':
                        case 'J':
                            GCodeJ = split[i];
                            break;
                        case 'k':
                        case 'K':
                            GCodeK = split[i];
                            break;
                        case 'g':
                        case 'G':
                            GCodeG = split[i];
                            break;
                        case 'q':
                        case 'Q':
                            GCodeQ = split[i];
                            break;
                        case 'p':
                        case 'P':
                            GCodeP = split[i];
                            break;
                    }
                }
                GCodeMove();
            }
            else
            {
                //invoke a messagebox and learn about lamda-expressions
            }
        }
        private void GCodeMove()
        {
            char[] splitX = GCodeX.ToCharArray();
            char[] splitY = GCodeY.ToCharArray();
            char[] splitZ = GCodeZ.ToCharArray();
            char[] splitA = GCodeA.ToCharArray();
            char[] splitB = GCodeB.ToCharArray();
            char[] splitC = GCodeC.ToCharArray();
            char[] splitI = GCodeI.ToCharArray();
            char[] splitJ = GCodeJ.ToCharArray();
            char[] splitK = GCodeK.ToCharArray();
            char[] splitG = GCodeG.ToCharArray();
            char[] splitQ = GCodeQ.ToCharArray();
            char[] splitP = GCodeP.ToCharArray();
            ///////X
            if (GCodeX != "" && GCodeX != null)
            {
                char[] zahlenwertX = new char[splitX.Length - 1];
                for (int i = 0; i < splitX.Length - 1; i++)
                {
                    zahlenwertX[i] = splitX[i + 1];
                }
                string x__ = new string(zahlenwertX);
                GCodeXZeil = double.Parse(x__);
            }
            else
            {
                GCodeXZeil = 0;
            }
            ////////Y
            if (GCodeY != "" && GCodeY != null)
            {
                char[] zahlenwertY = new char[splitY.Length - 1];
                for (int i = 0; i < splitY.Length - 1; i++)
                {
                    zahlenwertY[i] = splitY[i + 1];
                }
                string y__ = new string(zahlenwertY);
                GCodeYZeil = double.Parse(y__);
            }
            else
            {
                GCodeYZeil = 0;
            }
            ///////////Z
            if (GCodeZ != "" && GCodeZ != null)
            {
                char[] zahlenwertZ = new char[splitZ.Length - 1];
                for (int i = 0; i < splitZ.Length - 1; i++)
                {
                    zahlenwertZ[i] = splitZ[i + 1];
                }
                string z__ = new string(zahlenwertZ);
                GCodeZZeil = double.Parse(z__);
            }
            else
            {
                GCodeZZeil = 0;
            }
            ////////A
            if (GCodeA != "" && GCodeA != null)
            {
                char[] zahlenwertA = new char[splitA.Length - 1];
                for (int i = 0; i < splitA.Length - 1; i++)
                {
                    zahlenwertA[i] = splitA[i + 1];
                }
                string A__ = new string(zahlenwertA);
                GCodeAZeil = double.Parse(A__);
            }
            else
            {
                GCodeAZeil = 0;
            }
            ////////B
            if (GCodeB != "" && GCodeB != null)
            {
                char[] zahlenwertB = new char[splitB.Length - 1];
                for (int i = 0; i < splitB.Length - 1; i++)
                {
                    zahlenwertB[i] = splitB[i + 1];
                }
                string B__ = new string(zahlenwertB);
                GCodeBZeil = double.Parse(B__) - 90;//-Math.PI/2, weil nullstellung bei 90 grad ist
            }
            else
            {
                GCodeBZeil = 0;
            }
            /////////C
            if (GCodeC != "" && GCodeC != null)
            {
                char[] zahlenwertC = new char[splitC.Length - 1];
                for (int i = 0; i < splitC.Length - 1; i++)
                {
                    zahlenwertC[i] = splitC[i + 1];
                }
                string C__ = new string(zahlenwertC);
                GCodeCZeil = double.Parse(C__);
            }
            else
            {
                GCodeCZeil = 0;
            }
            ////////I
            if (GCodeI != "" && GCodeI != null)
            {
                char[] zahlenwertI = new char[splitI.Length - 1];
                for (int i = 0; i < splitI.Length - 1; i++)
                {
                    zahlenwertI[i] = splitI[i + 1];
                }
                string i_ = new string(zahlenwertI);
                GCodeIDouble = double.Parse(i_);
                I_On = false;
            }
            else
            {
                GCodeIDouble = 0;
                I_On = true;
            }
            ////////J
            if (GCodeJ != "" && GCodeJ != null)
            {
                char[] zahlenwertJ = new char[splitJ.Length - 1];
                for (int i = 0; i < splitJ.Length - 1; i++)
                {
                    zahlenwertJ[i] = splitJ[i + 1];
                }
                string j_ = new string(zahlenwertJ);
                GCodeJDouble = double.Parse(j_);
                J_On = false;
            }
            else
            {
                GCodeJDouble = 0;
                J_On = true;
            }
            /////////K
            if (GCodeK != "" && GCodeK != null)
            {
                char[] zahlenwertK = new char[splitK.Length - 1];
                for (int i = 0; i < splitK.Length - 1; i++)
                {
                    zahlenwertK[i] = splitK[i + 1];
                }
                string K_ = new string(zahlenwertK);
                GCodeKDouble = double.Parse(K_);
                K_On = false;
            }
            else
            {
                GCodeKDouble = 0;
                K_On = true;
            }
            ///////G
            if (GCodeG != "" && GCodeG != null)
            {
                char[] zahlenwertG = new char[splitG.Length - 1];
                for (int i = 0; i < splitG.Length - 1; i++)
                {
                    zahlenwertG[i] = splitG[i + 1];
                }
                string G_ = new string(zahlenwertG);
                GCodeGDouble = double.Parse(G_);
                if (GCodeGDouble == 2)
                {
                    cw = true;
                }
                else if (GCodeGDouble == 3)
                {
                    cw = false;
                }
                ///////Q
                if (GCodeQ != "" && GCodeQ != null)
                {
                    char[] zahlenwertQ = new char[splitQ.Length - 1];
                    for (int i = 0; i < splitQ.Length - 1; i++)
                    {
                        zahlenwertQ[i] = splitQ[i + 1];
                    }
                    string q_ = new string(zahlenwertQ);
                    GCodeQDouble = double.Parse(q_);
                    Q_On = false;
                }
                else
                {
                    GCodeQDouble = 0;
                    Q_On = true;
                }
                ///////P
                if (GCodeP != "" && GCodeP != null)
                {
                    char[] zahlenwertP = new char[splitP.Length - 1];
                    for (int i = 0; i < splitP.Length - 1; i++)
                    {
                        zahlenwertP[i] = splitP[i + 1];
                    }
                    string p_ = new string(zahlenwertP);
                    GCodePDouble = double.Parse(p_);
                    P_On = false;
                }
                else
                {
                    GCodePDouble = 0;
                    P_On = true;
                }
            }

            GCodeDeltaX = 0;
            GCodeDeltaY = 0;
            GCodeDeltaZ = 0;
            GCodeDeltaA = 0;
            GCodeDeltaB = 0;
            GCodeDeltaC = 0;
            GCodeDeltaX = GCodeXZeil - GCodeXMomentan;
            GCodeDeltaY = GCodeYZeil - GCodeYMomentan;
            GCodeDeltaZ = GCodeZZeil - GCodeZMomentan;
            GCodeDeltaA = GCodeAZeil - GCodeAMomentan;
            GCodeDeltaB = GCodeBZeil - GCodeBMomentan;
            GCodeDeltaC = GCodeCZeil - GCodeCMomentan;
            vorzeichenX = (GCodeXZeil - GCodeXMomentan) / Math.Abs(GCodeXZeil - GCodeXMomentan);
            vorzeichenY = (GCodeYZeil - GCodeYMomentan) / Math.Abs(GCodeYZeil - GCodeYMomentan);
            vorzeichenZ = (GCodeZZeil - GCodeZMomentan) / Math.Abs(GCodeZZeil - GCodeZMomentan);
            vorzeichenA = (GCodeAZeil - GCodeAMomentan) / Math.Abs(GCodeAZeil - GCodeAMomentan);
            vorzeichenB = (GCodeBZeil - GCodeBMomentan) / Math.Abs(GCodeBZeil - GCodeBMomentan);
            vorzeichenC = (GCodeCZeil - GCodeCMomentan) / Math.Abs(GCodeCZeil - GCodeCMomentan);


            xbeiwert = vorzeichenX * Math.Abs(GCodeDeltaX) / (Math.Abs(GCodeDeltaX) + Math.Abs(GCodeDeltaY) + Math.Abs(GCodeDeltaZ));
            ybeiwert = vorzeichenY * Math.Abs(GCodeDeltaY) / (Math.Abs(GCodeDeltaX) + Math.Abs(GCodeDeltaY) + Math.Abs(GCodeDeltaZ));
            zbeiwert = vorzeichenZ * Math.Abs(GCodeDeltaZ) / (Math.Abs(GCodeDeltaX) + Math.Abs(GCodeDeltaY) + Math.Abs(GCodeDeltaZ));
            if (GCodeDeltaX == 0)
            {
                xbeiwert = 0;
            }
            if (GCodeDeltaY == 0)
            {
                ybeiwert = 0;
            }
            if (GCodeDeltaZ == 0)
            {
                zbeiwert = 0;
            }
            if (GCodeIDouble == 0 && GCodeJDouble == 0 && GCodeKDouble == 0 && I_On == true && K_On == true && J_On == true &&  Q_On == true)
            {
                doIK();
            }
            //bewegung in xy radius
            else if (K_On == true)
            {
                radius = Math.Sqrt(GCodeIDouble * GCodeIDouble + GCodeJDouble * GCodeJDouble);
                startgcodewinkel = Math.Atan2(-GCodeJDouble, -GCodeIDouble);
                double xhilfe = GCodeXZeil - (GCodeXMomentan + GCodeIDouble);
                double yhilfe = GCodeYZeil - (GCodeYMomentan + GCodeJDouble);
                stopgcodewinkel = Math.Atan2(yhilfe, xhilfe);
                radius_xz = 0;
                radius_yz = 0;
                double AnzahlAngenommenerInkrementSchritte = Math.Abs((startgcodewinkel - stopgcodewinkel) / (inkrement / radius));//zahl, die beschreibt, wie oft der winkel bis zum zielwinkel um den schritt inkrement/radius erhöht werden muss
                xbeiwert = 0;
                ybeiwert = 0;
                if (GCodeZMomentan != GCodeZZeil)
                {
                    zbeiwert = ((GCodeZZeil - GCodeZMomentan) / inkrement) / AnzahlAngenommenerInkrementSchritte;//verhültniss zwischen der anzahl erhöhungsschritten der grösse inkrement für die lineare bewegung zu der anzahl erhöhungschritten für die kurve
                                                                                                                 //wobei das vorzeichen durch "(GCodeZZeil - GCodeZMomentan)" berücksichtigt ist
                }
                else
                {
                    zbeiwert = 0;
                }
                xy_KurveBeiwert = 1;
                xz_KurveBeiwert = 1;
                yz_KurveBeiwert = 1;
                //Schiefe Bewegung
                if (Q_On == false || P_On == false ||(Q_On == false && P_On == false ))
                {
                    phi = GCodeQDouble / 180 * Math.PI;
                    nu = GCodePDouble / 180 * Math.PI;
                    alpha_start = startgcodewinkel;
                    if (phi != 0 && nu == 0)
                    {
                        radius_strich = radius * Math.Cos(alpha_start);
                    }
                    else if (phi == 0 && nu != 0)
                    {
                        radius_strich = radius * Math.Sin(alpha_start);
                    }
                    doIKCurveForAll();
                }
                //Schiefe Bewegung
                else
                {
                    doIKCurve2();
                }
            }
            //bewegung in xz radius
            else if (J_On == true && Q_On == true)
            {
                radius_xz = Math.Sqrt(GCodeIDouble * GCodeIDouble + GCodeKDouble * GCodeKDouble);
                startgcodewinkel_xz = Math.Atan2(-GCodeKDouble, -GCodeIDouble);
                double xhilfe_xz = GCodeXZeil - (GCodeXMomentan + GCodeIDouble);
                double zhilfe = GCodeZZeil - (GCodeZMomentan + GCodeKDouble);
                stopgcodewinkel_xz = Math.Atan2(zhilfe, xhilfe_xz);
                radius_yz = 0;
                radius = 0;
                double AnzahlAngenommenerInkrementSchritte = Math.Abs((startgcodewinkel_xz - stopgcodewinkel_xz) / (inkrement / radius_xz));
                xbeiwert = 0;
                if (GCodeYMomentan != GCodeYZeil)
                {
                    ybeiwert = ((GCodeYZeil - GCodeYMomentan) / inkrement) / AnzahlAngenommenerInkrementSchritte;
                }
                else
                {
                    ybeiwert = 0;
                }
                zbeiwert = 0;
                xy_KurveBeiwert = 1;
                xz_KurveBeiwert = 1;
                yz_KurveBeiwert = 1;
                doIKCurve2();
            }
            //bewegung in yz radius
            else if (I_On == true && Q_On == true)
            {
                radius_yz = Math.Sqrt(GCodeJDouble * GCodeJDouble + GCodeKDouble * GCodeKDouble);
                startgcodewinkel_yz = Math.Atan2(-GCodeKDouble, -GCodeJDouble);
                double xhilfe_yz = GCodeYZeil - (GCodeYMomentan + GCodeJDouble);
                double zhilfe_yz = GCodeZZeil - (GCodeZMomentan + GCodeKDouble);
                stopgcodewinkel_yz = Math.Atan2(zhilfe_yz, xhilfe_yz);
                radius_xz = 0;
                radius = 0;
                double AnzahlAngenommenerInkrementSchritte = Math.Abs((startgcodewinkel_yz - stopgcodewinkel_yz) / (inkrement / radius_yz));
                if (GCodeXMomentan != GCodeXZeil)
                {
                    xbeiwert = ((GCodeXZeil - GCodeXMomentan) / inkrement) / AnzahlAngenommenerInkrementSchritte;
                }
                else
                {
                    xbeiwert = 0;
                }
                ybeiwert = 0;
                zbeiwert = 0;
                xy_KurveBeiwert = 1;
                xz_KurveBeiwert = 1;
                yz_KurveBeiwert = 1;
                doIKCurve2();
            }

            else if (I_On == false && K_On == false && J_On == false)
            {
                if (GCodeIDouble == 0)
                {
                    radius = Math.Sqrt(GCodeIDouble * GCodeIDouble + GCodeJDouble * GCodeJDouble);
                    startgcodewinkel = Math.Atan2(-GCodeJDouble, -GCodeIDouble);
                    double xhilfe = GCodeXZeil - (GCodeXMomentan + GCodeIDouble);
                    double yhilfe = GCodeYZeil - (GCodeYMomentan + GCodeJDouble);
                    stopgcodewinkel = Math.Atan2(yhilfe, xhilfe);
                    double AnzahlAngenommenerInkrementSchritte_xy = Math.Abs((startgcodewinkel - stopgcodewinkel) / (inkrement / radius));

                    radius_yz = Math.Sqrt(GCodeJDouble * GCodeJDouble + GCodeKDouble * GCodeKDouble);
                    startgcodewinkel_yz = Math.Atan2(-GCodeKDouble, -GCodeJDouble);
                    double xhilfe_yz = GCodeYZeil - (GCodeYMomentan + GCodeIDouble);
                    double zhilfe_yz = GCodeZZeil - (GCodeZMomentan + GCodeKDouble);
                    stopgcodewinkel_yz = Math.Atan2(zhilfe_yz, xhilfe_yz);
                    double AnzahlAngenommenerInkrementSchritte_yz = Math.Abs((startgcodewinkel_yz - stopgcodewinkel_yz) / (inkrement / radius_yz));
                    //inkrement_yz_for_IJK = Math.Abs(startgcodewinkel_yz - stopgcodewinkel_yz) / AnzahlAngenommenerInkrementSchritte_xy;
                    xy_KurveBeiwert = 1;
                    xz_KurveBeiwert = 1;
                    yz_KurveBeiwert = 1;

                    radius_xz = Math.Sqrt(GCodeIDouble * GCodeIDouble + GCodeKDouble * GCodeKDouble);
                    startgcodewinkel_xz = Math.Atan2(-GCodeKDouble, -GCodeIDouble);
                    double xhilfe_xz = GCodeXZeil - (GCodeXMomentan + GCodeIDouble);
                    double zhilfe = GCodeZZeil - (GCodeZMomentan + GCodeKDouble);
                    stopgcodewinkel_xz = Math.Atan2(zhilfe, xhilfe_xz);

                    doIKCurve3();
                }
                //radius_xz = Math.Sqrt(GCodeIDouble * GCodeIDouble + GCodeKDouble * GCodeKDouble);
                //startgcodewinkel_xz = Math.Atan2(-GCodeKDouble, -GCodeIDouble);
                //double xhilfe_xz = GCodeXZeil - (GCodeXMomentan + GCodeIDouble);
                //double zhilfe = GCodeZZeil - (GCodeZMomentan + GCodeKDouble);
                //stopgcodewinkel_xz = Math.Atan2(zhilfe, xhilfe_xz);
                //double AnzahlAngenommenerInkrementSchritte_xz = Math.Abs((startgcodewinkel_xz - stopgcodewinkel_xz) / (inkrement / radius_xz));

                //radius_yz = Math.Sqrt(GCodeJDouble * GCodeJDouble + GCodeKDouble * GCodeKDouble);
                //startgcodewinkel_yz = Math.Atan2(-GCodeKDouble, -GCodeJDouble);
                //double xhilfe_yz = GCodeYZeil - (GCodeYMomentan + GCodeIDouble);
                //double zhilfe_yz = GCodeZZeil - (GCodeZMomentan + GCodeKDouble);
                //stopgcodewinkel_yz = Math.Atan2(zhilfe_yz, xhilfe_yz);
                //double AnzahlAngenommenerInkrementSchritte_yz = Math.Abs((startgcodewinkel_yz - stopgcodewinkel_yz) / (inkrement / radius_yz));

                //radius = Math.Sqrt(GCodeIDouble * GCodeIDouble + GCodeJDouble * GCodeJDouble);
                //startgcodewinkel = Math.Atan2(-GCodeJDouble, -GCodeIDouble);
                //double xhilfe = GCodeXZeil - (GCodeXMomentan + GCodeIDouble);
                //double yhilfe = GCodeYZeil - (GCodeYMomentan + GCodeJDouble);
                //stopgcodewinkel = Math.Atan2(yhilfe, xhilfe);
                //double AnzahlAngenommenerInkrementSchritte_xy = Math.Abs((startgcodewinkel - stopgcodewinkel) / (inkrement / radius));
                /////////////////inkrement über beiwerte regeln
                //xy_KurveBeiwert = AnzahlAngenommenerInkrementSchritte_xy / (AnzahlAngenommenerInkrementSchritte_xy + AnzahlAngenommenerInkrementSchritte_yz + AnzahlAngenommenerInkrementSchritte_yz);
                //xz_KurveBeiwert = AnzahlAngenommenerInkrementSchritte_xz / (AnzahlAngenommenerInkrementSchritte_xy + AnzahlAngenommenerInkrementSchritte_yz + AnzahlAngenommenerInkrementSchritte_yz);
                //yz_KurveBeiwert = AnzahlAngenommenerInkrementSchritte_yz / (AnzahlAngenommenerInkrementSchritte_xy + AnzahlAngenommenerInkrementSchritte_yz + AnzahlAngenommenerInkrementSchritte_yz);
                //xbeiwert = 0;
                //ybeiwert = 0;
                //zbeiwert = 0;
                //doIKCurve2();
            }
        }
        public void doIK()
        {
            while (Math.Round(GCodeXMomentan, 3) != GCodeXZeil || Math.Round(GCodeYMomentan, 3) != GCodeYZeil || Math.Round(GCodeZMomentan, 3) != GCodeZZeil)
            {
                double x__ = GCodeXMomentan;
                double y__ = GCodeYMomentan;
                double z__ = GCodeZMomentan;
                double alpha__ = GCodeAMomentan;
                double beta__ = GCodeBMomentan;
                double gamma__ = GCodeCMomentan;
                x__ = x__ + inkrement * xbeiwert;
                y__ = y__ + inkrement * ybeiwert;
                z__ = z__ + inkrement * zbeiwert;
                IKAngle2 = InvKinematicForNC.InverseKinematicForNC(x__, y__, z__, alpha__, beta__, gamma__);
                //////////Speed berechnen
                deltaschritte = new int[6];
                deltaschritte[0] = (int)(IKAngle2[6] - aktuelleposition[0]);
                deltaschritte[1] = (int)(IKAngle2[7] - aktuelleposition[1]);
                deltaschritte[2] = (int)(IKAngle2[8] - aktuelleposition[2]);
                deltaschritte[3] = (int)(IKAngle2[9] - aktuelleposition[3]);
                deltaschritte[4] = (int)(IKAngle2[10] - aktuelleposition[4]);
                deltaschritte[5] = (int)(IKAngle2[11] - aktuelleposition[5]);
                for (int c = 0; c < 6; c++)
                {
                    if (deltaschritte[c] < 0)
                    {
                        deltaSchritte2[c] = Math.Abs(deltaschritte[c]);
                    }
                    else
                    {
                        deltaSchritte2[c] = deltaschritte[c];
                    }
                }
                //int deltaSchritte1_2 = Math.Abs(deltaschritte[0]);
                //int deltaSchritte2_2 = Math.Abs(deltaschritte[1]);
                //int deltaSchritte3_2 = Math.Abs(deltaschritte[2]);
                //int deltaSchritte4_2 = Math.Abs(deltaschritte[3]);
                //int deltaSchritte5_2 = Math.Abs(deltaschritte[4]);
                //int deltaSchritte6_2 = Math.Abs(deltaschritte[5]);
                double[] IKSpeed2 = InvKinematicForNC.NextSpeedForNC(deltaSchritte2[0], deltaSchritte2[1], deltaSchritte2[2], deltaSchritte2[3], deltaSchritte2[4], deltaSchritte2[5], Milisek);
                IKSPeedInt2 = new int[6];
                IKSPeedInt2[0] = (int)Math.Abs(Math.Round(IKSpeed2[0], 0));//macht aus den Float-Werten eine positive Ganzzahl-Int
                IKSPeedInt2[1] = (int)Math.Abs(Math.Round(IKSpeed2[1], 0));
                IKSPeedInt2[2] = (int)Math.Abs(Math.Round(IKSpeed2[2], 0));
                IKSPeedInt2[3] = (int)Math.Abs(Math.Round(IKSpeed2[3], 0));
                IKSPeedInt2[4] = (int)Math.Abs(Math.Round(IKSpeed2[4], 0));
                IKSPeedInt2[5] = (int)Math.Abs(Math.Round(IKSpeed2[5], 0));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[0], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[1], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[2], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[3], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[4], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[5], 0)));
                GCodeWinkel.Add(IKAngle2[0]);
                GCodeWinkel.Add(IKAngle2[1]);
                GCodeWinkel.Add(IKAngle2[2]);
                GCodeWinkel.Add(IKAngle2[3]);
                GCodeWinkel.Add(IKAngle2[4]);
                GCodeWinkel.Add(IKAngle2[5]);
                GCodeDeltaPos.Add(deltaschritte[0]);
                GCodeDeltaPos.Add(deltaschritte[1]);
                GCodeDeltaPos.Add(deltaschritte[2]);
                GCodeDeltaPos.Add(deltaschritte[3]);
                GCodeDeltaPos.Add(deltaschritte[4]);
                GCodeDeltaPos.Add(deltaschritte[5]);
                GCodeListe.Add((int)IKAngle2[6]);
                GCodeListe.Add((int)IKAngle2[7]);
                GCodeListe.Add((int)IKAngle2[8]);
                GCodeListe.Add((int)IKAngle2[9]);
                GCodeListe.Add((int)IKAngle2[10]);
                GCodeListe.Add((int)IKAngle2[11]);
                aktuelleposition[0] = (int)IKAngle2[6];
                aktuelleposition[1] = (int)IKAngle2[7];
                aktuelleposition[2] = (int)IKAngle2[8];
                aktuelleposition[3] = (int)IKAngle2[9];
                aktuelleposition[4] = (int)IKAngle2[10];
                aktuelleposition[5] = (int)IKAngle2[11];
                X.Add(x__);
                Y.Add(y__);
                Z.Add(z__);
                A.Add(alpha__);
                B.Add(beta__);
                C.Add(gamma__);
                DiagStr.Add("Gerade");
                DiagStrThreadNr.Add(ThreadNr);
                //Gcode    
                //alle die schon auf zielposition sind sollen nicht mehr erhöht werden. Darum die beiwerte auf 0 setzen
                if (Math.Round(GCodeXMomentan, 3) == GCodeXZeil)
                {
                    xbeiwert = 0;
                }
                if (Math.Round(GCodeYMomentan, 3) == GCodeYZeil)
                {
                    ybeiwert = 0;
                }
                if (Math.Round(GCodeZMomentan, 3) == GCodeZZeil)
                {
                    zbeiwert = 0;
                }
                GCodeXMomentan = GCodeXMomentan + inkrement * xbeiwert;//neue position in mm
                GCodeYMomentan = GCodeYMomentan + inkrement * ybeiwert;
                GCodeZMomentan = GCodeZMomentan + inkrement * zbeiwert;
                if (xbeiwert != 0)
                {
                    anzahl_erhoehungen_x = Math.Abs(GCodeDeltaX / (inkrement * xbeiwert));
                }
                else
                {
                    anzahl_erhoehungen_x = 0;
                }
                if (ybeiwert != 0)
                {
                    anzahl_erhoehungen_y = Math.Abs(GCodeDeltaY / (inkrement * ybeiwert));
                }
                else
                {
                    anzahl_erhoehungen_y = 0;
                }
                if (zbeiwert != 0)
                {
                    anzahl_erhoehungen_z = Math.Abs(GCodeDeltaZ / (inkrement * zbeiwert));
                }
                else
                {
                    anzahl_erhoehungen_z = 0;
                }
                if ((anzahl_erhoehungen_x + anzahl_erhoehungen_y + anzahl_erhoehungen_z) != 0)
                {
                    inkrement_eulerwinkel_alpha = Math.Abs(GCodeAMomentan - GCodeAZeil) / (anzahl_erhoehungen_x + anzahl_erhoehungen_z + anzahl_erhoehungen_y);//Dadurch habe ich die gleiche anzahl an erhöhungschritten in den eulerwinkel wie ich habe um das bogensegment im kartesischen raum zurückzulegen
                    inkrement_eulerwinkel_beta = Math.Abs(GCodeBMomentan - GCodeBZeil) / (anzahl_erhoehungen_x + anzahl_erhoehungen_z + anzahl_erhoehungen_y);
                    inkrement_eulerwinkel_gamma = Math.Abs(GCodeCMomentan - GCodeCZeil) / (anzahl_erhoehungen_x + anzahl_erhoehungen_z + anzahl_erhoehungen_y);
                }
                else
                {
                    inkrement_eulerwinkel_alpha = 0;
                    inkrement_eulerwinkel_beta = 0;
                    inkrement_eulerwinkel_gamma = 0;
                }
                if (GCodeAMomentan > GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan - inkrement_eulerwinkel_alpha;
                }
                else if (GCodeAMomentan < GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan + inkrement_eulerwinkel_alpha;
                }
                if (GCodeBMomentan > GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan - inkrement_eulerwinkel_beta;
                }
                else if (GCodeBMomentan < GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan + inkrement_eulerwinkel_beta;
                }
                if (GCodeCMomentan > GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan - inkrement_eulerwinkel_gamma;
                }
                else if (GCodeCMomentan < GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan + inkrement_eulerwinkel_gamma;
                }
            }
            GCodeXMomentan = GCodeXZeil;
            GCodeYMomentan = GCodeYZeil;
            GCodeZMomentan = GCodeZZeil;
            GCodeAMomentan = GCodeAZeil;
            GCodeBMomentan = GCodeBZeil;
            GCodeCMomentan = GCodeCZeil;
            counter1++;
            GCodeAblauf();

        }      
        public void MovePoint()
        {

        }
        public void doIKCurve2()
        {
            while (Math.Round(startgcodewinkel, 2) != stopgcodewinkel || Math.Round(startgcodewinkel_xz, 2) != stopgcodewinkel_xz || Math.Round(startgcodewinkel_yz, 2) != stopgcodewinkel_yz)
            {
                double x__ = GCodeXMomentan;
                double y__ = GCodeYMomentan;
                double z__ = GCodeZMomentan;
                double alpha__ = GCodeAMomentan;
                double beta__ = GCodeBMomentan;
                double gamma__ = GCodeCMomentan;
                double[] MittelPunkt = new double[3];
                MittelPunkt[0] = GCodeXMomentan + GCodeIDouble;
                MittelPunkt[1] = GCodeYMomentan + GCodeJDouble;
                MittelPunkt[2] = GCodeZMomentan + GCodeKDouble;
                x__ = MittelPunkt[0] + radius_xz * Math.Cos(startgcodewinkel_xz) + radius * Math.Cos(startgcodewinkel) + inkrement * xbeiwert;
                y__ = MittelPunkt[1] + radius_yz * Math.Cos(startgcodewinkel_yz) + radius * Math.Sin(startgcodewinkel) + inkrement * ybeiwert;
                z__ = MittelPunkt[2] + radius_xz * Math.Sin(startgcodewinkel_xz) + radius_yz * Math.Sin(startgcodewinkel_yz) + inkrement * zbeiwert;
                IKAngle2 = InvKinematicForNC.InverseKinematicForNC(x__, y__, z__, alpha__, beta__, gamma__);
                //////////Speed berechnen
                deltaschritte = new int[6];
                deltaschritte[0] = (int)(IKAngle2[6] - aktuelleposition[0]);
                deltaschritte[1] = (int)(IKAngle2[7] - aktuelleposition[1]);
                deltaschritte[2] = (int)(IKAngle2[8] - aktuelleposition[2]);
                deltaschritte[3] = (int)(IKAngle2[9] - aktuelleposition[3]);
                deltaschritte[4] = (int)(IKAngle2[10] - aktuelleposition[4]);
                deltaschritte[5] = (int)(IKAngle2[11] - aktuelleposition[5]);
                int deltaSchritte1_2 = Math.Abs(deltaschritte[0]);
                int deltaSchritte2_2 = Math.Abs(deltaschritte[1]);
                int deltaSchritte3_2 = Math.Abs(deltaschritte[2]);
                int deltaSchritte4_2 = Math.Abs(deltaschritte[3]);
                int deltaSchritte5_2 = Math.Abs(deltaschritte[4]);
                int deltaSchritte6_2 = Math.Abs(deltaschritte[5]);
                double[] IKSpeed2 = InvKinematicForNC.NextSpeedForNC(deltaSchritte1_2, deltaSchritte2_2, deltaSchritte3_2, deltaSchritte4_2, deltaSchritte5_2, deltaSchritte6_2, Milisek);
                IKSPeedInt2 = new int[6];
                IKSPeedInt2[0] = (int)Math.Abs(Math.Round(IKSpeed2[0], 0));//macht aus den Float-Werten eine positive Ganzzahl-Int
                IKSPeedInt2[1] = (int)Math.Abs(Math.Round(IKSpeed2[1], 0));
                IKSPeedInt2[2] = (int)Math.Abs(Math.Round(IKSpeed2[2], 0));
                IKSPeedInt2[3] = (int)Math.Abs(Math.Round(IKSpeed2[3], 0));
                IKSPeedInt2[4] = (int)Math.Abs(Math.Round(IKSpeed2[4], 0));
                IKSPeedInt2[5] = (int)Math.Abs(Math.Round(IKSpeed2[5], 0));

                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[0], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[1], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[2], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[3], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[4], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[5], 0)));
                GCodeDeltaPos.Add(deltaschritte[0]);
                GCodeDeltaPos.Add(deltaschritte[1]);
                GCodeDeltaPos.Add(deltaschritte[2]);
                GCodeDeltaPos.Add(deltaschritte[3]);
                GCodeDeltaPos.Add(deltaschritte[4]);
                GCodeDeltaPos.Add(deltaschritte[5]);
                GCodeListe.Add((int)IKAngle2[6]);
                GCodeListe.Add((int)IKAngle2[7]);
                GCodeListe.Add((int)IKAngle2[8]);
                GCodeListe.Add((int)IKAngle2[9]);
                GCodeListe.Add((int)IKAngle2[10]);
                GCodeListe.Add((int)IKAngle2[11]);
                GCodeWinkel.Add(IKAngle2[0]);
                GCodeWinkel.Add(IKAngle2[1]);
                GCodeWinkel.Add(IKAngle2[2]);
                GCodeWinkel.Add(IKAngle2[3]);
                GCodeWinkel.Add(IKAngle2[4]);
                GCodeWinkel.Add(IKAngle2[5]);
                X.Add(x__);
                Y.Add(y__);
                Z.Add(z__);
                A.Add(alpha__);
                B.Add(beta__);
                C.Add(gamma__);
                DiagStr.Add("Kurve");
                DiagStrThreadNr.Add(ThreadNr);

                aktuelleposition[0] = (int)IKAngle2[6];
                aktuelleposition[1] = (int)IKAngle2[7];
                aktuelleposition[2] = (int)IKAngle2[8];
                aktuelleposition[3] = (int)IKAngle2[9];
                aktuelleposition[4] = (int)IKAngle2[10];
                aktuelleposition[5] = (int)IKAngle2[11];
                //Gcode
                if (radius != 0)
                {
                    if (cw == true && startgcodewinkel > stopgcodewinkel)
                    {
                        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                        if (startgcodewinkel < stopgcodewinkel)
                        { break; }
                    }
                    else if (cw == true && startgcodewinkel < stopgcodewinkel)
                    {
                        if (startgcodewinkel <= -Math.PI)
                        {
                            startgcodewinkel = Math.PI;
                            startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel < stopgcodewinkel)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                            }
                        }
                        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                    }
                    else if (cw == false && startgcodewinkel < stopgcodewinkel)
                    {
                        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                        if (startgcodewinkel > stopgcodewinkel)
                        { break; }
                    }
                    else if (cw == false && startgcodewinkel > stopgcodewinkel)
                    {
                        if (startgcodewinkel >= Math.PI)
                        {
                            startgcodewinkel = -Math.PI;
                            startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel > stopgcodewinkel)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                            }
                        }
                        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                    }
                    anzahl_erhoehungen_xy = Math.Abs((startgcodewinkel - stopgcodewinkel)) / (inkrement / radius * xy_KurveBeiwert);
                    mitellwertbildner++;//sonst würde, falls aller radien !=0 sind die auflösung der euelerwinkel 3 mal zu hoch
                }
                else
                {
                    anzahl_erhoehungen_xy = 0;
                }
                if (radius_xz != 0)
                {
                    if (cw == true && startgcodewinkel_xz > stopgcodewinkel_xz)
                    {
                        startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                        if (startgcodewinkel_xz < stopgcodewinkel_xz)
                        { break; }
                    }
                    else if (cw == true && startgcodewinkel_xz < stopgcodewinkel_xz)
                    {
                        if (startgcodewinkel_xz <= -Math.PI)
                        {
                            startgcodewinkel_xz = Math.PI;
                            startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;
                            if (startgcodewinkel_xz < stopgcodewinkel_xz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;
                    }
                    else if (cw == false && startgcodewinkel_xz < stopgcodewinkel_xz)
                    {
                        startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                        if (startgcodewinkel_xz > stopgcodewinkel_xz)
                        { break; }
                    }
                    else if (cw == false && startgcodewinkel_xz > stopgcodewinkel_xz)
                    {
                        if (startgcodewinkel_xz >= Math.PI)
                        {
                            startgcodewinkel_xz = -Math.PI;
                            startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel_xz > stopgcodewinkel_xz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                    }
                    anzahl_erhoehungen_xz = Math.Abs((startgcodewinkel_xz - stopgcodewinkel_xz)) / (inkrement / radius_xz * xz_KurveBeiwert);
                    mitellwertbildner++;
                }
                else
                {
                    anzahl_erhoehungen_xz = 0;
                }
                if (radius_yz != 0)
                {
                    if (cw == true && startgcodewinkel_yz > stopgcodewinkel_yz)
                    {
                        startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                        if (startgcodewinkel_yz < stopgcodewinkel_yz)
                        { break; }
                    }
                    else if (cw == true && startgcodewinkel_yz < stopgcodewinkel_yz)
                    {
                        if (startgcodewinkel_yz <= -Math.PI)
                        {
                            startgcodewinkel_yz = Math.PI;
                            startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;
                            if (startgcodewinkel_yz < stopgcodewinkel_yz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;
                    }
                    else if (cw == false && startgcodewinkel_yz < stopgcodewinkel_yz)
                    {
                        startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                        if (startgcodewinkel_yz > stopgcodewinkel_yz)
                        { break; }
                    }
                    else if (cw == false && startgcodewinkel_yz > stopgcodewinkel_yz)
                    {
                        if (startgcodewinkel_yz >= Math.PI)
                        {
                            startgcodewinkel_yz = -Math.PI;
                            startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel_yz > stopgcodewinkel_yz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                    }
                    anzahl_erhoehungen_yz = Math.Abs((startgcodewinkel_yz - stopgcodewinkel_yz)) / (inkrement / radius_yz * yz_KurveBeiwert);
                    mitellwertbildner++;
                }
                else
                {
                    anzahl_erhoehungen_yz = 0;
                }
                inkrement_eulerwinkel_alpha = Math.Abs(GCodeAMomentan - GCodeAZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);//Dadurch habe ich die gleiche anzahl an erhöhungschritten in den eulerwinkel wie ich habe um das bogensegment im kartesischen raum zurückzulegen
                inkrement_eulerwinkel_beta = Math.Abs(GCodeBMomentan - GCodeBZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);
                inkrement_eulerwinkel_gamma = Math.Abs(GCodeCMomentan - GCodeCZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);
                mitellwertbildner = 0;
                if (GCodeAMomentan > GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan - inkrement_eulerwinkel_alpha;
                }
                else if (GCodeAMomentan < GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan + inkrement_eulerwinkel_alpha;
                }
                if (GCodeBMomentan > GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan - inkrement_eulerwinkel_beta;
                }
                else if (GCodeBMomentan < GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan + inkrement_eulerwinkel_beta;
                }
                if (GCodeCMomentan > GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan - inkrement_eulerwinkel_gamma;
                }
                else if (GCodeCMomentan < GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan + inkrement_eulerwinkel_gamma;
                }
                GCodeXMomentan = GCodeXMomentan + inkrement * xbeiwert;
                GCodeYMomentan = GCodeYMomentan + inkrement * ybeiwert;
                GCodeZMomentan = GCodeZMomentan + inkrement * zbeiwert;
            }
            GCodeXMomentan = GCodeXZeil;
            GCodeYMomentan = GCodeYZeil;
            GCodeZMomentan = GCodeZZeil;
            GCodeAMomentan = GCodeAZeil;
            GCodeBMomentan = GCodeBZeil;
            GCodeCMomentan = GCodeCZeil;
            counter1++;
            GCodeAblauf();
        }
        public void doIKCurve3()
        {
            while (Math.Round(startgcodewinkel, 2) != stopgcodewinkel || Math.Round(startgcodewinkel_xz, 2) != stopgcodewinkel_xz || Math.Round(startgcodewinkel_yz, 2) != stopgcodewinkel_yz)
            {
                double x__ = GCodeXMomentan;
                double y__ = GCodeYMomentan;
                double z__ = GCodeZMomentan;
                double alpha__ = GCodeAMomentan;
                double beta__ = GCodeBMomentan;
                double gamma__ = GCodeCMomentan;
                double[] MittelPunkt = new double[3];
                MittelPunkt[0] = GCodeXMomentan + GCodeIDouble;
                MittelPunkt[1] = GCodeYMomentan + GCodeJDouble;
                MittelPunkt[2] = GCodeZMomentan + GCodeKDouble;
                //x__ = MittelPunkt[0] + radius * Math.Cos(startgcodewinkel);
                //y__ = MittelPunkt[1] + radius * Math.Sin(startgcodewinkel);
                //z__ = MittelPunkt[2] + radius_yz * Math.Sin(startgcodewinkel_yz);
                x__ = MittelPunkt[0] + (radius_xz * Math.Cos(startgcodewinkel_xz) + radius * Math.Cos(startgcodewinkel)) /2;
                y__ = MittelPunkt[1] + (radius_yz * Math.Cos(startgcodewinkel_yz) + radius * Math.Sin(startgcodewinkel))/2;
                z__ = MittelPunkt[2] + (radius_xz * Math.Sin(startgcodewinkel_xz) + radius_yz * Math.Sin(startgcodewinkel_yz))/2;
                IKAngle2 = InvKinematicForNC.InverseKinematicForNC(x__, y__, z__, alpha__, beta__, gamma__);
                //////////Speed berechnen
                deltaschritte = new int[6];
                deltaschritte[0] = (int)(IKAngle2[6] - aktuelleposition[0]);
                deltaschritte[1] = (int)(IKAngle2[7] - aktuelleposition[1]);
                deltaschritte[2] = (int)(IKAngle2[8] - aktuelleposition[2]);
                deltaschritte[3] = (int)(IKAngle2[9] - aktuelleposition[3]);
                deltaschritte[4] = (int)(IKAngle2[10] - aktuelleposition[4]);
                deltaschritte[5] = (int)(IKAngle2[11] - aktuelleposition[5]);
                int deltaSchritte1_2 = Math.Abs(deltaschritte[0]);
                int deltaSchritte2_2 = Math.Abs(deltaschritte[1]);
                int deltaSchritte3_2 = Math.Abs(deltaschritte[2]);
                int deltaSchritte4_2 = Math.Abs(deltaschritte[3]);
                int deltaSchritte5_2 = Math.Abs(deltaschritte[4]);
                int deltaSchritte6_2 = Math.Abs(deltaschritte[5]);
                double[] IKSpeed2 = InvKinematicForNC.NextSpeedForNC(deltaSchritte1_2, deltaSchritte2_2, deltaSchritte3_2, deltaSchritte4_2, deltaSchritte5_2, deltaSchritte6_2, Milisek);
                IKSPeedInt2 = new int[6];
                IKSPeedInt2[0] = (int)Math.Abs(Math.Round(IKSpeed2[0], 0));//macht aus den Float-Werten eine positive Ganzzahl-Int
                IKSPeedInt2[1] = (int)Math.Abs(Math.Round(IKSpeed2[1], 0));
                IKSPeedInt2[2] = (int)Math.Abs(Math.Round(IKSpeed2[2], 0));
                IKSPeedInt2[3] = (int)Math.Abs(Math.Round(IKSpeed2[3], 0));
                IKSPeedInt2[4] = (int)Math.Abs(Math.Round(IKSpeed2[4], 0));
                IKSPeedInt2[5] = (int)Math.Abs(Math.Round(IKSpeed2[5], 0));

                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[0], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[1], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[2], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[3], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[4], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[5], 0)));
                GCodeDeltaPos.Add(deltaschritte[0]);
                GCodeDeltaPos.Add(deltaschritte[1]);
                GCodeDeltaPos.Add(deltaschritte[2]);
                GCodeDeltaPos.Add(deltaschritte[3]);
                GCodeDeltaPos.Add(deltaschritte[4]);
                GCodeDeltaPos.Add(deltaschritte[5]);
                GCodeListe.Add((int)IKAngle2[6]);
                GCodeListe.Add((int)IKAngle2[7]);
                GCodeListe.Add((int)IKAngle2[8]);
                GCodeListe.Add((int)IKAngle2[9]);
                GCodeListe.Add((int)IKAngle2[10]);
                GCodeListe.Add((int)IKAngle2[11]);
                GCodeWinkel.Add(IKAngle2[0]);
                GCodeWinkel.Add(IKAngle2[1]);
                GCodeWinkel.Add(IKAngle2[2]);
                GCodeWinkel.Add(IKAngle2[3]);
                GCodeWinkel.Add(IKAngle2[4]);
                GCodeWinkel.Add(IKAngle2[5]);
                X.Add(x__);
                Y.Add(y__);
                Z.Add(z__);
                A.Add(alpha__);
                B.Add(beta__);
                C.Add(gamma__);
                DiagStr.Add("Kurve");
                DiagStrThreadNr.Add(ThreadNr);

                aktuelleposition[0] = (int)IKAngle2[6];
                aktuelleposition[1] = (int)IKAngle2[7];
                aktuelleposition[2] = (int)IKAngle2[8];
                aktuelleposition[3] = (int)IKAngle2[9];
                aktuelleposition[4] = (int)IKAngle2[10];
                aktuelleposition[5] = (int)IKAngle2[11];
                //Gcode
                //if (radius != 0)
                //{
                //    if (cw == true && startgcodewinkel > stopgcodewinkel)
                //    {
                //        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                //        if (startgcodewinkel < stopgcodewinkel)
                //        { break; }
                //    }
                //    else if (cw == true && startgcodewinkel < stopgcodewinkel)
                //    {
                //        if (startgcodewinkel <= -Math.PI)
                //        {
                //            startgcodewinkel = Math.PI;
                //            startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                //            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                //            if (startgcodewinkel < stopgcodewinkel)
                //            { break; }
                //            else
                //            {
                //                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                //                startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                //            }
                //        }
                //        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                //    }
                //    else if (cw == false && startgcodewinkel < stopgcodewinkel)
                //    {
                //        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                //        if (startgcodewinkel > stopgcodewinkel)
                //        { break; }
                //    }
                //    else if (cw == false && startgcodewinkel > stopgcodewinkel)
                //    {
                //        if (startgcodewinkel >= Math.PI)
                //        {
                //            startgcodewinkel = -Math.PI;
                //            startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                //            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                //            if (startgcodewinkel > stopgcodewinkel)
                //            { break; }
                //            else
                //            {
                //                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                //                startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                //            }
                //        }
                //        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                //    }
                //    anzahl_erhoehungen_xy = Math.Abs((startgcodewinkel - stopgcodewinkel)) / (inkrement / radius * xy_KurveBeiwert);
                //    mitellwertbildner++;//sonst würde, falls aller radien !=0 sind die auflösung der euelerwinkel 3 mal zu hoch
                //}
                //else
                //{
                //    anzahl_erhoehungen_xy = 0;
                //}
                //if (radius_yz != 0)
                //{
                //    if (cw == true && startgcodewinkel_yz > stopgcodewinkel_yz)
                //    {
                //        startgcodewinkel_yz = startgcodewinkel_yz - inkrement_yz_for_IJK;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                //        if (startgcodewinkel_yz < stopgcodewinkel_yz)
                //        { break; }
                //    }
                //    else if (cw == true && startgcodewinkel_yz < stopgcodewinkel_yz)
                //    {
                //        if (startgcodewinkel_yz <= -Math.PI)
                //        {
                //            startgcodewinkel_yz = Math.PI;
                //            startgcodewinkel_yz = startgcodewinkel_yz - inkrement_yz_for_IJK;
                //            if (startgcodewinkel_yz < stopgcodewinkel_yz)
                //            { break; }
                //            else
                //            {
                //                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                //                startgcodewinkel_yz = startgcodewinkel_yz + inkrement_yz_for_IJK;
                //            }
                //        }
                //        startgcodewinkel_yz = startgcodewinkel_yz - inkrement_yz_for_IJK;
                //    }
                //    else if (cw == false && startgcodewinkel_yz < stopgcodewinkel_yz)
                //    {
                //        startgcodewinkel_yz = startgcodewinkel_yz + inkrement_yz_for_IJK;
                //        if (startgcodewinkel_yz > stopgcodewinkel_yz)
                //        { break; }
                //    }
                //    else if (cw == false && startgcodewinkel_yz > stopgcodewinkel_yz)
                //    {
                //        if (startgcodewinkel_yz >= Math.PI)
                //        {
                //            startgcodewinkel_yz = -Math.PI;
                //            startgcodewinkel_yz = startgcodewinkel_yz + inkrement_yz_for_IJK;
                //            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                //            if (startgcodewinkel_yz > stopgcodewinkel_yz)
                //            { break; }
                //            else
                //            {
                //                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                //                startgcodewinkel_yz = startgcodewinkel_yz - inkrement_yz_for_IJK;
                //            }
                //        }
                //        startgcodewinkel_yz = startgcodewinkel_yz + inkrement_yz_for_IJK;
                //    }
                //    anzahl_erhoehungen_yz = Math.Abs((startgcodewinkel_yz - stopgcodewinkel_yz)) / (inkrement_yz_for_IJK);
                //    mitellwertbildner++;
                //}
                //else
                //{
                //    anzahl_erhoehungen_yz = 0;
                //}
                //inkrement_eulerwinkel_alpha = Math.Abs(GCodeAMomentan - GCodeAZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_yz) / mitellwertbildner);//Dadurch habe ich die gleiche anzahl an erhöhungschritten in den eulerwinkel wie ich habe um das bogensegment im kartesischen raum zurückzulegen
                //inkrement_eulerwinkel_beta = Math.Abs(GCodeBMomentan - GCodeBZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_yz) / mitellwertbildner);
                //inkrement_eulerwinkel_gamma = Math.Abs(GCodeCMomentan - GCodeCZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_yz) / mitellwertbildner);
                //mitellwertbildner = 0;
                if (radius != 0)
                {
                    if (cw == true && startgcodewinkel > stopgcodewinkel)
                    {
                        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                        if (startgcodewinkel < stopgcodewinkel)
                        { break; }
                    }
                    else if (cw == true && startgcodewinkel < stopgcodewinkel)
                    {
                        if (startgcodewinkel <= -Math.PI)
                        {
                            startgcodewinkel = Math.PI;
                            startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel < stopgcodewinkel)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                            }
                        }
                        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                    }
                    else if (cw == false && startgcodewinkel < stopgcodewinkel)
                    {
                        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                        if (startgcodewinkel > stopgcodewinkel)
                        { break; }
                    }
                    else if (cw == false && startgcodewinkel > stopgcodewinkel)
                    {
                        if (startgcodewinkel >= Math.PI)
                        {
                            startgcodewinkel = -Math.PI;
                            startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel > stopgcodewinkel)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                            }
                        }
                        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                    }
                    anzahl_erhoehungen_xy = Math.Abs((startgcodewinkel - stopgcodewinkel)) / (inkrement / radius * xy_KurveBeiwert);
                    mitellwertbildner++;//sonst würde, falls aller radien !=0 sind die auflösung der euelerwinkel 3 mal zu hoch
                }
                else
                {
                    anzahl_erhoehungen_xy = 0;
                }
                if (radius_xz != 0)
                {
                    if (cw == true && startgcodewinkel_xz > stopgcodewinkel_xz)
                    {
                        startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                        if (startgcodewinkel_xz < stopgcodewinkel_xz)
                        { break; }
                    }
                    else if (cw == true && startgcodewinkel_xz < stopgcodewinkel_xz)
                    {
                        if (startgcodewinkel_xz <= -Math.PI)
                        {
                            startgcodewinkel_xz = Math.PI;
                            startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;
                            if (startgcodewinkel_xz < stopgcodewinkel_xz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;
                    }
                    else if (cw == false && startgcodewinkel_xz < stopgcodewinkel_xz)
                    {
                        startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                        if (startgcodewinkel_xz > stopgcodewinkel_xz)
                        { break; }
                    }
                    else if (cw == false && startgcodewinkel_xz > stopgcodewinkel_xz)
                    {
                        if (startgcodewinkel_xz >= Math.PI)
                        {
                            startgcodewinkel_xz = -Math.PI;
                            startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel_xz > stopgcodewinkel_xz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
                    }
                    anzahl_erhoehungen_xz = Math.Abs((startgcodewinkel_xz - stopgcodewinkel_xz)) / (inkrement / radius_xz * xz_KurveBeiwert);
                    mitellwertbildner++;
                }
                else
                {
                    anzahl_erhoehungen_xz = 0;
                }
                if (radius_yz != 0)
                {
                    if (cw == true && startgcodewinkel_yz > stopgcodewinkel_yz)
                    {
                        startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                        if (startgcodewinkel_yz < stopgcodewinkel_yz)
                        { break; }
                    }
                    else if (cw == true && startgcodewinkel_yz < stopgcodewinkel_yz)
                    {
                        if (startgcodewinkel_yz <= -Math.PI)
                        {
                            startgcodewinkel_yz = Math.PI;
                            startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;
                            if (startgcodewinkel_yz < stopgcodewinkel_yz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;
                    }
                    else if (cw == false && startgcodewinkel_yz < stopgcodewinkel_yz)
                    {
                        startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                        if (startgcodewinkel_yz > stopgcodewinkel_yz)
                        { break; }
                    }
                    else if (cw == false && startgcodewinkel_yz > stopgcodewinkel_yz)
                    {
                        if (startgcodewinkel_yz >= Math.PI)
                        {
                            startgcodewinkel_yz = -Math.PI;
                            startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel_yz > stopgcodewinkel_yz)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;
                            }
                        }
                        startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
                    }
                    anzahl_erhoehungen_yz = Math.Abs((startgcodewinkel_yz - stopgcodewinkel_yz)) / (inkrement / radius_yz * yz_KurveBeiwert);
                    mitellwertbildner++;
                }
                else
                {
                    anzahl_erhoehungen_yz = 0;
                }
                inkrement_eulerwinkel_alpha = Math.Abs(GCodeAMomentan - GCodeAZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);//Dadurch habe ich die gleiche anzahl an erhöhungschritten in den eulerwinkel wie ich habe um das bogensegment im kartesischen raum zurückzulegen
                inkrement_eulerwinkel_beta = Math.Abs(GCodeBMomentan - GCodeBZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);
                inkrement_eulerwinkel_gamma = Math.Abs(GCodeCMomentan - GCodeCZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);
                mitellwertbildner = 0;
                if (GCodeAMomentan > GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan - inkrement_eulerwinkel_alpha;
                }
                else if (GCodeAMomentan < GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan + inkrement_eulerwinkel_alpha;
                }
                if (GCodeBMomentan > GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan - inkrement_eulerwinkel_beta;
                }
                else if (GCodeBMomentan < GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan + inkrement_eulerwinkel_beta;
                }
                if (GCodeCMomentan > GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan - inkrement_eulerwinkel_gamma;
                }
                else if (GCodeCMomentan < GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan + inkrement_eulerwinkel_gamma;
                }

            }
            GCodeXMomentan = GCodeXZeil;
            GCodeYMomentan = GCodeYZeil;
            GCodeZMomentan = GCodeZZeil;
            GCodeAMomentan = GCodeAZeil;
            GCodeBMomentan = GCodeBZeil;
            GCodeCMomentan = GCodeCZeil;
            counter1++;
            GCodeAblauf();
        }
        public void doIKCurveForAll()
        {
            while (Math.Round(startgcodewinkel, 2) != stopgcodewinkel)
            {
                double x__ = GCodeXMomentan;
                double y__ = GCodeYMomentan;
                double z__ = GCodeZMomentan;
                double alpha__ = GCodeAMomentan;
                double beta__ = GCodeBMomentan;
                double gamma__ = GCodeCMomentan;
                double[] MittelPunkt = new double[3];
                MittelPunkt[0] = GCodeXMomentan + GCodeIDouble;
                MittelPunkt[1] = GCodeYMomentan + GCodeJDouble;
                MittelPunkt[2] = GCodeZMomentan + GCodeKDouble;
                if (nu == 0)
                {
                    x__ = MittelPunkt[0] + Math.Cos(phi) * radius_strich;
                    z__ = MittelPunkt[2] + Math.Sin(phi) * radius_strich;
                    y__ = MittelPunkt[1] + radius * Math.Sin(startgcodewinkel);
                    ////berechnungen für schiefe ebene
                    if (cw == true && alpha_start < -Math.PI)
                    {
                        alpha_start = -Math.PI;
                        alpha_start = -inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    else if (cw == true)
                    {
                        alpha_start = -inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    else if (cw == false && alpha_start > Math.PI)
                    {
                        alpha_start = -Math.PI;
                        alpha_start = inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    else if (cw == false)
                    {
                        alpha_start = inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    radius_strich = radius * Math.Cos(alpha_start);
                }
                else if (phi == 0)
                {
                    x__ = MittelPunkt[0] + radius * Math.Cos(startgcodewinkel);
                    z__ = MittelPunkt[2] + Math.Sin(nu) * radius_strich;
                    y__ = MittelPunkt[1] + Math.Cos(nu) * radius_strich;
                    ////berechnungen für schiefe ebene
                    if (cw == true && alpha_start < -Math.PI)
                    {
                        alpha_start = -Math.PI;
                        alpha_start = -inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    else if (cw == true)
                    {
                        alpha_start = -inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    else if (cw == false && alpha_start > Math.PI)
                    {
                        alpha_start = -Math.PI;
                        alpha_start = inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    else if (cw == false)
                    {
                        alpha_start = inkrement / radius * xy_KurveBeiwert + alpha_start;
                    }
                    radius_strich = radius * Math.Sin(alpha_start);
                }
                else if (phi != 0 && nu != 0)
                {
                    // noch durchdenken->brauch neuen radius_strich und dessen ganze berechnung
                }
                IKAngle2 = InvKinematicForNC.InverseKinematicForNC(x__, y__, z__, alpha__, beta__, gamma__);
                //////////Speed berechnen
                deltaschritte = new int[6];
                deltaschritte[0] = (int)(IKAngle2[6] - aktuelleposition[0]);
                deltaschritte[1] = (int)(IKAngle2[7] - aktuelleposition[1]);
                deltaschritte[2] = (int)(IKAngle2[8] - aktuelleposition[2]);
                deltaschritte[3] = (int)(IKAngle2[9] - aktuelleposition[3]);
                deltaschritte[4] = (int)(IKAngle2[10] - aktuelleposition[4]);
                deltaschritte[5] = (int)(IKAngle2[11] - aktuelleposition[5]);
                int deltaSchritte1_2 = Math.Abs(deltaschritte[0]);
                int deltaSchritte2_2 = Math.Abs(deltaschritte[1]);
                int deltaSchritte3_2 = Math.Abs(deltaschritte[2]);
                int deltaSchritte4_2 = Math.Abs(deltaschritte[3]);
                int deltaSchritte5_2 = Math.Abs(deltaschritte[4]);
                int deltaSchritte6_2 = Math.Abs(deltaschritte[5]);
                double[] IKSpeed2 = InvKinematicForNC.NextSpeedForNC(deltaSchritte1_2, deltaSchritte2_2, deltaSchritte3_2, deltaSchritte4_2, deltaSchritte5_2, deltaSchritte6_2, Milisek);
                IKSPeedInt2 = new int[6];
                IKSPeedInt2[0] = (int)Math.Abs(Math.Round(IKSpeed2[0], 0));//macht aus den Float-Werten eine positive Ganzzahl-Int
                IKSPeedInt2[1] = (int)Math.Abs(Math.Round(IKSpeed2[1], 0));
                IKSPeedInt2[2] = (int)Math.Abs(Math.Round(IKSpeed2[2], 0));
                IKSPeedInt2[3] = (int)Math.Abs(Math.Round(IKSpeed2[3], 0));
                IKSPeedInt2[4] = (int)Math.Abs(Math.Round(IKSpeed2[4], 0));
                IKSPeedInt2[5] = (int)Math.Abs(Math.Round(IKSpeed2[5], 0));

                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[0], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[1], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[2], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[3], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[4], 0)));
                GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[5], 0)));
                GCodeDeltaPos.Add(deltaschritte[0]);
                GCodeDeltaPos.Add(deltaschritte[1]);
                GCodeDeltaPos.Add(deltaschritte[2]);
                GCodeDeltaPos.Add(deltaschritte[3]);
                GCodeDeltaPos.Add(deltaschritte[4]);
                GCodeDeltaPos.Add(deltaschritte[5]);
                GCodeListe.Add((int)IKAngle2[6]);
                GCodeListe.Add((int)IKAngle2[7]);
                GCodeListe.Add((int)IKAngle2[8]);
                GCodeListe.Add((int)IKAngle2[9]);
                GCodeListe.Add((int)IKAngle2[10]);
                GCodeListe.Add((int)IKAngle2[11]);
                GCodeWinkel.Add(IKAngle2[0]);
                GCodeWinkel.Add(IKAngle2[1]);
                GCodeWinkel.Add(IKAngle2[2]);
                GCodeWinkel.Add(IKAngle2[3]);
                GCodeWinkel.Add(IKAngle2[4]);
                GCodeWinkel.Add(IKAngle2[5]);
                X.Add(x__);
                Y.Add(y__);
                Z.Add(z__);
                A.Add(alpha__);
                B.Add(beta__);
                C.Add(gamma__);
                DiagStr.Add("Kurve_E");
                DiagStrThreadNr.Add("Thread1_E");

                aktuelleposition[0] = (int)IKAngle2[6];
                aktuelleposition[1] = (int)IKAngle2[7];
                aktuelleposition[2] = (int)IKAngle2[8];
                aktuelleposition[3] = (int)IKAngle2[9];
                aktuelleposition[4] = (int)IKAngle2[10];
                aktuelleposition[5] = (int)IKAngle2[11];
                //Gcode
                if (radius != 0)
                {
                    if (cw == true && startgcodewinkel > stopgcodewinkel)
                    {
                        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
                        if (startgcodewinkel < stopgcodewinkel)
                        { break; }
                    }
                    else if (cw == true && startgcodewinkel < stopgcodewinkel)
                    {
                        if (startgcodewinkel <= -Math.PI)
                        {
                            startgcodewinkel = Math.PI;
                            startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel < stopgcodewinkel)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                            }
                        }
                        startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                    }
                    else if (cw == false && startgcodewinkel < stopgcodewinkel)
                    {
                        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                        if (startgcodewinkel > stopgcodewinkel)
                        { break; }
                    }
                    else if (cw == false && startgcodewinkel > stopgcodewinkel)
                    {
                        if (startgcodewinkel >= Math.PI)
                        {
                            startgcodewinkel = -Math.PI;
                            startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                            //cheken, ob der starcodewinkel nach nur einer erhöhung schon kleiner ist als der stopwinkel...wenn ja->break
                            if (startgcodewinkel > stopgcodewinkel)
                            { break; }
                            else
                            {
                                //fals das abbruchkriterium nicht zutrifft sollte der startgcodewinkel in dern zustand vor der if-schlaufe versetzt werden.
                                startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
                            }
                        }
                        startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
                    }
                    anzahl_erhoehungen_xy = Math.Abs((startgcodewinkel - stopgcodewinkel)) / (inkrement / radius * xy_KurveBeiwert);
                }
                else
                {
                    anzahl_erhoehungen_xy = 0;
                }
                
                inkrement_eulerwinkel_alpha = Math.Abs(GCodeAMomentan - GCodeAZeil) / (anzahl_erhoehungen_xy);//Dadurch habe ich die gleiche anzahl an erhöhungschritten in den eulerwinkel wie ich habe um das bogensegment im kartesischen raum zurückzulegen
                inkrement_eulerwinkel_beta = Math.Abs(GCodeBMomentan - GCodeBZeil) / (anzahl_erhoehungen_xy);
                inkrement_eulerwinkel_gamma = Math.Abs(GCodeCMomentan - GCodeCZeil) / (anzahl_erhoehungen_xy);
                if (GCodeAMomentan > GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan - inkrement_eulerwinkel_alpha;
                }
                else if (GCodeAMomentan < GCodeAZeil)
                {
                    GCodeAMomentan = GCodeAMomentan + inkrement_eulerwinkel_alpha;
                }
                if (GCodeBMomentan > GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan - inkrement_eulerwinkel_beta;
                }
                else if (GCodeBMomentan < GCodeBZeil)
                {
                    GCodeBMomentan = GCodeBMomentan + inkrement_eulerwinkel_beta;
                }
                if (GCodeCMomentan > GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan - inkrement_eulerwinkel_gamma;
                }
                else if (GCodeCMomentan < GCodeCZeil)
                {
                    GCodeCMomentan = GCodeCMomentan + inkrement_eulerwinkel_gamma;
                }               
            }
            GCodeXMomentan = GCodeXZeil;
            GCodeYMomentan = GCodeYZeil;
            GCodeZMomentan = GCodeZZeil;
            GCodeAMomentan = GCodeAZeil;
            GCodeBMomentan = GCodeBZeil;
            GCodeCMomentan = GCodeCZeil;
            counter1++;
            GCodeAblauf();
            //while (Math.Round(startgcodewinkel, 2) != stopgcodewinkel || Math.Round(startgcodewinkel_xz, 2) != stopgcodewinkel_xz || Math.Round(startgcodewinkel_yz, 2) != stopgcodewinkel_yz)
            //{
            //    double x__ = GCodeXMomentan;
            //    double y__ = GCodeYMomentan;
            //    double z__ = GCodeZMomentan;
            //    double alpha__ = GCodeAMomentan;
            //    double beta__ = GCodeBMomentan;
            //    double gamma__ = GCodeCMomentan;
            //    double[] MittelPunkt = new double[3];
            //    MittelPunkt[0] = GCodeXMomentan + GCodeIDouble;
            //    MittelPunkt[1] = GCodeYMomentan + GCodeJDouble;
            //    MittelPunkt[2] = GCodeZMomentan + GCodeKDouble;
            //    x__ = MittelPunkt[0] + radius_xz * Math.Sin(startgcodewinkel_xz) + radius * Math.Cos(startgcodewinkel) + inkrement * xbeiwert;
            //    y__ = MittelPunkt[1] + radius_yz * Math.Cos(startgcodewinkel_yz) + radius * Math.Sin(startgcodewinkel) + inkrement * ybeiwert;
            //    z__ = MittelPunkt[2] + radius_xz * Math.Cos(startgcodewinkel_xz) + radius_yz * Math.Sin(startgcodewinkel_yz) + inkrement * zbeiwert;
            //    IKAngle2 = WK.Angle1(x__, y__, z__, alpha__, beta__, gamma__, a2, d4, d6);
            //    //////////Speed berechnen
            //    deltaschritte = new int[6];
            //    deltaschritte[0] = (int)(IKAngle2[7] - aktuelleposition[0]);
            //    deltaschritte[1] = (int)(IKAngle2[8] - aktuelleposition[1]);
            //    deltaschritte[2] = (int)(IKAngle2[10] - aktuelleposition[2]);
            //    deltaschritte[3] = (int)(IKAngle2[6] - aktuelleposition[3]);
            //    deltaschritte[4] = (int)(IKAngle2[9] - aktuelleposition[4]);
            //    deltaschritte[5] = (int)(IKAngle2[11] - aktuelleposition[5]);
            //    int deltaSchritte1_2 = Math.Abs(deltaschritte[0]);
            //    int deltaSchritte2_2 = Math.Abs(deltaschritte[1]);
            //    int deltaSchritte3_2 = Math.Abs(deltaschritte[2]);
            //    int deltaSchritte4_2 = Math.Abs(deltaschritte[3]);
            //    int deltaSchritte5_2 = Math.Abs(deltaschritte[4]);
            //    int deltaSchritte6_2 = Math.Abs(deltaschritte[5]);
            //    double[] IKSpeed2 = WK.NextSpeedGCode(deltaSchritte1_2, deltaSchritte2_2, deltaSchritte3_2, deltaSchritte4_2, deltaSchritte5_2, deltaSchritte6_2, Milisek);
            //    IKSPeedInt2 = new int[6];
            //    IKSPeedInt2[0] = (int)Math.Abs(Math.Round(IKSpeed2[0], 0));//macht aus den Float-Werten eine positive Ganzzahl-Int
            //    IKSPeedInt2[1] = (int)Math.Abs(Math.Round(IKSpeed2[1], 0));
            //    IKSPeedInt2[2] = (int)Math.Abs(Math.Round(IKSpeed2[2], 0));
            //    IKSPeedInt2[3] = (int)Math.Abs(Math.Round(IKSpeed2[3], 0));
            //    IKSPeedInt2[4] = (int)Math.Abs(Math.Round(IKSpeed2[4], 0));
            //    IKSPeedInt2[5] = (int)Math.Abs(Math.Round(IKSpeed2[5], 0));

            //    GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[0], 0)));
            //    GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[1], 0)));
            //    GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[2], 0)));
            //    GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[3], 0)));
            //    GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[4], 0)));
            //    GCodeListeSpeed.Add((int)Math.Abs(Math.Round(IKSpeed2[5], 0)));
            //    GCodeDeltaPos.Add(deltaschritte[0]);
            //    GCodeDeltaPos.Add(deltaschritte[1]);
            //    GCodeDeltaPos.Add(deltaschritte[2]);
            //    GCodeDeltaPos.Add(deltaschritte[3]);
            //    GCodeDeltaPos.Add(deltaschritte[4]);
            //    GCodeDeltaPos.Add(deltaschritte[5]);
            //    GCodeListe.Add((int)IKAngle2[7]);
            //    GCodeListe.Add((int)IKAngle2[8]);
            //    GCodeListe.Add((int)IKAngle2[10]);
            //    GCodeListe.Add((int)IKAngle2[6]);
            //    GCodeListe.Add((int)IKAngle2[9]);
            //    GCodeListe.Add((int)IKAngle2[11]);
            //    GCodeWinkel.Add(IKAngle2[1]);
            //    GCodeWinkel.Add(IKAngle2[2]);
            //    GCodeWinkel.Add(IKAngle2[4]);
            //    GCodeWinkel.Add(IKAngle2[0]);
            //    GCodeWinkel.Add(IKAngle2[3]);
            //    GCodeWinkel.Add(IKAngle2[5]);
            //    X.Add(x__);
            //    Y.Add(y__);
            //    Z.Add(z__);
            //    A.Add(alpha__);
            //    B.Add(beta__);
            //    C.Add(gamma__);
            //    DiagStr.Add("Kurve");

            //    aktuelleposition[0] = (int)IKAngle2[7];
            //    aktuelleposition[1] = (int)IKAngle2[8];
            //    aktuelleposition[2] = (int)IKAngle2[10];
            //    aktuelleposition[3] = (int)IKAngle2[6];
            //    aktuelleposition[4] = (int)IKAngle2[9];
            //    aktuelleposition[5] = (int)IKAngle2[11];
            //    //Gcode
            //    if (radius != 0)
            //    {
            //        if (cw == true && startgcodewinkel > stopgcodewinkel)
            //        {
            //            startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
            //        }
            //        else if (cw == true && startgcodewinkel < stopgcodewinkel)
            //        {
            //            if (startgcodewinkel <= -Math.PI)
            //            {
            //                startgcodewinkel = Math.PI;
            //            }
            //            startgcodewinkel = startgcodewinkel - inkrement / radius * xy_KurveBeiwert;
            //        }
            //        else if (cw == false && startgcodewinkel < stopgcodewinkel)
            //        {
            //            startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
            //        }
            //        else if (cw == false && startgcodewinkel > stopgcodewinkel)
            //        {
            //            if (startgcodewinkel >= Math.PI)
            //            {
            //                startgcodewinkel = -Math.PI;
            //            }
            //            startgcodewinkel = startgcodewinkel + inkrement / radius * xy_KurveBeiwert;
            //        }
            //        anzahl_erhoehungen_xy = Math.Abs((startgcodewinkel - stopgcodewinkel)) / (inkrement / radius * xy_KurveBeiwert);
            //        mitellwertbildner++;//sonst würde, falls aller radien !=0 sind die auflösung der euelerwinkel 3 mal zu hoch
            //    }
            //    else
            //    {
            //        anzahl_erhoehungen_xy = 0;
            //    }
            //    if (radius_xz != 0)
            //    {
            //        if (cw == true && startgcodewinkel_xz > stopgcodewinkel_xz)
            //        {
            //            startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
            //        }
            //        else if (cw == true && startgcodewinkel_xz < stopgcodewinkel_xz)
            //        {
            //            if (startgcodewinkel_xz <= -Math.PI)
            //            {
            //                startgcodewinkel_xz = Math.PI;
            //            }
            //            startgcodewinkel_xz = startgcodewinkel_xz - inkrement / radius_xz * xz_KurveBeiwert;
            //        }
            //        else if (cw == false && startgcodewinkel_xz < stopgcodewinkel_xz)
            //        {
            //            startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
            //        }
            //        else if (cw == false && startgcodewinkel_xz > stopgcodewinkel_xz)
            //        {
            //            if (startgcodewinkel_xz >= Math.PI)
            //            {
            //                startgcodewinkel_xz = -Math.PI;
            //            }
            //            startgcodewinkel_xz = startgcodewinkel_xz + inkrement / radius_xz * xz_KurveBeiwert;
            //        }
            //        anzahl_erhoehungen_xz = Math.Abs((startgcodewinkel_xz - stopgcodewinkel_xz)) / (inkrement / radius_xz * xz_KurveBeiwert);
            //        mitellwertbildner++;
            //    }
            //    else
            //    {
            //        anzahl_erhoehungen_xz = 0;
            //    }
            //    if (radius_yz != 0)
            //    {
            //        if (cw == true && startgcodewinkel_yz > stopgcodewinkel_yz)
            //        {
            //            startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;//inkrement/radius = gegenkathete/ankathete = tan(deltawinkel)-->gibt also den deltawinkel, während dem eine bogenlänge der länge inkrement zurückgelegt wird                   
            //        }
            //        else if (cw == true && startgcodewinkel_yz < stopgcodewinkel_yz)
            //        {
            //            if (startgcodewinkel_yz <= -Math.PI)
            //            {
            //                startgcodewinkel_yz = Math.PI;
            //            }
            //            startgcodewinkel_yz = startgcodewinkel_yz - inkrement / radius_yz * yz_KurveBeiwert;
            //        }
            //        else if (cw == false && startgcodewinkel_yz < stopgcodewinkel_yz)
            //        {
            //            startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
            //        }
            //        else if (cw == false && startgcodewinkel_yz > stopgcodewinkel_yz)
            //        {
            //            if (startgcodewinkel_yz >= Math.PI)
            //            {
            //                startgcodewinkel_yz = -Math.PI;
            //            }
            //            startgcodewinkel_yz = startgcodewinkel_yz + inkrement / radius_yz * yz_KurveBeiwert;
            //        }
            //        anzahl_erhoehungen_yz = Math.Abs((startgcodewinkel_yz - stopgcodewinkel_yz)) / (inkrement / radius_yz * yz_KurveBeiwert);
            //        mitellwertbildner++;
            //    }
            //    else
            //    {
            //        anzahl_erhoehungen_yz = 0;
            //    }
            //    inkrement_eulerwinkel_alpha = Math.Abs(GCodeBMomentan - GCodeBZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);//Dadurch habe ich die gleiche anzahl an erhöhungschritten in den eulerwinkel wie ich habe um das bogensegment im kartesischen raum zurückzulegen
            //    inkrement_eulerwinkel_beta = Math.Abs(GCodeBMomentan - GCodeBZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);
            //    inkrement_eulerwinkel_gamma = Math.Abs(GCodeBMomentan - GCodeBZeil) / ((anzahl_erhoehungen_xy + anzahl_erhoehungen_xz + anzahl_erhoehungen_yz) / mitellwertbildner);
            //    mitellwertbildner = 0;
            //    if (GCodeAMomentan > GCodeAZeil)
            //    {
            //        GCodeAMomentan = GCodeAMomentan - inkrement_eulerwinkel_alpha;
            //    }
            //    else if (GCodeAMomentan < GCodeAZeil)
            //    {
            //        GCodeAMomentan = GCodeAMomentan + inkrement_eulerwinkel_alpha;
            //    }
            //    if (GCodeBMomentan > GCodeBZeil)
            //    {
            //        GCodeBMomentan = GCodeBMomentan - inkrement_eulerwinkel_beta;
            //    }
            //    else if (GCodeBMomentan < GCodeBZeil)
            //    {
            //        GCodeBMomentan = GCodeBMomentan + inkrement_eulerwinkel_beta;
            //    }
            //    if (GCodeCMomentan > GCodeCZeil)
            //    {
            //        GCodeCMomentan = GCodeCMomentan - inkrement_eulerwinkel_gamma;
            //    }
            //    else if (GCodeCMomentan < GCodeCZeil)
            //    {
            //        GCodeCMomentan = GCodeCMomentan + inkrement_eulerwinkel_gamma;
            //    }
            //    GCodeXMomentan = GCodeXMomentan + inkrement * xbeiwert;
            //    GCodeYMomentan = GCodeYMomentan + inkrement * ybeiwert;
            //    GCodeZMomentan = GCodeZMomentan + inkrement * zbeiwert;
            //}
            //GCodeXMomentan = GCodeXZeil;
            //GCodeYMomentan = GCodeYZeil;
            //GCodeZMomentan = GCodeZZeil;
            //GCodeAMomentan = GCodeAZeil;
            //GCodeBMomentan = GCodeBZeil;
            //GCodeCMomentan = GCodeCZeil;
            //counter1++;
            //GCodeAblauf();
        }
        public ArrayList GCodePos
        {
            get { return GCodeListe; }
            set { GCodeListe = value; }
        }
        public ArrayList GCodeSpeed
        {
            get { return GCodeListeSpeed; }
            set { GCodeListeSpeed = value; }
        }
        public ArrayList GCodeDelta
        {
            get { return GCodeDeltaPos; }
            set { GCodeDeltaPos = value; }
        }
        public ArrayList GCodeW
        {
            get { return GCodeWinkel; }
            set { GCodeWinkel = value; }
        }
        public ArrayList GX
        {
            get { return X; }
            set { X = value; }
        }
        public ArrayList GY
        {
            get { return Y; }
            set { Y = value; }
        }
        public ArrayList GZ
        {
            get { return Z; }
            set { Z = value; }
        }
        public ArrayList GA
        {
            get { return A; }
            set { A = value; }
        }
        public ArrayList GB
        {
            get { return B; }
            set { B = value; }
        }
        public ArrayList GC
        {
            get { return C; }
            set { C = value; }
        }
        public ArrayList GStr
        {
            get { return DiagStr; }
            set { DiagStr = value; }
        }
        public ArrayList GStrThreadNr
        {
            get { return DiagStrThreadNr; }
            set { DiagStrThreadNr = value; }
        }  
    }
}

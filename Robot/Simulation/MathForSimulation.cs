using System;
using System.Collections.Generic;
using System.Drawing;// add a reference to System.Drawing.dll
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;//add a reference to PresentationFramework.dll 
using System.Windows.Media;//add a reference to PresentationCore.dll (for imageprocessing to)
using System.Windows.Media.Media3D;
using System.Drawing;

namespace Simulation
{
    public static class MathForSimulation
    {
        public static GeometryModel3D DrawCubus(Point3D[] pointarray, Material material)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();
            for (int i = 0; i < pointarray.Length; i++)
            {
                mesh.Positions.Add(pointarray[i]);
            }
            #region Grundfläche in XY1
            //ist sichtbar mit  Cam.Position = new Point3D(0, 0, -100);  Cam.LookDirection = new Vector3D(0, 0, 1); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, 0, 1)));
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);

            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(1);
            #endregion
            #region Dachfläche in XY2
            //ist sichtbar mit  Cam.Position = new Point3D(0, 0, 100);  Cam.LookDirection = new Vector3D(0, 0, -1); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, 0, -1)));
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(4);

            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(5);
            #endregion
            #region Seitenfläch in YZ1
            //ist sichtbar mit  Cam.Position = new Point3D(-100, 0, 0);  Cam.LookDirection = new Vector3D(1, 0, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(1, 0, 0)));
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(0);

            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(5);
            #endregion
            #region Seitenfläch in YZ2
            //ist sichtbar mit  Cam.Position = new Point3D(100, 0, 0);  Cam.LookDirection = new Vector3D(-1, 0, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(-1, 0, 0)));
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(2);

            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(7);
            #endregion
            #region Seitenfläch in XZ1
            //ist sichtbar mit  Cam.Position = new Point3D(0, -100, 0);  Cam.LookDirection = new Vector3D(0, 1, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, 1, 0)));
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(0);

            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(6);
            #endregion
            #region Seitenfläch in XZ2
            //ist sichtbar mit  Cam.Position = new Point3D(0, 100, 0);  Cam.LookDirection = new Vector3D(0, -1, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, -1, 0)));
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(1);

            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);
            #endregion


            return new GeometryModel3D(mesh, material); ;
        }

        public static GeometryModel3D DrawCubusWithTexture(Point3D[] pointarray, Material material, Point text1, Point text2, Point text3, Point text4)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();
            for (int i = 0; i < pointarray.Length; i++)
            {
                mesh.Positions.Add(pointarray[i]);
            }
            #region Grundfläche in XY1
            //ist sichtbar mit  Cam.Position = new Point3D(0, 0, -100);  Cam.LookDirection = new Vector3D(0, 0, 1); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, 0, 1)));
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);

            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(1);
            #endregion
            #region Dachfläche in XY2
            //ist sichtbar mit  Cam.Position = new Point3D(0, 0, 100);  Cam.LookDirection = new Vector3D(0, 0, -1); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, 0, -1)));
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(4);

            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(5);
            #endregion
            #region Seitenfläch in YZ1
            //ist sichtbar mit  Cam.Position = new Point3D(-100, 0, 0);  Cam.LookDirection = new Vector3D(1, 0, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(1, 0, 0)));
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(0);

            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(5);
            #endregion
            #region Seitenfläch in YZ2
            //ist sichtbar mit  Cam.Position = new Point3D(100, 0, 0);  Cam.LookDirection = new Vector3D(-1, 0, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(-1, 0, 0)));
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(2);

            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(7);
            #endregion
            #region Seitenfläch in XZ1
            //ist sichtbar mit  Cam.Position = new Point3D(0, -100, 0);  Cam.LookDirection = new Vector3D(0, 1, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, 1, 0)));
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(0);

            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(6);
            #endregion
            #region Seitenfläch in XZ2
            //ist sichtbar mit  Cam.Position = new Point3D(0, 100, 0);  Cam.LookDirection = new Vector3D(0, -1, 0); Geometry3DContainer.Children.Add(new DirectionalLight(Colors.White, new Vector3D(0, -1, 0)));
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(1);

            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);
            #endregion
            //mesh.TextureCoordinates.Add(text1);
            //mesh.TextureCoordinates.Add(text2);
            //mesh.TextureCoordinates.Add(text3);
            //mesh.TextureCoordinates.Add(text4);

            return new GeometryModel3D(mesh, material); ;
        }

        public static Vector3D CrossP(Vector3D v1, Vector3D v2)
        {
            Vector3D returnVector3D = new Vector3D(0, 0, 0);
            returnVector3D.X = v1.Y * v2.Z - v1.Z * v2.Y;
            returnVector3D.Y = v1.Z * v2.X - v1.X * v2.Z;
            returnVector3D.Z = v1.X * v2.Y - v1.Y * v2.X;
            return returnVector3D;
        }

        public static GeometryModel3D Make3DGeometry_Over3Points(Point3D p1_, Point3D p2_, double thickness)
        {
            Point3D[] PointsToMesch = new Point3D[8];
            MeshGeometry3D m = new MeshGeometry3D();
            Vector3D v12 = new Vector3D(p2_.X - p1_.X, p2_.Y - p1_.Y, p2_.Z - p1_.Z);//vektro senkrecht auf die ebene in der der punkt Q liegen wird. es ist die Achse der Rotation
            Point3D Q1 = GetQ(v12, p1_, thickness);
            AxisAngleRotation3D rotation_90_1 = new AxisAngleRotation3D(v12, -90);
            RotateTransform3D rot3D_90_1 = new RotateTransform3D(rotation_90_1, p1_);
            Point3D resultatQ1_90 = new Point3D();
            resultatQ1_90 = rot3D_90_1.Transform(Q1);
            AxisAngleRotation3D rotation_180_1 = new AxisAngleRotation3D(v12, -180);
            RotateTransform3D rot3D_180_1 = new RotateTransform3D(rotation_180_1, p1_);
            Point3D resultatQ1_180 = new Point3D();
            resultatQ1_180 = rot3D_180_1.Transform(Q1);
            AxisAngleRotation3D rotation_270_1 = new AxisAngleRotation3D(v12, -270);
            RotateTransform3D rot3D_270_1 = new RotateTransform3D(rotation_270_1, p1_);
            Point3D resultatQ1_270 = new Point3D();
            resultatQ1_270 = rot3D_270_1.Transform(Q1);

            Point3D Q2 = GetQ(v12, p2_, thickness);
            AxisAngleRotation3D rotation_90_2 = new AxisAngleRotation3D(v12, -90);
            RotateTransform3D rot3D_90_2 = new RotateTransform3D(rotation_90_2, p2_);
            Point3D resultatQ2_90 = new Point3D();
            resultatQ2_90 = rot3D_90_2.Transform(Q2);
            AxisAngleRotation3D rotation_180_2 = new AxisAngleRotation3D(v12, -180);
            RotateTransform3D rot3D_180_2 = new RotateTransform3D(rotation_180_2, p2_);
            Point3D resultatQ2_180 = new Point3D();
            resultatQ2_180 = rot3D_180_2.Transform(Q2);
            AxisAngleRotation3D rotation_270_2 = new AxisAngleRotation3D(v12, -270);
            RotateTransform3D rot3D_270_2 = new RotateTransform3D(rotation_270_2, p2_);
            Point3D resultatQ2_270 = new Point3D();
            resultatQ2_270 = rot3D_270_2.Transform(Q2);
            PointsToMesch[0] = Q1;
            PointsToMesch[1] = resultatQ1_90;
            PointsToMesch[2] = resultatQ1_270;
            PointsToMesch[3] = resultatQ1_180;
            PointsToMesch[4] = Q2;
            PointsToMesch[5] = resultatQ2_90;
            PointsToMesch[6] = resultatQ2_270;
            PointsToMesch[7] = resultatQ2_180;
            DiffuseMaterial DiffuseMaterial = new DiffuseMaterial(System.Windows.Media.Brushes.White);
            MaterialGroup MaterialGroup = new MaterialGroup();
            MaterialGroup.Children.Add(DiffuseMaterial);
            System.Windows.Media.Color c = new System.Windows.Media.Color();
            c.ScA = 255;
            c.ScB = 255;
            c.ScR = 255;
            c.ScG = 255;
            EmissiveMaterial EmissiveMaterial = new EmissiveMaterial(new SolidColorBrush(c));
            MaterialGroup.Children.Add(EmissiveMaterial);
            return DrawCubus(PointsToMesch, MaterialGroup);
        }
        //für statische simulation
        public static GeometryModel3D Make3DGeometry(Point3D p1, Point3D p2, double thickness)
        {
            MeshGeometry3D m = new MeshGeometry3D();
            Point3D mittelpunkt = Mittelpunkt(p1, p2);
            Vector3D n = new Vector3D(p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z);//nicht normiert
            Point3D Q = GetQ(n, mittelpunkt, thickness);
            AxisAngleRotation3D rotation_90 = new AxisAngleRotation3D(n, 90);
            RotateTransform3D rot3D_90 = new RotateTransform3D();
            rot3D_90.Rotation = rotation_90;
            rot3D_90.CenterX = mittelpunkt.X;
            rot3D_90.CenterY = mittelpunkt.Y;
            rot3D_90.CenterZ = mittelpunkt.Z;
            Point3D resultatQ_90 = new Point3D();
            resultatQ_90 = rot3D_90.Transform(Q);
            AxisAngleRotation3D rotation_180 = new AxisAngleRotation3D(n, 180);
            RotateTransform3D rot3D_180 = new RotateTransform3D();
            rot3D_180.Rotation = rotation_180;
            rot3D_180.CenterX = mittelpunkt.X;
            rot3D_180.CenterY = mittelpunkt.Y;
            rot3D_180.CenterZ = mittelpunkt.Z;
            Point3D resultatQ_180 = new Point3D();
            resultatQ_180 = rot3D_180.Transform(Q);
            AxisAngleRotation3D rotation_270 = new AxisAngleRotation3D(n, 270);
            RotateTransform3D rot3D_270 = new RotateTransform3D();
            rot3D_270.Rotation = rotation_270;
            rot3D_270.CenterX = mittelpunkt.X;
            rot3D_270.CenterY = mittelpunkt.Y;
            rot3D_270.CenterZ = mittelpunkt.Z;
            Point3D resultatQ_270 = new Point3D();
            resultatQ_270 = rot3D_270.Transform(Q);

            //Matrix3D rotationshalter = Matrix3D.Identity;//entspricht: RotateTransform3D rot3D = new RotateTransform3D();
            //Quaternion q = new Quaternion(new Vector3D(p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z), 90);//beshreibt die Rotation um eine besimmte Achse. entspricht: AxisAngleRotation3D rotation = new AxisAngleRotation3D(new Vector3D(x, y, z), 90)
            //rotationshalter.Rotate(q);
            //Vector3D Q = GetQ(p1, p2, mittelpunkt); die Methode GetQ muss umgeschrieben werden, so dass Sie einen Vektor3D zurückgibt
            //rotationshalter.Transform(Q);


            //Eckpunkte der Geometire
            Point3D[] Point3DArray = new Point3D[6];
            Point3DArray[0] = p1;
            Point3DArray[1] = Q;
            Point3DArray[2] = resultatQ_90;
            Point3DArray[3] = resultatQ_180;
            Point3DArray[4] = resultatQ_270;
            Point3DArray[5] = p2;



            m.Positions.Add(Point3DArray[0]);
            m.Positions.Add(Point3DArray[1]);
            m.Positions.Add(Point3DArray[2]);
            m.Positions.Add(Point3DArray[3]);
            m.Positions.Add(Point3DArray[4]);
            m.Positions.Add(Point3DArray[5]);


            m.TriangleIndices.Add(0);
            m.TriangleIndices.Add(1);
            m.TriangleIndices.Add(4);

            m.TriangleIndices.Add(0);
            m.TriangleIndices.Add(4);
            m.TriangleIndices.Add(3);

            m.TriangleIndices.Add(0);
            m.TriangleIndices.Add(3);
            m.TriangleIndices.Add(2);

            m.TriangleIndices.Add(0);
            m.TriangleIndices.Add(2);
            m.TriangleIndices.Add(1);

            m.TriangleIndices.Add(5);
            m.TriangleIndices.Add(4);
            m.TriangleIndices.Add(1);

            m.TriangleIndices.Add(5);
            m.TriangleIndices.Add(1);
            m.TriangleIndices.Add(2);

            m.TriangleIndices.Add(5);
            m.TriangleIndices.Add(2);
            m.TriangleIndices.Add(3);

            m.TriangleIndices.Add(5);
            m.TriangleIndices.Add(3);
            m.TriangleIndices.Add(4);

            return new GeometryModel3D(m, new DiffuseMaterial(System.Windows.Media.Brushes.White));

        }

        private static Point3D Mittelpunkt(Point3D p1, Point3D p2)
        {
            Point3D MittelPunkt = new Point3D();
            MittelPunkt.X = p1.X + ((p2.X - p1.X) / 2);
            MittelPunkt.Y = p1.Y + ((p2.Y - p1.Y) / 2);
            MittelPunkt.Z = p1.Z + ((p2.Z - p1.Z) / 2);
            return MittelPunkt;
        }

        //bildet einen Punkte der in der eben Phi liegt und somit senkrecht auf die Gerade g in deren Mittelpunkt steht
        private static Point3D GetQ(Vector3D n, Point3D mittelpunkt, double thickness)
        {
            Point3D q = new Point3D();//punkt in der ebene Phi mit ungewissem absteand zum punkt mittelpunkt der auch in der ebene liegt
            q.X = 30;
            q.Y = 30;
            if (n.Z != 0)
            {
                q.Z = (dotP(n, mittelpunkt) - n.X * q.X - n.Y * q.Y) / n.Z;
            }
            else
            {
                q.Z = 30;
            }
            Point3D q_norm = Norm(q, mittelpunkt, thickness);//punkt in der ebene Phi mit bestimmtem abstand zum punkt "mittelpunkt"
            return q_norm;
        }
        private static double dotP(Vector3D v1, Point3D p1)
        {
            double result = v1.X * p1.X + v1.Y * p1.Y + v1.Z * p1.Z;
            return result;
        }
        private static Point3D Norm(Point3D p, Point3D mittelpunkt, double thickness)
        {
            Vector3D qm = new Vector3D(p.X - mittelpunkt.X, p.Y - mittelpunkt.Y, p.Z - mittelpunkt.Z);
            double betrag = Math.Sqrt(qm.X * qm.X + qm.Y * qm.Y + qm.Z * qm.Z) / (thickness / 2);
            p.X = mittelpunkt.X + qm.X / betrag;
            p.Y = mittelpunkt.Y + qm.Y / betrag;
            p.Z = mittelpunkt.Z + qm.Z / betrag;
            return p;
        }
    }
}

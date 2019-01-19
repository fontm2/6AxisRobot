using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinematics
{
    public abstract class KinematicBase
    {
        public DHParameter DHParam;

         public KinematicBase(DHParameter dhparam)
        {
            this.DHParam = dhparam;
        }
        protected double[,] MultiplyMatricesSeriell(double[,] matA, double[,] matB)
        {
            int matACols = matA.GetLength(1);
            int matBCols = matB.GetLength(1);
            int matARows = matA.GetLength(0);
            double[,] result = new double[matARows, matBCols];       
            for (int i = 0; i < matARows; i++)
            {
                for (int j = 0; j < matBCols; j++)
                {
                    for (int k = 0; k < matACols; k++)
                    {
                        result[i, j] += matA[i, k] * matB[k, j];
                    }
                    if (Math.Abs(result[i, j]) < 0.0000001)
                    {
                        result[i, j] = 0;
                    }
                    else
                    {
                        result[i, j] = result[i, j];
                    }
                }
            }
            return result;
        }
        protected double[,] DH(double teta, double d, double a, double alpha)
        {
            double[,] DHMatrixTETA = new double[4, 4];
            double[,] DHMatrixALPHA = new double[4, 4];
            if (teta == Math.PI / 2)
            {
                DHMatrixTETA = new double[4, 4] { { 0, -1, 0, 0 }, { 1, 0, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//rotZ
            }
            else if (teta == -Math.PI / 2)
            {
                DHMatrixTETA = new double[4, 4] { { 0, 1, 0, 0 }, { -1, 0, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//rotZ
            }
            else if (teta == 0)
            {
                DHMatrixTETA = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//rotZ
            }
            else
            {
                DHMatrixTETA = new double[4, 4] { { Math.Cos(teta), -Math.Sin(teta), 0, 0 }, { Math.Sin(teta), Math.Cos(teta), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//rotZ
            }
            double[,] DHMatrixD = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, d }, { 0, 0, 0, 1 } };//transZ
            double[,] DHMatrixA = new double[4, 4] { { 1, 0, 0, a }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//transX
            if (alpha == Math.PI / 2)
            {
                DHMatrixALPHA = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 0, -1, 0 }, { 0, 1, 0, 0 }, { 0, 0, 0, 1 } };//rotX
            }
            else if (alpha == -Math.PI / 2)
            {
                DHMatrixALPHA = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 0, 1, 0 }, { 0, -1, 0, 0 }, { 0, 0, 0, 1 } };//rotX
            }
            else if (alpha == 0)
            {
                DHMatrixALPHA = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//rotX
            }
            double[,] DHMatrixTETA_D = MultiplyMatricesSeriell(DHMatrixTETA, DHMatrixD);
            double[,] DHMatrixTETA_D_A = MultiplyMatricesSeriell(DHMatrixTETA_D, DHMatrixA);
            double[,] DHMatrixTETA_D_A_ALPHA = MultiplyMatricesSeriell(DHMatrixTETA_D_A, DHMatrixALPHA);
            return DHMatrixTETA_D_A_ALPHA;
        }
        protected double[,] DH123(double[,] mat1, double[,] mat2, double[,] mat3)
        {
            double[,] DH12 = MultiplyMatricesSeriell(mat1, mat2);
            double[,] DH123__ = MultiplyMatricesSeriell(DH12, mat3);
            return DH123__;
        }
        protected double[,] ExtractRotationMatrix(double[,] mat)
        {
            double[,] rot = new double[3, 3];
            rot[0, 0] = mat[0, 0];
            rot[0, 1] = mat[0, 1];
            rot[0, 2] = mat[0, 2];
            rot[1, 0] = mat[1, 0];
            rot[1, 1] = mat[1, 1];
            rot[1, 2] = mat[1, 2];
            rot[2, 0] = mat[2, 0];
            rot[2, 1] = mat[2, 1];
            rot[2, 2] = mat[2, 2];
            return rot;
        }
        protected double[,] TransposeMatrix(double[,] mat)
        {
            double[,] transpose = new double[3, 3];
            transpose[0, 0] = mat[0, 0];
            transpose[1, 0] = mat[0, 1];
            transpose[2, 0] = mat[0, 2];
            transpose[0, 1] = mat[1, 0];
            transpose[1, 1] = mat[1, 1];
            transpose[2, 1] = mat[1, 2];
            transpose[0, 2] = mat[2, 0];
            transpose[1, 2] = mat[2, 1];
            transpose[2, 2] = mat[2, 2];
            return transpose;
        }
        protected double[] MultiplyMatrixVektor(double[,] matrix1, double[] vektor)
        {
            double[] Vektor = new double[3];
            Vektor[0] = matrix1[0, 0] * vektor[0] + matrix1[0, 1] * vektor[1] + matrix1[0, 2] * vektor[2];
            Vektor[1] = matrix1[1, 0] * vektor[0] + matrix1[1, 1] * vektor[1] + matrix1[1, 2] * vektor[2];
            Vektor[2] = matrix1[2, 0] * vektor[0] + matrix1[2, 1] * vektor[1] + matrix1[2, 2] * vektor[2];
            return Vektor;
        }
        protected double[,] MultiplySkalarMatrix(double skalar_, double[,] matrix)
        {
            double[,] MatrixMalSkalar = new double[4, 4];
            MatrixMalSkalar[0, 0] = matrix[0, 0] * skalar_;
            MatrixMalSkalar[0, 1] = matrix[0, 1] * skalar_;
            MatrixMalSkalar[0, 2] = matrix[0, 2] * skalar_;
            MatrixMalSkalar[0, 3] = matrix[0, 3] * skalar_;
            MatrixMalSkalar[1, 0] = matrix[1, 0] * skalar_;
            MatrixMalSkalar[1, 1] = matrix[1, 1] * skalar_;
            MatrixMalSkalar[1, 2] = matrix[1, 2] * skalar_;
            MatrixMalSkalar[1, 3] = matrix[1, 3] * skalar_;
            MatrixMalSkalar[2, 0] = matrix[2, 0] * skalar_;
            MatrixMalSkalar[2, 1] = matrix[2, 1] * skalar_;
            MatrixMalSkalar[2, 2] = matrix[2, 2] * skalar_;
            MatrixMalSkalar[2, 3] = matrix[2, 3] * skalar_;
            MatrixMalSkalar[3, 0] = matrix[3, 0] * skalar_;
            MatrixMalSkalar[3, 1] = matrix[3, 1] * skalar_;
            MatrixMalSkalar[3, 2] = matrix[3, 2] * skalar_;
            MatrixMalSkalar[3, 3] = matrix[3, 3] * skalar_;
            return MatrixMalSkalar;
        }
        protected double[,] Get4x4(double[,] mat)
        {
            double[,] Mat4x4 = new double[4, 4];
            Mat4x4[0, 0] = mat[0, 0];
            Mat4x4[0, 1] = mat[0, 1];
            Mat4x4[0, 2] = mat[0, 2];
            Mat4x4[0, 3] = 0;
            Mat4x4[1, 0] = mat[1, 0];
            Mat4x4[1, 1] = mat[1, 1];
            Mat4x4[1, 2] = mat[1, 2];
            Mat4x4[1, 3] = 0;
            Mat4x4[2, 0] = mat[2, 0];
            Mat4x4[2, 1] = mat[2, 1];
            Mat4x4[2, 2] = mat[2, 2];
            Mat4x4[2, 3] = 0;
            Mat4x4[3, 0] = 0;
            Mat4x4[3, 1] = 0;
            Mat4x4[3, 2] = 0;
            Mat4x4[3, 3] = 1;
            return Mat4x4;
        }
        protected double[,] GetInverse(double[,] a)
        {
            var s0 = a[0, 0] * a[1, 1] - a[1, 0] * a[0, 1];
            var s1 = a[0, 0] * a[1, 2] - a[1, 0] * a[0, 2];
            var s2 = a[0, 0] * a[1, 3] - a[1, 0] * a[0, 3];
            var s3 = a[0, 1] * a[1, 2] - a[1, 1] * a[0, 2];
            var s4 = a[0, 1] * a[1, 3] - a[1, 1] * a[0, 3];
            var s5 = a[0, 2] * a[1, 3] - a[1, 2] * a[0, 3];

            var c5 = a[2, 2] * a[3, 3] - a[3, 2] * a[2, 3];
            var c4 = a[2, 1] * a[3, 3] - a[3, 1] * a[2, 3];
            var c3 = a[2, 1] * a[3, 2] - a[3, 1] * a[2, 2];
            var c2 = a[2, 0] * a[3, 3] - a[3, 0] * a[2, 3];
            var c1 = a[2, 0] * a[3, 2] - a[3, 0] * a[2, 2];
            var c0 = a[2, 0] * a[3, 1] - a[3, 0] * a[2, 1];

            // Should check for 0 determinant
            var invdet = 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

            var b = new double[4, 4];

            b[0, 0] = (a[1, 1] * c5 - a[1, 2] * c4 + a[1, 3] * c3) * invdet;
            b[0, 1] = (-a[0, 1] * c5 + a[0, 2] * c4 - a[0, 3] * c3) * invdet;
            b[0, 2] = (a[3, 1] * s5 - a[3, 2] * s4 + a[3, 3] * s3) * invdet;
            b[0, 3] = (-a[2, 1] * s5 + a[2, 2] * s4 - a[2, 3] * s3) * invdet;

            b[1, 0] = (-a[1, 0] * c5 + a[1, 2] * c2 - a[1, 3] * c1) * invdet;
            b[1, 1] = (a[0, 0] * c5 - a[0, 2] * c2 + a[0, 3] * c1) * invdet;
            b[1, 2] = (-a[3, 0] * s5 + a[3, 2] * s2 - a[3, 3] * s1) * invdet;
            b[1, 3] = (a[2, 0] * s5 - a[2, 2] * s2 + a[2, 3] * s1) * invdet;

            b[2, 0] = (a[1, 0] * c4 - a[1, 1] * c2 + a[1, 3] * c0) * invdet;
            b[2, 1] = (-a[0, 0] * c4 + a[0, 1] * c2 - a[0, 3] * c0) * invdet;
            b[2, 2] = (a[3, 0] * s4 - a[3, 1] * s2 + a[3, 3] * s0) * invdet;
            b[2, 3] = (-a[2, 0] * s4 + a[2, 1] * s2 - a[2, 3] * s0) * invdet;

            b[3, 0] = (-a[1, 0] * c3 + a[1, 1] * c1 - a[1, 2] * c0) * invdet;
            b[3, 1] = (a[0, 0] * c3 - a[0, 1] * c1 + a[0, 2] * c0) * invdet;
            b[3, 2] = (-a[3, 0] * s3 + a[3, 1] * s1 - a[3, 2] * s0) * invdet;
            b[3, 3] = (a[2, 0] * s3 - a[2, 1] * s1 + a[2, 2] * s0) * invdet;

            return b;
        }
    }
    public enum RobotType
    { SixAxis }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SVD
{
	const float _gamma = 5.828427124f;
	const float _cstar = 0.923879532f;
	const float _sstar = 0.3826834323f;
	const float EPSILON = 1e-6f;

	float accurateSqrt(float x)
	{
	    return x / Mathf.Sqrt(x);
	}

	void condSwap(bool c, ref float X, ref float Y)
	{
	    // used in step 2
	    float Z = X;
	    X = c ? Y : X;
	    Y = c ? Z : Y;
	}

	void condNegSwap(bool c, ref float X, ref float Y)
	{
	    // used in step 2 and 3
	    float Z = -X;
	    X = c ? Y : X;
	    Y = c ? Z : Y;
	}

	void multAB(float a11, float a12, float a13,
          float a21, float a22, float a23,
          float a31, float a32, float a33,
          //
          float b11, float b12, float b13,
          float b21, float b22, float b23,
          float b31, float b32, float b33,
          //
          ref float m11, ref float m12, ref float m13,
          ref float m21, ref float m22, ref float m23,
          ref float m31, ref float m32, ref float m33)
	{
	    m11=a11*b11 + a12*b21 + a13*b31; m12=a11*b12 + a12*b22 + a13*b32; m13=a11*b13 + a12*b23 + a13*b33;
	    m21=a21*b11 + a22*b21 + a23*b31; m22=a21*b12 + a22*b22 + a23*b32; m23=a21*b13 + a22*b23 + a23*b33;
	    m31=a31*b11 + a32*b21 + a33*b31; m32=a31*b12 + a32*b22 + a33*b32; m33=a31*b13 + a32*b23 + a33*b33;
	}

	// matrix multiplication M = Transpose[A] * B
	void multAtB(float a11, float a12, float a13,
	          float a21, float a22, float a23,
	          float a31, float a32, float a33,
	          //
	          float b11, float b12, float b13,
	          float b21, float b22, float b23,
	          float b31, float b32, float b33,
	          //
	          ref float m11, ref float m12, ref float m13,
	          ref float m21, ref float m22, ref float m23,
	          ref float m31, ref float m32, ref float m33)
	{
	 	m11=a11*b11 + a21*b21 + a31*b31; m12=a11*b12 + a21*b22 + a31*b32; m13=a11*b13 + a21*b23 + a31*b33;
	  	m21=a12*b11 + a22*b21 + a32*b31; m22=a12*b12 + a22*b22 + a32*b32; m23=a12*b13 + a22*b23 + a32*b33;
	  	m31=a13*b11 + a23*b21 + a33*b31; m32=a13*b12 + a23*b22 + a33*b32; m33=a13*b13 + a23*b23 + a33*b33;
	}

	void quatToMat3(float[] qV,
	ref float m11, ref float m12, ref float m13,
	ref float m21, ref float m22, ref float m23,
	ref float m31, ref float m32, ref float m33
	)
	{
	    float w = qV[3];
	    float x = qV[0];
	    float y = qV[1];
	    float z = qV[2];

	    float qxx = x*x;
	    float qyy = y*y;
	    float qzz = z*z;
	    float qxz = x*z;
	    float qxy = x*y;
	    float qyz = y*z;
	    float qwx = w*x;
	    float qwy = w*y;
	    float qwz = w*z;

	     m11=1 - 2*(qyy + qzz); m12=2*(qxy - qwz); m13=2*(qxz + qwy);
	    m21=2*(qxy + qwz); m22=1 - 2*(qxx + qzz); m23=2*(qyz - qwx);
	    m31=2*(qxz - qwy); m32=2*(qyz + qwx); m33=1 - 2*(qxx + qyy);
	}

	void approximateGivensQuaternion(float a11, float a12, float a22, ref float ch, ref float sh)
	{
	/*
	     * Given givens angle computed by approximateGivensAngles,
	     * compute the corresponding rotation quaternion.
	     */
	    ch = 2*(a11-a22);
	    sh = a12;
	    bool b = _gamma*sh*sh < ch*ch;
	    float w = 1/Mathf.Sqrt(ch*ch+sh*sh);
	    ch=b?w*ch:_cstar;
	    sh=b?w*sh:_sstar;
	}

	void jacobiConjugation(int x, int y, int z,
                        ref float s11,
                        ref float s21, ref float s22,
                        ref float s31, ref float s32, ref float s33,
                        float[] qV)
	{
	    float ch = 0;
	    float sh = 0;
	    approximateGivensQuaternion(s11,s21,s22,ref ch, ref sh);

	    float scale = ch*ch+sh*sh;
	    float a = (ch*ch-sh*sh)/scale;
	    float b = (2*sh*ch)/scale;

	    // make temp copy of S
	    float _s11 = s11;
	    float _s21 = s21; float _s22 = s22;
	    float _s31 = s31; float _s32 = s32; float _s33 = s33;

	    // perform conjugation S = Q'*S*Q
	    // Q already implicitly solved from a, b
	    s11 =a*(a*_s11 + b*_s21) + b*(a*_s21 + b*_s22);
	    s21 =a*(-b*_s11 + a*_s21) + b*(-b*_s21 + a*_s22);	s22=-b*(-b*_s11 + a*_s21) + a*(-b*_s21 + a*_s22);
	    s31 =a*_s31 + b*_s32;								s32=-b*_s31 + a*_s32; s33=_s33;

	    // update cumulative rotation qV
	    float[] tmp=new float[3];
	    tmp[0]=qV[0]*sh;
	    tmp[1]=qV[1]*sh;
	    tmp[2]=qV[2]*sh;
	    sh *= qV[3];

	    qV[0] *= ch;
	    qV[1] *= ch;
	    qV[2] *= ch;
	    qV[3] *= ch;

	    // (x,y,z) corresponds to ((0,1,2),(1,2,0),(2,0,1))
	    // for (p,q) = ((0,1),(1,2),(0,2))
	    qV[z] += sh;
	    qV[3] -= tmp[z]; // w
	    qV[x] += tmp[y];
	    qV[y] -= tmp[x];

	    // re-arrange matrix for next iteration
	    _s11 = s22;
	    _s21 = s32; _s22 = s33;
	    _s31 = s21; _s32 = s31; _s33 = s11;
	    s11 = _s11;
	    s21 = _s21; s22 = _s22;
	    s31 = _s31; s32 = _s32; s33 = _s33;

	}

	float dist2(float x, float y, float z)
	{
	    return x*x+y*y+z*z;
	}

	// finds transformation that diagonalizes a symmetric matrix
	void jacobiEigenanlysis( // symmetric matrix
	                                ref float s11,
	                                ref float s21, ref float s22,
	                                ref float s31, ref float s32, ref float s33,
	                                // quaternion representation of V
	                                float[] qV)
	{
	    qV[3]=1; qV[0]=0;qV[1]=0;qV[2]=0; // follow same indexing convention as GLM
	    for (int i=0;i<10;i++)
	    {
	        // we wish to eliminate the maximum off-diagonal element
	        // on every iteration, but cycling over all 3 possible rotations
	        // in fixed order (p,q) = (1,2) , (2,3), (1,3) still retains
	        //  asymptotic convergence
	        jacobiConjugation(0,1,2,ref s11,ref s21,ref s22,ref s31,ref s32,ref s33,qV); // p,q = 0,1
	        jacobiConjugation(1,2,0,ref s11,ref s21,ref s22,ref s31,ref s32,ref s33,qV); // p,q = 1,2
	        jacobiConjugation(2,0,1,ref s11,ref s21,ref s22,ref s31,ref s32,ref s33,qV); // p,q = 0,2
	    }
	}


	void sortSingularValues(// matrix that we want to decompose
	                            ref float b11, ref float b12, ref float b13,
	                            ref float b21, ref float b22, ref float b23,
	                            ref float b31, ref float b32, ref float b33,
	                          // sort V simultaneously
	                            ref float v11, ref float v12, ref float v13,
	                            ref float v21, ref float v22, ref float v23,
	                            ref float v31, ref float v32, ref float v33)
	{
	    float rho1 = dist2(b11,b21,b31);
	    float rho2 = dist2(b12,b22,b23);
	    float rho3 = dist2(b13,b23,b33);
	    bool c;
	    c = rho1 < rho2;
	    condNegSwap(c,ref b11,ref b12); condNegSwap(c,ref v11,ref v12);
	    condNegSwap(c,ref b21,ref b22); condNegSwap(c,ref v21,ref v22);
	    condNegSwap(c,ref b31,ref b32); condNegSwap(c,ref v31,ref v32);
	    condSwap(c,ref rho1,ref rho2);
	    c = rho1 < rho3;
	    condNegSwap(c,ref b11,ref b13); condNegSwap(c,ref v11,ref v13);
	    condNegSwap(c,ref b21,ref b23); condNegSwap(c,ref v21,ref v23);
	    condNegSwap(c,ref b31,ref b33); condNegSwap(c,ref v31,ref v33);
	    condSwap(c,ref rho1,ref rho3);
	    c = rho2 < rho3;
	    condNegSwap(c,ref b12,ref b13); condNegSwap(c,ref v12,ref v13);
	    condNegSwap(c,ref b22,ref b23); condNegSwap(c,ref v22,ref v23);
	    condNegSwap(c,ref b32,ref b33); condNegSwap(c,ref v32,ref v33);
	}

	void QRGivensQuaternion(float a1, float a2, ref float ch, ref float sh)
	{
	    // a1 = pivot point on diagonal
	    // a2 = lower triangular entry we want to annihilate
	    float epsilon = EPSILON;
	    float rho = accurateSqrt(a1*a1 + a2*a2);

	    sh = rho > epsilon ? a2 : 0;
	    ch = Mathf.Abs(a1) + Mathf.Max(rho,epsilon);
	    bool b = a1 < 0;
	    condSwap(b,ref sh,ref ch);
	    float w = 1/Mathf.Sqrt(ch*ch+sh*sh);
	    ch *= w;
	    sh *= w;
	}

	void QRDecomposition(// matrix that we want to decompose
	                            float b11, float b12, float b13,
	                            float b21, float b22, float b23,
	                            float b31, float b32, float b33,
	                            // output Q
	                            ref float q11, ref float q12, ref float q13,
	                            ref float q21, ref float q22, ref float q23,
	                            ref float q31, ref float q32, ref float q33,
	                            // output R
	                            ref float r11, ref float r12, ref float r13,
	                            ref float r21, ref float r22, ref float r23,
	                            ref float r31, ref float r32, ref float r33)
	{
	    float ch1=0,sh1=0,ch2=0,sh2=0,ch3=0,sh3=0;
	    float a,b;

	    // first givens rotation (ch,0,0,sh)
	    QRGivensQuaternion(b11,b21,ref ch1,ref sh1);
	    a=1-2*sh1*sh1;
	    b=2*ch1*sh1;
	    // apply B = Q' * B
	    r11=a*b11+b*b21;  r12=a*b12+b*b22;  r13=a*b13+b*b23;
	    r21=-b*b11+a*b21; r22=-b*b12+a*b22; r23=-b*b13+a*b23;
	    r31=b31;          r32=b32;          r33=b33;

	    // second givens rotation (ch,0,-sh,0)
	    QRGivensQuaternion(r11,r31,ref ch2,ref sh2);
	    a=1-2*sh2*sh2;
	    b=2*ch2*sh2;
	    // apply B = Q' * B;
	    b11=a*r11+b*r31;  b12=a*r12+b*r32;  b13=a*r13+b*r33;
	    b21=r21;           b22=r22;           b23=r23;
	    b31=-b*r11+a*r31; b32=-b*r12+a*r32; b33=-b*r13+a*r33;

	    // third givens rotation (ch,sh,0,0)
	    QRGivensQuaternion(b22,b32,ref ch3,ref sh3);
	    a=1-2*sh3*sh3;
	    b=2*ch3*sh3;
	    // R is now set to desired value
	    r11=b11;             r12=b12;           r13=b13;
	    r21=a*b21+b*b31;     r22=a*b22+b*b32;   r23=a*b23+b*b33;
	    r31=-b*b21+a*b31;    r32=-b*b22+a*b32;  r33=-b*b23+a*b33;

	    // construct the cumulative rotation Q=Q1 * Q2 * Q3
	    // the number of floating point operations for three quaternion multiplications
	    // is more or less comparable to the explicit form of the joined matrix.
	    // certainly more memory-efficient!
	    float sh12=sh1*sh1;
	    float sh22=sh2*sh2;
	    float sh32=sh3*sh3;

	    q11=(-1+2*sh12)*(-1+2*sh22);
	    q12=4*ch2*ch3*(-1+2*sh12)*sh2*sh3+2*ch1*sh1*(-1+2*sh32);
	    q13=4*ch1*ch3*sh1*sh3-2*ch2*(-1+2*sh12)*sh2*(-1+2*sh32);

	    q21=2*ch1*sh1*(1-2*sh22);
	    q22=-8*ch1*ch2*ch3*sh1*sh2*sh3+(-1+2*sh12)*(-1+2*sh32);
	    q23=-2*ch3*sh3+4*sh1*(ch3*sh1*sh3+ch1*ch2*sh2*(-1+2*sh32));

	    q31=2*ch2*sh2;
	    q32=2*ch3*(1-2*sh22)*sh3;
	    q33=(-1+2*sh22)*(-1+2*sh32);
	}

	void svd(// input A
	        float a11, float a12, float a13,
	        float a21, float a22, float a23,
	        float a31, float a32, float a33,
	        // output U
	        ref float u11, ref float u12, ref float u13,
	        ref float u21, ref float u22, ref float u23,
	        ref float u31, ref float u32, ref float u33,
	        // output S
	        ref float s11, ref float s12, ref float s13,
	        ref float s21, ref float s22, ref float s23,
	        ref float s31, ref float s32, ref float s33,
	        // output V
	        ref float v11, ref float v12, ref float v13,
	        ref float v21, ref float v22, ref float v23,
	        ref float v31, ref float v32, ref float v33)
	{
	    // normal equations matrix
	    float ATA11=0, ATA12=0, ATA13=0;
	    float ATA21=0, ATA22=0, ATA23=0;
	    float ATA31=0, ATA32=0, ATA33=0;

	    multAtB(a11,a12,a13,a21,a22,a23,a31,a32,a33,
	          a11,a12,a13,a21,a22,a23,a31,a32,a33,
	          ref ATA11,ref ATA12,ref ATA13,ref ATA21,ref ATA22,ref ATA23,ref ATA31,ref ATA32,ref ATA33);

	    // symmetric eigenalysis
	    float[] qV=new float[4];
	    jacobiEigenanlysis(ref ATA11,ref ATA21,ref ATA22,ref ATA31,ref ATA32,ref ATA33,qV);
	    quatToMat3(qV,ref v11,ref v12,ref v13,ref v21,ref v22,ref v23,ref v31,ref v32,ref v33);

	    float b11=0, b12=0, b13=0;
	    float b21=0, b22=0, b23=0;
	    float b31=0, b32=0, b33=0;
	    multAB(a11,a12,a13,a21,a22,a23,a31,a32,a33,
	        v11,v12,v13,v21,v22,v23,v31,v32,v33,
	        ref b11, ref b12, ref b13, ref b21, ref b22, ref b23, ref b31, ref b32, ref b33);

	    // sort singular values and find V
	    sortSingularValues(ref b11,ref b12,ref b13,ref b21,ref b22,ref b23,ref b31,ref b32,ref b33,
	                       ref v11,ref v12,ref v13,ref v21,ref v22,ref v23,ref v31,ref v32,ref v33);

	    // QR decomposition
	    QRDecomposition(b11, b12, b13, b21, b22, b23, b31, b32, b33,
		    ref u11,ref u12,ref u13,ref u21,ref u22,ref u23,ref u31,ref u32,ref u33,
		    ref s11,ref s12,ref s13,ref s21,ref s22,ref s23,ref s31,ref s32,ref s33);
	}

	public void svd(Matrix4x4 A, ref Matrix4x4 U, ref Matrix4x4 S, ref Matrix4x4 V)
	{
		float u11=0, u12=0, u13=0;
		float u21=0, u22=0, u23=0;
		float u31=0, u32=0, u33=0;

		float s11=0, s12=0, s13=0;
		float s21=0, s22=0, s23=0;
		float s31=0, s32=0, s33=0;

		float v11=0, v12=0, v13=0;
		float v21=0, v22=0, v23=0;
		float v31=0, v32=0, v33=0;

		svd(A[0, 0], A[0, 1], A[0, 2], 
			A[1, 0], A[1, 1], A[1, 2],
			A[2, 0], A[2, 1], A[2, 2],
			ref u11,ref u12,ref u13,ref u21,ref u22,ref u23,ref u31,ref u32,ref u33,
		    ref s11,ref s12,ref s13,ref s21,ref s22,ref s23,ref s31,ref s32,ref s33,
		    ref v11,ref v12,ref v13,ref v21,ref v22,ref v23,ref v31,ref v32,ref v33);
		
		U[0,0]=u11; U[0,1]=u12; U[0,2]=u13;
		U[1,0]=u21; U[1,1]=u22; U[1,2]=u23;
		U[2,0]=u31; U[2,1]=u32; U[2,2]=u33;

		S[0,0]=s11; S[0,1]=s12; S[0,2]=s13;
		S[1,0]=s21; S[1,1]=s22; S[1,2]=s23;
		S[2,0]=s31; S[2,1]=s32; S[2,2]=s33;

		V[0,0]=v11; V[0,1]=v12; V[0,2]=v13;
		V[1,0]=v21; V[1,1]=v22; V[1,2]=v23;
		V[2,0]=v31; V[2,1]=v32; V[2,2]=v33;		
	}
}

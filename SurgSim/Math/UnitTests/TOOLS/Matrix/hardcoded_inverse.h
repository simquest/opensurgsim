#ifndef MATRIX__HARDCODED_INVERSE__H
#define MATRIX__HARDCODED_INVERSE__H

#include <math.h>
#include <iostream>
#include <sstream>

//#include <SqAssert.h>
#include <SurgSim/Framework/Log.h>

// ###########################################################
// ###########################################################
// 3x3 matrix functionalities

//! Compute 3x3 matrix addition
inline void addMatrix3x3(const double *A,const double *B,double *AB)
{
	for(int line=0 ; line<3 ; line++)
	{
		for(int column=0 ; column<3 ; column++)
		{
			AB[3*line+column] = A[3*line+column]+B[3*line+column];
		}
	}
}

//! Compute 3x3 matrix addition
inline void subMatrix3x3(const double *A,const double *B,double *AB)
{
	for(int line=0 ; line<3 ; line++)
	{
		for(int column=0 ; column<3 ; column++)
		{
			AB[3*line+column] = A[3*line+column]-B[3*line+column];
		}
	}
}

//! Compute 3x3 matrix multiplication
inline void multMatrix3x3(const double *A,const double *B,double *AB)
{
	for(int line=0 ; line<3 ; line++)
	{
		for(int column=0 ; column<3 ; column++)
		{
			AB[3*line+column] = A[3*line+0]*B[3*0+column] + A[3*line+1]*B[3*1+column] + A[3*line+2]*B[3*2+column];
		}
	}
}

//! Compute the inverse of a 3x3 matrix.
//       ( a b c )          ( minv[0] minv[1] minv[2] )
//   m = ( d e f )   minv = ( minv[3] minv[4] minv[5] )
//       ( g h i )          ( minv[6] minv[7] minv[8] )
inline bool inverseMatrix3x3(double a, double b, double c, double d, double e, double f, double g, double h, double i, double *minv)
{
	double biggestAbsValue;
	double tmp;
	double inv[9];
	inv[0] = (e*i - f*h); tmp=fabs(inv[0]); biggestAbsValue=tmp;
	inv[1] = (c*h - b*i); tmp=fabs(inv[1]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;
	inv[2] = (b*f - c*e); tmp=fabs(inv[2]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;

	inv[3] = (f*g - d*i); tmp=fabs(inv[3]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;
	inv[4] = (a*i - g*c); tmp=fabs(inv[4]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;
	inv[5] = (c*d - a*f); tmp=fabs(inv[5]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;

	inv[6] = (d*h - e*g); tmp=fabs(inv[6]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;
	inv[7] = (b*g - a*h); tmp=fabs(inv[7]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;
	inv[8] = (a*e - d*b); tmp=fabs(inv[8]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;

	double det = a*inv[0] + d*inv[1] + g*inv[2];

	// Here we test if the values of the inverse matrix are not extreme from a too small determinant
	if ( fabs(det) <= biggestAbsValue*1e-50 )
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Singular matrix (determinant = "<<det<<") : " << std::endl <<
			a << " " << b << " " << c << std::endl <<
			d << " " << e << " " << f << std::endl <<
			g << " " << h << " " << i << std::endl <<
			"\t biggest cofactor absolute value = " << biggestAbsValue << std::endl <<
			"\t determinant = " << det << std::endl <<
			"\t\t biggest cofactor absolute value/det = " << biggestAbsValue/det << " > 1e50 !?" << std::endl;
		return false;
	}

	double invDet = 1.0/det;
	for (int j = 0;  j < 9;  ++j)
		minv[j] = invDet*inv[j];

	return true;
}


//! Compute the inverse of the 3x3 matrix m.
//       ( m[0] m[1] m[2] )
//   m = ( m[3] m[4] m[5] )
//       ( m[6] m[7] m[8] )
inline bool inverseMatrix3x3(const double *m,double *minv)
{
	return inverseMatrix3x3(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], minv);
}

//! Compute the inverse of a compact 3x3 sub-matrix of a matrix.
//
//       ( m[0]   m[1]     m[2]     )          ( minv[0] minv[1] minv[2] )
//   m = ( m[o]   m[o+1]   m[o+2]   )   minv = ( minv[3] minv[4] minv[5] )
//       ( m[2*o] m[2*o+1] m[2*o+2] )          ( minv[6] minv[7] minv[8] )
//
// In other words, if M is a RxC matrix, then to compute the inverse of a 3x3 sub-matrix whose upper left corner is at M[a,b],
// you would call inverseSubmatrix(&M[a*C+b], C, minv).
inline bool inverseSubmatrix3x3(const double *m, int offset, double *minv)
{
	// Note: an older version of this code used to scale the matrix elements so the biggest would end up close to 1.
	// But I'm not convinced that this really helps maximize the precision, unless any of the numbers become denormalized.
	// (Consider that scaling by 2^n just changes the exponent, while keeping the mantissa the same!)
	// Of course, I could be wrong here, which is why I'm adding this comment.  --bert 2010-06-15
	return inverseMatrix3x3(m[0], m[1], m[2], m[offset], m[offset+1], m[offset+2], m[2*offset], m[2*offset+1], m[2*offset+2], minv);
}


//! Compute the determinant of a 3x3 sub-matrix of a larger matrix m.
//  \arg <tt>nbCol</tt>: specifies the number of columns of m.  (The number of rows of the full matrix is not needed.)
//  \arg <tt>l0</tt>, <tt>l1</tt>, <tt>l2</tt>: the row indices of m that are present in the 3x3 sub-matrix.
//  \arg <tt>c0</tt>, <tt>c1</tt>, <tt>c2</tt>: the column indices of m that are present in the 3x3 sub-matrix.
inline double determinantMatrix3x3(const double *m,int nbCol,int l0,int l1,int l2,int c0,int c1,int c2)
{
	const double &m0=m[nbCol*l0 + c0] , &m1=m[nbCol*l0+c1] , &m2=m[nbCol*l0 + c2];
	const double &m3=m[nbCol*l1 + c0] , &m4=m[nbCol*l1+c1] , &m5=m[nbCol*l1 + c2];
	const double &m6=m[nbCol*l2 + c0] , &m7=m[nbCol*l2+c1] , &m8=m[nbCol*l2 + c2];

	return (m0*m4*m8 + m3*m7*m2 + m6*m1*m5 - ( m6*m4*m2 + m3*m1*m8 + m0*m7*m5 ));
}



// ###########################################################
// ###########################################################
// 2x2 matrix functionalities

//! Compute 2x2 matrix addition
inline void addMatrix2x2(const double *A,const double *B,double *AB)
{
	for(int line=0 ; line<2 ; line++)
	{
		for(int column=0 ; column<2 ; column++)
		{
			AB[2*line+column] = A[2*line+column]+B[2*line+column];
		}
	}
}

//! Compute 2x2 matrix addition
inline void subMatrix2x2(const double *A,const double *B,double *AB)
{
	for(int line=0 ; line<2 ; line++)
	{
		for(int column=0 ; column<2 ; column++)
		{
			AB[2*line+column] = A[2*line+column]-B[2*line+column];
		}
	}
}

//! Compute 2x2 matrix multiplication
inline void multMatrix2x2(const double *A,const double *B,double *AB)
{
	for(int line=0 ; line<2 ; line++)
	{
		for(int column=0 ; column<2 ; column++)
		{
			AB[2*line+column] = A[2*line+0]*B[2*0+column] + A[2*line+1]*B[2*1+column];
		}
	}
}

//! Compute the inverse of a 2x2 matrix.
//       ( a b )          ( minv[0] minv[1] )
//   m = ( c d )   minv = ( minv[2] minv[3] )
inline bool inverseMatrix2x2(double a, double b, double c, double d, double *minv)
{
	double biggestAbsValue, tmp;
	double det = a*d-b*c;

	minv[0] =  d; tmp=fabs(minv[0]); biggestAbsValue=tmp;
	minv[1] = -b; tmp=fabs(minv[1]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;

	minv[2] = -c; tmp=fabs(minv[2]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;
	minv[3] =  a; tmp=fabs(minv[3]); if(tmp>biggestAbsValue) biggestAbsValue=tmp;

	// Here we test if the values of the inverse matrix are not extreme from a too small determinant
	if ( fabs(det) <= biggestAbsValue*1e-50 )
	{
		std::cerr << "Singular matrix in 2x2 inverse (determinant = "<<det<<") : " << std::endl;
		std::cerr << a << " " << b << std::endl;
		std::cerr << c << " " << d << std::endl;
		std::cerr << "\t biggest cofactor absolute value = " << biggestAbsValue << std::endl;
		std::cerr << "\t determinant = " << det << std::endl;
		std::cerr << "\t\t biggest cofactor absolute value/det = " << biggestAbsValue/det << " > 1e50 !?" << std::endl;
		return false;
	}

	double invDet = 1.0/det;
	for (int j = 0;  j < 4;  ++j)
		minv[j] *= invDet;

	return true;
}


//! Compute the inverse of the 2x2 matrix m.
//       ( m[0] m[1] )
//   m = ( m[2] m[3] )
inline bool inverseMatrix2x2(const double *m,double *minv)
{
	return inverseMatrix2x2(m[0], m[1], m[2], m[3], minv);
}





// ###########################################################
// ###########################################################
// 4x4 matrix functionalities

//! Compute the matrix-vector product 4x4    A.x=b
inline void matrixVectorProduct4x4(const double *A,const double *x, double *b)
{
	for(int i=0 ; i<4 ; i++)
		b[i] = A[4*i+0]*x[0] + A[4*i+1]*x[1] + A[4*i+2]*x[2] + A[4*i+3]*x[3];
}

//! Compute the inverse of the 4x4 matrix m.
//       ( m[0]  m[1]  m[2]  m[3]  )
//   m = ( m[4]  m[5]  m[6]  m[7]  )
//       ( m[8]  m[9]  m[10] m[11] )
//       ( m[12] m[13] m[14] m[15] )
//
// Use a block inversion algorithm, decomposition the 6x6 matrix as 3x3 blocks:
// m=(A B)
//   (C D)
// mInv=( A^-1 + A^-1.B.(D - C.A^-1.B)^-1.C.A^-1    -A^-1.B.(D - C.A^-1.B)^-1 )
//      ( -(D - C.A^-1.B)^-1.C.A^-1                  (D - C.A^-1.B)^-1        )
inline bool inverseMatrix4x4(const double m0 , const double m1 , const double m2 , const double m3,
														 const double m4 , const double m5 , const double m6 , const double m7,
														 const double m8 , const double m9 , const double m10, const double m11,
														 const double m12, const double m13, const double m14, const double m15,
														 double *minv)
{
	double A[2][2]={m0,m1 , m4,m5};
	double Ainv[2][2];
	double B[2][2]={m2,m3 , m6,m7};
	double AinvB[2][2];
	double C[2][2]={m8,m9 , m12,m13};
	double D[2][2]={m10,m11 , m14,m15};

	double D_CAinvB[2][2];
	double D_CAinvB_Inv[2][2];
	double D_CAinvB_InvCAinv[2][2];

	if(!inverseMatrix2x2((const double *)A,(double *)Ainv)) return false;
	multMatrix2x2((const double *)Ainv,(const double *)B,(double *)AinvB);

	double tmp1[2][2],tmp2[2][2];

	multMatrix2x2((const double *)C,(const double *)Ainv,(double *)tmp1);    // tmp1=C.Ainv
	multMatrix2x2((const double *)tmp1,(const double *)B,(double *)tmp2);    // tmp2=C.Ainv.B
	subMatrix2x2((const double *)D,(const double *)tmp2,(double *)D_CAinvB); // D_CAinvB=D-C.Ainv.B

	if(!inverseMatrix2x2((const double *)D_CAinvB,(double *)D_CAinvB_Inv)) return false;

	multMatrix2x2((const double *)D_CAinvB_Inv,(const double *)C,(double *)tmp1);
	multMatrix2x2((const double *)tmp1,(const double *)Ainv,(double *)D_CAinvB_InvCAinv);

	multMatrix2x2((const double *)AinvB,(const double *)D_CAinvB_InvCAinv,(double *)tmp1); // tmp1 = A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	addMatrix2x2((const double *)Ainv,(const double *)tmp1,(double *)tmp2);                // tmp2 = A^-1 + A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	for(int i=0 ; i<2 ; i++)
		for(int j=0 ; j<2 ; j++)
			minv[4*i+j]=tmp2[i][j];

	multMatrix2x2((const double *)AinvB,(const double *)D_CAinvB_Inv,(double *)tmp1); // tmp1 = A^-1.B.(D-C.A^-1.B)^-1
	for(int i=0 ; i<2 ; i++)
		for(int j=0 ; j<2 ; j++)
			minv[4*i+2+j]=-tmp1[i][j];

	for(int i=0 ; i<2 ; i++)
		for(int j=0 ; j<2 ; j++)
			minv[4*(2+i)+j]=-D_CAinvB_InvCAinv[i][j];

	for(int i=0 ; i<2 ; i++)
		for(int j=0 ; j<2 ; j++)
			minv[4*(2+i)+2+j]=D_CAinvB_Inv[i][j];

	return true;
}

inline bool inverseMatrix4x4(const double *m,double *minv)
{
	return inverseMatrix4x4(m[0],m[1],m[2],m[3],
		m[4] ,m[5] ,m[6] ,m[7] ,
		m[8] ,m[9] ,m[10],m[11],
		m[12],m[13],m[14],m[15],
		minv);
}



inline bool inverseMatrix4x4fully(const double *m,double *minv)
{
	double inv[16];
	double biggestAbsValue, tmp;
	inv[0]  = determinantMatrix3x3(m,4, 1,2,3, 1,2,3);  tmp = fabs(inv[0]);   biggestAbsValue = tmp;
	inv[1]  = determinantMatrix3x3(m,4, 1,2,3, 0,2,3);  tmp = fabs(inv[1]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[2]  = determinantMatrix3x3(m,4, 1,2,3, 0,1,3);  tmp = fabs(inv[2]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[3]  = determinantMatrix3x3(m,4, 1,2,3, 0,1,2);  tmp = fabs(inv[3]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;

	inv[4]  = determinantMatrix3x3(m,4, 0,2,3, 1,2,3);  tmp = fabs(inv[4]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[5]  = determinantMatrix3x3(m,4, 0,2,3, 0,2,3);  tmp = fabs(inv[5]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[6]  = determinantMatrix3x3(m,4, 0,2,3, 0,1,3);  tmp = fabs(inv[6]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[7]  = determinantMatrix3x3(m,4, 0,2,3, 0,1,2);  tmp = fabs(inv[7]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;

	inv[8]  = determinantMatrix3x3(m,4, 0,1,3, 1,2,3);  tmp = fabs(inv[8]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[9]  = determinantMatrix3x3(m,4, 0,1,3, 0,2,3);  tmp = fabs(inv[9]);   if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[10] = determinantMatrix3x3(m,4, 0,1,3, 0,1,3);  tmp = fabs(inv[10]);  if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[11] = determinantMatrix3x3(m,4, 0,1,3, 0,1,2);  tmp = fabs(inv[11]);  if (tmp > biggestAbsValue) biggestAbsValue = tmp;

	inv[12] = determinantMatrix3x3(m,4, 0,1,2, 1,2,3);  tmp = fabs(inv[12]);  if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[13] = determinantMatrix3x3(m,4, 0,1,2, 0,2,3);  tmp = fabs(inv[13]);  if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[14] = determinantMatrix3x3(m,4, 0,1,2, 0,1,3);  tmp = fabs(inv[14]);  if (tmp > biggestAbsValue) biggestAbsValue = tmp;
	inv[15] = determinantMatrix3x3(m,4, 0,1,2, 0,1,2);  tmp = fabs(inv[15]);  if (tmp > biggestAbsValue) biggestAbsValue = tmp;

	double det = 
		m[0] * inv[0] -
		m[4] * inv[4] +
		m[8] * inv[8] -
		m[12]* inv[12];

	// Singular matrix ?!
	if ( fabs(det) < 1e-20*biggestAbsValue )
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Singular matrix (determinant = " << det << ") : " << std::endl <<
			m[0] << " " << m[1] << " " << m[2] << " " << m[3] << std::endl <<
			m[4] << " " << m[5] << " " << m[6] << " " << m[7] << std::endl <<
			m[8] << " " << m[9] << " " << m[10] << " " << m[11] << std::endl <<
			m[12] << " " << m[13] << " " << m[14] << " " << m[15] << std::endl;
		return false;
	}

	//( m[0]  m[1]  m[2]  m[3]  )
	//( m[4]  m[5]  m[6]  m[7]  )
	//( m[8]  m[9]  m[10] m[11] )
	//( m[12] m[13] m[14] m[15] )

	double invDet = 1.0/det;
	//for (int j = 0;  j < 16;  ++j)
	//	minv[j] = invDet * inv[j];

	minv[0] = invDet * inv[0];
	minv[1] = -invDet * inv[4];
	minv[2] = invDet * inv[8];
	minv[3] = -invDet * inv[12];

	minv[4] = -invDet * inv[1];
	minv[5] = invDet * inv[5];
	minv[6] = -invDet * inv[9];
	minv[7] = invDet * inv[13];

	minv[8] = invDet * inv[2];
	minv[9] = -invDet * inv[6];
	minv[10] = invDet * inv[10];
	minv[11] = -invDet * inv[14];

	minv[12] = -invDet * inv[3];
	minv[13] = invDet * inv[7];
	minv[14] = -invDet * inv[11];
	minv[15] = invDet * inv[15];

	//for(int i=0 ; i<4 ; i++)
	//{
	//	for(int j=0 ; j<4 ; j++)
	//	{
	//		double coef = 0;
	//		for(int k=0 ; k<4 ; k++)
	//			coef += m[4*i+k]*minv[4*k+j];
	//		printf("%g ",coef);
	//	}
	//	printf("\n");
	//}

	return true;
}





// ###########################################################
// ###########################################################
// 6x6 matrix functionalities

//! Compute the 6x6 inverse of a compact 6x6 matrix.
inline bool inverseMatrix6x6(
	const double m0 , const double m1 , const double m2 , const double m3 , const double m4 , const double m5 ,
	const double m6 , const double m7 , const double m8 , const double m9 , const double m10, const double m11,
	const double m12, const double m13, const double m14, const double m15, const double m16, const double m17,
	const double m18, const double m19, const double m20, const double m21, const double m22, const double m23,
	const double m24, const double m25, const double m26, const double m27, const double m28, const double m29,
	const double m30, const double m31, const double m32, const double m33, const double m34, const double m35,
	double *minv)
{
	// Use a block inversion algorithm, decomposition the 6x6 matrix as 3x3 blocks:
	// m=(A B)
	//   (C D)
	// mInv=( A^-1 + A^-1.B.(D - C.A^-1.B)^-1.C.A^-1    -A^-1.B.(D - C.A^-1.B)^-1 )
	//      ( -(D - C.A^-1.B)^-1.C.A^-1                  (D - C.A^-1.B)^-1        )

	double A[3][3]={m0,m1,m2 , m6,m7,m8 , m12,m13,m14};
	double Ainv[3][3];
	double B[3][3]={m3,m4,m5 , m9,m10,m11 , m15,m16,m17};
	double AinvB[3][3];
	double C[3][3]={m18,m19,m20 , m24,m25,m26 , m30,m31,m32};
	double D[3][3]={m21,m22,m23 , m27,m28,m29 , m33,m34,m35};

	double D_CAinvB[3][3];
	double D_CAinvB_Inv[3][3];
	double D_CAinvB_InvCAinv[3][3];

	if(!inverseMatrix3x3((const double *)A,(double *)Ainv)) return false;
	multMatrix3x3((const double *)Ainv,(const double *)B,(double *)AinvB);

	double tmp1[3][3],tmp2[3][3];

	multMatrix3x3((const double *)C,(const double *)Ainv,(double *)tmp1);    // tmp1=C.Ainv
	multMatrix3x3((const double *)tmp1,(const double *)B,(double *)tmp2);    // tmp2=C.Ainv.B
	subMatrix3x3((const double *)D,(const double *)tmp2,(double *)D_CAinvB); // D_CAinvB=D-C.Ainv.B

	if(!inverseMatrix3x3((const double *)D_CAinvB,(double *)D_CAinvB_Inv)) return false;

	multMatrix3x3((const double *)D_CAinvB_Inv,(const double *)C,(double *)tmp1);
	multMatrix3x3((const double *)tmp1,(const double *)Ainv,(double *)D_CAinvB_InvCAinv);

	multMatrix3x3((const double *)AinvB,(const double *)D_CAinvB_InvCAinv,(double *)tmp1); // tmp1 = A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	addMatrix3x3((const double *)Ainv,(const double *)tmp1,(double *)tmp2);                // tmp2 = A^-1 + A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*i+j]=tmp2[i][j];

	multMatrix3x3((const double *)AinvB,(const double *)D_CAinvB_Inv,(double *)tmp1); // tmp1 = A^-1.B.(D-C.A^-1.B)^-1
	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*i+3+j]=-tmp1[i][j];

	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*(3+i)+j]=-D_CAinvB_InvCAinv[i][j];

	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*(3+i)+3+j]=D_CAinvB_Inv[i][j];

	return true;
}

inline bool inverseMatrix6x6(const double *m, double *minv)
{
	// Use a block inversion algorithm, decomposition the 6x6 matrix as 3x3 blocks:
	// m=(A B)
	//   (C D)
	// mInv=( A^-1 + A^-1.B.(D - C.A^-1.B)^-1.C.A^-1    -A^-1.B.(D - C.A^-1.B)^-1 )
	//      ( -(D - C.A^-1.B)^-1.C.A^-1                  (D - C.A^-1.B)^-1        )

	double A[3][3]={m[0],m[1],m[2] , m[6],m[7],m[8] , m[12],m[13],m[14]};
	double Ainv[3][3];
	double B[3][3]={m[3],m[4],m[5] , m[9],m[10],m[11] , m[15],m[16],m[17]};
	double AinvB[3][3];
	double C[3][3]={m[18],m[19],m[20] , m[24],m[25],m[26] , m[30],m[31],m[32]};
	double D[3][3]={m[21],m[22],m[23] , m[27],m[28],m[29] , m[33],m[34],m[35]};

	double D_CAinvB[3][3];
	double D_CAinvB_Inv[3][3];
	double D_CAinvB_InvCAinv[3][3];

	if(!inverseMatrix3x3((const double *)A,(double *)Ainv)) return false;
	multMatrix3x3((const double *)Ainv,(const double *)B,(double *)AinvB);

	double tmp1[3][3],tmp2[3][3];

	multMatrix3x3((const double *)C,(const double *)Ainv,(double *)tmp1);    // tmp1=C.Ainv
	multMatrix3x3((const double *)tmp1,(const double *)B,(double *)tmp2);    // tmp2=C.Ainv.B
	subMatrix3x3((const double *)D,(const double *)tmp2,(double *)D_CAinvB); // D_CAinvB=D-C.Ainv.B

	if(!inverseMatrix3x3((const double *)D_CAinvB,(double *)D_CAinvB_Inv)) return false;

	multMatrix3x3((const double *)D_CAinvB_Inv,(const double *)C,(double *)tmp1);
	multMatrix3x3((const double *)tmp1,(const double *)Ainv,(double *)D_CAinvB_InvCAinv);

	multMatrix3x3((const double *)AinvB,(const double *)D_CAinvB_InvCAinv,(double *)tmp1); // tmp1 = A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	addMatrix3x3((const double *)Ainv,(const double *)tmp1,(double *)tmp2);                // tmp2 = A^-1 + A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*i+j]=tmp2[i][j];

	multMatrix3x3((const double *)AinvB,(const double *)D_CAinvB_Inv,(double *)tmp1); // tmp1 = A^-1.B.(D-C.A^-1.B)^-1
	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*i+3+j]=-tmp1[i][j];

	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*(3+i)+j]=-D_CAinvB_InvCAinv[i][j];

	for(int i=0 ; i<3 ; i++)
		for(int j=0 ; j<3 ; j++)
			minv[6*(3+i)+3+j]=D_CAinvB_Inv[i][j];

	return true;
}





// ###########################################################
// ###########################################################
// nxn matrix functionalities
//

template <class T,int nbLine, int nbColumn> void addMatrix_nxm(const T *A , const T *B, T * A_plus_B)
{
	for(int line=0 ; line<nbLine ; line++)
	{
		for(int column=0 ; column<nbColumn ; column++)
		{
			A_plus_B[nbColumn*line+column] = A[nbColumn*line+column] + B[nbColumn*line+column];
		}
	}
}

template <class T,int nbLine, int nbColumn> void subMatrix_nxm(const T *A , const T *B, T * A_minus_B)
{
	for(int line=0 ; line<nbLine ; line++)
	{
		for(int column=0 ; column<nbColumn ; column++)
		{
			A_minus_B[nbColumn*line+column] = A[nbColumn*line+column] - B[nbColumn*line+column];
		}
	}
}

template <class T,int nbLineA, int nbColumnB,int nbColumnA> void multMatrix_nxm(const T *A , const T *B, T * AB)
{
	for(int line=0 ; line<nbLineA ; line++)
	{
		for(int column=0 ; column<nbColumnB ; column++)
		{
			AB[nbColumnB*line+column]=(T)0.0;
			for(int columnToAdd=0 ; columnToAdd<nbColumnA ; columnToAdd++)
				AB[nbColumnB*line+column] += A[nbColumnA*line+columnToAdd]*B[nbColumnB*columnToAdd+column];
		}
	}
}

template <class T,int nbLineA, int nbColumnA> void MatrixVectorProduct_nxm(const T *A , const T *b, T * Ab)
{
	for(int line=0 ; line<nbLineA ; line++)
	{
		Ab[line] = (T)0.0;
		for(int column=0 ; column<nbColumnA ; column++)
		{
			Ab[line] += A[nbColumnA*line+column]*b[column];
		}
	}
}

//! Matrix nxn inversion using a block decomposition
//
// ( A B ) size nxn
// ( C D )
//
// with A size n1xn1
// with D size n2xn2
// => B of size n1xn2
// => C of size n2xn1
//
// Use a block inversion algorithm:
// m=(A B)
//   (C D)
// mInv=( A^-1 + A^-1.B.(D - C.A^-1.B)^-1.C.A^-1    -A^-1.B.(D - C.A^-1.B)^-1 )
//      ( -(D - C.A^-1.B)^-1.C.A^-1                  (D - C.A^-1.B)^-1        )
//
template <class T,int n,int n1,int n2> bool inverseMatrix_using_BlockDecomposition(const T *m, T *minv)
{
	T A[n1][n1];
	T Ainv[n1][n1];
	T B[n1][n2];
	T AinvB[n1][n2];
	T C[n2][n1];
	T D[n2][n2];

	for(int line=0 ; line<n1 ; line++)
	{
		for(int column=0 ; column<n1 ; column++)
			A[line][column] = m[n*line+column];
		for(int column=0 ; column<n2 ; column++)
			B[line][column] = m[n*line+n1+column];
	}
	for(int line=0 ; line<n2 ; line++)
	{
		for(int column=0 ; column<n1 ; column++)
			C[line][column] = m[n*(n1+line)+column];
		for(int column=0 ; column<n2 ; column++)
			D[line][column] = m[n*(n1+line)+n1+column];
	}

	T D_CAinvB[n2][n2];
	T D_CAinvB_Inv[n2][n2];
	T D_CAinvB_InvCAinv[n2][n1];

	if(!inverseMatrix_using_BlockDecomposition<T,n1,(n1+1)/2,n1/2>((const T *)A,(T *)Ainv)) return false;
	multMatrix_nxm<T,n1,n2,n1>((const T *)Ainv,(const T *)B,(T *)AinvB);

	T tmp_n2_n1[n2][n1], tmp_n2_n2[n2][n2];
	T tmp_n1_n1[n1][n1], tmp_n1_n2[n1][n2];

	multMatrix_nxm<T,n2,n1,n1>((const T *)C,(const T *)Ainv,(T *)tmp_n2_n1);       // tmp_n2_n1=C.Ainv
	multMatrix_nxm<T,n2,n2,n1>((const T *)tmp_n2_n1,(const T *)B,(T *)tmp_n2_n2);  // tmp2=C.Ainv.B
	subMatrix_nxm<T,n2,n2>((const T *)D,(const T *)tmp_n2_n2,(T *)D_CAinvB);       // D_CAinvB=D-C.Ainv.B

	if(!inverseMatrix_using_BlockDecomposition<T,n2,(n2+1)/2,n2/2>((const T *)D_CAinvB,(T *)D_CAinvB_Inv)) return false;

	multMatrix_nxm<T,n2,n1,n2>((const T *)D_CAinvB_Inv,(const T *)C,(T *)tmp_n2_n1);
	multMatrix_nxm<T,n2,n1,n1>((const T *)tmp_n2_n1,(const T *)Ainv,(T *)D_CAinvB_InvCAinv);

	multMatrix_nxm<T,n1,n1,n2>((const T *)AinvB,(const T *)D_CAinvB_InvCAinv,(T *)tmp_n1_n1); // tmp_n1_n1 = A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	addMatrix_nxm<T,n1,n1>((const T *)Ainv,(const T *)tmp_n1_n1,(T *)tmp_n1_n1);              // tmp_n1_n1 = A^-1 + A^-1.B.(D-C.A^-1.B)^-1.C.A^-1
	for(int i=0 ; i<n1 ; i++)
		for(int j=0 ; j<n1 ; j++)
			minv[n*i+j]=tmp_n1_n1[i][j];

	multMatrix_nxm<T,n1,n2,n2>((const T *)AinvB,(const T *)D_CAinvB_Inv,(T *)tmp_n1_n2); // tmp_n1_n2 = A^-1.B.(D-C.A^-1.B)^-1
	for(int i=0 ; i<n1 ; i++)
		for(int j=0 ; j<n2 ; j++)
			minv[n*i+n1+j]=-tmp_n1_n2[i][j];

	for(int i=0 ; i<n2 ; i++)
		for(int j=0 ; j<n1 ; j++)
			minv[n*(n1+i)+j]=-D_CAinvB_InvCAinv[i][j];

	for(int i=0 ; i<n2 ; i++)
		for(int j=0 ; j<n2 ; j++)
			minv[n*(n1+i)+n1+j]=D_CAinvB_Inv[i][j];

	return true;
}
template <> inline bool inverseMatrix_using_BlockDecomposition<double,4,2,2>(const double *m, double *minv)
{ return inverseMatrix4x4(m,minv); }
template <> inline bool inverseMatrix_using_BlockDecomposition<double,3,2,1>(const double *m, double *minv)
{ return inverseMatrix3x3(m,minv); }
template <> inline bool inverseMatrix_using_BlockDecomposition<double,3,1,2>(const double *m, double *minv)
{ return inverseMatrix3x3(m,minv); }
template <> inline bool inverseMatrix_using_BlockDecomposition<double,2,1,1>(const double *m, double *minv)
{ return inverseMatrix2x2(m,minv); }
template <> inline bool inverseMatrix_using_BlockDecomposition<double,1,1,0>(const double *m, double *minv)
{ if( fabs(*m)<1e-15 ) return false; *minv = 1.0/(*m); return true; }
template <> inline bool inverseMatrix_using_BlockDecomposition<double,1,0,1>(const double *m, double *minv)
{ if( fabs(*m)<1e-15 ) return false; *minv = 1.0/(*m); return true; }



#endif // MATRIX__HARDCODED_INVERSE__H

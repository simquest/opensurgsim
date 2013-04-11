#include "tetrahedron.h"
#include <assert.h>

#include "TOOLS/Matrix/matrix.h"
#include "TOOLS/Matrix/mat_vec.h"
#include "TOOLS/Matrix/polar_decomposition.h"

//  2          Y
//  |\         ^  Z
//  |  \       | /
//  |    \     |/
//  |      \   ----> X
//  |  3     \
//  |__________\
// 0            1
//
// centered on (0,0,0)
// barycentric coordinates from 0 to 1 in all 3 directions
// Shape functions:
//  phi0(x,y,z) = 1-x-y-z
//    dphi0/dx = -1
//    dphi0/dy = -1
//    dphi0/dz = -1

//  phi1(x,y,z) = x
//    dphi1/dx =  1
//    dphi1/dy =  0
//    dphi1/dz =  0

//  phi2(x,y,z) = y
//    dphi2/dx =  0
//    dphi2/dy =  1
//    dphi2/dz =  0

//  phi3(x,y,z) = z
//    dphi3/dx =  0
//    dphi3/dy =  0
//    dphi3/dz =  1

// Shape Function
double Tetrahedron::f(int i,double x,double y,double z)
{
	assert(i>=0 && i<=3);
	return inv_6V*(ai[i] + bi[i]*x + ci[i]*y + di[i]*z);
}

double Tetrahedron::dfdx(int i,double x,double y,double z)
{
	assert(i>=0 && i<=3);
	return inv_6V*bi[i];
}
double Tetrahedron::dfdy(int i,double x,double y,double z)
{
	assert(i>=0 && i<=3);
	return inv_6V*ci[i];
}
double Tetrahedron::dfdz(int i,double x,double y,double z)
{
	assert(i>=0 && i<=3);
	return inv_6V*di[i];
}


//void Tetrahedron::ComputeStiffnessMatrices(void)
//{
//  double a,b,c;
//  double d,e;
//
//  double V =  Volume();
//
//  a = V*(2*mu + lambda);  // E*(1.-nu)/( (1.+nu)*(1.-2.*nu) );
//  b = V*lambda;           // E*nu/((1.+nu)*(1.-2.*nu));
//  c = V*mu;               // E/(2.+2.*nu);
//
//
//  //cout << "Tetrahedron Volume = " << V << endl;
//  d = a+2.*c;          // (2*mu+lambda) + 2*mu
//  e = b+c;             // lambda + mu
//
//  // Diagonal 3x3 blocks
//  {
//    fillUp3x3(0,0, d,e,e, e,d,e, e,e,d);
//    fillUp3x3(1,1, a,0,0, 0,c,0, 0,0,c);
//    fillUp3x3(2,2, c,0,0, 0,a,0, 0,0,c);
//    fillUp3x3(3,3, c,0,0, 0,c,0, 0,0,a);
//  }
//
//  // Non diagonal 3x3 block
//  {
//    fillUp3x3(0,1, -a,-c,-c, -b,-c, 0, -b, 0,-c);  fillUp3x3(1,0, -a,-b,-b, -c,-c, 0, -c, 0,-c);
//    fillUp3x3(0,2, -c,-b, 0, -c,-a,-c,  0,-b,-c);  fillUp3x3(2,0, -c,-c, 0, -b,-a,-b,  0,-c,-c);
//    fillUp3x3(0,3, -c, 0,-b,  0,-c,-b, -c,-c,-a);  fillUp3x3(3,0, -c, 0,-c,  0,-c,-c, -b,-b,-a);
//
//    fillUp3x3(1,2, 0,b,0, c,0,0, 0,0,0);  fillUp3x3(2,1, 0,c,0, b,0,0, 0,0,0);
//    fillUp3x3(1,3, 0,0,b, 0,0,0, c,0,0);  fillUp3x3(3,1, 0,0,c, 0,0,0, b,0,0);
//
//    fillUp3x3(2,3, 0,0,0, 0,0,b, 0,c,0);  fillUp3x3(3,2, 0,0,0, 0,0,c, 0,b,0);
//  }
//
//  // Compute global stiffness matrix: KGlobal = R^t*KLocal*R
//  Static_Matrix<double,12,12> tmp;
//  mat_mul(Rt,KeLocal,tmp);
//  mat_mul(tmp,R,KeGlobal);
//  // Force the symetry
//  for(int i=0;i<12;i++)
//    for(int j=i+1;j<12;j++)
//    {
//      double avg = (KeGlobal[i][j]+KeGlobal[j][i])/2.0;
//      KeGlobal[j][i] = avg;
//      KeGlobal[i][j] = avg;
//    }
//}

// http://books.google.fr/books?id=8H6OF5jZENoC&pg=PA164&lpg=PA164&dq=analytical+stiffness+tetrahedron&source=bl&ots=L5F3sR94He&sig=9Jhk8IApNX0oTUkLh4UeSrz6Ga8&hl=fr&ei=j5xDSpe8I5y_twe1t4m4Ag&sa=X&oi=book_result&ct=result&resnum=2
void Tetrahedron::ComputeStiffnessMatrices(void)
{
	// Compute the Stress and Strain matrices
	{
		mat_null<double>(B);
		mat_null<double>(Bt);
		mat_null<double>(Em);
		for (int i=0 ; i<4 ; i++)
		{
			B[0][3*i  ] = inv_6V*bi[i];
			B[1][3*i+1] = inv_6V*ci[i];
			B[2][3*i+2] = inv_6V*di[i];
			B[3][3*i  ] = inv_6V*ci[i];
			B[3][3*i+1] = inv_6V*bi[i];
			B[4][3*i+1] = inv_6V*di[i];
			B[4][3*i+2] = inv_6V*ci[i];
			B[5][3*i  ] = inv_6V*di[i];
			B[5][3*i+2] = inv_6V*bi[i];
		}
		mat_transpose(B,Bt);

		Em[0][0]=Em[1][1]=Em[2][2]=2.*mu+lambda;
		Em[0][1]=Em[0][2]=Em[1][0]=Em[1][2]=Em[2][0]=Em[2][1]=lambda;
		Em[3][3]=Em[4][4]=Em[5][5]=mu;
	}

	//  cout << "Bt=" << endl << Bt << endl;
	//  cout << endl << "Em=" << endl << Em << endl;
	//  cout << endl << "B=" << endl << B << endl;
	// Compute the local stiffness matrix K = 1/(36V^2) B.Em.Bt V = 1/(36V) B.Em.Bt
	{
		Static_Matrix<double,6,12> EmB;
		mat_mul(Em,B,EmB);
		//    cout << "EmB=" << endl << EmB << endl;
		mat_mul(Bt,EmB,KeLocal);
		//    cout << "BtEmB=" << endl << KeLocal << endl;
		//mat_invScale(KeLocal,36.*VolumeInitial()); // We do this if B does not include the factor inv_6V...
		mat_scale(KeLocal,VolumeInitial());
	}

	// Ke is symmetric but computation might be slightly (1e-10) different
	// => Force the symmetry
	for (int i=0; i<12; i++)
	{
		for (int j=i+1; j<12; j++)
		{
			double avg = (KeLocal[i][j]+KeLocal[j][i])/2.0;
			KeLocal[j][i] = avg;
			KeLocal[i][j] = avg;
		}
	}

	// KeGlobal = KeLocal in 3D as the shape functions are directly expressed in the global space => R0=I
	mat_copy(KeLocal, KeGlobal, 12,12);

	//###########################################
	// Set the relative transformation for the corotational model to Identity
	// This set the rotation R, Rt
	// but also the matrices RK, RKRt, RMRt
	SqMatrix33<double> Id33;
	Id33.setIdentity();
	setCurrentRotation(Id33);


	//// Compute global stiffness taking into account rotation matrix R
	//mat_mul(R ,KeGlobal, RKeGlobal , 12,12);
	//mat_mul(RKeGlobal, Rt, RKeGlobalRt , 12,12);

	//// Compute initial rotation matrix
	//ComputeChangeOfBasisMatrix(undeformedMesh,undeformedMesh);

	//// Compute global stiffness matrix: KGlobal = (R_initial.R^t*KLocal*R.Rt_initial)
	//{
	//	Static_Matrix<double,3,3> Rinitial_Rt;
	//	mat_mul(R_initial,Rt,Rinitial_Rt,3,3);

	//	// Compute R_initial.R^t*KLocal
	//	Static_Matrix<double,12,12> Rinitial_Rt_Klocal;
	//	for(int lineBlock=0 ; lineBlock<4 ; lineBlock++)
	//	{
	//		for(int colBlock=0 ; colBlock<4 ; colBlock++)
	//		{
	//			// R[0..2][0..2].Ke[3*lineBlock..3*lineBlock+2][3*colBlock..3*colBlock+2]
	//			for(int line=0 ; line<3 ; line++)
	//				for(int col=0 ; col<3 ; col++)
	//					Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+col] =
	//					Rinitial_Rt[line][0]*KeLocal[3*lineBlock+0][3*colBlock+col] +
	//					Rinitial_Rt[line][1]*KeLocal[3*lineBlock+1][3*colBlock+col] +
	//					Rinitial_Rt[line][2]*KeLocal[3*lineBlock+2][3*colBlock+col];
	//		}
	//	}
	//	// KGlobal = (R_initial.R^t*KLocal*R.Rt_initial)
	//	for(int lineBlock=0 ; lineBlock<4 ; lineBlock++)
	//	{
	//		for(int colBlock=0 ; colBlock<4 ; colBlock++)
	//		{
	//			// R[0..2][0..2].Ke[3*lineBlock..3*lineBlock+2][3*colBlock..3*colBlock+2]
	//			for(int line=0 ; line<3 ; line++)
	//				for(int col=0 ; col<3 ; col++)
	//					KeGlobal[3*lineBlock+line][3*colBlock+col] =
	//					Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+0] * Rinitial_Rt[col][0] +
	//					Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+1] * Rinitial_Rt[col][1] +
	//					Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+2] * Rinitial_Rt[col][2];
	//		}
	//	}
	//	//mat_mul(Rt,KeLocal,tmp);
	// //  mat_mul(tmp,R,KeGlobal);
	//   // Force the symmetry
	//   for(int i=0;i<12;i++)
	//     for(int j=i+1;j<12;j++)
	//     {
	//			double diff=fabs(KeGlobal[i][j]-KeGlobal[j][i]);
	//			if( diff>1e-10 )
	//				printf("KeGlobal is not really symmetric K[%d][%d] diff=%g \n",i,j,diff);
	//       double avg = (KeGlobal[i][j]+KeGlobal[j][i])/2.0;
	//       KeGlobal[j][i] = avg;
	//       KeGlobal[i][j] = avg;
	//     }
	// }


	//  double _1_over_36V =  inv_6V / 6.;
	//
	//  double a = _1_over_36V*(2*mu + lambda);    // E*(1.-nu)/( (1.+nu)*(1.-2.*nu) );
	//  double b = _1_over_36V*(lambda);
	//  double c = _1_over_36V*(mu);
	//
	//  //cout << "Tetrahedron Volume = " << V << endl;
	//  double e = b+c;             // lambda + mu
	//
	//  // Diagonal 3x3 blocks
	//  {
	//    fillUp3x3(0,0,
	//      bi[0]*bi[0]*a + (ci[0]*ci[0]+di[0]*di[0])*c , bi[0]*ci[0]*e , bi[0]*di[0]*e ,
	//      bi[0]*ci[0]*e , ci[0]*ci[0]*a + (bi[0]*bi[0]+di[0]*di[0])*c , ci[0]*di[0]*e ,
	//      bi[0]*di[0]*e , ci[0]*di[0]*e , di[0]*di[0]*a + (bi[0]*bi[0]+ci[0]*ci[0])*c );
	//    fillUp3x3(1,1,
	//      bi[1]*bi[1]*a + (ci[1]*ci[1]+di[1]*di[1])*c , bi[1]*ci[1]*e , bi[1]*di[1]*e ,
	//      bi[1]*ci[1]*e , ci[1]*ci[1]*a + (bi[1]*bi[1]+di[1]*di[1])*c , ci[1]*di[1]*e ,
	//      bi[1]*di[1]*e , ci[1]*di[1]*e , di[1]*di[1]*a + (bi[1]*bi[1]+ci[1]*ci[1])*c );
	//    fillUp3x3(2,2,
	//      bi[2]*bi[2]*a + (ci[2]*ci[2]+di[2]*di[2])*c , bi[2]*ci[2]*e , bi[2]*di[2]*e ,
	//      bi[2]*ci[2]*e , ci[2]*ci[2]*a + (bi[2]*bi[2]+di[2]*di[2])*c , ci[2]*di[2]*e ,
	//      bi[2]*di[2]*e , ci[2]*di[2]*e , di[2]*di[2]*a + (bi[2]*bi[2]+ci[2]*ci[2])*c );
	//    fillUp3x3(3,3,
	//      bi[3]*bi[3]*a + (ci[3]*ci[3]+di[3]*di[3])*c , bi[3]*ci[3]*e , bi[3]*di[3]*e ,
	//      bi[3]*ci[3]*e , ci[3]*ci[3]*a + (bi[3]*bi[3]+di[3]*di[3])*c , ci[3]*di[3]*e ,
	//      bi[3]*di[3]*e , ci[3]*di[3]*e , di[3]*di[3]*a + (bi[3]*bi[3]+ci[3]*ci[3])*c );
	//  }
	//
	//  // Non diagonal 3x3 block
	//  {
	//    fillUp3x3(0,1,
	//      bi[0]*bi[1]*a + (ci[0]*ci[1]+di[0]*di[1])*c , bi[0]*ci[1]*b + ci[0]*bi[1]*c , bi[0]*di[1]*b + di[0]*bi[1]*c ,
	//      ci[0]*bi[1]*b + bi[0]*ci[1]*c , ci[0]*ci[1]*a + (bi[0]*bi[1]+di[0]*di[1])*c , ci[0]*di[1]*b + di[0]*ci[1]*c ,
	//      di[0]*bi[1]*e + bi[0]*di[1]*c , di[0]*ci[1]*b + ci[0]*di[1]*c , di[0]*di[1]*a + (bi[0]*bi[1]+ci[0]*ci[1])*c );
	//    fillUp3x3(1,0,
	//      bi[0]*bi[1]*a + (ci[0]*ci[1]+di[0]*di[1])*c , ci[0]*bi[1]*b + bi[0]*ci[1]*c , di[0]*bi[1]*e + bi[0]*di[1]*c ,
	//      bi[0]*ci[1]*b + ci[0]*bi[1]*c , ci[0]*ci[1]*a + (bi[0]*bi[1]+di[0]*di[1])*c , di[0]*ci[1]*b + ci[0]*di[1]*c ,
	//      bi[0]*di[1]*b + di[0]*bi[1]*c , ci[0]*di[1]*b + di[0]*ci[1]*c , di[0]*di[1]*a + (bi[0]*bi[1]+ci[0]*ci[1])*c );
	//
	//    fillUp3x3(0,2,
	//      bi[0]*bi[2]*a + (ci[0]*ci[2]+di[0]*di[2])*c , bi[0]*ci[2]*b + ci[0]*bi[2]*c , bi[0]*di[2]*b + di[0]*bi[2]*c ,
	//      ci[0]*bi[2]*b + bi[0]*ci[2]*c , ci[0]*ci[2]*a + (bi[0]*bi[2]+di[0]*di[2])*c , ci[0]*di[2]*b + di[0]*ci[2]*c ,
	//      di[0]*bi[2]*b + bi[0]*di[2]*c , di[0]*ci[2]*b + ci[0]*di[2]*c , di[0]*di[2]*a + (ci[0]*ci[2]+bi[0]*bi[2])*c );
	//    fillUp3x3(2,0,
	//      bi[0]*bi[2]*a + (ci[0]*ci[2]+di[0]*di[2])*c , ci[0]*bi[2]*b + bi[0]*ci[2]*c , di[0]*bi[2]*b + bi[0]*di[2]*c ,
	//      bi[0]*ci[2]*b + ci[0]*bi[2]*c , ci[0]*ci[2]*a + (bi[0]*bi[2]+di[0]*di[2])*c , di[0]*ci[2]*b + ci[0]*di[2]*c ,
	//      bi[0]*di[2]*b + di[0]*bi[2]*c , ci[0]*di[2]*b + di[0]*ci[2]*c , di[0]*di[2]*a + (ci[0]*ci[2]+bi[0]*bi[2])*c );
	//
	//
	//    fillUp3x3(0,3,
	//      bi[0]*bi[3]*a + (ci[0]*ci[3]+di[0]*di[3])*c , bi[0]*ci[3]*b + ci[0]*bi[3]*c , bi[0]*di[3]*b + di[0]*bi[3]*c ,
	//      ci[0]*bi[3]*b + bi[0]*ci[3]*c , ci[0]*ci[3]*a + (bi[0]*bi[3]+di[0]*di[3])*c , ci[0]*di[3]*b + di[0]*ci[3]*c ,
	//      di[0]*bi[3]*b + bi[0]*di[3]*c , di[0]*ci[3]*b + ci[0]*di[3]*c , di[0]*di[3]*a + (ci[0]*ci[3]+bi[0]*bi[3])*c );
	//    fillUp3x3(3,0,
	//      bi[0]*bi[3]*a + (ci[0]*ci[3]+di[0]*di[3])*c , ci[0]*bi[3]*b + bi[0]*ci[3]*c , di[0]*bi[3]*b + bi[0]*di[3]*c ,
	//      bi[0]*ci[3]*b + ci[0]*bi[3]*c , ci[0]*ci[3]*a + (bi[0]*bi[3]+di[0]*di[3])*c , di[0]*ci[3]*b + ci[0]*di[3]*c ,
	//      bi[0]*di[3]*b + di[0]*bi[3]*c , ci[0]*di[3]*b + di[0]*ci[3]*c , di[0]*di[3]*a + (ci[0]*ci[3]+bi[0]*bi[3])*c );
	//
	//    fillUp3x3(1,2,
	//      bi[1]*bi[2]*a + (ci[1]*ci[2]+di[1]*di[2])*c , bi[1]*ci[2]*b + ci[1]*bi[2]*c , bi[1]*di[2]*b + di[1]*bi[2]*c ,
	//      ci[1]*bi[2]*b + bi[1]*ci[2]*c , ci[1]*ci[2]*a + (bi[1]*bi[2]+di[1]*di[2])*c , ci[1]*di[2]*b + di[1]*ci[2]*c ,
	//      di[1]*bi[2]*b + bi[1]*di[2]*c , di[1]*ci[2]*b + ci[1]*di[2]*c , di[1]*di[2]*a + (ci[1]*ci[2]+bi[1]*bi[2])*c );
	//    fillUp3x3(2,1,
	//      bi[1]*bi[2]*a + (ci[1]*ci[2]+di[1]*di[2])*c , ci[1]*bi[2]*b + bi[1]*ci[2]*c , di[1]*bi[2]*b + bi[1]*di[2]*c ,
	//      bi[1]*ci[2]*b + ci[1]*bi[2]*c , ci[1]*ci[2]*a + (bi[1]*bi[2]+di[1]*di[2])*c , di[1]*ci[2]*b + ci[1]*di[2]*c ,
	//      bi[1]*di[2]*b + di[1]*bi[2]*c , ci[1]*di[2]*b + di[1]*ci[2]*c , di[1]*di[2]*a + (ci[1]*ci[2]+bi[1]*bi[2])*c );
	//
	//    fillUp3x3(1,3,
	//      bi[1]*bi[3]*a + (ci[1]*ci[3]+di[1]*di[3])*c , bi[1]*ci[3]*b + ci[1]*bi[3]*c , bi[1]*di[3]*b + di[1]*bi[3]*c ,
	//      ci[1]*bi[3]*b + bi[1]*ci[3]*c , ci[1]*ci[3]*a + (bi[1]*bi[3]+di[1]*di[3])*c , ci[1]*di[3]*b + di[1]*ci[3]*c ,
	//      di[1]*bi[3]*b + bi[1]*di[3]*c , di[1]*ci[3]*b + ci[1]*di[3]*c , di[1]*di[3]*a + (ci[1]*ci[3]+bi[1]*bi[3])*c );
	//    fillUp3x3(3,1,
	//      bi[1]*bi[3]*a + (ci[1]*ci[3]+di[1]*di[3])*c , ci[1]*bi[3]*b + bi[1]*ci[3]*c , di[1]*bi[3]*b + bi[1]*di[3]*c ,
	//      bi[1]*ci[3]*b + ci[1]*bi[3]*c , ci[1]*ci[3]*a + (bi[1]*bi[3]+di[1]*di[3])*c , di[1]*ci[3]*b + ci[1]*di[3]*c ,
	//      bi[1]*di[3]*b + di[1]*bi[3]*c , ci[1]*di[3]*b + di[1]*ci[3]*c , di[1]*di[3]*a + (ci[1]*ci[3]+bi[1]*bi[3])*c );
	//
	//    fillUp3x3(2,3,
	//      bi[2]*bi[3]*a + (ci[2]*ci[3]+di[2]*di[3])*c , bi[2]*ci[3]*b + ci[2]*bi[3]*c , bi[2]*di[3]*b + di[2]*bi[3]*c ,
	//      ci[2]*bi[3]*b + bi[2]*ci[3]*c , ci[2]*ci[3]*a + (bi[2]*bi[3]+di[2]*di[3])*c , ci[2]*di[3]*b + di[2]*ci[3]*c ,
	//      di[2]*bi[3]*b + bi[2]*di[3]*c , di[2]*ci[3]*b + ci[2]*di[3]*c , di[2]*di[3]*a + (ci[2]*ci[3]+bi[2]*bi[3])*c );
	//    fillUp3x3(3,2,
	//      bi[2]*bi[3]*a + (ci[2]*ci[3]+di[2]*di[3])*c , ci[2]*bi[3]*b + bi[2]*ci[3]*c , di[2]*bi[3]*b + bi[2]*di[3]*c ,
	//      bi[2]*ci[3]*b + ci[2]*bi[3]*c , ci[2]*ci[3]*a + (bi[2]*bi[3]+di[2]*di[3])*c , di[2]*ci[3]*b + ci[2]*di[3]*c ,
	//      bi[2]*di[3]*b + di[2]*bi[3]*c , ci[2]*di[3]*b + di[2]*ci[3]*c , di[2]*di[3]*a + (ci[2]*ci[3]+bi[2]*bi[3])*c );
	//  }
	//
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  0,0,0,0,0,0);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  0,3,0,0,0,3);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  0,6,0,0,0,6);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  0,9,0,0,0,9);
	//
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  3,0,0,0,3,0);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  3,3,0,0,3,3);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  3,6,0,0,3,6);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  3,9,0,0,3,9);
	//
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  6,0,0,0,6,0);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  6,3,0,0,6,3);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  6,6,0,0,6,6);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  6,9,0,0,6,9);
	//
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  9,0,0,0,9,0);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  9,3,0,0,9,3);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  9,6,0,0,9,6);
	//  //mat_mul(KeLocal , Jt , KeGlobal, 3,3,3,  9,9,0,0,9,9);
	//
	//
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,0,0,0,0);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,0,3,0,3);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,0,6,0,6);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,0,9,0,9);
	//
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,3,0,3,0);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,3,3,3,3);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,3,6,3,6);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,3,9,3,9);
	//
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,6,0,6,0);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,6,3,6,3);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,6,6,6,6);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,6,9,6,9);
	//
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,9,0,9,0);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,9,3,9,3);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,9,6,9,6);
	//  //mat_mul(J, KeGlobal , KeLocal, 3,3,3,  0,0,9,9,9,9);
	//
	//  //// Force the symetry
	//  //for(int i=0;i<12;i++)
	//  //  for(int j=i+1;j<12;j++)
	//  //  {
	//  //    double avg = (KeLocal[i][j]+KeLocal[j][i])/2.0;
	//  //    KeLocal[j][i] = avg;
	//  //    KeLocal[i][j] = avg;
	//  //  }
	//
	//
	////  mat_copy<double,double>(KeGlobal , KeLocal , 12,12);
	//
	//  // Compute global stiffness matrix: KGlobal = R^t*KLocal*R
	//  Static_Matrix<double,12,12> tmp;
	//  mat_mul(Rt,KeLocal,tmp);
	//  mat_mul(tmp,R,KeGlobal);
	//
	//  // Force the symetry
	//  for(int i=0;i<12;i++)
	//    for(int j=i+1;j<12;j++)
	//    {
	//      double avg = (KeGlobal[i][j]+KeGlobal[j][i])/2.0;
	//      KeGlobal[j][i] = avg;
	//      KeGlobal[i][j] = avg;
	//    }
}

// http://books.google.fr/books?id=8H6OF5jZENoC&pg=PA164&lpg=PA164&dq=analytical+stiffness+tetrahedron&source=bl&ots=L5F3sR94He&sig=9Jhk8IApNX0oTUkLh4UeSrz6Ga8&hl=fr&ei=j5xDSpe8I5y_twe1t4m4Ag&sa=X&oi=book_result&ct=result&resnum=2
void Tetrahedron::UpdateGlobalStiffnessMatrix(const Pt3D* undeformedMesh, const Pt3D* deformedMesh)
{
	// Compute transformation matrix R and R^t
	ComputeChangeOfBasisMatrix(undeformedMesh,deformedMesh);

	// Compute global stiffness matrix: KGlobal = (R*KLocal*Rt)
	{
		const Static_Matrix<double,12,12>& R  = Get_R();
		const Static_Matrix<double,12,12>& Rt = Get_Rt();

		mat_mul(R,KeGlobal,RKeGlobal,12,12,12);
		mat_mul(RKeGlobal,Rt,RKeGlobalRt,12,12,12);

		//// Compute R_initial.R^t*KLocal
		//Static_Matrix<double,12,12> Rinitial_Rt_Klocal;
		//for(int lineBlock=0 ; lineBlock<4 ; lineBlock++)
		//{
		//	for(int colBlock=0 ; colBlock<4 ; colBlock++)
		//	{
		//		// R[0..2][0..2].Ke[3*lineBlock..3*lineBlock+2][3*colBlock..3*colBlock+2]
		//		for(int line=0 ; line<3 ; line++)
		//			for(int col=0 ; col<3 ; col++)
		//				Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+col] =
		//				Rinitial_Rt[line][0]*KeLocal[3*lineBlock+0][3*colBlock+col] +
		//				Rinitial_Rt[line][1]*KeLocal[3*lineBlock+1][3*colBlock+col] +
		//				Rinitial_Rt[line][2]*KeLocal[3*lineBlock+2][3*colBlock+col];
		//	}
		//}
		//// KGlobal = (R_initial.R^t*KLocal*R.Rt_initial)
		//for(int lineBlock=0 ; lineBlock<4 ; lineBlock++)
		//{
		//	for(int colBlock=0 ; colBlock<4 ; colBlock++)
		//	{
		//		// R[0..2][0..2].Ke[3*lineBlock..3*lineBlock+2][3*colBlock..3*colBlock+2]
		//		for(int line=0 ; line<3 ; line++)
		//			for(int col=0 ; col<3 ; col++)
		//				KeGlobal[3*lineBlock+line][3*colBlock+col] =
		//				Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+0] * Rinitial_Rt[col][0] +
		//				Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+1] * Rinitial_Rt[col][1] +
		//				Rinitial_Rt_Klocal[3*lineBlock+line][3*colBlock+2] * Rinitial_Rt[col][2];
		//	}
		//}
		//// Force the symmetry
		//for(int i=0;i<12;i++)
		//	for(int j=i+1;j<12;j++)
		//	{
		//		//double diff=fabs(RKeLocalRt[i][j]-RKeLocalRt[j][i]);
		//		//if( diff>1e-10 )
		//		//	printf("KeGlobal is not really symmetric K[%d][%d] diff=%g \n",i,j,diff);
		//		double avg = (RKeLocalRt[i][j]+RKeLocalRt[j][i])/2.0;
		//		RKeLocalRt[j][i] = avg;
		//		RKeLocalRt[i][j] = avg;
		//	}
	}


	//// Compute global stiffness matrix: KGlobal = R^t*KLocal*R
	//{
	//	Static_Matrix<double,12,12> tmp;
	//	mat_mul(Rt,KeLocal,tmp);
	//	mat_mul(tmp,R,KeGlobal);
	//	// Force the symmetry
	//	for(int i=0;i<12;i++)
	//		for(int j=i+1;j<12;j++)
	//		{
	//			double avg = (KeGlobal[i][j]+KeGlobal[j][i])/2.0;
	//			KeGlobal[j][i] = avg;
	//			KeGlobal[i][j] = avg;
	//		}
	//}
}

// x & res general vector (use internal indexing for accessing them)
void Tetrahedron::MatVecProduct_K_u(const double* u, const double* x, const double* x0, double* f)
{
	if (!m_corotational)
	{
		const Static_Matrix<double,12,12>& Ke = GetLocalStiffnessMatrixInGlobalSpace();

		for (int i=0; i<4; i++)
		{
			int I = pointsID[i];

			for (int j=0; j<4; j++)
			{
				int J = pointsID[j];

				f[3*I  ] += Ke[3*i  ][3*j]*u[3*J] + Ke[3*i  ][3*j+1]*u[3*J+1] + Ke[3*i  ][3*j+2]*u[3*J+2];
				f[3*I+1] += Ke[3*i+1][3*j]*u[3*J] + Ke[3*i+1][3*j+1]*u[3*J+1] + Ke[3*i+1][3*j+2]*u[3*J+2];
				f[3*I+2] += Ke[3*i+2][3*j]*u[3*J] + Ke[3*i+2][3*j+1]*u[3*J+1] + Ke[3*i+2][3*j+2]*u[3*J+2];
			}
		}
	}
	else
	{
		const Static_Matrix<double,12,12>& K = GetGlobalStiffnessMatrix();
		const Static_Matrix<double,12,12>& RKe = Get_RKeGlobal();

		for (int i=0; i<4; i++)
		{
			int I = pointsID[i];

			for (int j=0; j<4; j++)
			{
				int J = pointsID[j];

				// R.Ke.(Rt x(t+dt) - x(0)) = (R.Ke.Rt).(x(t)+u) - (R.Ke).x(0)
				f[3*I  ] += K[3*i  ][3*j]*(x[3*J]+u[3*J]) + K[3*i  ][3*j+1]*(x[3*J+1]+u[3*J+1]) + K[3*i  ][3*j+2]*(x[3*J+2]+u[3*J+2]);
				f[3*I+1] += K[3*i+1][3*j]*(x[3*J]+u[3*J]) + K[3*i+1][3*j+1]*(x[3*J+1]+u[3*J+1]) + K[3*i+1][3*j+2]*(x[3*J+2]+u[3*J+2]);
				f[3*I+2] += K[3*i+2][3*j]*(x[3*J]+u[3*J]) + K[3*i+2][3*j+1]*(x[3*J+1]+u[3*J+1]) + K[3*i+2][3*j+2]*(x[3*J+2]+u[3*J+2]);

				f[3*I  ] -= RKe[3*i  ][3*j]*x0[3*J] + RKe[3*i  ][3*j+1]*x0[3*J+1] + RKe[3*i  ][3*j+2]*x0[3*J+2];
				f[3*I+1] -= RKe[3*i+1][3*j]*x0[3*J] + RKe[3*i+1][3*j+1]*x0[3*J+1] + RKe[3*i+1][3*j+2]*x0[3*J+2];
				f[3*I+2] -= RKe[3*i+2][3*j]*x0[3*J] + RKe[3*i+2][3*j+1]*x0[3*J+1] + RKe[3*i+2][3*j+2]*x0[3*J+2];
				//f[3*I  ] -= RKe[3*i  ][3*j]*undeformedRotatedElement[j][0] + RKe[3*i  ][3*j+1]*undeformedRotatedElement[j][1] + RKe[3*i  ][3*j+2]*undeformedRotatedElement[j][2];
				//f[3*I+1] -= RKe[3*i+1][3*j]*undeformedRotatedElement[j][0] + RKe[3*i+1][3*j+1]*undeformedRotatedElement[j][1] + RKe[3*i+1][3*j+2]*undeformedRotatedElement[j][2];
				//f[3*I+2] -= RKe[3*i+2][3*j]*undeformedRotatedElement[j][0] + RKe[3*i+2][3*j+1]*undeformedRotatedElement[j][1] + RKe[3*i+2][3*j+2]*undeformedRotatedElement[j][2];
			}
		}
	}

}

// x & res general vector (use internal indexing for accessing them)
void Tetrahedron::MatVecProduct_MDK_u(double dt, double rayleight_coef_mass, double rayleight_coef_stif, const double* u, const double* x, const double* x0, double* res)
{
	double coefK = (1.0          +  rayleight_coef_stif/dt);
	double coefM = (1.0/(dt*dt)) + (rayleight_coef_mass/dt);

	const Static_Matrix<double,12,12>& M = GetGlobalMassMatrix();

	if (!m_corotational)
	{
		const Static_Matrix<double,12,12>& K = GetLocalStiffnessMatrixInGlobalSpace();

		for (int i=0; i<4; i++)
		{
			int I = pointsID[i];

			for (int j=0; j<4; j++)
			{
				int J = pointsID[j];

				res[3*I  ] += coefK*K[3*i  ][3*j]*(u[3*J]) + coefK*K[3*i  ][3*j+1]*(u[3*J+1]) + coefK*K[3*i  ][3*j+2]*(u[3*J+2]);
				res[3*I+1] += coefK*K[3*i+1][3*j]*(u[3*J]) + coefK*K[3*i+1][3*j+1]*(u[3*J+1]) + coefK*K[3*i+1][3*j+2]*(u[3*J+2]);
				res[3*I+2] += coefK*K[3*i+2][3*j]*(u[3*J]) + coefK*K[3*i+2][3*j+1]*(u[3*J+1]) + coefK*K[3*i+2][3*j+2]*(u[3*J+2]);

				res[3*I  ] += coefM*M[3*i  ][3*j]*(u[3*J]) + coefM*M[3*i  ][3*j+1]*(u[3*J+1]) + coefM*M[3*i  ][3*j+2]*(u[3*J+2]);
				res[3*I+1] += coefM*M[3*i+1][3*j]*(u[3*J]) + coefM*M[3*i+1][3*j+1]*(u[3*J+1]) + coefM*M[3*i+1][3*j+2]*(u[3*J+2]);
				res[3*I+2] += coefM*M[3*i+2][3*j]*(u[3*J]) + coefM*M[3*i+2][3*j+1]*(u[3*J+1]) + coefM*M[3*i+2][3*j+2]*(u[3*J+2]);
			}
		}
	}
	else
	{
		const Static_Matrix<double,12,12>& K = GetGlobalStiffnessMatrix();

		for (int i=0; i<4; i++)
		{
			int I = pointsID[i];

			for (int j=0; j<4; j++)
			{
				int J = pointsID[j];

				res[3*I  ] += coefK*K[3*i  ][3*j]*(u[3*J]) + coefK*K[3*i  ][3*j+1]*(u[3*J+1]) + coefK*K[3*i  ][3*j+2]*(u[3*J+2]);
				res[3*I+1] += coefK*K[3*i+1][3*j]*(u[3*J]) + coefK*K[3*i+1][3*j+1]*(u[3*J+1]) + coefK*K[3*i+1][3*j+2]*(u[3*J+2]);
				res[3*I+2] += coefK*K[3*i+2][3*j]*(u[3*J]) + coefK*K[3*i+2][3*j+1]*(u[3*J+1]) + coefK*K[3*i+2][3*j+2]*(u[3*J+2]);

				res[3*I  ] += coefM*M[3*i  ][3*j]*(u[3*J]) + coefM*M[3*i  ][3*j+1]*(u[3*J+1]) + coefM*M[3*i  ][3*j+2]*(u[3*J+2]);
				res[3*I+1] += coefM*M[3*i+1][3*j]*(u[3*J]) + coefM*M[3*i+1][3*j+1]*(u[3*J+1]) + coefM*M[3*i+1][3*j+2]*(u[3*J+2]);
				res[3*I+2] += coefM*M[3*i+2][3*j]*(u[3*J]) + coefM*M[3*i+2][3*j+1]*(u[3*J+1]) + coefM*M[3*i+2][3*j+2]*(u[3*J+2]);
			}
		}
	}
}

// x, v & res general vectors (use internal indexing for accessing them)
void Tetrahedron::addMatVecProduct_MD_x_Mt_v(double dt, double rayleight_coef_mass, double rayleight_coef_stif,
                                             const double* x0, const double* xt, const double* ut, const double* vt, double* res)
{
	const double coefMx = (1.0/(dt*dt)) + (rayleight_coef_mass/dt);
	const double coefKx = rayleight_coef_stif/dt;
	const double coefMv = 1.0/dt;

	const Static_Matrix<double,12,12>& M  = GetGlobalMassMatrix();

	if (!m_corotational)
	{
		// Calculates a local contribution to the following calculation:
		//   res = res + MD * x + M/dt * v
		// where
		//   MD = (1/dt^2 + rayleight_coef_mass/dt) * M  +  (rayleight_coef_stif/dt) * K
		// i.e.
		//   res = res + ((1/dt^2 + rayleight_coef_mass/dt) * M  +  (rayleight_coef_stif/dt) * K) * x + M/dt * v
		const Static_Matrix<double,12,12>& K  = GetLocalStiffnessMatrixInGlobalSpace();

		for (int i=0; i<4; i++)
		{
			int I = pointsID[i];

			for (int j=0; j<4; j++)
			{
				int J = pointsID[j];

				res[3*I  ] += coefKx*K[3*i  ][3*j]*ut[3*J] + coefKx*K[3*i  ][3*j+1]*ut[3*J+1] + coefKx*K[3*i  ][3*j+2]*ut[3*J+2];
				res[3*I+1] += coefKx*K[3*i+1][3*j]*ut[3*J] + coefKx*K[3*i+1][3*j+1]*ut[3*J+1] + coefKx*K[3*i+1][3*j+2]*ut[3*J+2];
				res[3*I+2] += coefKx*K[3*i+2][3*j]*ut[3*J] + coefKx*K[3*i+2][3*j+1]*ut[3*J+1] + coefKx*K[3*i+2][3*j+2]*ut[3*J+2];

				res[3*I  ] += coefMx*M[3*i  ][3*j]*ut[3*J] + coefMx*M[3*i  ][3*j+1]*ut[3*J+1] + coefMx*M[3*i  ][3*j+2]*ut[3*J+2];
				res[3*I+1] += coefMx*M[3*i+1][3*j]*ut[3*J] + coefMx*M[3*i+1][3*j+1]*ut[3*J+1] + coefMx*M[3*i+1][3*j+2]*ut[3*J+2];
				res[3*I+2] += coefMx*M[3*i+2][3*j]*ut[3*J] + coefMx*M[3*i+2][3*j+1]*ut[3*J+1] + coefMx*M[3*i+2][3*j+2]*ut[3*J+2];

				res[3*I  ] += coefMv*M[3*i  ][3*j]*vt[3*J] + coefMv*M[3*i  ][3*j+1]*vt[3*J+1] + coefMv*M[3*i  ][3*j+2]*vt[3*J+2];
				res[3*I+1] += coefMv*M[3*i+1][3*j]*vt[3*J] + coefMv*M[3*i+1][3*j+1]*vt[3*J+1] + coefMv*M[3*i+1][3*j+2]*vt[3*J+2];
				res[3*I+2] += coefMv*M[3*i+2][3*j]*vt[3*J] + coefMv*M[3*i+2][3*j+1]*vt[3*J+1] + coefMv*M[3*i+2][3*j+2]*vt[3*J+2];
			}
		}
	}
	else
	{
		// Calculates the FEM corotational contribution to the RHS of the equation:
		// M d2q/dt2(t+dt)                       + D dq/dt(t+dt)       + RK(R^t.q(t+dt) - q0) = F
		// M (dq/dt(t+dt) - dq/dt(t))/dt         + D (q(t+dt)-q(t))/dt + RKR^t.(q(t+dt)-q(t)) = F + RK.q0 - RKR^t.q(t)
		// M (q(t+dt)-q(t))/dt^2 - M.dq/dt(t)/dt + D (q(t+dt)-q(t))/dt + RKR^t.(q(t+dt)-q(t)) = F + RK.q0 - RKR^t.q(t)
		// (M/dt^2 + D/dt + RKR^t).(q(t+dt)-q(t))                                             = F + RK.q0 - RKR^t.q(t) + M.dq/dt(t)
		// So it computes:
		//   res = res + RK.q0 - RKR^t.q(t) + M.v(t)
		//   Because of the updated Lagrangian approach, no Rayleigh damping parameters comes into place here, as they are sticking to the LHS.
		const Static_Matrix<double,12,12>& K  = GetGlobalStiffnessMatrix();
		const Static_Matrix<double,12,12>& RKe  = Get_RKeGlobal();

		for (int i=0; i<4; i++)
		{
			int I = pointsID[i];

			for (int j=0; j<4; j++)
			{
				int J = pointsID[j];

				//res[3*I  ] += RKe[3*i  ][3*j]*undeformedRotatedElement[j][0] + RKe[3*i  ][3*j+1]*undeformedRotatedElement[j][1] + RKe[3*i  ][3*j+2]*undeformedRotatedElement[j][2];
				//res[3*I+1] += RKe[3*i+1][3*j]*undeformedRotatedElement[j][0] + RKe[3*i+1][3*j+1]*undeformedRotatedElement[j][1] + RKe[3*i+1][3*j+2]*undeformedRotatedElement[j][2];
				//res[3*I+2] += RKe[3*i+2][3*j]*undeformedRotatedElement[j][0] + RKe[3*i+2][3*j+1]*undeformedRotatedElement[j][1] + RKe[3*i+2][3*j+2]*undeformedRotatedElement[j][2];
				res[3*I  ] += RKe[3*i  ][3*j]*x0[3*J] + RKe[3*i  ][3*j+1]*x0[3*J+1] + RKe[3*i  ][3*j+2]*x0[3*J+2];
				res[3*I+1] += RKe[3*i+1][3*j]*x0[3*J] + RKe[3*i+1][3*j+1]*x0[3*J+1] + RKe[3*i+1][3*j+2]*x0[3*J+2];
				res[3*I+2] += RKe[3*i+2][3*j]*x0[3*J] + RKe[3*i+2][3*j+1]*x0[3*J+1] + RKe[3*i+2][3*j+2]*x0[3*J+2];

				res[3*I  ] -= K[3*i  ][3*j]*xt[3*J  ] + K[3*i  ][3*j+1]*xt[3*J+1] + K[3*i  ][3*j+2]*xt[3*J+2];
				res[3*I+1] -= K[3*i+1][3*j]*xt[3*J  ] + K[3*i+1][3*j+1]*xt[3*J+1] + K[3*i+1][3*j+2]*xt[3*J+2];
				res[3*I+2] -= K[3*i+2][3*j]*xt[3*J  ] + K[3*i+2][3*j+1]*xt[3*J+1] + K[3*i+2][3*j+2]*xt[3*J+2];

				res[3*I  ] += coefMv*M[3*i  ][3*j]*vt[3*J  ] + coefMv*M[3*i  ][3*j+1]*vt[3*J+1] + coefMv*M[3*i  ][3*j+2]*vt[3*J+2];
				res[3*I+1] += coefMv*M[3*i+1][3*j]*vt[3*J  ] + coefMv*M[3*i+1][3*j+1]*vt[3*J+1] + coefMv*M[3*i+1][3*j+2]*vt[3*J+2];
				res[3*I+2] += coefMv*M[3*i+2][3*j]*vt[3*J  ] + coefMv*M[3*i+2][3*j+1]*vt[3*J+1] + coefMv*M[3*i+2][3*j+2]*vt[3*J+2];
			}
		}
	}
}

void Tetrahedron::ComputeChangeOfBasisMatrix(const Pt3D* undeformedMesh, const Pt3D* deformedMesh)
{
	// Using Muller & Gross 2004 paper idea...
	const double* A0 = undeformedMesh[ pointsID[0] ];
	const double* B0 = undeformedMesh[ pointsID[1] ];
	const double* C0 = undeformedMesh[ pointsID[2] ];
	const double* D0 = undeformedMesh[ pointsID[3] ];

	const double* A = deformedMesh[ pointsID[0] ];
	const double* B = deformedMesh[ pointsID[1] ];
	const double* C = deformedMesh[ pointsID[2] ];
	const double* D = deformedMesh[ pointsID[3] ];

	Static_Matrix<double,4,4> P; // Matrix in the undeformed configuration
	Static_Matrix<double,4,4> Q; // Matrix in the   deformed configuration
	for (int i=0 ; i<3 ; i++)
	{
		P[i][0]=A0[i];
		P[i][1]=B0[i];
		P[i][2]=C0[i];
		P[i][3]=D0[i];
		Q[i][0]=A[i];
		Q[i][1]=B[i];
		Q[i][2]=C[i];
		Q[i][3]=D[i];
	}
	P[3][0]=1.0;
	P[3][1]=1.0;
	P[3][2]=1.0;
	P[3][3]=1.0;
	Q[3][0]=1.0;
	Q[3][1]=1.0;
	Q[3][2]=1.0;
	Q[3][3]=1.0;

	Static_Matrix<double,4,4> Pinv;
	if (!inverseMatrix4x4fully(P.getPointer(),Pinv.getPointer()))
	{
		printf("Tetrahedron::ComputeChangeOfBasisMatrix could not inverse P \n");
	}

	Static_Matrix<double,4,4> QPinv; // Matrix for changing from undeformed configuration to deformed configuration (contains rotation, translation and stretching)
	multMatrix_nxm<double,4,4,4>(Q.getPointer(), Pinv.getPointer(), QPinv.getPointer());

	// Use a Polar decomposition on the 3x3 part to extract the rotation
	Static_Matrix<double,3,3> tetRotationStretching;
	Static_Matrix<double,3,3> R3x3 , S3x3;
	for (int i=0 ; i<3 ; i++)
		for (int j=0 ; j<3 ; j++)
		{
			tetRotationStretching[i][j] = QPinv[i][j];
		}

	polar_decomp<double,Static_Matrix<double,3,3> >(tetRotationStretching, R3x3, S3x3);

	SqMatrix33<double> localR(R3x3.getPointer(),true);
	setCurrentRotation(localR);


	//Static_Matrix<double,3,3> R;
	//mat_transpose(R3x3,3,3,R);
	//setCorotationalRotations(R);

	//// Check if the Orthogonal matrix R is actually a rotation matrix (i.e. in SO(3))
	//double detR = R[0][0]*R[1][1]*R[2][2] + R[0][1]*R[1][2]*R[2][0] + R[0][2]*R[1][0]*R[2][1] -
	//	R[2][0]*R[1][1]*R[0][2] - R[2][1]*R[1][2]*R[0][0] - R[2][2]*R[1][0]*R[0][1];
	//if(detR<0)
	//	printf("Tetrahedron::ComputeChangeOfBasisMatrix   Rotation matrix contains a reflection -> |R| = %g\n",detR);
	//if( fabs(detR-1.0)>1e-7 )
	//	printf("Tetrahedron::ComputeChangeOfBasisMatrix   |Rotation|-1.0 = %15.12lf\n",detR-1.0);







	//Static_Matrix<double,3,3> tetFrame; // Matrix in the undeformed configuration
	//tetFrame[0][0]=B[0]-A[0];  tetFrame[0][1]=B[1]-A[1];  tetFrame[0][2]=B[2]-A[2];
	//tetFrame[1][0]=C[0]-A[0];  tetFrame[1][1]=C[1]-A[1];  tetFrame[1][2]=C[2]-A[2];
	//tetFrame[2][0]=D[0]-A[0];  tetFrame[2][1]=D[1]-A[1];  tetFrame[2][2]=D[2]-A[2];

	//// Gramm-Schimdt
	//double u1[3],e1[3] , u2[3],e2[3] , u3[3],e3[3];
	//for(int axis=0;axis<3;axis++) u1[axis]=tetFrame[0][axis];
	//vec_generic_copy<double,double>(e1,u1,3);
	//vec_generic_normalize<double,double>(e1,3);

	//for(int axis=0;axis<3;axis++) u2[axis]=tetFrame[1][axis];
	//double distanceONe1 = vec_generic_dotProduct<double,double,double>(u2,e1,3);
	//for(int axis=0;axis<3;axis++) u2[axis]-=distanceONe1*e1[axis];	// We get rid of the e1 component to have an orthogonal frame in the end !
	//vec_generic_copy<double,double>(e2,u2,3);
	//vec_generic_normalize<double,double>(e2,3);

	//for(int axis=0;axis<3;axis++) u3[axis]=tetFrame[2][axis];
	//distanceONe1 = vec_generic_dotProduct<double,double,double>(u3,e1,3);
	//for(int axis=0;axis<3;axis++) u3[axis]-=distanceONe1*e1[axis];	// We get rid of the e1 component to have an orthogonal frame in the end !
	//double distanceONe2 = vec_generic_dotProduct<double,double,double>(u3,e2,3);
	//for(int axis=0;axis<3;axis++) u3[axis]-=distanceONe2*e2[axis];	// We get rid of the e2 component to have an orthogonal frame in the end !
	//vec_generic_copy<double,double>(e3,u3,3);
	//vec_generic_normalize<double,double>(e3,3);

	//// R is a rotation matrix transforming a global
	//for(int axis=0 ; axis<3 ; axis++)
	//{
	//	Rt[axis][0]=e1[axis];
	//	Rt[axis][1]=e2[axis];
	//	Rt[axis][2]=e3[axis];
	//	R[0][axis]=e1[axis];
	//	R[1][axis]=e2[axis];
	//	R[2][axis]=e3[axis];
	//	//R[axis][0]=(axis==0?1.0:0.0);//e1[axis];
	//	//R[axis][1]=(axis==1?1.0:0.0);//e2[axis];
	//	//R[axis][2]=(axis==2?1.0:0.0);//e3[axis];
	//	//Rt[0][axis]=(axis==0?1.0:0.0);//e1[axis];
	//	//Rt[1][axis]=(axis==1?1.0:0.0);//e2[axis];
	//	//Rt[2][axis]=(axis==2?1.0:0.0);//e3[axis];
	//}




	//// Polar decomposition
	//Static_Matrix<double,3,3> R3x3 , S3x3;
	//polar_decomp<double,Static_Matrix<double,3,3> >(tetFrame, R3x3, S3x3);

	//// We suppose that R and Rt has been nulled at the beginning
	//// => we just overwrite the useful terms...all other ones never being modified !
	//for(int line=0 ; line<3 ; line++)
	//	for(int col=0 ; col<3 ; col++)
	//	{
	//		R [line][col]=R3x3[line][col];
	//		Rt[line][col]=R3x3[col][line];
	//	}



	//Pt3D Xdir(0.,0.,0.);
	//Pt3D Ydir(0.,0.,0.);
	//Pt3D Zdir(0.,0.,0.);

	//// Compute the 3 direction point (X,Y,Z), X=(01) Y=(02) and Z=(03)
	//{
	//  double l;

	//  Xdir[0] = undeformedMesh[ pointsID[1] ][0] - undeformedMesh[ pointsID[0] ][0];
	//  Xdir[1] = undeformedMesh[ pointsID[1] ][1] - undeformedMesh[ pointsID[0] ][1];
	//  Xdir[2] = undeformedMesh[ pointsID[1] ][2] - undeformedMesh[ pointsID[0] ][2];
	//  l = sqrt(Xdir[0]*Xdir[0]+Xdir[1]*Xdir[1]+Xdir[2]*Xdir[2]);
	//  if(l<1e-5){ cerr << "Problem may occur, element direction X way too small for normalization l<1e-5" << endl; }
	//  Xdir[0]/=l; Xdir[1]/=l; Xdir[2]/=l;

	//  Zdir[0] = undeformedMesh[ pointsID[3] ][0] - undeformedMesh[ pointsID[0] ][0];
	//  Zdir[1] = undeformedMesh[ pointsID[3] ][1] - undeformedMesh[ pointsID[0] ][1];
	//  Zdir[2] = undeformedMesh[ pointsID[3] ][2] - undeformedMesh[ pointsID[0] ][2];
	//  l = sqrt(Zdir[0]*Zdir[0]+Zdir[1]*Zdir[1]+Zdir[2]*Zdir[2]);
	//  if(l<1e-5){ cerr << "Problem may occur, element direction Z way too small for normalization l<1e-5" << endl; }
	//  Zdir[0]/=l; Zdir[1]/=l; Zdir[2]/=l;

	//  Ydir[0] = Zdir[1]*Xdir[2] - Zdir[2]*Xdir[1];
	//  Ydir[1] = Zdir[2]*Xdir[0] - Zdir[0]*Xdir[2];
	//  Ydir[2] = Zdir[0]*Xdir[1] - Zdir[1]*Xdir[0];
	//  l = sqrt(Ydir[0]*Ydir[0]+Ydir[1]*Ydir[1]+Ydir[2]*Ydir[2]);
	//  if(l<1e-5){ cerr << "Problem may occur, element direction Y way too small for normalization l<1e-5" << endl; }
	//  Ydir[0]/=l; Ydir[1]/=l; Ydir[2]/=l;

	//  Zdir[0] = Xdir[1]*Ydir[2] - Xdir[2]*Ydir[1];
	//  Zdir[1] = Xdir[2]*Ydir[0] - Xdir[0]*Ydir[2];
	//  Zdir[2] = Xdir[0]*Ydir[1] - Xdir[1]*Ydir[0];
	//  l = sqrt(Zdir[0]*Zdir[0]+Zdir[1]*Zdir[1]+Zdir[2]*Zdir[2]);
	//  if(l<1e-5){ cerr << "Problem may occur, element direction Z way too small for normalization l<1e-5" << endl; }
	//  Zdir[0]/=l; Zdir[1]/=l; Zdir[2]/=l;
	//}

	//// Fill up the change of basis matrix, duplicating the 3x3 rotation matrix for each node !
	//mat_null<double>(R);
	//mat_null<double>(Rt);
	//for(int i=0 ; i<4 ; i++)
	//{
	//  R[3*i  ][3*i  ]=Xdir[0]; R[3*i  ][3*i+1]=Xdir[1]; R[3*i  ][3*i+2]=Xdir[2];
	//  R[3*i+1][3*i  ]=Ydir[0]; R[3*i+1][3*i+1]=Ydir[1]; R[3*i+1][3*i+2]=Ydir[2];
	//  R[3*i+2][3*i  ]=Zdir[0]; R[3*i+2][3*i+1]=Zdir[1]; R[3*i+2][3*i+2]=Zdir[2];

	//  Rt[3*i  ][3*i  ]=Xdir[0]; Rt[3*i  ][3*i+1]=Ydir[0]; Rt[3*i  ][3*i+2]=Zdir[0];
	//  Rt[3*i+1][3*i  ]=Xdir[1]; Rt[3*i+1][3*i+1]=Ydir[1]; Rt[3*i+1][3*i+2]=Zdir[1];
	//  Rt[3*i+2][3*i  ]=Xdir[2]; Rt[3*i+2][3*i+1]=Ydir[2]; Rt[3*i+2][3*i+2]=Zdir[2];
	//}
}

void Tetrahedron::computeR0(void)
{
	//const double *A0 = undeformedMesh[ pointsID[0] ];
	//const double *B0 = undeformedMesh[ pointsID[1] ];
	//const double *C0 = undeformedMesh[ pointsID[2] ];
	//const double *D0 = undeformedMesh[ pointsID[3] ];

	//Static_Matrix<double,3,3> tetFrame; // Matrix in the undeformed configuration
	//tetFrame[0][0]=B0[0]-A0[0];  tetFrame[0][1]=B0[1]-A0[1];  tetFrame[0][2]=B0[2]-A0[2];
	//tetFrame[1][0]=C0[0]-A0[0];  tetFrame[1][1]=C0[1]-A0[1];  tetFrame[1][2]=C0[2]-A0[2];
	//tetFrame[2][0]=D0[0]-A0[0];  tetFrame[2][1]=D0[1]-A0[1];  tetFrame[2][2]=D0[2]-A0[2];

	//// Polar decomposition
	//polar_decomp<double,Static_Matrix<double,3,3> >(tetFrame, R0, R0t);
	//mat_transpose(R0,3,3,R0t);

	mat_identity(R0 ,12,12);
	mat_identity(R0t,12,12);
	//for(int i=0 ; i<4 ; i++)
	//{
	//	for(int axis=0 ; axis<3 ; axis++)
	//	{
	//		undeformedRotatedElement[i][axis] = 0;
	//		for(int col=0 ; col<3 ; col++)
	//			undeformedRotatedElement[i][axis] += R0[axis][col]*undeformedMesh[ pointsID[i] ][col];
	//	}
	//}


	//computeShapeFunctionParameters(undeformedRotatedElement[0], undeformedRotatedElement[1], undeformedRotatedElement[2], undeformedRotatedElement[3]);
	//ComputeStiffnessMatrices();
}

void Tetrahedron::updateStressAndStrain(const Pt3D* mesh)
{
	calculateGreenStrain(mesh, m_greenStrain);

	if (m_greenStrain[0] < m_minEverGreenStrain)
	{
		m_minEverGreenStrain = m_greenStrain[0];
	}
	if (m_greenStrain[2] > m_maxEverGreenStrain)
	{
		m_maxEverGreenStrain = m_greenStrain[2];
	}
	for (int i = 0; i < 3; i++)
	{
		m_meanGreenStrain[i] += m_greenStrain[i];
		m_stdDevGreenStrain[i] += pow(m_greenStrain[i], 2.0);
	}

	calculateStress(mesh, m_stress);

	if (m_stress[0] < m_minEverStress)
	{
		m_minEverStress = m_stress[0];
	}
	if (m_stress[2] > m_maxEverStress)
	{
		m_maxEverStress = m_stress[2];
	}
	for (int i = 0; i < 3; i++)
	{
		m_meanStress[i] += m_stress[i];
		m_stdDevStress[i] += pow(m_stress[i], 2.0);
	}

	m_numStressStrainUpdates++;
}

void Tetrahedron::calculateGreenStrain(const Pt3D* mesh, double strain[3]) const
{
	// Green Strain
	double displacement_gradient_matrix[3][3];
	calculateDisplacementGradientMatrix(mesh, displacement_gradient_matrix);

	double green_strain_matrix[3][3];

	double displacement_gradient_matrix_transpose[3][3];
	mat_transpose(displacement_gradient_matrix, 3, 3, displacement_gradient_matrix_transpose);

	mat_mul(displacement_gradient_matrix_transpose, displacement_gradient_matrix, green_strain_matrix, 3, 3);

	mat_scale(green_strain_matrix, 0.5, 3, 3);

	double identity[3][3];
	mat_identity(identity, 3, 3);

	mat_sub(green_strain_matrix, identity, 3, 3);

	calculateEigenvalues(green_strain_matrix, strain);
}

void Tetrahedron::calculateStress(const Pt3D* mesh, double stress[3]) const
{
	// Green Strain should have been calculated first;

	for (int i = 0; i < 3; i++)
	{
		stress[i] = E * m_greenStrain[i];
	}
}

void Tetrahedron::calculateEigenvalues(const double symmetricMatrix[3][3], double sortedLambda[3]) const
{
	double x11 = symmetricMatrix[0][0];
	double x22 = symmetricMatrix[1][1];
	double x33 = symmetricMatrix[2][2];
	double x21 = symmetricMatrix[1][0];
	double x31 = symmetricMatrix[2][0];
	double x32 = symmetricMatrix[2][1];

	double m = (x11 + x22 + x33) / 3.0;
	double a11 = x11 - m, a22 = x22 - m, a33 = x33 - m, a12_sqr = x21 * x21, a13_sqr = x31 * x31, a23_sqr = x32 * x32;
	double p = (a11 * a11 + a22 * a22 + a33 * a33 + 2 * (a12_sqr + a13_sqr + a23_sqr)) / 6.0;
	double q = 0.5 * (a11 * (a22 * a33 - a23_sqr) - a22 * a13_sqr - a33 * a12_sqr) + x21 * x31 * x32;
	double sqrt_p = sqrt(p), disc = p * p * p - q * q;
	double phi = atan2(sqrt(max(0.0,disc)),q) / 3.0, c = cos(phi), s = sin(phi);
	double sqrt_p_cos = sqrt_p * c, root_three_sqrt_p_sin = sqrt(3.0) * sqrt_p * s;

	double lambda[3] = { m + 2.0 * sqrt_p_cos,
	                     m-sqrt_p_cos - root_three_sqrt_p_sin,
	                     m-sqrt_p_cos + root_three_sqrt_p_sin
	                   };

	// Sort lambda from least to greatest.
	if (lambda[0] < lambda[1])
	{
		if (lambda[0] < lambda[2])
		{
			if (lambda[1] < lambda[2])
			{
				sortedLambda[0] = lambda[0];
				sortedLambda[1] = lambda[1];
				sortedLambda[2] = lambda[2];
			}
			else
			{
				sortedLambda[0] = lambda[0];
				sortedLambda[1] = lambda[2];
				sortedLambda[2] = lambda[1];
			}
		}
		else
		{
			sortedLambda[0] = lambda[2];
			sortedLambda[1] = lambda[0];
			sortedLambda[2] = lambda[1];
		}
	}
	else
	{
		if (lambda[1] < lambda[2])
		{
			if (lambda[0] < lambda[2])
			{
				sortedLambda[0] = lambda[1];
				sortedLambda[1] = lambda[0];
				sortedLambda[2] = lambda[2];
			}
			else
			{
				sortedLambda[0] = lambda[1];
				sortedLambda[1] = lambda[2];
				sortedLambda[2] = lambda[0];
			}
		}
		else
		{
			sortedLambda[0] = lambda[2];
			sortedLambda[1] = lambda[1];
			sortedLambda[2] = lambda[0];
		}
	}
}

void Tetrahedron::calculateDisplacementGradientMatrix(const Pt3D* pts, double displacementGradientMatrix[3][3]) const
{
	double Ds[3][3];

	Ds[0][0] = pts[pointsID[1]][0] - pts[pointsID[0]][0];
	Ds[0][1] = pts[pointsID[2]][0] - pts[pointsID[0]][0];
	Ds[0][2] = pts[pointsID[3]][0] - pts[pointsID[0]][0];

	Ds[1][0] = pts[pointsID[1]][1] - pts[pointsID[0]][1];
	Ds[1][1] = pts[pointsID[2]][1] - pts[pointsID[0]][1];
	Ds[1][2] = pts[pointsID[3]][1] - pts[pointsID[0]][1];

	Ds[2][0] = pts[pointsID[1]][2] - pts[pointsID[0]][2];
	Ds[2][1] = pts[pointsID[2]][2] - pts[pointsID[0]][2];
	Ds[2][2] = pts[pointsID[3]][2] - pts[pointsID[0]][2];

	mat_mul(Ds, invJ, displacementGradientMatrix, 3, 3);
}
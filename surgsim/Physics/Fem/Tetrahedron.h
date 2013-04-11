#ifndef __tetrahedron_h__
#define __tetrahedron_h__

#include <limits>
#undef max
#undef min

#include "element.h"

#include "TOOLS/Matrix/hardcoded_inverse.h"

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

class Tetrahedron:public Element<4,3>
{
private:
	Static_Matrix<double,6,12> B;     // Strain matrix dNi/dx,y,z
	Static_Matrix<double,12,6> Bt;    // Transpose of B
	Static_Matrix<double,6,6> Em;      // Stress Matrix (contains the material propertie, Young modulus, poison ratio...)

	Static_Matrix<double,3,3> J,Jt;   // Matrix to go from parametric position to full 3D geometric position
	Static_Matrix<double,3,3> invJ;   // Matrix to go from global 3D space to parametrization
	double detJ;                      // Determinant(J)

	// Base function = Ni(x,y,z) = 1/6V ( ai + x.bi + y.ci + z.di )
	double inv_6V,ai[4],bi[4],ci[4],di[4];

	double m_greenStrain[3];
	double m_meanGreenStrain[3];
	double m_stdDevGreenStrain[3];
	double m_minEverGreenStrain;
	double m_maxEverGreenStrain;

	double m_stress[3];
	double m_meanStress[3];
	double m_stdDevStress[3];
	double m_minEverStress;
	double m_maxEverStress;

	size_t m_numStressStrainUpdates;

	void computeShapeFunctionParameters(const Pt3D &A, const Pt3D &B, const Pt3D &C, const Pt3D &D)
	{
		Pt3D a(A);//undeformedMesh[pointsID[0]][0] , undeformedMesh[pointsID[0]][1] , undeformedMesh[pointsID[0]][2]);
		Pt3D b(B);//undeformedMesh[pointsID[1]][0] , undeformedMesh[pointsID[1]][1] , undeformedMesh[pointsID[1]][2]);
		Pt3D c(C);//undeformedMesh[pointsID[2]][0] , undeformedMesh[pointsID[2]][1] , undeformedMesh[pointsID[2]][2]);
		Pt3D d(D);//undeformedMesh[pointsID[3]][0] , undeformedMesh[pointsID[3]][1] , undeformedMesh[pointsID[3]][2]);
		double tmp,tmpa,tmpb,tmpc,tmpd;

		//cout << "a=("<<a[0]<<" , "<<a[1]<<" , "<<a[2]<<")"<<endl;
		//cout << "b=("<<b[0]<<" , "<<b[1]<<" , "<<b[2]<<")"<<endl;
		//cout << "c=("<<c[0]<<" , "<<c[1]<<" , "<<c[2]<<")"<<endl;
		//cout << "d=("<<d[0]<<" , "<<d[1]<<" , "<<d[2]<<")"<<endl;

		inv_6V = 1.0/_6Volume(undeformedMesh);

		ai[0] =  ( det( b , c , d ) );
		ai[1] = -( det( a , c , d ) );
		ai[2] =  ( det( a , b , d ) );
		ai[3] = -( det( a , b , c ) );

		// Save the 'x' component of the 4 vertices
		tmpa=a[0]; tmpb=b[0]; tmpc=c[0]; tmpd=d[0];

		// And replace them by '1'
		a[0]=1.; b[0]=1.; c[0]=1.; d[0]=1.;

		// HERE WE HAVE for example with BCD:
		// | 1 yb zb |
		// | 1 yc zc |
		// | 1 yd zd |
		bi[0] = -( det( b , c , d ) );
		bi[1] =  ( det( a , c , d ) );
		bi[2] = -( det( a , b , d ) );
		bi[3] =  ( det( a , b , c ) );

		// Save the 'y' component of the 4 vertices and replace it with the 'x' one which was stored in tmpx values
		tmp=a[1]; a[1]=tmpa; tmpa=tmp;
		tmp=b[1]; b[1]=tmpb; tmpb=tmp;
		tmp=c[1]; c[1]=tmpc; tmpc=tmp;
		tmp=d[1]; d[1]=tmpd; tmpd=tmp;
		// HERE WE HAVE for example with BCD:
		// | 1 xb zb |
		// | 1 xc zc |
		// | 1 xd zd |
		ci[0] =  ( det( b , c , d ) );
		ci[1] = -( det( a , c , d ) );
		ci[2] =  ( det( a , b , d ) );
		ci[3] = -( det( a , b , c ) );

		// Replace the 'z' component of the 4 vertices with the 'y' component stores in the tmpx values
		// No need to save the 'z' component as it won't be used any further !
		// HERE WE HAVE for example with BCD:
		// | 1 xb yb |
		// | 1 xc yc |
		// | 1 xd yd |
		a[2]=tmpa;
		b[2]=tmpb;
		c[2]=tmpc;
		d[2]=tmpd;
		di[0] = -( det( b , c , d ) );
		di[1] =  ( det( a , c , d ) );
		di[2] = -( det( a , b , d ) );
		di[3] =  ( det( a , b , c ) );
	};

	inline double det(Pt3D a, Pt3D b, Pt3D c) const
	{
		return 
			a[0]*b[1]*c[2] + a[2]*b[0]*c[1] + a[1]*b[2]*c[0]
		- a[2]*b[1]*c[0] - a[1]*b[0]*c[2] - a[0]*b[2]*c[1];
	};

	//protected:
public:
	// Shape Function
	double f(int i,double x,double y,double z);
	double dfdx(int i,double x,double y,double z);
	double dfdy(int i,double x,double y,double z);
	double dfdz(int i,double x,double y,double z);

	// Compute initial rotation R0
	virtual void computeR0(void);
	// Compute relative rotation R (from R0 to the current rigid transformation)...for corotational approach
	virtual void ComputeChangeOfBasisMatrix(const Pt3D *undeformedMesh, const Pt3D *deformedMesh);

public:
	Tetrahedron():Element<4,3>()
	{
		for (int i = 0; i < 3; i++)
		{
			m_greenStrain[i] = 0.0;
			m_meanGreenStrain[i] = 0.0;
			m_stdDevGreenStrain[i] = 0.0;

			m_stress[i] = 0.0;
			m_meanStress[i] = 0.0;
			m_stdDevStress[i] = 0.0;	
		}

		m_minEverGreenStrain = std::numeric_limits<double>::max();
		m_maxEverGreenStrain = -std::numeric_limits<double>::max();

		m_minEverStress = std::numeric_limits<double>::max();
		m_maxEverStress = -std::numeric_limits<double>::max();

		m_numStressStrainUpdates = 0;
	};
	Tetrahedron(Pt3D *pts, int i, int j, int k, int l):Element<4,3>(pts)
	{
		// Make sure that the vertex i j k are counter clock wise from the point of view of l !
		// Therefore, the following relation must be verified:
		//  li^lj = normal of face lij pointing outward
		//  => (li^lj).lk < 0 !!
		Pt3D li(pts[i][0]-pts[l][0] , pts[i][1]-pts[l][1] , pts[i][2]-pts[l][2]);
		Pt3D lj(pts[j][0]-pts[l][0] , pts[j][1]-pts[l][1] , pts[j][2]-pts[l][2]);
		Pt3D lk(pts[k][0]-pts[l][0] , pts[k][1]-pts[l][1] , pts[k][2]-pts[l][2]);
		Pt3D li_lj(li[1]*lj[2]-li[2]*lj[1] , li[2]*lj[0]-li[0]*lj[2] , li[0]*lj[1]-li[1]*lj[0]);

		//double orientation = li_lj[0]*lk[0] + li_lj[1]*lk[1] + li_lj[2]*lk[2];
		//if( fabs(orientation)<1e-18 )
		//  cerr << " Tetrahedron illed< NO > ";

		if( li_lj[0]*lk[0] + li_lj[1]*lk[1] + li_lj[2]*lk[2] < 0. )
		{ pointsID[0]=i;  pointsID[1]=j;  pointsID[2]=k;  pointsID[3]=l; }
		else
		{ pointsID[0]=i;  pointsID[1]=k;  pointsID[2]=j;  pointsID[3]=l; }

		Jt[0][0]=J[0][0]=pts[pointsID[1]][0]-pts[pointsID[0]][0];
		Jt[1][0]=J[0][1]=pts[pointsID[2]][0]-pts[pointsID[0]][0];
		Jt[2][0]=J[0][2]=pts[pointsID[3]][0]-pts[pointsID[0]][0];

		Jt[0][1]=J[1][0]=pts[pointsID[1]][1]-pts[pointsID[0]][1];
		Jt[1][1]=J[1][1]=pts[pointsID[2]][1]-pts[pointsID[0]][1];
		Jt[2][1]=J[1][2]=pts[pointsID[3]][1]-pts[pointsID[0]][1];

		Jt[0][2]=J[2][0]=pts[pointsID[1]][2]-pts[pointsID[0]][2];
		Jt[1][2]=J[2][1]=pts[pointsID[2]][2]-pts[pointsID[0]][2];
		Jt[2][2]=J[2][2]=pts[pointsID[3]][2]-pts[pointsID[0]][2];

		detJ = J[0][0]*J[1][1]*J[2][2] + J[1][0]*J[2][1]*J[0][2] + J[2][0]*J[0][1]*J[1][2] -
			J[2][0]*J[1][1]*J[0][2] - J[1][0]*J[0][1]*J[2][2] - J[0][0]*J[2][1]*J[1][2];

		if(detJ!=0.)
		{
			double I = 1.0/detJ;

			// J-1 = 1/det(J) coMat( J^T )
			invJ[0][0] =   I*(J[1][1]*J[2][2] - J[2][1]*J[1][2]);
			invJ[1][0] = - I*(J[1][0]*J[2][2] - J[2][0]*J[1][2]);
			invJ[2][0] =   I*(J[1][0]*J[2][1] - J[2][0]*J[1][1]);

			invJ[0][1] = - I*(J[0][1]*J[2][2] - J[2][1]*J[0][2]);
			invJ[1][1] =   I*(J[0][0]*J[2][2] - J[2][0]*J[0][2]);
			invJ[2][1] = - I*(J[0][0]*J[2][1] - J[2][0]*J[0][1]);

			invJ[0][2] =   I*(J[0][1]*J[1][2] - J[1][1]*J[0][2]);
			invJ[1][2] = - I*(J[0][0]*J[1][2] - J[1][0]*J[0][2]);
			invJ[2][2] =   I*(J[0][0]*J[1][1] - J[0][1]*J[1][0]);
		}
		else
		{
			cerr << " Tetrahedron illed conditionned => matrix J singular" << endl;
			cerr << "J=" << endl << J << endl;
		}

		ComputeChangeOfBasisMatrix(pts,pts);
		computeShapeFunctionParameters( pts[pointsID[0]], pts[pointsID[1]], pts[pointsID[2]], pts[pointsID[3]] );
		ComputeStiffnessMatrices();

		for (int i = 0; i < 3; i++)
		{
			m_greenStrain[i] = 0.0;
			m_meanGreenStrain[i] = 0.0;
			m_stdDevGreenStrain[i] = 0.0;

			m_stress[i] = 0.0;
			m_meanStress[i] = 0.0;
			m_stdDevStress[i] = 0.0;
		}

		m_minEverGreenStrain = std::numeric_limits<double>::max();
		m_maxEverGreenStrain = -std::numeric_limits<double>::max();

		m_minEverStress = std::numeric_limits<double>::max();
		m_maxEverStress = -std::numeric_limits<double>::max();

		m_numStressStrainUpdates = 0;

		updateStressAndStrain(pts);
		computeR0();
	};
	~Tetrahedron(){};

	void setPoints(Pt3D *pts ,int i,int j,int k,int l)
	{
		undeformedMesh=pts;

		// Make sure that the vertex i j k are counter clock wise from the point of view of l !
		// Therefore, the following relation must be verified:
		//  li^lj = normal of face lij pointing outward
		//  => (li^lj).lk < 0 !!
		Pt3D li(pts[i][0]-pts[l][0] , pts[i][1]-pts[l][1] , pts[i][2]-pts[l][2]);
		Pt3D lj(pts[j][0]-pts[l][0] , pts[j][1]-pts[l][1] , pts[j][2]-pts[l][2]);
		Pt3D lk(pts[k][0]-pts[l][0] , pts[k][1]-pts[l][1] , pts[k][2]-pts[l][2]);
		Pt3D li_lj(li[1]*lj[2]-li[2]*lj[1] , li[2]*lj[0]-li[0]*lj[2] , li[0]*lj[1]-li[1]*lj[0]);
		if( li_lj[0]*lk[0] + li_lj[1]*lk[1] + li_lj[2]*lk[2] < 0. )
		{ pointsID[0]=i;  pointsID[1]=j;  pointsID[2]=k;  pointsID[3]=l; }
		else
		{ pointsID[0]=i;  pointsID[1]=k;  pointsID[2]=j;  pointsID[3]=l; }

		Jt[0][0]=J[0][0]=pts[pointsID[1]][0]-pts[pointsID[0]][0];
		Jt[1][0]=J[0][1]=pts[pointsID[2]][0]-pts[pointsID[0]][0];
		Jt[2][0]=J[0][2]=pts[pointsID[3]][0]-pts[pointsID[0]][0];

		Jt[0][1]=J[1][0]=pts[pointsID[1]][1]-pts[pointsID[0]][1];
		Jt[1][1]=J[1][1]=pts[pointsID[2]][1]-pts[pointsID[0]][1];
		Jt[2][1]=J[1][2]=pts[pointsID[3]][1]-pts[pointsID[0]][1];

		Jt[0][2]=J[2][0]=pts[pointsID[1]][2]-pts[pointsID[0]][2];
		Jt[1][2]=J[2][1]=pts[pointsID[2]][2]-pts[pointsID[0]][2];
		Jt[2][2]=J[2][2]=pts[pointsID[3]][2]-pts[pointsID[0]][2];

		detJ = J[0][0]*J[1][1]*J[2][2] + J[1][0]*J[2][1]*J[0][2] + J[2][0]*J[0][1]*J[1][2] -
			J[2][0]*J[1][1]*J[0][2] - J[1][0]*J[0][1]*J[2][2] - J[0][0]*J[2][1]*J[1][2];

		if(detJ!=0.)
		{
			double I = 1.0/detJ;

			// J-1 = 1/det(J) coMat( J^T )
			invJ[0][0] =   I*(J[1][1]*J[2][2] - J[2][1]*J[1][2]);
			invJ[1][0] = - I*(J[1][0]*J[2][2] - J[2][0]*J[1][2]);
			invJ[2][0] =   I*(J[1][0]*J[2][1] - J[2][0]*J[1][1]);

			invJ[0][1] = - I*(J[0][1]*J[2][2] - J[2][1]*J[0][2]);
			invJ[1][1] =   I*(J[0][0]*J[2][2] - J[2][0]*J[0][2]);
			invJ[2][1] = - I*(J[0][0]*J[2][1] - J[2][0]*J[0][1]);

			invJ[0][2] =   I*(J[0][1]*J[1][2] - J[1][1]*J[0][2]);
			invJ[1][2] = - I*(J[0][0]*J[1][2] - J[1][0]*J[0][2]);
			invJ[2][2] =   I*(J[0][0]*J[1][1] - J[0][1]*J[1][0]);
		}
		else
		{
			cerr << " Tetrahedron failure => matrix J singular (none valid tetrahedron)" << endl;
			cerr << "J=" << endl << J << endl;
		}

		ComputeChangeOfBasisMatrix(pts,pts);
		computeShapeFunctionParameters(pts[pointsID[0]] , pts[pointsID[1]] , pts[pointsID[2]] , pts[pointsID[3]]);
		ComputeStiffnessMatrices();

		for (int i = 0; i < 3; i++)
		{
			m_greenStrain[i] = 0.0;
			m_meanGreenStrain[i] = 0.0;
			m_stdDevGreenStrain[i] = 0.0;

			m_stress[i] = 0.0;
			m_meanStress[i] = 0.0;
			m_stdDevStress[i] = 0.0;	
		}

		m_minEverGreenStrain = std::numeric_limits<double>::max();
		m_maxEverGreenStrain = -std::numeric_limits<double>::max();

		m_minEverStress = std::numeric_limits<double>::max();
		m_maxEverStress = -std::numeric_limits<double>::max();

		m_numStressStrainUpdates = 0;

		updateStressAndStrain(pts);
		computeR0();
	};

	//void setCorotational(void){ m_corotational=true; computeR0(); }; // If corotational, compute R0 rotation matrix and adjust the Strain Tensor with the adjusted initial element.
	void setNonCorotational(void) // If non corotational, recompute the Shape function and the stiffness with the original data (switching from corotational might have modify that !!)
	{
		computeShapeFunctionParameters(undeformedMesh[pointsID[0]], undeformedMesh[pointsID[1]], undeformedMesh[pointsID[2]], undeformedMesh[pointsID[3]]);
		ComputeStiffnessMatrices();
	};

	virtual void ComputeStiffnessMatrices(void);
	virtual void UpdateGlobalStiffnessMatrix(const Pt3D *undeformedMesh, const Pt3D *deformedMesh);

	// x & res general vector (use internal indexing for accessing them)
	virtual void MatVecProduct_K_u(const double* u, const double* x, const double* x0, double* result);

	// x & res general vector (use internal indexing for accessing them)
	virtual void MatVecProduct_MDK_u(double dt, double rayleight_coef_mass, double rayleight_coef_stif, const double* u, 
		const double* x, const double* x0, double* result);

	// x, v & res general vectors (use internal indexing for accessing them)
	virtual void addMatVecProduct_MD_x_Mt_v(double dt, double rayleight_coef_mass, double rayleight_coef_stif,
		const double *x0, const double *xt, const double *ut, const double *vt, double *result);

	static enum StressStrainMode { 
		GREEN_STRAIN_I=0, 
		GREEN_STRAIN_J, 
		GREEN_STRAIN_K, 
		TOTAL_GREEN_STRAIN,
		STRESS_I, 
		STRESS_J, 
		STRESS_K, 
		TOTAL_STRESS,
		NUM_STRESS_STRAIN_MODES};

	double getStressStrainValue(StressStrainMode mode) const
	{
		double strain[3];
		getGreenStrain(strain);

		double stress[3];
		getStress(stress);

		switch (mode)
		{
		case GREEN_STRAIN_I: 
			return strain[0];
		case GREEN_STRAIN_J: 
			return strain[1];
		case GREEN_STRAIN_K:
			return strain[2];
		case TOTAL_GREEN_STRAIN:
			return sqrt(pow(strain[0], 2.0) + pow(strain[1], 2.0) + pow(strain[2], 2.0));
		case STRESS_I: 
			return stress[0];
		case STRESS_J: 
			return stress[1];
		case STRESS_K:
			return stress[2];
		case TOTAL_STRESS:
			return sqrt(pow(stress[0], 2.0) + pow(stress[1], 2.0) + pow(stress[2], 2.0));
		default:
			return 0.0;
		}
	}

	void getGreenStrain(double strain[3]) const
	{ 
		for (int i = 0; i < 3; i++) 
			strain[i] = m_greenStrain[i]; 
	}
	void getMeanGreenStrain(double meanStrain[3]) const
	{ 
		for (int i = 0; i < 3; i++) 
			meanStrain[i] = m_meanGreenStrain[i] / m_numStressStrainUpdates; 
	}
	void getStdDevGreenStrain(double stdDevStrain[3]) const
	{ 
		double meanStrain[3];
		getMeanGreenStrain(meanStrain);
		for (int i = 0; i < 3; i++) 
			stdDevStrain[i] = sqrt(m_stdDevGreenStrain[i] / m_numStressStrainUpdates - pow(meanStrain[i], 2.0)); 
	}
	double getMinimumEverGreenStrain() const
	{
		return m_minEverGreenStrain;
	}
	double getMaximumEverGreenStrain() const
	{
		return m_maxEverGreenStrain;
	}
	double getLargestEverGreenStrain() const
	{
		if (abs(m_maxEverGreenStrain) > abs(m_minEverGreenStrain))
		{
			return m_maxEverGreenStrain;
		}
		else
		{
			return m_minEverGreenStrain;
		}
	}

	void getStress(double stress[3]) const
	{ 
		for (int i = 0; i < 3; i++) 
			stress[i] = m_stress[i]; 
	}
	void getMeanStress(double meanStress[3]) const
	{ 
		for (int i = 0; i < 3; i++) 
			meanStress[i] = m_meanStress[i] / m_numStressStrainUpdates; 
	}
	void getStdDevStress(double stdDevStress[3]) const
	{ 
		double meanStress[3];
		getMeanStress(meanStress);
		for (int i = 0; i < 3; i++) 
			stdDevStress[i] = sqrt(m_stdDevStress[i] / m_numStressStrainUpdates - pow(meanStress[i], 2.0)); 
	}
	double getMinimumEverStress() const
	{
		return m_minEverStress;
	}
	double getMaximumEverStress() const
	{
		return m_maxEverStress;
	}
	double getLargestEverStress() const
	{
		if (abs(m_maxEverStress) > abs(m_minEverStress))
		{
			return m_maxEverStress;
		}
		else
		{
			return m_minEverStress;
		}
	}

	void updateStressAndStrain(const Pt3D* mesh);
	void calculateGreenStrain(const Pt3D* mesh, double strain[3]) const;
	void calculateStress(const Pt3D* mesh, double stress[3]) const;

	void calculateDisplacementGradientMatrix(const Pt3D* mesh, double displacementGradientMatrix[3][3]) const;
	void calculateEigenvalues(const double symmetricMatrix[3][3], double sortedLambda[3]) const;

	double getInitialVolume(void) const { return VolumeInitial(); };
	double getCurrentVolume(Pt3D *deformedMesh) const { return Volume(deformedMesh); };

	void ComputeMassMatrix()
	{
		mat_null<double>(MeLocal);
		massPerNode = getInitialVolume()*volumetricMass / (double)4.0;

		for(register unsigned int i=0;i<4;i++)
		{
			MeLocal[_X_(i)][_X_(i)] = massPerNode;
			MeLocal[_Y_(i)][_Y_(i)] = massPerNode;
			MeLocal[_Z_(i)][_Z_(i)] = massPerNode;
		}

		//###########################################
		// Transformation Local -> Global
		Static_Matrix<double,12,12> tmp;
		mat_mul(R0,MeLocal,tmp,12,12,12);
		mat_mul(tmp,R0t,MeGlobal,12,12,12);

		//###########################################
		// Set the relative trasnformation for the corotational model to Identity
		// This set the rotation R, Rt
		// but also the matrices RK, RKRt, RMRt
		SqMatrix33<double> Id33;
		Id33.setIdentity();
		setCurrentRotation(Id33);
	};

	// Tetrahedron volume = 1/6 * | 1 x0 y0 z0 |
	//                            | 1 x1 y1 z1 |
	//                            | 1 x2 y2 z2 |
	//                            | 1 x3 y3 z3 |
	inline double _6Volume(const Pt3D* mesh) const
	{
		// fabs is necessary if we don't pay attention to the indexing !
		// If the tetrahedron verify ABC counter clock wise viewed from D, this determinant is always positive = 6V
		// Otherwise, it can happen that this determinant is negative = -6V !!
		return
			( det(mesh[ pointsID[1] ],mesh[ pointsID[2] ],mesh[ pointsID[3] ])
			- det(mesh[ pointsID[0] ],mesh[ pointsID[2] ],mesh[ pointsID[3] ])
			+ det(mesh[ pointsID[0] ],mesh[ pointsID[1] ],mesh[ pointsID[3] ])
			- det(mesh[ pointsID[0] ],mesh[ pointsID[1] ],mesh[ pointsID[2] ]));
	};
	inline double VolumeInitial(void)              const { return 1./6. * _6Volume(undeformedMesh); };
	inline double Volume(const Pt3D *deformedMesh) const { return 1./6. * _6Volume(deformedMesh);   };

	inline void calculateBarycentricCoordinates(const Pt3D* mesh, const Pt3D point, double tetBaryCoord[4]) const
	{
		double T[9];

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				T[3 * i + j] = mesh[pointsID[j]][i] - mesh[pointsID[3]][i];
			}
		}

		double Tinv[9];
		inverseMatrix3x3(T, Tinv);

		Pt3D point_minus_tetPoint4;
		for (int i = 0; i < 3; i++)
		{
			point_minus_tetPoint4[i] = point[i] - mesh[pointsID[3]][i];
		}

		for (int i = 0; i < 3; i++)
		{
			tetBaryCoord[i] = 0.0;
			for (int j = 0; j < 3; j++)
			{
				tetBaryCoord[i] += Tinv[3 * i + j] * point_minus_tetPoint4[j];
			}
		}

		tetBaryCoord[3] = 1.0 - (tetBaryCoord[0] + tetBaryCoord[1] + tetBaryCoord[2]);
	}

};

#endif

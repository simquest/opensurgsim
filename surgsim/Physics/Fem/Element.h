// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_PHYSICS_FEM_ELEMENT_H
#define SURGSIM_PHYSICS_FEM_ELEMENT_H

#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

template <int nbNode,int nbDOFPerNode> class Element
{
protected:
	long ID;

	// Mechanic
	double E,nu;      // Young modulus & Poisson ratio
	double lambda,mu; // Lame coefficient

	// Dynamic
	double volumetricMass;

protected:
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> MeLocal;      // Element mass matrix
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> MeGlobal;		// Element stiffness matrix (in global coordinates, i.e. rotated by R0 only)
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> RMeGlobalRt;	// Element stiffness matrix (in the global coordinates) updated with rotation matrix R

	// Stiffness matrix in the local frame
	// Note: For FEM3D          , the local frame is equal       to the global frame R0==Identity (because the shape functions are directly expressed in global space)
	//       For FEM1D and FEM2D, the local frame is different from the global frame R0!=Identity
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> KeLocal;		// Element stiffness matrix (in its local coordinates)
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> KeGlobal;		// Element stiffness matrix (in global coordinates, i.e. rotated by R0 only)
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> RKeGlobalRt;	// Element stiffness matrix (in the global coordinates) updated with rotation matrix R
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> RKeGlobal;	// Element stiffness matrix to transform the initial force into the rotated frame

	// Initial rotation to transform the local stiffness matrix into the actual initial orientation.
	SqMatrix33<double> R03x3, R0t3x3;
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> R0;  //LocalTOGlobal at t=0
	Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> R0t; //GlobalTOLocal at t=0

protected:

public:  //!!! Public?  Really??? --bert
	int pointsID[nbNode];      // List of points ID defining the elements
protected:


	// The internal structure is only known by the daughter class
	Pt3D *undeformedMesh; // List of points (external), passed on to the constructor !
	// useful for the NON muller and Gross co-rotational method !

	double massPerNode;   // Distribution of mass for each node

	void fillUp3x3_KeLocal(int i,int j,double a11,double a12,double a13,double a21,double a22,double a23,double a31,double a32,double a33)
	{
		KeLocal[_X_(i)][_X_(j)]=a11;  KeLocal[_X_(i)][_Y_(j)]=a12;  KeLocal[_X_(i)][_Z_(j)]=a13;
		KeLocal[_Y_(i)][_X_(j)]=a21;  KeLocal[_Y_(i)][_Y_(j)]=a22;  KeLocal[_Y_(i)][_Z_(j)]=a23;
		KeLocal[_Z_(i)][_X_(j)]=a31;  KeLocal[_Z_(i)][_Y_(j)]=a32;  KeLocal[_Z_(i)][_Z_(j)]=a33;
	}

	// Shape Function
	virtual double f(int i,double x,double y,double z) = 0;
	virtual double dfdx(int i,double x,double y,double z) = 0;
	virtual double dfdy(int i,double x,double y,double z) = 0;
	virtual double dfdz(int i,double x,double y,double z) = 0;

	// Compute initial rotation R0
	virtual void computeR0() = 0;
	// Compute relative rotation R (from R0 to the current rigid transformation)...for corotational approach
	virtual void ComputeChangeOfBasisMatrix(const Pt3D *undeformedMesh, const Pt3D *deformedMesh) = 0;

public:
	Element()          : ID(-1), undeformedMesh(0) {};
	Element(Pt3D *pts) : ID(-1), undeformedMesh(pts) {};

	void setID(unsigned long _ID){ ID=_ID; };
	unsigned long getID() const { return ID; };

	


	void setInitialRotation(const SqMatrix33<double>& R0) { R03x3=R0; R0t3x3=R0.transpose(); };
	
protected:
	// Use the current R3x3 etc. to update the full-DOF rotation matrices, R and Rt, as well as the resulting RKeGlobal and RKeGlobalRt.
	void computeFullDofRotation()
	{
		mat_null<double>(R  , nbNode*nbDOFPerNode,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode);
		//mat_null<double>(Rt , nbNode*nbDOFPerNode,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode);

		const SqMatrix33d& elementRotation = getCurrentRotation();

		// Duplicate the rotation matrix for each node of the element
		for(int nodeID=0 ; nodeID<nbNode ; nodeID++)
		{
			// Translational (always in 3D)
			for(int line=0 ; line<3 ; line++)
			{
				for(int col=0 ; col<3 ; col++)
				{
					R [nodeID*nbDOFPerNode + line][nodeID*nbDOFPerNode + col] = elementRotation(line,col);
				}
			}

			// For old, global formulation of rotation DOFs, the node rotations always equal the element rotation.
			// For body-relative formulation of rotation DOFs, these will be individual rotations-- one per node.  Crazy, isn't it?
			const SqMatrix33d& nodeRotation = getCurrentNodeRotation(nodeID);

			// Rotational (2D or 3D)
			for(int line=3 ; line<nbDOFPerNode ; line++)
			{
				for(int col=3 ; col<nbDOFPerNode ; col++)
				{
					R [nodeID*nbDOFPerNode + line][nodeID*nbDOFPerNode + col] = nodeRotation(line-3,col-3);
				}
			}
		}
		mat_transpose(R,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode,Rt);

		// Update the global mass matrix for the corotational approach
		mat_mul(        R, MeGlobal,   RKeGlobal, nbNode*nbDOFPerNode,nbNode*nbDOFPerNode);
		mat_mul(RKeGlobal,       Rt, RMeGlobalRt, nbNode*nbDOFPerNode,nbNode*nbDOFPerNode);

		// Update the global stiffness matrix for the corotational approach
		mat_mul(        R, KeGlobal,   RKeGlobal, nbNode*nbDOFPerNode,nbNode*nbDOFPerNode);
		mat_mul(RKeGlobal,       Rt, RKeGlobalRt, nbNode*nbDOFPerNode,nbNode*nbDOFPerNode);
	};

public:

	const SqMatrix33<double>& getInitialRotation() const { return R03x3; };
	const SqMatrix33<double>& getInitialRotationTranspose() const { return R0t3x3; };
	const SqMatrix33<double>& getCurrentRotation() const { return R3x3; };
	const SqMatrix33<double>& getCurrentRotationTranspose() const { return Rt3x3; };

	const SqMatrix33<double>& getCurrentNodeRotation(int nodeIndex) const
	{
		SQ_ASSERT(nodeIndex >= 0 && nodeIndex < nbNode, "bad node index");
		// If we're using body-relative rotation DOFs, each of the nodes gets a separate rotation from the array.  (Crazy, no?)
		// If we're using global (not body-relative) rotation DOFs, all node rotations equal the element rotation.
		return R3x3;
	}

	void setMechanicalParameters_lambda_mu(double _lambda, double _mu)
	{ lambda=_lambda; mu=_mu; E=mu*(3.*lambda+2.*mu)/(lambda+mu); nu=lambda/(2.*lambda+2.*mu); };

	void setMechanicalParameters_E_nu(double _E, double _nu)
	{ E=_E; nu=_nu; lambda=E*nu/((1.+nu)*(1.-2.*nu)); mu=E/(2.+2.*nu); };

	void setVolumetricMass(double volMass){ volumetricMass=volMass; };
	double getVolumetricMass(void) const { return volumetricMass; };

	virtual double getInitialVolume(void) const = 0;
	virtual double getCurrentVolume(Pt3D *deformedMesh) const = 0;

	void setYoungModulus(double _E){ E=_E; };
	void setPoissonRatio(double _nu){ nu=_nu; };
	double getYoungModulus(void) const { return E; };
	double getPoissonRatio(void) const { return nu; };

	const int getPointID(int i) const{ return pointsID[i]; };
	int &getPointID(int i) { return pointsID[i]; };

	bool containsPointID(int id) const
	{
		for (int i = 0;  i < nbNode;  ++i)
		{
			if (pointsID[i] == id)
			{
				return true;
			}
		}
		return false;
	}

	void setUndeformedMesh(Pt3D *pts){ undeformedMesh=pts; };
	// This is useful if the pointer get modified after a realloc for example !
	const Pt3D* getUndeformedMesh() const { return undeformedMesh; };
	const Pt3D& getUndeformedPoint(int i) const { return undeformedMesh[ pointsID[i] ]; };

	void calculateEmbeddedLocation(const Pt3D* mesh, const double* weights, Pt3D& embeddedPoint) const
	{
		embeddedPoint[0] = weights[0] * mesh[ pointsID[0] ][0];
		embeddedPoint[1] = weights[0] * mesh[ pointsID[0] ][1];
		embeddedPoint[2] = weights[0] * mesh[ pointsID[0] ][2];
		for (int i = 1;  i < nbNode;  ++i)
		{
			embeddedPoint[0] += weights[i] * mesh[ pointsID[i] ][0];
			embeddedPoint[1] += weights[i] * mesh[ pointsID[i] ][1];
			embeddedPoint[2] += weights[i] * mesh[ pointsID[i] ][2];
		}
	}
	void calculateUndeformedEmbeddedLocation(const double* weights, Pt3D& embeddedPoint) const
	{
		calculateEmbeddedLocation(getUndeformedMesh(), weights, embeddedPoint);
	}

	virtual void ComputeMassMatrix(void) = 0;
	virtual void ComputeStiffnessMatrices(void) = 0;

	const Static_Matrix<double,nbDOFPerNode*nbNode,nbDOFPerNode*nbNode>& GetLocalMassMatrix() const { return MeLocal; };
	const Static_Matrix<double,nbDOFPerNode*nbNode,nbDOFPerNode*nbNode>& GetLocalMassMatrixInGlobalSpace() const { return MeGlobal; };
	const Static_Matrix<double,nbDOFPerNode*nbNode,nbDOFPerNode*nbNode>& GetGlobalMassMatrix() const { if(!m_corotational) return MeGlobal; return RMeGlobalRt; };

	const Static_Matrix<double,nbDOFPerNode*nbNode,nbDOFPerNode*nbNode>& GetLocalStiffnessMatrix() const { return KeLocal; };
	const Static_Matrix<double,nbDOFPerNode*nbNode,nbDOFPerNode*nbNode>& GetLocalStiffnessMatrixInGlobalSpace() const { return KeGlobal; };
	const Static_Matrix<double,nbDOFPerNode*nbNode,nbDOFPerNode*nbNode>& GetGlobalStiffnessMatrix() const { if(!m_corotational) return KeGlobal; return RKeGlobalRt; };
	
	const Static_Matrix<double,nbDOFPerNode*nbNode,nbDOFPerNode*nbNode>& Get_RKeGlobal() const { return RKeGlobal; };
	const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode>& Get_R0() const { return R0; };
	const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode>& Get_R0t() const { return R0t; };
	const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode>& Get_R() const { return R; };
	const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode>& Get_Rt() const { return Rt; };

	//###############################################
	//### Stress/Strain measurement
	virtual void computeCurrentStressStrain(const double uniformAbs[3],const double* xt_nD, const double* x0_nD, double strain[6], double stress[6]){};
	virtual void computeCurrentStressStrain(const double uniformAbs[3],const Pt3D* deformedMesh, const Pt3D* undeformedMesh, double strain[6], double stress[6]){};
	//virtual void computeCurrentStressStrain(const Pt3D* deformedMesh, const Pt3D* undeformedMesh){};
	//###############################################

	// x & res general vector (use internal indexing for accessing them)
	virtual void MatVecProduct_K_u(const double* u, const double* x, const double* x0, double* result)
	{		
		const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> &Ke = GetGlobalStiffnessMatrix();

		for (int ptI = 0; ptI < nbNode; ptI++)
		{
			int nodeI = pointsID[ptI];

			for (int ptJ = 0; ptJ < nbNode; ptJ++)
			{
				int nodeJ = pointsID[ptJ];

				for (int i = 0; i < nbDOFPerNode; i++)
				{
					for (int j = 0; j < nbDOFPerNode; j++)
					{
						result[nbDOFPerNode*nodeI + i] += Ke[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j] * u[nbDOFPerNode*nodeJ + j];
					}
				}
			}
		}
	}

	// x & res general vector (use internal indexing for accessing them)
	virtual void MatVecProduct_MDK_u(double dt, double rayleight_coef_mass, double rayleight_coef_stif, const double* u, 
		const double* x, const double* x0, double* result)
	{
		double coefK = (1.0            +  rayleight_coef_stif / dt);
		double coefM = (1.0 / (dt*dt)) + (rayleight_coef_mass / dt);

		const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> &M = GetGlobalMassMatrix();

		const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> &K = GetGlobalStiffnessMatrix();

		for(int ptI = 0; ptI < nbNode; ptI++)
		{
			int nodeI = pointsID[ptI];

			for(int ptJ = 0; ptJ < nbNode; ptJ++)
			{
				int nodeJ = pointsID[ptJ];

				for (int i = 0; i < nbDOFPerNode; i++)
				{
					for (int j = 0; j < nbDOFPerNode; j++)
					{
						result[nbDOFPerNode*nodeI + i] += coefK * K[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j] * u[nbDOFPerNode*nodeJ + j];
						result[nbDOFPerNode*nodeI + i] += coefM * M[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j] * u[nbDOFPerNode*nodeJ + j];
					}
				}
			}
		}
	}

	// x, v & res general vectors (use internal indexing for accessing them)
	virtual void addMatVecProduct_MD_x_Mt_v(double dt, double rayleight_coef_mass, double rayleight_coef_stif,
		const double *x0, const double *xt, const double *ut, const double *vt, double *result)
	{
		const double coefMx = (1.0 / (dt*dt)) + (rayleight_coef_mass / dt);
		const double coefKx = rayleight_coef_stif / dt;
		const double coefMv = 1.0 / dt;

		const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> &M  = GetGlobalMassMatrix();

		// Calculates a local contribution to the following calculation:
		//   res = res + MD * x + M/dt * v
		// where
		//   MD = (1/dt^2 + rayleight_coef_mass/dt) * M  +  (rayleight_coef_stif/dt) * K
		// i.e.
		//   res = res + ((1/dt^2 + rayleight_coef_mass/dt) * M  +  (rayleight_coef_stif/dt) * K) * x + M/dt * v
		const Static_Matrix<double,nbNode*nbDOFPerNode,nbNode*nbDOFPerNode> &K  = GetGlobalStiffnessMatrix();

		for(int ptI = 0; ptI < nbNode; ptI++)
		{
			int nodeI = pointsID[ptI];

			for(int ptJ = 0; ptJ < nbNode; ptJ++)
			{
				int nodeJ = pointsID[ptJ];

				for (int i = 0; i < nbDOFPerNode; i++)
				{
					for (int j = 0; j < nbDOFPerNode; j++)
					{
						result[nbDOFPerNode*nodeI + i] += coefKx * K[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j] * ut[nbDOFPerNode*nodeJ + j];
						result[nbDOFPerNode*nodeI + i] += coefMx * M[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j] * ut[nbDOFPerNode*nodeJ + j];
						result[nbDOFPerNode*nodeI + i] += coefMv * M[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j] * vt[nbDOFPerNode*nodeJ + j];
					}
				}
			}
		}
	}

};

#endif

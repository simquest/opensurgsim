#ifndef SURGSIM_PHYSICS_FEM_FEM_H
#define SURGSIM_PHYSICS_FEM_FEM_H

#include <limits>
#include <map>
#include <set>
#include "Tetrahedron.h"

#include "Fem.h"

class 3dFem : public Fem<4, 3>
{
	friend ostream& operator << (ostream &o, 3dFem &fem);
	friend istream& operator >> (istream &i, 3dFem &fem);

public:
	3dFem(double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP);

	3dFem(int nbPts, Pt3D* pts, double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP);

	3dFem(int nbPts, Pt3D* pts, int nbSurfacePts, Pt3D* surfacePts, double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP);

	~3dFem();

	void allocate();

	void applyCut(std::string cutFilename);

	const Eigen::MatrixXd& getInitialComplianceMatrix();
	virtual const Eigen::MatrixXd& getComplianceMatrix();

	void addTetrahedron(Tetrahedron* tet);
	int getNbTetrahedron() const;
	const Tetrahedron* getTetrahedron(int i) const;
	int findTetrahedronWithNodes(int idA, int idB = -1, int idC = -1, int idD = -1) const;
	
	void addSurfaceTriangle(const int ptIds[3], const int tetId, const double tetWeights[3][4]);
	int getNbSurfaceTriangles() const;
	int getNbSurfacePoints() const;
	void getSurfaceTriangle(const int triID, int ptID[3]) const;
	void getSurfaceTriangleTetWeights(const int triID, int& tetID, double tetWeights[3][4]) const;
	void getSurfacePointTetWeights(const int ptID, int& tetID, double tetWeights[4]) const;

	int getTetContainingPoint(const Pt3D* mesh, const Pt3D point, double tetBaryCoord[4]) const;

	//! Computes the total volume of the FEM
	double computeVolume() const;

	//! Assembles the global stiffness matrix (K)
	/*!
	Necessary for using LU decomposition (should not be necessary for using CG, but it is)
	*/
	virtual void assembleK();

	//! Performs dynamic resolution using LU
	/*!
	Solve (M/dt + D/dt + K).u = F + + (M/dt2 + D/dt) x(t) + M/dt v(t) using LU decomposition
	*/
	virtual void solve_BackwardEuler_directSolver();

	//! Performs dynamic resolution using CG
	/*!
	Solve (M/dt + D/dt + K).u = F + + (M/dt2 + D/dt) x(t) + M/dt v(t) using Conjugate Gradient
	*/
	virtual void solve_BackwardEuler_CG();

	//! Prepares for static resolution using LU
	/*!
	Prepares data to solve K.U=F using LU decomposition
	Assemble_K() should be called before.
	*/
	virtual void prepare_Static_directSolver();

	//! Performs static resolution using LU
	/*!
	Solves K.U=F using LU decomposition
	*/
	virtual void solve_Static_directSolver();

	//! Performs static resolution using CG
	/*!
	Solves K.U=F for a given F, and a starting guess of U, using Conjugate Gradient
	\param useGlobalMatrix Whether to use the global stiffness matrix
	*/
	virtual void solve_StaticCG(bool useGlobalMatrix = false);

	//! Performs one step of the solver
	virtual void solveOneStep();

	void clampSurfacePoints(const Pt3D* volumeMesh, Pt3D* surfaceMesh) const;
	void clampDeformedSurfacePoints();

	void resizeSurfaceMesh(int numPoints);
	void setLastDeformedSurfaceMesh(const Pt3D* points);
	const Pt3D* getLastDeformedSurfaceMesh() const;
	const Pt3D* getDeformedSurfaceMesh() const;
	const Pt3D* getUndeformedSurfaceMesh() const;

	void buildSurfaceTrisFromTets();

	void mapTriToTet(const int triID, const double triBaryCoord[3], int& tetID, double tetBaryCoord[4]) const;

	void updateStressStrains();
	double getMaximumStressStrainValue(Tetrahedron::StressStrainMode mode) const;
	double getMinimumStressStrainValue(Tetrahedron::StressStrainMode mode) const;

	//! Sets nodes to compute warped compliance matrix (C) for
	/*!
	\param nodes Nodes to compute
	*/
	void setConstrainedNodes(const std::set<int>& nodes); 
	//! Computes warped compliance matrix (C) for co-rotational
	void computePartialWarpedComplianceMatrix();
	//! Gets the warped compliance matrix (C) if co-rotational, else normal C
	const Dynamic_Matrix<double>& getComplianceMatrixForLCP();

	int isThisNodeDuplicated(int nodeID);

private:
	// If the tet is bound to a node that has been duplicated and the tet is on the right side of the cut
	//  it will be now pointing to the duplicate node instead of the initial one !
	void updateTetForCut( const Tetrahedron *t , double *cutPlaneNormal);

protected:
	void addRHS_BackwardEuler(const double dt , const double *xt, const double *vt, double *F);

	//! Calculates product of the global stiffness matrix (K) and current global displacement vector (u).
	/*!
	\param u Global displacement vector (size nbDOFPerNode * m_nbNodes)
	\param x Current deformed node positions (size m_nbNodes)
	\param x0 Undeformed node positions (size m_nbNodes)
	\param result Resulting product (K * u)
	\param useGlobalMatrix Whether to use the pre-calculated global stiffness matrix K, otherwise the produce is computed without building K.
	This is used by the Static Conjugate Gradient solver.
	*/
	virtual void MatVecProduct_K_u(const double* u, const double* x, const double* x0, double* result, bool useGlobalMatrix);

	//! Calculates the product MD * K * u without building the matrices.
	/*!
	MD = (1/(dt*dt) + mu)*M + (lambda/dt)*K, 
	dt is the timestep, 
	mu is the mass proportional Rayleigh damping coefficient, 
	lambda is the stiffness proportional Rayleigh damping coefficient, 
	M is the mass matrix, 
	K is the global stiffness matrix, 
	u is the current global displacement matrix
	\param dt Time step
	\param rayleighDampingMassCoefficient Mass proportional Rayleigh damping coefficient
	\param rayleighDampingStiffnessCoefficient Stiffness proportional Rayleigh damping coefficient
	\param u Global displacement vector (size nbDOFPerNode * m_nbNodes)
	\param x Current deformed node positions (size m_nbNodes)
	\param x0 Undeformed node positions (size m_nbNodes)
	\param result Resulting product (K * u)
	This is used by the Dynamic Conjugate Gradient solver.
	*/
	virtual void MatVecProduct_MDK_u(double dt, double rayleighDampingMassCoefficient, double rayleighDampingStiffnessCoefficient,
		const double* u, const double* x, const double* x0, double* result);

	//! Updates the mesh point positions with the current node positions (xt)
	virtual void updateDeformedMesh();	
};

ostream& operator <<(ostream &o, FEMTetrahedra &fem);
istream& operator >>(istream &i, FEMTetrahedra &fem);

#endif

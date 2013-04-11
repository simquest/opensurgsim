#ifndef __FEM_h__
#define __FEM_h__

#include <string>
#include <map>

#include <Math/SqMKL/SqMklSparseMatrix.h>
#include <Math/Algebra/SqVec3.h>
#include <Math/Algebra/SqMatrix33.h>
#include <Math/Algebra/SqQuaternion.h>
#include <Math/Algebra/SqTransform.h>

#include "element.h"

/*!
  FEM class usage:
  1) Initialization
	{ FEM(dt,...) + setPts(nbPts,pts)        OR         FEM(nbPts,pts,...)
	{                 |                                  |     
	{                 + set [m_deformedMesh, m_undeformedMesh, m_lastDeformedMesh]
	{
	{ + in any order:
	{ setBC_VirtualMass(const double BC_virtualMass, const double BC_virtualInvMass);
	{ setTimeStep(double dt);
	{ setRayleightDamping(double massCoefficient, double stiffnessCoefficient);
	{ setCGParameters(int maxIteration, double epsilon);
	{ addBoundaryCondition(int nodeID);
	{ addElement(FEMElement* element);
	{
	{ + after addElement()
	{ setCorotational or setNonCorotational (non corotational by default)
	{
	{ + after addElement
	{ InitializeAfterLoad()
	{ |           
	{ + call allocate()
	{ + call initialize(nbPts,pts)
	{   |
	{   + set [m_x0, m_xt]
	{   + all co-rotational related data
	{
	{ + in any order:
	{ Assemble_K();
	{ Assemble_M();
	{
	{ + at the end:
	{ prepareSolver() -> Necessary for linear model as this is the only time the entire system will be build and optimized for solving
	{                    Not necessary for co-rotational model as the system has to be rebuilt all the time, prior to solving

  2) Runtime
	{ solveOneStep()
*/

class SutureInteractions_FEM1D;

//! Base FEM class
template <int nbNodePerElement, int nbDOFPerNode> 
class Fem
{
	friend class SutureInteractions_FEM1D;
protected:
	//! Allocate memories, vectors, matrices...using m_nbNodes
	virtual void allocate(void);

	//! Initialize the model and all co-rotational data
	/*!
	Virtual method has it depends on the dimension 1D, 2D, 3D for the corotational part
	Common part: m_xt and m_x0 are initialized here
	If re-written by daughter class, this mother method should be called anyway to set m_x0 and m_xt !
	*/
	virtual void initialize(int nbPts, Pt3D *pts){ initialize(nbPts, &pts[0][0] , sizeof(Pt3D)); };
	virtual void initialize(int nbPts, SqVec3d *pts){ initialize(nbPts, &pts[0][0] , sizeof(SqVec3d)); };
	virtual void initialize(int nbPts, double *pts, int byteOffset);

	//! preOneStep (pre treatment for co-rotational mode)
	//! After this method, all system matrix should be ready to be used (K, MD, MDK) and
	//! Any stiffness correction terms due to co-rotational should have been added to the RHS.
	//! NOTE : LU decomposition, TriDiagBlockMatrix, etc... will be using these matrices.
	//! NOTE2: Corrective terms due to Backward Euler will be added later.
	virtual void preOneStep(void){};

	//! postOneStep (post treatment for co-rotational mode)
	//! Recompute frames (nodes and beams)
	//! Do not update the beam element stiffness or internal rotation R (yet !)
	//! -> this should be done in preOneStep() in the next iteration using the frames computed by this method
	virtual void postOneStep(double *U, double *V)
	{
		// Clamp any value in m_Ut and m_Vt that is smaller than 1e-15 to 0 (this helps MKL to not struggle !).
		for(int i=0 ; i<getNbDOF() ; i++)
		{
			if( U[i]<1e-15 && U[i]>-1e-15 )
				U[i] = 0.0;

			if( V[i]<1e-15 && V[i]>-1e-15 )
				V[i] = 0.0;
		}
	};

public:
	//! FEM element type
	typedef Element<nbNodePerElement, nbDOFPerNode> FEMElement;

	//! Constructs an FEM with no nodes or elements
	/*!
	\param dt Time step
	\param usesPrecomputedDecomposition Whether LU decomposition is used, otherwise CG is used
	\param isDynamic Whether dynamic resolution (Backward Euler) is used, otherwise static is used
	\param isUsingMLCP Whether MLCP will be used to resolve constraints
	*/
	Fem(double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP);

	//! Constructs an FEM with the given points, but no elements
	/*!
	\param nbPts Number of points in the FEM
	\param pts Point positions
	\param dt Time step
	\param usesPrecomputedDecomposition Whether LU decomposition is used, otherwise CG is used
	\param isDynamic Whether dynamic resolution (Backward Euler) is used, otherwise static is used
	\param isUsingMLCP Whether MLCP will be used to resolve constraints
	*/
	Fem(int nbPts, Pt3D* pts, double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP);

	//! Destructor
	virtual ~Fem();

	void reset()
	{
		// Initialize, init x0, xt, as well as beamTransformation, nodeTransformation
		initialize(m_nbNodes, &m_undeformedMesh[0][0], 3*sizeof(double));

		// Reset the deformed meshes
		for(int nodeID=0 ; nodeID<m_nbNodes ; nodeID++)
		{
			m_deformedMesh[nodeID] = m_undeformedMesh[nodeID];
			m_lastDeformedMesh[nodeID] = m_undeformedMesh[nodeID];
		}


		// Reset the simulation data m_xt,...
		for(int dof=0 ; dof<m_nbNodes*nbDOFPerNode ; dof++)
		{
			m_xt[dof] = m_x0[dof];
			m_Vt[dof] = 0.0;
			m_Ut[dof] = 0.0;
			m_Ut_minus_dt[dof] = 0.0;
			m_Uc_scaled[dof] = 0.0; // Just to be sure

			m_F[dof]  = 0.0;        // Just to be sure
			m_FwithBC[dof] = 0.0;   // Just to be sure
		}
	}

	/// Gets whether the simulation is dynamic (Backward Euler)
	/// \return True if dynamic, otherwise false
	virtual bool isDynamic() const;
	/// Gets whether the simulation is static (no velocity effects)
	/// \return True if static, otherwise false
	virtual bool isStatic() const;

	/// Gets whether the simulation uses precomputed LU decomposition (for direct solution)
	/// \return True if using precomputed decomposition, otherwise false
	virtual bool usesPrecomputedDecomposition() const;
	/// Gets whether the simulation uses conjugate gradient (iterative)
	/// \return True if using conjugate gradient, otherwise false
	virtual bool usesConjugateGradient() const;

	/// Gets whether the simulation uses MLCP (Mixed Linear Complementarity Problem)
	/// \return True if using MLCP, otherwise false
	/// If using MLCP the compliance matrix is computed.
	virtual bool usesMLCP() const;

	/// Gets whether the simulation computes the compliance matrix (C)
	/// \return True if computes compliance matrix, otherwise false
	/// The compliance matrix is computed for precomputed decomposition and MLCP.
	virtual bool computesComplianceMatrix() const;

	/// Gets whether the simulation needs the full MD and MDK matrices (for Rayleigh Damping with Backward Euler) are needed
	/// \return True if the full MD and MDK matrices are needed
	virtual bool simulationNeedsFullMDx() const;
	/// Gets whether the simulation needs the full global stiffness matrix (K)
	/// \return True if the full K is needed
	virtual bool simulationNeedsFullK() const;
	/// Gets whether the simulation needs the full mass matrix (M)
	/// \return True if the full M is needed
	virtual bool simulationNeedsFullM() const;

	/// Gets the number of nodes in the FEM mesh
	/// \return Number of nodes
	int getNbNodes() const;
	/// Gets the undeformed FEM node positions
	/// \return Position of each node
	const Pt3D* getUndeformedMesh() const;
	/// Gets the deformed FEM node positions
	/// \return Position of each node
	const Pt3D* getDeformedMesh() const;

	//! Gets the number of FEM elements
	/*!
	\return Number of elements
	*/
	unsigned int getNbElements() const;
	//! Gets an FEM element
	/*!
	\param Element ID
	\return FEM element
	*/
	const std::weak_ptr<FEMElement> getElement(int i) const;

	//! Gets the number of boundary conditions (fixed nodes)
	/*!
	\return Number of boundary conditions
	*/
	int getNbBC() const;
	//! Gets a boundary condition (fixed node ID)
	/*!
	\param i Boundary condition ID
	\return Boundary condition node ID
	*/
	int getBC(int i) const;

	//! Gets the total number of degrees of freedom
	/*!
	\return Number of DOF
	*/
	int getNbDOF() const;

	//! Sets the Boundary Conditions virtual mass
	/*!
	\param BC_virtualMass    = the value that will be set at the diagonal element of the system's matrix (K or MDK).
	\param BC_virtualInvMass = the value that will be set at the diagonal element of the system's compliance matrix (inverse of the system's matrix).
	*/
	virtual void setBC_VirtualMass(const double BC_virtualMass, const double BC_virtualInvMass);
	
	//! Sets the simulation time step
	/*!
	\param dt Time step
	*/
	virtual void setTimeStep(double dt);

	//! Sets the Rayleigh damping parameters
	/*!
	\param massCoefficient Mass proportional Rayleigh damping coefficient
	\param stiffnessCoefficient Stiffness proportional Rayleigh damping coefficient
	*/
	virtual void setRayleightDamping(double massCoefficient, double stiffnessCoefficient);

	//! Sets the Conjugate Gradient parameters
	/*!
	\param maxIteration Maximum number of iterations
	\param epsilon Convergence epsilon
	*/
	virtual void setCGParameters(int maxIteration, double epsilon);

	//! Sets the mesh node positions
	/*!
	\param nbPts Number of mesh nodes
	\param pts Mesh node positions
	*/
	virtual void setPts(int nbPts, Pt3D* pts);

	//! Adds a boundary condition (fixed node)
	/*!
	\param nodeID Node to fix
	*/
	virtual void addBoundaryCondition(int nodeID);

	//! Adds an FEM element
	/*!
	\param element FEM element to add
	*/
	virtual void addElement(FEMElement* element);

	//! Gets the mass on an FEM node
	/*!
	\param nodeID Index of node
	*/
	virtual double getMassOnNode(int nodeID) const;

	//! Resets external forces to 0
	virtual void resetExternalForces();
	//! Adds an external gravity force on all nodes
	/*!
	\param gravity Gravity force
	*/
	virtual void addGravity(const double gravity[3]);
	//! Adds an external gravity force on all nodes
	/*!
	\param gravity Gravity force
	*/
	virtual void addGravity_nD(const double gravity[nbDOFPerNode]);
	//! Resets external forces to a gravity force on all nodes
	/*!
	\param gravity Gravity force
	*/
	virtual void resetExternalForces_WithGravity(const double gravity[3]);
	//! Resets external forces to a gravity force on all nodes
	/*!
	\param gravity Gravity force
	*/
	virtual void resetExternalForces_WithGravity_nD(const double gravity[nbDOFPerNode]);
	//! Adds external force on a specific node
	/*!
	\param force Force
	\param nodeID Index of node
	*/
	virtual void addExternalForceOnNode(const double force[nbDOFPerNode], int nodeID);

	//! Applies a rigid transformation to the initial mesh
	/*!
	\param translation Translation to apply
	\param quaternion Rotation to apply
	This should be called before adding elements (so that the shape functions are computed correctly).
	*/
	virtual void applyRigidTransformToInitialMesh(SqVec3d translation, SqQuaterniond quaternion);

	//! Removes any unused nodes from the mesh.
	/*!
	\return Number of unused nodes, which were removed
	*/
	virtual int removeUnusedMeshNodes();

	//! Initializes the FEM after the mesh is loaded.
	/*!
	  allocate matrices, vectors, etc...
	  Initialize data (corotational, ...)
	*/
	virtual void initializeAfterLoad();

	//! Gets the compliance matrix
	/*!
	\return Compliance matrix (size m_nbNodes*nbDOFPerNode x m_nbNodes*nbDOFPerNode)
	*/
	const Dynamic_Matrix<double>& getComplianceMatrix();

	//! Gets the current global displacement of a node from its initial position
	/*!
	\param nodeID Node index
	\return Displacement of node (size nbDOFPerNode)
	*/
	const double* getDisplacement(int nodeID) const;
	//! Gets the current global displacement vector
	/*!
	\return Displacements (size m_nbNodes*nbDOFPerNode)
	*/
	const double* getDisplacements() const;
	
	//! Assembles the mass matrix (M)
	/*!
	Necessary for using LU decomposition (should not be necessary for using CG, but it is)
	*/
	virtual void Assemble_M();
	//! Assembles the global stiffness matrix (K)
	/*!
	Necessary for using LU decomposition (should not be necessary for using CG, but it is)
	*/
	virtual void Assemble_K();
	//! Modifies the global stiffness matrix (K) for the boundary conditions (fixed nodes)
	/*!
	The rows and columns of the fixed nodes are set to 0.
	Necessary for using LU decomposition (should not be necessary for using CG, but it is)
	*/
	virtual void Modify_K_withBC();



	//! Prepares the linear system for resolution using MKL, TriDiagBlockMatrix, ...
	/*!
	Suppose the system matrix is stored in M_K_LU, no matter what model we have (Static, Dynamic) (Linear, Co-rotational)
	*/
	virtual void directSolver_prepare(void);

	//! Solve the linear system using MKL, TriDiagBlockMatrix, ...
	/*!
	This method use the output of directSolver_prepare to solve the linear system.
	1) Backup m_Ut into m_Ut_minus_1
	2) Solve the system directSolver_prepare[m_K_LU] . m_Ut = m_F
	3) m_F unchanged ; m_Ut contains the solution
	*/
	virtual void directSolver_solve(void);

	//! Compute the compliance matrix using MKL, TriDiagBlockMatrix, ...
	/*!
	This method use the output of directSolver_prepare to accelerate the inverse matrix computation.
	*/
	virtual void directSolver_computeCompliance(void);


	//! Prepares for dynamic resolution using either LU or CG
	/*!
	Prepare Matrix MDK and MD to solve (M/dt + D/dt + K).u = F + + (M/dt2 + D/dt) x(t) + M/dt v(t)
	Assemble_M(double) and Assemble_K() should be called before.
	*/
	virtual void prepare_BackwardEuler_Matrices();
	//! Prepares dynamic system to be solved using a direct Solver (MKL LU decomposition, TriDiagBlock preocessing,...)
	/*!
	Prepare LU decomposition to solve (M/dt + D/dt + K).u = F + + (M/dt2 + D/dt) x(t) + M/dt v(t)
	Assemble_M(double) and Assemble_K() should be called before.
	prepare_BackwardEuler_Matrices() should be called before
	*/
	virtual void prepare_BackwardEuler_directSolver();
	//! Prepares for dynamic resolution using LU
	/*!
	Solve (M/dt + D/dt + K).u = F + + (M/dt2 + D/dt) x(t) + M/dt v(t) using LU decomposition, TriDiagBlockMatrix, ...
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

	//! Prepares the solver after the necessary parameters and geometry have been set
	virtual void prepareSolver();

	//! Performs one step of the solver
	virtual void solveOneStep();

	//! Adds resulting displacements due to constraints
	/*!
	\param offset Offset into the displacement vector to the start of this entity's degrees of freedom
	\param Uc Displacement vector (size total number of degrees of freedom = nDOFPerNode * m_nbNodes)
	\param scale Scale to apply to displacement
	*/
	virtual void addConstraintDisplacement(const int offset, const double* Uc, double scale);

	//! Sets the previous FEM node positions
	/*!
	\return Position of each node
	*/
	virtual void setLastDeformedMesh(const Pt3D* points);
	//! Gets the previous FEM node positions
	/*!
	\return Position of each node
	*/
	const Pt3D* getLastDeformedMesh() const;

	//! Compute the current stress/strain for each elements
	void computeCurrentStressStrain(const int eID, const double abs[3], double strain[6], double stress[6])
	{ m_elements[eID]->computeCurrentStressStrain(abs,m_xt.getPointer(),m_x0.getPointer(),strain,stress); }

protected:
	//! Verbosity of convergence output, -1 to disable
	static int m_convergenceVerbosity;

	//! Number of nodes in the mesh
	int m_nbNodes;
	//! Whether this FEM owns the undeformed mesh node array, and should handle its deletion
	bool  m_isUndeformedMeshOwned;
	//! Undeformed positions of the nodes in the mesh
	Pt3D* m_undeformedMesh;
	//! Positions of the nodes in the mesh at the end of the previous time-step
	Pt3D* m_lastDeformedMesh;
	//! Current deformed positions of the nodes in the mesh
	Pt3D* m_deformedMesh;

	//! Initial node poses
	Dynamic_Vector<double> m_x0;
	//! Current node poses
	Dynamic_Vector<double> m_xt;

	//! Number of Boundary Conditions (fixed nodes)
	int m_nbBC;
	//! Number of Boundary Conditions allocated 
	int m_nbBCAllocated;
	//! List of Boundary Conditions
	int* m_BC;
	/*! Mass and inverse mass used in Stiffness and Compliance matrices for the Boundary Conditions
	  Note that this is more a numerical trick than a real mass...as it will be used even in STATIC !
	*/
	double m_BCmass, m_BCinvMass;

	//! Number of elements in the mesh
	int m_nbElements;
	//! Number of elements allocated in the mesh
	int m_nbElementsAllocated;
	//! Whether this FEM owns the elements, and should handle their deletion
	bool m_areElementsOwned;
	//! Elements in the mesh
	/*!
	The first dimension is owned and can be destroyed safely.
	The elements are owned if m_areElementsOwned is true.
	*/
	FEMElement** m_elements;

	//! Whether the simulation uses precomputed LU decomposition (for direct solution); if not Conjugate Gradient (iterative) is used
	const bool m_usesPrecomputedDecomposition;
	//! Whether the simulation is dynamic (Backwards Euler); if not, it is static (no velocity effects)
	const bool m_isDynamic;
	//! Whether MLCP (Mixed Linear Complementarity Problem) is used, for which the compliance matrix needs to be computed
	const bool m_usesMLCP;

	double m_staticCorot_DispScaleFactor;	// Scaling factor applied to the found displacement when in Static Co-rotational (for stability)

	//! Maximum number of Conjugate Gradient iterations
	int m_CG_maxIteration;
	//! Conjugate Gradient convergence epsilon
	double m_CG_epsilon;
	
	//! Mass proportional Rayleigh damping coefficient
	double m_RayleightDamping_Mass_coef;
	//! Stiffness proportional Rayleigh damping coefficient
	double m_RayleightDamping_Stif_coef;

	//! LU data structures
	Eigen::MatrixXd m_K_LU;
	//! LU decomposition permutation
	Eigen::VectorXi m_LUpermutation;

	//! CG data structures
	Eigen::MatrixXd m_r, m_d, m_q;
	//! Force vector with boundary conditions, used for CG
	Eigen::MatrixXd m_FwithBC;

	//! Mass matrix
	Eigen::MatrixXd m_M;
	//! Global stiffness matrix
	Eigen::MatrixXd m_K ;
	//! Sparse global stiffness matrix
	Eigen::SparseMatrixXd m_K_sparse;
	//! Matrices with Rayleigh damping for Backward Euler
	Dynamic_Matrix<double> m_MDK,m_MD;
	//! Compliance matrix
	/*!
	K-1 in static
	(M/dt2 + D/dt + K)-1 in dynamic
	Used for contact solving
	*/
	Dynamic_Matrix<double> m_C;
	//! Node mass
	/*!
	m_M_node[i] == m_M[nbDOFPerNode*i][nbDOFPerNode*i], except we may never assemble the full m_M
	*/
	Dynamic_Vector<double> m_M_node;
	//! Time step (assumed to be fixed)
	double m_dt;

	//! Solution (current global displacement)
	Dynamic_Vector<double> m_Ut;
	//! Previous displacement vector
	Dynamic_Vector<double> m_Ut_minus_dt;
	//! Current velocity ([U(t-dt) - U(t)] / dt)
	Dynamic_Vector<double> m_Vt;
	//! Current external forces
	Dynamic_Vector<double> m_F;


	//! Current gravity portion of external forces to avoid recomputing when gravity does not change
	Dynamic_Vector<double> m_gravityForce;
	//! Current gravitational acceleration applied to all nodes (if only using the 3D interface, the last 3 components are 0)
	double m_gravity[nbDOFPerNode];

	//! Scaled constraint displacement vector
	Dynamic_Vector<double> m_Uc_scaled;

protected:
	//! Global stiffness matrix used for computing corotational offset forces in 3D
	/*! RK(R^t.x - x0) = F
	 => RKR^t.x = F + RK.x0
	*/
	Dynamic_Matrix<double> m_RK;
	//! Sparse version of the same matrix
	SqMklSparseMatrix<double> m_RK_sparse;

	//! Rotation per node (quaternion form)
	SqQuaternion<double>* m_rotationPerNodeQ;
	//! Rotation per node (3x3 matrix form)
	Static_Matrix<double, 3, 3>* m_rotationPerNode;

	//! Partial warped compliance matrix with only the columns of the current set of constrained nodes (for co-rotational)
	Dynamic_Matrix<double> m_warpedC;
	//! Mapping from constrained node IDs to warped C columns (used to build LCP)
	std::map<int, int> m_constrainedNodesToColumnsInWarpedC; 

	// Corotational frame calculation variables
	typedef struct{
		SqTransformd world_T_elem_t0;						// Beam global transformation at t=0
		SqTransformd world_T_elem_t;						// Beam global transformation at t
		SqTransformd elem_T_node0_t0 , elem_T_node1_t0;		// Node{0|1} transformation in beam local frame at t=0
		SqTransformd elem_T_node0_t  , elem_T_node1_t ;		// Node{0|1} transformation in beam local frame at t
	} BeamTransformation;
	typedef struct{
		SqTransformd world_T_node_t0;		// Global node transformation at t=0
		SqTransformd world_T_node_t ;		// Global node transformation at t
	} NodeTransformation;
	std::vector<BeamTransformation> beamTransformations;
	std::vector<NodeTransformation> nodeTransformations;

	//! Computes the left hand side of the Backward Euler system (MD and MDK)
	/*!
	\param dt Time step
	\param xt Current node poses
	\param vt Current node velocities
	*/
	virtual void computeLHS_BackwardEuler(const double dt, const double* xt, const double* vt);
	//! Adds the right hand side of the Backward Euler system (F)
	/*!
	\param dt Time step
	\param xt Current node poses
	\param vt Current node velocities
	\param F Resulting right hand side
	*/
	virtual void addRHS_BackwardEuler(const double dt, const double* xt, const double* vt, double* F);
	//! Adds the right hand side of the Backward Euler system (F) using Conjugate Gradient
	/*!
	\param dt Time step
	\param xt Current node poses
	\param vt Current node velocities
	\param F Resulting right hand side
	*/
	virtual void addRHS_BackwardEuler_CG(const double dt, const double* xt, const double* vt, double* F);

	//! Calculates product of the global stiffness matrix (K) and current global displacement vector (u).
	/*!
	\param u Global displacement vector (size nbDOFPerNode * m_nbNodes)
	\param x Current deformed node positions (size m_nbNodes)
	\param x0 Undeformed node positions (size m_nbNodes)
	\param result Resulting product (K * u)
	\param useGlobalMatrix Whether to use the pre-calculated global stiffness matrix K, otherwise the produce is computed without building K.
	This is used by the Static Conjugate Gradient solver.
	*/
	virtual void MatVecProduct_K_u(const double* u, const double* x, const double* x0, double* result, bool useGlobalMatrix = false);

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

	//! Calculates MD*x + M/dt * v and adds it to the given vector without building the matrices.
	/*!
	MD = (1/(dt*dt) + mu)*M + (lambda/dt)*K, 
	dt is the timestep, 
	mu is the mass proportional Rayleigh damping coefficient, 
	lambda is the stiffness proportional Rayleigh damping coefficient, 
	M is the mass matrix, 
	K is the global stiffness matrix, 
	x is the current deformed node positions, 
	v is the current node velocities
	\param dt Time step
	\param rayleighDampingMassCoefficient Mass proportional Rayleigh damping coefficient
	\param rayleighDampingStiffnessCoefficient Stiffness proportional Rayleigh damping coefficient
	\param u Global displacement vector (size nbDOFPerNode * m_nbNodes)
	\param v Global velocity of nodes (size nbDOFPerNode * m_nbNodes)
	\param x Current deformed node positions (size m_nbNodes)
	\param x0 Undeformed node positions (size m_nbNodes)
	\param result Resulting product (K * u)
	This is used by the Dynamic Conjugate Gradient solver.
	*/
	virtual void addMatVecProduct_MD_x_Mt_v(double dt, double rayleighDampingMassCoefficient, double rayleighDampingStiffnessCoefficient,
		const double* u, const double* v, const double* x, const double* x0, double* result);

	//! Updates the mesh point positions with the current node positions (xt)
	virtual void updateDeformedMesh();
};

#include "Fem.inl.h"

#endif
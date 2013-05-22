#ifndef __MLCP_GaussSeidel_Christian__
#define __MLCP_GaussSeidel_Christian__

#include <SurgSim/Math/MLCP.h>              //XXX temporary??
#include <SurgSim/Math/MLCP_Constraint.h>   //XXX temporary??
#include <Eigen/Core>

//! Resolution of a mixed LCP problem (Gauss Seidel iterative solver)
/*!
  Iterative solver based on Gauss-Seidel.

  Problem can contains:
    - CONSTRAINT  = Bilateral constraint (all atomic, a fixed 3D point=3 atomics independents constraints)
    - CONTACT     = Unilateral constraint
      * frictionless => 1 atomic constraint per contact
      * frictional with Coulomb friction (1 mu parameter per contact) => 3 atomic dependent constraints per contact (1 directional + 2 tangentials)
    - SUTURING    = Sliding constraint for suturing
      * Frictionless suturing constraint => 2 atomic constraints per sliding point
      * Frictional suturing constraint   => 3 atomic constraints per sliding point (2 directional + 1 tangential with friction on it) => 1 mu parameter per frictional suturing

  cf. Christian Duriez TVCG05 paper
  Realistic Haptic Rendering of Interacting
  Deformable Objects in Virtual Environments
  Christian Duriez, Student Member, IEEE, Fre´de´ ric Dubois,
  Abderrahmane Kheddar, Member, IEEE, and Claude Andriot
  +
  + recent Christian's work
*/
template <class Matrix, class Vector> class MLCP_GaussSeidel_Christian : public MLCP<Matrix,Vector>
{
private:
	double       epsilonConvergence;
	double       contactTolerance;
	unsigned int maxIterations;

	int nbEnforcedAtomicConstraint;  // from id 0..nbEnforcedAtomicConstraint-1
	Eigen::MatrixXd LHS_enforcedLocalSystem;
	Eigen::VectorXd RHS_enforcedLocalSystem;

	void computeEnforcementSystem(IN int n, IN Matrix& A, IN int nbColumnInA, IN Vector& b, INOUT Vector& initialGuess_and_solution, IN Vector& frictionCoefs,
	                              IN vector<MLCP_Constraint>& constraintsType, IN double subStep,
	                              IN int constraintID,IN int matrixEntryForConstraintID);

	void calculateConvergenceCriteria(IN int n, IN Matrix& A, IN int nbColumnInA, IN Vector& b,
	                                  IN Vector& initialGuess_and_solution, IN vector<MLCP_Constraint>& constraintsType, IN double subStep,
	                                  double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES], double& convergence_criteria, bool& signoriniVerified, bool& signoriniValid);

	void doOneIteration(IN int n, IN Matrix& A, IN int nbColumnInA, IN Vector& b, INOUT Vector& initialGuess_and_solution, IN Vector& frictionCoefs,
	                    IN vector<MLCP_Constraint>& constraintsType, IN double subStep,
	                    OUT double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES], OUT double& convergence_criteria, OUT bool& signoriniVerified);

	void printViolationsAndConvergence(IN int n, IN Matrix& A, IN int nbColumnInA, IN Vector& b, IN Vector& initialGuess_and_solution,
	                                   IN vector<MLCP_Constraint>& constraintsType, IN double subStep, IN double convergence_criteria, IN bool signorini_verified, IN int nbLoop);

public:
	MLCP_GaussSeidel_Christian(double _epsilonConvergence, double _contactTolerance, unsigned int _maxIterations)
		:epsilonConvergence(_epsilonConvergence),contactTolerance(_contactTolerance),maxIterations(_maxIterations) {};

	~MLCP_GaussSeidel_Christian() {};

	//! Resolution of a given LCP (empty)
	/*!
		@param n an integer argument, the number of atomic constraint in the MLCP
		@param A a Matrix, the MLCP matrix
		@param nbColumnInA an integer argument, the actual number of column (alocated in memory) of the matrix A
		@param b a Vector, the MLCP right hand side (the initial constraints violation)
		@param initialGuess_and_solution a Vector, contains the initial guess and the final solution on exit !
		@param frictionCoefs a Vector, contains the list of friction coefficient for each constraint (or 0 if not needed)
		@param constraintsList a vector, contains the list of constraint type, to determine how much atomic constraint each takes and how to handle it !

	This method will be implemented by inherited classes.
	Each of them will implement a different algorithm.
	USING DIFFERENT FRICTION COEFFICIENT FOR EACH CONTACT
	*/
	bool solve(IN int n, IN Matrix& A, IN int nbColumnInA, IN Vector& b, INOUT Vector& initialGuess_and_solution,
	           IN Vector& frictionCoefs, IN vector<MLCP_Constraint>& constraintsType, IN double subStep = 1.0,
	           OUT int* MLCP_nbIterations=0, OUT bool* validConvergence=0, OUT bool* validSignorini=0,
	           OUT double* convergenceCriteria = 0, OUT double* initialConvergenceCriteria = 0,
	           OUT double* constraintConvergenceCriteria = 0, OUT double* initialConstraintConvergenceCriteria = 0,
	           IN bool catchExplodingConvergenceCriteria = false, IN bool verbose = false);
};

#include "MLCP_GaussSeidel_Christian-inl.h"

#endif

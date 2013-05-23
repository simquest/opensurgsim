#ifndef SURGSIM_MATH_MLCPSOLVER_H
#define SURGSIM_MATH_MLCPSOLVER_H

namespace SurgSim
{
namespace Math
{

struct MlcpProblem;
struct MlcpSolution;

/*!
This class defines a Mixed LCP problem defined by:
A x + b = c
0 <= x orthogonal c => 0 for a subset

Ax+b (=c) measures de violation of the constraint (inequality)
Either we have Ax+b>0 AND x=0 (On the good side of the inequality => no violation therefore no force (x=0))
Or     we have Ax+b=0 AND x>0 (On the wrong side of the inequality => violation therefore a force (x>0) bring back the constraint to verified Ax+b=0)

We have a mixed of bilateral constraints and unilateral constraints
*/
class MlcpSolver
{
public:
	//! Empty constructor for easy usage.
	MlcpSolver()
	{
	}

	//! Destructor empty....implicit call to the member's destructor.
	virtual ~MlcpSolver()
	{
	}

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
	virtual bool solve(const MlcpProblem& problem, MlcpSolution* solution) = 0;
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPSOLVER_H

#ifndef SURGSIM_MATH_MLCPSOLVER_H
#define SURGSIM_MATH_MLCPSOLVER_H

namespace SurgSim
{
namespace Math
{

struct MlcpProblem;
struct MlcpSolution;

/// This class provides a solver interface for mixed linear complementarity problems.
///
/// \sa MlcpProblem
class MlcpSolver
{
public:
	/// Constructor.
	MlcpSolver()
	{
	}

	// Destructor.
	virtual ~MlcpSolver()
	{
	}

	/// Attempts to solve the specified MLCP problem.
	/// \param problem the MLCP problem.
	/// \param [out] solution the solution to the problem, if available.
	/// \return true if solved (in which case solution will be set to the solution); false if failed.
	virtual bool solve(const MlcpProblem& problem, MlcpSolution* solution) = 0;
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPSOLVER_H

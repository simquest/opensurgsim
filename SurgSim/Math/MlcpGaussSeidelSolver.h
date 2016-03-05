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

#ifndef SURGSIM_MATH_MLCPGAUSSSEIDELSOLVER_H
#define SURGSIM_MATH_MLCPGAUSSSEIDELSOLVER_H

#include <memory.h>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/MlcpProblem.h"
#include "SurgSim/Math/MlcpSolver.h"
#include "SurgSim/Math/MlcpSolution.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Framework
{
class Logger;
}

namespace Math
{

/// A solver for mixed LCP problems using the Gauss-Seidel iterative method.
///
/// The problem can contain:
///  - CONSTRAINT  = Bilateral constraint (all atomic, a fixed 3D point=3 atomics independents constraints)
///  - CONTACT     = Unilateral constraint
///    * frictionless => 1 atomic constraint per contact
///    * frictional with Coulomb friction (1 mu parameter per contact) => 3 atomic dependent constraints per contact
///      (1 directional + 2 tangentials)
///  - SUTURING    = Sliding constraint for suturing
///    * Frictionless suturing constraint => 2 atomic constraints per sliding point
///    * Frictional suturing constraint   => 3 atomic constraints per sliding point (2 directional + 1 tangential with
///      friction on it) => 1 mu parameter per frictional suturing
///
/// See e.g.: Duriez, Christian; Dubois, F.; Kheddar, A.; Andriot, C., "Realistic haptic rendering of interacting
/// deformable objects in virtual environments," <i>IEEE Transactions on Visualization and Computer Graphics,</i>
/// vol.12, no.1, pp.36,47, Jan.-Feb. 2006.
class MlcpGaussSeidelSolver : public MlcpSolver
{
public:
	/// Constructor.
	MlcpGaussSeidelSolver();

	/// Constructor.
	/// \param epsilonConvergence The precision.
	/// \param contactTolerance The contact tolerance.
	/// \param maxIterations The max iterations.
	MlcpGaussSeidelSolver(double epsilonConvergence, double contactTolerance, size_t maxIterations);

	/// Destructor.
	virtual ~MlcpGaussSeidelSolver();

	/// Resolution of a given MLCP (Gauss Seidel iterative solver)
	/// \param problem The mlcp problem
	/// \param [out] solution The mlcp solution
	/// \return true if successfully converged.
	bool solve(const MlcpProblem& problem, MlcpSolution* solution);

	/// \return The precision.
	double getEpsilonConvergence() const;

	/// Set the precision.
	/// \param precision The precision.
	void setEpsilonConvergence(double precision);

	/// \return The contact tolerance.
	std::pair<double, double> getContactTolerance() const;

	/// Set the contact tolerance.  The greater value will be used to determine if the solution has succeeded.
	/// The lesser value will be used to terminate the iterations early.
	/// \param tolerance The contact tolerance.
	void setContactTolerance(std::pair<double, double> tolerance);

	/// \return The max number of iterations.
	size_t getMaxIterations() const;

	/// Set the max number of iterations.
	/// \param maxIterations The max number of iterations.
	void setMaxIterations(size_t maxIterations);

private:
	void computeEnforcementSystem(size_t problemSize, const MlcpProblem::Matrix& A,
								  const MlcpProblem::Vector& b,
								  const MlcpSolution::Vector& initialGuessAndSolution,
								  const std::vector<MlcpConstraintType>& constraintsType,
								  size_t constraintID, size_t matrixEntryForConstraintID);

	void calculateConvergenceCriteria(size_t problemSize, const MlcpProblem::Matrix& A,
									  const MlcpProblem::Vector& b,
									  const MlcpSolution::Vector& initialGuessAndSolution,
									  const std::vector<MlcpConstraintType>& constraintsType,
									  double constraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES],
									  double* convergenceCriteria,
									  bool* validSignorini, bool* tighterSignorini);

	void doOneIteration(size_t problemSize, const MlcpProblem::Matrix& A,
						const MlcpProblem::Vector& b,
						MlcpSolution::Vector* initialGuessAndSolution,
						const MlcpProblem::Vector& frictionCoefs,
						const std::vector<MlcpConstraintType>& constraintsType,
						double constraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES], double* convergenceCriteria,
						bool* validSignorini);

	void printViolationsAndConvergence(size_t problemSize, const MlcpProblem::Matrix& A,
									   const MlcpProblem::Vector& b,
									   const MlcpSolution::Vector& initialGuessAndSolution,
									   const std::vector<MlcpConstraintType>& constraintsType,
									   double convergenceCriteria,
									   bool validSignorini, size_t iterations);

	/// The precision.
	double m_epsilonConvergence;

	/// The contact tolerance.  The greater value will be used to determine if the solution has succeeded.
	/// The lesser value will be used to terminate the iterations early.
	std::pair<double, double> m_contactTolerance;

	/// The maximum number of iterations
	size_t m_maxIterations;

	/// The number of atomic constraints, aka the system size.
	size_t m_numEnforcedAtomicConstraints;

	/// The left-hand side matrix.
	Matrix m_lhsEnforcedLocalSystem;

	/// The right-hand side vector.
	Vector m_rhsEnforcedLocalSystem;

	/// The logger.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPGAUSSSEIDELSOLVER_H

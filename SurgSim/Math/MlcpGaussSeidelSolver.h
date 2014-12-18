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

#include "SurgSim/Math/MlcpSolver.h"
#include <Eigen/Core>
#include "SurgSim/Math/MlcpProblem.h"
#include "SurgSim/Math/MlcpSolution.h"

namespace SurgSim
{
namespace Math
{

/// A solver for mixed LCP problems using the Gauss-Seidel iterative method.
///
/// \todo Clean this up more...
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
	MlcpGaussSeidelSolver() :
		m_epsilonConvergence(defaultEpsilonConvergence()),
		m_contactTolerance(defaultContactTolerance()),
		m_substepRatio(1.0),
		m_maxIterations(defaultMaxIterations()),
		m_catchExplodingConvergenceCriteria(true),
		m_verbose(false),
		m_numEnforcedAtomicConstraints(-1)
	{
	}

	MlcpGaussSeidelSolver(double epsilonConvergence, double contactTolerance, unsigned int maxIterations) :
		m_epsilonConvergence(epsilonConvergence),
		m_contactTolerance(contactTolerance),
		m_substepRatio(1.0),
		m_maxIterations(maxIterations),
		m_catchExplodingConvergenceCriteria(true),
		m_verbose(false),
		m_numEnforcedAtomicConstraints(-1)
	{
	}

	virtual ~MlcpGaussSeidelSolver()
	{
	}


	bool solve(const MlcpProblem& problem, MlcpSolution* solution);


	double getEpsilonConvergence() const
	{
		return m_epsilonConvergence;
	}
	void setEpsilonConvergence(double val)
	{
		m_epsilonConvergence = val;
	}
	double getContactTolerance() const
	{
		return m_contactTolerance;
	}
	void setContactTolerance(double val)
	{
		m_contactTolerance = val;
	}
	double getSubstepRatio() const
	{
		return m_substepRatio;
	}
	void setSubstepRatio(double val)
	{
		m_substepRatio = val;
	}
	unsigned int getMaxIterations() const
	{
		return m_maxIterations;
	}
	void setMaxIterations(unsigned int val)
	{
		m_maxIterations = val;
	}
	bool isCatchingExplodingConvergenceCriteria() const
	{
		return m_catchExplodingConvergenceCriteria;
	}
	void setCatchingExplodingConvergenceCriteria(bool val)
	{
		m_catchExplodingConvergenceCriteria = val;
	}
	bool isVerbose() const
	{
		return m_verbose;
	}
	void setVerbose(bool val)
	{
		m_verbose = val;
	}


	static double defaultEpsilonConvergence()
	{
		return 1e-4;
	}
	static double defaultContactTolerance()
	{
		return 2e-5;
	}
	static int defaultMaxIterations()
	{
		return 30;
	}

private:
	void computeEnforcementSystem(int n, const MlcpProblem::Matrix& A, int nbColumnInA,
								  const MlcpProblem::Vector& b,
								  const MlcpSolution::Vector& initialGuess_and_solution,
								  const MlcpProblem::Vector& frictionCoefs,
								  const std::vector<MlcpConstraintType>& constraintsType, double subStep,
								  int constraintID, int matrixEntryForConstraintID);

	void calculateConvergenceCriteria(int n, const MlcpProblem::Matrix& A, int nbColumnInA,
									  const MlcpProblem::Vector& b,
									  const MlcpSolution::Vector& initialGuess_and_solution,
									  const std::vector<MlcpConstraintType>& constraintsType,
									  double subStep,
									  double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES],
									  double* convergence_criteria,
									  bool* signoriniVerified, bool* signoriniValid);

	void doOneIteration(int n, const MlcpProblem::Matrix& A, int nbColumnInA, const MlcpProblem::Vector& b,
						MlcpSolution::Vector* initialGuess_and_solution,
						const MlcpProblem::Vector& frictionCoefs,
						const std::vector<MlcpConstraintType>& constraintsType, double subStep,
						double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES], double* convergence_criteria,
						bool* signoriniVerified);

	void printViolationsAndConvergence(int n, const MlcpProblem::Matrix& A, int nbColumnInA,
									   const MlcpProblem::Vector& b,
									   const MlcpSolution::Vector& initialGuess_and_solution,
									   const std::vector<MlcpConstraintType>& constraintsType,
									   double subStep, double convergence_criteria,
									   bool signorini_verified, int nbLoop);


	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;


	double       m_epsilonConvergence;
	double       m_contactTolerance;
	double       m_substepRatio;
	unsigned int m_maxIterations;
	bool         m_catchExplodingConvergenceCriteria;
	bool         m_verbose;

	int m_numEnforcedAtomicConstraints;
	Matrix m_lhsEnforcedLocalSystem;
	Vector m_rhsEnforcedLocalSystem;
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPGAUSSSEIDELSOLVER_H

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

#include "SurgSim/Math/MlcpGaussSeidelSolver.h"

#include <math.h>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Valid.h"


namespace SurgSim
{
namespace Math
{

MlcpGaussSeidelSolver::MlcpGaussSeidelSolver() :
	m_epsilonConvergence(1e-4),
	m_contactTolerance(2e-5),
	m_maxIterations(30),
	m_numEnforcedAtomicConstraints(0),
	m_logger(SurgSim::Framework::Logger::getLogger("Math/MlcpGaussSeidelSolver"))
{
}

MlcpGaussSeidelSolver::MlcpGaussSeidelSolver(double epsilonConvergence, double contactTolerance, size_t maxIterations) :
	m_epsilonConvergence(epsilonConvergence),
	m_contactTolerance(contactTolerance),
	m_maxIterations(maxIterations),
	m_numEnforcedAtomicConstraints(0),
	m_logger(SurgSim::Framework::Logger::getLogger("Math/MlcpGaussSeidelSolver"))
{
}

MlcpGaussSeidelSolver::~MlcpGaussSeidelSolver()
{
}

double MlcpGaussSeidelSolver::getEpsilonConvergence() const
{
	return m_epsilonConvergence;
}

void MlcpGaussSeidelSolver::setEpsilonConvergence(double precision)
{
	m_epsilonConvergence = precision;
}

double MlcpGaussSeidelSolver::getContactTolerance() const
{
	return m_contactTolerance;
}

void MlcpGaussSeidelSolver::setContactTolerance(double tolerance)
{
	m_contactTolerance = tolerance;
}

size_t MlcpGaussSeidelSolver::getMaxIterations() const
{
	return m_maxIterations;
}

void MlcpGaussSeidelSolver::setMaxIterations(size_t maxIterations)
{
	m_maxIterations = maxIterations;
}

bool MlcpGaussSeidelSolver::solve(const MlcpProblem& problem, MlcpSolution* solution)
{
	const size_t problemSize = problem.getSize();
	MlcpProblem::Matrix A = problem.A;
	const MlcpProblem::Vector& b = problem.b;
	MlcpSolution::Vector& initialGuessAndSolution = solution->x;
	const std::vector<MlcpConstraintType>& constraintsType = problem.constraintTypes;
	size_t* iteration = &solution->numIterations;
	bool* validConvergence = &solution->validConvergence;
	bool* validSignorini = &solution->validSignorini;
	double* convergenceCriteria = &solution->convergenceCriteria;
	double* initialConvergenceCriteria = &solution->initialConvergenceCriteria;
	double* constraintConvergenceCriteria = solution->constraintConvergenceCriteria;
	double* initialConstraintConvergenceCriteria = solution->initialConstraintConvergenceCriteria;

	// Loop until it converges or maxIterations are reached
	*iteration = 0;
	*validSignorini = true;

	calculateConvergenceCriteria(problemSize, A, b, initialGuessAndSolution,
								 constraintsType, initialConstraintConvergenceCriteria, initialConvergenceCriteria,
								 validSignorini);

	// If it is already converged, fill the output and return true.
	if (*initialConvergenceCriteria <= m_epsilonConvergence && *validSignorini)
	{
		*validConvergence = true;
		*convergenceCriteria = *initialConvergenceCriteria;
		return true;
	}

	do
	{
		doOneIteration(problemSize, A, b, &initialGuessAndSolution, problem.mu, constraintsType,
					   constraintConvergenceCriteria, convergenceCriteria, validSignorini);

		calculateConvergenceCriteria(problemSize, A, b, initialGuessAndSolution,
									 constraintsType, constraintConvergenceCriteria, convergenceCriteria,
									 validSignorini);
		++(*iteration);

		// If we have an incredibly high convergence criteria value, the displacements are going to be very large,
		// causing problems in the next iteration, so we should break out here. The convergence_criteria should
		// really only be a couple order of magnitudes higher than epsilon.
		if (!SurgSim::Math::isValid(*convergenceCriteria) || *convergenceCriteria > 1.0)
		{
			SURGSIM_LOG_WARNING(m_logger) << "Convergence (" << *convergenceCriteria <<
				") is NaN, infinite, or greater than 1.0! MLCP is exploding after " << *iteration <<
				" Gauss Seidel iterations!!";
			break;
		}
		if ((*iteration >= m_maxIterations) && !(*validSignorini) &&
			(!isValid(initialGuessAndSolution) || (initialGuessAndSolution.cwiseAbs().maxCoeff() > 1e5)))
		{
			MlcpSolution::Vector::Index count = 0;
			for (MlcpSolution::Vector::Index i = 0; i < initialGuessAndSolution.size(); ++i)
			{
				if (!SurgSim::Math::isValid(initialGuessAndSolution[i]) || (initialGuessAndSolution[i] > 1e5))
				{
					A.col(i).setZero();
					++count;
				}
			}
			initialGuessAndSolution.setZero();
			if (count == initialGuessAndSolution.size())
			{
				SURGSIM_LOG_WARNING(m_logger) << "Failed to find a solution: all entries are invalid or too big.";
			}
			else
			{
				*iteration = 0;
				SURGSIM_LOG_WARNING(m_logger) <<
					"Failed to find a solution. Re-solving without conflicting constraints.";
			}
		}
	}
	while ((!(*validSignorini) || (*convergenceCriteria > m_epsilonConvergence)) && *iteration < m_maxIterations);

	*validConvergence = SurgSim::Math::isValid(*convergenceCriteria) && *convergenceCriteria <= 1.0;

	SURGSIM_LOG_IF(*convergenceCriteria >= sqrt(m_epsilonConvergence), m_logger, WARNING) <<
		"Convergence criteria (" << *convergenceCriteria << ") is greater than " << sqrt(m_epsilonConvergence) <<
		" at end of " << *iteration << " Gauss Seidel iterations.";

	SURGSIM_LOG_IF(*convergenceCriteria > *initialConvergenceCriteria, m_logger, WARNING) <<
		"Convergence criteria (" << *convergenceCriteria << ") is greater than before " << *iteration <<
		" Gauss Seidel iterations (" << *initialConvergenceCriteria << ").";

	SURGSIM_LOG_IF(!(*validSignorini), m_logger, WARNING) <<
		"Signorini not verified after " << *iteration << " Gauss Seidel iterations.";

	return (SurgSim::Math::isValid(*convergenceCriteria) && *convergenceCriteria <= m_epsilonConvergence);
}


void MlcpGaussSeidelSolver::calculateConvergenceCriteria(size_t problemSize, const MlcpProblem::Matrix& A,
													const MlcpProblem::Vector& b,
													const MlcpSolution::Vector& initialGuessAndSolution,
													const std::vector<MlcpConstraintType>& constraintsType,
													double constraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES],
													double* convergenceCriteria,
													bool* validSignorini)
{
	// Calculate initial convergence criteria.
	for (size_t constraint = 0; constraint < MLCP_NUM_CONSTRAINT_TYPES; ++constraint)
	{
		constraintConvergenceCriteria[constraint] = 0.0;
	}
	*convergenceCriteria = 0.0;
	*validSignorini = true;

	size_t currentAtomicIndex = 0;
	const size_t nbConstraints = constraintsType.size();
	size_t nbNonContactConstraints = 0;

	for (size_t constraint = 0; constraint < nbConstraints; ++constraint)
	{
		switch (constraintsType[constraint])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			const double criteria =
				(b.segment<1>(currentAtomicIndex) + A.row(currentAtomicIndex) * initialGuessAndSolution).norm();
			*convergenceCriteria += criteria;
			constraintConvergenceCriteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
			currentAtomicIndex += 1;
			break;
		}

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			const double criteria = (b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * initialGuessAndSolution).norm();
			*convergenceCriteria += criteria;
			constraintConvergenceCriteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
			currentAtomicIndex += 2;
			break;
		}

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			const double criteria = (b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * initialGuessAndSolution).norm();
			*convergenceCriteria += criteria;
			constraintConvergenceCriteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
			currentAtomicIndex += 3;
			break;
		}

		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			const double violation = b[currentAtomicIndex] + A.row(currentAtomicIndex) * initialGuessAndSolution;
			// Enforce orthogonality condition
			if (!SurgSim::Math::isValid(violation) || violation < -m_contactTolerance ||
				(initialGuessAndSolution[currentAtomicIndex] > m_epsilonConvergence &&
				 violation > m_contactTolerance))
			{
				*validSignorini = false;
			}
			currentAtomicIndex += 1;
			break;
		}

		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			const double violation = b[currentAtomicIndex] + A.row(currentAtomicIndex) * initialGuessAndSolution;
			// Enforce orthogonality condition
			if (!SurgSim::Math::isValid(violation) || violation < -m_contactTolerance ||
				(initialGuessAndSolution[currentAtomicIndex] > m_epsilonConvergence &&
				 violation > m_contactTolerance))
			{
				*validSignorini = false;
			}
			currentAtomicIndex += 3;
			break;
		}

		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			const double criteria = (b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * initialGuessAndSolution).norm();
			*convergenceCriteria += criteria;
			constraintConvergenceCriteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
			currentAtomicIndex += 2;
			break;
		}

		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			// We verify that the sliding point is on the line...no matter what the friction violation is
			// (3rd component)
			const double criteria = (b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * initialGuessAndSolution).norm();
			*convergenceCriteria += criteria;
			constraintConvergenceCriteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
			currentAtomicIndex += 3;
			break;
		}

		default:
			SURGSIM_FAILURE() << "unknown constraint type [" << constraintsType[constraint] << "]";
			break;
		}

	}

	if (nbNonContactConstraints > 0)
	{
		*convergenceCriteria /= nbNonContactConstraints;    // normalize if necessary
	}
}

void MlcpGaussSeidelSolver::computeEnforcementSystem(
	size_t problemSize, const MlcpProblem::Matrix& A, const MlcpProblem::Vector& b,
	const MlcpSolution::Vector& initialGuessAndSolution,
	const std::vector<MlcpConstraintType>& constraintsType,
	size_t constraintID, size_t matrixEntryForConstraintID)
{
	const size_t nbConstraints = constraintsType.size();
	// Total size of the system (number of line and column in the final matrix)
	size_t systemSize = 0;
	// Total size of the system counting only the {1D,2D,3D} bilateral constraints
	size_t systemSizeWithoutConstraintID = 0;

	// 1st) compute the size of the final system
	// We suppose that the constraint to enforce are only 1D, 2D or 3D bilateral constraints
	// and all appearing first in the list !
	{
		bool done = false;
		for (size_t i = 0; i < nbConstraints; ++i)
		{
			switch (constraintsType[i])
			{
			case MLCP_BILATERAL_1D_CONSTRAINT:
			{
				systemSize += 1;
				systemSizeWithoutConstraintID += 1;
				break;
			}

			case MLCP_BILATERAL_2D_CONSTRAINT:
			{
				systemSize += 2;
				systemSizeWithoutConstraintID += 2;
				break;
			}

			case MLCP_BILATERAL_3D_CONSTRAINT:
			{
				systemSize += 3;
				systemSizeWithoutConstraintID += 3;
				break;
			}

			default:
				done = true;
				break;
			}

			if (done)
			{
				break;
			}
		}

		// We added all the constraints, now we need to add the constraintID (contact, sliding,...) to have the total
		// system size!
		switch (constraintsType[constraintID])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		// The system will solve the normal contact, not the frictional parts !
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			systemSize += 1;
			break;
		}

		case MLCP_BILATERAL_2D_CONSTRAINT:
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		// The system will solve the sliding case, not the frictional part !
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			systemSize += 2;
			break;
		}

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			systemSize += 3;
			break;
		}

		default:
			SURGSIM_LOG_SEVERE(m_logger) << "MlcpGaussSeidelSolver::computeEnforcementSystem  Unknown constraint !?";
			break;
		}
	}
	m_numEnforcedAtomicConstraints = systemSize;
	m_rhsEnforcedLocalSystem.resize(systemSize);
	m_lhsEnforcedLocalSystem.resize(systemSize, systemSize);

	// 2nd) Fill up the system
	// We suppose that the constraint to enforce are only 1D, 2D or 3D bilateral constraints
	{
		// Here we fill up the core part, compliance between all the constraints themselves !
		m_rhsEnforcedLocalSystem.head(systemSizeWithoutConstraintID) = b.head(systemSizeWithoutConstraintID) +
			A.block(0, 0, systemSizeWithoutConstraintID, problemSize) * initialGuessAndSolution;
		m_lhsEnforcedLocalSystem.block(0, 0, systemSizeWithoutConstraintID, systemSizeWithoutConstraintID) =
			A.block(0, 0, systemSizeWithoutConstraintID, systemSizeWithoutConstraintID);

		// Now we complete the contact matrix by adding the coupling constraint/{contact|sliding} and the compliance
		// for {contact|sliding}
		switch (constraintsType[constraintID])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		// In any case of contact, we only register the normal part...the friction part is computed afterward !
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] =
				b[matrixEntryForConstraintID] +
				A.row(matrixEntryForConstraintID) * initialGuessAndSolution;
			m_lhsEnforcedLocalSystem.block(0, systemSizeWithoutConstraintID, systemSizeWithoutConstraintID, 1) =
				A.block(0, matrixEntryForConstraintID, systemSizeWithoutConstraintID, 1);
			m_lhsEnforcedLocalSystem.block(systemSizeWithoutConstraintID, 0, 1, systemSizeWithoutConstraintID) =
				A.block(matrixEntryForConstraintID, 0, 1, systemSizeWithoutConstraintID);
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID);
			break;
		}

		case MLCP_BILATERAL_2D_CONSTRAINT:
		// In any case of sliding, we only register the normals part...the friction part along the tangent is computed
		// afterward !
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem.segment<2>(systemSizeWithoutConstraintID) =
				b.segment<2>(matrixEntryForConstraintID) +
				A.block(matrixEntryForConstraintID, 0, 2, problemSize) * initialGuessAndSolution;
			m_lhsEnforcedLocalSystem.block(0, systemSizeWithoutConstraintID, systemSizeWithoutConstraintID, 2) =
				A.block(0, matrixEntryForConstraintID, systemSizeWithoutConstraintID, 2);
			m_lhsEnforcedLocalSystem.block(systemSizeWithoutConstraintID, 0, 2, systemSizeWithoutConstraintID) =
				A.block(matrixEntryForConstraintID, 0, 2, systemSizeWithoutConstraintID);
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem.block(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID, 2, 2) =
				A.block(matrixEntryForConstraintID, matrixEntryForConstraintID, 2, 2);
			break;
		}

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem.segment<3>(systemSizeWithoutConstraintID) =
				b.segment<3>(matrixEntryForConstraintID) +
				A.block(matrixEntryForConstraintID, 0, 3, problemSize) * initialGuessAndSolution;
			m_lhsEnforcedLocalSystem.block(0, systemSizeWithoutConstraintID, systemSizeWithoutConstraintID, 3) =
				A.block(0, matrixEntryForConstraintID, systemSizeWithoutConstraintID, 3);
			m_lhsEnforcedLocalSystem.block(systemSizeWithoutConstraintID, 0, 3, systemSizeWithoutConstraintID) =
				A.block(matrixEntryForConstraintID, 0, 3, systemSizeWithoutConstraintID);
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem.block(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID, 3, 3) =
				A.block(matrixEntryForConstraintID, matrixEntryForConstraintID, 3, 3);
			break;
		}

		default:
			SURGSIM_LOG_SEVERE(m_logger) << "MlcpGaussSeidelSolver::computeEnforcementSystem  Unknown constraint !?";
			break;
		}
	}

}

// Solve the system A x = b for x, with the assumption that the size is "size"
static inline void solveSystem(const MlcpProblem::Matrix& A, const MlcpProblem::Vector& b, size_t size,
							   MlcpSolution::Vector* x)
{
	MlcpProblem::Matrix AA = A.block(0, 0, size, size);
	MlcpProblem::Vector bb = b.head(size);

	MlcpSolution::Vector solution = AA.partialPivLu().solve(bb);
	//MlcpSolution::Vector solution = AA.colPivHouseholderQr().solve(bb);
	//MlcpSolution::Vector solution = AA.householderQr().solve(bb);
	*x = solution;
}

void MlcpGaussSeidelSolver::doOneIteration(size_t problemSize, const MlcpProblem::Matrix& A,
										   const MlcpProblem::Vector& b,
										   MlcpSolution::Vector* initialGuessAndSolution,
										   const MlcpProblem::Vector& frictionCoefs,
										   const std::vector<MlcpConstraintType>& constraintsType,
										   double constraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES],
										   double* convergenceCriteria, bool* validSignorini)
{
	for (size_t constraint = 0; constraint < MLCP_NUM_CONSTRAINT_TYPES; ++constraint)
	{
		constraintConvergenceCriteria[constraint] = 0.0;
	}
	*convergenceCriteria = 0.0;
	*validSignorini = true;

	size_t currentAtomicIndex = 0;
	size_t nbConstraints = constraintsType.size();

	// For each constraint, we look if the constraint is violated or not !
	for (size_t constraint = 0; constraint < nbConstraints; ++constraint)
	{
		switch (constraintsType[constraint])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			(*initialGuessAndSolution)[currentAtomicIndex] -=
				(b[currentAtomicIndex] + A.row(currentAtomicIndex) * (*initialGuessAndSolution)) /
				A(currentAtomicIndex, currentAtomicIndex);
			++currentAtomicIndex;
			break;
		}

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			(*initialGuessAndSolution).segment<2>(currentAtomicIndex) -=
				A.block<2, 2>(currentAtomicIndex, currentAtomicIndex).inverse() *
				(b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * (*initialGuessAndSolution));
			currentAtomicIndex += 2;
			break;
		}

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			(*initialGuessAndSolution).segment<3>(currentAtomicIndex) -=
				A.block<3, 3>(currentAtomicIndex, currentAtomicIndex).inverse() *
				(b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * (*initialGuessAndSolution));
			currentAtomicIndex += 3;
			break;
		}

		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			// Form the local system
			computeEnforcementSystem(problemSize, A, b, *initialGuessAndSolution, constraintsType,
				constraint, currentAtomicIndex);

			// Solve A.f = violation
			solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
				&m_rhsEnforcedLocalSystem);

			// Correct the forces accordingly
			(*initialGuessAndSolution).head(m_numEnforcedAtomicConstraints - 1) -=
				m_rhsEnforcedLocalSystem.head(m_numEnforcedAtomicConstraints - 1);

			double& Fn  = (*initialGuessAndSolution)[currentAtomicIndex];
			Fn -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 1];

			if (Fn < 0.0)
			{
				Fn = 0;      // inactive contact on normal
			}
			++currentAtomicIndex;
			break;
		}

		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			// Form the local system
			computeEnforcementSystem(problemSize, A, b, *initialGuessAndSolution, constraintsType,
				constraint, currentAtomicIndex);

			// Solve A.f = violation
			solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
				&m_rhsEnforcedLocalSystem);

			// Correct the forces accordingly
			(*initialGuessAndSolution).head(m_numEnforcedAtomicConstraints - 1) -=
				m_rhsEnforcedLocalSystem.head(m_numEnforcedAtomicConstraints - 1);

			double& Fn  = (*initialGuessAndSolution)[currentAtomicIndex];
			Eigen::VectorBlock<MlcpSolution::Vector, 2> Ft =
				(*initialGuessAndSolution).segment<2>(currentAtomicIndex + 1);
			Fn -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 1];

			if (Fn > 0.0)
			{
				// Compute the frictions violation
				Ft -= 2.0 * (b.segment<2>(currentAtomicIndex + 1) +
					A.block(currentAtomicIndex + 1, 0, 2, problemSize) * (*initialGuessAndSolution)) /
					(A(currentAtomicIndex + 1, currentAtomicIndex + 1) +
					A(currentAtomicIndex + 2, currentAtomicIndex + 2));

				const double maxFriction = frictionCoefs[constraint] * Fn;
				if (Ft.norm() > maxFriction)
				{
					// Here, the Friction is too strong, we keep the direction, but modulate its length
					// to verify the Coulomb's law: |Ft| = mu |Fn|
					Ft = Ft.normalized() * maxFriction;
				}
			}
			else
			{
				Fn = 0;      // inactive contact on normal
				// inactive contact on tangent
				Ft.setZero();
			}
			currentAtomicIndex += 3;
			break;
		}

		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			// Form the local system
			computeEnforcementSystem(problemSize, A, b, *initialGuessAndSolution, constraintsType,
				constraint, currentAtomicIndex);

			// Solve A.f = violation
			solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
				&m_rhsEnforcedLocalSystem);

			// Correct the forces accordingly
			(*initialGuessAndSolution).head(m_numEnforcedAtomicConstraints - 2) -=
				m_rhsEnforcedLocalSystem.head(m_numEnforcedAtomicConstraints - 2);
			(*initialGuessAndSolution).segment<2>(currentAtomicIndex) -=
				m_rhsEnforcedLocalSystem.segment<2>(m_numEnforcedAtomicConstraints - 2);
			currentAtomicIndex += 2;
			break;
		}

		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			// Form the local system
			computeEnforcementSystem(problemSize, A, b, *initialGuessAndSolution, constraintsType,
				constraint, currentAtomicIndex);

			// Solve A.f = violation
			solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
				&m_rhsEnforcedLocalSystem);

			// Correct the forces accordingly
			(*initialGuessAndSolution).head(m_numEnforcedAtomicConstraints - 2) -=
				m_rhsEnforcedLocalSystem.head(m_numEnforcedAtomicConstraints - 2);
			Eigen::VectorBlock<MlcpSolution::Vector, 2> Fn =
				(*initialGuessAndSolution).segment<2>(currentAtomicIndex);
			Fn -= m_rhsEnforcedLocalSystem.segment<2>(m_numEnforcedAtomicConstraints - 2);

			// No Signorini to verify here, it is NOT a unilateral constraint, but bilateral
			{
				// Complete the violation of the friction along t, with the missing terms...
				double& Ft = (*initialGuessAndSolution)[currentAtomicIndex + 2];
				Ft -= (b[currentAtomicIndex + 2] + A.row(currentAtomicIndex + 2) * (*initialGuessAndSolution)) /
					A(currentAtomicIndex + 2, currentAtomicIndex + 2);

				const double maxFriction = frictionCoefs[constraint] * Fn.norm();
				const double ftNorm = fabs(Ft);
				if (ftNorm > maxFriction)
				{
					// Here, the Friction is too strong, we keep the direction, but modulate its length
					// to verify the Coulomb's law: |Ft| = mu |Fn|
					Ft *= maxFriction / ftNorm;
				}
			}

			currentAtomicIndex += 3;
			break;
		}

		default:
			SURGSIM_FAILURE() << "unknown constraint type [" << constraintsType[constraint] << "]";
			break;
		}
	}
}

void MlcpGaussSeidelSolver::printViolationsAndConvergence(size_t problemSize,
														  const MlcpProblem::Matrix& A,
														  const MlcpProblem::Vector& b,
														  const MlcpSolution::Vector& initialGuessAndSolution,
														  const std::vector<MlcpConstraintType>& constraintsType,
														  double convergenceCriteria, bool validSignorini,
														  size_t iterations)
{
	SURGSIM_LOG_INFO(m_logger) << "MLCP at iteration " << iterations << " =";

	size_t currentAtomicIndex = 0;
	size_t nbConstraints = constraintsType.size();

	for (size_t constraint = 0; constraint < nbConstraints; ++constraint)
	{
		switch (constraintsType[constraint])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex] + A.row(currentAtomicIndex) * initialGuessAndSolution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint << "] of type BILATERAL_1D_CONSTRAINT" <<
				std::endl << "\t with initial violation b=(" << b[currentAtomicIndex] << ")" << std::endl <<
				"\t with final   violation b-Ax=(" << violation << ")" << std::endl <<
				"\t force=(" << initialGuessAndSolution[currentAtomicIndex] << ")";
			currentAtomicIndex += 1;
			break;
		}

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			Vector2d violation = b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * initialGuessAndSolution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint << "] of type BILATERAL_2D_CONSTRAINT" <<
				std::endl <<
				"\t with initial violation b=(" << b.segment<2>(currentAtomicIndex).transpose() << ") " << std::endl <<
				"\t with final   violation b-Ax=(" << violation.transpose() << ")" << std::endl <<
				"\t force=(" << initialGuessAndSolution.segment<2>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 2;
			break;
		}

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			Vector3d violation = b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * initialGuessAndSolution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint << "] of type BILATERAL_3D_CONSTRAINT" <<
				std::endl <<
				"\t with initial violation b=(" << b.segment<3>(currentAtomicIndex).transpose() << ")" << std::endl <<
				"\t with final   violation b-Ax=(" << violation.transpose() << ") " << std::endl <<
				"\t force=(" << initialGuessAndSolution.segment<3>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 3;
			break;
		}

		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex] + A.row(currentAtomicIndex) * initialGuessAndSolution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint <<
				"] of type UNILATERAL_FRICTIONLESS_CONSTRAINT" <<
				std::endl << "\t with initial violation b=(" << b[currentAtomicIndex] << ") " << std::endl <<
				"\t with final   violation b-Ax=(" << violation << ") ";
			if (violation < -m_contactTolerance)
			{
				SURGSIM_LOG_INFO(m_logger) << "\t  => normal violation = " << violation <<
					" < -m_contactTolerance => Signorini not verified yet !";
			}
			SURGSIM_LOG_INFO(m_logger) << "\t force=(" << initialGuessAndSolution[currentAtomicIndex]  << ")";
			currentAtomicIndex += 1;
			break;
		}

		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			Vector3d violation = b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * initialGuessAndSolution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint <<
				"] of type UNILATERAL_3D_FRICTIONAL_CONSTRAINT" <<
				std::endl << "\t with initial violation b=(" << b.segment<3>(currentAtomicIndex).transpose() << ")" <<
				std::endl << "\t with final   violation b-Ax=(" << violation.transpose() << ")";
			if (violation[0] < -m_contactTolerance)
			{
				SURGSIM_LOG_INFO(m_logger) << "\t  => normal violation = " << violation[0] <<
					" < -contactTolerance => Signorini not verified yet !";
			}
			SURGSIM_LOG_INFO(m_logger) << "\t force=(" <<
				initialGuessAndSolution.segment<3>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 3;
			break;
		}

		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			Vector2d violation = b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * initialGuessAndSolution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint <<
				"] of type UNILATERAL_3D_FRICTIONLESS_SUTURING" <<
				std::endl << "\t with initial violation b=(" << b.segment<2>(currentAtomicIndex).transpose() << ") " <<
				std::endl << "\t with final   violation b-Ax=(" << violation.transpose() << ") " << std::endl <<
				"\t force=(" << initialGuessAndSolution.segment<2>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 2;
			break;
		}

		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			Vector3d violation = b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * initialGuessAndSolution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint <<
				"] of type UNILATERAL_3D_FRICTIONAL_SUTURING" <<
				std::endl << "\t with initial violation b=(" << b.segment<3>(currentAtomicIndex).transpose() << ") " <<
				std::endl << "\t with final   violation b-Ax=(" << violation.transpose() << ")" << std::endl <<
				"\t force=(" << initialGuessAndSolution.segment<3>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 3;
			break;
		}

		default:
			break;
		}
	}
	SURGSIM_LOG_INFO(m_logger) << "convergence_criteria=" << convergenceCriteria << "  Signorini verified=" <<
		(validSignorini ? "yes" : "NO");
}

};  // namespace Math
};  // namespace SurgSim

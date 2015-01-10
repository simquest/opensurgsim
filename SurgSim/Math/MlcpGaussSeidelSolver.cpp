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

void MlcpGaussSeidelSolver::setLogger(std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	m_logger = logger;
}

bool MlcpGaussSeidelSolver::solve(const MlcpProblem& problem, MlcpSolution* solution)
{
	size_t problemSize = problem.getSize();
	const MlcpProblem::Matrix& A = problem.A;
	const size_t nbColumnInA = A.cols();
	const MlcpProblem::Vector& b = problem.b;
	MlcpSolution::Vector& initialGuess_and_solution = solution->x;
	const MlcpProblem::Vector& frictionCoefs = problem.mu;
	const std::vector<MlcpConstraintType>& constraintsType = problem.constraintTypes;
	double subStep = 1.0;//XXX
	size_t* MLCP_nbIterations = &solution->numIterations;
	bool* validConvergence = &solution->validConvergence;
	bool* validSignorini = &solution->validSignorini;
	double* convergenceCriteria = &solution->convergenceCriteria;
	double* initialConvergenceCriteria = &solution->initialConvergenceCriteria;
	double* constraintConvergenceCriteria = solution->constraintConvergenceCriteria;
	double* initialConstraintConvergenceCriteria = solution->initialConstraintConvergenceCriteria;
	bool catchExplodingConvergenceCriteria = true;

	// Loop until it converges or maxIterations are reached
	size_t nbLoop = 0;

	double convergence_criteria;
	double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES];
	bool signorini_verified;
	bool signorini_valid;

	double initial_convergence_criteria = 0.0;
	double initial_constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES];
	bool initialSignoriniVerified = true;
	bool initialSignoriniValid = true;

	calculateConvergenceCriteria(problemSize, A, nbColumnInA, b,
								 initialGuess_and_solution, constraintsType, subStep,
								 initial_constraint_convergence_criteria, &initial_convergence_criteria,
								 &initialSignoriniVerified, &initialSignoriniValid);

	// If it is already converged, fill the output and return true.
	if (initial_convergence_criteria <= m_epsilonConvergence && initialSignoriniVerified)
	{
		if (validSignorini)
		{
			*validSignorini = initialSignoriniVerified;
		}
		if (validConvergence)
		{
			*validConvergence = true;
		}
		if (initialConvergenceCriteria)
		{
			*initialConvergenceCriteria = initial_convergence_criteria;
		}
		if (convergenceCriteria)
		{
			*convergenceCriteria = initial_convergence_criteria;
		}
		if (MLCP_nbIterations)
		{
			*MLCP_nbIterations = 0;
		}

		if (initialConstraintConvergenceCriteria)
		{
			for (size_t i = 0; i < MLCP_NUM_CONSTRAINT_TYPES; ++i)
			{
				initialConstraintConvergenceCriteria[i] = initial_constraint_convergence_criteria[i];
			}
		}

		if (constraintConvergenceCriteria)
		{
			for (size_t i = 0; i < MLCP_NUM_CONSTRAINT_TYPES; ++i)
			{
				initialConstraintConvergenceCriteria[i] = initial_constraint_convergence_criteria[i];
			}
		}

		return true;
	}

	do
	{
		doOneIteration(problemSize, A, nbColumnInA, b, &initialGuess_and_solution, frictionCoefs,
					   constraintsType, subStep, constraint_convergence_criteria, &convergence_criteria,
					   &signorini_verified);

		calculateConvergenceCriteria(problemSize, A, nbColumnInA, b,
									 initialGuess_and_solution, constraintsType, subStep,
									 constraint_convergence_criteria, &convergence_criteria,
									 &signorini_verified, &signorini_valid);
		++nbLoop;

		if (catchExplodingConvergenceCriteria)
		{
			// If we have an incredibly high convergence criteria value, the displacements are going to be very large,
			// causing problems in the next iteration, so we should break out here. The convergence_criteria should
			// really only be a couple order of magnitudes higher than epsilon.
			if (!SurgSim::Math::isValid(convergence_criteria) || convergence_criteria > 1.0)
			{
				SURGSIM_LOG_WARNING(m_logger) << "Convergence (" << convergence_criteria <<
					") is NaN, infinite, or greater than 1.0! MLCP is exploding after " << nbLoop <<
					" Gauss Seidel iterations!!";
				break;
			}
		}
	}
	while ((!signorini_verified ||
			(SurgSim::Math::isValid(convergence_criteria) && convergence_criteria>m_epsilonConvergence)) &&
		   nbLoop < m_maxIterations);

	if (MLCP_nbIterations)
	{
		*MLCP_nbIterations = nbLoop;
	}

	if (validConvergence)
	{
		*validConvergence = true;

		if (!SurgSim::Math::isValid(convergence_criteria) || convergence_criteria > 1.0)
		{
			*validConvergence = false;
		}

		if (convergence_criteria >= sqrt(m_epsilonConvergence))
		{
			SURGSIM_LOG_WARNING(m_logger) << "Convergence criteria (" << convergence_criteria <<
				") is greater than " << sqrt(m_epsilonConvergence) << " at end of " << nbLoop <<
				" Gauss Seidel iterations.";
		}

		if (convergence_criteria > initial_convergence_criteria)
		{
			SURGSIM_LOG_WARNING(m_logger) << "Convergence criteria (" << convergence_criteria <<
				") is greater than before " << nbLoop << " Gauss Seidel iterations (" <<
				initial_convergence_criteria << ").";
		}
	}

	if (validSignorini)
	{
		*validSignorini = true;

		if (!signorini_verified)
		{
			SURGSIM_LOG_WARNING(m_logger) << "Signorini not verified after " << nbLoop << " Gauss Seidel iterations.";
			*validSignorini = false;
		}
	}

	if (convergenceCriteria)
	{
		*convergenceCriteria = convergence_criteria;
	}
	if (initialConvergenceCriteria)
	{
		*initialConvergenceCriteria = initial_convergence_criteria;
	}

	if (convergence_criteria > m_epsilonConvergence || !SurgSim::Math::isValid(convergence_criteria) ||
		!signorini_valid)
	{
		return false;
	}
	return true;
}


void MlcpGaussSeidelSolver::calculateConvergenceCriteria(size_t problemSize, const MlcpProblem::Matrix& A,
													size_t nbColumnInA,
													const MlcpProblem::Vector& b,
													const MlcpSolution::Vector& initialGuess_and_solution,
													const std::vector<MlcpConstraintType>& constraintsType,
													double subStep,
													double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES],
													double* convergence_criteria,
													bool* signoriniVerified, bool* signoriniValid)
{
	// Calculate initial convergence criteria.
	for (size_t constraint = 0; constraint < MLCP_NUM_CONSTRAINT_TYPES; ++constraint)
	{
		constraint_convergence_criteria[constraint] = 0.0;
	}
	*convergence_criteria = 0.0;
	*signoriniVerified = true;
	*signoriniValid = true;

	size_t currentAtomicIndex = 0;
	const size_t nbConstraints = constraintsType.size();
	size_t nbNonContactConstraints = 0;

	for (size_t constraint = 0; constraint < nbConstraints; ++constraint)
	{
		switch (constraintsType[constraint])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex] * subStep;
			//XXX REWRITE
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation += A(currentAtomicIndex, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation * violation);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
		}
		currentAtomicIndex += 1;
		break;

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			// XXX REWRITE
			double violation[2] = { b[currentAtomicIndex] * subStep , b[currentAtomicIndex + 1] * subStep };
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex + 1, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0] * violation[0] + violation[1] * violation[1]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
		}
		currentAtomicIndex += 2;
		break;

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			double violation[3] =
			{
				b[currentAtomicIndex] * subStep,
				b[currentAtomicIndex + 1] * subStep,
				b[currentAtomicIndex + 2] * subStep
			};
			// XXX REWRITE
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex + 1, j) * initialGuess_and_solution[j];
				violation[2] += A(currentAtomicIndex + 2, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0]*violation[0] + violation[1]*violation[1] + violation[2]*violation[2]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
		}
		currentAtomicIndex += 3;
		break;

		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex] * subStep;
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation += A(currentAtomicIndex, j) * initialGuess_and_solution[j];
			}
			// Enforce orthogonality condition
			if (! SurgSim::Math::isValid(violation) || violation < -m_contactTolerance ||
				(initialGuess_and_solution[currentAtomicIndex] > m_epsilonConvergence &&
				 violation > m_contactTolerance))
			{
				*signoriniVerified = false;
			}
		}
		currentAtomicIndex += 1;
		break;

		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex] * subStep;
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation += A(currentAtomicIndex, j) * initialGuess_and_solution[j];
			}
			// Enforce orthogonality condition
			if (! SurgSim::Math::isValid(violation) || violation < -m_contactTolerance ||
				(initialGuess_and_solution[currentAtomicIndex] > m_epsilonConvergence &&
				 violation > m_contactTolerance))
			{
				*signoriniVerified = false;
			}
		}
		currentAtomicIndex += 3;
		break;

		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			double violation[2] = { b[currentAtomicIndex] , b[currentAtomicIndex + 1] };
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex + 1, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0] * violation[0] + violation[1] * violation[1]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
		}
		currentAtomicIndex += 2;
		break;

		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			// We verify that the sliding point is on the line...no matter what the friction violation is
			// (3rd component)
			double violation[2] = { b[currentAtomicIndex], b[currentAtomicIndex + 1] };
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex + 1, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0] * violation[0] + violation[1] * violation[1]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[constraint]] += criteria;

			++nbNonContactConstraints;
		}
		currentAtomicIndex += 3;
		break;

		default:
			SURGSIM_FAILURE() << "unknown constraint type [" << constraintsType[constraint] << "]";
			break;
		}

	}

	if (nbNonContactConstraints > 0)
	{
		*convergence_criteria /= nbNonContactConstraints;    // normalize if necessary
	}
}

void MlcpGaussSeidelSolver::computeEnforcementSystem(
	size_t problemSize, const MlcpProblem::Matrix& A, size_t nbColumnInA, const MlcpProblem::Vector& b,
	const MlcpSolution::Vector& initialGuess_and_solution,
	const MlcpProblem::Vector& frictionCoefs,
	const std::vector<MlcpConstraintType>& constraintsType, double subStep,
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
				systemSize += 1;
				systemSizeWithoutConstraintID += 1;
				break;
			case MLCP_BILATERAL_2D_CONSTRAINT:
				systemSize += 2;
				systemSizeWithoutConstraintID += 2;
				break;
			case MLCP_BILATERAL_3D_CONSTRAINT:
				systemSize += 3;
				systemSizeWithoutConstraintID += 3;
				break;
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
		case MLCP_BILATERAL_1D_CONSTRAINT                  :
			systemSize += 1;
			break; // That should not be the case...
		case MLCP_BILATERAL_2D_CONSTRAINT                  :
			systemSize += 2;
			break; // That should not be the case...
		case MLCP_BILATERAL_3D_CONSTRAINT                  :
			systemSize += 3;
			break; // That should not be the case...
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT    :
			systemSize += 1;
			break;
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT      :
			systemSize += 1;
			break; // The system will solve the normal contact, not the frictional parts !
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
			systemSize += 2;
			break;
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT  :
			systemSize += 2;
			break; // The system will solve the sliding case, not the frictional part !
		default:
			SURGSIM_LOG_SEVERE(m_logger) << "MlcpGaussSeidelSolver::computeEnforcementSystem  Unkown constraint !?";
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
		for (size_t line = 0; line < systemSizeWithoutConstraintID; ++line)
		{
			// At the same time, we compute the violation for the constraints
			m_rhsEnforcedLocalSystem[line] = b[line] * subStep;
			for (size_t column = 0; column < systemSizeWithoutConstraintID; ++column)
			{
				m_lhsEnforcedLocalSystem(line, column) = A(line, column);
				m_rhsEnforcedLocalSystem[line] += A(line, column) * initialGuess_and_solution[column];
			}
			// Now we complete the violation[line] computation by taking into account the effect of all remaining
			// contacts/slidings/constraints
			for (size_t column = systemSizeWithoutConstraintID; column < problemSize; ++column)
			{
				m_rhsEnforcedLocalSystem[line] += A(line, column) * initialGuess_and_solution[column];
			}
		}

		// Now we complete the contact matrix by adding the coupling constraint/{contact|sliding} and the compliance
		// for {contact|sliding}
		switch (constraintsType[constraintID])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] = b[matrixEntryForConstraintID] * subStep;
			for (size_t line = 0; line < systemSizeWithoutConstraintID; ++line)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, line) = A(matrixEntryForConstraintID, line);
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] +=
					A(matrixEntryForConstraintID, line) * initialGuess_and_solution[line];
			}
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID);
			//...and complete the violation
			for (size_t column = systemSizeWithoutConstraintID; column < problemSize; ++column)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] +=
					A(matrixEntryForConstraintID, column) * initialGuess_and_solution[column];
			}
		}
		break; // That should not be the case...

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] = b[matrixEntryForConstraintID  ] * subStep;
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] = b[matrixEntryForConstraintID+1] * subStep;
			for (size_t line=0; line < systemSizeWithoutConstraintID; ++line)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID + 1) =
					A(line, matrixEntryForConstraintID + 1);

				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,     line) =
					A(matrixEntryForConstraintID,   line);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, line) =
					A(matrixEntryForConstraintID + 1, line);

				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID,   line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] +=
					A(matrixEntryForConstraintID + 1, line) * initialGuess_and_solution[line];
			}
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID + 1) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID + 1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID + 1, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, systemSizeWithoutConstraintID + 1) =
				A(matrixEntryForConstraintID + 1, matrixEntryForConstraintID + 1);
			//...and complete the violation
			for (size_t column = systemSizeWithoutConstraintID; column < problemSize; ++column)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID,   column) * initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] +=
					A(matrixEntryForConstraintID + 1, column) * initialGuess_and_solution[column];
			}
		}
		break; // That should not be the case...

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] = b[matrixEntryForConstraintID  ] * subStep;
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] = b[matrixEntryForConstraintID + 1] * subStep;
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 2] = b[matrixEntryForConstraintID + 2] * subStep;
			for (size_t line = 0; line < systemSizeWithoutConstraintID; ++line)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID + 1) =
					A(line, matrixEntryForConstraintID + 1);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID + 2) =
					A(line, matrixEntryForConstraintID + 2);

				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   line) = A(matrixEntryForConstraintID,   line);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, line) =
					A(matrixEntryForConstraintID + 1, line);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 2, line) =
					A(matrixEntryForConstraintID + 2, line);

				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID, line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] +=
					A(matrixEntryForConstraintID + 1, line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 2] +=
					A(matrixEntryForConstraintID + 2, line) * initialGuess_and_solution[line];
			}
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID + 1) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID + 1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID + 2) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID + 2);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID + 1, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, systemSizeWithoutConstraintID + 1) =
				A(matrixEntryForConstraintID + 1, matrixEntryForConstraintID + 1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, systemSizeWithoutConstraintID + 2) =
				A(matrixEntryForConstraintID + 1, matrixEntryForConstraintID + 2);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 2, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID + 2, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 2, systemSizeWithoutConstraintID + 1) =
				A(matrixEntryForConstraintID + 2, matrixEntryForConstraintID + 1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 2, systemSizeWithoutConstraintID + 2) =
				A(matrixEntryForConstraintID + 2, matrixEntryForConstraintID + 2);
			//...and complete the violation
			for (size_t column = systemSizeWithoutConstraintID; column < problemSize; ++column)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID,   column)*initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] +=
					A(matrixEntryForConstraintID + 1, column)*initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 2] +=
					A(matrixEntryForConstraintID + 2, column)*initialGuess_and_solution[column];
			}
		}
		break; // That should not be the case...

		// In any case of contact, we only register the normal part...the friction part is computed afterward !
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] = b[matrixEntryForConstraintID] * subStep;
			for (size_t line = 0; line < systemSizeWithoutConstraintID; ++line)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, line) = A(matrixEntryForConstraintID, line);

				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] +=
					A(matrixEntryForConstraintID, line) * initialGuess_and_solution[line];
			}

			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID);
			//...and complete the violation for the normal contact constraint
			for (size_t column = systemSizeWithoutConstraintID; column < problemSize; ++column)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] +=
					A(matrixEntryForConstraintID, column) * initialGuess_and_solution[column];
			}
		}
		break;

		// In any case of sliding, we only register the normals part...the friction part along the tangent is computed
		// afterward !
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			// Coupling part  (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] = b[matrixEntryForConstraintID  ] * subStep;
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] = b[matrixEntryForConstraintID + 1] * subStep;
			for (size_t line = 0; line < systemSizeWithoutConstraintID; ++line)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID + 1) =
					A(line, matrixEntryForConstraintID + 1);

				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   line) = A(matrixEntryForConstraintID,   line);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, line) =
					A(matrixEntryForConstraintID + 1, line);

				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID, line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] +=
					A(matrixEntryForConstraintID + 1, line) * initialGuess_and_solution[line];
			}

			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID + 1) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID + 1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID + 1, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID + 1, systemSizeWithoutConstraintID + 1) =
				A(matrixEntryForConstraintID + 1, matrixEntryForConstraintID + 1);
			//...and complete the violation for the normal contact constraints
			for (size_t column = systemSizeWithoutConstraintID; column < problemSize; ++column)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID, column) * initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID + 1] +=
					A(matrixEntryForConstraintID + 1, column) * initialGuess_and_solution[column];
			}

		}
		break;

		default:
			SURGSIM_LOG_SEVERE(m_logger) << "MlcpGaussSeidelSolver::computeEnforcementSystem  Unkown constraint !?";
			break;
		}
	}

}

// Solve the system A x = b for x, with the assumption that the size is "size"
static inline bool solveSystem(const MlcpProblem::Matrix& A, const MlcpProblem::Vector& b, size_t size,
							   MlcpSolution::Vector* x)
{
	MlcpProblem::Matrix AA = A.block(0, 0, size, size);
	MlcpProblem::Vector bb = b.head(size);

	MlcpSolution::Vector solution = AA.partialPivLu().solve(bb);
	//MlcpSolution::Vector solution = AA.colPivHouseholderQr().solve(bb);
	//MlcpSolution::Vector solution = AA.householderQr().solve(bb);
	*x = solution;
	return true;
}

void MlcpGaussSeidelSolver::doOneIteration(size_t problemSize, const MlcpProblem::Matrix& A, size_t nbColumnInA,
										   const MlcpProblem::Vector& b,
										   MlcpSolution::Vector* initialGuess_and_solution,
										   const MlcpProblem::Vector& frictionCoefs,
										   const std::vector<MlcpConstraintType>& constraintsType, double subStep,
										   double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES],
										   double* convergence_criteria, bool* signoriniVerified)
{
	for (size_t constraint = 0; constraint < MLCP_NUM_CONSTRAINT_TYPES; ++constraint)
	{
		constraint_convergence_criteria[constraint] = 0.0;
	}
	*convergence_criteria = 0.0;
	*signoriniVerified = true;

	size_t currentAtomicIndex = 0;
	size_t nbConstraints = constraintsType.size();

	// For each constraint, we look if the constraint is violated or not !
	for (size_t constraint = 0; constraint < nbConstraints; ++constraint)
	{
		switch (constraintsType[constraint])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			double& F  = (*initialGuess_and_solution)[currentAtomicIndex];
			double violation = b[currentAtomicIndex] * subStep;
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation += A(currentAtomicIndex, j) * (*initialGuess_and_solution)[j];
			}
			F -= violation / A(currentAtomicIndex, currentAtomicIndex);
		}
		++currentAtomicIndex;
		break;

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			double& F1  = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& F2  = (*initialGuess_and_solution)[currentAtomicIndex + 1];
			double violation[2] = { b[currentAtomicIndex] * subStep , b[currentAtomicIndex + 1] * subStep };
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation[0] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
				violation[1] += A(currentAtomicIndex + 1, j) * (*initialGuess_and_solution)[j];
			}
			// det = ad-bc
			// [ a b ]   [  d -b ]       [ 1 0 ]
			// [ c d ] . [ -c  a ]/det = [ 0 1 ]
			double A_determinant =
				A(currentAtomicIndex, currentAtomicIndex) * A(currentAtomicIndex + 1, currentAtomicIndex + 1) -
				A(currentAtomicIndex, currentAtomicIndex + 1) * A(currentAtomicIndex + 1, currentAtomicIndex);
			double Ainv[2][2] =
			{
				{
					A(currentAtomicIndex + 1, currentAtomicIndex + 1) / A_determinant,
					-A(currentAtomicIndex,   currentAtomicIndex + 1) / A_determinant
				},
				{
					-A(currentAtomicIndex + 1, currentAtomicIndex) / A_determinant,
					A(currentAtomicIndex,   currentAtomicIndex) / A_determinant
				}
			};
			F1 -= (Ainv[0][0]*violation[0] + Ainv[0][1]*violation[1]);
			F2 -= (Ainv[1][0]*violation[0] + Ainv[1][1]*violation[1]);
		}
		currentAtomicIndex += 2;
		break;

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			double& F1  = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& F2  = (*initialGuess_and_solution)[currentAtomicIndex + 1];
			double& F3  = (*initialGuess_and_solution)[currentAtomicIndex + 2];
			double violation[3] = { b[currentAtomicIndex] * subStep, b[currentAtomicIndex + 1] * subStep,
									b[currentAtomicIndex + 2] * subStep
								  };
			for (size_t j = 0; j < problemSize; ++j)
			{
				violation[0] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
				violation[1] += A(currentAtomicIndex + 1, j) * (*initialGuess_and_solution)[j];
				violation[2] += A(currentAtomicIndex + 2, j) * (*initialGuess_and_solution)[j];
			}
			Eigen::Matrix3d Ainv = A.block<3,3>(currentAtomicIndex, currentAtomicIndex).inverse();
			F1 -= (Ainv(0, 0) * violation[0] + Ainv(0, 1) * violation[1] + Ainv(0, 2) * violation[2]);
			F2 -= (Ainv(1, 0) * violation[0] + Ainv(1, 1) * violation[1] + Ainv(1, 2) * violation[2]);
			F3 -= (Ainv(2, 0) * violation[0] + Ainv(2, 1) * violation[1] + Ainv(2, 2) * violation[2]);
		}
		currentAtomicIndex += 3;
		break;

		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			double& Fn  = (*initialGuess_and_solution)[currentAtomicIndex];

			// Form the local system
			computeEnforcementSystem(problemSize, A, nbColumnInA, b, *initialGuess_and_solution, frictionCoefs,
				constraintsType, subStep, constraint, currentAtomicIndex);

			// Solve A.f = violation
			if (!solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
				&m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (size_t i = 0; i < m_numEnforcedAtomicConstraints - 1; ++i)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 1];

			if (Fn<0.0)
			{
				Fn  = 0;      // inactive contact on normal
			}
		}
		++currentAtomicIndex;
		break;

		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			double local_mu = frictionCoefs[constraint];
			double& Fn  = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& Ft1 = (*initialGuess_and_solution)[currentAtomicIndex + 1];
			double& Ft2 = (*initialGuess_and_solution)[currentAtomicIndex + 2];

			// Form the local system
			computeEnforcementSystem(problemSize, A, nbColumnInA, b, *initialGuess_and_solution, frictionCoefs,
				constraintsType, subStep, constraint, currentAtomicIndex);

			// Solve A.f = violation
			if (! solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
							  &m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (size_t i = 0; i < m_numEnforcedAtomicConstraints - 1; ++i)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 1];

			if (Fn>0.0)
			{
				// Compute the frictions violation
				double violation[2]= { b[currentAtomicIndex + 1] * subStep, b[currentAtomicIndex + 2] * subStep };
				for (size_t i = 0; i < problemSize; ++i)
				{
					violation[0] += A(currentAtomicIndex + 1, i) * (*initialGuess_and_solution)[i];
					violation[1] += A(currentAtomicIndex + 2, i) * (*initialGuess_and_solution)[i];
				}

				Ft1 -= 2.0 * violation[0] / (A(currentAtomicIndex + 1, currentAtomicIndex + 1) +
									   A(currentAtomicIndex + 2, currentAtomicIndex + 2));
				Ft2 -= 2.0 * violation[1] / (A(currentAtomicIndex + 1, currentAtomicIndex + 1) +
									   A(currentAtomicIndex + 2, currentAtomicIndex + 2));

				double normFt = sqrt(Ft1 * Ft1 + Ft2 * Ft2);
				if (normFt>local_mu*Fn)
				{
					// Here, the Friction is too strong, we keep the direction, but modulate its length
					// to verify the Coulomb's law: |Ft| = mu |Fn|
					Ft1 *= local_mu*Fn/normFt;
					Ft2 *= local_mu*Fn/normFt;
				}
			}
			else
			{
				Fn  = 0;      // inactive contact on normal
				Ft1 = 0;      // inactive contact on tangent
				Ft2 = 0;      // inactive contact on tangent
			}
		}
		currentAtomicIndex += 3;
		break;

		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			double& Fn1 = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& Fn2 = (*initialGuess_and_solution)[currentAtomicIndex + 1];

			// Form the local system
			computeEnforcementSystem(problemSize, A, nbColumnInA, b, *initialGuess_and_solution, frictionCoefs,
				constraintsType, subStep, constraint, currentAtomicIndex);

			// Solve A.f = violation
			if (! solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
							  &m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (size_t i = 0; i < m_numEnforcedAtomicConstraints - 2; ++i)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn1 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 2];
			Fn2 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 1];
		}
		currentAtomicIndex += 2;
		break;

		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			double local_mu = frictionCoefs[constraint];
			double& Fn1 = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& Fn2 = (*initialGuess_and_solution)[currentAtomicIndex + 1];
			double& Ft  = (*initialGuess_and_solution)[currentAtomicIndex + 2];

			// Form the local system
			computeEnforcementSystem(problemSize, A, nbColumnInA, b, *initialGuess_and_solution, frictionCoefs,
				constraintsType, subStep, constraint, currentAtomicIndex);

			// Solve A.f = violation
			if (! solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
							  &m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (size_t i = 0; i < m_numEnforcedAtomicConstraints - 2 ; ++i)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn1 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 2];
			Fn2 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints - 1];

			// No Signorini to verify here, it is NOT a unilateral constraint, but bilateral
			{
				// Complete the violation of the friction along t, with the missing terms...
				double violation = b[currentAtomicIndex + 2] * subStep;
				for (size_t i = 0; i < problemSize; ++i)
				{
					violation += A(currentAtomicIndex + 2, i) * (*initialGuess_and_solution)[i];
				}

				Ft -= violation/A(currentAtomicIndex + 2, currentAtomicIndex + 2);

				double normFn = sqrt(Fn1*Fn1 + Fn2*Fn2);
				double normFt = fabs(Ft);
				if (normFt>local_mu*normFn)
				{
					// Here, the Friction is too strong, we keep the direction, but modulate its length
					// to verify the Coulomb's law: |Ft| = mu |Fn|
					Ft *= local_mu*normFn/normFt;
				}
			}
		}
		currentAtomicIndex += 3;
		break;

		default:
			SURGSIM_FAILURE() << "unknown constraint type [" << constraintsType[constraint] << "]";
			break;
		}
	}
}

void MlcpGaussSeidelSolver::printViolationsAndConvergence(size_t problemSize,
														  const MlcpProblem::Matrix& A,
														  const MlcpProblem::Vector& b,
														  const MlcpSolution::Vector& initialGuess_and_solution,
														  const std::vector<MlcpConstraintType>& constraintsType,
														  double subStep,
														  double convergence_criteria, bool signorini_verified,
														  size_t nbLoop)
{
	SURGSIM_LOG_INFO(m_logger) << "MLCP at iteration " << nbLoop << " =";

	size_t currentAtomicIndex = 0;
	size_t nbConstraints = constraintsType.size();

	for (size_t constraint = 0; constraint < nbConstraints; ++constraint)
	{
		switch (constraintsType[constraint])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex] + A.row(currentAtomicIndex) * initialGuess_and_solution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint << "] of type BILATERAL_1D_CONSTRAINT" <<
				std::endl << "\t with initial violation b=(" << b[currentAtomicIndex] << ")" << std::endl <<
				"\t with final   violation b-Ax=(" << violation << ")" << std::endl <<
				"\t force=(" << initialGuess_and_solution[currentAtomicIndex] << ")";
			currentAtomicIndex += 1;
		}
		break;
		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			Vector2d violation = b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * initialGuess_and_solution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint << "] of type BILATERAL_2D_CONSTRAINT" <<
				std::endl <<
				"\t with initial violation b=(" << b.segment<2>(currentAtomicIndex).transpose() << ") " << std::endl <<
				"\t with final   violation b-Ax=(" << violation.transpose() << ")" << std::endl <<
				"\t force=(" << initialGuess_and_solution.segment<2>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 2;
		}
		break;
		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			Vector3d violation = b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * initialGuess_and_solution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint << "] of type BILATERAL_3D_CONSTRAINT" <<
				std::endl <<
				"\t with initial violation b=(" << b.segment<3>(currentAtomicIndex).transpose() << ")" << std::endl <<
				"\t with final   violation b-Ax=(" << violation.transpose() << ") " << std::endl <<
				"\t force=(" << initialGuess_and_solution.segment<3>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 3;
		}
		break;
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex] + A.row(currentAtomicIndex) * initialGuess_and_solution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint <<
				"] of type UNILATERAL_FRICTIONLESS_CONSTRAINT" <<
				std::endl << "\t with initial violation b=(" << b[currentAtomicIndex] << ") " << std::endl <<
				"\t with final   violation b-Ax=(" << violation << ") ";
			if (violation < -m_contactTolerance)
			{
				SURGSIM_LOG_INFO(m_logger) << "\t  => normal violation = " << violation <<
					" < -m_contactTolerance => Signorini not verified yet !";
			}
			SURGSIM_LOG_INFO(m_logger) << "\t force=(" << initialGuess_and_solution[currentAtomicIndex]  << ")";
			currentAtomicIndex += 1;
		}
		break;
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			Vector3d violation = b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * initialGuess_and_solution;
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
				initialGuess_and_solution.segment<3>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 3;
		}
		break;
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			Vector2d violation = b.segment<2>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 2, problemSize) * initialGuess_and_solution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint <<
				"] of type UNILATERAL_3D_FRICTIONLESS_SUTURING" <<
				std::endl << "\t with initial violation b=(" << b.segment<2>(currentAtomicIndex).transpose() << ") " <<
				std::endl << "\t with final   violation b-Ax=(" << violation.transpose() << ") " << std::endl <<
				"\t force=(" << initialGuess_and_solution.segment<2>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 2;
		}
		break;
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			Vector3d violation = b.segment<3>(currentAtomicIndex) +
				A.block(currentAtomicIndex, 0, 3, problemSize) * initialGuess_and_solution;
			SURGSIM_LOG_INFO(m_logger) << "Constraint [" << constraint <<
				"] of type UNILATERAL_3D_FRICTIONAL_SUTURING" <<
				std::endl << "\t with initial violation b=(" << b.segment<3>(currentAtomicIndex).transpose() << ") " <<
				std::endl << "\t with final   violation b-Ax=(" << violation.transpose() << ")" << std::endl <<
				"\t force=(" << initialGuess_and_solution.segment<3>(currentAtomicIndex).transpose() << ")";
			currentAtomicIndex += 3;
		}
		break;
		default:
			break;
		}
	}
	SURGSIM_LOG_INFO(m_logger) << "convergence_criteria=" << convergence_criteria << "  Signorini verified=" <<
		(signorini_verified ? "yes" : "NO");
}

};  // namespace Math
};  // namespace SurgSim

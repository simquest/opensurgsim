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
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

#include "SurgSim/Math/Valid.h"
#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Math
{

// cf. Christian Duriez TVCG05 paper
// Realistic Haptic Rendering of Interacting
// Deformable Objects in Virtual Environments
// Christian Duriez, Student Member, IEEE, Frederic Dubois,
// Abderrahmane Kheddar, Member, IEEE, and Claude Andriot
// +
// recent work from Christian

//! MLCP_GaussSeidel::solve Resolution of a given MLCP (Gauss Seidel iterative solver)
/*!
@param problem The mlcp problem
@param solution The mlcp solution
*/
bool MlcpGaussSeidelSolver::solve(const MlcpProblem& problem, MlcpSolution* solution)
{
	// TODO(wschoen) 2014-06-04: Modify to word-sized types
	int n = static_cast<int>(problem.getSize());
	const MlcpProblem::Matrix& A = problem.A;
	const int nbColumnInA = static_cast<int>(A.cols());
	const MlcpProblem::Vector& b = problem.b;
	MlcpSolution::Vector& initialGuess_and_solution = solution->x;
	const MlcpProblem::Vector& frictionCoefs = problem.mu;
	const std::vector<MlcpConstraintType>& constraintsType = problem.constraintTypes;
	double subStep = 1.0;//XXX
	int* MLCP_nbIterations = &solution->numIterations;
	bool* validConvergence = &solution->validConvergence;
	bool* validSignorini = &solution->validSignorini;
	double* convergenceCriteria = &solution->convergenceCriteria;
	double* initialConvergenceCriteria = &solution->initialConvergenceCriteria;
	double* constraintConvergenceCriteria = solution->constraintConvergenceCriteria;
	double* initialConstraintConvergenceCriteria = solution->initialConstraintConvergenceCriteria;
	bool catchExplodingConvergenceCriteria = true;
	bool verbose = true;

	//int nbConstraints = static_cast<int>(constraintsType.size());
	//cout << "======== MLCP ========" << endl;
	//cout << "\tMLCP: nbAtomicEntry (nb Line In Matrix)="<<n<<" - nbConstraints=" << nbConstraints << endl;

	// Loop until it converges or maxIterations are reached
	unsigned int nbLoop=0;

	double convergence_criteria;
	double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES];
	bool signorini_verified;
	bool signorini_valid;

	double initial_convergence_criteria = 0.0;
	double initial_constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES];
	bool initialSignoriniVerified = true;
	bool initialSignoriniValid = true;

	calculateConvergenceCriteria(n, A, nbColumnInA, b,
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
			for (int i = 0; i < MLCP_NUM_CONSTRAINT_TYPES; i++)
			{
				initialConstraintConvergenceCriteria[i] = initial_constraint_convergence_criteria[i];
			}
		}

		if (constraintConvergenceCriteria)
		{
			for (int i = 0; i < MLCP_NUM_CONSTRAINT_TYPES; i++)
			{
				initialConstraintConvergenceCriteria[i] = initial_constraint_convergence_criteria[i];
			}
		}

		return true;
	}

	do
	{
		doOneIteration(n, A, nbColumnInA, b, &initialGuess_and_solution, frictionCoefs,
					   constraintsType, subStep, constraint_convergence_criteria, &convergence_criteria,
					   &signorini_verified);

		calculateConvergenceCriteria(n, A, nbColumnInA, b,
									 initialGuess_and_solution, constraintsType, subStep,
									 constraint_convergence_criteria, &convergence_criteria,
									 &signorini_verified, &signorini_valid);

// 	  printViolationsAndConvergence(n, A, nbColumnInA, b, initialGuess_and_solution, constraintsType, subStep,
// 		  convergence_criteria, signorini_verified, nbLoop);

		nbLoop++;

		if (catchExplodingConvergenceCriteria)
		{
			// If we have an incredibly high convergence criteria value, the displacements are going to be very large,
			// causing problems in the next iteration, so we should break out here. The convergence_criteria should
			// really only be a couple order of magnitudes higher than epsilon.
			if (!SurgSim::Math::isValid(convergence_criteria) || convergence_criteria > 1.0)
			{
				printf("Convergence (%e) is NaN, infinite, or greater than 1.0! MLCP is exploding"
					   " after %d Gauss Seidel iterations!!\n", convergence_criteria, nbLoop);
				break;
			}
		}

		//printf("  Solver [loop %2d] convergence_criteria=%g<?%g\n",nbLoop,convergence_criteria,m_epsilonConvergence);

	}
	while ((!signorini_verified ||
			(SurgSim::Math::isValid(convergence_criteria) && convergence_criteria>m_epsilonConvergence)) &&
		   nbLoop<m_maxIterations);

	if (MLCP_nbIterations)
	{
		*MLCP_nbIterations=nbLoop;
	}

	if (validConvergence)
	{
		*validConvergence = true;

		if (!SurgSim::Math::isValid(convergence_criteria) || convergence_criteria > 1.0)
		{
			*validConvergence = false;
		}

		if (!(convergence_criteria < sqrt(m_epsilonConvergence)))
		{
			if (verbose)
			{
				printf("Convergence criteria (%e) is greater than %e at end of %d Gauss Seidel iterations.\n",
					   convergence_criteria, sqrt(m_epsilonConvergence), nbLoop);
			}
			//*validConvergence = false;
		}

		if (!(convergence_criteria <= initial_convergence_criteria))
		{
			if (verbose)
			{
				printf("Convergence criteria (%e) is greater than before %d Gauss Seidel iterations (%e).\n",
					   convergence_criteria, nbLoop, initial_convergence_criteria);
			}
			//		  *validConvergence = false;   // This is a bit strict but it is useful to know when diverging.
		}
	}

	if (validSignorini)
	{
		*validSignorini = true;

		if (!signorini_verified)
		{
			if (verbose)
			{
				printf("Signorini not verified after %d Gauss Seidel iterations.\n", nbLoop);
			}
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
//#ifdef PRINTOUT_TEST_APP
//		cout << "\tLCP_3DContactFriction_GaussSeidel_Christian::solve did <<<NOT>>> converge after " <<
//			nbLoop << " iterations" << endl;
//		cout << "\t  SignoriniVerified = "<< SignoriniVerified <<"   convergence_criteria = " <<
//			convergence_criteria << endl;
//
//		printViolationsAndConvergence(n, A, nbColumnInA, b, initialGuess_and_solution, constraintsType, subStep,
//			convergence_criteria, signorini_verified);
//#endif // PRINTOUT_TEST_APP

		return false;
	}
#ifdef PRINTOUT_TEST_APP
	else
	{
		std::cout << "LCP_3DContactFriction_GaussSeidel_Christian::solve converged after " <<
				  nbLoop << " iterations" << std::endl;
	}
#endif // PRINTOUT_TEST_APP

	return true;
}


void MlcpGaussSeidelSolver::calculateConvergenceCriteria(int n, const MlcpProblem::Matrix& A, int nbColumnInA,
														 const MlcpProblem::Vector& b,
														 const MlcpSolution::Vector& initialGuess_and_solution,
														 const std::vector<MlcpConstraintType>& constraintsType,
														 double subStep,
														 double constraint_convergence_criteria[],
														 double* convergence_criteria,
														 bool* signoriniVerified, bool* signoriniValid)
{
	// Calculate initial convergence criteria.

	for (int i = 0; i < MLCP_NUM_CONSTRAINT_TYPES; i++)
	{
		constraint_convergence_criteria[i] = 0.0;
	}
	*convergence_criteria = 0.0;
	*signoriniVerified = true;
	*signoriniValid = true;

	int currentAtomicIndex=0;
	int nbConstraints = static_cast<int>(constraintsType.size());

	int nbNonContactConstraints = 0;

	for (int i=0 ; i<nbConstraints ; i++)
	{
		switch (constraintsType[i])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex]*subStep;
			//XXX REWRITE
			for (int j=0 ; j<n ; j++)
			{
				violation += A(currentAtomicIndex, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation * violation);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[i]] += criteria;

			nbNonContactConstraints++;
		}
		currentAtomicIndex+=1;
		break;

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			// XXX REWRITE
			double violation[2] = { b[currentAtomicIndex]* subStep , b[currentAtomicIndex+1]* subStep };
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0]*violation[0] + violation[1]*violation[1]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[i]] += criteria;

			nbNonContactConstraints++;
		}
		currentAtomicIndex+=2;
		break;

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			double violation[3] =
			{
				b[currentAtomicIndex]* subStep , b[currentAtomicIndex+1]* subStep , b[currentAtomicIndex+2]* subStep
			};
			// XXX REWRITE
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
				violation[2] += A(currentAtomicIndex+2, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0]*violation[0] + violation[1]*violation[1] + violation[2]*violation[2]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[i]] += criteria;

			nbNonContactConstraints++;
		}
		currentAtomicIndex+=3;
		break;

		//case MLCP_BILATERAL_4D_CONSTRAINT:
		//...

		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex]*subStep;
			for (int j=0 ; j<n ; j++)
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
		currentAtomicIndex++;
		break;

		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			double violation = b[currentAtomicIndex]*subStep;
			for (int j=0 ; j<n ; j++)
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
		currentAtomicIndex+=3;
		break;

		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			double violation[2] = { b[currentAtomicIndex] , b[currentAtomicIndex+1] };
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0]*violation[0] + violation[1]*violation[1]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[i]] += criteria;

			nbNonContactConstraints++;
		}
		currentAtomicIndex+=2;
		break;

		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			// We verify that the sliding point is on the line...no matter what the friction violation is
			// (3rd component)
			double violation[2] = { b[currentAtomicIndex] , b[currentAtomicIndex+1] };
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
			}
			double criteria = sqrt(violation[0]*violation[0] + violation[1]*violation[1]);
			*convergence_criteria += criteria;
			constraint_convergence_criteria[constraintsType[i]] += criteria;

			nbNonContactConstraints++;
		}
		currentAtomicIndex+=3;
		break;

		default:
			//XXX
			SURGSIM_FAILURE() << "unknown constraint type [" << constraintsType[i] << "]";
			break;
		}

	}

	if (nbNonContactConstraints)
	{
		*convergence_criteria /= nbNonContactConstraints;    // normalize if necessary
	}
}

void MlcpGaussSeidelSolver::computeEnforcementSystem(
	int n, const MlcpProblem::Matrix& A, int nbColumnInA, const MlcpProblem::Vector& b,
	const MlcpSolution::Vector& initialGuess_and_solution,
	const MlcpProblem::Vector& frictionCoefs,
	const std::vector<MlcpConstraintType>& constraintsType, double subStep,
	int constraintID, int matrixEntryForConstraintID)
{
	int nbConstraints = static_cast<int>(constraintsType.size());
	int systemSize=0;                    // Total size of the system (number of line and column in the final matrix)
	int systemSizeWithoutConstraintID=0; // Total size of the system counting only the {1D,2D,3D} bilateral constraints

	// 1st) compute the size of the final system
	// We suppose that the constraint to enforce are only 1D, 2D or 3D bilateral constraints
	// and all appearing first in the list !
	{
		bool done=false;
		for (int i=0 ; i<nbConstraints ; i++)
		{
			switch (constraintsType[i])
			{
			case MLCP_BILATERAL_1D_CONSTRAINT:
				systemSize+=1;
				systemSizeWithoutConstraintID+=1;
				break;
			case MLCP_BILATERAL_2D_CONSTRAINT:
				systemSize+=2;
				systemSizeWithoutConstraintID+=2;
				break;
			case MLCP_BILATERAL_3D_CONSTRAINT:
				systemSize+=3;
				systemSizeWithoutConstraintID+=3;
				break;
			default:
				done=true;
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
			systemSize+=1;
			break; // That should not be the case...
		case MLCP_BILATERAL_2D_CONSTRAINT                  :
			systemSize+=2;
			break; // That should not be the case...
		case MLCP_BILATERAL_3D_CONSTRAINT                  :
			systemSize+=3;
			break; // That should not be the case...
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT    :
			systemSize+=1;
			break;
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT      :
			systemSize+=1;
			break; // The system will solve the normal contact, not the frictional parts !
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
			systemSize+=2;
			break;
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT  :
			systemSize+=2;
			break; // The system will solve the sliding case, not the frictional part !
		default:
			printf("MlcpGaussSeidelSolver::computeEnforcementSystem  Unkown constraint !?\n");
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
		for (int line=0 ; line<systemSizeWithoutConstraintID ; line++)
		{
			// At the same time, we compute the violation for the constraints
			m_rhsEnforcedLocalSystem[line] = b[line] * subStep;
			for (int column=0 ; column<systemSizeWithoutConstraintID ; column++)
			{
				m_lhsEnforcedLocalSystem(line, column) = A(line, column);
				m_rhsEnforcedLocalSystem[line] += A(line, column)*initialGuess_and_solution[column];
			}
			// Now we complete the violation[line] computation by taking into account the effect of all remaining
			// contacts/slidings/constraints
			for (int column=systemSizeWithoutConstraintID; column<n ; column++)
			{
				m_rhsEnforcedLocalSystem[line] += A(line, column)*initialGuess_and_solution[column];
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
			for (int line=0 ; line<systemSizeWithoutConstraintID ; line++)
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
			for (int column=systemSizeWithoutConstraintID; column<n ; column++)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] +=
					A(matrixEntryForConstraintID, column)*initialGuess_and_solution[column];
			}
		}
		break; // That should not be the case...

		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] = b[matrixEntryForConstraintID  ] * subStep;
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] = b[matrixEntryForConstraintID+1] * subStep;
			for (int line=0 ; line<systemSizeWithoutConstraintID ; line++)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID+1) = A(line, matrixEntryForConstraintID+1);

				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,     line) =
					A(matrixEntryForConstraintID,   line);
				m_lhsEnforcedLocalSystem((systemSizeWithoutConstraintID+1), line) =
					A(matrixEntryForConstraintID+1, line);

				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID,   line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] +=
					A(matrixEntryForConstraintID+1, line) * initialGuess_and_solution[line];
			}
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID+1) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID+1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID+1, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, systemSizeWithoutConstraintID+1) =
				A(matrixEntryForConstraintID+1, matrixEntryForConstraintID+1);
			//...and complete the violation
			for (int column=systemSizeWithoutConstraintID; column<n ; column++)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID,   column)*initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] +=
					A(matrixEntryForConstraintID+1, column)*initialGuess_and_solution[column];
			}
		}
		break; // That should not be the case...

		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] = b[matrixEntryForConstraintID  ] * subStep;
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] = b[matrixEntryForConstraintID+1] * subStep;
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+2] = b[matrixEntryForConstraintID+2] * subStep;
			for (int line=0 ; line<systemSizeWithoutConstraintID ; line++)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID+1) = A(line, matrixEntryForConstraintID+1);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID+2) = A(line, matrixEntryForConstraintID+2);

				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   line) = A(matrixEntryForConstraintID,   line);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, line) = A(matrixEntryForConstraintID+1, line);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+2, line) = A(matrixEntryForConstraintID+2, line);

				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID, line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] +=
					A(matrixEntryForConstraintID+1, line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+2] +=
					A(matrixEntryForConstraintID+2, line) * initialGuess_and_solution[line];
			}
			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID+1) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID+1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID, systemSizeWithoutConstraintID+2) =
				A(matrixEntryForConstraintID, matrixEntryForConstraintID+2);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID+1, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, systemSizeWithoutConstraintID+1) =
				A(matrixEntryForConstraintID+1, matrixEntryForConstraintID+1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, systemSizeWithoutConstraintID+2) =
				A(matrixEntryForConstraintID+1, matrixEntryForConstraintID+2);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+2, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID+2, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+2, systemSizeWithoutConstraintID+1) =
				A(matrixEntryForConstraintID+2, matrixEntryForConstraintID+1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+2, systemSizeWithoutConstraintID+2) =
				A(matrixEntryForConstraintID+2, matrixEntryForConstraintID+2);
			//...and complete the violation
			for (int column=systemSizeWithoutConstraintID; column<n ; column++)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID,   column)*initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] +=
					A(matrixEntryForConstraintID+1, column)*initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+2] +=
					A(matrixEntryForConstraintID+2, column)*initialGuess_and_solution[column];
			}
		}
		break; // That should not be the case...

		// In any case of contact, we only register the normal part...the friction part is computed afterward !
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			// Coupling part (fill up LHS and RHS)
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] = b[matrixEntryForConstraintID] * subStep;
			for (int line=0 ; line<systemSizeWithoutConstraintID ; line++)
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
			for (int column=systemSizeWithoutConstraintID; column<n ; column++)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID] +=
					A(matrixEntryForConstraintID, column)*initialGuess_and_solution[column];
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
			m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] = b[matrixEntryForConstraintID+1] * subStep;
			for (int line=0 ; line<systemSizeWithoutConstraintID ; line++)
			{
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID) = A(line, matrixEntryForConstraintID);
				m_lhsEnforcedLocalSystem(line, systemSizeWithoutConstraintID+1) = A(line, matrixEntryForConstraintID+1);

				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   line) = A(matrixEntryForConstraintID,   line);
				m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, line) = A(matrixEntryForConstraintID+1, line);

				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID, line) * initialGuess_and_solution[line];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] +=
					A(matrixEntryForConstraintID+1, line) * initialGuess_and_solution[line];
			}

			// Compliance part for the {contact|sliding}
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID,   systemSizeWithoutConstraintID+1) =
				A(matrixEntryForConstraintID,   matrixEntryForConstraintID+1);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, systemSizeWithoutConstraintID) =
				A(matrixEntryForConstraintID+1, matrixEntryForConstraintID);
			m_lhsEnforcedLocalSystem(systemSizeWithoutConstraintID+1, systemSizeWithoutConstraintID+1) =
				A(matrixEntryForConstraintID+1, matrixEntryForConstraintID+1);
			//...and complete the violation for the normal contact constraints
			for (int column=systemSizeWithoutConstraintID; column<n ; column++)
			{
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID  ] +=
					A(matrixEntryForConstraintID, column)*initialGuess_and_solution[column];
				m_rhsEnforcedLocalSystem[systemSizeWithoutConstraintID+1] +=
					A(matrixEntryForConstraintID+1, column)*initialGuess_and_solution[column];
			}

		}
		break;

		default:
			printf("MlcpGaussSeidelSolver::computeEnforcementSystem  Unkown constraint !?\n");
			break;
		}
	}

}

// Solve the system A x = b for x, with the assumption that the size is "size"
static inline bool solveSystem(const MlcpProblem::Matrix& A, const MlcpProblem::Vector& b, int size,
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

void MlcpGaussSeidelSolver::doOneIteration(int n, const MlcpProblem::Matrix& A, int nbColumnInA,
										   const MlcpProblem::Vector& b,
										   MlcpSolution::Vector* initialGuess_and_solution,
										   const MlcpProblem::Vector& frictionCoefs,
										   const std::vector<MlcpConstraintType>& constraintsType, double subStep,
										   double constraint_convergence_criteria[MLCP_NUM_CONSTRAINT_TYPES],
										   double* convergence_criteria, bool* signoriniVerified)
{
	for (int i = 0; i < MLCP_NUM_CONSTRAINT_TYPES; i++)
	{
		constraint_convergence_criteria[i] = 0.0;
	}
	*convergence_criteria = 0.0;
	*signoriniVerified = true;

	int currentAtomicIndex=0;
	int nbConstraints = static_cast<int>(constraintsType.size());

	// For each constraint, we look if the constraint is violated or not !
	for (int i=0 ; i<nbConstraints ; i++)
	{
		switch (constraintsType[i])
		{

			//####################################
			//####################################
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			double& F  = (*initialGuess_and_solution)[currentAtomicIndex];
			double violation = b[currentAtomicIndex]*subStep;
			for (int j=0 ; j<n ; j++)
			{
				violation += A(currentAtomicIndex, j) * (*initialGuess_and_solution)[j];
			}
			F -= violation / A(currentAtomicIndex, currentAtomicIndex);
		}
		currentAtomicIndex++;
		break;


		//####################################
		//####################################
		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			double& F1  = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& F2  = (*initialGuess_and_solution)[currentAtomicIndex+1];
			double violation[2] = { b[currentAtomicIndex]* subStep , b[currentAtomicIndex+1]* subStep };
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
				violation[1] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
			}
			// det = ad-bc
			// [ a b ]   [  d -b ]       [ 1 0 ]
			// [ c d ] . [ -c  a ]/det = [ 0 1 ]
			double A_determinant =
				A(currentAtomicIndex, currentAtomicIndex)*A(currentAtomicIndex+1, currentAtomicIndex+1) -
				A(currentAtomicIndex, currentAtomicIndex+1)*A(currentAtomicIndex+1, currentAtomicIndex);
			double Ainv[2][2] =
			{
				{
					A(currentAtomicIndex+1, currentAtomicIndex+1)/A_determinant,
					-A(currentAtomicIndex,   currentAtomicIndex+1)/A_determinant
				},
				{
					-A(currentAtomicIndex+1, currentAtomicIndex)/A_determinant,
					A(currentAtomicIndex,   currentAtomicIndex)/A_determinant
				}
			};
			F1 -= (Ainv[0][0]*violation[0] + Ainv[0][1]*violation[1]);
			F2 -= (Ainv[1][0]*violation[0] + Ainv[1][1]*violation[1]);
		}
		currentAtomicIndex+=2;
		break;


		//####################################
		//####################################
		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			double& F1  = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& F2  = (*initialGuess_and_solution)[currentAtomicIndex+1];
			double& F3  = (*initialGuess_and_solution)[currentAtomicIndex+2];
			double violation[3] = { b[currentAtomicIndex]* subStep , b[currentAtomicIndex+1]* subStep ,
									b[currentAtomicIndex+2]* subStep
								  };
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
				violation[1] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
				violation[2] += A(currentAtomicIndex+2, j) * (*initialGuess_and_solution)[j];
			}
			Eigen::Matrix3d Ainv = A.block<3,3>(currentAtomicIndex, currentAtomicIndex).inverse();
			F1 -= (Ainv(0, 0)*violation[0] + Ainv(0, 1)*violation[1] + Ainv(0, 2)*violation[2]);
			F2 -= (Ainv(1, 0)*violation[0] + Ainv(1, 1)*violation[1] + Ainv(1, 2)*violation[2]);
			F3 -= (Ainv(2, 0)*violation[0] + Ainv(2, 1)*violation[1] + Ainv(2, 2)*violation[2]);
		}
		currentAtomicIndex+=3;
		break;


		//####################################
		//####################################
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			double& Fn  = (*initialGuess_and_solution)[currentAtomicIndex];

			// Form the local system
			computeEnforcementSystem(n,A,nbColumnInA,b,(*initialGuess_and_solution),frictionCoefs,constraintsType,
									 subStep, i, currentAtomicIndex);

			// Solve A.f = violation
			if (! solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
							  &m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (int i=0 ; i<m_numEnforcedAtomicConstraints-1 ; i++)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints-1];

			if (Fn<0.0)
			{
				Fn  = 0;      // inactive contact on normal
			}
		}
		currentAtomicIndex++;
		break;


		//####################################
		//####################################
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			double local_mu = frictionCoefs[i];
			double& Fn  = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& Ft1 = (*initialGuess_and_solution)[currentAtomicIndex+1];
			double& Ft2 = (*initialGuess_and_solution)[currentAtomicIndex+2];

			// Form the local system
			computeEnforcementSystem(n,A,nbColumnInA,b,(*initialGuess_and_solution),frictionCoefs,constraintsType,
									 subStep, i, currentAtomicIndex);

			// Solve A.f = violation
			if (! solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
							  &m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (int i=0 ; i<m_numEnforcedAtomicConstraints-1 ; i++)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints-1];

			if (Fn>0.0)
			{
				// Compute the frictions violation
				double violation[2]= { b[currentAtomicIndex+1]* subStep , b[currentAtomicIndex+2]* subStep };
				for (int i=0 ; i<n ; i++)
				{
					violation[0] += A(currentAtomicIndex+1, i)*(*initialGuess_and_solution)[i];
					violation[1] += A(currentAtomicIndex+2, i)*(*initialGuess_and_solution)[i];
				}

				Ft1 -= 2*violation[0]/(A(currentAtomicIndex+1, currentAtomicIndex+1) +
									   A(currentAtomicIndex+2, currentAtomicIndex+2));
				Ft2 -= 2*violation[1]/(A(currentAtomicIndex+1, currentAtomicIndex+1) +
									   A(currentAtomicIndex+2, currentAtomicIndex+2));

				double normFt = sqrt(Ft1 * Ft1 + Ft2 * Ft2);
				if (normFt>local_mu*Fn)
				{
					// Here, the Friction is too strong, we keep the direction, but modulate its lenght
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
		currentAtomicIndex+=3;
		break;


		//####################################
		//####################################
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			double& Fn1 = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& Fn2 = (*initialGuess_and_solution)[currentAtomicIndex+1];

			// Form the local system
			computeEnforcementSystem(n,A,nbColumnInA,b,(*initialGuess_and_solution),frictionCoefs,constraintsType,
									 subStep, i, currentAtomicIndex);

			// Solve A.f = violation
			if (! solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
							  &m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (int i=0 ; i<m_numEnforcedAtomicConstraints-2 ; i++)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn1 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints-2];
			Fn2 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints-1];

// 			// 1st we analyze the constraints in the system to see if we should enforce any of them while solving
// 			// the contact !
// 			if(constraintsType[0]==MLCP_BILATERAL_3D_CONSTRAINT) // Bilateral 3D ?
// 			{
// 				if(constraintsType[1]==MLCP_BILATERAL_2D_CONSTRAINT) // Bilateral 3D+Directional constraints ?
// 				{
// 					if(constraintsType[2]==MLCP_BILATERAL_1D_CONSTRAINT) // Bilateral 3D+Directional+Axial constraints?
// 					{
// 						// HERE, we have:
// 						// 1 bilateral   3D constraint store first in the system
// 						// 1 directional 2D constraint store second in the system
// 						// 1 axial       1D constraint store third in the system
// 						// We want to enforce these constraints while solving the contact
// 						// => solve 7x7 system, including the bilateral 3D constraint + directional 2D constraint +
// 						//    + axial 1D rotation + contact normal force (1D)
// 						// => if the resulting contact normal force is positive, we will compute some frictional forces
// 						double &FX  = (*initialGuess_and_solution)[0];
// 						double &FY  = (*initialGuess_and_solution)[1];
// 						double &FZ  = (*initialGuess_and_solution)[2];
//
// 						double &FdirX  = (*initialGuess_and_solution)[3];
// 						double &FdirY  = (*initialGuess_and_solution)[4];
//
// 						double &Faxial  = (*initialGuess_and_solution)[5];
//
// 						double violation[8] = { b[0]*subStep , b[1]*subStep , b[2]*subStep ,
// 							b[3]*subStep, b[4]*subStep, b[5]*subStep,
// 							b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep };
// 						for( int j=0 ; j<n ; j++ )
// 						{
// 							violation[0] += A(0, j) * (*initialGuess_and_solution)[j];
// 							violation[1] += A(1, j) * (*initialGuess_and_solution)[j];
// 							violation[2] += A(2, j) * (*initialGuess_and_solution)[j];
// 							violation[3] += A(3, j) * (*initialGuess_and_solution)[j];
// 							violation[4] += A(4, j) * (*initialGuess_and_solution)[j];
// 							violation[5] += A(5, j) * (*initialGuess_and_solution)[j];
// 							violation[6] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 							violation[7] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 						}
// 						double localA[64]={
// 							A(0, 0), A(0, 1), A(0, 2), A(0, 3), A(0, 4), A(0, 5),
// 								A(0, currentAtomicIndex), A(0, currentAtomicIndex+1),
// 							A(1, 0), A(1, 1), A(1, 2), A(1, 3), A(1, 4), A(1, 5),
// 								A(1, currentAtomicIndex), A(1, currentAtomicIndex+1),
// 							A(2, 0), A(2, 1), A(2, 2), A(2, 3), A(2, 4), A(2, 5),
// 								A(2, currentAtomicIndex), A(2, currentAtomicIndex+1),
// 							A(3, 0), A(3, 1), A(3, 2), A(3, 3), A(3, 4), A(3, 5),
// 								A(3, currentAtomicIndex), A(3, currentAtomicIndex+1),
// 							A(4, 0), A(4, 1), A(4, 2), A(4, 3), A(4, 4), A(4, 5),
// 								A(4, currentAtomicIndex), A(4, currentAtomicIndex+1),
// 							A(5, 0), A(5, 1), A(5, 2), A(5, 3), A(5, 4), A(5, 5),
// 								A(5, currentAtomicIndex), A(5, currentAtomicIndex+1),
// 							A(currentAtomicIndex,   0), A(currentAtomicIndex,   1), A(currentAtomicIndex,   2),
// 								A(currentAtomicIndex,   3), A(currentAtomicIndex,   4), A(currentAtomicIndex,   5),
// 									A(currentAtomicIndex,   currentAtomicIndex),
// 										A(currentAtomicIndex,   currentAtomicIndex+1),
// 							A(currentAtomicIndex+1, 0), A(currentAtomicIndex+1, 1), A(currentAtomicIndex+1, 2),
// 								A(currentAtomicIndex+1, 3), A(currentAtomicIndex+1, 4), A(currentAtomicIndex+1, 5),
// 								A(currentAtomicIndex+1, currentAtomicIndex),
// 									A(currentAtomicIndex+1, currentAtomicIndex+1)
// 						};
// 						double Ainv[64];
// 						// Try a block decomposition 7=5+3
// 						if( !inverseMatrix_using_BlockDecomposition<double,8,5,3>(localA,Ainv) )
// 						{
// 							cerr << " MLCP could not inverse a local 6x6 matrix for contact + " <<
// 								"(3D bilateral + 2D directional)" << endl;
// 							cerr << " Press any key + [ENTER] to unlock the simulation" << endl;
// 							char wait;
// 							cin >> wait;
// 							exit(0);
// 						}
// 						double F[8];
// 						MatrixVectorProduct_nxm<double,8,8>((const double *)Ainv , (const double *)violation,
// 							(double *)F);
// 						FX    -= F[0];
// 						FY    -= F[1];
// 						FZ    -= F[2];
// 						FdirX -= F[3];
// 						FdirY -= F[4];
// 						Faxial-= F[5];
// 						Fn1   -= F[6];
// 						Fn2   -= F[7];
// 					}
// 					else  // if(constraintsType[2]==MLCP_BILATERAL_1D_CONSTRAINT)
// 					{
// 						// HERE, we have:
// 						// 1 bilateral   3D constraint store first in the system
// 						// 1 directional 2D constraint store second in the system
// 						// We want to enforce these constraints while solving the contact
// 						// => solve 6x6 system, including the bilateral 3D constraint + directional 2D constraint +
// 						//    + contact normal force (1D)
// 						// => if the resulting contact normal force is positive, we will compute some frictional forces
// 						double &FX  = (*initialGuess_and_solution)[0];
// 						double &FY  = (*initialGuess_and_solution)[1];
// 						double &FZ  = (*initialGuess_and_solution)[2];
//
// 						double &FdirX  = (*initialGuess_and_solution)[3];
// 						double &FdirY  = (*initialGuess_and_solution)[4];
//
// 						double violation[7] = { b[0]*subStep , b[1]*subStep , b[2]*subStep , b[3]*subStep, b[4]*subStep,
// 							b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep };
// 						for( int j=0 ; j<n ; j++ )
// 						{
// 							violation[0] += A(0, j) * (*initialGuess_and_solution)[j];
// 							violation[1] += A(1, j) * (*initialGuess_and_solution)[j];
// 							violation[2] += A(2, j) * (*initialGuess_and_solution)[j];
// 							violation[3] += A(3, j) * (*initialGuess_and_solution)[j];
// 							violation[4] += A(4, j) * (*initialGuess_and_solution)[j];
// 							violation[5] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 							violation[6] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 						}
// 						double localA[49]={
// 							A(0, 0), A(0, 1), A(0, 2), A(0, 3), A(0, 4),
// 								A(0, currentAtomicIndex), A(0, currentAtomicIndex+1),
// 							A(1, 0), A(1, 1), A(1, 2), A(1, 3), A(1, 4),
// 								A(1, currentAtomicIndex), A(1, currentAtomicIndex+1),
// 							A(2, 0), A(2, 1), A(2, 2), A(2, 3), A(2, 4),
// 								A(2, currentAtomicIndex), A(2, currentAtomicIndex+1),
// 							A(3, 0), A(3, 1), A(3, 2), A(3, 3), A(3, 4),
// 								A(3, currentAtomicIndex), A(3, currentAtomicIndex+1),
// 							A(4, 0), A(4, 1), A(4, 2), A(4, 3), A(4, 4),
// 								A(4, currentAtomicIndex), A(4, currentAtomicIndex+1),
// 							A(currentAtomicIndex,   0), A(currentAtomicIndex,   1), A(currentAtomicIndex,   2),
// 								A(currentAtomicIndex,   3), A(currentAtomicIndex,   4),
// 									A(currentAtomicIndex,   currentAtomicIndex),
// 										A(currentAtomicIndex,   currentAtomicIndex+1),
// 							A(currentAtomicIndex+1, 0), A(currentAtomicIndex+1, 1), A(currentAtomicIndex+1, 2),
// 								A(currentAtomicIndex+1, 3), A(currentAtomicIndex+1, 4),
// 									A(currentAtomicIndex+1, currentAtomicIndex),
// 										A(currentAtomicIndex+1, currentAtomicIndex+1)
// 						};
// 						double Ainv[49];
// 						if( !inverseMatrix_using_BlockDecomposition<double,7,4,3>(localA,Ainv) )
// 						{
// 							cerr << " MLCP could not inverse a local 6x6 matrix for contact + " <<
// 								"(3D bilateral + 2D directional)" << endl;
// 							cerr << " Press any key + [ENTER] to unlock the simulation" << endl;
// 							char wait;
// 							cin >> wait;
// 							exit(0);
// 						}
// 						double F[7];
// 						MatrixVectorProduct_nxm<double,7,7>((const double *)Ainv , (const double *)violation,
// 							(double *)F);
// 						FX    -= F[0];
// 						FY    -= F[1];
// 						FZ    -= F[2];
// 						FdirX -= F[3];
// 						FdirY -= F[4];
// 						Fn1   -= F[5];
// 						Fn2   -= F[6];
// 					}
// 				}
// 				else  // if(constraintsType[1]==MLCP_BILATERAL_2D_CONSTRAINT)
// 				{
// 					// HERE, we have:
// 					// 1 bilateral 3D constraint store first in the system
// 					// We want to enforce this constraint while solving the contact
// 					// => solve 4x4 system, including the bilateral 3D constraint + contact normal force (1D)
// 					// => if the resulting contact normal force is positive, we will compute some frictional forces
// 					double &FX  = (*initialGuess_and_solution)[0];
// 					double &FY  = (*initialGuess_and_solution)[1];
// 					double &FZ  = (*initialGuess_and_solution)[2];
// 					double violation[5] = { b[0]*subStep , b[1]*subStep , b[2]*subStep ,
// 						b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep };
// 					for( int j=0 ; j<n ; j++ )
// 					{
// 						violation[0] += A(0, j) * (*initialGuess_and_solution)[j];
// 						violation[1] += A(1, j) * (*initialGuess_and_solution)[j];
// 						violation[2] += A(2, j) * (*initialGuess_and_solution)[j];
// 						violation[3] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 						violation[4] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 					}
// 					double localA[25]={
// 						A(0, 0), A(0, 1), A(0, 2), A(0, currentAtomicIndex), A(0, currentAtomicIndex+1),
// 						A(1, 0), A(1, 1), A(1, 2), A(1, currentAtomicIndex), A(1, currentAtomicIndex+1),
// 						A(2, 0), A(2, 1), A(2, 2), A(2, currentAtomicIndex), A(2, currentAtomicIndex+1),
// 						A(currentAtomicIndex,   0), A(currentAtomicIndex,   1), A(currentAtomicIndex,   2),
// 							A(currentAtomicIndex,   currentAtomicIndex), A(currentAtomicIndex,   currentAtomicIndex+1),
// 						A(currentAtomicIndex+1, 0), A(currentAtomicIndex+1, 1), A(currentAtomicIndex+1, 2),
// 							A(currentAtomicIndex+1, currentAtomicIndex), A(currentAtomicIndex+1, currentAtomicIndex+1)
// 					};
// 					double Ainv[25];
// 					if( !inverseMatrix_using_BlockDecomposition<double,5,2,3>(localA,Ainv) )
// 					{
// 						cerr << " MLCP could not inverse a local 4x4 matrix for contact/bilateral" << endl;
// 						cerr << " Press any key + [ENTER] to unlock the simulation" << endl;
// 						char wait;
// 						cin >> wait;
// 						exit(0);
// 					}
// 					double F[5];
// 					MatrixVectorProduct_nxm<double,5,5>(Ainv,violation,F);
// 					FX -= F[0];
// 					FY -= F[1];
// 					FZ -= F[2];
// 					Fn1-= F[3];
// 					Fn2-= F[4];
// 				}
// 			}
// 			else  // if(constraintsType[0]==MLCP_BILATERAL_3D_CONSTRAINT)
// 			{
// 				// HERE, we do not have any expected constraints
// 				// We treat this contact normally, without taking into account any other constraint than the current
// 				// contact !
// 				double violation[2] = { b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep };
// 				for( int j=0 ; j<n ; j++ )
// 				{
// 					violation[0] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 					violation[1] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 				}
//
// 				// det = ad-bc
// 				// [ a b ]   [  d -b ]       [ 1 0 ]
// 				// [ c d ] . [ -c  a ]/det = [ 0 1 ]
// 				double A_determinant =
// 					A(currentAtomicIndex, currentAtomicIndex)*A(currentAtomicIndex+1, currentAtomicIndex+1)-
// 					A(currentAtomicIndex, currentAtomicIndex+1)*A(currentAtomicIndex+1, currentAtomicIndex);
// 				double Ainv[2][2]={
// 					{ A(currentAtomicIndex+1, currentAtomicIndex+1)/A_determinant  ,
// 					 -A(currentAtomicIndex,   currentAtomicIndex+1)/A_determinant },
// 					{-A(currentAtomicIndex+1, currentAtomicIndex  )/A_determinant  ,
// 					  A(currentAtomicIndex,   currentAtomicIndex  )/A_determinant }
// 				} ;
// 				Fn1 -= (Ainv[0][0]*violation[0] + Ainv[0][1]*violation[1]);
// 				Fn2 -= (Ainv[1][0]*violation[0] + Ainv[1][1]*violation[1]);
// 			}
		}
		currentAtomicIndex+=2;
		break;


		//####################################
		//####################################
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			double local_mu = frictionCoefs[i];
			double& Fn1 = (*initialGuess_and_solution)[currentAtomicIndex  ];
			double& Fn2 = (*initialGuess_and_solution)[currentAtomicIndex+1];
			double& Ft  = (*initialGuess_and_solution)[currentAtomicIndex+2];

			// Form the local system
			computeEnforcementSystem(n,A,nbColumnInA,b,(*initialGuess_and_solution),frictionCoefs,constraintsType,
									 subStep, i, currentAtomicIndex);

			// Solve A.f = violation
			if (! solveSystem(m_lhsEnforcedLocalSystem, m_rhsEnforcedLocalSystem, m_numEnforcedAtomicConstraints,
							  &m_rhsEnforcedLocalSystem))
			{
				return;
			}

			// Correct the forces accordingly
			for (int i=0 ; i<m_numEnforcedAtomicConstraints-2 ; i++)
			{
				(*initialGuess_and_solution)[i] -= m_rhsEnforcedLocalSystem[i];
			}
			Fn1 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints-2];
			Fn2 -= m_rhsEnforcedLocalSystem[m_numEnforcedAtomicConstraints-1];

			// No Signorini to verify here, it is NOT a unilateral constraint, but bilateral
			//if(Fn>0.0)
			{
				// Complete the violation of the friction along t, with the missing terms...
				double violation = b[currentAtomicIndex+2]*subStep;
				for (int i=0 ; i<n ; i++)
				{
					violation += A(currentAtomicIndex+2, i)*(*initialGuess_and_solution)[i];
				}

				Ft -= violation/A(currentAtomicIndex+2, currentAtomicIndex+2);

				double normFn = sqrt(Fn1*Fn1 + Fn2*Fn2);
				double normFt = fabs(Ft);
				if (normFt>local_mu*normFn)
				{
					// Here, the Friction is too strong, we keep the direction, but modulate its lenght
					// to verify the Coulomb's law: |Ft| = mu |Fn|
					Ft *= local_mu*normFn/normFt;
				}
			}

// 			// 1st we analyze the constraints in the system to see if we should enforce any of them while solving
// 			// the contact !
// 			if(constraintsType[0]==MLCP_BILATERAL_3D_CONSTRAINT) // Bilateral 3D ?
// 			{
// 				if(constraintsType[1]==MLCP_BILATERAL_2D_CONSTRAINT) // Bilateral 3D+Directional constraints ?
// 				{
// 					if(constraintsType[2]==MLCP_BILATERAL_1D_CONSTRAINT) // Bilateral 3D+Directional+Axial constraints?
// 					{
// 						// HERE, we have:
// 						// 1 bilateral   3D constraint store first in the system
// 						// 1 directional 2D constraint store second in the system
// 						// 1 axial       1D constraint store third in the system
// 						// We want to enforce these constraints while solving the contact
// 						// => solve 7x7 system, including the bilateral 3D constraint + directional 2D constraint +
// 						//    + axial 1D rotation + contact normal force (1D)
// 						// => if the resulting contact normal force is positive, we will compute some frictional forces
// 						double &FX  = (*initialGuess_and_solution)[0];
// 						double &FY  = (*initialGuess_and_solution)[1];
// 						double &FZ  = (*initialGuess_and_solution)[2];
//
// 						double &FdirX  = (*initialGuess_and_solution)[3];
// 						double &FdirY  = (*initialGuess_and_solution)[4];
//
// 						double &Faxial  = (*initialGuess_and_solution)[5];
//
// 						double violation[9] = { b[0]*subStep , b[1]*subStep , b[2]*subStep ,
// 							b[3]*subStep, b[4]*subStep, b[5]*subStep,
// 							b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep ,
// 							b[currentAtomicIndex+2]*subStep };
// 						for( int j=0 ; j<n ; j++ )
// 						{
// 							violation[0] += A(0, j) * (*initialGuess_and_solution)[j];
// 							violation[1] += A(1, j) * (*initialGuess_and_solution)[j];
// 							violation[2] += A(2, j) * (*initialGuess_and_solution)[j];
// 							violation[3] += A(3, j) * (*initialGuess_and_solution)[j];
// 							violation[4] += A(4, j) * (*initialGuess_and_solution)[j];
// 							violation[5] += A(5, j) * (*initialGuess_and_solution)[j];
// 							violation[6] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 							violation[7] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 							if(j>=7 && j!=currentAtomicIndex)
// 							{
// 								violation[8] += A(currentAtomicIndex+2, j) * (*initialGuess_and_solution)[j];
// 							}
// 						}
// 						double localA[64]={
// 							A(0, 0), A(0, 1), A(0, 2), A(0, 3), A(0, 4), A(0, 5),
// 								A(0, currentAtomicIndex), A(0, currentAtomicIndex+1),
// 							A(1, 0), A(1, 1), A(1, 2), A(1, 3), A(1, 4), A(1, 5),
// 								A(1, currentAtomicIndex), A(1, currentAtomicIndex+1),
// 							A(2, 0), A(2, 1), A(2, 2), A(2, 3), A(2, 4), A(2, 5),
// 								A(2, currentAtomicIndex), A(2, currentAtomicIndex+1),
// 							A(3, 0), A(3, 1), A(3, 2), A(3, 3), A(3, 4), A(3, 5),
// 								A(3, currentAtomicIndex), A(3, currentAtomicIndex+1),
// 							A(4, 0), A(4, 1), A(4, 2), A(4, 3), A(4, 4), A(4, 5),
// 								A(4, currentAtomicIndex), A(4, currentAtomicIndex+1),
// 							A(5, 0), A(5, 1), A(5, 2), A(5, 3), A(5, 4), A(5, 5),
// 								A(5, currentAtomicIndex), A(5, currentAtomicIndex+1),
// 							A(currentAtomicIndex,   0), A(currentAtomicIndex,   1), A(currentAtomicIndex,   2),
// 								A(currentAtomicIndex,   3), A(currentAtomicIndex,   4), A(currentAtomicIndex,   5),
// 									A(currentAtomicIndex,   currentAtomicIndex),
// 										A(currentAtomicIndex,   currentAtomicIndex+1),
// 							A(currentAtomicIndex+1, 0), A(currentAtomicIndex+1, 1), A(currentAtomicIndex+1, 2),
// 								A(currentAtomicIndex+1, 3), A(currentAtomicIndex+1, 4), A(currentAtomicIndex+1, 5),
// 									A(currentAtomicIndex+1, currentAtomicIndex),
// 										A(currentAtomicIndex+1, currentAtomicIndex+1)
// 						};
// 						double Ainv[64];
// 						// Try a block decomposition 7=5+3
// 						if( !inverseMatrix_using_BlockDecomposition<double,8,5,3>(localA,Ainv) )
// 						{
// 							cerr << " MLCP could not inverse a local 6x6 matrix for contact +" <<
// 								" (3D bilateral + 2D directional)" << endl;
// 							cerr << " Press any key + [ENTER] to unlock the simulation" << endl;
// 							char wait;
// 							cin >> wait;
// 							exit(0);
// 						}
// 						double F[8];
// 						MatrixVectorProduct_nxm<double,8,8>((const double *)Ainv , (const double *)violation,
// 							(double *)F);
// 						FX    -= F[0];
// 						FY    -= F[1];
// 						FZ    -= F[2];
// 						FdirX -= F[3];
// 						FdirY -= F[4];
// 						Faxial-= F[5];
// 						Fn1   -= F[6];
// 						Fn2   -= F[7];
//
// 						// No Signorini to verify here, it is NOT a unilateral constraint, but bilateral
// 						//if(Fn>0.0)
// 						{
// 							// Complete the violation of the friction along t, with the missing terms...
// 							violation[8] += A(currentAtomicIndex+2,      0)*FX +
// 								A(currentAtomicIndex+2,                    1)*FY +
// 								A(currentAtomicIndex+2,                    2)*FZ +
// 								A(currentAtomicIndex+2,                    3)*FdirX +
// 								A(currentAtomicIndex+2,                    4)*FdirY +
// 								A(currentAtomicIndex+2,                    5)*Faxial+
// 								A(currentAtomicIndex+2, currentAtomicIndex  )*Fn1 +
// 								A(currentAtomicIndex+2, currentAtomicIndex+1)*Fn2;
//
// 							Ft -= violation[8]/A(currentAtomicIndex+2, currentAtomicIndex+2);
//
// 							double normFn = sqrt(Fn1*Fn1 + Fn2*Fn2);
// 							double normFt = fabs(Ft);
// 							if(normFt>local_mu*normFn)
// 							{
// 								// Here, the Friction is too strong, we keep the direction, but modulate its lenght
// 								// to verify the Coulomb's law: |Ft| = mu |Fn|
// 								Ft *= local_mu*normFn/normFt;
// 							}
// 						}
// 					}
// 					else  // if(constraintsType[2]==MLCP_BILATERAL_1D_CONSTRAINT)
// 					{
// 						// HERE, we have:
// 						// 1 bilateral   3D constraint store first in the system
// 						// 1 directional 2D constraint store second in the system
// 						// We want to enforce these constraints while solving the contact
// 						// => solve 6x6 system, including the bilateral 3D constraint + directional 2D constraint +
// 						//    + contact normal force (1D)
// 						// => if the resulting contact normal force is positive, we will compute some frictional forces
// 						double &FX  = (*initialGuess_and_solution)[0];
// 						double &FY  = (*initialGuess_and_solution)[1];
// 						double &FZ  = (*initialGuess_and_solution)[2];
//
// 						double &FdirX  = (*initialGuess_and_solution)[3];
// 						double &FdirY  = (*initialGuess_and_solution)[4];
//
// 						double violation[8] = { b[0]*subStep , b[1]*subStep , b[2]*subStep , b[3]*subStep, b[4]*subStep,
// 							b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep ,
// 							b[currentAtomicIndex+2]*subStep };
// 						for( int j=0 ; j<n ; j++ )
// 						{
// 							violation[0] += A(0, j) * (*initialGuess_and_solution)[j];
// 							violation[1] += A(1, j) * (*initialGuess_and_solution)[j];
// 							violation[2] += A(2, j) * (*initialGuess_and_solution)[j];
// 							violation[3] += A(3, j) * (*initialGuess_and_solution)[j];
// 							violation[4] += A(4, j) * (*initialGuess_and_solution)[j];
// 							violation[5] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 							violation[6] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 							if(j>=6 && j!=currentAtomicIndex)
// 							{
// 								violation[7] += A(currentAtomicIndex+2, j) * (*initialGuess_and_solution)[j];
// 							}
// 						}
// 						double localA[49]={
// 							A(0, 0), A(0, 1), A(0, 2), A(0, 3), A(0, 4), A(0, currentAtomicIndex),
// 								A(0, currentAtomicIndex+1),
// 							A(1, 0), A(1, 1), A(1, 2), A(1, 3), A(1, 4), A(1, currentAtomicIndex),
// 								A(1, currentAtomicIndex+1),
// 							A(2, 0), A(2, 1), A(2, 2), A(2, 3), A(2, 4), A(2, currentAtomicIndex),
// 								A(2, currentAtomicIndex+1),
// 							A(3, 0), A(3, 1), A(3, 2), A(3, 3), A(3, 4), A(3, currentAtomicIndex),
// 								A(3, currentAtomicIndex+1),
// 							A(4, 0), A(4, 1), A(4, 2), A(4, 3), A(4, 4), A(4, currentAtomicIndex),
// 								A(4, currentAtomicIndex+1),
// 							A(currentAtomicIndex,   0), A(currentAtomicIndex,   1), A(currentAtomicIndex,   2),
// 								A(currentAtomicIndex,   3), A(currentAtomicIndex,   4),
// 									A(currentAtomicIndex,   currentAtomicIndex),
// 										A(currentAtomicIndex,   currentAtomicIndex+1),
// 							A(currentAtomicIndex+1, 0), A(currentAtomicIndex+1, 1), A(currentAtomicIndex+1, 2),
// 								A(currentAtomicIndex+1, 3), A(currentAtomicIndex+1, 4),
// 									A(currentAtomicIndex+1, currentAtomicIndex),
// 										A(currentAtomicIndex+1, currentAtomicIndex+1)
// 						};
// 						double Ainv[49];
// 						if( !inverseMatrix_using_BlockDecomposition<double,7,4,3>(localA,Ainv) )
// 						{
// 							cerr << " MLCP could not inverse a local 6x6 matrix for contact +" <<
// 								" (3D bilateral + 2D directional)" << endl;
// 							cerr << " Press any key + [ENTER] to unlock the simulation" << endl;
// 							char wait;
// 							cin >> wait;
// 							exit(0);
// 						}
// 						double F[7];
// 						MatrixVectorProduct_nxm<double,7,7>((const double *)Ainv , (const double *)violation,
// 							(double *)F);
// 						FX    -= F[0];
// 						FY    -= F[1];
// 						FZ    -= F[2];
// 						FdirX -= F[3];
// 						FdirY -= F[4];
// 						Fn1   -= F[5];
// 						Fn2   -= F[6];
//
// 						// No Signorini to verify here, we have a bilateral constraint, not a unilateral one
// 						//if(Fn>0.0)
// 						{
// 							// Complete the violation of the friction along t, with the missing terms...
// 							violation[7] += A(currentAtomicIndex+2,      0)*FX +
// 								A(currentAtomicIndex+2,                    1)*FY +
// 								A(currentAtomicIndex+2,                    2)*FZ +
// 								A(currentAtomicIndex+2,                    3)*FdirX +
// 								A(currentAtomicIndex+2,                    4)*FdirY +
// 								A(currentAtomicIndex+2, currentAtomicIndex  )*Fn1+
// 								A(currentAtomicIndex+2, currentAtomicIndex+1)*Fn2;
//
// 							Ft -= violation[7]/A(currentAtomicIndex+2, currentAtomicIndex+2);
//
// 							double normFn = sqrt(Fn1*Fn1 + Fn2*Fn2);
// 							double normFt = fabs(Ft);
// 							if(normFt>local_mu*normFn)
// 							{
// 								// Here, the Friction is too strong, we keep the direction, but modulate its lenght
// 								// to verify the Coulomb's law: |Ft| = mu |Fn|
// 								Ft *= local_mu*normFn/normFt;
// 							}
// 						}
// 					}
// 				}
// 				else  // if(constraintsType[1]==MLCP_BILATERAL_2D_CONSTRAINT)
// 				{
// 					// HERE, we have:
// 					// 1 bilateral 3D constraint store first in the system
// 					// We want to enforce this constraint while solving the contact
// 					// => solve 4x4 system, including the bilateral 3D constraint + contact normal force (1D)
// 					// => if the resulting contact normal force is positive, we will compute some frictional forces
// 					double &FX  = (*initialGuess_and_solution)[0];
// 					double &FY  = (*initialGuess_and_solution)[1];
// 					double &FZ  = (*initialGuess_and_solution)[2];
// 					double violation[6] = { b[0]*subStep , b[1]*subStep , b[2]*subStep ,
// 						b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep ,
// 						b[currentAtomicIndex+2]*subStep };
// 					for( int j=0 ; j<n ; j++ )
// 					{
// 						violation[0] += A(0, j) * (*initialGuess_and_solution)[j];
// 						violation[1] += A(1, j) * (*initialGuess_and_solution)[j];
// 						violation[2] += A(2, j) * (*initialGuess_and_solution)[j];
// 						violation[3] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 						violation[4] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 						if(j>=4 && j!=currentAtomicIndex)
// 						{
// 							violation[5] += A(currentAtomicIndex+2, j) * (*initialGuess_and_solution)[j];
// 						}
// 					}
// 					double localA[25]={
// 						A(0, 0), A(0, 1), A(0, 2), A(0, currentAtomicIndex), A(0, currentAtomicIndex+1),
// 						A(1, 0), A(1, 1), A(1, 2), A(1, currentAtomicIndex), A(1, currentAtomicIndex+1),
// 						A(2, 0), A(2, 1), A(2, 2), A(2, currentAtomicIndex), A(2, currentAtomicIndex+1),
// 						A(currentAtomicIndex,   0), A(currentAtomicIndex,   1), A(currentAtomicIndex,   2),
// 							A(currentAtomicIndex,   currentAtomicIndex), A(currentAtomicIndex,   currentAtomicIndex+1),
// 						A(currentAtomicIndex+1, 0), A(currentAtomicIndex+1, 1), A(currentAtomicIndex+1, 2),
// 							A(currentAtomicIndex+1, currentAtomicIndex), A(currentAtomicIndex+1, currentAtomicIndex+1)
// 					};
// 					double Ainv[25];
// 					if( !inverseMatrix_using_BlockDecomposition<double,5,2,3>(localA,Ainv) )
// 					{
// 						cerr << " MLCP could not inverse a local 4x4 matrix for contact/bilateral" << endl;
// 						cerr << " Press any key + [ENTER] to unlock the simulation" << endl;
// 						char wait;
// 						cin >> wait;
// 						exit(0);
// 					}
// 					double F[5];
// 					MatrixVectorProduct_nxm<double,5,5>(Ainv,violation,F);
// 					FX -= F[0];
// 					FY -= F[1];
// 					FZ -= F[2];
// 					Fn1-= F[3];
// 					Fn2-= F[4];
//
// 					// No Signorini to verify here, we have a bilateral constraint, not a unilateral one !
// 					//if(Fn>0.0)
// 					{
// 						// Complete the violation of the friction along t, with the missing terms...
// 						violation[5] += A(currentAtomicIndex+2,      0)*FX +
// 							A(currentAtomicIndex+2,                    1)*FY +
// 							A(currentAtomicIndex+2,                    2)*FZ +
// 							A(currentAtomicIndex+2, currentAtomicIndex  )*Fn1+
// 							A(currentAtomicIndex+2, currentAtomicIndex+1)*Fn2;
//
// 						Ft -= violation[5]/A(currentAtomicIndex+2, currentAtomicIndex+2);
//
// 						double normFn = sqrt(Fn1*Fn1 + Fn2*Fn2);
// 						double normFt = fabs(Ft);
// 						if(normFt>local_mu*normFn)
// 						{
// 							// Here, the Friction is too strong, we keep the direction, but modulate its lenght
// 							// to verify the Coulomb's law: |Ft| = mu |Fn|
// 							Ft *= local_mu*normFn/normFt;
// 						}
// 					}
// 				}
// 			}
// 			else  // if(constraintsType[0]==MLCP_BILATERAL_3D_CONSTRAINT)
// 			{
// 				// HERE, we do not have any expected constraints
// 				// We treat this contact normally, without taking into account any other constraint than the current
// 				// contact !
// 				double violation[3] = { b[currentAtomicIndex]*subStep , b[currentAtomicIndex+1]*subStep ,
// 					b[currentAtomicIndex+2]*subStep };
// 				for( int j=0 ; j<currentAtomicIndex ; j++ )
// 				{
// 					violation[0] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 					violation[1] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 					violation[2] += A(currentAtomicIndex+2, j) * (*initialGuess_and_solution)[j];
// 				}
// 				for( int j=currentAtomicIndex+3 ; j<n ; j++ )
// 				{
// 					violation[0] += A(currentAtomicIndex,   j) * (*initialGuess_and_solution)[j];
// 					violation[1] += A(currentAtomicIndex+1, j) * (*initialGuess_and_solution)[j];
// 					violation[2] += A(currentAtomicIndex+2, j) * (*initialGuess_and_solution)[j];
// 				}
// 				violation[0] += A(currentAtomicIndex, currentAtomicIndex)*Fn1 +
// 					A(currentAtomicIndex, currentAtomicIndex+1)*Fn2 +
// 					A(currentAtomicIndex, currentAtomicIndex+2)*Ft;
// 				violation[1] += A(currentAtomicIndex+1, currentAtomicIndex)*Fn1 +
// 					A(currentAtomicIndex+1, currentAtomicIndex+1)*Fn2 +
// 					A(currentAtomicIndex+1, currentAtomicIndex+2)*Ft;
//
// 				// det = ad-bc
// 				// [ a b ]   [  d -b ]       [ 1 0 ]
// 				// [ c d ] . [ -c  a ]/det = [ 0 1 ]
// 				double A_determinant =
// 					A(currentAtomicIndex, currentAtomicIndex)*A(currentAtomicIndex+1, currentAtomicIndex+1)-
// 					A(currentAtomicIndex, currentAtomicIndex+1)*A(currentAtomicIndex+1, currentAtomicIndex);
// 				double Ainv[2][2]={
// 					{ A(currentAtomicIndex+1, currentAtomicIndex+1)/A_determinant  ,
// 					 -A(currentAtomicIndex,   currentAtomicIndex+1)/A_determinant },
// 					{-A(currentAtomicIndex+1, currentAtomicIndex  )/A_determinant  ,
// 					  A(currentAtomicIndex,   currentAtomicIndex  )/A_determinant }
// 				} ;
// 				Fn1 -= (Ainv[0][0]*violation[0] + Ainv[0][1]*violation[1]);
// 				Fn2 -= (Ainv[1][0]*violation[0] + Ainv[1][1]*violation[1]);
//
// 				// No Signorini to verify here, we have bilaterals constraints, not unilateral ones !
// 				//if(Fn>0.0)
// 				{
// 					violation[2] += A(currentAtomicIndex+2, currentAtomicIndex)*Fn1
// 						+ A(currentAtomicIndex+2, currentAtomicIndex+1)*Fn2
// 						+ A(currentAtomicIndex+2, currentAtomicIndex+2)*Ft;
//
// 					Ft -= violation[2]/A(currentAtomicIndex+2, currentAtomicIndex+2);
//
// 					double normFt = fabs(Ft);
// 					double normFn = sqrt(Fn1*Fn1 + Fn2*Fn2);
// 					if(normFt>local_mu*normFn)
// 					{
// 						// Here, the Friction is too strong, we keep the direction, but modulate its lenght
// 						// to verify the Coulomb's law: |Ft| = mu |Fn|
// 						Ft *= local_mu*normFn/normFt;
// 					}
// 				}
// 			}
		}
		currentAtomicIndex+=3;
		break;

		//####################################
		//####################################
		default:
			//XXX
			SURGSIM_FAILURE() << "unknown constraint type [" << constraintsType[i] << "]";
			break;
		}
	}
}

void MlcpGaussSeidelSolver::printViolationsAndConvergence(int n, const MlcpProblem::Matrix& A, int nbColumnInA,
														  const MlcpProblem::Vector& b,
														  const MlcpSolution::Vector& initialGuess_and_solution,
														  const std::vector<MlcpConstraintType>& constraintsType,
														  double subStep,
														  double convergence_criteria, bool signorini_verified,
														  int nbLoop)
{
	printf("MLCP at iteration %d =\n",nbLoop);

	int currentAtomicIndex=0;
	int nbConstraints = static_cast<int>(constraintsType.size());

	for (int i=0 ; i<nbConstraints ; i++)
	{
		printf("Constraint [%2d] of type ",i);
		switch (constraintsType[i])
		{
		case MLCP_BILATERAL_1D_CONSTRAINT:
		{
			printf("BILATERAL_1D_CONSTRAINT");
			printf("\n\t with initial violation b=(%g) ",b[currentAtomicIndex]);
			double violation = b[currentAtomicIndex];
			for (int j=0 ; j<n ; j++)
			{
				violation += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
			}
			printf("\n\t with final   violation b-Ax=(%g) ",violation);
			printf("\n\t force=(%g) ",initialGuess_and_solution[currentAtomicIndex]);
			currentAtomicIndex+=1;
		}
		break;
		case MLCP_BILATERAL_2D_CONSTRAINT:
		{
			printf("BILATERAL_2D_CONSTRAINT");
			printf("\n\t with initial violation b=(%g %g) ",b[currentAtomicIndex],b[currentAtomicIndex+1]);
			double violation[2] = {b[currentAtomicIndex],b[currentAtomicIndex+1]};
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
			}
			printf("\n\t with final   violation b-Ax=(%g %g) ",violation[0],violation[1]);
			printf("\n\t force=(%g %g) ",
				   initialGuess_and_solution[currentAtomicIndex],
				   initialGuess_and_solution[currentAtomicIndex+1]);
			currentAtomicIndex+=2;
		}
		break;
		case MLCP_BILATERAL_3D_CONSTRAINT:
		{
			printf("BILATERAL_3D_CONSTRAINT");
			printf("\n\t with initial violation b=(%g %g %g) ",
				   b[currentAtomicIndex], b[currentAtomicIndex+1], b[currentAtomicIndex+2]);
			double violation[3] = {b[currentAtomicIndex],b[currentAtomicIndex+1],b[currentAtomicIndex+2]};
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
				violation[2] += A(currentAtomicIndex+2, j) * initialGuess_and_solution[j];
			}
			printf("\n\t with final   violation b-Ax=(%g %g %g) ",violation[0],violation[1],violation[2]);
			printf("\n\t force=(%g %g %g) ",
				   initialGuess_and_solution[currentAtomicIndex],
				   initialGuess_and_solution[currentAtomicIndex+1],
				   initialGuess_and_solution[currentAtomicIndex+2]);
			currentAtomicIndex+=3;
		}
		break;
		case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		{
			printf("UNILATERAL_FRICTIONLESS_CONSTRAINT");
			printf("\n\t with initial violation b=(%g) ",b[currentAtomicIndex]);
			double violation = b[currentAtomicIndex];
			for (int j=0 ; j<n ; j++)
			{
				violation += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
			}
			printf("\n\t with final   violation b-Ax=(%g) ",violation);
			if (violation < -m_contactTolerance)
			{
				printf("\n\t  => normal violation = %g < -m_contactTolerance => Signorini not verified yet !",
					   violation);
			}
			printf("\n\t force=(%g) ",initialGuess_and_solution[currentAtomicIndex]);
			currentAtomicIndex+=1;
		}
		break;
		case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		{
			printf("UNILATERAL_3D_FRICTIONAL_CONSTRAINT");
			printf("\n\t with initial violation b=(%g %g %g) ",
				   b[currentAtomicIndex],
				   b[currentAtomicIndex+1],
				   b[currentAtomicIndex+2]);
			double violation[3] = {b[currentAtomicIndex],b[currentAtomicIndex+1],b[currentAtomicIndex+2]};
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
				violation[2] += A(currentAtomicIndex+2, j) * initialGuess_and_solution[j];
			}
			printf("\n\t with final   violation b-Ax=(%g %g %g) ",violation[0],violation[1],violation[2]);
			if (violation[0] < -m_contactTolerance)
			{
				printf("\n\t  => normal violation = %g < -contactTolerance => Signorini not verified yet !",
					   violation[0]);
			}
			printf("\n\t force=(%g %g %g) ",
				   initialGuess_and_solution[currentAtomicIndex],
				   initialGuess_and_solution[currentAtomicIndex+1],
				   initialGuess_and_solution[currentAtomicIndex+2]);
			currentAtomicIndex+=3;
		}
		break;
		case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		{
			printf("UNILATERAL_3D_FRICTIONLESS_SUTURING");
			printf("\n\t with initial violation b=(%g %g) ",b[currentAtomicIndex],b[currentAtomicIndex+1]);
			double violation[2] = {b[currentAtomicIndex],b[currentAtomicIndex+1]};
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
			}
			printf("\n\t with final   violation b-Ax=(%g %g) ",violation[0],violation[1]);
			printf("\n\t force=(%g %g) ",
				   initialGuess_and_solution[currentAtomicIndex],
				   initialGuess_and_solution[currentAtomicIndex+1]);
			currentAtomicIndex+=2;
		}
		break;
		case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		{
			printf("UNILATERAL_3D_FRICTIONAL_SUTURING");
			printf("\n\t with initial violation b=(%g %g %g) ",
				   b[currentAtomicIndex],
				   b[currentAtomicIndex+1],
				   b[currentAtomicIndex+2]);
			double violation[3] = {b[currentAtomicIndex],b[currentAtomicIndex+1],b[currentAtomicIndex+2]};
			for (int j=0 ; j<n ; j++)
			{
				violation[0] += A(currentAtomicIndex,   j) * initialGuess_and_solution[j];
				violation[1] += A(currentAtomicIndex+1, j) * initialGuess_and_solution[j];
				violation[2] += A(currentAtomicIndex+2, j) * initialGuess_and_solution[j];
			}
			printf("\n\t with final   violation b-Ax=(%g %g %g) ",violation[0],violation[1],violation[2]);
			printf("\n\t force=(%g %g %g) ",
				   initialGuess_and_solution[currentAtomicIndex],
				   initialGuess_and_solution[currentAtomicIndex+1],
				   initialGuess_and_solution[currentAtomicIndex+2]);
			currentAtomicIndex+=3;
		}
		break;
		default:
			break;
		}
		printf("\n");
	}
	printf("convergence_criteria=%g  Signorini verified=%s\n",
		   convergence_criteria, (signorini_verified ? "yes" : "NO"));
}

};  // namespace Math
};  // namespace SurgSim

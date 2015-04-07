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

#include "SurgSim/Blocks/MassSpring1DRepresentation.h"
#include "SurgSim/Blocks/MassSpringNDRepresentationUtils.h"
#include "SurgSim/Math/LinearSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/LinearSpring.h"

using SurgSim::Math::Vector3d;
using SurgSim::Physics::Mass;

namespace SurgSim
{

namespace Blocks
{

void MassSpring1DRepresentation::init1D(
	const std::vector<Vector3d> nodes,
	std::vector<size_t> nodeBoundaryConditions,
	double totalMass,
	double stiffnessStretching, double dampingStretching,
	double stiffnessBending, double dampingBending)
{
	std::shared_ptr<SurgSim::Math::OdeState> state;
	state = std::make_shared<SurgSim::Math::OdeState>();
	state->setNumDof(getNumDofPerNode(), nodes.size());

	SURGSIM_ASSERT(nodes.size() > 0) << "Number of nodes incorrect: " << nodes.size();

	// Initialize the nodes position, velocity and mass
	// Note: no need to apply the initialPose here, initialize will take care of it !
	for (size_t massId = 0; massId < nodes.size(); massId++)
	{
		addMass(std::make_shared<Mass>(totalMass / static_cast<double>(nodes.size())));

		SurgSim::Math::setSubVector(nodes[massId], massId, 3, &state->getPositions());
	}

	// Initialize the stretching springs
	if (stiffnessStretching || dampingStretching)
	{
		for (size_t massId = 0; massId < nodes.size() - 1; massId++)
		{
			addSpring(createLinearSpring(state, massId, massId + 1, stiffnessStretching, dampingStretching));
		}
	}

	// Initialize the bending springs
	if (stiffnessBending || dampingBending)
	{
		for (size_t massId = 0; massId < nodes.size() - 2; massId++)
		{
			addSpring(createLinearSpring(state, massId, massId + 2, stiffnessBending, dampingBending));
		}
	}

	// Sets the boundary conditions
	for (auto boundaryCondition = std::begin(nodeBoundaryConditions);
		 boundaryCondition != std::end(nodeBoundaryConditions);
		 boundaryCondition++)
	{
		state->addBoundaryCondition(*boundaryCondition);
	}

	// setInitialState: Initialize all the states + apply initialPose if any
	setInitialState(state);
}

bool MassSpring1DRepresentation::doWakeUp()
{
	using SurgSim::Math::LinearSparseSolveAndInverseLU;

	if (!MassSpringRepresentation::doWakeUp())
	{
		return false;
	}

	// For implicit ode solver, we solve (M/dt + D + dt.K).deltaV = F - dt.K.v(t)
	// with M diagonal, K tri-diagonal block of a given size and D a linear combination of M and K.
	// So (M/dt + D + dt.K) is a tri-diagonal block matrix of the following size:
	//
	// If we only have stretching springs, each node is connected with its direct neighbors
	// The matrix K has the following structure (each X being a 3x3 matrix):
	//  nodeId       3(i-3)  3(i-2)  3(i-1)    3(i)  3(i+1)  3(i+2)  3(i+3)
	//  3(i-3) = 0........X       X       0       0       0       0       0......0
	//  3(i-2) = 0........X       X       X       0       0       0       0......0
	//  3(i-1) = 0........0       X       X       X       0       0       0......0
	//  3(i)   = 0........0       0       X       X       X       0       0......0
	//  3(i+1) = 0........0       0       0       X       X       X       0......0
	//  3(i+2) = 0........0       0       0       0       X       X       X......0
	//  3(i+3) = 0........0       0       0       0       0       X       X......0
	// => blockSize = 3
	//
	// If we also have bending springs, each nodes is also connected with its second direct neighbors
	// The matrix K has the following structure (each X being a 3x3 matrix):
	//  nodeId       3(i-3)  3(i-2)  3(i-1)    3(i)  3(i+1)  3(i+2)  3(i+3)
	//  3(i-3) = 0........X       X       X       0       0       0       0......0
	//  3(i-2) = 0........X       X       X       X       0       0       0......0
	//  3(i-1) = 0........X       X       X       X       X       0       0......0
	//  3(i)   = 0........0       X       X       X       X       X       0......0
	//  3(i+1) = 0........0       0       X       X       X       X       X......0
	//  3(i+2) = 0........0       0       0       X       X       X       X......0
	//  3(i+3) = 0........0       0       0       0       X       X       X......0
	// => we define blockSize as 6 (if we have an even number of nodes: 3*2n / 6 = n blocks)

	switch (m_integrationScheme)
	{
		case SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER:
		case SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER:
			if (getInitialState()->getNumNodes() % 2 == 0)
			{
				m_odeSolver->setLinearSolver(std::make_shared<LinearSparseSolveAndInverseLU>());
			}
			else
			{
				// We should use a band matrix solver here in general when available
			}
			break;
		default:
			break;
	}

	return true;
}

}; // namespace Blocks

}; // namespace SurgSim

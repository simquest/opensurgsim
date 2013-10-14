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

#include <SurgSim/Blocks/MassSpring2DRepresentation.h>
#include <SurgSim//Physics/LinearSpring.h>

using SurgSim::Physics::LinearSpring;
using SurgSim::Physics::Mass;

namespace SurgSim
{

namespace Blocks
{

void MassSpring2DRepresentation::initSpring(const std::shared_ptr<DeformableRepresentationState> state,
	unsigned int nodeId0, unsigned int nodeId1,
	double stiffness, double damping)
{
	std::shared_ptr<LinearSpring> spring = std::make_shared<LinearSpring>(nodeId0, nodeId1);

	const Vector3d& A = SurgSim::Math::getSubVector(state->getPositions(), nodeId0, 3);
	const Vector3d& B = SurgSim::Math::getSubVector(state->getPositions(), nodeId1, 3);
	spring->setStiffness(stiffness);
	spring->setDamping(damping);
	spring->setInitialLength((B-A).norm());

	addSpring(spring);
}

void MassSpring2DRepresentation::init2DStretchingSprings(const std::shared_ptr<DeformableRepresentationState> state,
	unsigned int numNodesPerDim[2], double stiffness, double damping)
{
	const int rowOffset = numNodesPerDim[0];
	const int colOffset = 1;

	// Initialize the stretching springs
	if (stiffness || damping)
	{
		// ...along X
		for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
		{
			for (unsigned int col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				unsigned int nodeId = row * rowOffset + col * colOffset;
				initSpring(state, nodeId, nodeId + colOffset, stiffness, damping);
			}
		}
		// ...along Y
		for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
		{
			for (unsigned int row = 0; row < numNodesPerDim[1] - 1; row++)
			{
				unsigned int nodeId = row * rowOffset + col * colOffset;
				initSpring(state, nodeId, nodeId + rowOffset, stiffness, damping);
			}
		}
	}
}

void MassSpring2DRepresentation::init2DBendingSprings(const std::shared_ptr<DeformableRepresentationState> state,
	unsigned int numNodesPerDim[2], double stiffness, double damping)
{
	const int rowOffset = numNodesPerDim[0];
	const int colOffset = 1;

	// Initialize the bending springs
	if (stiffness || damping)
	{
		// ... along X
		for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
		{
			for (unsigned int col = 0; col < numNodesPerDim[0] - 2; col++)
			{
				unsigned int nodeId = row * rowOffset + col * colOffset;
				initSpring(state, nodeId, nodeId + 2 * colOffset, stiffness, damping);
			}
		}
		// ... along Y
		for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
		{
			for (unsigned int row = 0; row < numNodesPerDim[1] - 2; row++)
			{
				unsigned int nodeId = row * rowOffset + col * colOffset;
				initSpring(state, nodeId, nodeId + 2 * rowOffset, stiffness, damping);
			}
		}
	}
}

void MassSpring2DRepresentation::init2DFaceDiagonalSprings(const std::shared_ptr<DeformableRepresentationState> state,
	unsigned int numNodesPerDim[2], double stiffness, double damping)
{
	const int rowOffset = numNodesPerDim[0];
	const int colOffset = 1;

	// Initialize the face diagonal springs
	if (stiffness || damping)
	{
		for (unsigned int row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (unsigned int col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				unsigned int nodeId = row * rowOffset + col * colOffset;
				initSpring(state, nodeId, nodeId + rowOffset + colOffset, stiffness, damping);
				initSpring(state, nodeId + colOffset, nodeId + rowOffset, stiffness, damping);
			}
		}
	}
}

void MassSpring2DRepresentation::init2D(
	const Vector3d extremities[2][2], unsigned int numNodesPerDim[2],
	std::vector<unsigned int> boundaryConditions,
	double totalMass,
	double stiffnessStretching, double dampingStretching,
	double stiffnessBending, double dampingBending,
	double stiffnessFaceDiagonal, double dampingFaceDiagonal)
{
	std::shared_ptr<DeformableRepresentationState> state;
	state = std::make_shared<DeformableRepresentationState>();
	state->setNumDof(numNodesPerDim[0] * numNodesPerDim[1] * 3);

	SURGSIM_ASSERT(numNodesPerDim[0] > 0) << "Number of nodes for dimension 1 is incorrect: " << numNodesPerDim[0];
	SURGSIM_ASSERT(numNodesPerDim[1] > 0) << "Number of nodes for dimension 2 is incorrect: " << numNodesPerDim[1];

	// Initialize the nodes position, velocity and mass
	// Note: no need to apply the initialPose here, initialize will take care of it !
	Vector3d rowExtremititiesDelta[2] =
	{
		(extremities[0][1] - extremities[0][0]) / static_cast<double>(numNodesPerDim[1] - 1) ,
		(extremities[1][1] - extremities[1][0]) / static_cast<double>(numNodesPerDim[1] - 1)
	};
	unsigned int nodeId = 0;
	for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
	{
		Vector3d rowExtremities[2];
		rowExtremities[0] = extremities[0][0] + rowExtremititiesDelta[0] * static_cast<double>(row);
		rowExtremities[1] = extremities[1][0] + rowExtremititiesDelta[1] * static_cast<double>(row);

		Vector3d delta = (rowExtremities[1] - rowExtremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
		for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
		{
			addMass(std::make_shared<Mass>(totalMass / static_cast<double>(numNodesPerDim[0] * numNodesPerDim[1])));

			SurgSim::Math::Vector3d position(rowExtremities[0] + col * delta);
			SurgSim::Math::setSubVector(position, nodeId, 3, &state->getPositions());

			nodeId++;
		}
	}

	// Initialize all the stretching springs
	init2DStretchingSprings(state, numNodesPerDim, stiffnessStretching, dampingStretching);

	// Initialize all the bending springs
	init2DBendingSprings(state, numNodesPerDim, stiffnessBending, dampingBending);

	// Initialize all the face diagonal springs
	init2DFaceDiagonalSprings(state, numNodesPerDim, stiffnessFaceDiagonal, dampingFaceDiagonal);

	// Sets the boundary conditions
	for (auto boundaryCondition = std::begin(boundaryConditions);
		boundaryCondition != std::end(boundaryConditions);
		boundaryCondition++)
	{
		state->addBoundaryCondition(*boundaryCondition);
	}

	// setInitialState: Initialize all the states + apply initialPose if any
	setInitialState(state);
}

}; // namespace Blocks

}; // namespace SurgSim

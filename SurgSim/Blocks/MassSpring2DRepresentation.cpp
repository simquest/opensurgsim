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

#include "SurgSim/Blocks/MassSpring2DRepresentation.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Mass.h"
#include "SurgSim/Physics/MassSpring.h"

using SurgSim::Math::Vector3d;
using SurgSim::Physics::LinearSpring;
using SurgSim::Physics::Mass;

namespace SurgSim
{

namespace Blocks
{

void MassSpring2DRepresentation::init2DStretchingSprings(const std::shared_ptr<Physics::MassSpring> mesh,
	size_t numNodesPerDim[2], double stiffness, double damping)
{
	const size_t rowOffset = numNodesPerDim[0];
	const size_t colOffset = 1;

	// Initialize the stretching springs
	if (stiffness || damping)
	{
		// ...along X
		for (size_t row = 0; row < numNodesPerDim[1]; row++)
		{
			for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				size_t nodeId = row * rowOffset + col * colOffset;
				addSpring(std::make_shared<LinearSpring>(mesh, nodeId, nodeId + colOffset, stiffness, damping));
			}
		}
		// ...along Y
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
			{
				size_t nodeId = row * rowOffset + col * colOffset;
				addSpring(std::make_shared<LinearSpring>(mesh, nodeId, nodeId + rowOffset, stiffness, damping));
			}
		}
	}
}

void MassSpring2DRepresentation::init2DBendingSprings(const std::shared_ptr<Physics::MassSpring> mesh,
	size_t numNodesPerDim[2], double stiffness, double damping)
{
	const size_t rowOffset = numNodesPerDim[0];
	const size_t colOffset = 1;

	// Initialize the bending springs
	if (stiffness || damping)
	{
		// ... along X
		for (size_t row = 0; row < numNodesPerDim[1]; row++)
		{
			for (size_t col = 0; col < numNodesPerDim[0] - 2; col++)
			{
				size_t nodeId = row * rowOffset + col * colOffset;
				addSpring(std::make_shared<LinearSpring>(mesh, nodeId, nodeId + 2 * colOffset, stiffness, damping));
			}
		}
		// ... along Y
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			for (size_t row = 0; row < numNodesPerDim[1] - 2; row++)
			{
				size_t nodeId = row * rowOffset + col * colOffset;
				addSpring(std::make_shared<LinearSpring>(mesh, nodeId, nodeId + 2 * rowOffset, stiffness, damping));
			}
		}
	}
}

void MassSpring2DRepresentation::init2DFaceDiagonalSprings(const std::shared_ptr<Physics::MassSpring> mesh,
	size_t numNodesPerDim[2], double stiffness, double damping)
{
	const size_t rowOffset = numNodesPerDim[0];
	const size_t colOffset = 1;

	// Initialize the face diagonal springs
	if (stiffness || damping)
	{
		for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				size_t nodeId = row * rowOffset + col * colOffset;
				addSpring(std::make_shared<LinearSpring>(mesh, nodeId, nodeId + rowOffset + colOffset, stiffness,
					damping));
				addSpring(std::make_shared<LinearSpring>(mesh, nodeId + colOffset, nodeId + rowOffset, stiffness,
					damping));
			}
		}
	}
}

void MassSpring2DRepresentation::init2D(
	const std::array<std::array<Vector3d, 2>, 2> extremities,
	size_t numNodesPerDim[2],
	std::vector<size_t> nodeBoundaryConditions,
	double totalMass,
	double stiffnessStretching, double dampingStretching,
	double stiffnessBending, double dampingBending,
	double stiffnessFaceDiagonal, double dampingFaceDiagonal)
{
	SURGSIM_ASSERT(numNodesPerDim[0] > 0) << "Number of nodes for dimension 1 is incorrect: " << numNodesPerDim[0];
	SURGSIM_ASSERT(numNodesPerDim[1] > 0) << "Number of nodes for dimension 2 is incorrect: " << numNodesPerDim[1];

	auto mesh = std::make_shared<Physics::MassSpring>();

	// Initialize the nodes position, velocity and mass
	// Note: no need to apply the initialPose here, initialize will take care of it !
	Vector3d rowExtremititiesDelta[2] =
	{
		(extremities[0][1] - extremities[0][0]) / static_cast<double>(numNodesPerDim[1] - 1) ,
		(extremities[1][1] - extremities[1][0]) / static_cast<double>(numNodesPerDim[1] - 1)
	};
	for (size_t row = 0; row < numNodesPerDim[1]; row++)
	{
		Vector3d rowExtremities[2];
		rowExtremities[0] = extremities[0][0] + rowExtremititiesDelta[0] * static_cast<double>(row);
		rowExtremities[1] = extremities[1][0] + rowExtremititiesDelta[1] * static_cast<double>(row);

		Vector3d delta = (rowExtremities[1] - rowExtremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			auto mass = std::make_shared<Mass>(totalMass / static_cast<double>(numNodesPerDim[0] * numNodesPerDim[1]));
			mesh->addMass(mass);
			SurgSim::Math::Vector3d position(rowExtremities[0] + static_cast<double>(col) * delta);
			mesh->addVertex(DataStructures::Vertices<Mass>::VertexType(position, *mass));
		}
	}
	
	// Initialize the 2D elements, triangles.
	for (size_t row = 0; row < numNodesPerDim[1] - 1; ++row)
	{
		for (size_t col = 0; col < numNodesPerDim[0] - 1; ++col)
		{
			size_t firstCorner = col + row * numNodesPerDim[0];
			mesh->addElement({ firstCorner, firstCorner + numNodesPerDim[0], firstCorner + 1 });
			mesh->addElement({ firstCorner + 1, firstCorner + numNodesPerDim[0], firstCorner + numNodesPerDim[0] + 1 });
		}
	}

	// Initialize all the stretching springs
	init2DStretchingSprings(mesh, numNodesPerDim, stiffnessStretching, dampingStretching);

	// Initialize all the bending springs
	init2DBendingSprings(mesh, numNodesPerDim, stiffnessBending, dampingBending);

	// Initialize all the face diagonal springs
	init2DFaceDiagonalSprings(mesh, numNodesPerDim, stiffnessFaceDiagonal, dampingFaceDiagonal);

	// Sets the boundary conditions
	for (const auto& boundaryCondition : nodeBoundaryConditions)
	{
		mesh->addBoundaryCondition(boundaryCondition);
	}

	setMassSpring(std::dynamic_pointer_cast<Framework::Asset>(mesh));
}

}; // namespace Blocks

}; // namespace SurgSim

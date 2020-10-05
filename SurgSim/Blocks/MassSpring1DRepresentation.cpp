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
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Mass.h"
#include "SurgSim/Physics/MassSpring.h"

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
	SURGSIM_ASSERT(nodes.size() > 0) << "Number of nodes incorrect: " << nodes.size();

	auto mesh = std::make_shared<Physics::MassSpring>();
	// Initialize the nodes position, velocity and mass
	// Note: no need to apply the initialPose here, initialize will take care of it !
	for (size_t massId = 0; massId < nodes.size(); massId++)
	{
		auto mass = std::make_shared<Mass>(totalMass / static_cast<double>(nodes.size()));
		mesh->addMass(mass);
		mesh->addVertex(DataStructures::Vertices<Mass>::VertexType(nodes[massId], *mass));
	}

	// Initialize the 1D elements, segments.
	for (size_t massId = 0; massId < nodes.size() - 1; ++massId)
	{
		mesh->addElement({ massId, massId + 1 });
	}

	// Initialize the stretching springs
	if (stiffnessStretching || dampingStretching)
	{
		for (size_t massId = 0; massId < nodes.size() - 1; massId++)
		{
			mesh->addSpring(std::make_shared<Physics::LinearSpring>(mesh, massId, massId + 1, stiffnessStretching,
				dampingStretching));
		}
	}

	// Initialize the bending springs
	if (stiffnessBending || dampingBending)
	{
		for (size_t massId = 0; massId < nodes.size() - 2; massId++)
		{
			mesh->addSpring(std::make_shared<Physics::LinearSpring>(mesh, massId, massId + 2, stiffnessBending,
				dampingBending));
		}
	}

	// Sets the boundary conditions
	for (const auto& boundaryCondition : nodeBoundaryConditions)
	{
		mesh->addBoundaryCondition(boundaryCondition);
	}

	setMassSpring(std::dynamic_pointer_cast<Framework::Asset>(mesh));
}

}; // namespace Blocks

}; // namespace SurgSim

// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/FemConstraintFrictionalSliding.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/SlidingConstraintData.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemLocalization.h"
#include "SurgSim/Physics/FemRepresentation.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

FemConstraintFrictionalSliding::FemConstraintFrictionalSliding()
{
}

FemConstraintFrictionalSliding::~FemConstraintFrictionalSliding()
{
}

void FemConstraintFrictionalSliding::doBuild(double dt,
	const ConstraintData& data,
	const std::shared_ptr<Localization>& localization,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation,
	size_t indexOfConstraint,
	ConstraintSideSign sign)
{
	auto fem = std::static_pointer_cast<FemRepresentation>(localization->getRepresentation());
	if (!fem->isActive())
	{
		return;
	}

	const size_t numDofPerNode = fem->getNumDofPerNode();
	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const SlidingConstraintData& constraintData = static_cast<const SlidingConstraintData&>(data);

	std::array<Math::Vector3d, 3> directions;
	directions[0] = constraintData.getNormals()[0];
	directions[1] = constraintData.getNormals()[1];
	directions[2] = constraintData.getTangent();

	DataStructures::IndexedLocalCoordinate coord
		= std::static_pointer_cast<FemLocalization>(localization)->getLocalPosition();
	Vector3d globalPosition = localization->calculatePosition();

	m_newH.resize(fem->getNumDof());
	auto femElement = fem->getFemElement(coord.index);
	auto numNodes = fem->getFemElement(coord.index)->getNumNodes();
	m_newH.reserve(numNodes * 3);

	for (size_t i = 0; i < 2; ++i)
	{
		// Update b with new violation
		double violation = directions[i].dot(globalPosition);
		mlcp->b[indexOfConstraint + i] += violation * scale;

		// Fill the new H.
		m_newH.setZero();
		for (size_t j = 0; j < numNodes; ++j)
		{
			auto nodeId = femElement->getNodeId(j);
			m_newH.insert(numDofPerNode * nodeId + 0) = coord.coordinate[j] * directions[i][0] * scale * dt;
			m_newH.insert(numDofPerNode * nodeId + 1) = coord.coordinate[j] * directions[i][1] * scale * dt;
			m_newH.insert(numDofPerNode * nodeId + 2) = coord.coordinate[j] * directions[i][2] * scale * dt;
		}

		mlcp->updateConstraint(m_newH, fem->getComplianceMatrix() * m_newH.transpose(), indexOfRepresentation,
			indexOfConstraint + i);
	}

	// The friction is like a bilateral 3d constraint.
	if (numNodes == 2)
	{
		auto previousLocalization = constraintData.getPreviousFirstLocalization();
		coord = std::static_pointer_cast<FemLocalization>(previousLocalization)->getLocalPosition();
		globalPosition = previousLocalization->calculatePosition();
	}
	mlcp->b.segment<3>(indexOfConstraint + 2) += globalPosition * scale;
	m_newH.resize(fem->getNumDof());
	femElement = fem->getFemElement(coord.index);
	numNodes = femElement->getNumNodes();
	size_t numNodeToConstrain = (coord.coordinate.array() != 0.0).count();
	m_newH.reserve(3 * numNodeToConstrain);

	for (size_t axis = 0; axis < 3; axis++)
	{
		m_newH.setZero();
		for (size_t index = 0; index < numNodes; index++)
		{
			if (coord.coordinate[index] != 0.0)
			{
				size_t nodeIndex = femElement->getNodeId(index);
				m_newH.insert(numDofPerNode * nodeIndex + axis) = coord.coordinate[index] * (dt * scale);
			}
		}
		mlcp->updateConstraint(m_newH, fem->getComplianceMatrix() * m_newH.transpose(),
			indexOfRepresentation, indexOfConstraint + axis + 2);
	}

	mlcp->mu[indexOfConstraint] = constraintData.getFrictionCoefficient();
}

SurgSim::Physics::ConstraintType FemConstraintFrictionalSliding::getConstraintType() const
{
	return SurgSim::Physics::FRICTIONAL_SLIDING;
}

size_t FemConstraintFrictionalSliding::doGetNumDof() const
{
	return 5;
}

}; //  namespace Physics

}; //  namespace SurgSim

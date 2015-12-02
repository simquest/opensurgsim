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

#include "SurgSim/Physics/FemConstraintFrictionlessSliding.h"

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

FemConstraintFrictionlessSliding::FemConstraintFrictionlessSliding()
{
}

FemConstraintFrictionlessSliding::~FemConstraintFrictionlessSliding()
{
}

void FemConstraintFrictionlessSliding::doBuild(double dt,
	const ConstraintData& data,
	const std::shared_ptr<Localization>& localization,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation,
	size_t indexOfConstraint,
	ConstraintSideSign sign)
{
	auto fem = std::static_pointer_cast<FemRepresentation>(localization->getRepresentation());
	const size_t numDofPerNode = fem->getNumDofPerNode();

	if (!fem->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const SlidingConstraintData& constraintData = static_cast<const SlidingConstraintData&>(data);
	const Vector3d normal[2] = {constraintData.getNormal1(), constraintData.getNormal2()};
	double d[2] = {constraintData.getD1(), constraintData.getD2()};

	const DataStructures::IndexedLocalCoordinate& coord
		= std::static_pointer_cast<FemLocalization>(localization)->getLocalPosition();
	Vector3d globalPosition = localization->calculatePosition();

	m_newH.resize(fem->getNumDof());
	auto femElement = fem->getFemElement(coord.index);
	auto numNodes = fem->getFemElement(coord.index)->getNumNodes();
	m_newH.reserve(numNodes * 3);

	for (size_t i = 0; i < 2; ++i)
	{
		// Update b with new violation
		double violation = normal[i].dot(globalPosition) + d[i];
		mlcp->b[indexOfConstraint + i] += violation * scale;

		// Fill the new H.
		m_newH.setZero();
		for (size_t j = 0; j < numNodes; ++j)
		{
			auto nodeId = femElement->getNodeId(j);
			m_newH.insert(numDofPerNode * nodeId + 0) = coord.coordinate[j] * normal[i][0] * scale * dt;
			m_newH.insert(numDofPerNode * nodeId + 1) = coord.coordinate[j] * normal[i][1] * scale * dt;
			m_newH.insert(numDofPerNode * nodeId + 2) = coord.coordinate[j] * normal[i][2] * scale * dt;
		}

		mlcp->updateConstraint(m_newH, fem->getComplianceMatrix() * m_newH.transpose(), indexOfRepresentation,
			indexOfConstraint + i);
	}
}

SurgSim::Physics::ConstraintType FemConstraintFrictionlessSliding::getConstraintType() const
{
	return SurgSim::Physics::FRICTIONLESS_SLIDING;
}

size_t FemConstraintFrictionlessSliding::doGetNumDof() const
{
	return 2;
}

}; //  namespace Physics

}; //  namespace SurgSim

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

#include "SurgSim/Physics/MassSpringConstraintFrictionalSliding.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/SlidingConstraintData.h"
#include "SurgSim/Physics/MassSpringLocalization.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

MassSpringConstraintFrictionalSliding::MassSpringConstraintFrictionalSliding()
{
}

MassSpringConstraintFrictionalSliding::~MassSpringConstraintFrictionalSliding()
{
}

void MassSpringConstraintFrictionalSliding::doBuild(double dt,
	const ConstraintData& data,
	const std::shared_ptr<Localization>& localization,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation,
	size_t indexOfConstraint,
	ConstraintSideSign sign)
{
	auto massSpring = std::static_pointer_cast<MassSpringRepresentation>(localization->getRepresentation());
	if (!massSpring->isActive())
	{
		return;
	}

	const size_t numDofPerNode = massSpring->getNumDofPerNode();
	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const SlidingConstraintData& constraintData = static_cast<const SlidingConstraintData&>(data);

	std::array<Math::Vector3d, 3> directions;
	directions[0] = constraintData.getNormals()[0];
	directions[1] = constraintData.getNormals()[1];
	directions[2] = constraintData.getTangent();

	m_newH.resize(massSpring->getNumDof());

	auto typedLocalization = std::static_pointer_cast<MassSpringLocalization>(localization);
	Vector3d globalPosition = localization->calculatePosition();
	std::vector<size_t> nodeIds;
	Math::Vector coordinates;

	for (int i = 0; i < 3; ++i)
	{
		if ((i == 2) && (massSpring->getNodeIds(0).size() == 2))
		{
			typedLocalization =
				std::static_pointer_cast<MassSpringLocalization>(constraintData.getPreviousFirstLocalization());
			globalPosition = typedLocalization->calculatePosition();
		}
		if (typedLocalization->getLocalNode().hasValue())
		{
			nodeIds.push_back(typedLocalization->getLocalNode().getValue());
			coordinates.resize(1);
			coordinates << 1.0;
		}
		else if (typedLocalization->getLocalPosition().hasValue())
		{
			const auto& coord = typedLocalization->getLocalPosition().getValue();
			nodeIds = massSpring->getNodeIds(coord.index);
			coordinates = coord.coordinate;
		}
		else
		{
			SURGSIM_FAILURE() <<
				"MassSpringConstraintFrictionalSliding requires a localization with either a node or a local position.";
		}
		m_newH.reserve(3 * nodeIds.size());

		// Update b with new violation
		double violation = directions[i].dot(globalPosition);
		mlcp->b[indexOfConstraint + i] += violation * scale;

		// Fill the new H.
		m_newH.setZero();
		for (size_t index = 0; index < nodeIds.size(); ++index)
		{
			m_newH.insert(numDofPerNode * nodeIds[index] + 0) = coordinates[index] * directions[i][0] * scale * dt;
			m_newH.insert(numDofPerNode * nodeIds[index] + 1) = coordinates[index] * directions[i][1] * scale * dt;
			m_newH.insert(numDofPerNode * nodeIds[index] + 2) = coordinates[index] * directions[i][2] * scale * dt;
		}
		mlcp->updateConstraint(m_newH, massSpring->getComplianceMatrix() * m_newH.transpose(), indexOfRepresentation,
			indexOfConstraint + i);
	}
	mlcp->mu[indexOfConstraint] = constraintData.getFrictionCoefficient();
}

SurgSim::Physics::ConstraintType MassSpringConstraintFrictionalSliding::getConstraintType() const
{
	return SurgSim::Physics::FRICTIONAL_SLIDING;
}

size_t MassSpringConstraintFrictionalSliding::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim

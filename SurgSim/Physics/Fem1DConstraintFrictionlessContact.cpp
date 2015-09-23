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

#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/Fem1DConstraintFrictionlessContact.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Localization.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

Fem1DConstraintFrictionlessContact::Fem1DConstraintFrictionlessContact()
{
}

Fem1DConstraintFrictionlessContact::~Fem1DConstraintFrictionlessContact()
{
}

void Fem1DConstraintFrictionlessContact::doBuild(double dt,
												 const ConstraintData& data,
												 const std::shared_ptr<Localization>& localization,
												 MlcpPhysicsProblem* mlcp,
												 size_t indexOfRepresentation,
												 size_t indexOfConstraint,
												 ConstraintSideSign sign)
{
	std::shared_ptr<Fem1DRepresentation> fem
		= std::static_pointer_cast<Fem1DRepresentation>(localization->getRepresentation());

	if (!fem->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const SurgSim::Math::Vector3d& n = static_cast<const ContactConstraintData&>(data).getNormal();
	const SurgSim::DataStructures::IndexedLocalCoordinate& coord
		= std::static_pointer_cast<Fem1DLocalization>(localization)->getLocalPosition();
	Vector3d globalPosition = localization->calculatePosition();

	// Update b with new violation
	double violation = n.dot(globalPosition);
	mlcp->b[indexOfConstraint] += violation * scale;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(fem->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	std::shared_ptr<FemElement> femElement = fem->getFemElement(coord.index);
	size_t numNodes = femElement->getNumNodes();
	size_t numNodeToConstrain = 0;
	for (size_t index = 0; index < numNodes; index++)
	{
		if (coord.coordinate[index] != 0.0)
		{
			numNodeToConstrain++;
		}
	}
	m_newH.reserve(3 * numNodeToConstrain);

	for (size_t index = 0; index < numNodes; index++)
	{
		if (coord.coordinate[index] != 0.0)
		{
			size_t nodeIndex = femElement->getNodeId(index);
			m_newH.insert(6 * nodeIndex + 0) = coord.coordinate[index] * n[0] * scale * dt;
			m_newH.insert(6 * nodeIndex + 1) = coord.coordinate[index] * n[1] * scale * dt;
			m_newH.insert(6 * nodeIndex + 2) = coord.coordinate[index] * n[2] * scale * dt;
		}
	}

	mlcp->updateConstraint(m_newH, fem->getComplianceMatrix() * m_newH.transpose(), indexOfRepresentation,
		indexOfConstraint);
}

SurgSim::Physics::ConstraintType Fem1DConstraintFrictionlessContact::getConstraintType() const
{
	return SurgSim::Physics::FRICTIONLESS_3DCONTACT;
}

size_t Fem1DConstraintFrictionlessContact::doGetNumDof() const
{
	return 1;
}

}; //  namespace Physics

}; //  namespace SurgSim

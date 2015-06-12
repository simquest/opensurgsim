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
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DConstraintFrictionlessContact.h"
#include "SurgSim/Physics/Fem3DLocalization.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{

namespace Physics
{

Fem3DConstraintFrictionlessContact::Fem3DConstraintFrictionlessContact()
{
}

Fem3DConstraintFrictionlessContact::~Fem3DConstraintFrictionlessContact()
{
}

void Fem3DConstraintFrictionlessContact::doBuild(double dt,
		const ConstraintData& data,
		const std::shared_ptr<Localization>& localization,
		MlcpPhysicsProblem* mlcp,
		size_t indexOfRepresentation,
		size_t indexOfConstraint,
		ConstraintSideSign sign)
{
	using SurgSim::Math::Vector3d;

	auto fem3d = std::static_pointer_cast<Fem3DRepresentation>(localization->getRepresentation());

	if (!fem3d->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;

	const ContactConstraintData& contactData = static_cast<const ContactConstraintData&>(data);
	const SurgSim::Math::Vector3d& n = contactData.getNormal();

	const SurgSim::DataStructures::IndexedLocalCoordinate& coord
		= std::static_pointer_cast<Fem3DLocalization>(localization)->getLocalPosition();

	// FRICTIONLESS CONTACT in a LCP
	//   (n, d) defines the plane of contact
	//   p(t) the point of contact (usually after free motion)
	//   p(free) is the point of contact after free motion
	//   u is the displacement needed to verify the constraint
	// note that u = sum ui * baryCoord[i]
	//
	// The constraint equation for a plane is
	// n^t.[p(free)+u] + d >= 0
	// n^t.p(free) + n^t.u + d >= 0
	// n^t.u + (n^t.p(free) + d) >= 0
	//
	// For implicit integration, u = dt.v(t+dt)
	//
	// Since the d term will be added to the constraint for one side of the contact and subtracted from the other,
	// and because it is not clear which distance should be used, we leave it out.

	// Update b with new violation
	Vector3d globalPosition = localization->calculatePosition();
	double violation = n.dot(globalPosition);

	mlcp->b[indexOfConstraint] += violation * scale;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(fem3d->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	std::shared_ptr<FemElement> femElement = fem3d->getFemElement(coord.index);
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
			m_newH.insert(3 * nodeIndex + 0) = coord.coordinate[index] * n[0] * scale * dt;
			m_newH.insert(3 * nodeIndex + 1) = coord.coordinate[index] * n[1] * scale * dt;
			m_newH.insert(3 * nodeIndex + 2) = coord.coordinate[index] * n[2] * scale * dt;
		}
	}

	mlcp->updateConstraint(m_newH, fem3d->getComplianceMatrix() * m_newH.transpose(), indexOfRepresentation,
						   indexOfConstraint);
}

SurgSim::Physics::ConstraintType Fem3DConstraintFrictionlessContact::getConstraintType() const
{
	return SurgSim::Physics::FRICTIONLESS_3DCONTACT;
}

size_t Fem3DConstraintFrictionlessContact::doGetNumDof() const
{
	return 1;
}

};  //  namespace Physics

};  //  namespace SurgSim

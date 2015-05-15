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

#include <memory>

#include "SurgSim/Physics/MassSpringConstraintFrictionlessContact.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"

#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/MassSpringLocalization.h"

namespace SurgSim
{

namespace Physics
{

MassSpringConstraintFrictionlessContact::MassSpringConstraintFrictionlessContact()
{

}

MassSpringConstraintFrictionlessContact::~MassSpringConstraintFrictionlessContact()
{

}

void MassSpringConstraintFrictionlessContact::doBuild(double dt,
			const ConstraintData& data,
			const std::shared_ptr<Localization>& localization,
			MlcpPhysicsProblem* mlcp,
			size_t indexOfRepresentation,
			size_t indexOfConstraint,
			ConstraintSideSign sign)
{
	using SurgSim::Math::Vector3d;

	auto massSpring = std::static_pointer_cast<MassSpringRepresentation>(localization->getRepresentation());

	if (!massSpring->isActive())
	{
		return;
	}

	size_t nodeId = std::static_pointer_cast<MassSpringLocalization>(localization)->getLocalNode();
	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;

	auto& contactData = static_cast<const ContactConstraintData&>(data);
	const Vector3d& n = contactData.getNormal();

	// FRICTIONLESS CONTACT in a LCP
	//   (n, d) defines the plane of contact
	//   p(t) the point of contact (usually after free motion)
	//
	// The constraint equation for a plane is
	// U(t) = n^t.p(t) + d >= 0
	//
	// dU/dt = H.dp/dt
	// => H = n^t
	// Since the d term will be added to the constraint for one side of the contact and subtracted from the other,
	// and because it is not clear which distance should be used, we leave it out.

	// Update b with new violation U
	Vector3d globalPosition = localization->calculatePosition();
	double violation = n.dot(globalPosition);

	mlcp->b[indexOfConstraint] += violation * scale;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(massSpring->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	m_newH.reserve(3);
	m_newH.insert(3 * nodeId + 0) = n[0] * scale;
	m_newH.insert(3 * nodeId + 1) = n[1] * scale;
	m_newH.insert(3 * nodeId + 2) = n[2] * scale;

	mlcp->updateConstraint(m_newH, massSpring->applyCompliance(*(massSpring->getCurrentState()), m_newH),
						   indexOfRepresentation, indexOfConstraint);
}

SurgSim::Math::MlcpConstraintType MassSpringConstraintFrictionlessContact::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
}

size_t MassSpringConstraintFrictionlessContact::doGetNumDof() const
{
	return 1;
}

}; // namespace Physics

}; // namespace SurgSim
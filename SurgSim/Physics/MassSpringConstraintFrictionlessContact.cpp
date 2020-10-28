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

#include "SurgSim/Math/SegmentMeshShape.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/MassSpringLocalization.h"

namespace SurgSim
{

namespace Physics
{

MassSpringConstraintFrictionlessContact::MassSpringConstraintFrictionlessContact() : m_mlcpNumericalPrecision(1.0e-04)
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

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	auto& contactData = static_cast<const ContactConstraintData&>(data);
	const Vector3d& n = contactData.getNormal();
	size_t numNodesToConstrain = 1;
	std::vector<size_t> nodeIds;
	Math::Vector coordinates;
	auto typedLocalization = std::static_pointer_cast<MassSpringLocalization>(localization);
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
		numNodesToConstrain = (coord.coordinate.array() != 0.0).count();
		coordinates = coord.coordinate;
	}
	else
	{
		SURGSIM_FAILURE() <<
			"MassSpringConstraintFrictionlessContact requires a localization with either a node or a local position.";
	}
	double radius = 0.0;
	{
		auto rep = std::dynamic_pointer_cast<Collision::Representation>(massSpring->getCollisionRepresentation());
		if (rep != nullptr)
		{
			auto segmentShape =
				std::dynamic_pointer_cast<Math::SegmentMeshShape>(rep->getShape());
			if (segmentShape != nullptr)
			{
				radius = segmentShape->getRadius();
			}
		}
	}

	// FRICTIONLESS CONTACT in a LCP
	//   (n, d) defines the plane of contact
	//   p(t) the point of contact (usually after free motion)
	//
	// The constraint equation for a plane is
	// U(t) = n^t.p(t) + d >= 0
	//
	// See formulation in FemConstraintFrictionlessContact and RigidConstraintFrictionlessContact.
	// Uses the backward Euler formulation.
	// Since the d term will be added to the constraint for one side of the contact and subtracted from the other,
	// and because it is not clear which distance should be used, we leave it out.

	// Update b with new violation U
	Vector3d globalPosition = localization->calculatePosition();
	double violation = n.dot(globalPosition);

	mlcp->b[indexOfConstraint] += violation * scale - radius - m_mlcpNumericalPrecision;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(massSpring->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	m_newH.reserve(3 * numNodesToConstrain); // why was dt missing on the next lines???
	for (size_t index = 0; index < nodeIds.size(); ++index)
	{
		if (coordinates[index] != 0.0)
		{
			m_newH.insert(3 * nodeIds[index] + 0) = coordinates[index] * n[0] * scale * dt;
			m_newH.insert(3 * nodeIds[index] + 1) = coordinates[index] * n[1] * scale * dt;
			m_newH.insert(3 * nodeIds[index] + 2) = coordinates[index] * n[2] * scale * dt;
		}
	}
	mlcp->updateConstraint(m_newH, massSpring->getComplianceMatrix() * m_newH.transpose(),
		indexOfRepresentation, indexOfConstraint);
}

SurgSim::Physics::ConstraintType MassSpringConstraintFrictionlessContact::getConstraintType() const
{
	return SurgSim::Physics::FRICTIONLESS_3DCONTACT;
}

size_t MassSpringConstraintFrictionlessContact::doGetNumDof() const
{
	return 1;
}

}; // namespace Physics

}; // namespace SurgSim

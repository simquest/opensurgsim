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

#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/MassSpringConstraintFixedPoint.h"
#include "SurgSim/Physics/MassSpringLocalization.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

MassSpringConstraintFixedPoint::MassSpringConstraintFixedPoint()
{
}

MassSpringConstraintFixedPoint::~MassSpringConstraintFixedPoint()
{
}

void MassSpringConstraintFixedPoint::doBuild(double dt,
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
	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
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
			"MassSpringConstraintFixedPoint requires a localization with either a node or a local position.";
	}

	// Fixed point constraint in MCLP
	//   n the number of dof in the Mass-Spring
	//   p     is the constrained point before free motion
	//   pfree is the constrained point after free motion
	//   vfree is the constrained point's velocity after free motion
	//   v is the velocity in general
	//   u  is the position variation needed to enforce the constraint from the free motion
	//   u' is the velocity variation needed to enforce the constraint from the free motion
	//   target is defined as position to be constrained to (in this method, it corresponds to the other part of the
	//                                                       constraint that we don't have access to)
	//
	// The constraint equation is
	//   C: pfree + u - target = 0 (note that the constraint is defined with 3 equations, each along an axis X-Y-Z)
	//
	// Using backward-Euler integration, the constraint can be expressed on the velocity level:
	//   pfree = p + dt.vfree            -> free motion
	//   pfree + u = p + dt.[vfree + u'] -> free motion + constraint correction
	//   => u = dt.u'
	//   C: pfree - target + dt.u' = 0
	//
	// Noting the system matrix S, the constraint matrix H = dC/dv is expressed on the velocity level as the
	// general system to solve is on the velocity level:
	// (S H^T) (   v   ) = (Impulse)
	// (H 0  ) (-lambda)   (initial constraint violation)
	//
	// H = dC/dv
	// The matrix H is of size 3xn (3 equations to fix each axis X-Y-Z relating to the n dof of the model).
	// If the localization contains a node, then:
	// the constraint only involves the velocity of a single node (the constrained node), so the matrix H
	// is full of zero except for a 3x3 part corresponding to the constrained node for which we have:
	// dC/du' = du/du' = dt.Id(3x3)
	// If the localization contains a local coordinate, each of the element's nodes are involved.

	// Update b with new violation: P(free motion)
	Vector3d globalPosition = localization->calculatePosition();
	mlcp->b.segment<3>(indexOfConstraint) += globalPosition * scale;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(massSpring->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	m_newH.reserve(3 * numNodesToConstrain);

	for (size_t axis = 0; axis < 3; ++axis)
	{
		m_newH.setZero();
		for (size_t index = 0; index < nodeIds.size(); ++index)
		{
			if (coordinates[index] != 0.0)
			{
				m_newH.insert(3 * nodeIds[index] + axis) = coordinates[index] * (dt * scale);
			}
		}
		mlcp->updateConstraint(m_newH, massSpring->getComplianceMatrix() * m_newH.transpose(),
			indexOfRepresentation, indexOfConstraint + axis);
	}
}

SurgSim::Physics::ConstraintType MassSpringConstraintFixedPoint::getConstraintType() const
{
	return SurgSim::Physics::FIXED_3DPOINT;
}

size_t MassSpringConstraintFixedPoint::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim

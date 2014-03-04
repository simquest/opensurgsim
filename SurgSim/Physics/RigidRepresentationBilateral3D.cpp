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

#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationBilateral3D.h"
#include "SurgSim/Physics/RigidRepresentationLocalization.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

RigidRepresentationBilateral3D::RigidRepresentationBilateral3D()
{
}

RigidRepresentationBilateral3D::~RigidRepresentationBilateral3D()
{
}

void RigidRepresentationBilateral3D::doBuild(double dt,
											 const ConstraintData& data,
											 const std::shared_ptr<Localization>& localization,
											 MlcpPhysicsProblem* mlcp,
											 unsigned int indexOfRepresentation,
											 unsigned int indexOfConstraint,
											 ConstraintSideSign sign)
{
	std::shared_ptr<RigidRepresentation> rigid
		= std::static_pointer_cast<RigidRepresentation>(localization->getRepresentation());

	if (!rigid->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;

	Vector3d globalPosition = localization->calculatePosition();

	// Fixed point constraint in MCLP
	//   p(t) is defined as the point after motion
	//   f is defined as position to be constrained to
	//   u is defined as the displacement needed to enforce the constraint
	// The equation is
	//   u + (p(t) - f) = 0
	//
	// For implicit integration, u = dt.I.v(t+dt)
	//
	// Note that in the physics pipeline, the constraint consits of two implementations and two localizations.  The
	// implementation is processed once for each localization, with the first being processed with scale = 1 and the
	// second with scale = -1.  This effectively takes the _difference_ of two constraints.  Therefore we can
	// effectively calculate the bilateral constraint using _only_ the position information of the localization.

	// Fill up b with the constraint violation
	mlcp->b.segment<3>(indexOfConstraint) += globalPosition * scale;

	m_newH.resize(rigid->getNumDof());
	m_newH.reserve(3);
	for (size_t index = 0; index < 3; index++)
	{
		m_newH.setZero();
		m_newH.insert(index) = dt * scale;
		mlcp->updateConstraint(m_newH, rigid->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + index);
	}
}

SurgSim::Math::MlcpConstraintType RigidRepresentationBilateral3D::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;
}

SurgSim::Physics::RepresentationType RigidRepresentationBilateral3D::getRepresentationType() const
{
	return REPRESENTATION_TYPE_RIGID;
}

unsigned int RigidRepresentationBilateral3D::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim

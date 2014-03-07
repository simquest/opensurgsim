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
	//   p(t) is defined as the point before motion
	//   s is defined as position to be constrained to [see note 1]
	//   u is defined as the displacement needed to enforce the constraint
	// The equation is
	//   u + (p(t) - s) = 0
	//
	// Using backward-euler integration,
	//   u = dt.v(t + dt)
	//
	// In rigid body dynamics, the screw vectors are represented as (x, y, z, wx, wy, wz), where the GP = (x, y, z)
	// represents the vector from the origin to the center of mass, and w = (wx, wy, wz) represents angular rotation
	// about the center of mass.  The twist vector in cartesian space is--
	//
	//   dt.v(t+dt) = dt.dGP(t+dt) + dt.GP^w(t+dt)
	//
	// The update pipeline uses a modified technique of matrix multiplication for rigid bodies.  It takes the cross
	// product with suppiled constraint components rather than the matrix product, so we must provide the appropriately
	// scaled position vector to the angular velocity component.
	//
	// We construct H to transform v(t + dt) into constrained space.  From the previous equation and comments, this
	// implies that we must multiply the translational velocity by dt and cross-multiply the angular velocity with
	// dt * GP.
	//
	// [note 1]: In the physics pipeline, the constraint consits of two implementations and two localizations.  The
	// implementation is processed once for each localization, with the first being processed with scale = 1 and the
	// second with scale = -1.  When applied together in the Mlcp, this effectively takes the difference of the two
	// constraints.  Therefore we can effectively calculate the bilateral constraint using only the position information
	// of the localization.

	// Fill up b with the constraint violation
	mlcp->b.segment<3>(indexOfConstraint) += globalPosition * scale;

	m_newH.resize(rigid->getNumDof());
	m_newH.reserve(2);
	for (size_t index = 0; index < 3; index++)
	{
		m_newH.setZero();
		m_newH.insert(index) = dt * scale;
		m_newH.insert(index + 3) = dt * scale * globalPosition[index];
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

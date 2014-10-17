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
											 size_t indexOfRepresentation,
											 size_t indexOfConstraint,
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
	// Using backward-Euler integration,
	//   u = dt.v(t + dt)
	//
	// In rigid body dynamics, the screw vectors are represented as (x, y, z, wx, wy, wz), where the G = (x, y, z)
	// represents the vector from the origin to the center of mass, and w = (wx, wy, wz) represents angular rotation
	// about the center of mass.  The twist vector in Cartesian space is--
	//
	//   dt.v(t+dt) = dt.dG(t+dt) + dt.GP^w(t+dt), where G is the position of the center of mass,
	//                                             dG is the velocity of the center of mass,
	//                                             and GP is the vector from the center of mass to the point of interest
	//              = dt.dG(t+dt) + dt.|i   j   k  |
	//                                 |GPx GPy GPz|, where i, j, k are unit vectors for the x, y, and z directions
	//                                 |wx  wy  wz |
	//              = dt.dG(t+dt) + dt.[ GPy.wz - GPz.wy]
	//                                 [-GPx.wz + GPz.wx]
	//                                 [ GPx.wy - GPy.wx]
	//              = dt.dG(t+dt) + dt.[ 0   -GPz  GPy].w
	//                                 [ GPz  0   -GPx]
	//                                 [-GPy  GPx  0  ]
	//
	// We construct H to transform v(t + dt) into constrained space.  Therefore we multiply the translational velocity
	// by dt, and we must multiply the angular velocity with the skew-symmetric matrix of GP times dt.
	//
	// [note 1]: In the physics pipeline, the constraint consists of two implementations and two localizations.  The
	// implementation is processed once for each localization, with the first being processed with scale = 1 and the
	// second with scale = -1.  When applied together in the Mlcp, this effectively takes the difference of the two
	// constraints.  Therefore we can effectively calculate the bilateral constraint using only the position information
	// of the localization.

	// Fill up b with the constraint violation
	mlcp->b.segment<3>(indexOfConstraint) += globalPosition * scale;

	// Fill up H with the transform from rigid body velocity -> constraint space
	Vector3d GP = globalPosition - rigid->getCurrentState().getPose() * rigid->getMassCenter();
	m_newH.resize(rigid->getNumDof());
	m_newH.reserve(3);

	m_newH.insert(0) = dt * scale;
	m_newH.insert(3 + 1) = -dt * scale * GP.z();
	m_newH.insert(3 + 2) = dt * scale * GP.y();
	mlcp->updateConstraint(m_newH, rigid->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + 0);

	m_newH.setZero();
	m_newH.insert(1) = dt * scale;
	m_newH.insert(3 + 0) = dt * scale * GP.z();
	m_newH.insert(3 + 2) = -dt * scale * GP.x();
	mlcp->updateConstraint(m_newH, rigid->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + 1);

	m_newH.setZero();
	m_newH.insert(2) = dt * scale;
	m_newH.insert(3 + 0) = -dt * scale * GP.y();
	m_newH.insert(3 + 1) = dt * scale * GP.x();
	mlcp->updateConstraint(m_newH, rigid->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + 2);
}

SurgSim::Math::MlcpConstraintType RigidRepresentationBilateral3D::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;
}

SurgSim::Physics::RepresentationType RigidRepresentationBilateral3D::getRepresentationType() const
{
	return REPRESENTATION_TYPE_RIGID;
}

size_t RigidRepresentationBilateral3D::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim

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

#include "SurgSim/Physics/RigidRepresentationContact.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/Localization.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

RigidRepresentationContact::RigidRepresentationContact()
{

}

RigidRepresentationContact::~RigidRepresentationContact()
{

}

void RigidRepresentationContact::doBuild(double dt,
			const ConstraintData& data,
			const std::shared_ptr<Localization>& localization,
			MlcpPhysicsProblem* mlcp,
			size_t indexOfRepresentation,
			size_t indexOfConstraint,
			ConstraintSideSign sign)
{
	std::shared_ptr<Representation> representation = localization->getRepresentation();
	std::shared_ptr<RigidRepresentation> rigid = std::static_pointer_cast<RigidRepresentation>(representation);

	if (! rigid->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE ? 1.0 : -1.0);
	const Eigen::Matrix<double, 6,6, Eigen::RowMajor>& C = rigid->getComplianceMatrix();
	const ContactConstraintData& contactData = static_cast<const ContactConstraintData&>(data);
	const Vector3d& n = contactData.getNormal();

	// FRICTIONLESS CONTACT in a LCP
	//   (n, d) defines the plane of contact
	//   P(t) the point of contact (usually after free motion)
	// The constraint equation is: n.P(t+dt) + d >= 0
	// n.[ P(t) + dt.V(t+dt) ] + d >= 0   (using the numerical integration scheme Backward Euler)
	// n.dt.[dG(t+dt) + w(t+dt)^GP] + n.P(t) + d >= 0
	// n.dt.[dGx(t+dt) + (wy(t+dt).GPz-wz(t+dt).GPy)] + n.P(t) + d >= 0        ]
	//      [dGy(t+dt) + (wz(t+dt).GPx-wx(t+dt).GPz)]
	//      [dGz(t+dt) + (wx(t+dt).GPy-wy(t+dt).GPx)]
	// H.v(t+dt) + b >= 0
	// H = dt.[nx  ny  nz  nz.GPy-ny.GPz  nx.GPz-nz.GPx  ny.GPx-nx.GPy]
	// b = n.P(t) + d             -> P(t) evaluated after free motion
	// Since the d term will be added to the constraint for one side of the contact and subtracted from the other,
	// and because it is not clear which distance should be used, we leave it out.

	Vector3d globalPosition = localization->calculatePosition();
	Vector3d GP = globalPosition - rigid->getCurrentState().getPose() * rigid->getMassCenter();

	// Fill up b with the constraint equation...
	double violation = n.dot(globalPosition);
	mlcp->b[indexOfConstraint] += violation * scale;

	m_newH.resize(rigid->getNumDof());
	m_newH.reserve(6);
	m_newH.insert(0) = dt * scale * n[0];
	m_newH.insert(1) = dt * scale * n[1];
	m_newH.insert(2) = dt * scale * n[2];
	Eigen::Vector3d rotation = GP.cross(n);
	m_newH.insert(3) = dt * scale * rotation[0];
	m_newH.insert(4) = dt * scale * rotation[1];
	m_newH.insert(5) = dt * scale * rotation[2];

	mlcp->updateConstraint(m_newH, C, indexOfRepresentation, indexOfConstraint);
}

SurgSim::Math::MlcpConstraintType RigidRepresentationContact::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
}

SurgSim::Physics::RepresentationType RigidRepresentationContact::getRepresentationType() const
{
	return REPRESENTATION_TYPE_RIGID;
}

size_t RigidRepresentationContact::doGetNumDof() const
{
	return 1;
}

}; // namespace Physics

}; // namespace SurgSim

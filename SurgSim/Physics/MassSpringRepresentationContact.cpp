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

#include "SurgSim/Physics/MassSpringRepresentationContact.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"

#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/MassSpringRepresentationLocalization.h"

namespace SurgSim
{

namespace Physics
{

MassSpringRepresentationContact::MassSpringRepresentationContact()
{

}

MassSpringRepresentationContact::~MassSpringRepresentationContact()
{

}

void MassSpringRepresentationContact::doBuild(double dt,
			const ConstraintData& data,
			const std::shared_ptr<Localization>& localization,
			MlcpPhysicsProblem* mlcp,
			unsigned int indexOfRepresentation,
			unsigned int indexOfConstraint,
			ConstraintSideSign sign)
{
	MlcpPhysicsProblem::Matrix& H    = mlcp->H;
	MlcpPhysicsProblem::Matrix& CHt  = mlcp->CHt;
	MlcpPhysicsProblem::Matrix& HCHt = mlcp->A;
	MlcpPhysicsProblem::Vector& b    = mlcp->b;

	unsigned int nodeId = std::static_pointer_cast<MassSpringRepresentationLocalization>(localization)->getLocalNode();
	std::shared_ptr<Representation> representation = localization->getRepresentation();
	auto massSpring = std::static_pointer_cast<MassSpringRepresentation>(representation);

	if ( !massSpring->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const SurgSim::Math::Matrix& C = massSpring->getComplianceMatrix();
	const ContactConstraintData& contactData = static_cast<const ContactConstraintData&>(data);
	const SurgSim::Math::Vector3d& n = contactData.getNormal();
	const double d = contactData.getDistance();

	// FRICTIONLESS CONTACT in a LCP
	//   (n, d) defines the plane of contact
	//   P(t) the point of contact (usually after free motion)
	//
	// The constraint equation for a plane is
	//   n.P(t+dt) + d >= 0
	// Using the Backward Euler numerical integration scheme
	//   n.[ P(t) + dt.V(t+dt) ] + d >= 0
	//   [n.dt].V(t+dt) + [n.P(t) + d] >= 0
	// The solver requires an equation of the form
	//   H.V(t+dt) + b >= 0
	// Therefore
	//   H = n.dt
	//   b = n.P(t) + d             -> P(t) evaluated after free motion

	// Fill up b with the constraint equation...
	SurgSim::Math::Vector3d globalPosition = localization->calculatePosition();
	double violation = n.dot(globalPosition) + d;
	b[indexOfConstraint] += violation * scale;

	// Fill matrices with just the non null values
	// (H+H') = H+H'
	// => H += H';
	//
	// C(H+H')t = CHt + CH't
	// => CHt += CH't;
	//
	// (H+H')C(H+H')t = HCHt + HCH't + H'C(H+H')t
	// => HCHt += H(CH't) + H'[C(H+H')t];

	// H'
	SurgSim::Math::Vector3d localH = dt * scale * n;

	// CH't
	SurgSim::Math::Vector localCHt = C.middleCols(indexOfRepresentation + 3*nodeId, 3) * localH;

	// HCHt += H(CH't)
	HCHt.col(indexOfConstraint) += H * localCHt;

	// H += H'
	H.block<1,3>(indexOfConstraint, indexOfRepresentation + 3*nodeId) += localH.transpose();

	// CHt += CH't
	CHt.col(indexOfConstraint) += localCHt;

	// HCHt += H'[C(H+H')t]
	HCHt.row(indexOfConstraint) += localH.transpose() * CHt.middleRows(indexOfRepresentation + 3*nodeId, 3);

}

SurgSim::Math::MlcpConstraintType MassSpringRepresentationContact::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
}

SurgSim::Physics::RepresentationType MassSpringRepresentationContact::getRepresentationType() const
{
	return REPRESENTATION_TYPE_MASSSPRING;
}

unsigned int MassSpringRepresentationContact::doGetNumDof() const
{
	return 1;
}



}; // namespace Physics

}; // namespace SurgSim
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

	std::shared_ptr<Representation> representation = localization->getRepresentation();
	std::shared_ptr<MassSpringRepresentation> massSpring = std::static_pointer_cast<MassSpringRepresentation>(representation);

	if (!massSpring->isActive())
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

	// Fill up H with just the non null values
	H.block<1,3>(indexOfConstraint, indexOfRepresentation + 0) += dt * scale * n;

	// Fill up CH^t with just the non null values
	for (unsigned int CHt_line = 0; CHt_line < massSpring->getNumDof(); CHt_line++)
	{
		CHt(indexOfRepresentation + CHt_line, indexOfConstraint) +=
			C.block<1,3>(CHt_line, 0) * H.block<1,3>(indexOfConstraint, indexOfRepresentation).transpose();
	}

	// Fill up HCHt (add 1 line and 1 column to it)
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (unsigned int col = 0; col < indexOfConstraint; col++)
	{
		HCHt(indexOfConstraint, col) +=
			H.block<1, 3>(indexOfConstraint, indexOfRepresentation) * CHt.block<3, 1>(indexOfRepresentation, col);
		HCHt(col, indexOfConstraint) = HCHt(indexOfConstraint, col);
	}
	HCHt(indexOfConstraint, indexOfConstraint) +=
		H.block<1, 3>(indexOfConstraint, indexOfRepresentation) *
		CHt.block<3, 1>(indexOfRepresentation, indexOfConstraint);
}

SurgSim::Math::MlcpConstraintType MassSpringRepresentationContact::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
}

SurgSim::Physics::RepresentationType MassSpringRepresentationContact::getRepresentationType() const
{
	return 	REPRESENTATION_TYPE_MASSSPRING;
}

unsigned int MassSpringRepresentationContact::doGetNumDof() const
{
	return 1;
}



}; // namespace Physics

}; // namespace SurgSim
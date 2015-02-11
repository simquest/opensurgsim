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
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/FixedRepresentationContact.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{

namespace Physics
{

FixedRepresentationContact::FixedRepresentationContact()
{
}

FixedRepresentationContact::~FixedRepresentationContact()
{
}

void FixedRepresentationContact::doBuild(double dt,
	const ConstraintData& data,
	const std::shared_ptr<Localization>& localization,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation,
	size_t indexOfConstraint,
	ConstraintSideSign sign)
{
	MlcpPhysicsProblem::Vector& b = mlcp->b;

	std::shared_ptr<Representation> representation = localization->getRepresentation();
	std::shared_ptr<FixedRepresentation> fixed = std::static_pointer_cast<FixedRepresentation>(representation);

	if (! fixed->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE ? 1.0 : -1.0);
	const ContactConstraintData& contactData = static_cast<const ContactConstraintData&>(data);
	const SurgSim::Math::Vector3d& n = contactData.getNormal();

	// FRICTIONLESS CONTACT in a LCP
	//   (n, d) defines the plane of contact
	//   P(t) the point of contact
	// b = n.P(t) + d
	// Since the d term will be added to the constraint for one side of the contact and subtracted from the other,
	// and because it is not clear which distance should be used, we leave it out.

	SurgSim::Math::Vector3d globalPosition = localization->calculatePosition();

	// Fill up b with the constraint equation...
	double violation = n.dot(globalPosition);
	b[indexOfConstraint] += violation * scale;
}

SurgSim::Math::MlcpConstraintType FixedRepresentationContact::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
}

std::string FixedRepresentationContact::getRepresentationType() const
{
	//	auto representation = std::make_shared<FixedRepresentation>("temp");
	//	return representation->getClassName();
	return "SurgSim::Physics::FixedRepresentation";
}

size_t FixedRepresentationContact::doGetNumDof() const
{
	return 1;
}

};  //  namespace Physics

};  //  namespace SurgSim

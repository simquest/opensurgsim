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

#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/FixedRepresentationBilateral3D.h"
#include "SurgSim/Physics/Localization.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

FixedRepresentationBilateral3D::FixedRepresentationBilateral3D()
{
}

FixedRepresentationBilateral3D::~FixedRepresentationBilateral3D()
{
}

void FixedRepresentationBilateral3D::doBuild(double dt,
											 const ConstraintData& data,
											 const std::shared_ptr<Localization>& localization,
											 MlcpPhysicsProblem* mlcp,
											 size_t indexOfRepresentation,
											 size_t indexOfConstraint,
											 ConstraintSideSign sign)
{
	std::shared_ptr<Representation> representation = localization->getRepresentation();

	if (!representation->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;

	Vector3d globalPosition = localization->calculatePosition();

	// Fill up b with the constraint equation...
	mlcp->b.segment<3>(indexOfConstraint) += globalPosition * scale;
}

SurgSim::Math::MlcpConstraintType FixedRepresentationBilateral3D::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;
}

size_t FixedRepresentationBilateral3D::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim

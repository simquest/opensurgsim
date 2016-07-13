// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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
#include "SurgSim/Physics/RigidConstraintFixedRotationVector.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RotationVectorConstraintData.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

RigidConstraintFixedRotationVector::RigidConstraintFixedRotationVector()
{
}

RigidConstraintFixedRotationVector::~RigidConstraintFixedRotationVector()
{
}

void RigidConstraintFixedRotationVector::doBuild(double dt,
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
	const RotationVectorRigidFem1DConstraintData& constRotVecData =
		static_cast<const RotationVectorRigidFem1DConstraintData&>(data);
	RotationVectorRigidFem1DConstraintData& rotVecData =
		const_cast<RotationVectorRigidFem1DConstraintData&>(constRotVecData);
	SurgSim::Math::Vector3d rotationVector = rotVecData.getCurrentRotationVector();

	// Fill up b with the constraint violation
	mlcp->b.segment<3>(indexOfConstraint) += rotationVector * scale;
}

SurgSim::Physics::ConstraintType RigidConstraintFixedRotationVector::getConstraintType() const
{
	return SurgSim::Physics::FIXED_3DROTATION_VECTOR;
}

size_t RigidConstraintFixedRotationVector::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim

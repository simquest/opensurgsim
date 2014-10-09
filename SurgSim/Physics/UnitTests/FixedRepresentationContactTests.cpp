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

#include <gtest/gtest.h>
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/FixedRepresentationContact.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationContact.h"

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{
TEST (FixedRepresentationContactTests, SetGet_BuildMlcp_Test)
{
	SurgSim::Math::Vector3d n(0.0, 1.0, 0.0);
	double d = 0.0;
	double violation = -0.01;

	SurgSim::Math::Vector3d contactPosition = -n * (d - violation);
	SurgSim::Math::RigidTransform3d poseFixed;
	poseFixed.setIdentity();

	std::shared_ptr<FixedRepresentation> fixed = std::make_shared<FixedRepresentation>("Fixed");
	fixed->setLocalActive(true);
	fixed->setIsGravityEnabled(false);
	fixed->setLocalPose(poseFixed);

	auto loc = std::make_shared<FixedRepresentationLocalization>(fixed);
	loc->setLocalPosition(contactPosition);
	std::shared_ptr<FixedRepresentationContact> implementation = std::make_shared<FixedRepresentationContact>();

	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, implementation->getMlcpConstraintType());
	EXPECT_EQ(1u, implementation->getNumDof());

	ContactConstraintData constraintData;
	constraintData.setPlaneEquation(n, d);

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(fixed->getNumDof(), 1, 1);

	// Fill up the Mlcp
	double dt = 1e-3;
	implementation->build(dt, constraintData, loc, &mlcpPhysicsProblem,
						  0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	// b should be exactly the violation
	EXPECT_NEAR(violation, mlcpPhysicsProblem.b[0], epsilon);

	// Constraint H should be [] (a fixed representation has no dof !)

	// ConstraintTypes should contain 0 entry as it is setup by the constraint and not the ConstraintImplementation
	// This way, the constraint can verify that both ConstraintImplementation are the same type
	ASSERT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

};  //  namespace Physics
};  //  namespace SurgSim

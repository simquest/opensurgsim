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
#include <SurgSim/Physics/Constraint.h>
#include <SurgSim/Physics/ConstraintData.h>
#include <SurgSim/Physics/ContactConstraintData.h>
#include <SurgSim/Physics/FixedRepresentation.h>
#include <SurgSim/Physics/FixedRepresentationContact.h>
#include <SurgSim/Physics/FixedRepresentationLocalization.h>
#include <SurgSim/Physics/MlcpPhysicsProblem.h>
#include <SurgSim/Physics/RigidRepresentation.h>
#include <SurgSim/Physics/RigidRepresentationContact.h>
#include <SurgSim/Physics/RigidRepresentationLocalization.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>
#include <SurgSim/Physics/SphereShape.h>
using SurgSim::Physics::Constraint;
using SurgSim::Physics::ConstraintData;
using SurgSim::Physics::ContactConstraintData;
using SurgSim::Physics::ConstraintImplementation;
using SurgSim::Physics::Localization;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::FixedRepresentationContact;
using SurgSim::Physics::FixedRepresentationLocalization;
using SurgSim::Physics::MlcpPhysicsProblem;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationContact;
using SurgSim::Physics::RigidRepresentationLocalization;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::SphereShape;

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace
{
	const double epsilon = 1e-10;
};

TEST (FixedRepresentationContactTests, SetGet_BuildMlcp_Test)
{
	RigidTransform3d poseFixed;
	poseFixed.setIdentity();

	std::shared_ptr<FixedRepresentation> fixed = std::make_shared<FixedRepresentation>("Fixed");
	fixed->setIsActive(true);
	fixed->setIsGravityEnabled(false);
	fixed->setInitialPose(poseFixed);

	std::shared_ptr<FixedRepresentationLocalization> loc = std::make_shared<FixedRepresentationLocalization>(fixed);
	loc->setLocalPosition(Vector3d(0.0, 0.0, 0.0));
	std::shared_ptr<FixedRepresentationContact> implementation = std::make_shared<FixedRepresentationContact>(loc);

	EXPECT_EQ(loc, implementation->getLocalization());
	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, implementation->getMlcpConstraintType());
	EXPECT_EQ(1u, implementation->getNumDof());

	ContactConstraintData constraintData;
	Vector3d n(0.0, 1.0, 0.0);
	double d = 0.0;
	constraintData.setPlaneEquation(n, d);

	MlcpPhysicsProblem mlcpPhysicsProblem;
	// Resize and zero all Eigen types
	mlcpPhysicsProblem.A.resize(1u, 1u);
	mlcpPhysicsProblem.A.setZero();
	mlcpPhysicsProblem.b.resize(1u);
	mlcpPhysicsProblem.b.setZero();
	mlcpPhysicsProblem.mu.resize(1u);
	mlcpPhysicsProblem.mu.setZero();
	mlcpPhysicsProblem.CHt.resize(fixed->getNumDof(), 1u);
	mlcpPhysicsProblem.CHt.setZero();
	mlcpPhysicsProblem.H.resize(1u, fixed->getNumDof());
	mlcpPhysicsProblem.H.setZero();
	// Empty all std::vector types
	mlcpPhysicsProblem.constraintTypes.clear();

	// Fill up the Mlcp
	double dt = 1e-3;
	implementation->build(dt, constraintData, mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	// Violation b should be exactly -radius (the sphere center is on the plane)
	EXPECT_NEAR(0.0, mlcpPhysicsProblem.b[0], epsilon);
	
	// Constraint H should be [] (a fixed representation has no dof !)

	// ConstraintTypes should contain 0 entry as it is setup by the constraint and not the ConstraintImplementation
	// This way, the constraint can verify that both ConstraintImplementation are the same type
	ASSERT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

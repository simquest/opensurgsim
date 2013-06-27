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


TEST (ConstraintTests, TestDefaultEmptyConstraint)
{
	Constraint constraint;

	EXPECT_EQ(nullptr, constraint.getData());
	EXPECT_EQ(nullptr, constraint.getImplementations().first);
	EXPECT_EQ(nullptr, constraint.getImplementations().second);
}

TEST (ConstraintTests, TestGetNumDof)
{
	Constraint constraint;

	// 0 by default
	EXPECT_EQ(0u, constraint.getNumDof());

	// 1 for a frictionless contact between 2 fixed representations
	{
		std::shared_ptr<Localization> loc1 = std::make_shared<FixedRepresentationLocalization>();
		std::shared_ptr<Localization> loc2 = std::make_shared<FixedRepresentationLocalization>();
		std::shared_ptr<FixedRepresentationContact> implementation1 = std::make_shared<FixedRepresentationContact>(loc1);
		std::shared_ptr<FixedRepresentationContact> implementation2 = std::make_shared<FixedRepresentationContact>(loc2);
		constraint.setImplementations(implementation1, implementation2);
		EXPECT_EQ(1u, constraint.getNumDof());
	}

	// 1 for a frictionless contact between 1 fixed representation and 1 rigid representation
	{
		std::shared_ptr<Localization> loc1 = std::make_shared<FixedRepresentationLocalization>();
		std::shared_ptr<Localization> loc2 = std::make_shared<RigidRepresentationLocalization>();
		std::shared_ptr<ConstraintImplementation> implementation1 = std::make_shared<FixedRepresentationContact>(loc1);
		std::shared_ptr<ConstraintImplementation> implementation2 = std::make_shared<RigidRepresentationContact>(loc2);
		constraint.setImplementations(implementation1, implementation2);
		EXPECT_EQ(1u, constraint.getNumDof());
	}
}

TEST (ConstraintTests, TestSetGetData)
{
	Constraint constraint;
	std::shared_ptr<ConstraintData> constraintData = std::make_shared<ConstraintData>();

	constraint.setData(constraintData);
	EXPECT_NE(nullptr, constraint.getData());
	EXPECT_EQ(constraintData, constraint.getData());
	EXPECT_EQ(nullptr, constraint.getImplementations().first);
	EXPECT_EQ(nullptr, constraint.getImplementations().second);
}

TEST (ConstraintTests, TestSetGetImplementations)
{
	Constraint constraint;
	std::shared_ptr<Localization> loc1 = std::make_shared<FixedRepresentationLocalization>();
	std::shared_ptr<Localization> loc2 = std::make_shared<FixedRepresentationLocalization>();
	std::shared_ptr<FixedRepresentationContact> implementation1 = std::make_shared<FixedRepresentationContact>(loc1);
	std::shared_ptr<FixedRepresentationContact> implementation2 = std::make_shared<FixedRepresentationContact>(loc2);

	constraint.setImplementations(implementation1, implementation2);
	EXPECT_EQ(nullptr, constraint.getData());
	EXPECT_NE(nullptr, constraint.getImplementations().first);
	EXPECT_NE(nullptr, constraint.getImplementations().second);
	EXPECT_EQ(implementation1, constraint.getImplementations().first);
	EXPECT_EQ(implementation2, constraint.getImplementations().second);
}

TEST (ConstraintTests, TestClear)
{
	Constraint constraint;
	std::shared_ptr<Localization> loc1 = std::make_shared<FixedRepresentationLocalization>();
	std::shared_ptr<Localization> loc2 = std::make_shared<FixedRepresentationLocalization>();
	std::shared_ptr<FixedRepresentationContact> implementation1 = std::make_shared<FixedRepresentationContact>(loc1);
	std::shared_ptr<FixedRepresentationContact> implementation2 = std::make_shared<FixedRepresentationContact>(loc2);
	std::shared_ptr<ConstraintData> constraintData = std::make_shared<ConstraintData>();

	constraint.setData(constraintData);
	constraint.setImplementations(implementation1, implementation2);
	EXPECT_NE(nullptr, constraint.getData());
	EXPECT_NE(nullptr, constraint.getImplementations().first);
	EXPECT_NE(nullptr, constraint.getImplementations().second);
	constraint.clear();
	EXPECT_EQ(nullptr, constraint.getData());
	EXPECT_EQ(nullptr, constraint.getImplementations().first);
	EXPECT_EQ(nullptr, constraint.getImplementations().second);
}

// Contact plane equation is (n=(0 1 0) d=0)
// 2 representations involved are: 1 fixed and 1 rigid (sphere of radius 0.01)
// Localization on the fixed is (0 0 0)
// Localization on the rigid is (0 -0.01 0)
// => violation of -0.1m
TEST (ConstraintTests, TestBuildMlcp)
{
	RigidTransform3d poseFixed, poseRigid;
	Vector3d n(0.0, 1.0, 0.0);
	double d = 0.0;
	double radius = 0.01;
	double dt = 1e-3;
	unsigned int indexRepresentation0 = 0;
	unsigned int indexRepresentation1;

	poseFixed.setIdentity();
	poseRigid.setIdentity();

	std::shared_ptr<FixedRepresentation> fixed = std::make_shared<FixedRepresentation>("Fixed");
	fixed->setIsActive(true);
	fixed->setIsGravityEnabled(false);
	fixed->setInitialPose(poseFixed);

	indexRepresentation1 = indexRepresentation0 + fixed->getNumDof();
	std::shared_ptr<RigidRepresentation> rigid = std::make_shared<RigidRepresentation>("Rigid");
	rigid->setIsActive(true);
	rigid->setIsGravityEnabled(false);
	rigid->setInitialPose(poseRigid);
	{
		RigidRepresentationParameters param;
		param.setDensity(1000.0);
		std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(radius);
		param.setShapeUsedForMassInertia(shape);
		rigid->setInitialParameters(param);
	}

	// Simulate 1 time step...to make sure all representation have a valid compliance matrix...
	{
		fixed->beforeUpdate(dt);
		rigid->beforeUpdate(dt);

		fixed->update(dt);
		rigid->update(dt);

		fixed->afterUpdate(dt);
		rigid->afterUpdate(dt);
	}

	Constraint constraint;
	std::shared_ptr<FixedRepresentationLocalization> loc1 = std::make_shared<FixedRepresentationLocalization>(fixed);
	std::shared_ptr<RigidRepresentationLocalization> loc2 = std::make_shared<RigidRepresentationLocalization>(rigid);
	loc1->setLocalPosition(Vector3d(0.0, 0.0, 0.0));
	loc2->setLocalPosition(Vector3d(0.0, -radius, 0.0));
	std::shared_ptr<ConstraintImplementation> implementation1 = std::make_shared<FixedRepresentationContact>(loc1);
	std::shared_ptr<ConstraintImplementation> implementation2 = std::make_shared<RigidRepresentationContact>(loc2);
	constraint.setImplementations(implementation1, implementation2);

	std::shared_ptr<ContactConstraintData> constraintData = std::make_shared<ContactConstraintData>();
	constraintData->setPlaneEquation(n, d);
	constraint.setData(constraintData);

	MlcpPhysicsProblem mlcpPhysicsProblem;
	mlcpPhysicsProblem.A.resize(constraint.getNumDof(), constraint.getNumDof());
	mlcpPhysicsProblem.A.setZero();
	mlcpPhysicsProblem.b.resize(constraint.getNumDof());
	mlcpPhysicsProblem.b.setZero();
	mlcpPhysicsProblem.constraintTypes.resize(constraint.getNumDof());
	mlcpPhysicsProblem.mu.resize(constraint.getNumDof());
	mlcpPhysicsProblem.mu.setZero();
	mlcpPhysicsProblem.CHt.resize(fixed->getNumDof()+rigid->getNumDof(), constraint.getNumDof());
	mlcpPhysicsProblem.CHt.setZero();
	mlcpPhysicsProblem.H.resize(constraint.getNumDof(), fixed->getNumDof()+rigid->getNumDof());
	mlcpPhysicsProblem.H.setZero();
	constraint.build(dt, mlcpPhysicsProblem, indexRepresentation0, indexRepresentation1, 0);
}

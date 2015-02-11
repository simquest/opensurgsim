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

#include <gtest/gtest.h>

#include <memory>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationBilateral3D.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Math::makeSkewSymmetricMatrix;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace
{
const double epsilon = 1e-10;
const double dt = 1e-3;
};

namespace SurgSim
{
namespace Physics
{

TEST(RigidRepresentationBilateral3DTests, Constructor)
{
	ASSERT_NO_THROW(
		{ RigidRepresentationBilateral3D constraint; });
}

TEST(RigidRepresentationBilateral3DTests, Constants)
{
	RigidRepresentationBilateral3D constraint;

	EXPECT_EQ(SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT, constraint.getMlcpConstraintType());
	EXPECT_EQ("SurgSim::Physics::RigidRepresentation", constraint.getRepresentationType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(RigidRepresentationBilateral3DTests, BuildMlcp)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not neccessarily construct the physical parameters
	// neccessary to supply a realistic C.  It only checks H and b.
	RigidRepresentationBilateral3D constraint;

	Vector3d centerOfMass = Vector3d(3.0, 2.42, 9.54);
	Quaterniond objectRotation = Quaterniond(0.1, 0.35, 4.2, 5.0).normalized();

	RigidTransform3d objectPose = makeRigidTransform(objectRotation, centerOfMass);
	Vector3d constraintPoint = Vector3d(8.0, 6.4, 3.5);

	// Setup parameters for RigidRepresentationBilateral3D::build
	auto representation = std::make_shared<MockRigidRepresentation>();
	representation->setShape(std::make_shared<SurgSim::Math::SphereShape>(1.0));
	auto localization = std::make_shared<RigidRepresentationLocalization>(representation);
	localization->setLocalPosition(objectPose.inverse() * constraintPoint);
	representation->getCurrentState().setPose(objectPose);

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(6, 3, 1);

	ConstraintData emptyConstraint;

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = constraintPoint;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(dt * identity,
		0, 0, 3, 3, &H);
	SurgSim::Math::setSubMatrix(makeSkewSymmetricMatrix((dt * (constraintPoint - centerOfMass)).eval()),
		0, 1, 3, 3, &H);
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(RigidRepresentationBilateral3DTests, BuildMlcpTwoStep)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not neccessarily construct the physical parameters
	// neccessary to supply a realistic C.  It only checks H and b.
	RigidRepresentationBilateral3D constraint;

	Vector3d centerOfMassLhs = Vector3d(3.0, 2.42, 9.54);
	Vector3d centerOfMassRhs = Vector3d(1.0, 24.52, 8.00);

	Quaterniond objectRotationLhs = Quaterniond(0.1, 0.35, 4.2, 5.0).normalized();
	Quaterniond objectRotationRhs = Quaterniond(1.43, 6.21, 7.11, 0.55).normalized();

	RigidTransform3d objectPoseLhs = makeRigidTransform(objectRotationLhs, centerOfMassLhs);
	RigidTransform3d objectPoseRhs = makeRigidTransform(objectRotationRhs, centerOfMassRhs);

	Vector3d constraintPointLhs = Vector3d(8.0, 6.4, 3.5);
	Vector3d constraintPointRhs = Vector3d(3.0, 7.7, 0.0);

	// Setup parameters for RigidRepresentationBilateral3D::build
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(12, 3, 1);

	ConstraintData emptyConstraint;

	auto representation = std::make_shared<MockRigidRepresentation>();
	representation->setShape(std::make_shared<SurgSim::Math::SphereShape>(1.0));
	auto localization = std::make_shared<RigidRepresentationLocalization>(representation);

	localization->setLocalPosition(objectPoseLhs.inverse() * constraintPointLhs);
	representation->getCurrentState().setPose(objectPoseLhs);
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	localization->setLocalPosition(objectPoseRhs.inverse() * constraintPointRhs);
	representation->getCurrentState().setPose(objectPoseRhs);
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 6, 0, SurgSim::Physics::CONSTRAINT_NEGATIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = constraintPointLhs - constraintPointRhs;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 12> H = Eigen::Matrix<double, 3, 12>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(dt * identity,
		0, 0, 3, 3, &H);
	SurgSim::Math::setSubMatrix(makeSkewSymmetricMatrix((dt * (constraintPointLhs - centerOfMassLhs)).eval()),
		0, 1, 3, 3, &H);
	SurgSim::Math::setSubMatrix(-dt * identity,
		0, 2, 3, 3, &H);
	SurgSim::Math::setSubMatrix(makeSkewSymmetricMatrix((-dt * (constraintPointRhs - centerOfMassRhs)).eval()),
		0, 3, 3, 3, &H);
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);
}

};  //  namespace Physics
};  //  namespace SurgSim

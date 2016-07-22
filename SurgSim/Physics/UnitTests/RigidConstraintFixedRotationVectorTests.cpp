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

#include <gtest/gtest.h>

#include <memory>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RotationVectorConstraintData.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidConstraintFixedRotationVector.h"
#include "SurgSim/Physics/RigidRepresentation.h"
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

TEST(RigidConstraintFixedRotationVectorTests, Constructor)
{
	ASSERT_NO_THROW(
	{ RigidConstraintFixedRotationVector constraint; });
}

TEST(RigidConstraintFixedRotationVectorTests, Constants)
{
	RigidConstraintFixedRotationVector constraint;

	EXPECT_EQ(SurgSim::Physics::FIXED_3DROTATION_VECTOR, constraint.getConstraintType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(RigidConstraintFixedRotationVectorTests, BuildMlcp)
{
	using SurgSim::Framework::Runtime;

	RigidConstraintFixedRotationVector constraint;

	// Prepare the fem1d representation for this constraint type
	auto fem1d = std::make_shared<Fem1DRepresentation>("fem1d");
	auto initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(6, 2);
	initialState->getPositions().setZero();
	initialState->getPositions().segment<3>(6) = SurgSim::Math::Vector3d(1.0, 0.0, 0.0);
	fem1d->setInitialState(initialState);
	std::array<size_t, 2> nodeIds = { { 0, 1 } };
	auto beam = std::make_shared<Fem1DElementBeam>(nodeIds);
	beam->setMassDensity(900);
	beam->setPoissonRatio(0.4);
	beam->setRadius(0.1);
	beam->setShearingEnabled(false);
	beam->setYoungModulus(1e6);
	fem1d->addFemElement(beam);
	fem1d->initialize(std::make_shared<Runtime>()); // Initializes the beams initial rotation matrix

	// Prepare the rigid representation for this constraint type
	auto representation = std::make_shared<MockRigidRepresentation>();
	representation->setShape(std::make_shared<SurgSim::Math::SphereShape>(1.0));
	Vector3d centerOfMass = Vector3d(3.0, 2.42, 9.54);
	Quaterniond objectRotation = Quaterniond(0.1, 0.35, 4.2, 5.0).normalized();
	RigidTransform3d objectPose = makeRigidTransform(objectRotation, centerOfMass);
	representation->getCurrentState().setPose(objectPose);
	auto localization = std::make_shared<RigidLocalization>(representation);
	localization->setLocalPosition(objectPose.inverse() * Vector3d(8.0, 6.4, 3.5));

	// Prepare the specific constraint data
	RotationVectorRigidFem1DConstraintData constraintData;
	constraintData.setFem1DRotation(fem1d, 0);
	constraintData.setRigidOrFixedRotation(representation, representation->getPose().linear());

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(6, 3, 1);

	ASSERT_NO_THROW(constraint.build(
		dt, constraintData, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::AngleAxisd aa(objectRotation);
	Eigen::Matrix<double, 3, 1> violation = aa.axis() * aa.angle();
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

};  //  namespace Physics
};  //  namespace SurgSim

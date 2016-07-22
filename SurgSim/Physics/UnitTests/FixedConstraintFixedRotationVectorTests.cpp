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
#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RotationVectorConstraintData.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/FixedConstraintFixedRotationVector.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

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

TEST(FixedConstraintFixedRotationVectorTests, Constructor)
{
	ASSERT_NO_THROW(
	{ FixedConstraintFixedRotationVector constraint; });
}

TEST(FixedConstraintFixedRotationVectorTests, Constants)
{
	FixedConstraintFixedRotationVector constraint;

	EXPECT_EQ(SurgSim::Physics::FIXED_3DROTATION_VECTOR, constraint.getConstraintType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(FixedConstraintFixedRotationVectorTests, BuildMlcp)
{
	using SurgSim::Framework::Runtime;

	FixedConstraintFixedRotationVector constraint;

	// Prepare the fem1d representation for this constraint type
	auto fem1d = std::make_shared<Fem1DRepresentation>("fem1d");
	auto initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(6, 2);
	initialState->getPositions().setZero();
	initialState->getPositions().segment<3>(6) = SurgSim::Math::Vector3d(1.0, 0.0, 0.0);
	fem1d->setInitialState(initialState);
	std::array<size_t, 2> nodeIds = { {0, 1} };
	auto beam = std::make_shared<Fem1DElementBeam>(nodeIds);
	beam->setMassDensity(900);
	beam->setPoissonRatio(0.4);
	beam->setRadius(0.1);
	beam->setShearingEnabled(false);
	beam->setYoungModulus(1e6);
	fem1d->addFemElement(beam);
	fem1d->initialize(std::make_shared<Runtime>()); // Initializes the beams initial rotation matrix

	// Prepare the fixed representation for this constraint type
	auto fixed = std::make_shared<MockFixedRepresentation>();
	Vector3d centerOfMass = Vector3d(3.0, 2.42, 9.54);
	SurgSim::Math::Quaterniond objectRotation = SurgSim::Math::Quaterniond(0.1, 0.35, 4.2, 5.0).normalized();
	SurgSim::Math::RigidTransform3d objectPose = SurgSim::Math::makeRigidTransform(objectRotation, centerOfMass);
	fixed->getCurrentState().setPose(objectPose);
	auto localization = std::make_shared<FixedLocalization>(fixed);

	// Prepare the specific constraint data
	RotationVectorRigidFem1DConstraintData constraintData;
	constraintData.setFem1DRotation(fem1d, 0);
	constraintData.setRigidOrFixedRotation(fixed, fixed->getPose().linear());

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(0, 3, 1);

	ASSERT_NO_THROW(constraint.build(
		dt, constraintData, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::AngleAxisd aa(objectPose.linear());
	Eigen::Matrix<double, 3, 1> violation = aa.angle() * aa.axis();
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

};  //  namespace Physics
};  //  namespace SurgSim

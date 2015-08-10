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

#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/FixedConstraintFixedPoint.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"

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

TEST(FixedConstraintFixedPointTests, Constructor)
{
	ASSERT_NO_THROW(
		{ FixedConstraintFixedPoint constraint; });
}

TEST(FixedConstraintFixedPointTests, Constants)
{
	FixedConstraintFixedPoint constraint;

	EXPECT_EQ(SurgSim::Physics::FIXED_3DPOINT, constraint.getConstraintType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(FixedConstraintFixedPointTests, BuildMlcp)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	FixedConstraintFixedPoint constraint;

	Vector3d actual = Vector3d(8.0, 6.4, 3.5);

	// Setup parameters for FixedConstraintFixedPoint::build
	auto localization = std::make_shared<FixedLocalization>(
		std::make_shared<FixedRepresentation>("representation"));
	localization->setLocalPosition(actual);

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(0, 3, 1);

	ConstraintData emptyConstraint;

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(FixedConstraintFixedPointTests, BuildMlcpTwoStep)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	FixedConstraintFixedPoint constraint;

	Vector3d actual = Vector3d(8.0, 6.4, 3.5);
	Vector3d desired = Vector3d(3.0, 7.7, 0.0);

	// Setup parameters for FixedConstraintFixedPoint::build
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(0, 3, 1);

	ConstraintData emptyConstraint;

	auto localization = std::make_shared<FixedLocalization>(
		std::make_shared<FixedRepresentation>("representation"));

	localization->setLocalPosition(actual);
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	localization->setLocalPosition(desired);
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_NEGATIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual - desired;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);
}

};  //  namespace Physics
};  //  namespace SurgSim

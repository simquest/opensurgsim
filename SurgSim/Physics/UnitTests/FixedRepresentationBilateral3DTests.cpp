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
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/FixedRepresentationBilateral3D.h"
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

TEST(FixedRepresentationBilateral3DTests, Constructor)
{
	ASSERT_NO_THROW(
		{ FixedRepresentationBilateral3D constraint; });
}

TEST(FixedRepresentationBilateral3DTests, Constants)
{
	FixedRepresentationBilateral3D constraint;

	EXPECT_EQ(SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT, constraint.getMlcpConstraintType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(FixedRepresentationBilateral3DTests, BuildMlcp)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not neccessarily construct the physical parameters
	// neccessary to supply a realistic C.  It only checks H and b.
	FixedRepresentationBilateral3D constraint;

	Vector3d actual = Vector3d(8.0, 6.4, 3.5);

	// Setup parameters for FixedRepresentationBilateral3D::build
	auto localization = std::make_shared<FixedRepresentationLocalization>(
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

TEST(FixedRepresentationBilateral3DTests, BuildMlcpTwoStep)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not neccessarily construct the physical parameters
	// neccessary to supply a realistic C.  It only checks H and b.
	FixedRepresentationBilateral3D constraint;

	Vector3d actual = Vector3d(8.0, 6.4, 3.5);
	Vector3d desired = Vector3d(3.0, 7.7, 0.0);

	// Setup parameters for FixedRepresentationBilateral3D::build
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(0, 3, 1);

	ConstraintData emptyConstraint;

	auto localization = std::make_shared<FixedRepresentationLocalization>(
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

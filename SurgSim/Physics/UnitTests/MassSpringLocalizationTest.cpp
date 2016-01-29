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
#include <string>

#include "SurgSim/Blocks/MassSpring1DRepresentation.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/MassSpringLocalization.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

TEST (MassSpringLocalizationTest, ConstructorTest)
{
	ASSERT_NO_THROW( {MassSpringLocalization localization;});

	ASSERT_NO_THROW(
	{
		auto massSpring = std::make_shared<MassSpringRepresentation>("MassSpringRepresentation");
		MassSpringLocalization localization(massSpring);
	});
}

TEST (MassSpringLocalizationTest, SetGetRepresentationTest)
{
	MassSpringLocalization localization;
	auto massSpring = std::make_shared<MassSpringRepresentation>("MassSpringRepresentation");

	EXPECT_EQ(nullptr, localization.getRepresentation());

	localization.setRepresentation(massSpring);
	EXPECT_EQ(massSpring, localization.getRepresentation());

	localization.setRepresentation(nullptr);
	EXPECT_EQ(nullptr, localization.getRepresentation());
}

TEST (MassSpringLocalizationTest, CalculatePositionTest)
{
	using SurgSim::Math::Vector3d;

	// Create the mass spring
	auto massSpring = std::make_shared<SurgSim::Blocks::MassSpring1DRepresentation>("MassSpring");
	std::vector<Vector3d> extremities;
	extremities.push_back(Vector3d(0,0,0));
	extremities.push_back(Vector3d(1,0,0));
	std::vector<size_t> boundaryConditions;
	massSpring->init1D(
		extremities,
		boundaryConditions,
		0.1, // total mass (in Kg)
		100.0, // Stiffness stretching
		0.0, // Damping stretching
		10.0, // Stiffness bending
		0.0); // Damping bending

	MassSpringLocalization localization = MassSpringLocalization(massSpring);

	localization.setLocalNode(0);
	ASSERT_EQ(0u, localization.getLocalNode());
	ASSERT_TRUE(localization.calculatePosition().isZero(epsilon));

	localization.setLocalNode(1);
	ASSERT_EQ(1u, localization.getLocalNode());
	ASSERT_TRUE(localization.calculatePosition().isApprox(Vector3d(1.0, 0.0, 0.0), epsilon));

	// Out-Of-Range assertions
	EXPECT_THROW(localization.calculatePosition(-0.01), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(localization.calculatePosition(1.01), SurgSim::Framework::AssertionFailure);
}

TEST(MassSpringLocalizationTest, CalculateVelocityTest)
{
	using SurgSim::Math::Vector3d;

	// Create the mass spring
	auto massSpring = std::make_shared<SurgSim::Blocks::MassSpring1DRepresentation>("MassSpring");
	std::vector<Vector3d> extremities;
	extremities.push_back(Vector3d(0, 0, 0));
	extremities.push_back(Vector3d(1, 0, 0));
	std::vector<size_t> boundaryConditions;
	massSpring->init1D(
		extremities,
		boundaryConditions,
		0.1, // total mass (in Kg)
		100.0, // Stiffness stretching
		0.0, // Damping stretching
		10.0, // Stiffness bending
		0.0); // Damping bending

	auto state = std::make_shared<Math::OdeState>(*massSpring->getInitialState());
	state->getVelocities().segment<3>(0) = Vector3d(0, 0, 0);
	state->getVelocities().segment<3>(3) = Vector3d(1, 0, 0);
	massSpring->setInitialState(state);

	MassSpringLocalization localization = MassSpringLocalization(massSpring);

	localization.setLocalNode(0);
	ASSERT_EQ(0u, localization.getLocalNode());
	ASSERT_TRUE(localization.calculateVelocity().isZero(epsilon));

	localization.setLocalNode(1);
	ASSERT_EQ(1u, localization.getLocalNode());
	ASSERT_TRUE(localization.calculateVelocity().isApprox(Vector3d(1.0, 0.0, 0.0), epsilon));

	// Out-Of-Range assertions
	EXPECT_THROW(localization.calculateVelocity(-0.01), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(localization.calculateVelocity(1.01), SurgSim::Framework::AssertionFailure);
}

TEST (MassSpringLocalizationTest, IsValidRepresentationTest)
{
	MassSpringLocalization localization;

	EXPECT_TRUE(localization.isValidRepresentation(nullptr));
	EXPECT_TRUE(localization.isValidRepresentation(std::make_shared<MassSpringRepresentation>("massSpring")));
	EXPECT_TRUE(localization.isValidRepresentation(
		std::make_shared<MockDescendent<MassSpringRepresentation>>("descendent")));

	EXPECT_FALSE(localization.isValidRepresentation(std::make_shared<MockRepresentation>()));
	EXPECT_FALSE(localization.isValidRepresentation(std::make_shared<RigidRepresentation>("rigidRepresentation")));
}

};  //  namespace Physics
};  //  namespace SurgSim

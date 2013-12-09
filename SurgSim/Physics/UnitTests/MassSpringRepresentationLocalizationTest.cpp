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

#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/MassSpringRepresentationLocalization.h"
#include "SurgSim/Blocks/MassSpring1DRepresentation.h"

#include "SurgSim/Math/Vector.h"

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

TEST (MassSpringRepresentationLocalizationTest, ConstructorTest)
{
	ASSERT_NO_THROW( {MassSpringRepresentationLocalization localization;});

	ASSERT_NO_THROW(
	{
		auto massSpring = std::make_shared<MassSpringRepresentation>("MassSpringRepresentation");
		MassSpringRepresentationLocalization localization(massSpring);
	});
}

TEST (MassSpringRepresentationLocalizationTest, SetGetRepresentation)
{
	MassSpringRepresentationLocalization localization;
	auto massSpring = std::make_shared<MassSpringRepresentation>("MassSpringRepresentation");

	EXPECT_EQ(nullptr, localization.getRepresentation());

	localization.setRepresentation(massSpring);
	EXPECT_EQ(massSpring, localization.getRepresentation());

	localization.setRepresentation(nullptr);
	EXPECT_EQ(nullptr, localization.getRepresentation());
}

TEST (MassSpringRepresentationLocalizationTest, GetPositionTest)
{
	using SurgSim::Math::Vector3d;

	// Create the rigid body
	auto massSpring = std::make_shared<SurgSim::Blocks::MassSpring1DRepresentation>("MassSpring");
	std::array<Vector3d, 2> extremities = {{ Vector3d(0,0,0), Vector3d(1,0,0) }};
	unsigned int numNodesPerDim[1] = {2};
	std::vector<unsigned int> boundaryConditions;
	massSpring->init1D(
		extremities,
		numNodesPerDim,
		boundaryConditions,
		0.1, // total mass (in Kg)
		100.0, // Stiffness stretching
		0.0, // Damping stretching
		10.0, // Stiffness bending
		0.0); // Damping bending

	massSpring->setIsActive(true);

	MassSpringRepresentationLocalization localization = MassSpringRepresentationLocalization(massSpring);

	localization.setLocalNode(0);
	ASSERT_EQ(0, localization.getLocalNode());
	ASSERT_TRUE(localization.calculatePosition().isApprox(Vector3d(0,0,0), epsilon));

	localization.setLocalNode(1);
	ASSERT_EQ(1, localization.getLocalNode());
	ASSERT_TRUE(localization.calculatePosition().isApprox(Vector3d(1,0,0), epsilon));
}

};  //  namespace Physics
};  //  namespace SurgSim

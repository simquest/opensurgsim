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

#include <string>

#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Physics::MockRepresentation;
using SurgSim::Physics::Representation;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

TEST(RepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW({MockRepresentation representation();});
}

TEST(RepresentationTest, SetGetAndDefaultValueTest)
{
	/// Create the representation
	std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>();

	/// Get/Set active flag [default = true]
	EXPECT_TRUE(representation->isActive());
	representation->setIsActive(false);
	ASSERT_FALSE(representation->isActive());
	representation->setIsActive(true);
	ASSERT_TRUE(representation->isActive());

	/// Get numDof = 0
	ASSERT_EQ(0u, representation->getNumDof());

	/// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(representation->isGravityEnabled());
	representation->setIsGravityEnabled(false);
	ASSERT_FALSE(representation->isGravityEnabled());
	representation->setIsGravityEnabled(true);
	ASSERT_TRUE(representation->isGravityEnabled());

	/// Set/Get isDrivingSceneElementPose [default = true]
	EXPECT_TRUE(representation->isDrivingSceneElementPose());
	representation->setIsDrivingSceneElementPose(false);
	ASSERT_FALSE(representation->isDrivingSceneElementPose());
	representation->setIsDrivingSceneElementPose(true);
	ASSERT_TRUE(representation->isDrivingSceneElementPose());
}

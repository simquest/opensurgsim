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

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Physics::Representation;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::MockRepresentation;


TEST(RepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW({MockRepresentation representation("Representation");});
}

TEST(RepresentationTest, SetGetAndDefaultValueTest)
{
	/// Create the representation
	std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("Representation");

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
}

TEST(RepresentationTest, SetGetCollisionRepresentationTest)
{
	std::shared_ptr<Representation> physicsRepresentation = std::make_shared<MockRepresentation>("MockRepresentation");
	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("CollisionRepresentatoin");

	EXPECT_NO_THROW(physicsRepresentation->setCollisionRepresentation(collisionRepresentation));
	EXPECT_EQ(collisionRepresentation, physicsRepresentation->getCollisionRepresentation());
}

TEST(RepresentationTest, SerializationTest)
{
	std::shared_ptr<Representation> physicsRepresentation = std::make_shared<MockRepresentation>("MockRepresentation");
	physicsRepresentation->setIsActive(false);
	physicsRepresentation->setIsGravityEnabled(false);

	YAML::Node node;
	ASSERT_NO_THROW(node = physicsRepresentation->encode());
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(5u, node.size());

	std::shared_ptr<Representation> newRepresentation = std::make_shared<MockRepresentation>("Mock");
	ASSERT_NO_THROW(newRepresentation->decode(node));

	// Note: In current serialization strategy,
	// name of the decoded representation is different from the name of original representation.
	EXPECT_NE(physicsRepresentation->getName(), newRepresentation->getName());

	EXPECT_EQ("SurgSim::Physics::MockRepresentation", newRepresentation->getClassName());
	EXPECT_EQ(false, newRepresentation->isActive());
	EXPECT_EQ(false, newRepresentation->isGravityEnabled());
	EXPECT_EQ(0u, newRepresentation->getNumDof());
}

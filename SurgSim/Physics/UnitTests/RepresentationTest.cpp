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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Physics::MockRepresentation;
using SurgSim::Physics::Representation;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::MockRepresentation;


TEST(RepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW({MockRepresentation representation;});
}

TEST(RepresentationTest, SetGetAndDefaultValueTest)
{
	/// Create the representation
	std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>();

	/// Get/Set active flag [default = true]
	EXPECT_TRUE(representation->isActive());
	representation->setLocalActive(false);
	ASSERT_FALSE(representation->isLocalActive());
	ASSERT_FALSE(representation->isActive());
	representation->setLocalActive(true);
	ASSERT_TRUE(representation->isLocalActive());
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

TEST(RepresentationTest, SetGetCollisionRepresentationTest)
{
	std::shared_ptr<Representation> physicsRepresentation = std::make_shared<MockRepresentation>("MockRepresentation");
	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("CollisionRepresentatoin");

	EXPECT_NO_THROW(physicsRepresentation->setCollisionRepresentation(collisionRepresentation));
	EXPECT_EQ(collisionRepresentation, physicsRepresentation->getCollisionRepresentation());
}

TEST(RepresentationTest, SerializationTest)
{
	{
		SCOPED_TRACE("Encode instance, decoded as shared_ptr<>");
		std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("MockRepresentation");
		size_t numDof = 1;
		representation->setValue("NumDof", numDof);

		YAML::Node node;
		ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*representation));
		EXPECT_TRUE(node.IsMap());
		EXPECT_EQ(1u, node.size());

		YAML::Node data = node["SurgSim::Physics::MockRepresentation"];
		EXPECT_EQ(7u, data.size());

		std::shared_ptr<MockRepresentation> newRepresentation;
		ASSERT_NO_THROW(newRepresentation =
			std::dynamic_pointer_cast<MockRepresentation>(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

		EXPECT_EQ(representation->getName(), newRepresentation->getName());
		EXPECT_EQ("SurgSim::Physics::MockRepresentation", newRepresentation->getClassName());
		EXPECT_TRUE(newRepresentation->getValue<bool>("IsLocalActive"));
		EXPECT_TRUE(newRepresentation->getValue<bool>("IsGravityEnabled"));
		EXPECT_TRUE(newRepresentation->getValue<bool>("IsDrivingSceneElementPose"));
		EXPECT_EQ(1u, newRepresentation->getValue<size_t>("NumDof"));
	}

	{
		SCOPED_TRACE("Encode shared_ptr<>, decoded as shared_ptr<>");
		std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("MockRepresentation");

		YAML::Node node;
		ASSERT_NO_THROW(node = YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::encode(representation));
		EXPECT_TRUE(node.IsMap());
		EXPECT_EQ(1u, node.size());

		YAML::Node data = node["SurgSim::Physics::MockRepresentation"];
		EXPECT_EQ(2u, data.size());

		std::shared_ptr<MockRepresentation> newRepresentation;
		ASSERT_NO_THROW(newRepresentation =
			std::dynamic_pointer_cast<MockRepresentation>(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

		EXPECT_NE(nullptr, newRepresentation);
	}

	{
		SCOPED_TRACE("Test serialization for accesible boolean properties");
		std::shared_ptr<Representation> representation1 = std::make_shared<MockRepresentation>("MockRepresentation1");
		std::shared_ptr<Representation> representation2 = std::make_shared<MockRepresentation>("MockRepresentation2");
		std::shared_ptr<Representation> representation3 = std::make_shared<MockRepresentation>("MockRepresentation3");
		std::shared_ptr<Representation> representation4 = std::make_shared<MockRepresentation>("MockRepresentation4");

		representation1->setValue("IsLocalActive", false);

		representation2->setValue("IsGravityEnabled", false);

		representation3->setValue("IsDrivingSceneElementPose", false);

		representation4->setValue("IsLocalActive", false);
		representation4->setValue("IsGravityEnabled", false);
		representation4->setValue("IsDrivingSceneElementPose", false);

		YAML::Node node;
		ASSERT_NO_THROW(node.push_back(YAML::convert<SurgSim::Framework::Component>::encode(*representation1)));
		ASSERT_NO_THROW(node.push_back(YAML::convert<SurgSim::Framework::Component>::encode(*representation2)));
		ASSERT_NO_THROW(node.push_back(YAML::convert<SurgSim::Framework::Component>::encode(*representation3)));
		ASSERT_NO_THROW(node.push_back(YAML::convert<SurgSim::Framework::Component>::encode(*representation4)));

		std::shared_ptr<MockRepresentation> newRepresentation1;
		ASSERT_NO_THROW(newRepresentation1 =
		  std::dynamic_pointer_cast<MockRepresentation>(node[0].as<std::shared_ptr<SurgSim::Framework::Component>>()));
		std::shared_ptr<MockRepresentation> newRepresentation2;
		ASSERT_NO_THROW(newRepresentation2 =
		  std::dynamic_pointer_cast<MockRepresentation>(node[1].as<std::shared_ptr<SurgSim::Framework::Component>>()));
		std::shared_ptr<MockRepresentation> newRepresentation3;
		ASSERT_NO_THROW(newRepresentation3 =
		  std::dynamic_pointer_cast<MockRepresentation>(node[2].as<std::shared_ptr<SurgSim::Framework::Component>>()));
		std::shared_ptr<MockRepresentation> newRepresentation4;
		ASSERT_NO_THROW(newRepresentation4 =
		  std::dynamic_pointer_cast<MockRepresentation>(node[3].as<std::shared_ptr<SurgSim::Framework::Component>>()));

		EXPECT_EQ(representation1->getName(), newRepresentation1->getName());
		EXPECT_EQ(representation2->getName(), newRepresentation2->getName());
		EXPECT_EQ(representation3->getName(), newRepresentation3->getName());
		EXPECT_EQ(representation4->getName(), newRepresentation4->getName());

		EXPECT_FALSE(newRepresentation1->getValue<bool>("IsLocalActive"));
		EXPECT_TRUE(newRepresentation1->getValue<bool>("IsGravityEnabled"));
		EXPECT_TRUE(newRepresentation1->getValue<bool>("IsDrivingSceneElementPose"));

		EXPECT_TRUE(newRepresentation2->getValue<bool>("IsLocalActive"));
		EXPECT_FALSE(newRepresentation2->getValue<bool>("IsGravityEnabled"));
		EXPECT_TRUE(newRepresentation2->getValue<bool>("IsDrivingSceneElementPose"));

		EXPECT_TRUE(newRepresentation3->getValue<bool>("IsLocalActive"));
		EXPECT_TRUE(newRepresentation3->getValue<bool>("IsGravityEnabled"));
		EXPECT_FALSE(newRepresentation3->getValue<bool>("IsDrivingSceneElementPose"));

		EXPECT_FALSE(newRepresentation4->getValue<bool>("IsLocalActive"));
		EXPECT_FALSE(newRepresentation4->getValue<bool>("IsGravityEnabled"));
		EXPECT_FALSE(newRepresentation4->getValue<bool>("IsDrivingSceneElementPose"));
	}
}

// Local class to test constraint registration.
class MockRigidRepresentation : public SurgSim::Physics::RigidRepresentation
{
public:
	MockRigidRepresentation(const std::string& name) : RigidRepresentation(name)
	{}

	std::shared_ptr<SurgSim::Physics::ConstraintImplementation>
		getConstraintImplementation(SurgSim::Math::MlcpConstraintType type)
	{
		return RigidRepresentation::getConstraintImplementation(type);
	}
};

TEST(RepresentationTest, ConstraintTest)
{
	using SurgSim::Physics::ConstraintImplementation;
	using SurgSim::Physics::MockConstraintImplementation;

	auto implementation = std::make_shared<MockConstraintImplementation>();
	auto representation = std::make_shared<MockRigidRepresentation>("rep");

	// Test the getConstraintImplementation
	EXPECT_TRUE(representation->getConstraintImplementation(implementation->getMlcpConstraintType())
		== nullptr);

	// Add the constraint implementation.
	ConstraintImplementation::getFactory().addImplementation(typeid(MockRigidRepresentation),
		implementation);

	// Test the getConstraintImplementation
	EXPECT_TRUE(representation->getConstraintImplementation(implementation->getMlcpConstraintType())
		!= nullptr);
}

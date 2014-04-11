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

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Collision::Contact;
using SurgSim::Collision::Location;
using SurgSim::Collision::ShapeCollisionRepresentation;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Collision
{

struct RepresentationTest : public ::testing::Test
{
	virtual void SetUp()
	{
		element = std::make_shared<BasicSceneElement>("Element");
		plane = std::make_shared<PlaneShape>();
		sphere = std::make_shared<SphereShape>(1.0);
		planeRep = std::make_shared<ShapeCollisionRepresentation>("PlaneShape", plane, RigidTransform3d::Identity());
		sphereRep = std::make_shared<ShapeCollisionRepresentation>("SphereShape", sphere, RigidTransform3d::Identity());

		element->addComponent(planeRep);
		element->addComponent(sphereRep);
	}

	virtual void TearDown()
	{
	}

	std::shared_ptr<BasicSceneElement> element;
	std::shared_ptr<PlaneShape> plane;
	std::shared_ptr<SphereShape> sphere;
	std::shared_ptr<Representation> planeRep;
	std::shared_ptr<Representation> sphereRep;
};

TEST_F(RepresentationTest, InitTest)
{
	EXPECT_NO_THROW(
		{ShapeCollisionRepresentation("Plane", plane, RigidTransform3d::Identity());}
	);
}

TEST_F(RepresentationTest, PoseTest)
{
	RigidTransform3d initialPose = makeRigidTransform(Quaterniond::Identity(), Vector3d(1.0, 2.0, 3.0));
	planeRep->setLocalPose(initialPose);
	EXPECT_TRUE(initialPose.isApprox(planeRep->getPose(), epsilon));

	RigidTransform3d pose = makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 2.0, 0.0));
	element->setPose(pose);
	EXPECT_TRUE(pose.isApprox(sphereRep->getPose(), epsilon));

	sphereRep->setLocalPose(initialPose);
	EXPECT_TRUE((pose * initialPose).isApprox(sphereRep->getPose(), epsilon));
}

TEST_F(RepresentationTest, ShapeTest)
{
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_PLANE, planeRep->getShapeType());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SPHERE, sphereRep->getShapeType());

	EXPECT_EQ(plane, planeRep->getShape());
	EXPECT_EQ(sphere, sphereRep->getShape());
}

TEST_F(RepresentationTest, EmptyCollisionTest)
{
	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::list<std::shared_ptr<SurgSim::Collision::Contact>>> planeCollisions = planeRep->getCollisions();
	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::list<std::shared_ptr<SurgSim::Collision::Contact>>>  sphereCollisions = sphereRep->getCollisions();
	EXPECT_EQ(0u, planeCollisions.size());
	EXPECT_EQ(0u, sphereCollisions.size());

	EXPECT_FALSE(planeRep->isCollidingWith(sphereRep));
	EXPECT_FALSE(sphereRep->isCollidingWith(planeRep));

	std::list<std::shared_ptr<SurgSim::Collision::Contact>> sphereCollisionContacts =
		sphereRep->getCollisionsWith(planeRep);
	std::list<std::shared_ptr<SurgSim::Collision::Contact>> planeCollisionContacts =
		planeRep->getCollisionsWith(sphereRep);
	EXPECT_EQ(0u, sphereCollisionContacts.size());
	EXPECT_EQ(0u, planeCollisionContacts.size());
}

TEST_F(RepresentationTest, CollisionTest)
{
	EXPECT_FALSE(sphereRep->hasCollision());
	EXPECT_FALSE(planeRep->hasCollision());

	std::shared_ptr<Contact> dummyContact =
		std::make_shared<Contact>(0.0, Vector3d::Zero(), Vector3d::Zero(), std::make_pair(Location(), Location()));
	EXPECT_NO_THROW(sphereRep->addCollisionWith(planeRep, dummyContact));

	EXPECT_TRUE(sphereRep->hasCollision());
	EXPECT_TRUE(sphereRep->isCollidingWith(planeRep));
	// Collision is only added to 'sphereRep', thus the following check should return 'false'.
	EXPECT_FALSE(planeRep->isCollidingWith(sphereRep));

	EXPECT_NO_THROW(planeRep->addCollisionWith(sphereRep, dummyContact));
	EXPECT_TRUE(planeRep->hasCollision());
	EXPECT_TRUE(planeRep->isCollidingWith(sphereRep));

	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::list<std::shared_ptr<SurgSim::Collision::Contact>>>  sphereCollisions = sphereRep->getCollisions();
	EXPECT_EQ(1u, sphereCollisions.size());
	EXPECT_NE(std::end(sphereCollisions), sphereCollisions.find(planeRep));

	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::list<std::shared_ptr<SurgSim::Collision::Contact>>> planeCollisions = planeRep->getCollisions();
	EXPECT_EQ(1u, planeCollisions.size());
	EXPECT_NE(std::end(planeCollisions), planeCollisions.find(sphereRep));

	std::list<std::shared_ptr<SurgSim::Collision::Contact>> sphereCollisionContacts =
		sphereRep->getCollisionsWith(planeRep);
	EXPECT_EQ(1u, sphereCollisionContacts.size());
	EXPECT_EQ(dummyContact, sphereCollisionContacts.front());

	std::list<std::shared_ptr<SurgSim::Collision::Contact>> planeCollisionContacts =
		planeRep->getCollisionsWith(sphereRep);
	EXPECT_EQ(1u, planeCollisionContacts.size());
	EXPECT_EQ(dummyContact, planeCollisionContacts.front());

	EXPECT_EQ(planeCollisionContacts.front(), sphereCollisionContacts.front());

	sphereRep->clearCollisions();
	EXPECT_FALSE(sphereRep->hasCollision());
}


}; // namespace Collision
}; // namespace SurgSim

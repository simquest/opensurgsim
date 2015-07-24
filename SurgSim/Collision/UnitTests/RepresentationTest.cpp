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
#include "SurgSim/DataStructures/BufferedValue.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/ThreadPool.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Collision::Contact;
using SurgSim::Collision::ContactMapType;
using SurgSim::Collision::ShapeCollisionRepresentation;
using SurgSim::DataStructures::Location;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

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
		runtime = std::make_shared<Framework::Runtime>();
		scene = runtime->getScene();
		element = std::make_shared<BasicSceneElement>("Element");
		plane = std::make_shared<PlaneShape>();
		sphere = std::make_shared<SphereShape>(1.0);
		planeRep = std::make_shared<ShapeCollisionRepresentation>("PlaneShape");
		sphereRep = std::make_shared<ShapeCollisionRepresentation>("SphereShape");

		planeRep->setShape(plane);
		planeRep->setLocalPose(RigidTransform3d::Identity());

		sphereRep->setShape(sphere);
		sphereRep->setLocalPose(RigidTransform3d::Identity());

		element->addComponent(planeRep);
		element->addComponent(sphereRep);
		scene->addSceneElement(element);
		planeRep->wakeUp();
		sphereRep->wakeUp();
	}

	virtual void TearDown()
	{
	}

	std::shared_ptr<Framework::Runtime> runtime;
	std::shared_ptr<Framework::Scene> scene;

	std::shared_ptr<BasicSceneElement> element;
	std::shared_ptr<PlaneShape> plane;
	std::shared_ptr<SphereShape> sphere;
	std::shared_ptr<ShapeCollisionRepresentation> planeRep;
	std::shared_ptr<ShapeCollisionRepresentation> sphereRep;
};

TEST_F(RepresentationTest, InitTest)
{
	EXPECT_NO_THROW(ShapeCollisionRepresentation("Plane"));
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
	EXPECT_TRUE(planeRep->getCollisions().unsafeGet().empty());
	EXPECT_TRUE(sphereRep->getCollisions().unsafeGet().empty());
}

TEST_F(RepresentationTest, CollisionTest)
{
	ContactMapType& unsafePlaneCollisions = planeRep->getCollisions().unsafeGet();
	ContactMapType& unsafeSphereCollisions = sphereRep->getCollisions().unsafeGet();
	std::shared_ptr<const ContactMapType> safePlaneCollisions = planeRep->getCollisions().safeGet();

	EXPECT_TRUE(unsafePlaneCollisions.empty());
	EXPECT_TRUE(unsafeSphereCollisions.empty());
	EXPECT_TRUE(safePlaneCollisions->empty());

	std::shared_ptr<Contact> dummyContact =
		std::make_shared<Contact>(0.0, Vector3d::Zero(), Vector3d::Zero(), std::make_pair(Location(), Location()));
	unsafeSphereCollisions[planeRep].push_back(dummyContact);

	auto spherePlanePair = unsafeSphereCollisions.find(planeRep);
	EXPECT_NE(unsafeSphereCollisions.end(), spherePlanePair);
	std::list<std::shared_ptr<SurgSim::Collision::Contact>> spherePlaneContacts = spherePlanePair->second;
	EXPECT_EQ(dummyContact, spherePlaneContacts.front());

	// Collision is only added to 'sphereRep', thus the plane should have no collisions.
	EXPECT_TRUE(unsafePlaneCollisions.empty());

	// The thread-safe collision map is buffered, so it should still be empty before the publish.
	EXPECT_TRUE(planeRep->getCollisions().safeGet()->empty());

	// After the publish the thread-safe collision map should be up-to-date.
	planeRep->getCollisions().publish();
	EXPECT_EQ(unsafePlaneCollisions, *planeRep->getCollisions().safeGet());
}

// addContact method thread-safety test case.
// WARNING: Due to the nature of multi-threaded environment, a successful test does not imply thread-safety
//          also note the lack of reproducibility.
TEST_F(RepresentationTest, AddContactsInParallelTest)
{
	auto rep = std::make_shared<ShapeCollisionRepresentation>("collisionRepReference");
	auto contact = std::make_shared<Contact>(0.1, Math::Vector3d::Zero(), Math::Vector3d::Zero(),
		std::make_pair(DataStructures::Location(), DataStructures::Location()));
	auto threadPool = Framework::Runtime::getThreadPool();
	std::vector<std::future<void>> tasks;
	const size_t numContacts = 500;

	for (size_t i = 0; i < numContacts; i++)
	{
		tasks.push_back(threadPool->enqueue<void>([&rep, &contact](){ rep->addContact(rep, contact); }));
	}

	std::for_each(tasks.begin(), tasks.end(), [](std::future<void>& p){p.wait();});
	ASSERT_EQ(numContacts, rep->getCollisions().unsafeGet()[rep].size());
}

}; // namespace Collision
}; // namespace SurgSim

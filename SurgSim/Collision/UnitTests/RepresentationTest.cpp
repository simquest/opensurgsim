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

#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Collision/SpherePlaneDcdContact.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Collision::ShapeCollisionRepresentation;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::Shape;
using SurgSim::Math::SphereShape;

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
		plane = std::make_shared<PlaneShape>();
		sphere = std::make_shared<SphereShape>(1.0);
		planeRep = std::make_shared<ShapeCollisionRepresentation>("Plane Shape", plane,
														makeRigidTransform(Quaterniond::Identity(), Vector3d::Zero()));
		sphereRep = std::make_shared<ShapeCollisionRepresentation>("Sphere Shape", sphere,
														makeRigidTransform(Quaterniond::Identity(), Vector3d::Zero()));

		collisionPair = std::make_shared<CollisionPair>(sphereRep, planeRep);
		calContact = std::make_shared<SpherePlaneDcdContact>();
	}

	virtual void TearDown()
	{
	}

	std::shared_ptr<PlaneShape> plane;
	std::shared_ptr<SphereShape> sphere;
	std::shared_ptr<Representation> planeRep;
	std::shared_ptr<Representation> sphereRep;
	std::shared_ptr<CollisionPair> collisionPair;
	std::shared_ptr<SpherePlaneDcdContact> calContact;
};

TEST_F(RepresentationTest, InitTest)
{
	EXPECT_NO_THROW(
		{ShapeCollisionRepresentation("Temp", plane, makeRigidTransform(Quaterniond::Identity(), Vector3d::Zero()));}
	);
}

TEST_F(RepresentationTest, PoseTest)
{
	RigidTransform3d initialPose = makeRigidTransform(Quaterniond::Identity(), Vector3d(1.0, 2.0, 3.0));
	planeRep->setInitialPose(initialPose);
	EXPECT_TRUE(initialPose.isApprox(planeRep->getPose(), epsilon));

	RigidTransform3d pose =	makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 2.0, 0.0));
	sphereRep->setPose(pose);
	EXPECT_TRUE(pose.isApprox(sphereRep->getPose(), epsilon));
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
	const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& planeColliders = planeRep->getColliders();
	const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& sphereColliders = sphereRep->getColliders();
	EXPECT_EQ(0u, planeColliders.size());
	EXPECT_EQ(0u, sphereColliders.size());

	const std::deque<std::shared_ptr<SurgSim::Collision::Contact>>& sphereContacts = sphereRep->getContacts(planeRep);
	const std::deque<std::shared_ptr<SurgSim::Collision::Contact>>& planeContacts = planeRep->getContacts(sphereRep);
	EXPECT_EQ(0u, sphereContacts.size());
	EXPECT_EQ(0u, planeContacts.size());
}

TEST_F(RepresentationTest, NoCollisionTest)
{
	sphereRep->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 2.0, 0.0)));
	calContact->calculateContact(collisionPair);

	const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& planeColliders = planeRep->getColliders();
	const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& sphereColliders = sphereRep->getColliders();

	EXPECT_EQ(0u, planeColliders.size());
	EXPECT_EQ(0u, sphereColliders.size());
}


TEST_F(RepresentationTest, CollisionTest)
{
	planeRep->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.5, 0.0)));
	sphereRep->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 1.0, 0.0)));

	calContact->calculateContact(collisionPair);

	const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& planeColliders = planeRep->getColliders();
	const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& sphereColliders = sphereRep->getColliders();
	EXPECT_EQ(1u, planeColliders.size());
	EXPECT_EQ(1u, sphereColliders.size());

	std::shared_ptr<Representation> collisionRep1 = planeColliders.front();
	std::shared_ptr<Representation> collisionRep2 = sphereColliders.front();
	EXPECT_EQ(planeRep, collisionRep2);
	EXPECT_EQ(sphereRep, collisionRep1);

	const std::deque<std::shared_ptr<SurgSim::Collision::Contact>>& planeContacts = planeRep->getContacts(sphereRep);
	const std::deque<std::shared_ptr<SurgSim::Collision::Contact>>& sphereContacts = sphereRep->getContacts(planeRep);
	EXPECT_EQ(1u, planeContacts.size());
	EXPECT_EQ(1u, sphereContacts.size());

	// The contact point in each representation should be the same.
	EXPECT_EQ(planeContacts.front(), sphereContacts.front());
}

}; // namespace Collision
}; // namespace SurgSim

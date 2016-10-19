// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/UnitTests/RepresentationUtilities.h"
#include "SurgSim/DataStructures/BufferedValue.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/RigidState.h"

using SurgSim::Collision::ContactMapType;
using SurgSim::DataStructures::Location;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

TEST(CollisionPairTests, InitTest)
{
	// Default Constructor, needs to work for ReuseFrepresentationy
	EXPECT_NO_THROW({CollisionPair pair;});

	std::shared_ptr<Representation> rep0 = makeSphereRepresentation(1.0);
	std::shared_ptr<Representation> rep1 = makeSphereRepresentation(2.0);

	EXPECT_NO_THROW({CollisionPair pair(rep0, rep0);});
	EXPECT_ANY_THROW({CollisionPair pair(nullptr, rep0);});
	EXPECT_ANY_THROW({CollisionPair pair(nullptr, nullptr);});
	EXPECT_ANY_THROW({CollisionPair pair(rep0, nullptr);});

	ASSERT_NO_THROW({CollisionPair pair(rep0, rep1);});
	CollisionPair pair(rep0, rep1);

	EXPECT_EQ(rep0, pair.getFirst());
	EXPECT_EQ(rep1, pair.getSecond());
	EXPECT_FALSE(pair.hasContacts());
	EXPECT_FALSE(pair.isSwapped());

	std::pair<Location, Location> penetrationPoints;
	penetrationPoints.first.rigidLocalPosition.setValue(Vector3d(0.1, 0.2, 0.3));
	penetrationPoints.second.rigidLocalPosition.setValue(Vector3d(0.4, 0.5, 0.6));
	pair.addDcdContact(1.0, Vector3d(1.0, 0.0, 0.0), penetrationPoints);
	EXPECT_TRUE(pair.hasContacts());
}

TEST(CollisionPairTests, SwapTest)
{
	std::shared_ptr<Representation> rep0 = makeSphereRepresentation(1.0);
	std::shared_ptr<Representation> rep1 = makeSphereRepresentation(2.0);

	CollisionPair pair(rep0, rep1);
	EXPECT_FALSE(pair.isSwapped());
	EXPECT_EQ(rep0.get(), pair.getRepresentations().first.get());
	EXPECT_EQ(rep1.get(), pair.getRepresentations().second.get());
	pair.swapRepresentations();
	EXPECT_TRUE(pair.isSwapped());
	pair.swapRepresentations();
	EXPECT_FALSE(pair.isSwapped());

	std::pair<Location, Location> penetrationPoints;
	penetrationPoints.first.rigidLocalPosition.setValue(Vector3d(0.1, 0.2, 0.3));
	penetrationPoints.second.rigidLocalPosition.setValue(Vector3d(0.4, 0.5, 0.6));

	pair.addDcdContact(1.0, Vector3d(1.0, 0.0, 0.0), penetrationPoints);
	EXPECT_TRUE(pair.hasContacts());

	EXPECT_ANY_THROW(pair.swapRepresentations());
}

TEST(CollisionPairTests, setRepresentationsTest)
{
	std::shared_ptr<Representation> rep0 = makeSphereRepresentation(1.0);
	std::shared_ptr<Representation> rep1 = makeSphereRepresentation(2.0);
	std::shared_ptr<Representation> repA = makeSphereRepresentation(99.0);
	std::shared_ptr<Representation> repB = makeSphereRepresentation(100.0);

	CollisionPair pair(repA, repB);
	EXPECT_FALSE(pair.isSwapped());
	pair.swapRepresentations();

	pair.setRepresentations(rep0, rep1);

	EXPECT_EQ(rep0.get(), pair.getRepresentations().first.get());
	EXPECT_EQ(rep1.get(), pair.getRepresentations().second.get());
	EXPECT_FALSE(pair.isSwapped());
}

TEST(CollisionPairTests, CollisionDetectionTypeTest)
{
	std::shared_ptr<Representation> repCCD1 = makeSphereRepresentation(1.0);
	repCCD1->setCollisionDetectionType(COLLISION_DETECTION_TYPE_CONTINUOUS);
	std::shared_ptr<Representation> repCCD2 = makeSphereRepresentation(2.0);
	repCCD2->setCollisionDetectionType(COLLISION_DETECTION_TYPE_CONTINUOUS);
	std::shared_ptr<Representation> repDCD1 = makeSphereRepresentation(99.0);
	repDCD1->setCollisionDetectionType(COLLISION_DETECTION_TYPE_DISCRETE);
	std::shared_ptr<Representation> repDCD2 = makeSphereRepresentation(100.0);
	repDCD2->setCollisionDetectionType(COLLISION_DETECTION_TYPE_DISCRETE);
	std::shared_ptr<Representation> repNone = makeSphereRepresentation(300.0);
	repNone->setCollisionDetectionType(COLLISION_DETECTION_TYPE_NONE);

	{
		CollisionPair pair(repCCD1, repCCD2);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_CONTINUOUS, pair.getType());
	}
	{
		CollisionPair pair(repDCD1, repDCD2);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_DISCRETE, pair.getType());
	}
	{
		CollisionPair pair(repCCD1, repDCD1);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_DISCRETE, pair.getType());
	}
	{
		CollisionPair pair(repDCD2, repCCD2);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_DISCRETE, pair.getType());
	}
	{
		CollisionPair pair(repNone, repDCD1);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_NONE, pair.getType());
	}
	{
		CollisionPair pair(repCCD1, repNone);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_NONE, pair.getType());
	}
}

TEST(CollisionPairTests, SelfCollisionDetectionTypeTest)
{
	std::shared_ptr<Representation> repCCD = makeSphereRepresentation(1.0);
	repCCD->setSelfCollisionDetectionType(COLLISION_DETECTION_TYPE_CONTINUOUS);
	std::shared_ptr<Representation> repDCD = makeSphereRepresentation(99.0);
	repDCD->setSelfCollisionDetectionType(COLLISION_DETECTION_TYPE_DISCRETE);
	std::shared_ptr<Representation> repNone = makeSphereRepresentation(300.0);
	repNone->setSelfCollisionDetectionType(COLLISION_DETECTION_TYPE_NONE);

	{
		CollisionPair pair(repCCD, repCCD);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_CONTINUOUS, pair.getType());
	}
	{
		CollisionPair pair(repDCD, repDCD);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_DISCRETE, pair.getType());
	}
	{
		CollisionPair pair(repNone, repNone);
		EXPECT_EQ(COLLISION_DETECTION_TYPE_NONE, pair.getType());
	}
}

TEST(CollisionPairTests, addContactTest)
{
	auto rep0 = makeSphereRepresentation(1.0);
	auto rep1 = makeSphereRepresentation(2.0);
	auto other = makeSphereRepresentation(3.0);

	auto rep0Collisions = rep0->getCollisions().safeGet();
	auto rep1Collisions = rep1->getCollisions().safeGet();

	EXPECT_TRUE(rep0Collisions->empty());
	EXPECT_TRUE(rep1Collisions->empty());

	std::pair<Location, Location> penetrationPoints;
	penetrationPoints.first.rigidLocalPosition.setValue(Vector3d(0.1, 0.2, 0.3));
	penetrationPoints.second.rigidLocalPosition.setValue(Vector3d(0.4, 0.5, 0.6));

	CollisionPair pair(rep0, rep1);
	pair.addDcdContact(1.0, Vector3d::UnitY(), penetrationPoints);
	pair.updateRepresentations();

	rep0->update(0.0);
	rep0->getCollisions().publish();
	rep1->update(0.0);
	rep1->getCollisions().publish();

	rep0Collisions = rep0->getCollisions().safeGet();
	rep1Collisions = rep1->getCollisions().safeGet();

	EXPECT_EQ(1u, rep0Collisions->size());
	auto rep0CollisionContacts = rep0Collisions->find(rep1);
	EXPECT_NE(rep0Collisions->end(), rep0CollisionContacts);
	EXPECT_EQ(rep1, rep0CollisionContacts->first);
	std::shared_ptr<SurgSim::Collision::Contact> rep0FirstContact = rep0CollisionContacts->second.front();
	EXPECT_EQ(rep0FirstContact->depth, 1.0);
	EXPECT_TRUE(rep0FirstContact->normal.isApprox(Vector3d::UnitY()));
	EXPECT_TRUE(rep0FirstContact->penetrationPoints.first.rigidLocalPosition.getValue().isApprox(
					Vector3d(0.1, 0.2, 0.3)));
	EXPECT_TRUE(rep0FirstContact->penetrationPoints.second.rigidLocalPosition.getValue().isApprox(
					Vector3d(0.4, 0.5, 0.6)));
	EXPECT_TRUE(rep0->collidedWith(rep1));
	EXPECT_FALSE(rep0->collidedWith(other));

	EXPECT_EQ(1u, rep1Collisions->size());
	auto rep1CollisionContacts = rep1Collisions->find(rep0);
	EXPECT_NE(rep1Collisions->end(), rep1CollisionContacts);
	EXPECT_EQ(rep0, rep1CollisionContacts->first);
	std::shared_ptr<SurgSim::Collision::Contact> rep1FirstContact = rep1CollisionContacts->second.front();
	EXPECT_EQ(rep1FirstContact->depth, 1.0);
	EXPECT_TRUE(rep1FirstContact->normal.isApprox(-Vector3d::UnitY()));
	EXPECT_TRUE(rep1FirstContact->penetrationPoints.first.rigidLocalPosition.getValue().isApprox(
					Vector3d(0.4, 0.5, 0.6)));
	EXPECT_TRUE(rep1FirstContact->penetrationPoints.second.rigidLocalPosition.getValue().isApprox(
					Vector3d(0.1, 0.2, 0.3)));
	EXPECT_TRUE(rep1->collidedWith(rep0));
	EXPECT_FALSE(rep0->collidedWith(other));

	rep0->retire();
	rep1->retire();
}

}; // namespace Collision
}; // namespace SurgSim

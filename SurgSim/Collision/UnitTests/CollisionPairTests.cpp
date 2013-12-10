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
#include "SurgSim/Collision/UnitTests/RepresentationUtilities.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

#include "SurgSim/Physics/RigidRepresentationState.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/CollisionPair.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

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

	EXPECT_ANY_THROW({CollisionPair pair(rep0, rep0);});
	EXPECT_ANY_THROW({CollisionPair pair(nullptr, rep0);});
	EXPECT_ANY_THROW({CollisionPair pair(nullptr, nullptr);});
	EXPECT_ANY_THROW({CollisionPair pair(rep0, nullptr);});

	ASSERT_NO_THROW({CollisionPair pair(rep0, rep1);});
	CollisionPair pair(rep0,rep1);

	EXPECT_EQ(rep0, pair.getFirst());
	EXPECT_EQ(rep1, pair.getSecond());
	EXPECT_FALSE(pair.hasContacts());
	EXPECT_FALSE(pair.isSwapped());

	std::pair<Location,Location> penetrationPoints;
	pair.addContact(1.0, Vector3d(1.0,0.0,0.0),penetrationPoints);
	EXPECT_TRUE(pair.hasContacts());
}

TEST(CollisionPairTests, SwapTest)
{
	std::shared_ptr<Representation> rep0 = makeSphereRepresentation(1.0);
	std::shared_ptr<Representation> rep1 = makeSphereRepresentation(2.0);

	CollisionPair pair(rep0,rep1);
	EXPECT_FALSE(pair.isSwapped());
	EXPECT_EQ(rep0.get(),pair.getRepresentations().first.get());
	EXPECT_EQ(rep1.get(),pair.getRepresentations().second.get());
	pair.swapRepresentations();
	EXPECT_TRUE(pair.isSwapped());
	pair.swapRepresentations();
	EXPECT_FALSE(pair.isSwapped());

	std::pair<Location,Location> penetrationPoints;

	pair.addContact(1.0, Vector3d(1.0,0.0,0.0),penetrationPoints);
	EXPECT_TRUE(pair.hasContacts());

	EXPECT_ANY_THROW(pair.swapRepresentations());
}

TEST(CollisionPairTests, setRepresentationsTest)
{
	std::shared_ptr<Representation> rep0 = makeSphereRepresentation(1.0);
	std::shared_ptr<Representation> rep1 = makeSphereRepresentation(2.0);
	std::shared_ptr<Representation> repA = makeSphereRepresentation(99.0);
	std::shared_ptr<Representation> repB = makeSphereRepresentation(100.0);

	CollisionPair pair(repA,repB);
	EXPECT_FALSE(pair.isSwapped());
	pair.swapRepresentations();

	pair.setRepresentations(rep0,rep1);

	EXPECT_EQ(rep0.get(), pair.getRepresentations().first.get());
	EXPECT_EQ(rep1.get(), pair.getRepresentations().second.get());
	EXPECT_FALSE(pair.isSwapped());
}

}; // namespace Collision
}; // namespace SurgSim

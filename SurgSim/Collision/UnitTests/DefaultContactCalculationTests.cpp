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

#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/Collision/DefaultContactCalculation.h"
#include "SurgSim/Math/SphereShape.h"

namespace SurgSim
{
namespace Collision
{

TEST(DefaultContactCalculationTests, UnitTests)
{
	std::shared_ptr<SurgSim::Math::Shape> sphereShape = std::make_shared<SurgSim::Math::SphereShape>(1.0);

	std::shared_ptr<ShapeCollisionRepresentation> rep0 = std::make_shared<ShapeCollisionRepresentation>("TestSphere 1");
	rep0->setShape(sphereShape);
	rep0->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(3.0,0.0,0.0)));

	std::shared_ptr<ShapeCollisionRepresentation> rep1 = std::make_shared<ShapeCollisionRepresentation>("TestSphere 2");
	rep1->setShape(sphereShape);
	rep1->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(0.5,0.0,0.0)));

	std::shared_ptr<CollisionPair> pair01 = std::make_shared<CollisionPair>(rep0, rep1);

	DefaultContactCalculation calcShouldLog(false);
	EXPECT_NO_THROW(calcShouldLog.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());

	DefaultContactCalculation calcShouldThrow(true);
	EXPECT_ANY_THROW(calcShouldThrow.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());
}

}; // namespace Collision
}; // namespace SurgSim

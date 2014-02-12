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


#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "SurgSim/Collision//Representation.h"
#include "SurgSim/Math/Shape.h"

using ::testing::AtLeast;

class MockCollisionRepresentation: public SurgSim::Collision::Representation
{
public:
	MOCK_METHOD0(setInitialPose, void());
	MOCK_METHOD0(getInitialPose, const SurgSim::Math::RigidTransform3d& ());
	MOCK_METHOD0(setPose, void());
	MOCK_METHOD0(getPose, const SurgSim::Math::RigidTransform3d& ());
	
	MOCK_METHOD0(getShapeType, int());
	MOCK_METHOD0(getShape, const std::shared_ptr<SurgSim::Math::Shape>());
	MOCK_METHOD0(getPhysicsRepresentation, std::shared_ptr<SurgSim::Physics::Representation>());
};


TEST(CollisionRepresentationTest, Calls) {
	//MockCollisionRepresentation mockCollisionRepresentaion;
	//EXPECT_CALL(mockCollisionRepresentaion, getShapeType())
	//	.Times(AtLeast(1));

	//Painter painter(&turtle);                   // #4

	//EXPECT_TRUE(painter.DrawCircle(0, 0, 10));
}    

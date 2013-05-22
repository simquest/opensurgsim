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
#include <memory>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/Actors/FixedActor.h>
#include <SurgSim/Physics/Actors/RigidActor.h>
#include <SurgSim/Physics/Actors/RigidActorParameters.h>
#include <SurgSim/Physics/Actors/SphereShape.h>
#include <SurgSim/Physics/FreeMotion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Physics::Actor;
using SurgSim::Physics::RigidActor;
using SurgSim::Physics::RigidActorParameters;
using SurgSim::Physics::SphereShape;
using SurgSim::Physics::FreeMotion;



struct FreeMotionTest: public ::testing::Test
{
	virtual void SetUp()
	{
	}

	virtual void TearDown()
	{
	}
};

TEST(FreeMotionTest, RunTest)
{
	std::shared_ptr<std::vector<std::shared_ptr<Actor>>> actors =
		std::make_shared<std::vector<std::shared_ptr<Actor>>>();
	std::shared_ptr<RigidActor> actor = std::make_shared<RigidActor>("TestSphere");

	RigidActorParameters params;
	params.setDensity(700.0); // Wood

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.01); // 1cm Sphere
	params.setShapeUsedForMassInertia(shape);

	actor->setInitialParameters(params);
	actor->setInitialPose(SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond(), Vector3d(0.0,0.0,0.0)));

	actors->push_back(actor);

	FreeMotion computation(actors);

	EXPECT_TRUE(Vector3d(0.0,0.0,0.0).isApprox(actor->getPose().translation()));
	computation.update(1.0);
	EXPECT_FALSE(Vector3d(0.0,0.0,0.0).isApprox(actor->getPose().translation()));

}


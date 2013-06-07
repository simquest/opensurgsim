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

/// \file Simple Test for FreeMotion calculation

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/FixedRepresentation.h>
#include <SurgSim/Physics/RigidRepresentation.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>
#include <SurgSim/Physics/SphereShape.h>
#include <SurgSim/Physics/FreeMotion.h>
#include <SurgSim/Physics/PhysicsManagerState.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::SphereShape;
using SurgSim::Physics::FreeMotion;
using SurgSim::Physics::PhysicsManagerState;

TEST(FreeMotionTest, RunTest)
{
	std::vector<std::shared_ptr<Representation>> representations = std::vector<std::shared_ptr<Representation>>();
	std::shared_ptr<RigidRepresentation> representation = std::make_shared<RigidRepresentation>("TestSphere");

	RigidRepresentationParameters params;
	params.setDensity(700.0); // Wood

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.01); // 1cm Sphere
	params.setShapeUsedForMassInertia(shape);

	representation->setInitialParameters(params);
	representation->setInitialPose(SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond(), Vector3d(0.0,0.0,0.0)));

	representations.push_back(representation);

	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();
	state->setRepresentations(representations);

	FreeMotion computation;

	representation->setIsGravityEnabled(false);
	EXPECT_TRUE(representation->getPose().translation().isZero());
	state = computation.update(1.0,state);
	EXPECT_TRUE(representation->getPose().translation().isZero());
	
	representation->setIsGravityEnabled(true);
	EXPECT_TRUE(representation->getPose().translation().isZero());
	state = computation.update(1.0,state);
	EXPECT_FALSE(representation->getPose().translation().isZero());
}


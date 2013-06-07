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

/// \file Tests for the DcdCollision Class

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/DcdCollision.h>
#include <SurgSim/Physics/RigidRepresentationCollisionRepresentation.h>
#include <SurgSim/Physics/SphereShape.h>
#include <SurgSim/Physics/PhysicsManagerState.h>

#include <SurgSim/Math/Quaternion.h>

using SurgSim::Physics::CollisionRepresentation;
using SurgSim::Physics::CollisionPair;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::RigidRepresentationCollisionRepresentation;
using SurgSim::Physics::SphereShape;
using SurgSim::Physics::PhysicsManagerState;

using SurgSim::Math::Vector3d;


std::shared_ptr<Representation> createSphere(const std::string& name, const SurgSim::Math::Vector3d& position)
{
	std::shared_ptr<RigidRepresentation> representation = std::make_shared<RigidRepresentation>(name);

	RigidRepresentationParameters params;
	params.setDensity(700.0); // Wood

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.01); // 1cm Sphere
	params.setShapeUsedForMassInertia(shape);

	representation->setInitialParameters(params);
	representation->setInitialPose(SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond(), position));

	return representation;
}


TEST(DcdCollisionTest, SingleCollisionTest)
{
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();
	std::shared_ptr<Representation> sphere1 = createSphere("Sphere1", Vector3d(0.0,0.0,0.0));
	std::shared_ptr<Representation> sphere2 = createSphere("Sphere2", Vector3d(0.0,0.0,0.5));

	std::vector<std::shared_ptr<Representation>> representations;

	representations.push_back(sphere1);
	representations.push_back(sphere2);
	state->setRepresentations(representations);

	SurgSim::Physics::DcdCollision computation;
	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);

	EXPECT_EQ(1u, newState->getCollisionPairs().size());
}

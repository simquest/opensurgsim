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

#include <SurgSim/Collision/CollisionPair.h>
#include <SurgSim/Physics/DcdCollision.h>
#include <SurgSim/Collision/RigidCollisionRepresentation.h>
#include <SurgSim/Physics/FixedRepresentation.h>
#include <SurgSim/Physics/PhysicsManagerState.h>

#include <SurgSim/Math/DoubleSidedPlaneShape.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/SphereShape.h>

using SurgSim::Collision::CollisionPair;
using SurgSim::Collision::RigidCollisionRepresentation;
using SurgSim::Math::DoubleSidedPlaneShape;
using SurgSim::Math::Shape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::PhysicsManagerState;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;

std::shared_ptr<RigidRepresentation> createSphere(const std::string& name, const SurgSim::Math::Vector3d& position)
{
	std::shared_ptr<RigidRepresentation> representation = std::make_shared<RigidRepresentation>(name);

	RigidRepresentationParameters params;
	params.setDensity(700.0); // Wood

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(1.0); // 1cm Sphere
	params.setShapeUsedForMassInertia(shape);

	representation->setInitialParameters(params);
	representation->setInitialPose(SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), position));

	return representation;
}


TEST(DcdCollisionTest, RigidRigidCollisionTest)
{
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();

	std::shared_ptr<RigidRepresentation> sphere1 = createSphere("Sphere1", Vector3d(0.0,0.0,0.0));
	std::shared_ptr<RigidRepresentation> sphere2 = createSphere("Sphere2", Vector3d(0.0,0.0,0.5));

	std::shared_ptr<SurgSim::Collision::Representation> sphere1Collision =
		std::make_shared<RigidCollisionRepresentation>("Sphere1 Collision", sphere1);
	std::shared_ptr<SurgSim::Collision::Representation> sphere2Collision =
		std::make_shared<RigidCollisionRepresentation>("Sphere2 Collision", sphere2);

	std::vector<std::shared_ptr<Representation>> representations;
	representations.push_back(sphere1);
	representations.push_back(sphere2);
	state->setRepresentations(representations);

	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> collisions;
	collisions.push_back(sphere1Collision);
	collisions.push_back(sphere2Collision);
	state->setCollisionRepresentations(collisions);

	SurgSim::Physics::DcdCollision computation;
	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);

	ASSERT_EQ(1u, newState->getCollisionPairs().size());
	EXPECT_TRUE(newState->getCollisionPairs().at(0)->hasContacts());
}

TEST(DcdCollisionTest, FixedRigidCollisionTest)
{
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();

	std::shared_ptr<RigidRepresentation> sphere1 = createSphere("Sphere1", Vector3d(0.0,0.0,0.0));

	std::shared_ptr<SurgSim::Collision::Representation> sphere1Collision =
		std::make_shared<RigidCollisionRepresentation>("Sphere Collision", sphere1);

	RigidRepresentationParameters params;
	std::shared_ptr<Shape> shape = std::make_shared<DoubleSidedPlaneShape>();
	params.setShapeUsedForMassInertia(shape);
	std::shared_ptr<FixedRepresentation> fixed = std::make_shared<FixedRepresentation>("Fixed");
	fixed->setInitialParameters(params);
	std::shared_ptr<SurgSim::Collision::Representation> fixedCollision =
		std::make_shared<RigidCollisionRepresentation>("Plane Collision", fixed);

	std::vector<std::shared_ptr<Representation>> representations;
	representations.push_back(sphere1);
	representations.push_back(fixed);
	state->setRepresentations(representations);

	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> collisions;
	collisions.push_back(sphere1Collision);
	collisions.push_back(fixedCollision);
	state->setCollisionRepresentations(collisions);

	SurgSim::Physics::DcdCollision computation;
	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);

	ASSERT_EQ(1u, newState->getCollisionPairs().size());
	EXPECT_TRUE(newState->getCollisionPairs().at(0)->hasContacts());
}

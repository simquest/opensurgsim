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
#include <memory>

#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"
#include "SurgSim/Particles/Representation.h"
#include "SurgSim/Physics/ParticleCollisionResponse.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/Representation.h"


namespace SurgSim
{
namespace Physics
{

class MockParticleSystem : public SurgSim::Particles::Representation
{
public:
	explicit MockParticleSystem(const std::string& name) :
		SurgSim::Particles::Representation(name),
		handleCollisionsCount(0)
	{
	}

	int handleCollisionsCount;

private:
	bool doUpdate(double dt) override
	{
		return true;
	}

	bool doHandleCollisions(double dt, const SurgSim::Collision::ContactMapType& collisions) override
	{
		handleCollisionsCount++;
		return true;
	}
};


TEST(ParticleCollisionResponseTest, UpdateCallTest)
{
	auto mockRepresentation = std::make_shared<MockParticleSystem>("particles");
	auto physicsManagerState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<Particles::Representation>> allRepresentations;
	allRepresentations.push_back(mockRepresentation);
	physicsManagerState->setParticleRepresentations(allRepresentations);

	double dt = 1e-3;
	auto computation = std::make_shared<ParticleCollisionResponse>();
	EXPECT_EQ(0, mockRepresentation->handleCollisionsCount);
	computation->update(dt, physicsManagerState);
	EXPECT_EQ(0, mockRepresentation->handleCollisionsCount)
		<< "Particle Representation without a Collision Representation should not handle collisions.";

	auto collisionRepresentation = std::make_shared<Particles::ParticlesCollisionRepresentation>("collision");
	mockRepresentation->setCollisionRepresentation(collisionRepresentation);
	computation->update(dt, physicsManagerState);
	EXPECT_EQ(1, mockRepresentation->handleCollisionsCount);
	computation->update(dt, physicsManagerState);
	EXPECT_EQ(2, mockRepresentation->handleCollisionsCount);
	computation->update(dt, physicsManagerState);
	EXPECT_EQ(3, mockRepresentation->handleCollisionsCount);
}

};
};


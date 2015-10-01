// Copyright 2013-2015, SimQuest Solutions Inc.
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

/// \file PublishCollisionsTests.cpp
/// Simple Test for PublishCollisions computation

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/PublishCollisions.h"
#include "SurgSim/Physics/RigidRepresentation.h"

namespace SurgSim
{
namespace Physics
{

class PublishCollisionsTests : public ::testing::Test
{
public:
	void SetUp()
	{
		// Create the computation and the state
		m_computation = std::make_shared<PublishCollisions>();
		m_state = std::make_shared<PhysicsManagerState>();
	}

	void setCollisionRepAndCollision(size_t numRepresentation, size_t numCollisions)
	{
		std::vector<std::shared_ptr<SurgSim::Collision::Representation>> activeReps;
		for (size_t i = 0; i < numRepresentation; i++)
		{
			auto rigid = std::make_shared<SurgSim::Collision::ShapeCollisionRepresentation>(
				"Rigid" + std::to_string(i));
			for (size_t j = 0; j < numCollisions; j++)
			{
				std::pair<SurgSim::DataStructures::Location, SurgSim::DataStructures::Location> newPenetrationPoints;
				auto collision = std::make_shared<SurgSim::Collision::Contact>(
					SurgSim::Collision::COLLISION_TYPE_DISCRETE,
					1.0, 1.0, SurgSim::Math::Vector3d::Ones(), SurgSim::Math::Vector3d::UnitX(),
					newPenetrationPoints);
				rigid->addContact(rigid, collision);
			}
			activeReps.push_back(rigid);
		}
		m_state->setCollisionRepresentations(activeReps);
		m_state->setActiveCollisionRepresentations(activeReps);
	}

	size_t getNumUnpublishedCollisions() const
	{
		size_t numCollisions = 0;

		for (const auto& rep : m_state->getActiveCollisionRepresentations())
		{
			const SurgSim::Collision::ContactMapType& contactMap = rep->getCollisions().unsafeGet();
			for (auto const& contactAgainstOneRep : contactMap)
			{
				numCollisions += contactAgainstOneRep.second.size();
			}
		}

		return numCollisions;
	}

	size_t getNumPublishedCollisions() const
	{
		size_t numCollisions = 0;

		for (const auto& rep : m_state->getActiveCollisionRepresentations())
		{
			const SurgSim::Collision::ContactMapType& contactMap = *rep->getCollisions().safeGet();
			for (auto const& contactAgainstOneRep : contactMap)
			{
				numCollisions += contactAgainstOneRep.second.size();
			}
		}

		return numCollisions;
	}

protected:
	/// The computation
	std::shared_ptr<PublishCollisions> m_computation;

	/// The physics manager
	std::shared_ptr<PhysicsManagerState> m_state;
};

TEST_F(PublishCollisionsTests, ZeroRepZeroCollisionEach)
{
	setCollisionRepAndCollision(0, 0);

	// Run the computation...
	ASSERT_EQ(0, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
}

TEST_F(PublishCollisionsTests, OneRepZeroCollisionEach)
{
	setCollisionRepAndCollision(1, 0);

	// Run the computation...
	ASSERT_EQ(0, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
}

TEST_F(PublishCollisionsTests, OneRepOneCollisionEach)
{
	setCollisionRepAndCollision(1, 1);

	// Run the computation...
	ASSERT_EQ(1, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(1, getNumUnpublishedCollisions());
	ASSERT_EQ(1, getNumPublishedCollisions());
}

TEST_F(PublishCollisionsTests, OneRepTwoCollisionsEach)
{
	setCollisionRepAndCollision(1, 2);

	// Run the computation...
	ASSERT_EQ(2, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(2, getNumUnpublishedCollisions());
	ASSERT_EQ(2, getNumPublishedCollisions());
}

TEST_F(PublishCollisionsTests, TwoRepZeroCollisionEach)
{
	setCollisionRepAndCollision(2, 0);

	// Run the computation...
	ASSERT_EQ(0, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
}

TEST_F(PublishCollisionsTests, TwoRepOneCollisionEach)
{
	setCollisionRepAndCollision(2, 1);

	// Run the computation...
	ASSERT_EQ(2, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(2, getNumUnpublishedCollisions());
	ASSERT_EQ(2, getNumPublishedCollisions());
}

TEST_F(PublishCollisionsTests, TwoRepTwoCollisionsEach)
{
	setCollisionRepAndCollision(2, 2);

	// Run the computation...
	ASSERT_EQ(4, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(4, getNumUnpublishedCollisions());
	ASSERT_EQ(4, getNumPublishedCollisions());
}

TEST_F(PublishCollisionsTests, ThreeRepThreeCollisionsEach)
{
	setCollisionRepAndCollision(3, 3);

	// Run the computation...
	ASSERT_EQ(9, getNumUnpublishedCollisions());
	ASSERT_EQ(0, getNumPublishedCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(9, getNumUnpublishedCollisions());
	ASSERT_EQ(9, getNumPublishedCollisions());
}

}; // namespace Physics
}; // namespace SurgSim

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

/// \file ClearCollisionsTests.cpp
/// Simple Test for ClearCollisions computation

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Physics/ClearCollisions.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/RigidRepresentation.h"

namespace SurgSim
{
namespace Physics
{

class ClearCollisionsTests : public ::testing::Test
{
public:
	void SetUp()
	{
		// Create the computation and the state
		m_computation = std::make_shared<ClearCollisions>();
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
					SurgSim::Collision::COLLISION_DETECTION_TYPE_DISCRETE,
					1.0, 1.0, SurgSim::Math::Vector3d::Ones(), SurgSim::Math::Vector3d::UnitX(),
					newPenetrationPoints);
				rigid->addContact(rigid, collision);
			}
			activeReps.push_back(rigid);
		}
		m_state->setCollisionRepresentations(activeReps);
		m_state->setActiveCollisionRepresentations(activeReps);
	}

	size_t getNumCollisions() const
	{
		size_t numCollisions = 0;

		for (const auto& rep : m_state->getActiveCollisionRepresentations())
		{
			numCollisions += rep->getCollisions().unsafeGet()[rep].size();
		}

		return numCollisions;
	}

protected:
	/// The computation
	std::shared_ptr<ClearCollisions> m_computation;

	/// The physics manager
	std::shared_ptr<PhysicsManagerState> m_state;
};

TEST_F(ClearCollisionsTests, ZeroRepZeroCollisionEach)
{
	setCollisionRepAndCollision(0, 0);

	// Run the computation...
	ASSERT_EQ(0, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

TEST_F(ClearCollisionsTests, OneRepZeroCollisionEach)
{
	setCollisionRepAndCollision(1, 0);

	// Run the computation...
	ASSERT_EQ(0, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

TEST_F(ClearCollisionsTests, OneRepOneCollisionEach)
{
	setCollisionRepAndCollision(1, 1);

	// Run the computation...
	ASSERT_EQ(1, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

TEST_F(ClearCollisionsTests, OneRepTwoCollisionsEach)
{
	setCollisionRepAndCollision(1, 2);

	// Run the computation...
	ASSERT_EQ(2, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

TEST_F(ClearCollisionsTests, TwoRepZeroCollisionEach)
{
	setCollisionRepAndCollision(2, 0);

	// Run the computation...
	ASSERT_EQ(0, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

TEST_F(ClearCollisionsTests, TwoRepOneCollisionEach)
{
	setCollisionRepAndCollision(2, 1);

	// Run the computation...
	ASSERT_EQ(2, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

TEST_F(ClearCollisionsTests, TwoRepTwoCollisionsEach)
{
	setCollisionRepAndCollision(2, 2);

	// Run the computation...
	ASSERT_EQ(4, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

TEST_F(ClearCollisionsTests, ThreeRepThreeCollisionsEach)
{
	setCollisionRepAndCollision(3, 3);

	// Run the computation...
	ASSERT_EQ(9, getNumCollisions());
	ASSERT_NO_THROW(m_computation->update(1e-3, m_state));
	ASSERT_EQ(0, getNumCollisions());
}

}; // namespace Physics
}; // namespace SurgSim

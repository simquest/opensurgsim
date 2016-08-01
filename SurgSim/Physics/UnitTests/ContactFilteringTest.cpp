// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/ContactFilter.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Physics/ContactFiltering.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"

using ::testing::_;

namespace SurgSim
{

class MockContactFilter : public Collision::ContactFilter
{
public:
	MockContactFilter(const std::string& name) : Collision::ContactFilter(name) {}
	MOCK_METHOD0(doWakeUp, bool());
	MOCK_METHOD0(doInitialize, bool());
	MOCK_METHOD1(doUpdate, void(double));
	MOCK_METHOD2(doFilterContacts, void(const std::shared_ptr<Physics::PhysicsManagerState>& state,
										const std::shared_ptr<Collision::CollisionPair>& pair));


};

void removeOne(const std::shared_ptr<Physics::PhysicsManagerState>& state,
			   const std::shared_ptr<Collision::CollisionPair>& pair)
{
	pair->getContacts().pop_back();
}

namespace Physics
{


struct ContactFilteringTest : public ::testing::Test
{
	virtual void SetUp()
	{
		// Setup Framework
		runtime = std::make_shared<Framework::Runtime>();
		state = std::make_shared<PhysicsManagerState>();
		contactFiltering = std::make_shared<ContactFiltering>(false);

		std::vector<std::shared_ptr<Collision::ContactFilter>> filters;
		filter = std::make_shared<MockContactFilter>("Filter");
		filters.push_back(filter);
		state->setContactFilters(filters);

		collision0 = std::make_shared<Physics::RigidCollisionRepresentation>("Collision0");


		pairWithContacts = std::make_shared<Collision::CollisionPair>(collision0, collision0);
		auto contact = std::make_shared<Collision::Contact>(Collision::COLLISION_DETECTION_TYPE_NONE,
					   0.0, 0.0, Math::Vector3d::Zero(), Math::Vector3d::Zero(),
					   std::make_pair(DataStructures::Location(), DataStructures::Location()));
		pairWithContacts->addContact(contact);
		pairWithContacts->addContact(contact);
		pairWithoutContacts = std::make_shared<Collision::CollisionPair>(collision0, collision0);

	}

	std::shared_ptr<PhysicsManagerState> state;
	std::shared_ptr<ContactFiltering> contactFiltering;
	std::shared_ptr<Framework::Runtime> runtime;
	std::shared_ptr<MockContactFilter> filter;

	std::shared_ptr<Physics::RigidCollisionRepresentation> collision0;

	std::shared_ptr<Collision::CollisionPair> pairWithContacts;
	std::shared_ptr<Collision::CollisionPair> pairWithoutContacts;

	std::vector<std::shared_ptr<Collision::CollisionPair>> pairs;

};

TEST_F(ContactFilteringTest, DontProcessWithoutPairs)
{
	EXPECT_CALL(*filter, doFilterContacts(_, _)).Times(0);
	contactFiltering->update(1.0, state);
}


TEST_F(ContactFilteringTest, ProcessAllPairsWithContacts)
{
	pairs.push_back(pairWithContacts);
	pairs.push_back(pairWithContacts);
	pairs.push_back(pairWithoutContacts);
	state->setCollisionPairs(pairs);

	EXPECT_CALL(*filter, doFilterContacts(_, _)).Times(2);
	contactFiltering->update(1.0, state);
}


TEST_F(ContactFilteringTest, ModifyContacts)
{
	pairs.push_back(pairWithContacts);

	state->setCollisionPairs(pairs);

	EXPECT_CALL(*filter, doFilterContacts(_, _)).WillOnce(testing::Invoke(removeOne));
	contactFiltering->update(1.0, state);
	EXPECT_EQ(1u, pairWithContacts->getContacts().size());
}

TEST_F(ContactFilteringTest, ProcessAllFilters)
{
	// Gmock is might not be threadsafe on windows, need separate instances
	std::vector<std::shared_ptr<Collision::ContactFilter>> filters;
	auto filter1 = std::make_shared<MockContactFilter>("Filter1");
	filters.push_back(filter1);
	auto filter2 = std::make_shared<MockContactFilter>("Filter2");
	filters.push_back(filter2);
	state->setContactFilters(filters);

	pairs.push_back(pairWithContacts);
	state->setCollisionPairs(pairs);

	EXPECT_CALL(*filter1, doFilterContacts(_, _)).Times(1);
	EXPECT_CALL(*filter2, doFilterContacts(_, _)).Times(1);
	contactFiltering->update(1.0, state);
}

}

}
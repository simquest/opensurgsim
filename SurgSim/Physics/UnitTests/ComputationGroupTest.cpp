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

#include "SurgSim/Physics/ComputationGroup.h"
#include "SurgSim/Physics/Computation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

typedef std::shared_ptr<SurgSim::Physics::PhysicsManagerState> State;

class MockComputation : public SurgSim::Physics::Computation
{
public:
	MockComputation(bool val) : Computation(val)
	{
		testing::DefaultValue<State>::Set(std::make_shared<SurgSim::Physics::PhysicsManagerState>());
	};

	SURGSIM_CLASSNAME(MockComputation);

	MOCK_METHOD2(doUpdate, State(const double&, const State&));

};

class MockGroup : public SurgSim::Physics::ComputationGroup
{
public:
	MockGroup(bool val) : ComputationGroup(val)
	{
		testing::DefaultValue<State>::Set(std::make_shared<SurgSim::Physics::PhysicsManagerState>());
		ON_CALL(*this, endIteration()).WillByDefault(testing::Return(true));
	}
	MOCK_METHOD0(endIteration, bool(void));
};


}

namespace SurgSim
{

namespace Physics
{

TEST(ComputationGroupTest, Empty)
{
	auto state = std::make_shared<PhysicsManagerState>();
	auto computation = std::make_shared<ComputationGroup>(false);

	EXPECT_NO_THROW(computation->update(0.0, state));
}

TEST(ComputationGroupTest, RunOnce)
{
	using ::testing::_;
	auto state = std::make_shared<PhysicsManagerState>();
	auto computation = std::make_shared<MockGroup>(false);
	auto mock = std::make_shared<MockComputation>(false);

	computation->addComputation(mock);

	EXPECT_CALL(*mock, doUpdate(0.0, _));
	EXPECT_CALL(*computation, endIteration());

	computation->update(0.0, state);
}

TEST(ComputationGroupTest, MultiplesRunMultiples)
{
	using ::testing::_;
	using ::testing::Return;
	auto state = std::make_shared<PhysicsManagerState>();
	auto computation = std::make_shared<MockGroup>(false);
	auto mock = std::make_shared<MockComputation>(false);

	computation->addComputation(mock);
	computation->addComputation(mock);

	EXPECT_CALL(*mock, doUpdate(_, _)).Times(4);
	EXPECT_CALL(*computation, endIteration()).Times(2).WillOnce(Return(false)).WillOnce(Return(true));

	computation->update(0.0, state);
}

TEST(ComputationGroupTest, AbortInTheMiddle)
{
	using ::testing::_;
	using ::testing::Return;
	std::vector<std::shared_ptr<Computation>> computations;
	auto state = std::make_shared<PhysicsManagerState>();
	auto state2 = std::make_shared<PhysicsManagerState>();
	state2->setAbortLoop(true);
	auto computation = std::make_shared<MockGroup>(false);
	auto mock = std::make_shared<MockComputation>(false);
	computations.push_back(mock);
	computations.push_back(mock);

	computation->setComputations(computations);

	EXPECT_CALL(*mock, doUpdate(_, _)).Times(3)
	.WillOnce(Return(state))
	.WillOnce(Return(state))
	.WillOnce(Return(state2));

	EXPECT_CALL(*computation, endIteration()).WillOnce(Return(false));

	computation->update(0.0, state);

	ASSERT_FALSE(state2->shouldAbortLoop());
}
}
}
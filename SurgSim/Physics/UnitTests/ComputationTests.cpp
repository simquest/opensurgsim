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
#include <SurgSim/Physics/Computation.h>


namespace SurgSim 
{
namespace Physics
{

class MockComputation : public Computation
{
public:
	explicit MockComputation(bool doCopyState = false) : Computation(doCopyState) 
	{

	}

protected:
	virtual std::shared_ptr<PhysicsManagerState> doUpdate(
		const double& dt, 
		const std::shared_ptr<PhysicsManagerState>& state) override
	{
		return state;
	}
};

TEST(ComputationTests, InitTest)
{
	EXPECT_NO_THROW({MockComputation c;});
	MockComputation c;
	EXPECT_FALSE(c.isCopyingState());

	MockComputation d(true);
	EXPECT_TRUE(d.isCopyingState());
}

TEST(ComputationTests, CopyStateTest)
{
	std::shared_ptr<PhysicsManagerState> state0 = std::make_shared<PhysicsManagerState>();

	MockComputation c;
	auto state1 = c.update(1.0, state0);

	EXPECT_EQ(state0.get(), state1.get());

	c.setDoCopyState(true);
	EXPECT_TRUE(c.isCopyingState());

	auto state2 = c.update(1.0, state0);
	EXPECT_NE(state0.get(), state2.get());
}

}; // namespace Physics
}; // namespace SurgSim

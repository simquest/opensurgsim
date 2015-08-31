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

/// \file
/// Tests for the MouseScaffold class and its device interactions.

#include <gtest/gtest.h>

#include "SurgSim/Devices/Mouse/MouseScaffold.h"

using SurgSim::Devices::MouseScaffold;

TEST(MouseScaffoldTest, CreateAndDestroyScaffold)
{
	std::shared_ptr<MouseScaffold> scaffold = MouseScaffold::getOrCreateSharedInstance();
	ASSERT_TRUE(nullptr != scaffold) << "The scaffold was not created!";
	std::weak_ptr<MouseScaffold> scaffold1 = scaffold;
	{
		std::shared_ptr<MouseScaffold> stillHaveScaffold = scaffold1.lock();
		EXPECT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		EXPECT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
	{
		std::shared_ptr<MouseScaffold> sameScaffold = MouseScaffold::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, sameScaffold) << "Unable to get scaffold from class";
		EXPECT_EQ(scaffold, sameScaffold) << "Scaffold mismatch!";
	}

	scaffold.reset();
	{
		std::shared_ptr<MouseScaffold> dontHaveScaffold = scaffold1.lock();
		EXPECT_TRUE(nullptr == dontHaveScaffold) << "Able to get scaffold from weak ref (with no strong ref)";
	}

	scaffold = MouseScaffold::getOrCreateSharedInstance();
	ASSERT_TRUE(nullptr != scaffold) << "The scaffold was not created the 2nd time!";
	std::weak_ptr<MouseScaffold> scaffold2 = scaffold;
	{
		std::shared_ptr<MouseScaffold> stillHaveScaffold = scaffold2.lock();
		ASSERT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		ASSERT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
}
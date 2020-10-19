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
/// Tests for the TrakstarScaffold class and its device interactions.

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Devices/Trakstar/TrakstarDevice.h"
#include "SurgSim/Devices/Trakstar/TrakstarScaffold.h"

using SurgSim::Devices::TrakstarDevice;
using SurgSim::Devices::TrakstarScaffold;

TEST(TrakstarScaffoldTest, CreateAndDestroyScaffold)
{
	std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not created!";

	std::weak_ptr<TrakstarScaffold> weakScaffold = scaffold;
	{
		std::shared_ptr<TrakstarScaffold> stillHaveScaffold = weakScaffold.lock();
		EXPECT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		EXPECT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
	{
		std::shared_ptr<TrakstarScaffold> sameScaffold = TrakstarScaffold::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, sameScaffold) << "Unable to get scaffold from class";
		EXPECT_EQ(scaffold, sameScaffold) << "Scaffold mismatch!";
	}

	scaffold.reset();
	{
		std::shared_ptr<TrakstarScaffold> dontHaveScaffold = weakScaffold.lock();
		EXPECT_EQ(nullptr, dontHaveScaffold) << "Able to get scaffold from weak ref (with no strong ref)";
	}

	scaffold = TrakstarScaffold::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not created the 2nd time!";
	std::weak_ptr<TrakstarScaffold> yetAnotherWeakScaffold = scaffold;
	{
		std::shared_ptr<TrakstarScaffold> stillHaveScaffold = yetAnotherWeakScaffold.lock();
		ASSERT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		ASSERT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
}

TEST(TrakstarScaffoldTest, ScaffoldLifeCycle)
{
	std::weak_ptr<TrakstarScaffold> lastScaffold;
	{
		std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not created!";
		lastScaffold = scaffold;
	}
	{
		std::shared_ptr<TrakstarScaffold> dontHaveScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, dontHaveScaffold) << "Able to get scaffold from weak ref (with no strong ref)";
		lastScaffold.reset();
	}

	{
		std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
		ASSERT_NE(nullptr, device) << "Creation failed.  Is a Trakstar device plugged in?";
		// note: the device is NOT initialized!
		{
			std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			lastScaffold = scaffold;  // save the scaffold for later

			std::shared_ptr<TrakstarScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);
			EXPECT_EQ(scaffold, sameScaffold);
		}
		// The device has not been initialized, so it should NOT be hanging on to the device!
		{
			std::shared_ptr<TrakstarScaffold> deadScaffold = lastScaffold.lock();
			EXPECT_EQ(nullptr, deadScaffold);
		}
		// the ("empty") device is about to get destroyed
	}

	{
		std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";
		{
			std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			lastScaffold = scaffold;  // save the scaffold for later

			std::shared_ptr<TrakstarScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);
			EXPECT_EQ(scaffold, sameScaffold);
		}
		// The same scaffold is supposed to still be around because of the device
		{
			std::shared_ptr<TrakstarScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);

			std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			EXPECT_EQ(sameScaffold, scaffold);
		}
		// the device and the scaffold are about to get destroyed
	}

	{
		std::shared_ptr<TrakstarScaffold> deadScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, deadScaffold);
	}

	{
		std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Didn't this work a moment ago?";
		std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";

		std::shared_ptr<TrakstarScaffold> deadScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, deadScaffold);
	}
}


TEST(TrakstarScaffoldTest, CreateDeviceSeveralTimes)
{
	std::weak_ptr<TrakstarScaffold> lastScaffold;

	for (int i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		EXPECT_EQ(nullptr, lastScaffold.lock());
		std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";
		std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
		lastScaffold = scaffold;
		// the device and the scaffold will be destroyed here
	}
}

TEST(TrakstarScaffoldTest, CreateDeviceSeveralTimesWithScaffoldRef)
{
	std::shared_ptr<TrakstarScaffold> lastScaffold;

	for (int i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";
		std::shared_ptr<TrakstarScaffold> scaffold = TrakstarScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
		if (!lastScaffold)
		{
			lastScaffold = scaffold;
		}
		EXPECT_EQ(lastScaffold, scaffold);
		// the device will be destroyed here, but the scaffold stays around because we have a shared_ptr to it.
	}
}

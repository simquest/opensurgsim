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
/// Tests for the NovintScaffold class and its device interactions.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include "SurgSim/Devices/Novint/NovintDevice.h"
#include "SurgSim/Devices/Novint/NovintScaffold.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Device::NovintDevice;
using SurgSim::Device::NovintScaffold;


// Use common device names defined in the NovintDeviceTest code.
extern const char* const NOVINT_TEST_DEVICE_NAME;


TEST(NovintScaffoldTest, CreateAndDestroyScaffold)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not created!";
	std::weak_ptr<NovintScaffold> scaffold1 = scaffold;
	{
		std::shared_ptr<NovintScaffold> stillHaveScaffold = scaffold1.lock();
		EXPECT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		EXPECT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
	{
		std::shared_ptr<NovintScaffold> sameScaffold = NovintScaffold::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, sameScaffold) << "Unable to get scaffold from class";
		EXPECT_EQ(scaffold, sameScaffold) << "Scaffold mismatch!";
	}

	scaffold.reset();
	{
		std::shared_ptr<NovintScaffold> dontHaveScaffold = scaffold1.lock();
		EXPECT_EQ(nullptr, dontHaveScaffold) << "Able to get scaffold from weak ref (with no strong ref)";
	}

	scaffold = NovintScaffold::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not created the 2nd time!";
	std::weak_ptr<NovintScaffold> scaffold2 = scaffold;
	{
		std::shared_ptr<NovintScaffold> stillHaveScaffold = scaffold2.lock();
		ASSERT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		ASSERT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
}

TEST(NovintScaffoldTest, ScaffoldLifeCycle)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::weak_ptr<NovintScaffold> lastScaffold;
	{
		std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not created!";
		lastScaffold = scaffold;
	}
	{
		std::shared_ptr<NovintScaffold> dontHaveScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, dontHaveScaffold) << "Able to get scaffold from weak ref (with no strong ref)";
		lastScaffold.reset();
	}

	{
		std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
		ASSERT_NE(nullptr, device) << "Creation failed.  Is a Novint device plugged in?";
		// note: the device is NOT initialized!
		{
			std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			lastScaffold = scaffold;  // save the scaffold for later

			std::shared_ptr<NovintScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);
			EXPECT_EQ(scaffold, sameScaffold);
		}
		// The device has not been initialized, so it should NOT be hanging on to the device!
		{
			std::shared_ptr<NovintScaffold> deadScaffold = lastScaffold.lock();
			EXPECT_EQ(nullptr, deadScaffold);
		}
		// the ("empty") device is about to get destroyed
	}

	{
		std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
		{
			std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			lastScaffold = scaffold;  // save the scaffold for later

			std::shared_ptr<NovintScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);
			EXPECT_EQ(scaffold, sameScaffold);
		}
		// The same scaffold is supposed to still be around because of the device
		{
			std::shared_ptr<NovintScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);

			std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			EXPECT_EQ(sameScaffold, scaffold);
		}
		// the device and the scaffold are about to get destroyed
	}

	{
		std::shared_ptr<NovintScaffold> deadScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, deadScaffold);
	}

	{
		std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Didn't this work a moment ago?";
		std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";

		std::shared_ptr<NovintScaffold> deadScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, deadScaffold);
	}
}


TEST(NovintScaffoldTest, CreateDeviceSeveralTimes)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::weak_ptr<NovintScaffold> lastScaffold;

	for (int i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		EXPECT_EQ(nullptr, lastScaffold.lock());
		std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
		std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
		lastScaffold = scaffold;
		// the device and the scaffold will be destroyed here
	}
}


TEST(NovintScaffoldTest, CreateDeviceSeveralTimesWithScaffoldRef)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<NovintScaffold> lastScaffold;

	for (int i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
		std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
		if (! lastScaffold)
		{
			lastScaffold = scaffold;
		}
		EXPECT_EQ(lastScaffold, scaffold);
		// the device will be destroyed here, but the scaffold stays around because we have a shared_ptr to it.
	}
}

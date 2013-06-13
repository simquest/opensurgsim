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
/// Tests for the SixenseManager class and its device interactions.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include "SurgSim/Devices/Sixense/SixenseDevice.h"
#include "SurgSim/Devices/Sixense/SixenseManager.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Device::SixenseDevice;
using SurgSim::Device::SixenseManager;

TEST(SixenseManagerTest, CreateAndDestroyManager)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, manager) << "The manager was not created!";
	std::weak_ptr<SixenseManager> manager1 = manager;
	{
		std::shared_ptr<SixenseManager> stillHaveManager = manager1.lock();
		EXPECT_NE(nullptr, stillHaveManager) << "Unable to get manager from weak ref (while strong ref exists)";
		EXPECT_EQ(manager, stillHaveManager) << "Manager mismatch!";
	}
	{
		std::shared_ptr<SixenseManager> sameManager = SixenseManager::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, sameManager) << "Unable to get manager from class";
		EXPECT_EQ(manager, sameManager) << "Manager mismatch!";
	}

	manager.reset();
	{
		std::shared_ptr<SixenseManager> dontHaveManager = manager1.lock();
		EXPECT_EQ(nullptr, dontHaveManager) << "Able to get manager from weak ref (with no strong ref)";
	}

	manager = SixenseManager::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, manager) << "The manager was not created the 2nd time!";
	std::weak_ptr<SixenseManager> manager2 = manager;
	{
		std::shared_ptr<SixenseManager> stillHaveManager = manager2.lock();
		ASSERT_NE(nullptr, stillHaveManager) << "Unable to get manager from weak ref (while strong ref exists)";
		ASSERT_EQ(manager, stillHaveManager) << "Manager mismatch!";
	}
}

TEST(SixenseManagerTest, ManagerLifeCycle)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::weak_ptr<SixenseManager> lastManager;
	{
		std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, manager) << "The manager was not created!";
		lastManager = manager;
	}
	{
		std::shared_ptr<SixenseManager> dontHaveManager = lastManager.lock();
		EXPECT_EQ(nullptr, dontHaveManager) << "Able to get manager from weak ref (with no strong ref)";
		lastManager.reset();
	}

	{
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_NE(nullptr, device) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		{
			std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, manager) << "The manager was not retrieved!";
			lastManager = manager;  // save the manager for later

			std::shared_ptr<SixenseManager> sameManager = lastManager.lock();
			EXPECT_NE(nullptr, sameManager);
			EXPECT_EQ(manager, sameManager);
		}
		// The same manager is supposed to still be around because of the device
		{
			std::shared_ptr<SixenseManager> sameManager = lastManager.lock();
			EXPECT_NE(nullptr, sameManager);

			std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, manager) << "The manager was not retrieved!";
			EXPECT_EQ(sameManager, manager);
		}
		// the device and the manager are about to get destroyed
	}

	{
		std::shared_ptr<SixenseManager> deadManager = lastManager.lock();
		EXPECT_EQ(nullptr, deadManager);
	}

	{
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_NE(nullptr, device) << "Initialization failed.  Didn't this work a moment ago?";
		std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, manager) << "The manager was not retrieved!";

		std::shared_ptr<SixenseManager> deadManager = lastManager.lock();
		EXPECT_EQ(nullptr, deadManager);
	}
}


TEST(SixenseManagerTest, CreateDeviceSeveralTimes)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::weak_ptr<SixenseManager> lastManager;

	for (int i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		EXPECT_EQ(nullptr, lastManager.lock());
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_NE(nullptr, device) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, manager) << "The manager was not retrieved!";
		lastManager = manager;
		// the device and the manager will be destroyed here
	}
}


TEST(SixenseManagerTest, CreateDeviceSeveralTimesWithManagerRef)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseManager> lastManager;

	for (int i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_NE(nullptr, device) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, manager) << "The manager was not retrieved!";
		if (! lastManager)
		{
			lastManager = manager;
		}
		EXPECT_EQ(lastManager, manager);
		// the device will be destroyed here, but the manager stays around because we have a shared_ptr to it.
	}
}

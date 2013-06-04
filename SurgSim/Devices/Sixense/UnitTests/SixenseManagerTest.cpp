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

namespace SurgSim
{
namespace Device
{
namespace Test
{

std::shared_ptr<SixenseManager> extractManager(const SixenseDevice& device)
{
	return device.getManager();
}

};
};
};

using SurgSim::Device::Test::extractManager;


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
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		std::shared_ptr<SixenseManager> manager = extractManager(*device);
		ASSERT_TRUE(manager != nullptr) << "The device doesn't know its manager!";

		lastManager = manager;  // save the manager for later
		std::shared_ptr<SixenseManager> sameManager = lastManager.lock();
		EXPECT_TRUE(sameManager != nullptr);
		EXPECT_EQ(manager, sameManager);
		// the device and the manager are about to get destroyed
	}

	{
		std::shared_ptr<SixenseManager> deadManager = lastManager.lock();
		EXPECT_TRUE(deadManager == nullptr);
	}

	{
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		std::shared_ptr<SixenseManager> manager = extractManager(*device);
		ASSERT_TRUE(manager != nullptr) << "The device doesn't know its manager!";

		std::shared_ptr<SixenseManager> deadManager = lastManager.lock();
		EXPECT_TRUE(deadManager == nullptr);
	}
}


TEST(SixenseManagerTest, CreateDeviceSeveralTimesWithManager)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseManager> manager;

	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		if (! manager)
		{
			manager = extractManager(*device);
		}
		EXPECT_EQ(manager, extractManager(*device));
		// the device will be destroyed here, but the manager stays around because we have a shared_ptr to it.
	}
}

TEST(SixenseManagerTest, CreateSeveralDevices)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseDevice> device1 = SixenseDevice::create("Sixense1");
	ASSERT_TRUE(device1 != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";

	std::shared_ptr<SixenseDevice> device2 = SixenseDevice::create("Sixense2");
	ASSERT_TRUE(device2 != nullptr) << "Initialization failed for second controller." <<
		"  Is only one controller plugged in?";

	EXPECT_EQ(extractManager(*device1), extractManager(*device2));

	std::shared_ptr<SixenseDevice> device3 = SixenseDevice::create("Sixense3");
	if (! device3)
	{
		std::cerr << "[Warning: third Sixense/Hydra controller not actually created; is it plugged in?]" << std::endl;
	}
}

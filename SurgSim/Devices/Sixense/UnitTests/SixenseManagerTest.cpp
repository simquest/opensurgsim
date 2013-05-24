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


TEST(SixenseManagerTest, GetManagerFromDevice)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
	ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
	std::shared_ptr<SixenseManager> manager = device->getManager();
	ASSERT_TRUE(manager != nullptr) << "The device doesn't know its manager!";
}

TEST(SixenseManagerTest, ManagerLifeCycle)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::weak_ptr<SixenseManager> lastManager;

	{
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		std::shared_ptr<SixenseManager> manager = device->getManager();
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
		std::shared_ptr<SixenseManager> manager = device->getManager();
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
			manager = device->getManager();
		}
		EXPECT_EQ(manager, device->getManager());
		// the device will be destroyed here, but the manager stays around because we have a shared_ptr to it.
	}
}

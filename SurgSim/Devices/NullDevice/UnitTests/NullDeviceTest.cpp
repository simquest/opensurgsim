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

/** @file
 * Tests for the NullDevice class.
 */

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <SurgSim/Devices/NullDevice/NullDevice.h>
#include <SurgSim/Input/DataGroup.h>
#include <SurgSim/Input/InputDeviceListenerInterface.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Device::NullDevice;
using SurgSim::Input::DataGroup;
using SurgSim::Input::InputDeviceListenerInterface;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;


struct TestListener : public InputDeviceListenerInterface
{
public:
	TestListener() :
		m_numTimesReceivedInput(0),
		m_numTimesRequestedOutput(0)
	{
	}

	virtual void handleInput(const std::string& device, const DataGroup& inputData);
	virtual bool requestOutput(const std::string& device, DataGroup* outputData);

	int m_numTimesReceivedInput;
	int m_numTimesRequestedOutput;
	DataGroup m_lastReceivedInput;
};

void TestListener::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}

bool TestListener::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;
	return false;
}


TEST(NullDeviceTest, CanConstruct)
{
	EXPECT_NO_THROW({NullDevice device("MyNullDevice");});
}

TEST(NullDeviceTest, Name)
{
	NullDevice device("MyNullDevice");
	EXPECT_EQ("MyNullDevice", device.getName());
}

TEST(NullDeviceTest, AddListener)
{
	NullDevice device("MyNullDevice");
	std::shared_ptr<TestListener> listener = std::make_shared<TestListener>();
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputListener(listener));

	// NullDevice is supposed to shove "null" data at every listener when it's added.
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);
	EXPECT_TRUE(listener->m_lastReceivedInput.poses().hasCurrentData("pose"));
	EXPECT_TRUE(listener->m_lastReceivedInput.booleans().hasCurrentData("button0"));

	RigidTransform3d pose;
	EXPECT_TRUE(listener->m_lastReceivedInput.poses().get("pose", pose));
	EXPECT_NEAR(0, (pose.matrix() - Matrix44d::Identity()).norm(), 1e-6);

	bool button0;
	EXPECT_TRUE(listener->m_lastReceivedInput.booleans().get("button0", button0));
	EXPECT_FALSE(button0);
}

TEST(NullDeviceTest, RemoveListener)
{
	NullDevice device("MyNullDevice");
	std::shared_ptr<TestListener> listener = std::make_shared<TestListener>();
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_FALSE(device.removeListener(listener));
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputListener(listener));
	// NullDevice is supposed to shove "null" data at every listener when it's added.
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);

	EXPECT_TRUE(device.removeListener(listener));
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);

	EXPECT_FALSE(device.removeListener(listener));
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);
}

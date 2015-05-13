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
/// Tests for the IdentityPoseDevice class.

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Device::IdentityPoseDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Testing::MockInputOutput;


TEST(IdentityPoseDeviceTest, CanConstruct)
{
	EXPECT_NO_THROW({IdentityPoseDevice device("MyIdentityPoseDevice");});
}

TEST(IdentityPoseDeviceTest, Name)
{
	IdentityPoseDevice device("MyIdentityPoseDevice");
	EXPECT_EQ("MyIdentityPoseDevice", device.getName());
}

TEST(IdentityPoseDeviceTest, Factory)
{
	std::shared_ptr<SurgSim::Input::DeviceInterface> device;
	ASSERT_NO_THROW(device = SurgSim::Input::DeviceInterface::getFactory().create(
								 "SurgSim::Device::IdentityPoseDevice", "Device"));
	EXPECT_NE(nullptr, device);
}

TEST(IdentityPoseDeviceTest, AddInputConsumer)
{
	IdentityPoseDevice device("MyIdentityPoseDevice");
	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputConsumer(consumer));

	// IdentityPoseDevice is supposed to shove an identity pose (and a button) at every consumer when it's added.
	EXPECT_EQ(1, consumer->m_numTimesReceivedInput);
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData(SurgSim::DataStructures::Names::POSE));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData(SurgSim::DataStructures::Names::BUTTON_0));

	// Check the data.
	RigidTransform3d pose = SurgSim::Math::makeRigidTransform(SurgSim::Math::Vector3d(1.3, 32.0, 68.0),
							SurgSim::Math::Vector3d(13.2, 2.8, 8.0), SurgSim::Math::Vector3d(273.0, -32.0, -6.0));
	ASSERT_TRUE(consumer->m_lastReceivedInput.poses().get(SurgSim::DataStructures::Names::POSE, &pose));
	EXPECT_NEAR(0, (pose.matrix() - Matrix44d::Identity()).norm(), 1e-6);
	bool button0 = false;
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().get(SurgSim::DataStructures::Names::BUTTON_0, &button0));
	EXPECT_FALSE(button0);

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device.addInputConsumer(consumer));
	EXPECT_EQ(1, consumer->m_numTimesReceivedInput);

	// Adding a different device should push to it.
	std::shared_ptr<MockInputOutput> consumer2 = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer2->m_numTimesReceivedInput);
	EXPECT_TRUE(device.addInputConsumer(consumer2));
	EXPECT_EQ(1, consumer2->m_numTimesReceivedInput);
	// We don't care if the first consumer was updated again or not.
	EXPECT_TRUE((consumer->m_numTimesReceivedInput >= 1) && (consumer->m_numTimesReceivedInput <= 2)) <<
			"consumer->m_numTimesReceivedInput = " << consumer->m_numTimesReceivedInput << std::endl;
}

TEST(IdentityPoseDeviceTest, RemoveInputConsumer)
{
	IdentityPoseDevice device("MyIdentityPoseDevice");
	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device.removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputConsumer(consumer));
	// IdentityPoseDevice is supposed to shove an identity pose (and a button) at every consumer when it's added.
	EXPECT_EQ(1, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.removeInputConsumer(consumer));
	EXPECT_EQ(1, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device.removeInputConsumer(consumer));
	EXPECT_EQ(1, consumer->m_numTimesReceivedInput);
}

TEST(IdentityPoseDeviceTest, SetOutputProducer)
{
	IdentityPoseDevice device("MyIdentityPoseDevice");
	std::shared_ptr<MockInputOutput> producer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device.setOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
}

TEST(IdentityPoseDeviceTest, RemoveOutputProducer)
{
	IdentityPoseDevice device("MyIdentityPoseDevice");
	std::shared_ptr<MockInputOutput> producer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device.removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.setOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesReceivedInput);

	EXPECT_FALSE(device.removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesReceivedInput);
}

#include <memory>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>

#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/Testing/MockInputOutput.h>

#include "SurgSim/Devices/Oculus/OculusDevice.h"

using SurgSim::Device::OculusDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Testing::MockInputOutput;

TEST(OculusDeviceTest, CreateAndInitializeDevice)
{
	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";

	EXPECT_FALSE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";

	EXPECT_TRUE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());
}

TEST(OculusDeviceTest, FinalizeDevice)
{
	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";
	EXPECT_TRUE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());

	ASSERT_TRUE(device->finalize()) << "Finalization failed.";
	EXPECT_FALSE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());
}

TEST(OculusDeviceTest, RegisterMoreThanOneDevice)
{
	auto device1 = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device1) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";

	auto device2 = std::make_shared<OculusDevice>("Oculus2");
	ASSERT_TRUE(nullptr != device2) << "Device creation failed.";
	EXPECT_ANY_THROW(device2->initialize());
}

TEST(OculusDeviceTest, InputConsumer)
{
	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";

	auto consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));
	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	EXPECT_GE(consumer->m_numTimesReceivedInput, 800);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 1200);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("pose"));
}

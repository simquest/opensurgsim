//// This file is a part of the OpenSurgSim project.
//// Copyright 2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.
//
///// \file
///// Tests for the TrackIRDevice class.
//
//#include <memory>
//#include <string>
//#include <boost/thread.hpp>
//#include <boost/chrono.hpp>
//#include <gtest/gtest.h>
//#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
////#include "SurgSim/Devices/TrackIR/TrackIRScaffold.h"  // only needed if calling setDefaultLogLevel()
//#include "SurgSim/DataStructures/DataGroup.h"
//#include "SurgSim/Input/InputConsumerInterface.h"
//#include "SurgSim/Input/OutputProducerInterface.h"
//#include "SurgSim/Math/RigidTransform.h"
//#include "SurgSim/Math/Matrix.h"
//
//using SurgSim::Device::TrackIRDevice;
//using SurgSim::Device::TrackIRScaffold;
//using SurgSim::DataStructures::DataGroup;
//using SurgSim::Input::InputConsumerInterface;
//using SurgSim::Input::OutputProducerInterface;
//using SurgSim::Math::RigidTransform3d;
//using SurgSim::Math::Matrix44d;
//
//
//struct TestListener : public InputConsumerInterface, public OutputProducerInterface
//{
//public:
//	TestListener() :
//		m_numTimesInitializedInput(0),
//		m_numTimesReceivedInput(0),
//		m_numTimesRequestedOutput(0)
//	{
//	}
//
//	virtual void initializeInput(const std::string& device, const DataGroup& inputData);
//	virtual void handleInput(const std::string& device, const DataGroup& inputData);
//	virtual bool requestOutput(const std::string& device, DataGroup* outputData);
//
//	int m_numTimesInitializedInput;
//	int m_numTimesReceivedInput;
//	int m_numTimesRequestedOutput;
//	DataGroup m_lastReceivedInput;
//};
//
//void TestListener::initializeInput(const std::string& device, const DataGroup& inputData)
//{
//	++m_numTimesInitializedInput;
//}
//
//void TestListener::handleInput(const std::string& device, const DataGroup& inputData)
//{
//	++m_numTimesReceivedInput;
//	m_lastReceivedInput = inputData;
//}
//
//bool TestListener::requestOutput(const std::string& device, DataGroup* outputData)
//{
//	++m_numTimesRequestedOutput;
//	return false;
//}
//
//
//TEST(PhantomDeviceTest, CreateUninitializedDevice)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TestPhantom", "Default PHANToM");
//	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//}
//
//TEST(PhantomDeviceTest, CreateAndInitializeDevice)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TestPhantom", "Default PHANToM");
//	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//	EXPECT_FALSE(device->isInitialized());
//	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//	EXPECT_TRUE(device->isInitialized());
//}
//
//TEST(PhantomDeviceTest, CreateAndInitializeDefaultDevice)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TestPhantom", "");
//	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//	EXPECT_FALSE(device->isInitialized());
//	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//	EXPECT_TRUE(device->isInitialized());
//}
//
//TEST(PhantomDeviceTest, Name)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TestPhantom", "Default PHANToM");
//	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//	EXPECT_EQ("TestPhantom", device->getName());
//	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//	EXPECT_EQ("TestPhantom", device->getName());
//}
//
//static void testCreateDeviceSeveralTimes(bool doSleep)
//{
//	for (int i = 0;  i < 6;  ++i)
//	{
//		std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TestPhantom", "Default PHANToM");
//		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//		if (doSleep)
//		{
//			boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
//		}
//		// the device will be destroyed here
//	}
//}
//
//TEST(PhantomDeviceTest, CreateDeviceSeveralTimes)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	testCreateDeviceSeveralTimes(true);
//}
//
//TEST(PhantomDeviceTest, CreateSeveralDevices)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device1 = std::make_shared<TrackIRDevice>("Phantom1", "Default PHANToM");
//	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
//	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//
//	// We can't check what happens with the scaffolds, since those are no longer a part of the device's API...
//
//	std::shared_ptr<TrackIRDevice> device2 = std::make_shared<TrackIRDevice>("Phantom2", "Second PHANToM");
//	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
//	if (! device2->initialize())
//	{
//		std::cerr << "[Warning: second TrackIR did not come up; is it plugged in?]" << std::endl;
//	}
//}
//
//TEST(PhantomDeviceTest, CreateDevicesWithSameName)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device1 = std::make_shared<TrackIRDevice>("TrackIR", "Default PHANToM");
//	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
//	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//
//	std::shared_ptr<TrackIRDevice> device2 = std::make_shared<TrackIRDevice>("TrackIR", "Second PHANToM");
//	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
//	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
//}
//
//TEST(PhantomDeviceTest, CreateDevicesWithSameInitializationName)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device1 = std::make_shared<TrackIRDevice>("Phantom1", "Default PHANToM");
//	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
//	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//
//	std::shared_ptr<TrackIRDevice> device2 = std::make_shared<TrackIRDevice>("Phantom2", "Default PHANToM");
//	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
//	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate initialization name.";
//}
//
//TEST(PhantomDeviceTest, InputConsumer)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TestPhantom", "Default PHANToM");
//	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//
//	std::shared_ptr<TestListener> consumer = std::make_shared<TestListener>();
//	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);
//
//	EXPECT_FALSE(device->removeInputConsumer(consumer));
//	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);
//
//	EXPECT_TRUE(device->addInputConsumer(consumer));
//
//	// Adding the same input consumer again should fail.
//	EXPECT_FALSE(device->addInputConsumer(consumer));
//
//	// Sleep for a second, to see how many times the consumer is invoked.
//	// (A TrackIR device is supposed to run at 1KHz.)
//	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));
//
//	EXPECT_TRUE(device->removeInputConsumer(consumer));
//
//	// Removing the same input consumer again should fail.
//	EXPECT_FALSE(device->removeInputConsumer(consumer));
//
//	// Check the number of invocations.
//	EXPECT_GE(consumer->m_numTimesReceivedInput, 700);
//	EXPECT_LE(consumer->m_numTimesReceivedInput, 1300);
//
//	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("pose"));
//	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button1"));
//}
//
//TEST(PhantomDeviceTest, OutputProducer)
//{
//	//TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TestPhantom", "Default PHANToM");
//	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
//
//	std::shared_ptr<TestListener> producer = std::make_shared<TestListener>();
//	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
//
//	EXPECT_FALSE(device->removeOutputProducer(producer));
//	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
//
//	EXPECT_TRUE(device->setOutputProducer(producer));
//
//	// Sleep for a second, to see how many times the producer is invoked.
//	// (A TrackIR device is supposed to run at 1KHz.)
//	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));
//
//	EXPECT_TRUE(device->removeOutputProducer(producer));
//
//	// Removing the same input producer again should fail.
//	EXPECT_FALSE(device->removeOutputProducer(producer));
//
//	// Check the number of invocations.
//	EXPECT_GE(producer->m_numTimesRequestedOutput, 700);
//	EXPECT_LE(producer->m_numTimesRequestedOutput, 1300);
//}

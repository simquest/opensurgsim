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
/// Tests for the ReplayPoseDevice class.

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <boost/thread/thread.hpp> // This is to access boost::this_thread namespace

#include "SurgSim/Devices/ReplayPoseDevice/ReplayPoseDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Devices::ReplayPoseDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Testing::MockInputOutput;

namespace
{
void createFakeRecord(const std::string& fileName, double rate)
{
	SurgSim::Framework::Timer timer;
	timer.setMaxNumberOfFrames(1);
	timer.start();

	std::ofstream f(fileName, std::ios::out | std::ios::trunc);

	timer.endFrame();
	double cumulativeTime = timer.getLastFramePeriod();
	double deltaTime = 1.0 / rate;

	SurgSim::Math::Quaterniond q(Eigen::AngleAxisd(2.0123, SurgSim::Math::Vector3d(1, 2, 3).normalized()));
	SurgSim::Math::Vector3d t(0.01, -0.04, 0.0035);
	SurgSim::Math::RigidTransform3d poseStart = SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d::Zero());
	SurgSim::Math::RigidTransform3d poseEnd = SurgSim::Math::makeRigidTransform(q, t);

	do
	{
		auto pose = SurgSim::Math::interpolate(poseStart, poseEnd, cumulativeTime);
		f << cumulativeTime << std::endl << pose.matrix() << std::endl;
		cumulativeTime += deltaTime;
	} while (cumulativeTime < 1.0);
	f << "1.0" << std::endl << poseEnd.matrix() << std::endl;

	f.close();
}

bool exist(const std::string& fileName)
{
	std::ifstream f(fileName.c_str());
	bool result = f.good();
	f.close();
	return result;
}

void clearFakeRecord(const std::string& fileName)
{
	if (exist(fileName))
	{
		std::remove(fileName.c_str());
	}
}
}

TEST(ReplayPoseDeviceTest, Name)
{
	auto device = std::make_shared<ReplayPoseDevice>("FakeReplayDevice");
	EXPECT_EQ("FakeReplayDevice", device->getName());
}

TEST(ReplayPoseDeviceTest, Filename)
{
	std::string fileName("FakeRecord.txt");
	createFakeRecord(fileName, 30);

	auto device = std::make_shared<ReplayPoseDevice>("FakeReplayDevice");
	EXPECT_EQ(0, device->getFileName().compare("ReplayPoseDevice.txt"));
	EXPECT_NO_THROW(device->setFileName(fileName));
	EXPECT_EQ(0, device->getFileName().compare(fileName));

	EXPECT_TRUE(device->initialize());

	clearFakeRecord(fileName);
	EXPECT_THROW(device->setFileName(fileName), SurgSim::Framework::AssertionFailure);
}

TEST(ReplayPoseDeviceTest, Initialize)
{
	std::string fileName("FakeRecord.txt");
	clearFakeRecord(fileName);

	{
		SCOPED_TRACE("Missing filename");
		auto device = std::make_shared<ReplayPoseDevice>("FakeReplayDevice");
		device->setFileName(fileName);
		EXPECT_FALSE(device->initialize()); // Missing the setFilename call, no file open
		EXPECT_FALSE(device->isInitialized());
	}
	{
		SCOPED_TRACE("Success");
		createFakeRecord(fileName, 30);

		auto device = std::make_shared<ReplayPoseDevice>("FakeReplayDevice");
		device->setFileName(fileName);
		EXPECT_TRUE(device->initialize());
		EXPECT_TRUE(device->isInitialized());

		clearFakeRecord(fileName);
	}
}

TEST(ReplayPoseDeviceTest, Factory)
{
	std::shared_ptr<SurgSim::Input::DeviceInterface> device;
	ASSERT_NO_THROW(device = SurgSim::Input::DeviceInterface::getFactory().create(
		"SurgSim::Devices::ReplayPoseDevice", "Device"));
	EXPECT_NE(nullptr, device);
}

namespace
{
void AddInputConsumerRecordXHzReplayYHz(double rateRecord, double rateReplay)
{
	std::string fileName("FakeRecord.txt");

	createFakeRecord(fileName, rateRecord);

	std::shared_ptr<ReplayPoseDevice> device = std::make_shared<ReplayPoseDevice>("MyReplayPoseDevice");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	device->setRate(rateReplay);
	device->setFileName("FakeRecord.txt");
	ASSERT_TRUE(device->initialize()) << "Initialization failed.";

	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	// Check the number of invocations.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	EXPECT_GE(consumer->m_numTimesReceivedInput, rateReplay * 0.8); // rateReplay@ 1000Hz => lower limit@ 800Hz
	EXPECT_LE(consumer->m_numTimesReceivedInput, rateReplay * 1.2); // rateReplay@ 1000Hz => higher limit@ 1200Hz

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData(SurgSim::DataStructures::Names::POSE));

	// Check the replay pose passed 1.0 second
	SurgSim::Math::RigidTransform3d pose;
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(50));
	ASSERT_TRUE(consumer->m_lastReceivedInput.poses().get(SurgSim::DataStructures::Names::POSE, &pose));
	SurgSim::Math::Quaterniond q(Eigen::AngleAxisd(2.0123, SurgSim::Math::Vector3d(1, 2, 3).normalized()));
	SurgSim::Math::Vector3d t(0.01, -0.04, 0.0035);
	SurgSim::Math::RigidTransform3d poseEnd = SurgSim::Math::makeRigidTransform(q, t);
	EXPECT_NEAR(0, (pose.matrix() - poseEnd.matrix()).norm(), 1e-6);

	EXPECT_TRUE(device->removeInputConsumer(consumer));
	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	clearFakeRecord(fileName);
}
}

TEST(ReplayPoseDeviceTest, AddInputConsumerRecord60HzReplay60Hz)
{
	AddInputConsumerRecordXHzReplayYHz(60.0, 60.0);
}

TEST(ReplayPoseDeviceTest, AddInputConsumerRecord60HzReplay1000Hz)
{
	AddInputConsumerRecordXHzReplayYHz(60.0, 1000.0);
}

TEST(ReplayPoseDeviceTest, AddInputConsumerRecord1000HzReplay60Hz)
{
	AddInputConsumerRecordXHzReplayYHz(1000.0, 60.0);
}

TEST(ReplayPoseDeviceTest, AddInputConsumerRecord1000HzReplay1000Hz)
{
	AddInputConsumerRecordXHzReplayYHz(1000.0, 1000.0);
}

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
/// Tests for the VelocityFromPose class.

#include <boost/thread.hpp>
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/VelocityFromPose.h"
#include "SurgSim/Framework/Clock.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Device::VelocityFromPose;
using SurgSim::Input::CommonDevice;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Testing::MockInputOutput;

namespace
{
const double ERROR_EPSILON = 1e-7;

/// Exposes protected members of CommonDevice.
class MockVelocityFromPose : public VelocityFromPose
{
public:
	explicit MockVelocityFromPose(const std::string& name) : VelocityFromPose(name)
	{
	}

	SurgSim::DataStructures::DataGroup& doGetInputData()
	{
		return getInputData();
	}
};
};

TEST(VelocityFromPoseDeviceFilterTest, InputDataFilter)
{
	auto filter = std::make_shared<MockVelocityFromPose>("VelocityFromPoseFilter");
	ASSERT_TRUE(filter->initialize());

	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addBoolean("extraData");

	DataGroup data = builder.createData();
	const double initialRotationAngle = 0.1;
	const Vector3d rotationAxis(0.3, -0.2, 0.4);
	Vector3d initialTranslation(2.0, 3.0, 4.0);
	const RigidTransform3d initialPose =
		makeRigidTransform(makeRotationQuaternion(initialRotationAngle, rotationAxis), initialTranslation);
	data.poses().set(SurgSim::DataStructures::Names::POSE, initialPose);
	const bool expectedBool = true;
	data.booleans().set("extraData", expectedBool);

	// Normally the input device's initial input data would be set by the constructor or scaffold, then
	// initializeInput would be called in addInputConsumer.
	filter->initializeInput("device", data);
	const SurgSim::Framework::Clock::time_point start = SurgSim::Framework::Clock::now();
	DataGroup actualInputData = filter->doGetInputData();

	// Test that the velocity entries were added
	EXPECT_TRUE(actualInputData.vectors().hasEntry(SurgSim::DataStructures::Names::LINEAR_VELOCITY));
	EXPECT_TRUE(actualInputData.vectors().hasEntry(SurgSim::DataStructures::Names::ANGULAR_VELOCITY));

	// After initialization, before the first handleInput, the data should be the same except for the added velocity
	// entries.
	RigidTransform3d actualPose;
	ASSERT_TRUE(actualInputData.poses().get(SurgSim::DataStructures::Names::POSE, &actualPose));
	EXPECT_TRUE(actualPose.isApprox(initialPose, ERROR_EPSILON));

	bool actualBoolean;
	ASSERT_TRUE(actualInputData.booleans().get("extraData", &actualBoolean));
	EXPECT_EQ(expectedBool, actualBoolean);


	// Now test integration and velocity calculation.
	// Normally the input device would PushInput, which would call the filter's handleInput.
	const double rotationVelocity = 0.4;
	const Vector3d translationVelocity(0.0, -0.36, 0.5);
	const int milliseconds = 10;

	for (int i = 1; i < 11; ++i)
	{
		boost::this_thread::sleep(boost::posix_time::millisec(milliseconds));
		const boost::chrono::duration<double> duration = SurgSim::Framework::Clock::now() - start;
		const double totalTime = duration.count();
		data.poses().set(SurgSim::DataStructures::Names::POSE,
			makeRigidTransform(makeRotationQuaternion(initialRotationAngle + totalTime * rotationVelocity,
			rotationAxis),
			initialTranslation + totalTime * translationVelocity));
		filter->handleInput("device", data);
	}

	const DataGroup filteredInputData = filter->doGetInputData();

	// The "pose" data should be the same as the last value passed to the filter.
	RigidTransform3d expectedPose;
	ASSERT_TRUE(data.poses().get(SurgSim::DataStructures::Names::POSE, &expectedPose));
	ASSERT_TRUE(filteredInputData.poses().get(SurgSim::DataStructures::Names::POSE, &actualPose));

	// The velocity data should approximate the truth.
	Vector3d actualLinearVelocity;
	ASSERT_TRUE(filteredInputData.vectors().get(SurgSim::DataStructures::Names::LINEAR_VELOCITY,
		&actualLinearVelocity));
	EXPECT_NEAR(translationVelocity[0], actualLinearVelocity[0], 1e-3);
	EXPECT_NEAR(translationVelocity[1], actualLinearVelocity[1], 1e-3);
	EXPECT_NEAR(translationVelocity[2], actualLinearVelocity[2], 1e-3);

	Vector3d actualAngularVelocity;
	ASSERT_TRUE(filteredInputData.vectors().get(SurgSim::DataStructures::Names::ANGULAR_VELOCITY,
		&actualAngularVelocity));
	const Vector3d expectedAngularVelocity = rotationVelocity * rotationAxis;
	EXPECT_NEAR(expectedAngularVelocity[0], actualAngularVelocity[0], 2e-3);
	EXPECT_NEAR(expectedAngularVelocity[1], actualAngularVelocity[1], 2e-3);
	EXPECT_NEAR(expectedAngularVelocity[2], actualAngularVelocity[2], 2e-3);
}

TEST(VelocityFromPoseDeviceFilterTest, OutputDataFilter)
{
	auto filter = std::make_shared<MockVelocityFromPose>("VelocityFromPoseFilter");
	ASSERT_TRUE(filter->initialize());

	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addBoolean("extraData");

	DataGroup data = builder.createData();
	const double initialRotationAngle = 0.1;
	const Vector3d rotationAxis = Vector3d::UnitX();
	const Vector3d initialTranslation(2.0, 3.0, 4.0);
	const RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(initialRotationAngle, rotationAxis),
		initialTranslation);
	data.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	data.booleans().set("extraData", true);

	// Normally the data would be set by a behavior, then the output device scaffold would call requestOutput on the
	// filter, which would call requestOutput on the OutputComponent.
	auto producer = std::make_shared<MockInputOutput>();
	producer->m_output.setValue(data);
	filter->setOutputProducer(producer);

	DataGroup actualData;
	filter->requestOutput("device", &actualData);

	// The VelocityFromPose should not alter output data.
	RigidTransform3d actualPose;
	ASSERT_TRUE(actualData.poses().get(SurgSim::DataStructures::Names::POSE, &actualPose));
	EXPECT_TRUE(actualPose.isApprox(pose, ERROR_EPSILON));

	bool actualBoolean;
	ASSERT_TRUE(actualData.booleans().get("extraData", &actualBoolean));
	const bool expectedBoolean = true;
	EXPECT_EQ(expectedBoolean, actualBoolean);
}

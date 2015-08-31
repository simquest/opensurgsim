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
/// Tests for the PoseIntegrator class.

#include <boost/thread.hpp>
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/PoseIntegrator.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Devices::PoseIntegrator;
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
class MockPoseIntegrator : public PoseIntegrator
{
public:
	explicit MockPoseIntegrator(const std::string& name) : PoseIntegrator(name)
	{
	}

	SurgSim::DataStructures::DataGroup& doGetInputData()
	{
		return getInputData();
	}
};

void TestInputDataGroup(const DataGroup& actualData, const DataGroup& expectedData)
{
	RigidTransform3d actualPose;
	ASSERT_TRUE(actualData.poses().get(SurgSim::DataStructures::Names::POSE, &actualPose));
	RigidTransform3d expectedPose;
	ASSERT_TRUE(expectedData.poses().get(SurgSim::DataStructures::Names::POSE, &expectedPose));
	EXPECT_TRUE(actualPose.isApprox(expectedPose, ERROR_EPSILON));

	Vector3d actualLinearVelocity;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::LINEAR_VELOCITY, &actualLinearVelocity));
	Vector3d expectedLinearVelocity;
	ASSERT_TRUE(expectedData.vectors().get(SurgSim::DataStructures::Names::LINEAR_VELOCITY,
		&expectedLinearVelocity));
	EXPECT_TRUE(actualLinearVelocity.isApprox(expectedLinearVelocity, ERROR_EPSILON));

	Vector3d actualAngularVelocity;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, &actualAngularVelocity));
	Vector3d expectedAngularVelocity;
	ASSERT_TRUE(expectedData.vectors().get(SurgSim::DataStructures::Names::ANGULAR_VELOCITY,
		&expectedAngularVelocity));
	EXPECT_TRUE(actualAngularVelocity.isApprox(expectedAngularVelocity, ERROR_EPSILON));

	bool actualBoolean;
	ASSERT_TRUE(actualData.booleans().get("extraData", &actualBoolean));
	bool expectedBoolean;
	ASSERT_TRUE(expectedData.booleans().get("extraData", &expectedBoolean));
	EXPECT_EQ(expectedBoolean, actualBoolean);
}
};

// If the Device being filtered does not provide the linear and/or angular velocity vectors in its DataGroup, the
// PoseIntegrator adds them.
TEST(PoseIntegratorDeviceFilterTest, AddVelocityDataEntries)
{
	auto integrator = std::make_shared<MockPoseIntegrator>("PoseIntegratorFilter");
	ASSERT_TRUE(integrator->initialize());

	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addVector("someVector");
	builder.addBoolean("extraData");

	DataGroup data = builder.createData();
	const double rotationAngle = 0.1;
	const Vector3d rotationAxis = Vector3d::UnitX();
	const Vector3d translation(2.0, 3.0, 4.0);
	const RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(rotationAngle, rotationAxis), translation);
	data.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	const Vector3d expectedVector(24.0, -3.2, 0.0);
	data.vectors().set("someVector", expectedVector);
	const bool expectedBoolean = true;
	data.booleans().set("extraData", expectedBoolean);

	{	// Test the code-path in initializeInput when the filter has to add entries.
		// Normally the input device's initial input data would be set by the constructor or scaffold, then
		// initializeInput would be called in addInputConsumer.
		integrator->initializeInput("device", data);
		const DataGroup actualInputData = integrator->doGetInputData();

		// Test that the velocity entries were added
		EXPECT_TRUE(actualInputData.vectors().hasEntry(SurgSim::DataStructures::Names::LINEAR_VELOCITY));
		EXPECT_TRUE(actualInputData.vectors().hasEntry(SurgSim::DataStructures::Names::ANGULAR_VELOCITY));

		RigidTransform3d actualPose;
		ASSERT_TRUE(actualInputData.poses().get(SurgSim::DataStructures::Names::POSE, &actualPose));
		EXPECT_TRUE(actualPose.isApprox(pose, ERROR_EPSILON));

		Vector3d actualVector;
		ASSERT_TRUE(actualInputData.vectors().get("someVector", &actualVector));
		EXPECT_TRUE(actualVector.isApprox(expectedVector, ERROR_EPSILON));

		bool actualBoolean;
		ASSERT_TRUE(actualInputData.booleans().get("extraData", &actualBoolean));
		EXPECT_EQ(expectedBoolean, actualBoolean);
	}

	const RigidTransform3d newPose = pose.inverse();
	data.poses().set(SurgSim::DataStructures::Names::POSE, newPose);
	const Vector3d newVector = Vector3d(-0.77, 3.9, 99.0);
	data.vectors().set("someVector", newVector);
	const bool newBoolean = !expectedBoolean;
	data.booleans().set("extraData", newBoolean);

	{   // Test that the code-path through handleInput if the filter had to add entries...did it correctly pass data.
		// The InputDataFilter test checks for correctness of the pose and velocity entries.
		integrator->handleInput("device", data);
		const DataGroup actualInputData = integrator->doGetInputData();

		Vector3d actualVector;
		ASSERT_TRUE(actualInputData.vectors().get("someVector", &actualVector));
		EXPECT_TRUE(actualVector.isApprox(newVector, ERROR_EPSILON));

		bool actualBoolean;
		ASSERT_TRUE(actualInputData.booleans().get("extraData", &actualBoolean));
		EXPECT_EQ(newBoolean, actualBoolean);
	}
};

TEST(PoseIntegratorDeviceFilterTest, InputDataFilter)
{
	auto integrator = std::make_shared<MockPoseIntegrator>("PoseIntegratorFilter");
	EXPECT_NO_THROW(integrator->setReset(SurgSim::DataStructures::Names::BUTTON_1));
	ASSERT_TRUE(integrator->initialize());

	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
	builder.addVector(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
	builder.addBoolean("extraData");
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_1);

	DataGroup data = builder.createData();
	const double rotationAngle = 0.1;
	const Vector3d rotationAxis = Vector3d::UnitX();
	const Vector3d translation(2.0, 3.0, 4.0);
	const RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(rotationAngle, rotationAxis), translation);
	data.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	data.vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, Vector3d(5.0, 6.0, 7.0));
	data.vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, Vector3d(8.0, 9.0, 10.0));
	data.booleans().set("extraData", true);

	// Normally the input device's initial input data would be set by the constructor or scaffold, then
	// initializeInput would be called in addInputConsumer.
	integrator->initializeInput("device", data);

	EXPECT_ANY_THROW(integrator->setReset(SurgSim::DataStructures::Names::BUTTON_2));

	// After initialization, before the first handleInput, the initial and current input data should be the same.
	// There is no DataGroup::operator==, so we just test both DataGroups.
	{
		SCOPED_TRACE("Testing Initial Input Data.");
		// Normally the InputComponent (or another device filter) would have its handleInput called with the
		// PoseIntegrator's input data.
		DataGroup actualInputData = integrator->doGetInputData();
		TestInputDataGroup(actualInputData, data);
	}

	{
		SCOPED_TRACE("Testing Input Data, before first HandleInput.");
		DataGroup actualInputData = integrator->doGetInputData();
		TestInputDataGroup(actualInputData, data);
	}

	// Now test integration and velocity calculation.
	// Normally the input device would PushInput, which would call the filter's handleInput.
	boost::this_thread::sleep(boost::posix_time::millisec(100));
	integrator->handleInput("device", data);

	DataGroup expectedData = builder.createData();

	// The "pose" data should have incremented its angle of rotation and its translation.
	RigidTransform3d expectedPose = makeRigidTransform(makeRotationQuaternion(2.0 * rotationAngle, rotationAxis),
		2.0 * translation);
	expectedData.poses().set(SurgSim::DataStructures::Names::POSE, expectedPose);

	// The linearVelocity should be the translation (as a delta) times the rate.  We don't know the rate, so we'll
	// back-calculate it from the linear velocity...then we'll make sure the same rate was used for both linear and
	// angular velocities.
	const DataGroup actualTransformedInputData = integrator->doGetInputData();
	Vector3d actualLinearVelocity;
	ASSERT_TRUE(actualTransformedInputData.vectors().get(SurgSim::DataStructures::Names::LINEAR_VELOCITY,
		&actualLinearVelocity));
	const double rate = actualLinearVelocity.norm() / translation.norm();

	Vector3d actualAngularVelocity;
	ASSERT_TRUE(actualTransformedInputData.vectors().get(SurgSim::DataStructures::Names::ANGULAR_VELOCITY,
		&actualAngularVelocity));
	ASSERT_NEAR(rate, actualAngularVelocity.norm() / rotationAngle, ERROR_EPSILON);

	Vector3d expectedLinearVelocity = translation * rate;
	expectedData.vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, expectedLinearVelocity);

	// The angularVelocity should be the angleAxis (as a delta) times the rate.
	Vector3d expectedAngularVelocity = rotationAxis * rotationAngle * rate;
	expectedData.vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, expectedAngularVelocity);

	expectedData.booleans().set("extraData", true);

	{
		SCOPED_TRACE("Testing Input Data, after HandleInput.");
		TestInputDataGroup(actualTransformedInputData, expectedData);
	}

	// Reset the pose
	data.booleans().set(SurgSim::DataStructures::Names::BUTTON_1, true);
	integrator->handleInput("device", data);
	const DataGroup resetInputData = integrator->doGetInputData();
	RigidTransform3d actualPose;
	ASSERT_TRUE(resetInputData.poses().get(SurgSim::DataStructures::Names::POSE, &actualPose));
	EXPECT_TRUE(actualPose.isApprox(RigidTransform3d::Identity(), ERROR_EPSILON));
}

TEST(PoseIntegratorDeviceFilterTest, OutputDataFilter)
{
	auto integrator = std::make_shared<MockPoseIntegrator>("PoseIntegratorFilter");
	ASSERT_TRUE(integrator->initialize());

	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addBoolean("extraData");

	DataGroup data = builder.createData();
	const double rotationAngle = 0.1;
	const Vector3d rotationAxis = Vector3d::UnitX();
	const Vector3d translation(2.0, 3.0, 4.0);
	const RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(rotationAngle, rotationAxis), translation);
	data.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	data.booleans().set("extraData", true);

	// Normally the data would be set by a behavior, then the output device scaffold would call requestOutput on the
	// filter, which would call requestOutput on the OutputComponent.
	auto producer = std::make_shared<MockInputOutput>();
	producer->m_output.setValue(data);
	integrator->setOutputProducer(producer);

	DataGroup actualData;
	integrator->requestOutput("device", &actualData);

	// The PoseIntegrator should not alter output data.
	RigidTransform3d actualPose;
	ASSERT_TRUE(actualData.poses().get(SurgSim::DataStructures::Names::POSE, &actualPose));
	EXPECT_TRUE(actualPose.isApprox(pose, ERROR_EPSILON));

	bool actualBoolean;
	ASSERT_TRUE(actualData.booleans().get("extraData", &actualBoolean));
	const bool expectedBoolean = true;
	EXPECT_EQ(expectedBoolean, actualBoolean);
}

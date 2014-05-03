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

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/PoseIntegrator.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Device::PoseIntegrator;
using SurgSim::Input::CommonDevice;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

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

	SurgSim::DataStructures::DataGroup& doGetInitialInputData()
	{
		return getInitialInputData();
	}

	SurgSim::DataStructures::DataGroup& doGetInputData()
	{
		return getInputData();
	}
};

class TestOutputProducerInterface : public OutputProducerInterface
{
public:
	TestOutputProducerInterface()
	{
	}

	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override
	{
		*outputData = m_data;
		return true;
	}

	DataGroup m_data;
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

TEST(PoseIntegratorDeviceFilterTest, InputDataFilter)
{
	auto integrator = std::make_shared<MockPoseIntegrator>("PoseIntegratorFilter");
	ASSERT_TRUE(integrator->initialize());

	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
	builder.addVector(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
	builder.addBoolean("extraData");

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

	// handleInput should not change the initial input data.
	{
		SCOPED_TRACE("Testing Initial Input Data, after HandleInput, expecting no change.");
		const DataGroup actualInitialInputData = integrator->doGetInitialInputData();
		TestInputDataGroup(actualInitialInputData, data);
	}
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
	auto testOutputProducer = std::make_shared<TestOutputProducerInterface>();
	testOutputProducer->m_data = data;
	integrator->setOutputProducer(testOutputProducer);

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

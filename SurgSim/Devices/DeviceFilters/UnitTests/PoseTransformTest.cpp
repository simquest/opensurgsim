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
/// Tests for the PoseTransform class.

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Devices.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Devices::PoseTransform;
using SurgSim::Input::CommonDevice;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Math::Matrix66d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Testing::MockInputOutput;

namespace
{
const double ERROR_EPSILON = 1e-7;
}

/// Exposes protected members of CommonDevice.
class MockPoseTransform : public PoseTransform
{
public:
	explicit MockPoseTransform(const std::string& name) : PoseTransform(name)
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

TEST(PoseTransformDeviceFilterTest, InputDataFilter)
{
	auto poseTransformer = std::make_shared<MockPoseTransform>("PoseTransformFilter");
	ASSERT_TRUE(poseTransformer->initialize());

	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
	builder.addVector(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
	builder.addBoolean("extraData");

	DataGroup data = builder.createData();
	RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(1.5, Vector3d(1.2, 5.6, 0.7).normalized()),
		Vector3d(2.0, 3.0, 4.0));
	data.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	data.vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, Vector3d(5.0, 6.0, 7.0));
	data.vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, Vector3d(8.0, 9.0, 10.0));
	data.booleans().set("extraData", true);

	// Normally the input device's initial input data would be set by the constructor or scaffold, then
	// initializeInput would be called in addInputConsumer.
	poseTransformer->initializeInput("device", data);

	// After initialization, before the first handleInput, the input data should be unchanged.
	{
		SCOPED_TRACE("Testing Input Data, no transform or scaling.");
		// Normally the InputComponent (or another device filter) would have its handleInput called with the
		// PoseTransform's input data.
		DataGroup actualInputData = poseTransformer->doGetInputData();
		TestInputDataGroup(actualInputData, data);
	}

	// Now test setting transform and scaling.
	RigidTransform3d transform = makeRigidTransform(makeRotationQuaternion(0.9, Vector3d(10.8, -7.6, 5.4)).normalized(),
		Vector3d(18.3, -12.6, 1.0));
	poseTransformer->setTransform(transform);
	poseTransformer->setTranslationScale(3.7);

	// Normally the input device would PushInput, which would call the filter's handleInput.
	poseTransformer->handleInput("device", data);

	DataGroup expectedData = builder.createData();

	// The "pose" data should have its translation scaled and be pre-transformed.
	RigidTransform3d expectedPose = makeRigidTransform(makeRotationQuaternion(2.4972022984476547,
		Vector3d(0.29211414245268102, -0.31582037986877071, 0.90273297017372756)),
		Vector3d(15.6146599514, -31.1502325138, -5.75927677405));

	expectedData.poses().set(SurgSim::DataStructures::Names::POSE, expectedPose);

	// The linearVelocity should be scaled and rotated.
	Vector3d expectedLinearVelocity(-6.28155746782, -37.3676130859, -8.37278496308);
	expectedData.vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, expectedLinearVelocity);

	// The angularVelocity should be rotated.
	Vector3d expectedAngularVelocity(-2.66966888839, -15.1851334211, -2.69899814922);
	expectedData.vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, expectedAngularVelocity);

	expectedData.booleans().set("extraData", true);

	{
		SCOPED_TRACE("Testing Input Data, with transform or scaling.");
		DataGroup actualTransformedInputData = poseTransformer->doGetInputData();
		TestInputDataGroup(actualTransformedInputData, expectedData);
	}

	// A new initializeInput should run the new input data through the filter with the new parameters.
	poseTransformer->initializeInput("device", data);
	{
		SCOPED_TRACE("Testing Input Data after initializeInput, with transform or scaling.");
		DataGroup actualInputData = poseTransformer->doGetInputData();
		TestInputDataGroup(actualInputData, expectedData);
	}
}

TEST(PoseTransformDeviceFilterTest, OutputDataFilter)
{
	auto poseTransformer = std::make_shared<MockPoseTransform>("PoseTransformFilter");
	ASSERT_TRUE(poseTransformer->initialize());

	DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::FORCE);
	builder.addVector(SurgSim::DataStructures::Names::TORQUE);
	builder.addMatrix(SurgSim::DataStructures::Names::SPRING_JACOBIAN);
	builder.addPose(SurgSim::DataStructures::Names::INPUT_POSE);
	builder.addMatrix(SurgSim::DataStructures::Names::DAMPER_JACOBIAN);
	builder.addVector(SurgSim::DataStructures::Names::INPUT_LINEAR_VELOCITY);
	builder.addVector(SurgSim::DataStructures::Names::INPUT_ANGULAR_VELOCITY);
	builder.addBoolean("extraData");

	DataGroup data = builder.createData();
	data.vectors().set(SurgSim::DataStructures::Names::FORCE, Vector3d(-6.0, 8.0, -10.0));
	data.vectors().set(SurgSim::DataStructures::Names::TORQUE, Vector3d(8.0, -4.0, 2.0));

	Matrix66d springJacobian;
	springJacobian << 2.0, 4.0, 6.0, 8.0, 10.0, 12.0,
		14.0, 16.0, 18.0, 20.0, 22.0, 24.0,
		26.0, 28.0, 30.0, 32.0, 34.0, 36.0,
		-2.0, -4.0, -6.0, -8.0, -10.0, -12.0,
		-14.0, -16.0, -18.0, -20.0, -22.0, -24.0,
		-26.0, -28.0, -30.0, -32.0, -34.0, -36.0;
	data.matrices().set(SurgSim::DataStructures::Names::SPRING_JACOBIAN, springJacobian);

	RigidTransform3d inputPose = makeRigidTransform(makeRotationQuaternion(M_PI_2, Vector3d::UnitX().eval()),
		Vector3d(3., 5., 7.));
	data.poses().set(SurgSim::DataStructures::Names::INPUT_POSE, inputPose);

	Matrix66d damperJacobian;
	damperJacobian << 6.0, 10.0, 14.0, 18.0, 22.0, 26.0,
		30.0, 34.0, 38.0, 42.0, 46.0, 50.0,
		54.0, 58.0, 62.0, 66.0, 70.0, 74.0,
		-6.0, -10.0, -14.0, -18.0, -22.0, -26.0,
		-30.0, -34.0, -38.0, -42.0, -46.0, -50.0,
		-54.0, -58.0, -62.0, -66.0, -70.0, -74.0;
	data.matrices().set(SurgSim::DataStructures::Names::DAMPER_JACOBIAN, damperJacobian);

	data.vectors().set(SurgSim::DataStructures::Names::INPUT_LINEAR_VELOCITY, Vector3d(10.0, 6.0, 14.0));
	data.vectors().set(SurgSim::DataStructures::Names::INPUT_ANGULAR_VELOCITY, Vector3d(8.0, 9.0, 10.0));
	data.booleans().set("extraData", true);

	// Normally the data would be set by a behavior, then the output device scaffold would call requestOutput on the
	// filter, which would call requestOutput on the OutputComponent.
	auto producer = std::make_shared<MockInputOutput>();
	producer->m_output.setValue(data);
	poseTransformer->setOutputProducer(producer);

	RigidTransform3d transform = makeRigidTransform(makeRotationQuaternion(M_PI_2, Vector3d::UnitY().eval()),
		Vector3d(11.0, 12.0, 13.0));
	poseTransformer->setTransform(transform);
	poseTransformer->setTranslationScale(2.0);

	DataGroup actualData;
	poseTransformer->requestOutput("device", &actualData);

	// The force should be anti-rotated.
	Vector3d actualForce;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::FORCE, &actualForce));
	Vector3d expectedForce(10.0, 8.0, -6.0);
	EXPECT_TRUE(actualForce.isApprox(expectedForce, ERROR_EPSILON));

	// The torque should be anti-rotated.
	Vector3d actualtorque;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::TORQUE, &actualtorque));
	Vector3d expectedtorque(-2.0, -4.0, 8.0);
	EXPECT_TRUE(actualtorque.isApprox(expectedtorque, ERROR_EPSILON));

	// The springJacobian should be transformed into device space, so each 3x3 block should be left-multiplied by
	// the rotation, and right-multiplied by the anti-rotation, then the first three columns of the 6x6 should
	// be scaled by the translation scaling factor so that the forces & torques are the right magnitude for
	// the scene.
	SurgSim::DataStructures::DataGroup::DynamicMatrixType actualSpringJacobian;
	ASSERT_TRUE(actualData.matrices().get(SurgSim::DataStructures::Names::SPRING_JACOBIAN, &actualSpringJacobian));
	Matrix66d expectedSpringJacobian;
	expectedSpringJacobian << 60.0, -56.0, -52.0, 36.0, -34.0, -32.0,
		-36.0, 32.0, 28.0, -24.0, 22.0, 20.0,
		-12.0, 8.0, 4.0, -12.0, 10.0, 8.0,
		-60.0, 56.0, 52.0, -36.0, 34.0, 32.0,
		36.0, -32.0, -28.0, 24.0, -22.0, -20.0,
		12.0, -8.0, -4.0, 12.0, -10.0, -8.0;
	EXPECT_TRUE(actualSpringJacobian.isApprox(expectedSpringJacobian, ERROR_EPSILON));

	// The inputPose should be anti-transformed, and have its translation un-scaled.
	RigidTransform3d actualInputPose;
	ASSERT_TRUE(actualData.poses().get(SurgSim::DataStructures::Names::INPUT_POSE, &actualInputPose));
	RigidTransform3d expectedInputPose = makeRigidTransform(makeRotationQuaternion(-M_PI_2, Vector3d::UnitY().eval()) *
		makeRotationQuaternion(M_PI_2, Vector3d::UnitX().eval()),
		Vector3d(3.0, -3.5, -4.0));
	EXPECT_TRUE(actualInputPose.isApprox(expectedInputPose, ERROR_EPSILON));

	// The damperJacobian should be transformed the same as the springJacobian.
	SurgSim::DataStructures::DataGroup::DynamicMatrixType actualDamperJacobian;
	ASSERT_TRUE(actualData.matrices().get(SurgSim::DataStructures::Names::DAMPER_JACOBIAN, &actualDamperJacobian));
	Matrix66d expectedDamperJacobian;
	expectedDamperJacobian << 124.0, -116.0, -108.0, 74.0, -70.0, -66.0,
		-76.0, 68.0, 60.0, -50.0, 46.0, 42.0,
		-28.0, 20.0, 12.0, -26.0, 22.0, 18.0,
		-124.0, 116.0, 108.0, -74.0, 70.0, 66.0,
		76.0, -68.0, -60.0, 50.0, -46.0, -42.0,
		28.0, -20.0, -12.0, 26.0, -22.0, -18.0;
	EXPECT_TRUE(actualDamperJacobian.isApprox(expectedDamperJacobian, ERROR_EPSILON));

	// The inputLinearVelocity should be anti-rotated and un-scaled.
	Vector3d actualInputLinearVelocity;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::INPUT_LINEAR_VELOCITY,
		&actualInputLinearVelocity));
	Vector3d expectedInputLinearVelocity(-7.0, 3.0, 5.0);
	EXPECT_TRUE(actualInputLinearVelocity.isApprox(expectedInputLinearVelocity, ERROR_EPSILON));

	// The inputAngularVelocity should be anti-rotated.
	Vector3d actualInputAngularVelocity;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::INPUT_ANGULAR_VELOCITY,
		&actualInputAngularVelocity));
	Vector3d expectedInputAngularVelocity(-10.0, 9.0, 8.0);
	EXPECT_TRUE(actualInputAngularVelocity.isApprox(expectedInputAngularVelocity, ERROR_EPSILON));

	// Other data should pass through unchanged.
	bool actualBoolean;
	ASSERT_TRUE(actualData.booleans().get("extraData", &actualBoolean));
	bool expectedBoolean = true;
	EXPECT_EQ(expectedBoolean, actualBoolean);
}

TEST(PoseTransformDeviceFilterTest, Serialization)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	std::string fileName = "PoseTransform.yaml";
	std::shared_ptr<SurgSim::Input::DeviceInterface> device;
	ASSERT_NO_THROW(device = SurgSim::Devices::loadDevice(fileName));
	ASSERT_NE(nullptr, device);
	auto typedDevice = std::dynamic_pointer_cast<PoseTransform>(device);
	ASSERT_NE(nullptr, typedDevice);
	EXPECT_NEAR(3.4, typedDevice->getTranslationScale(), 1e-10);

	Eigen::AngleAxisd angleAxis;
	angleAxis.angle() = 12.3;
	angleAxis.axis() = SurgSim::Math::Vector3d(0.5, 0.5, 0.0);
	SurgSim::Math::Vector3d translation = SurgSim::Math::Vector3d(7.8, 8.9, 9.0);
	SurgSim::Math::RigidTransform3d expectedTransform =
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond(angleAxis), translation);
	EXPECT_TRUE(typedDevice->getTransform().isApprox(expectedTransform));
}
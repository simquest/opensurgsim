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
/// Tests for the forceScale class.

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/ForceScale.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Device::ForceScale;
using SurgSim::Input::CommonDevice;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Math::Matrix66d;
using SurgSim::Math::Vector3d;

const double ERROR_EPSILON = 1e-7;

/// Exposes protected members of CommonDevice.
class MockForceScale : public ForceScale
{
public:
	explicit MockForceScale(const std::string& name) : ForceScale(name)
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

TEST(ForceScaleDeviceFilterTest, InputDataFilter)
{
	auto forceScaler = std::make_shared<MockForceScale>("ForceScaleFilter");
	ASSERT_TRUE(forceScaler->initialize());

	DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);

	DataGroup data = builder.createData();
	data.vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, Vector3d(5.0, 6.0, 7.0));

	// Normally the input device's initial input data would be set by the constructor or scaffold, then
	// initializeInput would be called in addInputConsumer.
	forceScaler->initializeInput("device", data);

	// The ForceScale device filter should pass through the input data unchanged.
	DataGroup actualInitialInputData = forceScaler->doGetInitialInputData();
	Vector3d actualInitialLinearVelocity;
	ASSERT_TRUE(actualInitialInputData.vectors().get(SurgSim::DataStructures::Names::LINEAR_VELOCITY,
		&actualInitialLinearVelocity));
	EXPECT_TRUE(actualInitialLinearVelocity.isApprox(Vector3d(5.0, 6.0, 7.0), ERROR_EPSILON));

	DataGroup actualInputData = forceScaler->doGetInputData();
	Vector3d actualLinearVelocity;
	ASSERT_TRUE(actualInitialInputData.vectors().get(SurgSim::DataStructures::Names::LINEAR_VELOCITY,
		&actualLinearVelocity));
	EXPECT_TRUE(actualLinearVelocity.isApprox(Vector3d(5.0, 6.0, 7.0), ERROR_EPSILON));
}

TEST(ForceScaleDeviceFilterTest, OutputDataFilter)
{
	auto forceScaler = std::make_shared<MockForceScale>("ForceScaleFilter");
	ASSERT_TRUE(forceScaler->initialize());

	DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::FORCE);
	builder.addVector(SurgSim::DataStructures::Names::TORQUE);
	builder.addMatrix(SurgSim::DataStructures::Names::SPRING_JACOBIAN);
	builder.addMatrix(SurgSim::DataStructures::Names::DAMPER_JACOBIAN);
	builder.addBoolean("extraData");

	DataGroup data = builder.createData();
	data.vectors().set(SurgSim::DataStructures::Names::FORCE, Vector3d(1.0, 2.5, -13.8));
	data.vectors().set(SurgSim::DataStructures::Names::TORQUE, Vector3d(-7.0, 5.0, -1.2));

	Matrix66d springJacobian;
	springJacobian << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
		0.7, 0.8, 0.9, 1.0, 1.1, 1.2,
		1.3, 1.4, 1.5, 1.6, 1.7, 1.8,
		0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
		0.7, 0.8, 0.9, 1.0, 1.1, 1.2,
		1.3, 1.4, 1.5, 1.6, 1.7, 1.8;
	data.matrices().set(SurgSim::DataStructures::Names::SPRING_JACOBIAN, springJacobian);

	Matrix66d damperJacobian;
	damperJacobian << 0.91, 0.82, 0.73, 0.64, 0.55, 0.46,
		0.37, 0.28, 0.19, 11.0, 21.1, 31.2,
		41.3, 51.4, 61.5, 71.6, 81.7, 91.8,
		0.91, 0.82, 0.73, 0.64, 0.55, 0.46,
		0.37, 0.28, 0.19, 11.0, 21.1, 31.2,
		41.3, 51.4, 61.5, 71.6, 81.7, 91.8;
	data.matrices().set(SurgSim::DataStructures::Names::DAMPER_JACOBIAN, damperJacobian);

	data.booleans().set("extraData", true);

	// Normally the data would be set by a behavior, then the output device scaffold would call requestOutput on the
	// filter, which would call requestOutput on the OutputComponent.
	auto testOutputProducer = std::make_shared<TestOutputProducerInterface>();
	testOutputProducer->m_data = data;

	// The OutputProducer sends data out to the filter, which sends data out to the device.
	forceScaler->setForceScale(10.0);
	forceScaler->setTorqueScale(0.1);
	forceScaler->setOutputProducer(testOutputProducer);

	DataGroup actualData;
	forceScaler->requestOutput("device", &actualData);

	// Check the scaling.
	Vector3d actualForce;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::FORCE, &actualForce));
	Vector3d expectedForce;
	expectedForce << 10.0, 25.0, -138.0;
	EXPECT_TRUE(actualForce.isApprox(expectedForce, ERROR_EPSILON));

	Vector3d actualTorque;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::TORQUE, &actualTorque));
	Vector3d expectedTorque;
	expectedTorque << -0.7, 0.5, -0.12;
	EXPECT_TRUE(actualTorque.isApprox(expectedTorque, ERROR_EPSILON));

	SurgSim::DataStructures::DataGroup::DynamicMatrixType actualSpringJacobian;
	ASSERT_TRUE(actualData.matrices().get(SurgSim::DataStructures::Names::SPRING_JACOBIAN, &actualSpringJacobian));
	Matrix66d expectedSpringJacobian;
	expectedSpringJacobian << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
		7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
		13.0, 14.0, 15.0, 16.0, 17.0, 18.0,
		0.01, 0.02, 0.03, 0.04, 0.05, 0.06,
		0.07, 0.08, 0.09, 0.1, 0.11, 0.12,
		0.13, 0.14, 0.15, 0.16, 0.17, 0.18;
	EXPECT_TRUE(actualSpringJacobian.isApprox(expectedSpringJacobian, ERROR_EPSILON));

	SurgSim::DataStructures::DataGroup::DynamicMatrixType actualDamperJacobian;
	ASSERT_TRUE(actualData.matrices().get(SurgSim::DataStructures::Names::DAMPER_JACOBIAN, &actualDamperJacobian));
	Matrix66d expectedDamperJacobian;
	expectedDamperJacobian << 9.1, 8.2, 7.3, 6.4, 5.5, 4.6,
		3.7, 2.8, 1.9, 110.0, 211.0, 312.0,
		413.0, 514.0, 615.0, 716.0, 817.0, 918.0,
		0.091, 0.082, 0.073, 0.064, 0.055, 0.046,
		0.037, 0.028, 0.019, 1.10, 2.11, 3.12,
		4.13, 5.14, 6.15, 7.16, 8.17, 9.18;
	EXPECT_TRUE(actualDamperJacobian.isApprox(expectedDamperJacobian, ERROR_EPSILON));

	// Other data should pass through.
	bool actualBoolean;
	ASSERT_TRUE(actualData.booleans().get("extraData", &actualBoolean));
	EXPECT_EQ(actualBoolean, true);
}

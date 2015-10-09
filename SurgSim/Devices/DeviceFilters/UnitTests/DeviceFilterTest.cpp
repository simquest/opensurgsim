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
/// Tests for the DeviceFilter class.

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Devices::DeviceFilter;
using SurgSim::Testing::MockInputOutput;

/// Exposes protected members of CommonDevice.
class MockDeviceFilter : public DeviceFilter
{
public:
	explicit MockDeviceFilter(const std::string& name) : DeviceFilter(name)
	{
	}

	SurgSim::DataStructures::DataGroup& doGetInputData()
	{
		return getInputData();
	}
};

TEST(DeviceFilterTest, InputDataFilter)
{
	auto filter = std::make_shared<MockDeviceFilter>("filter");
	ASSERT_TRUE(filter->initialize());

	DataGroupBuilder builder;
	std::string scalarName = "scalar";
	builder.addScalar(scalarName);
	DataGroup data = builder.createData();
	const double initialScalar = 17.3;
	data.scalars().set(scalarName, initialScalar);

	// Normally the input device's initial input data would be set by the constructor or scaffold, then
	// initializeInput would be called in addInputConsumer.
	filter->initializeInput("device", data);

	DataGroup actualInputData = filter->doGetInputData();
	double actualScalar;
	ASSERT_TRUE(actualInputData.scalars().get(scalarName, &actualScalar));
	EXPECT_EQ(initialScalar, actualScalar);
}

TEST(DeviceFilterTest, OutputDataFilter)
{
	auto filter = std::make_shared<MockDeviceFilter>("filter");
	ASSERT_TRUE(filter->initialize());

	DataGroupBuilder builder;
	std::string scalarName = "scalar";
	builder.addScalar(scalarName);
	DataGroup data = builder.createData();
	const double initialScalar = 17.3;
	data.scalars().set(scalarName, initialScalar);

	// Normally the data would be set by a behavior, then the output device scaffold would call requestOutput on the
	// filter, which would call requestOutput on the OutputComponent.
	auto producer = std::make_shared<MockInputOutput>();
	producer->m_output.setValue(data);

	// The OutputProducer sends data out to the filter, which sends data out to the device.
	filter->setOutputProducer(producer);

	DataGroup actualData;
	ASSERT_TRUE(filter->requestOutput("device", &actualData));

	double actualScalar;
	ASSERT_TRUE(actualData.scalars().get(scalarName, &actualScalar));
	EXPECT_EQ(initialScalar, actualScalar);
}

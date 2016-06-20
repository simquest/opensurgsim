// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <boost/thread.hpp>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/BoolToScalar.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Testing/MockInputOutput.h"

namespace SurgSim
{

namespace Devices
{

using namespace DataStructures;

class BoolToScalarFilterTest : public testing::Test
{
public:

	void SetUp()
	{
		// values for default buttons
		builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_1);
		builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_2);
		builder.addScalar(SurgSim::DataStructures::Names::TOOLDOF);

		defaultData = builder.createData();
	}

	DataStructures::DataGroupBuilder builder;
	DataStructures::DataGroup defaultData;
};

TEST_F(BoolToScalarFilterTest, GeneralSetters)
{
	auto filter = std::make_shared<BoolToScalar>("Filter");

	filter->setValue("Scalar", 0.5);
	EXPECT_DOUBLE_EQ(0.5, filter->getValue<double>("Scalar"));

	filter->setValue("Scale", 4.0);
	EXPECT_DOUBLE_EQ(4.0, filter->getValue<double>("Scale"));

	filter->setValue("Clamping", false);
	EXPECT_EQ(false, filter->getValue<bool>("Clamping"));

	filter->setValue("IncreaseField", std::string("IncreaseField"));
	EXPECT_EQ("IncreaseField", filter->getValue<std::string>("IncreaseField"));
	filter->setValue("DecreaseField", "DecreaseField");
	EXPECT_EQ("DecreaseField", filter->getValue<std::string>("DecreaseField"));
	filter->setValue("TargetField", "TargetField");
	EXPECT_EQ("TargetField", filter->getValue<std::string>("TargetField"));
}

TEST_F(BoolToScalarFilterTest, SetRange)
{
	auto filter = std::make_shared<BoolToScalar>("Filter");
	auto expectedRange = std::make_pair(5.0, 6.0);
	filter->setValue("Range", expectedRange);
	auto resultRange = filter->getValue<std::pair<double, double>>("Range");
	EXPECT_DOUBLE_EQ(expectedRange.first, resultRange.first);
	EXPECT_DOUBLE_EQ(expectedRange.second, resultRange.second);

	// Reverse the range if it's [max, min]
	auto inputRange = std::make_pair(10.0, 1.0);
	expectedRange = std::make_pair(1.0, 10.0);
	filter->setRange(inputRange);
	resultRange = filter->getRange();
	EXPECT_DOUBLE_EQ(expectedRange.first, resultRange.first);
	EXPECT_DOUBLE_EQ(expectedRange.second, resultRange.second);
}

TEST_F(BoolToScalarFilterTest, DataGroupHandling)
{
	{
		SCOPED_TRACE("Valid Data");
		auto filter = std::make_shared<BoolToScalar>("Filter");
		EXPECT_NO_THROW(filter->initializeInput("device", defaultData));
	}
	{
		SCOPED_TRACE("Invalid IncreaseField");
		auto filter = std::make_shared<BoolToScalar>("Filter");
		filter->setIncreaseField("wrongIncrease");
		EXPECT_ANY_THROW(filter->initializeInput("device", defaultData));
	}
	{
		SCOPED_TRACE("Invalid DecreaseField");
		auto filter = std::make_shared<BoolToScalar>("Filter");
		filter->setDecreaseField("wrongDecrease");
		EXPECT_ANY_THROW(filter->initializeInput("device", defaultData));
	}
	{
		SCOPED_TRACE("Append Target");
		DataStructures::DataGroupBuilder builder;
		builder.addBoolean(Names::BUTTON_1);
		builder.addBoolean(Names::BUTTON_2);
		auto data = builder.createData();
		auto filter = std::make_shared<BoolToScalar>("Filter");
		EXPECT_NO_THROW(filter->initializeInput("device", data));
	}
}

TEST_F(BoolToScalarFilterTest, UpdateTest)
{
	auto filter = std::make_shared<BoolToScalar>("Filter");
	filter->setScale(2.0);
	auto resultData = defaultData;
	defaultData.booleans().set(Names::BUTTON_1, false);
	defaultData.booleans().set(Names::BUTTON_2, true);
	double value;
	filter->initializeInput("device", defaultData);

	// Running .25 of a second we expect the value to go up .. by 0.25 * 2 (scale)
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	filter->filterInput("device", defaultData, &resultData);
	resultData.scalars().get(Names::TOOLDOF, &value);
	EXPECT_NEAR(0.5, value, 0.1);

	// Switiching the buttons, running for .125 of a second so the value should go down by 0.125 * 2 (scale)
	defaultData.booleans().set(Names::BUTTON_1, true);
	defaultData.booleans().set(Names::BUTTON_2, false);
	// prevent accumulated error in the test
	filter->setScalar(0.5);
	boost::this_thread::sleep(boost::posix_time::milliseconds(125));
	filter->filterInput("device", defaultData, &resultData);
	resultData.scalars().get(Names::TOOLDOF, &value);
	EXPECT_NEAR(0.25, value, 0.1);
}

TEST_F(BoolToScalarFilterTest, ClampTest)
{
	auto filter = std::make_shared<BoolToScalar>("Filter");
	filter->setScale(10.0);
	auto resultData = defaultData;
	defaultData.booleans().set(Names::BUTTON_1, false);
	defaultData.booleans().set(Names::BUTTON_2, true);
	double value;
	filter->initializeInput("device", defaultData);

	// Running .25 of a second we expect the value to go up .. by 0.25 * 10 (scale)
	// but it's clamped to [0,1]
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	filter->filterInput("device", defaultData, &resultData);
	resultData.scalars().get(Names::TOOLDOF, &value);
	EXPECT_DOUBLE_EQ(1.0, value);

	// Switiching the buttons, running for .25 of a second so the value should go down by 0.25 * 10 (scale)
	// but it's clamped to [0,1]
	defaultData.booleans().set(Names::BUTTON_1, true);
	defaultData.booleans().set(Names::BUTTON_2, false);
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	filter->filterInput("device", defaultData, &resultData);
	resultData.scalars().get(Names::TOOLDOF, &value);
	EXPECT_DOUBLE_EQ(0.0, value);
}

TEST_F(BoolToScalarFilterTest, UnClampedTest)
{
	auto filter = std::make_shared<BoolToScalar>("Filter");
	filter->setScale(10.0);
	filter->setClamping(false);
	auto resultData = defaultData;
	defaultData.booleans().set(Names::BUTTON_1, false);
	defaultData.booleans().set(Names::BUTTON_2, true);
	double value;
	filter->initializeInput("device", defaultData);

	// Running .25 of a second we expect the value to go up .. by 0.25 * 10 (scale)
	// but it's not clamped
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	filter->filterInput("device", defaultData, &resultData);
	resultData.scalars().get(Names::TOOLDOF, &value);
	EXPECT_TRUE(value > 2.0);

	// Switiching the buttons, running for .25 of a second so the value should go down by 0.25 * 10 (scale)
	// but it's clamped to [0,]1
	filter->setScalar(0.5);
	defaultData.booleans().set(Names::BUTTON_1, true);
	defaultData.booleans().set(Names::BUTTON_2, false);
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	filter->filterInput("device", defaultData, &resultData);
	resultData.scalars().get(Names::TOOLDOF, &value);
	EXPECT_TRUE(value < 0.0);
}


}
}


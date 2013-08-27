// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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
/// Tests for the NamedVariantData class.


#include <SurgSim/Framework/Assert.h>
#include "SurgSim/DataStructures/NamedVariantData.h"
#include "MockObjects.h"
#include "gtest/gtest.h"

using SurgSim::DataStructures::NamedVariantData;
using SurgSim::DataStructures::NamedVariantDataBuilder;

/// Creating a named variant data object.
TEST(NamedVariantDataTests, CanConstruct)
{
	SurgSim::DataStructures::NamedVariantDataBuilder builder;
	builder.addEntry("test");
	SurgSim::DataStructures::NamedVariantData data = builder.createData();

	EXPECT_EQ(1, data.getNumEntries());
	EXPECT_EQ(1u, data.size());

	EXPECT_TRUE(data.isValid());
	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("test"));
	EXPECT_FALSE(data.hasData(0));
	EXPECT_FALSE(data.hasData("test"));
	EXPECT_EQ(0, data.getIndex("test"));
	EXPECT_EQ("test", data.getName(0));

	EXPECT_FALSE(data.hasEntry(1));
	EXPECT_FALSE(data.hasEntry("missing"));
	EXPECT_FALSE(data.hasData(1));
	EXPECT_FALSE(data.hasData("missing"));
	EXPECT_EQ(-1, data.getIndex("missing"));
	EXPECT_EQ("", data.getName(1));
}

TEST(NamedVariantDataTests, Set)
{
	SurgSim::DataStructures::NamedVariantDataBuilder builder;
	builder.addEntry("test");
	SurgSim::DataStructures::NamedVariantData data = builder.createData();

	Mock3DData<float> mockData(5, 5, 5);
	mockData.set(1, 1, 1, 1.23f);
	mockData.set(4, 3, 2, 4.56f);

	data.set("test", mockData);
	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("test"));
	EXPECT_TRUE(data.hasData(0));
	EXPECT_TRUE(data.hasData("test"));
}

TEST(NamedVariantDataTests, HasTypedData)
{
	SurgSim::DataStructures::NamedVariantDataBuilder builder;
	builder.addEntry("test");
	SurgSim::DataStructures::NamedVariantData data = builder.createData();

	EXPECT_FALSE(data.hasTypedData<double>(0));
	EXPECT_FALSE(data.hasTypedData<double>("test"));

	Mock3DData<double> mockData(5, 5, 5);
	mockData.set(1, 1, 1, 1.23);
	mockData.set(4, 3, 2, 4.56);
	data.set("test", mockData);
	EXPECT_TRUE(data.hasTypedData<Mock3DData<double> >(0));
	EXPECT_TRUE(data.hasTypedData<Mock3DData<double> >("test"));

	EXPECT_FALSE(data.hasTypedData<double>(0));
	EXPECT_FALSE(data.hasTypedData<double>("test"));
	EXPECT_FALSE(data.hasTypedData<double>(5));
	EXPECT_FALSE(data.hasTypedData<double>("missing"));
}

TEST(NamedVariantDataTests, Get)
{
	SurgSim::DataStructures::NamedVariantDataBuilder builder;
	builder.addEntry("test");
	SurgSim::DataStructures::NamedVariantData data = builder.createData();

	Mock3DData<float> mockData(5, 5, 5);
	mockData.set(1, 1, 1, 1.23f);
	mockData.set(4, 3, 2, 4.56f);

	data.set("test", mockData);
	{
		Mock3DData<float> value;
		EXPECT_TRUE(data.get("test", &value));
		EXPECT_EQ(1.23f, value.get(1, 1, 1));
		EXPECT_EQ(4.56f, value.get(4, 3, 2));
	}
	{
		Mock3DData<float> value;
		EXPECT_TRUE(data.get(0, &value));
		EXPECT_EQ(1.23f, value.get(1, 1, 1));
		EXPECT_EQ(4.56f, value.get(4, 3, 2));
	}
	{
		Mock3DData<float> value;
		EXPECT_FALSE(data.get(5, &value));
		EXPECT_FALSE(data.get("missing", &value));
	}
	{
		Mock3DData<unsigned char> wrongType;
		EXPECT_THROW(data.get("test", &wrongType), SurgSim::Framework::AssertionFailure);
	}
}

TEST(NamedVariantDataTests, Reset)
{
	SurgSim::DataStructures::NamedVariantDataBuilder builder;
	builder.addEntry("test");
	SurgSim::DataStructures::NamedVariantData data = builder.createData();

	Mock3DData<float> mockData(5, 5, 5);
	mockData.set(1, 1, 1, 1.23f);
	mockData.set(4, 3, 2, 4.56f);

	data.set("test", mockData);
	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("test"));
	EXPECT_TRUE(data.hasData(0));
	EXPECT_TRUE(data.hasData("test"));

	data.resetAll();
	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("test"));
	EXPECT_FALSE(data.hasData(0));
	EXPECT_FALSE(data.hasData("test"));
}


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

/** @file
 * Tests for the NamedData<T> class.
 */

#include "SurgSim/DataStructures/NamedData.h"
#include "SurgSim/DataStructures/NamedDataBuilder.h"
#include "gtest/gtest.h"

using SurgSim::DataStructures::NamedData;
using SurgSim::DataStructures::NamedDataBuilder;


/// Creating a named data object.
TEST(NamedDataTests, CanConstruct)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("test");
	NamedData<float> data = builder.createData();

	EXPECT_EQ(1, data.getNumEntries());
	EXPECT_EQ(1U, data.size());

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


/// Creating a shared_ptr to a named data object.
TEST(NamedDataTests, CanCreateShared)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("test");
	std::shared_ptr<NamedData<float>> data = builder.createSharedData();

	EXPECT_EQ(1, data->getNumEntries());
	EXPECT_EQ(1U, data->size());

	EXPECT_TRUE(data->isValid());
	EXPECT_TRUE(data->hasEntry(0));
	EXPECT_TRUE(data->hasEntry("test"));
	EXPECT_FALSE(data->hasData(0));
	EXPECT_FALSE(data->hasData("test"));
	EXPECT_EQ(0, data->getIndex("test"));
	EXPECT_EQ("test", data->getName(0));

	EXPECT_FALSE(data->hasEntry(1));
	EXPECT_FALSE(data->hasEntry("missing"));
	EXPECT_FALSE(data->hasData(1));
	EXPECT_FALSE(data->hasData("missing"));
	EXPECT_EQ(-1, data->getIndex("missing"));
	EXPECT_EQ("", data->getName(1));
}


/// Creating a named data object using a vector of names, without a builder.
TEST(NamedDataTests, CanConstructFromNames)
{
	std::vector<std::string> names;
	names.push_back("test");
	NamedData<float> data(names);

	EXPECT_EQ(1, data.getNumEntries());
	EXPECT_EQ(1U, data.size());

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

/// Run a few tests against an empty NamedData structure.
TEST(NamedDataTests, Empty)
{
	NamedDataBuilder<float> builder;

	EXPECT_EQ(0, builder.getNumEntries());
	EXPECT_EQ("", builder.getName(0));

	EXPECT_EQ(-1, builder.getIndex("missing"));
	EXPECT_FALSE(builder.hasEntry("missing"));

	NamedData<float> data = builder.createData();

	EXPECT_EQ(0, data.getNumEntries());
	EXPECT_EQ("", data.getName(0));

	EXPECT_EQ(-1, data.getIndex("missing"));
	EXPECT_FALSE(data.hasEntry("missing"));
}

/// Creating an unitialized data object.
TEST(NamedDataTests, Uninitialized)
{
	NamedData<float> data;
	EXPECT_FALSE(data.isValid());

	EXPECT_EQ(0, data.getNumEntries());
	EXPECT_EQ("", data.getName(0));

	EXPECT_EQ(-1, data.getIndex("missing"));
	EXPECT_FALSE(data.hasEntry("missing"));
	EXPECT_FALSE(data.hasData("missing"));

	float value = 9.87f;
	EXPECT_FALSE(data.get("missing", &value));
	EXPECT_NEAR(9.87f, value, 1e-9);  // i.e. unchanged

	EXPECT_FALSE(data.reset("missing"));

	EXPECT_FALSE(data.set("missing", value));
}

/// Putting data into the container.
TEST(NamedDataTests, Put)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	builder.addEntry("third");
	NamedData<float> data = builder.createData();

	data.set("first", 1.23f);
	data.set(1, 4.56f);

	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("first"));
	EXPECT_TRUE(data.hasData(0));
	EXPECT_TRUE(data.hasData("first"));
	EXPECT_EQ(0, data.getIndex("first"));
	EXPECT_EQ("first", data.getName(0));

	EXPECT_TRUE(data.hasEntry(1));
	EXPECT_TRUE(data.hasEntry("second"));
	EXPECT_TRUE(data.hasData(1));
	EXPECT_TRUE(data.hasData("second"));
	EXPECT_EQ(1, data.getIndex("second"));
	EXPECT_EQ("second", data.getName(1));

	EXPECT_TRUE(data.hasEntry(2));
	EXPECT_TRUE(data.hasEntry("third"));
	EXPECT_FALSE(data.hasData(2));
	EXPECT_FALSE(data.hasData("third"));
	EXPECT_EQ(2, data.getIndex("third"));
	EXPECT_EQ("third", data.getName(2));
}

/// Getting data into the container.
TEST(NamedDataTests, Get)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	builder.addEntry("third");
	NamedData<float> data = builder.createData();

	data.set("first", 1.23f);
	data.set(1, 4.56f);

	{
		float value = 9.87f;
		EXPECT_TRUE(data.get(0, &value));
		EXPECT_NEAR(1.23f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_TRUE(data.get("first", &value));
		EXPECT_NEAR(1.23f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_TRUE(data.get(1, &value));
		EXPECT_NEAR(4.56f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_TRUE(data.get("second", &value));
		EXPECT_NEAR(4.56f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_FALSE(data.get(2, &value));
		EXPECT_NEAR(9.87f, value, 1e-9);  // i.e. unchanged
	}
	{
		float value = 9.87f;
		EXPECT_FALSE(data.get("third", &value));
		EXPECT_NEAR(9.87f, value, 1e-9);  // i.e. unchanged
	}
}

/// Resetting the data in the container.
TEST(NamedDataTests, ResetAll)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	builder.addEntry("third");
	NamedData<float> data = builder.createData();

	data.set("first", 1.23f);
	data.set(1, 4.56f);

	data.resetAll();

	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("first"));
	EXPECT_FALSE(data.hasData(0));
	EXPECT_FALSE(data.hasData("first"));

	EXPECT_TRUE(data.hasEntry(1));
	EXPECT_TRUE(data.hasEntry("second"));
	EXPECT_FALSE(data.hasData(1));
	EXPECT_FALSE(data.hasData("second"));

	EXPECT_TRUE(data.hasEntry(2));
	EXPECT_TRUE(data.hasEntry("third"));
	EXPECT_FALSE(data.hasData(2));
	EXPECT_FALSE(data.hasData("third"));
}

/// Resetting one data entry at a time.
TEST(NamedDataTests, ResetOne)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	builder.addEntry("third");
	NamedData<float> data = builder.createData();

	data.set("first", 1.23f);
	data.set(1, 4.56f);

	data.reset(0);

	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("first"));
	EXPECT_FALSE(data.hasData(0));
	EXPECT_FALSE(data.hasData("first"));

	EXPECT_TRUE(data.hasEntry(1));
	EXPECT_TRUE(data.hasEntry("second"));
	EXPECT_TRUE(data.hasData(1));
	EXPECT_TRUE(data.hasData("second"));

	EXPECT_TRUE(data.hasEntry(2));
	EXPECT_TRUE(data.hasEntry("third"));
	EXPECT_FALSE(data.hasData(2));
	EXPECT_FALSE(data.hasData("third"));

	data.reset("second");

	EXPECT_TRUE(data.hasEntry(1));
	EXPECT_TRUE(data.hasEntry("second"));
	EXPECT_FALSE(data.hasData(1));
	EXPECT_FALSE(data.hasData("second"));
}

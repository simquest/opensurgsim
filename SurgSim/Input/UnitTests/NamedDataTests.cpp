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

#include "SurgSim/Input/NamedData.h"
#include "SurgSim/Input/NamedDataBuilder.h"
#include "gtest/gtest.h"

using SurgSim::Input::NamedData;
using SurgSim::Input::NamedDataBuilder;


/// Creating a named data object.
TEST(NamedDataTests, CanConstruct)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("test");
	NamedData<float> data = builder.createData();

	EXPECT_TRUE(data.isValid());
	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("test"));
	EXPECT_FALSE(data.hasCurrentData(0));
	EXPECT_FALSE(data.hasCurrentData("test"));

	EXPECT_FALSE(data.hasEntry(1));
	EXPECT_FALSE(data.hasEntry("missing"));
	EXPECT_FALSE(data.hasCurrentData(1));
	EXPECT_FALSE(data.hasCurrentData("missing"));
}


/// Creating a shared_ptr to a named data object.
TEST(NamedDataTests, CanCreateShared)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("test");
	std::shared_ptr<NamedData<float>> data = builder.createSharedData();

	EXPECT_TRUE(data->isValid());
	EXPECT_TRUE(data->hasEntry(0));
	EXPECT_TRUE(data->hasEntry("test"));
	EXPECT_FALSE(data->hasCurrentData(0));
	EXPECT_FALSE(data->hasCurrentData("test"));

	EXPECT_FALSE(data->hasEntry(1));
	EXPECT_FALSE(data->hasEntry("missing"));
	EXPECT_FALSE(data->hasCurrentData(1));
	EXPECT_FALSE(data->hasCurrentData("missing"));
}

/// Creating an unitialized data object.
TEST(NamedDataTests, Uninitialized)
{
	NamedData<float> data;
	EXPECT_FALSE(data.isValid());
}

/// Putting data into the container.
TEST(NamedDataTests, Put)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	builder.addEntry("third");
	NamedData<float> data = builder.createData();

	data.put("first", 1.23f);
	data.put(1, 4.56f);

	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("first"));
	EXPECT_TRUE(data.hasCurrentData(0));
	EXPECT_TRUE(data.hasCurrentData("first"));

	EXPECT_TRUE(data.hasEntry(1));
	EXPECT_TRUE(data.hasEntry("second"));
	EXPECT_TRUE(data.hasCurrentData(1));
	EXPECT_TRUE(data.hasCurrentData("second"));

	EXPECT_TRUE(data.hasEntry(2));
	EXPECT_TRUE(data.hasEntry("third"));
	EXPECT_FALSE(data.hasCurrentData(2));
	EXPECT_FALSE(data.hasCurrentData("third"));
}

/// Getting data into the container.
TEST(NamedDataTests, Get)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	builder.addEntry("third");
	NamedData<float> data = builder.createData();

	data.put("first", 1.23f);
	data.put(1, 4.56f);

	{
		float value = 9.87f;
		EXPECT_TRUE(data.get(0, value));
		EXPECT_NEAR(1.23f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_TRUE(data.get("first", value));
		EXPECT_NEAR(1.23f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_TRUE(data.get(1, value));
		EXPECT_NEAR(4.56f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_TRUE(data.get("second", value));
		EXPECT_NEAR(4.56f, value, 1e-9);
	}
	{
		float value = 9.87f;
		EXPECT_FALSE(data.get(2, value));
		EXPECT_NEAR(9.87f, value, 1e-9);  // i.e. unchanged
	}
	{
		float value = 9.87f;
		EXPECT_FALSE(data.get("third", value));
		EXPECT_NEAR(9.87f, value, 1e-9);  // i.e. unchanged
	}
}

/// Resetting the data in the container.
TEST(NamedDataTests, Reset)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	builder.addEntry("third");
	NamedData<float> data = builder.createData();

	data.put("first", 1.23f);
	data.put(1, 4.56f);

	data.reset();

	EXPECT_TRUE(data.hasEntry(0));
	EXPECT_TRUE(data.hasEntry("first"));
	EXPECT_FALSE(data.hasCurrentData(0));
	EXPECT_FALSE(data.hasCurrentData("first"));

	EXPECT_TRUE(data.hasEntry(1));
	EXPECT_TRUE(data.hasEntry("second"));
	EXPECT_FALSE(data.hasCurrentData(1));
	EXPECT_FALSE(data.hasCurrentData("second"));

	EXPECT_TRUE(data.hasEntry(2));
	EXPECT_TRUE(data.hasEntry("third"));
	EXPECT_FALSE(data.hasCurrentData(2));
	EXPECT_FALSE(data.hasCurrentData("third"));
}

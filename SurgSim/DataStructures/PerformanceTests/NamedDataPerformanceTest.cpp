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

#include <gtest/gtest.h>

#include <boost/exception/to_string.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

#include "SurgSim/DataStructures/NamedData.h"
#include "SurgSim/DataStructures/NamedDataBuilder.h"
#include "SurgSim/Framework/Timer.h"

namespace
{
const std::string array[] =
	{"uzgfk1f41V",
	"PbpZficBR2",
	"M3OYjZgHXX",
	"WVIEjG3QaX",
	"2o6nN2nuaW",
	"3rWXCXR2gi",
	"XO2dATxWFq",
	"M2xpjE6PAL",
	"cT1Bmj3Z1U",
	"65UtrrfXn7",
	"tNltVIcurd",
	"jFVkKzn17i",
	"0QlA0FAPc3",
	"jCXGBopngK",
	"GTwY4YyVnC",
	"0uVPK4S9Le",
	"iNdq3p6ZQb",
	"LfREPeFczN",
	"9RKdlFnm1I",
	"N2acPVhGY2"};

const std::vector<std::string> names(std::begin(array), std::end(array));

const int numberOfLoops = 100000;
};

namespace SurgSim
{
namespace DataStructures
{
	

class NamedDataTest : public ::testing::Test
{
public:

	virtual void SetUp()
	{
		NamedDataBuilder<int> builder;
		for (std::string name : names)
		{
			builder.addEntry(name);
		}

		data = builder.createData();

		for (int i = 0; i < names.size(); ++i)
		{
			ASSERT_TRUE(data.set(i, i));
			indices.push_back(i);
		}
	}

	virtual void TearDown()
	{
	}

	NamedData<int> data;
	std::vector<int> indices;
};

TEST_F(NamedDataTest, GetByName)
{
	int value;
	const int numberOfEntries = static_cast<int>(names.size());

	SurgSim::Framework::Timer timer;
	for (int i = 0; i < numberOfLoops; ++i)
	{
		for (int j = 0; j < numberOfEntries; ++j)
		{
			ASSERT_TRUE(data.get(names[j], &value));
		}
	}
	timer.endFrame();
	RecordProperty("Duration", boost::to_string(timer.getCumulativeTime()));
}

TEST_F(NamedDataTest, GetByIndex)
{
	int value;
	const int numberOfEntries = static_cast<int>(indices.size());

	SurgSim::Framework::Timer timer;
	for (int i = 0; i < numberOfLoops; ++i)
	{
		for (int j = 0; j < numberOfEntries; ++j)
		{
			ASSERT_TRUE(data.get(indices[j], &value));
		}
	}
	timer.endFrame();
	RecordProperty("Duration", boost::to_string(timer.getCumulativeTime()));
}

} // namespace DataStructures
} // namespace SurgSim

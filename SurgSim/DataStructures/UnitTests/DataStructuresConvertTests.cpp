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
#include <yaml-cpp/yaml.h>

#include <memory>

#include "SurgSim/DataStructures/DataStructuresConvert.h"

namespace SurgSim
{
namespace DataStructures
{

template <typename Type, size_t Size>
void testStdArraySerialization(const std::array<Type, Size>& value)
{
	{
		SCOPED_TRACE("Normal test");

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = value;);

		// Decode
		std::array<Type, Size> newValue;
		newValue = node.as<std::array<Type, Size>>();

		// Verify
		for (int i = 0; i < Size; ++i)
		{
			EXPECT_EQ(value[i], newValue[i]);
		}
	}

	{
		SCOPED_TRACE("Decode into smaller array");

		typedef std::array<Type, Size - 1> SmallerArray;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = value;);

		// Try decoding into a smaller array.
		SmallerArray smallerNewValue;
		EXPECT_ANY_THROW(smallerNewValue = node.as<SmallerArray>());
	}

	{
		SCOPED_TRACE("Decode into larger array");

		typedef std::array<Type, Size + 1> LargerArray;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = value;);

		// Try decoding into a larger array.
		LargerArray largerNewValue;
		EXPECT_ANY_THROW(largerNewValue = node.as<LargerArray>());
	}

	{
		SCOPED_TRACE("Serialize empty node");

		std::array<Type, Size> emptyValue;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = emptyValue;);

		// Decode
		std::array<Type, Size> receiver = value;
		receiver = node.as<std::array<Type, Size>>();

		// Verify
		bool identicalValues = true;
		for (int i = 0; i < Size; ++i)
		{
			identicalValues &= value[i] == receiver[i];
		}
		EXPECT_FALSE(identicalValues);
	}
}

TEST(DataStructuresConvertTests, StdArray)
{
	{
		SCOPED_TRACE("Serialization of std::array of size 3");
		std::array<double, 3> doubleArray;
		doubleArray[0] = 534.34;
		doubleArray[1] = 0.8435e56;
		doubleArray[2] = -56754.3;
		testStdArraySerialization(doubleArray);
	}

	{
		SCOPED_TRACE("Serialization of std::array of size 0");

		std::array<double, 0> doubleEmptyArray;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = doubleEmptyArray;);

		// Decode
		std::array<double, 0> newValue;
		newValue = node.as<std::array<double, 0>>();
	}
}

}; // namespace DataStructures
}; // namespace SurgSim

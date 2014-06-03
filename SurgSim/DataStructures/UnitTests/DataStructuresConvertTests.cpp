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

	// Test for an empty node
	{
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
		SCOPED_TRACE("std::array<bool> serialization.");
		std::array<bool, 3> boolArray;
		boolArray[0] = true;
		boolArray[1] = false;
		boolArray[2] = true;
		testStdArraySerialization(boolArray);
	}
	{
		SCOPED_TRACE("std::array<int> serialization.");
		std::array<int, 3> intArray;
		intArray[0] = 534;
		intArray[1] = 84;
		intArray[2] = -567;
		testStdArraySerialization(intArray);
	}
	{
		SCOPED_TRACE("std::array<unsigned int> serialization.");
		std::array<unsigned int, 3> uintArray;
		uintArray[0] = 53434;
		uintArray[1] = 8435;
		uintArray[2] = 567543;
		testStdArraySerialization(uintArray);
	}
	{
		SCOPED_TRACE("std::array<float> serialization.");
		std::array<float, 3> floatArray;
		floatArray[0] = 534.34f;
		floatArray[1] = 0.845f;
		floatArray[2] = -56754.3f;
		testStdArraySerialization(floatArray);
	}
	{
		SCOPED_TRACE("std::array<double> serialization.");
		std::array<double, 3> doubleArray;
		doubleArray[0] = 534.34;
		doubleArray[1] = 0.8435e56;
		doubleArray[2] = -56754.3;
		testStdArraySerialization(doubleArray);
	}
	{
		SCOPED_TRACE("std::array<char> serialization.");
		std::array<char, 3> charArray;
		charArray[0] = 'a';
		charArray[1] = 'b';
		charArray[2] = 'c';
		testStdArraySerialization(charArray);
	}
	{
		SCOPED_TRACE("std::array<std::string> serialization.");
		std::array<std::string, 3> stringArray;
		stringArray[0] = "TestStringOne";
		stringArray[1] = "TestStringTwo";
		stringArray[2] = "TestStringThree";
		testStdArraySerialization(stringArray);
	}
}

template <typename Type>
void testStdListSerialization(const std::list<Type>& value)
{
	{
		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = value;);

		// Decode
		std::list<Type> newValue;
		newValue = node.as<std::list<Type>>();

		// Verify
		EXPECT_EQ(value.size(), newValue.size());
		for (auto it = value.cbegin(), it2 = newValue.cbegin(); it != value.cend(); ++it, ++it2)
		{
			EXPECT_EQ(*it, *it2);
		}
	}

	// Test for an empty node
	{
		std::list<Type> emptyValue;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = emptyValue;);

		// Decode
		std::list<Type> receiver = value;
		receiver = node.as<std::list<Type>>();

		// Verify
		EXPECT_TRUE(receiver.empty());
	}
}

TEST(DataStructuresConvertTests, StdList)
{
	{
		SCOPED_TRACE("std::list<bool> serialization.");
		std::list<bool> boolList;
		boolList.push_back(true);
		boolList.push_back(false);
		boolList.push_back(true);
		testStdListSerialization(boolList);
	}
	{
		SCOPED_TRACE("std::list<int> serialization.");
		std::list<int> intList;
		intList.push_back(534);
		intList.push_back(84);
		intList.push_back(-567);
		testStdListSerialization(intList);
	}
	{
		SCOPED_TRACE("std::list<unsigned int> serialization.");
		std::list<unsigned int> uintList;
		uintList.push_back(53434);
		uintList.push_back(8435);
		uintList.push_back(567543);
		testStdListSerialization(uintList);
	}
	{
		SCOPED_TRACE("std::list<float> serialization.");
		std::list<float> floatList;
		floatList.push_back(534.34f);
		floatList.push_back(0.845f);
		floatList.push_back(-56754.3f);
		testStdListSerialization(floatList);
	}
	{
		SCOPED_TRACE("std::list<double> serialization.");
		std::list<double> doubleList;
		doubleList.push_back(534.34);
		doubleList.push_back(0.8435e56);
		doubleList.push_back(-56754.3);
		testStdListSerialization(doubleList);
	}
	{
		SCOPED_TRACE("std::list<char> serialization.");
		std::list<char> charList;
		charList.push_back('a');
		charList.push_back('b');
		charList.push_back('c');
		testStdListSerialization(charList);
	}
	{
		SCOPED_TRACE("std::list<std::string> serialization.");
		std::list<std::string> stringList;
		stringList.push_back("TestStringOne");
		stringList.push_back("TestStringTwo");
		stringList.push_back("TestStringThree");
		testStdListSerialization(stringList);
	}
}

}; // namespace DataStructures
}; // namespace SurgSim

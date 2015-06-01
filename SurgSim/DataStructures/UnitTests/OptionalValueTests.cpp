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

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"

namespace SurgSim
{
namespace DataStructures
{

TEST(OptionalValueTests, InitTest)
{
	EXPECT_NO_THROW({OptionalValue<int> a;});
	EXPECT_NO_THROW({OptionalValue<double> b(10.0);});

	OptionalValue<int> a;
	EXPECT_FALSE(a.hasValue());
}

TEST(OptionalValueTests, AssertTest)
{
	OptionalValue<std::shared_ptr<int>> a;

	EXPECT_ANY_THROW({ auto i = a.getValue();});
}


TEST(OptionalValueTests, SetValueTest)
{
	OptionalValue<double> a;
	EXPECT_FALSE(a.hasValue());
	a.setValue(10.0);

	EXPECT_TRUE(a.hasValue());
	EXPECT_EQ(10.0, a.getValue());

	a.invalidate();

	EXPECT_FALSE(a.hasValue());
}

TEST(OptionalValueTests, ComparatorTest)
{
	OptionalValue<int> a;
	OptionalValue<int> b;

	EXPECT_TRUE(a == b);
	EXPECT_FALSE(a != b);

	a.setValue(10);
	EXPECT_FALSE(a == b);
	EXPECT_TRUE(a != b);

	b.setValue(10);
	EXPECT_TRUE(a == b);
	EXPECT_FALSE(a != b);

	b.setValue(20);
	EXPECT_FALSE(a == b);
	EXPECT_TRUE(a != b);

	a.invalidate();
	EXPECT_FALSE(a == b);
	EXPECT_TRUE(a != b);

	EXPECT_FALSE(a == 1); // NOLINT

	a = 2;
	EXPECT_FALSE(a == 1); // NOLINT
	EXPECT_TRUE(a != 1);  // NOLINT
	EXPECT_TRUE(a == 2);  // NOLINT
}

TEST(OptionalValueTests, CopyConstructorTest)
{
	OptionalValue<int> one;
	OptionalValue<int> copyOfOne(one);

	EXPECT_EQ(one, copyOfOne);

	OptionalValue<int> two(10);
	OptionalValue<int> copyOfTwo(two);

	EXPECT_EQ(two, copyOfTwo);
}

TEST(OptionalValueTests, AssignmentOperatorTest)
{
	OptionalValue<int> one;
	OptionalValue<int> two(10);
	OptionalValue<int> target(100);

	EXPECT_NE(one, target);
	EXPECT_NE(two, target);

	target = one;
	EXPECT_EQ(one, target);
	EXPECT_NE(two, target);

	target = two;
	EXPECT_NE(one, target);
	EXPECT_EQ(two, target);

	one = 1;
	EXPECT_TRUE(one.hasValue());
	EXPECT_EQ(1, one.getValue());
}

template <typename Type>
void testOptionalValueSerialization(Type value)
{
	{
		OptionalValue<Type> optionalValue;
		optionalValue.setValue(value);

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = optionalValue;);

		// Decode
		OptionalValue<Type> newOptionalValue;
		EXPECT_NO_THROW(newOptionalValue = node.as<OptionalValue<Type>>());

		// Verify
		EXPECT_TRUE(newOptionalValue.hasValue());
		EXPECT_NO_THROW(newOptionalValue.getValue());
		EXPECT_EQ(optionalValue.getValue(), newOptionalValue.getValue());
	}

	// Test for an empty node
	{
		OptionalValue<Type> optionalValue;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = optionalValue;);

		// Decode
		OptionalValue<Type> newOptionalValue;
		newOptionalValue.setValue(value);
		EXPECT_NO_THROW(newOptionalValue = node.as<OptionalValue<Type>>());

		// Verify
		EXPECT_FALSE(newOptionalValue.hasValue());
	}

	// Test for scalar assignment, assigning an optional value from a node that contains
	// only the value of the correct type should succeed and not throw
	{
		OptionalValue<Type> optionalValue;
		YAML::Node node;
		node = value;

		EXPECT_NO_THROW(optionalValue = node.as<OptionalValue<Type>>());

		// Verify
		EXPECT_TRUE(optionalValue.hasValue());
		EXPECT_EQ(value, *optionalValue);
	}
}

TEST(OptionalValueTests, Serialization)
{
	testOptionalValueSerialization<bool>(true);
	testOptionalValueSerialization<bool>(false);
	testOptionalValueSerialization<unsigned int>(144);
	testOptionalValueSerialization<int>(37451);
	testOptionalValueSerialization<float>(921.457f);
	testOptionalValueSerialization<double>(3.1415);
	testOptionalValueSerialization<char>('f');
	testOptionalValueSerialization<std::string>("TestString");
}

TEST(OptionalValueTests, DereferenceAccess)
{
	OptionalValue<int> one;
	OptionalValue<int> two(10);

	EXPECT_ANY_THROW(*one);
	EXPECT_NO_THROW(*two);
	EXPECT_EQ(10, *two);
}

}; // namespace DataStructures
}; // namespace SurgSim

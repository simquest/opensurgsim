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

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/DataStructuresConvert.h"

using SurgSim::DataStructures::OptionalValue;

template <typename Type>
void testOptionalValueConvert(Type value)
{
	OptionalValue<Type> optionalValue;
	optionalValue.setValue(value);

	{
		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = optionalValue;);

		// Decode
		OptionalValue<Type> newOptionalValue;
		EXPECT_NO_THROW(newOptionalValue = node.as<OptionalValue<Type>>());

		// Verify
		EXPECT_EQ(true, newOptionalValue.hasValue());
		EXPECT_NO_THROW(newOptionalValue.getValue());
		EXPECT_EQ(optionalValue.getValue(), newOptionalValue.getValue());
	}

	// Test for an empty node
	{
		// Encode
		YAML::Node node;

		// Decode
		OptionalValue<Type> newOptionalValue;
		EXPECT_FALSE(YAML::convert<OptionalValue<Type>>::decode(node, newOptionalValue));

		// Verify
		EXPECT_FALSE(newOptionalValue.hasValue());
	}
}

TEST(DataStructuresConvertTests, OptionalValue)
{
	testOptionalValueConvert<bool>(true);
	testOptionalValueConvert<bool>(false);
	testOptionalValueConvert<unsigned int>(144);
	testOptionalValueConvert<int>(37451);
	testOptionalValueConvert<float>(921.457f);
	testOptionalValueConvert<double>(3.1415);
	testOptionalValueConvert<char>('f');
	testOptionalValueConvert<std::string>("TestString");
}
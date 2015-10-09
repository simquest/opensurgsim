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

#include <memory>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"

namespace
{


class EmptyComponent : public SurgSim::Framework::Component
{
public:
	explicit EmptyComponent(const std::string& name) : SurgSim::Framework::Component(name)
	{

	}

	SURGSIM_CLASSNAME(EmptyComponent);

	bool doInitialize() override
	{
		return true;
	}

	bool doWakeUp() override
	{
		return true;
	}
};

SURGSIM_REGISTER(SurgSim::Framework::Component, EmptyComponent, EmptyComponent);

}


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
		for (size_t i = 0; i < Size; ++i)
		{
			EXPECT_EQ(value[i], newValue[i]);
		}
	}

	{
		SCOPED_TRACE("Decode into smaller array");

		typedef std::array < Type, Size - 1 > SmallerArray;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = value;);

		// Try decoding into a smaller array.
		SmallerArray smallerNewValue;
		EXPECT_ANY_THROW(smallerNewValue = node.as<SmallerArray>());
	}

	{
		SCOPED_TRACE("Decode into larger array");

		typedef std::array < Type, Size + 1 > LargerArray;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = value;);

		// Try decoding into a larger array.
		LargerArray largerNewValue;
		EXPECT_ANY_THROW(largerNewValue = node.as<LargerArray>());
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

		typedef std::array<double, 0> ZeroSizeArray;

		ZeroSizeArray doubleEmptyArray;

		// Encode
		YAML::Node node;
		EXPECT_NO_THROW(node = doubleEmptyArray;);

		// Decode
		ZeroSizeArray newValue;
		EXPECT_NO_THROW(newValue = node.as<ZeroSizeArray>(););
	}
}

TEST(DataStructuresConvertTests, StdUnorderedMapTests)
{
	{
		SCOPED_TRACE("Serialization of std::unordered_map with double as key and integer as values");
		typedef std::unordered_map<double, int> TestMapType;
		TestMapType originalMap;

		originalMap.insert(TestMapType::value_type(1.0, 2));
		originalMap.insert(TestMapType::value_type(3.0, 4));

		YAML::Node node;
		EXPECT_NO_THROW(node = originalMap);
		EXPECT_EQ(2u, node.size());

		TestMapType newMap;
		EXPECT_NO_THROW(newMap = node.as<TestMapType>());

		EXPECT_EQ(originalMap, newMap);
	}

	{
		SCOPED_TRACE("Serialization of std::unordered_map with integer as key and std::shared_ptr<> as values");
		typedef std::unordered_map<int, std::shared_ptr<EmptyComponent>> TestMapType;
		TestMapType originalMap;

		auto mockComponent = std::make_shared<EmptyComponent>("Component1");
		auto mockComponent2 = std::make_shared<EmptyComponent>("Component2");

		originalMap.insert(TestMapType::value_type(1, mockComponent));
		originalMap.insert(TestMapType::value_type(2, mockComponent2));

		YAML::Node node;
		EXPECT_NO_THROW(node = originalMap);
		EXPECT_EQ(2u, node.size());

		TestMapType newMap;
		EXPECT_NO_THROW(newMap = node.as<TestMapType>());

		EXPECT_EQ(originalMap.size(), newMap.size());
		for (auto it = std::begin(originalMap); it != std::end(originalMap); ++it)
		{
			auto result = std::find_if(std::begin(newMap), std::end(newMap),
									   [&it](const std::pair<int, std::shared_ptr<EmptyComponent>>& pair)
			{
				return it->second->getName() == pair.second->getName();
			});
			EXPECT_NE(std::end(newMap), result);
		}
	}

	{
		SCOPED_TRACE("Serialization of std::unordered_map with integer as key and std::unordered_set<> as values");
		typedef std::unordered_map<int, std::unordered_set<std::shared_ptr<EmptyComponent>>> TestMapType2;
		TestMapType2 originalMap;

		std::unordered_set<std::shared_ptr<EmptyComponent>> set1;
		std::unordered_set<std::shared_ptr<EmptyComponent>> set2;

		auto mockComponent = std::make_shared<EmptyComponent>("Component1");
		auto mockComponent2 = std::make_shared<EmptyComponent>("Component2");
		auto mockComponent3 = std::make_shared<EmptyComponent>("Component3");

		set1.insert(mockComponent);
		set2.insert(mockComponent2);
		set2.insert(mockComponent3);

		originalMap.insert(TestMapType2::value_type(1, set1));
		originalMap.insert(TestMapType2::value_type(2, set2));

		YAML::Node node;
		EXPECT_NO_THROW(node = originalMap);
		EXPECT_EQ(2u, node.size());

		TestMapType2 newMap;
		EXPECT_NO_THROW(newMap = node.as<TestMapType2>());

		EXPECT_EQ(originalMap.size(), newMap.size());
		for (auto it = std::begin(originalMap); it != std::end(originalMap); ++it)
		{
			auto representationSet = newMap.find(it->first)->second;
			EXPECT_EQ(it->second.size(), representationSet.size());
			for (auto item = std::begin(it->second); item != std::end(it->second); ++item)
			{
				auto match = std::find_if(std::begin(representationSet), std::end(representationSet),
										  [&item](const std::shared_ptr<EmptyComponent> rep)
				{
					return rep->getName() == (*item)->getName();
				});
				EXPECT_NE(std::end(representationSet), match);
			}
		}
	}
}


TEST(DataStructuresConvertTests, StdUnorderedSetTests)
{
	{
		SCOPED_TRACE("Serialization of std::unordered_set<> of integers");
		typedef std::unordered_set<int> TestSetType;
		TestSetType originalSet;
		originalSet.insert(1);
		originalSet.insert(2);

		YAML::Node node;
		EXPECT_NO_THROW(node = originalSet);
		EXPECT_EQ(2u, node.size());

		TestSetType newSet;
		EXPECT_NO_THROW(newSet = node.as<TestSetType>());

		EXPECT_EQ(originalSet, newSet);
	}

	{
		SCOPED_TRACE("Serialization of std::unordered_set<> of std::shared_ptr<>");
		typedef std::unordered_set<std::shared_ptr<EmptyComponent>> TestSetType;
		TestSetType originalSet;

		auto mockComponent = std::make_shared<EmptyComponent>("Component1");
		auto mockComponent2 = std::make_shared<EmptyComponent>("Component2");

		originalSet.insert(mockComponent);
		originalSet.insert(mockComponent2);

		YAML::Node node;
		EXPECT_NO_THROW(node = originalSet);
		EXPECT_EQ(2u, node.size());

		TestSetType newSet;
		EXPECT_NO_THROW(newSet = node.as<TestSetType>());

		EXPECT_EQ(originalSet.size(), newSet.size());
		for (auto it = std::begin(originalSet); it != std::end(originalSet); ++it)
		{
			auto result = std::find_if(std::begin(newSet), std::end(newSet),
									   [&it](const std::shared_ptr<EmptyComponent>& item)
			{
				return (*it)->getName() == item->getName();
			});
			EXPECT_NE(std::end(newSet), result);
		}
	}
}

}; // namespace DataStructures
}; // namespace SurgSim
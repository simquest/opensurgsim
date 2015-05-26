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

#ifndef SURGSIM_DATASTRUCTURES_DATASTRUCTURESCONVERT_INL_H
#define SURGSIM_DATASTRUCTURES_DATASTRUCTURESCONVERT_INL_H

#include <string>

#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace DataStructures
{
namespace Convert
{
const std::string serializeLogger = "Serialization";
const std::string hasValueName = "HasValue";
const std::string valueName = "Value";
};
};
};

template <class T>
YAML::Node YAML::convert<SurgSim::DataStructures::OptionalValue<T>>::encode(
			const SurgSim::DataStructures::OptionalValue<T>& rhs)
{
	Node node;
	node[SurgSim::DataStructures::Convert::hasValueName] = rhs.hasValue();
	if (rhs.hasValue())
	{
		node[SurgSim::DataStructures::Convert::valueName] = rhs.getValue();
	}
	else
	{
		node[SurgSim::DataStructures::Convert::valueName] = "Not set";
	}
	return node;
}

template <class T>
bool YAML::convert<SurgSim::DataStructures::OptionalValue<T>>::decode(
			const Node& node, SurgSim::DataStructures::OptionalValue<T>& rhs) //NOLINT
{
	bool result = true;
	if (node.IsMap())
	{
		if (node[SurgSim::DataStructures::Convert::hasValueName].as<bool>())
		{
			try
			{
				rhs.setValue(node[SurgSim::DataStructures::Convert::valueName].as<T>());
			}
			catch (YAML::RepresentationException)
			{
				result = false;
				auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::DataStructures::Convert::serializeLogger);
				SURGSIM_LOG(logger, WARNING) << "Bad conversion";
			}
		}
		else
		{
			rhs.invalidate();
		}
	}
	else if (node.IsScalar())
	{
		try
		{
			rhs.setValue(node.as<T>());
		}
		catch (YAML::RepresentationException)
		{
			result = false;
			auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::DataStructures::Convert::serializeLogger);
			SURGSIM_LOG(logger, WARNING) << "Bad conversion";
		}
	}
	return result;
}

template <class T, size_t N>
YAML::Node YAML::convert<std::array<T, N>>::encode(const std::array<T, N>& rhs)
{
	Node node(NodeType::Sequence);
	for (auto it = rhs.cbegin(); it != rhs.cend(); ++it)
	{
		node.push_back(*it);
	}
	return node;
}

template <class T, size_t N>
bool YAML::convert<std::array<T, N>>::decode(const Node& node, std::array<T, N>& rhs) //NOLINT
{
	if (!node.IsSequence() || node.size() != N)
	{
		return false;
	}

	bool result = true;
	auto rhsit = rhs.begin();
	for (YAML::const_iterator it = node.begin(); it != node.end(); ++it, ++rhsit)
	{
		try
		{
			(*rhsit) = it->as<T>();
		}
		catch (YAML::RepresentationException)
		{
			result = false;
			auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::DataStructures::Convert::serializeLogger);
			SURGSIM_LOG(logger, WARNING) << __FUNCTION__ << ": Bad conversion";
		}
	}
	return result;
}

template <class Key, class T>
YAML::Node YAML::convert<std::unordered_map<Key, T>>::encode(const std::unordered_map<Key, T>& rhs)
{
	Node node(NodeType::Map);
	for (auto it = std::begin(rhs); it != std::end(rhs); ++it)
	{
		node[it->first] = it->second;
	}
	return node;
}

template <class Key, class T>
bool YAML::convert<std::unordered_map<Key, T>>::decode(const Node& node, std::unordered_map<Key, T>& rhs) //NOLINT
{
	if (!node.IsMap())
	{
		return false;
	}

	bool result = true;
	for (auto it = node.begin(); it != node.end(); ++it)
	{
		try
		{
			rhs[it->first.as<Key>()] = it->second.as<T>();
		}
		catch (YAML::RepresentationException)
		{
			result = false;
			auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::DataStructures::Convert::serializeLogger);
			SURGSIM_LOG(logger, WARNING) << __FUNCTION__ << ": Bad conversion";
		}
	}
	return result;
}

template <class Value>
YAML::Node YAML::convert<std::unordered_set<Value>>::encode(const std::unordered_set<Value>& rhs)
{
	Node node(NodeType::Sequence);
	for (auto it = std::begin(rhs); it != std::end(rhs); ++it)
	{
		node.push_back(*it);
	}
	return node;
}

template <class Value>
bool YAML::convert<std::unordered_set<Value>>::decode(const Node& node, std::unordered_set<Value>& rhs) //NOLINT
{
	if (!node.IsSequence())
	{
		return false;
	}

	bool result = true;
	for (auto it = node.begin(); it != node.end(); ++it)
	{
		try
		{
			rhs.insert(it->as<Value>());
		}
		catch (YAML::RepresentationException)
		{
			result = false;
			auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::DataStructures::Convert::serializeLogger);
			SURGSIM_LOG(logger, WARNING) << __FUNCTION__ << ": Bad conversion";
		}
	}
	return result;
}

#endif // SURGSIM_DATASTRUCTURES_DATASTRUCTURESCONVERT_INL_H

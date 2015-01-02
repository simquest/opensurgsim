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

#ifndef SURGSIM_DATASTRUCTURES_DATASTRUCTURESCONVERT_H
#define SURGSIM_DATASTRUCTURES_DATASTRUCTURESCONVERT_H

#include <array>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Framework/Macros.h"

namespace YAML
{

/// YAML::convert specialization for OptionalValue.
SURGSIM_DOUBLE_SPECIALIZATION
template <class T>
struct convert<SurgSim::DataStructures::OptionalValue<T>>
{
	static Node encode(const SurgSim::DataStructures::OptionalValue<T>& rhs);
	static bool decode(const Node& node, SurgSim::DataStructures::OptionalValue<T>& rhs); //NOLINT
};

/// YAML::convert specialization for std::array.
SURGSIM_DOUBLE_SPECIALIZATION
template <class T, size_t N>
struct convert<std::array<T, N>>
{
	static Node encode(const std::array<T, N>& rhs);
	static bool decode(const Node& node, std::array<T, N>& rhs); //NOLINT
};

/// YAML::convert specialization for std::unordered_map.
SURGSIM_DOUBLE_SPECIALIZATION
template <class Key, class T>
struct convert<std::unordered_map<Key, T>>
{
	static Node encode(const std::unordered_map<Key, T>& rhs);
	static bool decode(const Node& node, std::unordered_map<Key, T>& rhs); //NOLINT
};

/// YAML::convert specialization for std::unordered_set.
SURGSIM_DOUBLE_SPECIALIZATION
template <class Value>
struct convert<std::unordered_set<Value>>
{
	static Node encode(const std::unordered_set<Value>& rhs);
	static bool decode(const Node& node, std::unordered_set<Value>& rhs); //NOLINT
};

} // namespace YAML

#include "SurgSim/DataStructures/DataStructuresConvert-inl.h"

#endif // SURGSIM_DATASTRUCTURES_DATASTRUCTURESCONVERT_H

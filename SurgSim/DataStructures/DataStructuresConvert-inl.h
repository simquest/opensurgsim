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

namespace
{
const std::string serializeLogger = "Serialization";
};

template <class T>
YAML::Node YAML::convert<SurgSim::DataStructures::OptionalValue<T>>::encode(
	const SurgSim::DataStructures::OptionalValue<T>& rhs)
{
	Node node;
	if (rhs.hasValue())
	{
		node = rhs.getValue();
	}
	return node;
}

template <class T>
bool YAML::convert<SurgSim::DataStructures::OptionalValue<T>>::decode(
	const Node& node, SurgSim::DataStructures::OptionalValue<T>& rhs)
{
	bool result = false;
	try
	{
		rhs.setValue(node.as<T>());
		result = true;
	}
	catch (YAML::RepresentationException)
	{
		auto logger = SurgSim::Framework::Logger::getLogger(serializeLogger);
		SURGSIM_LOG(logger, WARNING) << "Bad conversion";
	}
	return result;
}

#endif // SURGSIM_DATASTRUCTURES_DATASTRUCTURESCONVERT_INL_H
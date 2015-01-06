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

#ifndef SURGSIM_FRAMEWORK_FRAMEWORKCONVERT_INL_H
#define SURGSIM_FRAMEWORK_FRAMEWORKCONVERT_INL_H

#include <type_traits>
#include "SurgSim/Framework/Assert.h"

// Use of enable_if. this function is only created if 'T' is a subclass of SurgSim::Framework::Component
// Which means for each subclass that is being deserialized, a new converter function is created
template <class T>
YAML::Node YAML::convert<std::shared_ptr<T>>::encode(
			const typename std::enable_if <std::is_base_of <SurgSim::Framework::Component, T>::value,
			std::shared_ptr<T> >::type  rhs)
{
	return YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::encode(rhs);
}

template <class T>
bool YAML::convert<std::shared_ptr<T>>::decode(const Node& node,
									typename std::enable_if <std::is_base_of<SurgSim::Framework::Component, T>::value,
									std::shared_ptr<T> >::type& rhs) //NOLINT
{
	std::shared_ptr<SurgSim::Framework::Component> temporary;
	bool success = 	YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::decode(node, temporary);
	if (success)
	{
		rhs = std::dynamic_pointer_cast<T>(temporary);
		SURGSIM_ASSERT(rhs != nullptr) << "Failure to convert to target type in " << __FUNCTION__;
	}
	return success;
}


#endif

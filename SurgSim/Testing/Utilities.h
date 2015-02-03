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

#ifndef SURGSIM_TESTING_UTILITIES_H
#define SURGSIM_TESTING_UTILITIES_H

namespace SurgSim
{

namespace Testing
{

/// Predicate to use to test wether a container contains a certain element
/// \tparam Container type of the container, can usually be deduced
/// \param container the container to be tested
/// \param value the value to check for
/// \return true when the container contains the given value
/// \note for maps the value type is std::pair<key, mapped_type>
template <class Container>
bool contains(const Container& container, typename const Container::value_type& value)
{
	return std::find(container.begin(), container.end(), value) != container.end();
}


}
}

#endif

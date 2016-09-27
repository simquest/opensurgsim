// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_FRAMEWORK_SCENEELEMENT_INL_H
#define SURGSIM_FRAMEWORK_SCENEELEMENT_INL_H

#include "SurgSim/Framework/Component.h"


namespace SurgSim
{

namespace Framework
{

/// Implementation of getComponents method to get all the components with type T
/// \return The type T components
template <class T>
std::vector<std::shared_ptr<T>> SceneElement::getComponents() const
{
	std::vector<std::shared_ptr<T>> result;

	for (auto componentIt = m_components.begin(); componentIt != m_components.end(); ++componentIt)
	{
		std::shared_ptr<T> typedElement = std::dynamic_pointer_cast<T>(componentIt->second);
		if (typedElement)
		{
			result.emplace_back(std::move(typedElement));
		}
	}
	return result;
}

template <class T>
T SceneElement::getValue(const std::string& component, const std::string& property) const
{
	auto found = m_components.find(component);
	SURGSIM_ASSERT(found != m_components.end())
		<< "Component named " << component << " not found in SceneElement named " << getName()
		<< ". Cannot get " << property << " property.";
	return found->second->getValue<T>(property);
}

template <class T>
bool SceneElement::getValue(const std::string& component, const std::string& property, T* value) const
{
	bool result = false;
	auto found = m_components.find(component);
	if (found != m_components.end())
	{
		result = found->second->getValue(property, value);
	}
	return result;
}

}; // namespace Framework

}; // namespace SurgSim

#endif

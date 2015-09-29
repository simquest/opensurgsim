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

#ifndef SURGSIM_FRAMEWORK_COMPONENTMANAGER_INL_H
#define SURGSIM_FRAMEWORK_COMPONENTMANAGER_INL_H

namespace SurgSim
{
namespace Framework
{

/// Executes the add component operation.
/// \tparam	T	Type of the component to be added.
/// \param	component		 	The component that is being added.
/// \param [in,out]	container	The container that the component is being added to.
/// \return	The correctly cast component if it is of type T and does not exist in the container yet, nullptr otherwise.
template<class T>
std::shared_ptr<T> ComponentManager::tryAddComponent(std::shared_ptr<SurgSim::Framework::Component> component,
													 std::vector<std::shared_ptr<T>>* container)
{
	SURGSIM_ASSERT(component != nullptr) << "Trying to add a component that is null";
	SURGSIM_ASSERT(container != nullptr) << "Trying to use a component container that is null";
	std::shared_ptr<T> typedComponent = std::dynamic_pointer_cast<T>(component);
	if (typedComponent != nullptr)
	{
		auto found = std::find(container->cbegin(), container->cend(), typedComponent);
		if (found == container->cend())
		{
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Added component " << component->getName();
			container->push_back(typedComponent);
		}
		else
		{
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " component " << component->getName() <<
				" already added to " << getName();
			typedComponent = nullptr;
		}
	}
	return typedComponent;
};

template<class T>
bool ComponentManager::tryRemoveComponent(std::shared_ptr<SurgSim::Framework::Component> component,
										  std::vector<std::shared_ptr<T>>* container)
{
	SURGSIM_ASSERT(container != nullptr) << "Trying to use a component container that is null";
	bool result = false;
	std::shared_ptr<T> typedComponent = std::dynamic_pointer_cast<T>(component);
	if (typedComponent != nullptr && container->size() != 0)
	{
		auto found = std::find(container->begin(), container->end(), typedComponent);
		if (found != container->end())
		{
			container->erase(found);
			SURGSIM_LOG_DEBUG(m_logger) << __FUNCTION__ << " Removed component " << typedComponent->getName();
			result = true;
		}
		else
		{
			SURGSIM_LOG_INFO(m_logger) << SURGSIM_CURRENT_FUNCTION << " Unable to remove component " <<
				typedComponent->getName() << ". Not found.";
		}
	}
	return result;
};

template<class T>
void ComponentManager::retireComponents(const std::vector<std::shared_ptr<T>>& container)
{
	static_assert(std::is_base_of<Component, T>::value == true, "Class has to be of type component");
	for (const auto& component : container)
	{
		component->retire();
	}
}


}; // namespace Framework
}; // namespace SurgSim

#endif


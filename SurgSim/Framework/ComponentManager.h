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

#ifndef SURGSIM_FRAMEWORK_COMPONENTMANAGER_H
#define SURGSIM_FRAMEWORK_COMPONENTMANAGER_H

#include <memory>
#include <string>

#include <SurgSim/Framework/BasicThread.h>
#include <SurgSim/Framework/Log.h>

namespace SurgSim
{
namespace Framework
{

class Component;
class Runtime;
class Logger;

/// Base Component Manager class. Component Managers manage a collection of
/// components. The runtime will present each new component to the manager, and
/// it is up to the manger to decide whether to handle a component of a given
/// type or not.
class ComponentManager : public BasicThread
{
public:

	explicit ComponentManager(const std::string& name = "Unknown Component Manager");
	virtual ~ComponentManager();

	/// Handle representations, override for each thread
	/// \param component	The component to be removed.
	/// \return true on success
	virtual bool removeComponent(std::shared_ptr<Component> component) = 0;

	/// Adds a component.
	/// \param component The component to be added.
	/// \return true if it succeeds or the thread is not concerned with the component, false if it fails.
	virtual bool addComponent(std::shared_ptr<Component> component) = 0;

	/// @{
	/// Runtime accessors
	inline std::shared_ptr<Runtime> getRuntime() const
	{
		return m_runtime.lock();
	}

	void setRuntime(std::shared_ptr<Runtime> val);
	/// @}
	

	/// Template version of the addComponent method.
	/// \tparam	T	Specific type of the component that is being added.
	/// \param	component		 	The component that needs to be added.
	/// \param [in,out]	container	If non-null, the container, that should receive the component if of the correct type.
	/// \return	the correctly cast component pointer if successful and the component did not alread exist in the container
	template<class T>
	std::shared_ptr<T> tryAddComponent(std::shared_ptr<SurgSim::Framework::Component> component, std::vector<std::shared_ptr<T>>* container);

	/// Template version of the removeComponent method.
	/// \tparam	T	Specific type of the component that is being removed.
	/// \param	component		 	The component that needs to be removed.
	/// \param [in,out]	container	If non-null, the container, from which the component should be removed.
	/// \return	true if the component exists in the container or the component did not cast to T, otherwise.
	template<class T>
	bool tryRemoveComponent(std::shared_ptr<SurgSim::Framework::Component> component, std::vector<std::shared_ptr<T>>* container);

protected:
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

private:
	std::weak_ptr<Runtime> m_runtime;
};

#include <SurgSim/Framework/ComponentManager-inl.h>

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_COMPONENTMANAGER_H

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

#include "SurgSim/Framework/BasicThread.h"

namespace SurgSim
{
namespace Framework
{

class Component;
class Runtime;

/// Base Component Manager class. Component Managers manage a collection of
/// components. The runtime will present each new component to the manager, and
/// it is up to the manger to decide whether to handle a component of a given
/// type or not.
class ComponentManager : public BasicThread
{
public:
	explicit ComponentManager(const std::string& name = "Unknown Component Manager") :
		BasicThread(name)
	{
	}

	virtual ~ComponentManager()
	{
	}

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
	std::shared_ptr<Runtime> getRuntime() const
	{
		return m_runtime.lock();
	}

	void setRuntime(std::shared_ptr<Runtime> val)
	{
		m_runtime = val;
	}
	/// @}

private:
	std::weak_ptr<Runtime> m_runtime;
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_COMPONENTMANAGER_H

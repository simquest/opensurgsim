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

#ifndef SURGSIM_FRAMEWORK_COMPONENT_H
#define SURGSIM_FRAMEWORK_COMPONENT_H

#include <string>
#include <memory>

#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Framework
{

// Forward References
class Scene;
class SceneElement;
class Runtime;

/// Component is the main interface class to pass information to the system managers each will decide
/// whether to handle a component of a given type or not. Components will get initialized by having
/// doInit(), and doWakeUp() called in succession, all components together will have doInit() called before
/// any component will recieve doWakeUp()
class Component
{
public:
	/// Constructor
	/// \param	name	Name of the component
	explicit Component(const std::string& name);
	/// Destructor
	virtual ~Component();

	/// Gets component name.
	/// \return	Name of this component.
	std::string getName() const;
	/// Sets the name of component.
	/// \param	name	The name of this component.
	void setName(const std::string& name);

	/// \return True if this component is initialized; otherwise, false.
	bool isInitialized() const;
	/// Initialize this component.
	/// \param runtime	The runtime which contains this component.
	/// \return True if this component is initialized successfully; otherwise, false.
	bool initialize(const std::shared_ptr<Runtime>& runtime);

	/// \return True if this component is awake; otherwise, false.
	bool isAwake() const;
	/// Wakeup this component
	/// \return True if this component is woken up successfully; otherwise, false.
	bool wakeUp();

	/// Sets the scene.
	/// \param scene The scene for this component
	void setScene(std::weak_ptr<Scene> scene);
	/// Gets the scene.
	/// \return The scene for this component
	std::shared_ptr<Scene> getScene();

	/// Sets the scene element.
	/// \param sceneElement The scene element for this component
	void setSceneElement(std::weak_ptr<SceneElement> sceneElement);
	/// Gets the scene element.
	/// \return The scene element for this component
	std::shared_ptr<SceneElement> getSceneElement();

	/// Get the runtime which contains this component
	/// \return The runtime which contains this component
	std::shared_ptr<Runtime> getRuntime() const;

private:
	/// Name of this component
	std::string m_name;

	/// Runtime which contains this component
	std::weak_ptr<Runtime> m_runtime;

	/// Scene which contains this component
	std::weak_ptr<Scene> m_scene;

	/// SceneElement which contains this component
	std::weak_ptr<SceneElement> m_sceneElement;

	/// Interface to be implemented by derived classes
	/// \return True if component is initialized successfully; otherwise, false.
	virtual bool doInitialize() = 0;
	/// Interface to be implemented by derived classes
	/// \return True if component is woken up successfully; otherwise, false.
	virtual bool doWakeUp() = 0;

	/// Indicates if doInitialize() has been called
	bool m_didInit;
	/// Indicates if doWakeup() has been called
	bool m_didWakeUp;

	/// Indicates if this component is initialized
	bool m_isInitialized;
	/// Indicates if this component is awake
	bool m_isAwake;
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_COMPONENT_H

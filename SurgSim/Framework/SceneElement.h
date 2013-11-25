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

#ifndef SURGSIM_FRAMEWORK_SCENEELEMENT_H
#define SURGSIM_FRAMEWORK_SCENEELEMENT_H

#include <string>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace SurgSim
{
namespace Framework
{

class Component;
class Scene;
class Runtime;

/// SceneElement is the basic part of a scene, it is a container of components. SceneElements
/// doInit() will be called on all SceneElements before they are integrated into the system,
/// doWakeup() will be called after all doInit() have been completed and all the static information
/// has been initialized
class SceneElement : public std::enable_shared_from_this<SceneElement>
{
public:
	/// Constructor
	/// \param name Name of this SceneElement
	explicit SceneElement(const std::string& name);
	/// Destructor
	virtual ~SceneElement();

	/// Adds a component
	/// \param	component	The component.
	/// \return	true if it succeeds, false if it fails.
	bool addComponent(std::shared_ptr<Component> component);
	/// Removes a given component.
	/// \param	component	The component.
	/// \return	true if it succeeds, false if it fails or the component cannot be found.
	bool removeComponent(std::shared_ptr<Component> component);
	/// Removes the component described by name.
	/// \param	name	The name.
	/// \return	true if it succeeds, false if it fails or the component cannot be found.
	bool removeComponent(const std::string& name);

	/// Gets the component identified by name.
	/// \param	name	The name.
	/// \return	The component or nullptr if the component cannot be found.
	std::shared_ptr<Component> getComponent(const std::string& name) const;
	/// Gets all the components of this SceneElement.
	/// \return	The components.
	std::vector<std::shared_ptr<Component> > getComponents() const;
	/// Template version of getComponents method to get all the components with type T
	/// \return The type T components
	template <class T>
	std::vector<std::shared_ptr<T>> getComponents() const;

	/// Executes the initialize operation.
	/// \return	true if it succeeds, false if it fails.
	bool initialize();

	/// Executes the wake up operation.
	/// \return	true if it succeeds, false if it fails.
	bool wakeUp();

	/// Return the name of this SceneElement
	/// \return	The name.
	std::string getName() const;

	/// Sets the Scene.
	/// \param scene Pointer to the scene.
	void setScene(std::weak_ptr<Scene> scene);
	/// Gets the Scene.
	/// \return The scene.
	std::shared_ptr<Scene> getScene();

	/// Sets the Runtime.
	/// \param runtime Pointer to the runtime.
	void setRuntime(std::shared_ptr<Runtime> runtime);
	/// Gets the runtime.
	/// \return	The runtime.
	std::shared_ptr<Runtime> getRuntime();

	/// Return if this SceneElement is initialized.
	/// \return True if this SceneElement is initialized; Otherwise, false.
	bool isInitialized() const;
	/// Return if this SceneElement is awake.
	/// \return True if this SceneElement is awake; Otherwise, false.
	bool isAwake() const;

	/// Gets a shared pointer to the runtime.
	/// \return	The shared pointer.
	std::shared_ptr<SceneElement> getSharedPtr();

private:
	/// Name of this SceneElement
	std::string m_name;
	/// A collection of Components
	std::unordered_map<std::string, std::shared_ptr<Component>> m_components;
	/// A (weak) back pointer to the Scene containing this SceneElement
	std::weak_ptr<Scene> m_scene;
	/// A (weak) back pointer to the Runtime containing this SceneElement
	std::weak_ptr<Runtime> m_runtime;

	/// Method to initialized this SceneElement. To be overridden by derived class(es).
	/// \return Ture if initialization is successful; Otherwise, false.
	virtual bool doInitialize() = 0;
	/// Method to wake up this SceneElement. To be overridden by derived class(es).
	/// \return Ture if initialization is successful; Otherwise, false.
	virtual bool doWakeUp() = 0;

	/// Indicates if this SceneElement has been initialized or not.
	bool m_isInitialized;
	/// Indicates if this SceneElement has been waken or not.
	bool m_isAwake;
};

}; // namespace Framework
}; // namespace SurgSim

#include <SurgSim/Framework/SceneElement-inl.h>

#endif // SURGSIM_FRAMEWORK_SCENEELEMENT_H

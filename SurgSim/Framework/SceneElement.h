// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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
#include <unordered_set>
#include <vector>

#include "SurgSim/Math/RigidTransform.h"

namespace YAML
{
class Node;
}

namespace SurgSim
{

namespace DataStructures
{
template <typename Key, typename T>
class Groups;
}

namespace Framework
{

class Component;
class PoseComponent;
class Scene;
class Runtime;


/// SceneElement is the basic part of a scene, it is a container of components. When a
/// SceneElement is added to a Scene, the Scene will call initialize() on the SceneElement,
/// which in turn will call initialize() on all its components.
/// If a component is added to a SceneElement after the SceneElement was initialized, the component will
/// be initialized immediately.
/// \note A SceneElement needs to be created as a shared_ptr.
class SceneElement : public std::enable_shared_from_this<SceneElement>
{
public:

	/// Constructor
	/// \param name Name of this SceneElement
	explicit SceneElement(const std::string& name);

	/// Destructor
	virtual ~SceneElement();

	/// \return the class name for this class
	virtual std::string getClassName() const;

	/// Adds a component, calls initialize() on the component, if SceneElement::isInitialized() is true
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

	/// Removes all components
	void removeComponents();

	/// Gets the component identified by name.
	/// \param	name	The name.
	/// \return	The component or nullptr if the component cannot be found.
	std::shared_ptr<Component> getComponent(const std::string& name) const;

	/// Gets all the components of this SceneElement.
	/// \return	The components.
	std::vector<std::shared_ptr<Component>> getComponents() const;

	/// Template version of getComponents method to get all the components with type T
	/// \return The type T components
	template <class T>
	std::vector<std::shared_ptr<T>> getComponents() const;

	/// Add this scene element to the given group
	/// \param group name of the group
	void addToGroup(const std::string& group);

	/// Remove this scene element from the given group
	/// \param group name of the group
	void removeFromGroup(const std::string& group);

	/// Set the groups of this scene element
	/// \param groups all the groups that this scene element should be in
	void setGroups(const std::vector<std::string>& groups);

	/// \return all the groups of this scene element
	std::vector<std::string> getGroups() const;

	bool inGroup(const std::string& name);

	/// Executes the initialize operation.
	/// \return	true if it succeeds, false if it fails.
	bool initialize();

	/// Return the name of this SceneElement
	/// \return	The name.
	std::string getName() const;

	/// Set the pose of this SceneElement
	/// \param pose The pose
	void setPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Get the pose of this SceneElement
	/// \return the pose
	const SurgSim::Math::RigidTransform3d& getPose() const;

	/// Get the PoseComponent that controls the pose all Representations
	/// in this SceneElement
	/// \return the PoseComponent
	std::shared_ptr<PoseComponent> getPoseComponent();

	/// Sets the Scene.
	/// \param scene Pointer to the scene.
	void setScene(std::weak_ptr<Scene> scene);

	/// Gets the Scene.
	/// \return The scene.
	std::shared_ptr<Scene> getScene();

	/// Sets the Runtime.
	/// \param runtime Pointer to the runtime.
	void setRuntime(std::weak_ptr<Runtime> runtime);
	/// Gets the runtime.
	/// \return	The runtime.
	std::shared_ptr<Runtime> getRuntime();

	/// Return if this SceneElement is initialized.
	/// \return True if this SceneElement is initialized; Otherwise, false.
	bool isInitialized() const;

	/// Set this SceneElement's status (active/inactive)
	/// \note Set 'inactive' on an active SceneElement will cause all the components it contains to not be processed.
	/// \note Set 'active' on an inactive SceneElement will cause all the components it contains which are active
	/// \note to be processed again.
	/// \note Inactive components will not be processed no matter the SceneElement is active or inactive.
	/// \param val True to set the SceneElement active, false to set inactive.
	void setActive(bool val);

	/// \return True if this SceneElement is active; Otherwise, false.
	bool isActive() const;

	/// Gets a shared pointer to this SceneElement.
	/// \return	The shared pointer.
	std::shared_ptr<SceneElement> getSharedPtr();

	/// Convert to a YAML::Node
	/// \param standalone when true, all the components will be represented as full component, when false
	///                   they will be represented as references
	/// \return A node with all the public data of this instance
	virtual YAML::Node encode(bool standalone) const;

	/// Pull data from a YAML::Node.
	/// \throws SurgSim::Framework::AssertionFailure if the SceneElement is already initialized
	/// \param node the node to decode.
	/// \return true if the decoding succeeded and the node was formatted correctly, false otherwise
	virtual bool decode(const YAML::Node& node);

private:
	/// Initialize the given component
	/// \param component The component to be initialized.
	/// \return True if initialization succeeded, false otherwise.
	bool initializeComponent(std::shared_ptr<SurgSim::Framework::Component> component);

	/// Name of this SceneElement
	std::string m_name;

	std::shared_ptr<SurgSim::Framework::PoseComponent> m_pose;

	/// A collection of Components
	std::unordered_map<std::string, std::shared_ptr<Component>> m_components;

	/// A (weak) back pointer to the Scene containing this SceneElement
	std::weak_ptr<Scene> m_scene;

	/// A (weak) back pointer to the Runtime containing this SceneElement
	std::weak_ptr<Runtime> m_runtime;

	/// Method to initialize this SceneElement. To be overridden by derived class(es).
	/// \return True if initialization is successful; Otherwise, false.
	virtual bool doInitialize() = 0;

	/// Sets the name of this SceneElement.
	/// \param name The new name.
	void setName(const std::string& name);

	/// Indicates if this SceneElement has been initialized or not.
	bool m_isInitialized;

	/// Indicates if this SceneElement is active or not.
	bool m_isActive;

	/// Local groups for serialization local handling
	std::shared_ptr<SurgSim::DataStructures::Groups<std::string, std::shared_ptr<SceneElement>>> m_groups;

};

}; // namespace Framework
}; // namespace SurgSim

#include "SurgSim/Framework/SceneElement-inl.h"

#endif // SURGSIM_FRAMEWORK_SCENEELEMENT_H

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

#include <boost/uuid/uuid.hpp>

#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{
namespace Framework
{

// Forward References
class PoseComponent;
class Runtime;
class Scene;
class SceneElement;

/// Component is the main interface class to pass information to the system managers each will decide
/// whether to handle a component of a given type or not. Components will get initialized by having
/// doInit(), and doWakeUp() called in succession, all components together will have doInit() called before
/// any component will recieve doWakeUp()
class Component : public Accessible, public std::enable_shared_from_this<Component>
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

	/// Gets the id of the component
	boost::uuids::uuid getUuid() const;

	/// \return True if this component is initialized; otherwise, false.
	bool isInitialized() const;

	/// Initialize this component, this needs to be called before wakeUp() can be called.
	/// This will be done automatically by the Scene hierarchy, either in SceneElement::addComponent(), if
	/// SceneElement has already been added to the Scene, or through Scene::addSceneElement() on all Components
	/// on the SceneElement.
	/// \param runtime	The runtime which contains this component.
	/// \return True if this component is initialized successfully; otherwise, false.
	bool initialize(const std::weak_ptr<Runtime>& runtime);

	/// \return True if this component is awake; otherwise, false.
	bool isAwake() const;

	/// Wakeup this component, this will be called when the component is inserted into the ComponentManager that is
	/// responsible for handling this component.
	/// \return True if this component is woken up successfully; otherwise, false.
	bool wakeUp();

	/// Sets the scene.
	/// \param scene The scene for this component
	void setScene(std::weak_ptr<Scene> scene);

	/// Gets the scene.
	/// \return The scene for this component
	std::shared_ptr<Scene> getScene();

	/// Sets the scene element.
	/// \param sceneElement The scene element for this component.
	void setSceneElement(std::weak_ptr<SceneElement> sceneElement);

	/// Gets the scene element.
	/// \return The scene element for this component.
	std::shared_ptr<SceneElement> getSceneElement();

	/// Gets the scene element, constant version
	/// \return The scene element for this component.
	std::shared_ptr<const SceneElement> getSceneElement() const;

	/// Get the runtime which contains this component.
	/// \return The runtime which contains this component.
	std::shared_ptr<Runtime> getRuntime() const;

	/// The class name for this class, this being the base class it should
	/// return SurgSim::Framework::Component but this would make missing implemenentations
	/// of this hard to catch, therefore this calls SURGSIM_FAILURE.
	/// \note Use the SURGSIM_CLASSNAME macro in derived classes.
	/// \return The fully namespace qualified name of this class.
	virtual std::string getClassName() const;

	typedef SurgSim::Framework::ObjectFactory1<SurgSim::Framework::Component, std::string> FactoryType;

	/// \return The static class factory that is being used in the conversion.
	static FactoryType& getFactory();

	/// Gets a shared pointer to this component.
	/// \return	The shared pointer.
	std::shared_ptr<Component> getSharedPtr();

	/// Interface to be implemented by derived classes
	/// \return True if component is initialized successfully; otherwise, false.
	virtual bool doInitialize() = 0;

	/// Interface to be implemented by derived classes
	/// \return True if component is woken up successfully; otherwise, false.
	virtual bool doWakeUp() = 0;

	/// \return True if this component is active and its SceneElement (if any) is also active;
	/// Otherwise, false.
	bool isActive() const;

	/// Set the component's active state
	/// \param val If true component is active, inactive if false.
	virtual void setLocalActive(bool val);

	/// \return True if this component is active
	/// Otherwise, false.
	bool isLocalActive() const;

protected:
	/// Get the PoseComponent for this component
	/// \return The PoseComponent
	virtual std::shared_ptr<PoseComponent> getPoseComponent();

	/// Get the PoseComponent for this component, constant access
	/// \return The PoseComponent
	virtual std::shared_ptr<const PoseComponent> getPoseComponent() const;


private:


	/// Name of this component
	std::string m_name;

	/// Id of this component
	boost::uuids::uuid m_uuid;

	/// Runtime which contains this component
	std::weak_ptr<Runtime> m_runtime;

	/// Scene which contains this component
	std::weak_ptr<Scene> m_scene;

	/// SceneElement which contains this component
	std::weak_ptr<SceneElement> m_sceneElement;

	/// Indicates if doInitialize() has been called
	bool m_didInit;

	/// Indicates if doWakeup() has been called
	bool m_didWakeUp;

	/// Indicates if this component is initialized
	bool m_isInitialized;

	/// Indicates if this component is awake
	bool m_isAwake;

	/// Indicates if this component is active
	bool m_isLocalActive;

};

/// The function tries to convert the Source type to the Target type it will throw if Target is not a subclass
/// of Source.
/// \tparam Target type that is used as the target type for the conversion, can usually be deduced
/// \tparam Source type that is the type of the incoming parameter, target needs to be a subclass of Source for the
///         function to succeed
/// \param incoming pointer to an instance of Source that is supposed to be converted to a pointer to Target
/// \param expectedTypeName a name to be used in the error message if the conversion fails, use the full
///        namespace name of Source here.
/// \throws if
/// \return pointer of type Target if Target is a subclass of Source, throws otherwise.
template <class Target, class Source>
std::shared_ptr<Target> checkAndConvert(std::shared_ptr<Source> incoming, const std::string& expectedTypeName);

}; // namespace Framework
}; // namespace SurgSim

#include "SurgSim/Framework/Component-inl.h"

#endif // SURGSIM_FRAMEWORK_COMPONENT_H

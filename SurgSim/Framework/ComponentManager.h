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
#include <vector>

#include <boost/thread/mutex.hpp>

#include "SurgSim/Framework/BasicThread.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Component.h"

namespace SurgSim
{
namespace Framework
{

class Runtime;
class Logger;

/// Base Component Manager class. Component Managers manage a collection of components.
/// The runtime will present each new component to the manager, and it is up to
/// the manger to decide whether to handle a component of a given type or not.
/// Adding and removing components is thread-safe, when the [add|remove]Component
/// call is made, the component is added to an intermediary data structure, each
/// ComponentManager implementation must call processComponents() to trigger the
/// actual addition and removal. Each ComponentManager subclass needs to implement
/// doAddComponent() and doRemoveComponent() to the actual addition and removal of
/// components.
/// ComponentManager implements a custom executeInitialization() method that lets the
/// runtime schedule initialization of components that exist at the start of the simulation
class ComponentManager : public BasicThread
{
public:

	explicit ComponentManager(const std::string& name = "Unknown Component Manager");
	virtual ~ComponentManager();

	/// Queues a component to be added later.
	/// \param component The component to be added.
	/// \return true if the component was scheduled for addition, this does not indicate that
	/// 		the component will actually be added to this manager
	bool enqueueAddComponent(const std::shared_ptr<Component>& component);

	/// Queues a component to be removed
	/// \param component	The component to be removed.
	/// \return true if the component was scheduled for removal, this does not indicate that
	/// 		the component will actually be removed from this manager
	bool enqueueRemoveComponent(const std::shared_ptr<Component>& component);

	/// @{
	/// Runtime accessors
	std::shared_ptr<Runtime> getRuntime() const;
	void setRuntime(std::shared_ptr<Runtime> val);
	/// @}

protected:
	/// Template version of the addComponent method.
	/// \tparam	T	Specific type of the component that is being added.
	/// \param	component		 	The component that needs to be added.
	/// \param [in,out]	container	If non-null, the container that should receive the component if of the correct type.
	/// \return	the correctly cast component pointer if successful and the
	/// 		component did not already exist in the container
	template<class T>
	std::shared_ptr<T> tryAddComponent(std::shared_ptr<SurgSim::Framework::Component> component,
									   std::vector<std::shared_ptr<T>>* container);

	/// Template version of the removeComponent method.
	/// \tparam	T	Specific type of the component that is being removed.
	/// \param	component		 	The component that needs to be removed.
	/// \param [in,out]	container	If non-null, the container, from which the component should be removed.
	/// \return	true if the component exists in the container or the component did not cast to T, otherwise.
	template<class T>
	bool tryRemoveComponent(std::shared_ptr<SurgSim::Framework::Component> component,
							std::vector<std::shared_ptr<T>>* container);


	/// Processes all the components that are scheduled for addition or removal, this needs to be called
	/// inside the doUpdate() function.
	void processComponents();
	/// Processes behaviors
	/// This needs to be called inside doUpdate() function in each 'sub' manager.
	void processBehaviors(const double dt);

	/// Returns the type of Manager
	// Enum is defined in the beginning of this file
	virtual int getType() const = 0;

	/// Helper, blocks access to the additions and removal queue and copies the components
	/// from there to the intermediate inflight queues, after this call, the incoming
	/// queues will be empty.
	void copyScheduledComponents(std::vector<std::shared_ptr<Component>>* inflightAdditions,
								 std::vector<std::shared_ptr<Component>>* inflightRemovals,
								 std::vector<std::shared_ptr<SceneElement>>* inflightElements);

	/// Returns this manager's logger
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const;

	/// Blocks protects addition and removal queues
	boost::mutex m_componentMutex;

	///@{
	/// Data structures
	/// Contain components scheduled to be added/removed
	std::vector<std::shared_ptr<Component>> m_componentAdditions;
	std::vector<std::shared_ptr<Component>> m_componentRemovals;
	std::vector<std::shared_ptr<SceneElement>> m_elementCache;
	///@}

	/// Collection of behaviors
	// Each behavior will have a type to be matched with the corresponding manager
	// Managers will only handle matching behaviors
	std::vector<std::shared_ptr<SurgSim::Framework::Behavior>> m_behaviors;

	void doBeforeStop() override;

	template<class T>
	void retireComponents(const std::vector<std::shared_ptr<T>>& container);

private:
	/// Adds a component.
	/// \param component The component to be added.
	/// \return true if the component was scheduled for addition, this does not indicate that
	/// 		the component will actually be added to this manager.
	virtual bool executeAdditions(const std::shared_ptr<Component>& component) = 0;

	/// Handle representations, override for each thread
	/// \param component	The component to be removed.
	/// \return true if the component was scheduled for removal, this does not indicate that
	/// 		the component will actually be removed from this manager.
	virtual bool executeRemovals(const std::shared_ptr<Component>& component) = 0;

	/// Overridden from BasicThread, extends the initialization to contain component initialization
	/// including waiting for the other threads to conclude their component initialization and wakeup
	bool executeInitialization() override;

	/// Delegates to doRemoveComponent to remove all the components in the indicated array.
	/// \param	beginIt	The begin iterator.
	/// \param	endIt  	The end iterator.
	void removeComponents(const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
						  const std::vector<std::shared_ptr<Component>>::const_iterator& endIt);

	/// Delegates to doAddComponent and calls initialize on all the components
	/// \param	beginIt	The begin iterator.
	/// \param	endIt  	The end iterator.
	/// \param[out] actualAdditions	List of components actually added
	void addComponents(
		const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
		const std::vector<std::shared_ptr<Component>>::const_iterator& endIt,
		std::vector<std::shared_ptr<Component>>* actualAdditions);

	/// Wake all the components up, only the components that were successfully initialized get
	/// the wakeup call, check for isAwake because there to catch multiple versions of the same
	/// component from being awoken more than once. Will also remove components if they did not
	/// wake up as expected
	/// \param	beginIt	The begin iterator.
	/// \param	endIt  	The end iterator.
	void wakeUpComponents(const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
						  const std::vector<std::shared_ptr<Component>>::const_iterator& endIt);

	std::weak_ptr<Runtime> m_runtime;
};

}; // namespace Framework
}; // namespace SurgSim

#include "SurgSim/Framework/ComponentManager-inl.h"

#endif // SURGSIM_FRAMEWORK_COMPONENTMANAGER_H

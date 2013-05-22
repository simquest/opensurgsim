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

#ifndef SURGSIM_PHYSICS_PHYSICSMANAGER_H
#define SURGSIM_PHYSICS_PHYSICSMANAGER_H

#include <memory>
#include <vector>

#include <SurgSim/Framework/ComponentManager.h>
#include <Surgsim/Framework/Component.h>
#include <SurgSim/Framework/Log.h>


namespace SurgSim
{
namespace Framework
{
	class Logger;
	class Component;
}

namespace Physics
{

class Actor;
class FreeMotion;

/// PhyicsManager handles the physics and motion calculation, it uses Computations to
/// separate the algorithmic steps into smaller pieces.
class PhysicsManager : public SurgSim::Framework::ComponentManager
{
public:

	/// Condstructor
	PhysicsManager();
	~PhysicsManager();

	///@{
	/// Overridden from ComponentManager
	bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component);
	bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component);
	///@}

protected:

	///@{
	/// Overriden from ComponentManager
	virtual bool doInitialize();
	virtual bool doStartUp();
	virtual bool doUpdate(double dt);

	/// Template version of the addComponent method.
	/// \tparam	T	Specific type of the component that is being added.
	/// \param	component		 	The component that needs to be added.
	/// \param [in,out]	container	If non-null, the container, that should receive the component if of the correct type.
	/// \return	the correctly cast component pointer if successful and the component did not alread exist in the container
	template<class T>
	std::shared_ptr<T> doAddComponent(std::shared_ptr<SurgSim::Framework::Component> component, std::vector<std::shared_ptr<T>>* container);

	/// Template version of the removeComponent method.
	/// \tparam	T	Specific type of the component that is being removed.
	/// \param	component		 	The component that needs to be removed.
	/// \param [in,out]	container	If non-null, the container, from which the component should be removed.
	/// \return	true if the component exists in the container or the component did not cast to T, otherwise.
	template<class T>
	bool doRemoveComponent(std::shared_ptr<SurgSim::Framework::Component> component, std::vector<std::shared_ptr<T>>* container);

private:

	std::shared_ptr< std::vector<std::shared_ptr<Actor>> > m_actors;
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	///@{
	/// Steps to perform the physics update
	std::unique_ptr<FreeMotion> m_freeMotionStep;
	///@}

};

#include <SurgSim/Physics/PhysicsManager-inl.h>

}; // namespace Physics
}; // namespace SurgSim



#endif
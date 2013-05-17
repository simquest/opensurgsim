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

#ifndef SURGSIM_GRAPHICS_MANAGER_H
#define SURGSIM_GRAPHICS_MANAGER_H

#include <SurgSim/Framework/ComponentManager.h>

#include <memory>
#include <vector>

namespace SurgSim 
{

namespace Graphics
{

class Actor;
class Group;
class View;

/// Basic graphics manager class which manages graphics components to provide a visualization of the scene to the user.
///
/// Graphics::Manager manages Graphics::Actor, Graphics::Group, and Graphics::View components.
class Manager : public SurgSim::Framework::ComponentManager
{
public:
	/// Constructor
	Manager();
	/// Destructor
	virtual ~Manager();

	/// Adds a component
	/// \param	component	The component to be added.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component);

	/// Removes a component
	/// \param	component	The component to be removed.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component);
	
	/// Adds an actor to the manager
	/// \param	actor	The actor to be added.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool addActor(std::shared_ptr<Actor> actor);

	/// Adds a group to the manager
	/// \param	group	The group to be added.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool addGroup(std::shared_ptr<Group> group);

	/// Adds a view to the manager
	/// \param	view	The view to be added.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool addView(std::shared_ptr<View> view);

	/// Removes an actor from the manager
	/// \param	actor	The actor to be removed.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool removeActor(std::shared_ptr<Actor> actor);

	/// Removes a group from the manager
	/// \param	group	The group to be removed.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool removeGroup(std::shared_ptr<Group> group);

	/// Removes a view from the manager
	/// \param	view	The view to be removed.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool removeView(std::shared_ptr<View> view);

	/// Returns the actors assigned to the manager
	const std::vector<std::shared_ptr<Actor>>& getActors() const
	{
		return m_actors;
	}

	/// Returns the groups assigned to the manager
	const std::vector<std::shared_ptr<Group>>& getGroups() const
	{
		return m_groups;
	}

	/// Returns the views assigned to the manager
	const std::vector<std::shared_ptr<View>>& getViews() const
	{
		return m_views;
	}

protected:
	/// Performs an update for a single timestep
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual bool doUpdate(double dt);

private:
	/// Initializes the manager
	/// \return True if it succeeds, false if it fails
	virtual bool doInitialize();

	/// Starts up the manager after all threads have initialized
	/// \return True if it succeeds, false if it fails
	virtual bool doStartUp();

	/// Actors assigned to the manager
	std::vector<std::shared_ptr<Actor>> m_actors;
	/// Groups assigned to the manager
	std::vector<std::shared_ptr<Group>> m_groups;
	/// Views assigned to the manager
	std::vector<std::shared_ptr<View>> m_views;

	/// This manager's logger
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_MANAGER_H

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

#include "SurgSim/Framework/ComponentManager.h"

#include <memory>
#include <vector>

namespace SurgSim
{

namespace Graphics
{

class Group;
class Representation;
class View;

/// Basic graphics manager class which manages graphics components to provide a visualization of the scene to the user.
///
/// Graphics::Manager manages Graphics::Representation, Graphics::Group, and Graphics::View components.
class Manager : public SurgSim::Framework::ComponentManager
{
public:
	/// Constructor
	Manager();
	/// Destructor
	virtual ~Manager();

	/// Returns the representations assigned to the manager
	const std::vector<std::shared_ptr<Representation>>& getRepresentations() const
	{
		return m_representations;
	}

	/// Returns the groups assigned to the manager
	const std::unordered_map<std::string, std::shared_ptr<Group>>& getGroups() const
	{
		return m_groups;
	}

	/// Returns the views assigned to the manager
	const std::vector<std::shared_ptr<View>>& getViews() const
	{
		return m_views;
	}

	/// Generic unspecified debug handle, there are no requirements on this interface
	/// the manager implementation can decide what to do
	virtual void dumpDebugInfo() const = 0;

protected:

	/// Adds a component
	/// \param	component	The component to be added.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component);

	/// Removes a component
	/// \param	component	The component to be removed.
	/// \return	True if it succeeds or the manager is not concerned with the component, false if it fails.
	virtual bool executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component);

	/// Adds an representation to the manager. This will also add the representation to all of the groups
	/// contained in its groupRenferences.
	/// \param	representation	The representation to be added.
	/// \return	True if the representation was not in this manager and has been successfully added, false if it fails.
	virtual bool addRepresentation(std::shared_ptr<Representation> representation);

	/// Adds a view to the manager
	/// \param	view	The view to be added.
	/// \return	True if the view was not in this manager and has been successfully added, false if it fails.
	virtual bool addView(std::shared_ptr<View> view);

	/// Removes an representation from the manager
	/// \param	representation	The representation to be removed.
	/// \return	True if the representation was in this manager and has been successfully removed, false if it fails.
	virtual bool removeRepresentation(std::shared_ptr<Representation> representation);

	/// Removes a view from the manager
	/// \param	view	The view to be removed.
	/// \return	True if the view was in this manager and has been successfully removed, false if it fails.
	virtual bool removeView(std::shared_ptr<View> view);

	/// Performs an update for a single timestep
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual bool doUpdate(double dt);

	/// Overrides ComponentManager::getType()
	int getType() const override;

	/// Fetch a group with a given name, if the group does not exist, create it.
	/// \param name Name of the group to be fetched.
	/// \return group with the given name.
	virtual std::shared_ptr<Group> getOrCreateGroup(const std::string& name) = 0;

protected:
	/// Adds a group to the manager, override for manager specific behavior when adding
	/// \param group The group to be added.
	virtual void addGroup(std::shared_ptr<Group> group);

	void doBeforeStop() override;

private:



	/// Initializes the manager
	/// \return True if it succeeds, false if it fails
	virtual bool doInitialize();

	/// Starts up the manager after all threads have initialized
	/// \return True if it succeeds, false if it fails
	virtual bool doStartUp();

	/// Representations assigned to the manager
	std::vector<std::shared_ptr<Representation>> m_representations;
	/// Groups assigned to the manager
	std::unordered_map<std::string, std::shared_ptr<Group>> m_groups;
	/// Views assigned to the manager
	std::vector<std::shared_ptr<View>> m_views;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_MANAGER_H

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

#ifndef SURGSIM_GRAPHICS_OSGGROUP_H
#define SURGSIM_GRAPHICS_OSGGROUP_H

#include <SurgSim/Graphics/Group.h>

#include <osg/Group>
#include <osg/Switch>

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of a graphics group.
///
/// A Graphics::OsgGroup wraps a osg::Switch to provide group functionality.
class OsgGroup : public SurgSim::Graphics::Group
{
public:
	/// Constructor
	/// \param	name	Name of the group
	explicit OsgGroup(const std::string& name);

	/// Sets whether the group is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible);

	/// Gets whether the group is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const;

	/// Adds an actor
	/// \param	actor	Actor to add to this group
	/// \return	True if the actor is added successfully, false if failure
	virtual bool addActor(std::shared_ptr<Actor> actor);

	/// Removes an actor
	/// \param	actor	Actor to remove from this group
	/// \return	True if the actor is removed successfully, false if actor is not in this group or other failure
	virtual bool removeActor(std::shared_ptr<Actor> actor);

	/// Removes all actors
	virtual void clearActors();

	/// Adds a-group
	/// \param	group	Group to add as a sub-group of this group
	/// \return	True if the group is added successfully, false if failure
	virtual bool addGroup(std::shared_ptr<Group> group);

	/// Removes a-group
	/// \param	group	Sub-group to remove from this group
	/// \return	True if the group is removed successfully, false if not a sub-group of this group or other failure
	virtual bool removeGroup(std::shared_ptr<Group> group);

	/// Removes all groups
	virtual void clearGroups();

	/// Returns the root OSG group node
	osg::ref_ptr<osg::Group> getOsgGroup() const
	{
		return m_switch;
	}

private:
	/// Whether the group is currently visible or not
	/// Newly added actors or groups will have this visibility.
	bool m_isVisible;

	/// OSG group node
	/// A switch is used to provide visibility functionality.
	osg::ref_ptr<osg::Switch> m_switch;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGGROUP_H

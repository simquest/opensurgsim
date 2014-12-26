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

#include "SurgSim/Graphics/Group.h"

#include <osg/Group>
#include <osg/Switch>

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of a graphics group.
///
/// A Graphics::OsgGroup wraps a osg::Switch to provide group functionality.
class OsgGroup : public Group
{
public:
	/// Constructor
	/// \param	name	Name of the group
	explicit OsgGroup(const std::string& name);

	/// Sets whether this group is currently visible
	/// \param    visible    True for visible, false for invisible
	void setVisible(bool visible) override;

	/// Gets whether this group is currently visible
	/// \return    True for visible, false for invisible
	bool isVisible() const override;

	/// Adds an representation
	/// \param	representation	Representation to add to this group
	/// \return	True if the representation is added successfully, false if failure
	/// Only subclasses of OsgRepresentation will be added successfully.
	bool add(std::shared_ptr<Representation> representation) override;

	/// Adds all representations in another group to this group
	/// \param	group	Group of representations to add
	/// \return	True if all representations are added successfully, false if failure
	/// Only subclasses of OsgGroup will be appended successfully.
	bool append(std::shared_ptr<Group> group) override;

	/// Removes an representation
	/// \param	representation	Representation to remove from this group
	/// \return	True if the representation is removed successfully, false if representation is not in this group or
	///         other failure
	bool remove(std::shared_ptr<Representation> representation) override;

	/// Removes all representations
	void clear() override;

	/// Returns the root OSG group node
	osg::ref_ptr<osg::Group> getOsgGroup() const;

private:
	/// Whether the group is currently visible or not
	/// Newly added representations or groups will have this visibility.
	bool m_isVisible;

	/// OSG group node
	/// A switch is used to provide visibility functionality.
	osg::ref_ptr<osg::Switch> m_switch;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGGROUP_H

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

#ifndef SURGSIM_GRAPHICS_GROUP_H
#define SURGSIM_GRAPHICS_GROUP_H

#include "SurgSim/Framework/Component.h"

#include <memory>
#include <vector>

namespace SurgSim
{

namespace Graphics
{

class Representation;

/// Base graphics group class, which defines the interface that all graphics groups must implement.
///
/// Graphics::Group allows the organization of Graphics::Representation objects so that different algorithms can
/// operate on specific sub-sets rather than the entire scene.
class Group
{
public:
	/// Constructor. The group is initially empty.
	/// \param	name	Name of the group, this has to be unique over the whole system, otherwise
	/// 				adding the group will fail
	explicit Group(const std::string& name);

	/// Destructor
	virtual ~Group();

	/// Sets whether the group is currently visible
	/// \param    visible    True for visible, false for invisible
	virtual void setVisible(bool visible) = 0;

	/// Gets whether the group is currently visible
	/// \return    visible    True for visible, false for invisible
	virtual bool isVisible() const = 0;

	/// Adds an representation
	/// \param	representation	Representation to add to this group
	/// \return	True if the representation is added successfully, false if failure
	virtual bool add(std::shared_ptr<Representation> representation);

	/// Adds all representations in another group to this group
	/// \param	group	Group of representations to add
	/// \return	True if all representations are added successfully, false if failure
	virtual bool append(std::shared_ptr<Group> group);

	/// Removes an representation.
	/// \param	representation	Representation to remove from this group
	/// \return	True if the representation is removed successfully, false if representation is not in this group or
	/// other failure.
	virtual bool remove(std::shared_ptr<Representation> representation);

	/// \return a container with all the representations in this group.
	const std::vector<std::shared_ptr<Representation>>& getMembers() const;

	/// Removes all representations.
	virtual void clear();

	/// \return The name of this group.
	std::string getName() const;

private:

	std::string m_name;

	/// Representations in this group
	std::vector<std::shared_ptr<Representation>> m_representations;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_GROUP_H

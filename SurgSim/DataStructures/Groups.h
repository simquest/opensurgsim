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

#ifndef SURGSIM_DATASTRUCTURES_GROUPS_H
#define SURGSIM_DATASTRUCTURES_GROUPS_H

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/thread.hpp>

namespace SurgSim
{


namespace DataStructures
{

/// Class to wrap grouping operations, gives access to the members of a group and the groups of members.
/// Groups is threadsafe with regard to add and remove operations, and observations
/// \tparam Key the label to be used for the groups
/// \tparam T the type of the group members
template <typename Key, typename T>
class Groups
{
public:

	typedef Key IdentifierType;
	typedef T MemberType;

	/// Add an element to the given group, if the group doesn't exist it will be created, if the element
	/// is already a member of the group, nothing happens
	/// \param group the group to use
	/// \param element the element to add
	/// \return true if the element was actually added to the group
	bool add(const Key& group, const T& element);

	/// Add a member to the given groups, if any of the groups don't exist they will be created, if the element
	/// is already a member of a group, it won't be added to that specific group
	/// \param groups the groups to use
	/// \param element the element to add
	/// \return true if the element was added to at least one group
	bool add(const std::vector<Key>& groups, const T& element);

	/// Add all the members from the other group to this group, essentially forming a union of the two
	/// \param other object to add groups from
	/// \return true if at least one new element was added
	bool add(const Groups<Key, T>& other);

	/// Remove an element from a given group, if the group does not exist or the element is not a member of that
	/// group, nothing will happen.
	/// \param group the group to use
	/// \param element the element to remove
	/// \return true if the element was member of that group
	bool remove(const Key& group, const T& element);

	/// Remove an element from all known groups, if the element is not a member of any group, nothing happens
	/// \param element the element to remove
	/// \return true if there was an actual removal that was executed
	bool remove(const T& element);

	/// Return all the members of the given group
	/// \param group the group to query
	/// \return members of the given group, empty if the group has no members, or doesn't exist
	std::vector<T> getMembers(const Key& group) const;

	/// Return all the groups that the given member is a member of
	/// \param element the element to query
	/// \return groups which contain the given element, empty if the element is not member of any group
	std::vector<Key> getGroups(const T& element) const;

	/// \return all the known groups that have members
	std::vector<Key> getGroups() const;

	/// Return all the members of the given group
	/// \param group group to query
	/// \return members of the given group, empty if the group has no members
	std::vector<T> operator[](const Key& group) const;

	/// Erases all entries
	void clear();


private:

	typedef boost::shared_lock<boost::shared_mutex> SharedLock;
	typedef boost::unique_lock<boost::shared_mutex> UniqueLock;

	/// The mutex used to lock for reading and writing
	mutable boost::shared_mutex m_mutex;

	/// Map groups to members
	std::unordered_map<Key, std::unordered_set<T>> m_groups;

	/// Map members to groups
	std::unordered_map<T, std::unordered_set<Key>> m_membership;
};

}
}

#include "SurgSim/DataStructures/Groups-inl.h"

#endif

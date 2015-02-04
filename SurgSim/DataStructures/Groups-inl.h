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

#ifndef SURGSIM_DATASTRUCTURES_GROUPS_INL_H
#define SURGSIM_DATASTRUCTURES_GROUPS_INL_H

template <typename Key, typename T>
bool SurgSim::DataStructures::Groups<Key, T>::add(const Key& group, const T& value)
{
	UniqueLock lock(m_mutex);
	auto result = m_groups[group].insert(value);
	if (result.second == true)
	{
		m_membership[value].insert(group);
	}
	return result.second;
}


template <typename Key, typename T>
bool SurgSim::DataStructures::Groups<Key, T>::add(const std::vector<Key>& groups, const T& member)
{
	bool result = false;
	for (auto& group : groups)
	{
		result = add(group, member) || result;
	}
	return result;
}

template <typename Key, typename T>
bool SurgSim::DataStructures::Groups<Key, T>::remove(const Key& group, const T& value)
{
	bool result = false;
	UniqueLock lock(m_mutex);
	auto found = m_groups.find(group);
	if (found != m_groups.end())
	{
		auto count = found->second.erase(value);
		if (count > 0)
		{
			if (found->second.empty())
			{
				m_groups.erase(group);
			}
			m_membership[value].erase(group);
			result = true;
		}
	}
	return result;
}

template <typename Key, typename T>
std::vector<T> SurgSim::DataStructures::Groups<Key, T>::getMembers(const Key& group) const
{
	std::vector<T> result;
	SharedLock lock(m_mutex);
	auto found = m_groups.find(group);
	if (found != m_groups.end())
	{
		std::vector<T> temp(found->second.cbegin(), found->second.cend());
		std::swap(result, temp);
	}
	return std::move(result);
}

template <typename Key, typename T>
std::vector<Key> SurgSim::DataStructures::Groups<Key, T>::getGroups(const T& value) const
{
	std::vector<Key> result;
	SharedLock lock(m_mutex);
	auto found = m_membership.find(value);
	if (found != m_membership.end())
	{
		std::vector<Key> temp(found->second.cbegin(), found->second.cend());
		std::swap(result, temp);
	}
	return std::move(result);
}

template <typename Key, typename T>
std::vector<Key> SurgSim::DataStructures::Groups<Key, T>::getGroups() const
{
	std::unordered_set<Key> groups;
	{
		SharedLock lock(m_mutex);
		std::for_each(m_groups.cbegin(), m_groups.cend(),
					  [&groups](const std::pair<Key, std::unordered_set<T>>& value)
		{
			groups.insert(value.first);
		});
	}
	std::vector<Key> result(groups.begin(), groups.end());
	std::copy(groups.begin(), groups.end(), result.begin());
	return std::move(result);
}

template <typename Key, typename T>
bool SurgSim::DataStructures::Groups<Key, T>::remove(const T& value)
{
	bool result = false;
	UniqueLock lock(m_mutex);
	if (m_membership.find(value) != m_membership.end())
	{
		for (auto group : m_membership[value])
		{
			m_groups[group].erase(value);
		}
		m_membership.erase(value);
		result = true;
	}
	return result;
}

template <typename Key, typename T>
std::vector<T> SurgSim::DataStructures::Groups<Key, T>::operator[](const Key& group) const
{
	// Get member does the locking, not needed here
	return getMembers(group);
}


#endif
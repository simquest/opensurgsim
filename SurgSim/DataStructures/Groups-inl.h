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

namespace SurgSim
{
namespace DataStructures
{

template <typename Key, typename T>
bool Groups<Key, T>::add(const Key& group, const T& element)
{
	UniqueLock lock(m_mutex);
	auto result = m_groups[group].insert(element);
	if (result.second == true)
	{
		m_membership[element].insert(group);
	}
	return result.second;
}


template <typename Key, typename T>
bool Groups<Key, T>::add(const std::vector<Key>& groups, const T& element)
{
	bool result = false;
	for (auto& group : groups)
	{
		result = add(group, element) || result;
	}
	return result;
}

template <typename Key, typename T>
bool Groups<Key, T>::remove(const Key& group, const T& element)
{
	bool result = false;
	UniqueLock lock(m_mutex);
	auto found = m_groups.find(group);
	if (found != m_groups.end())
	{
		auto count = found->second.erase(element);
		if (count > 0)
		{
			if (found->second.empty())
			{
				m_groups.erase(group);
			}

			m_membership[element].erase(group);

			if (m_membership[element].empty())
			{
				m_membership.erase(element);
			}

			result = true;
		}
	}
	return result;
}

template <typename Key, typename T>
std::vector<T> Groups<Key, T>::getMembers(const Key& group) const
{
	std::vector<T> result;
	SharedLock lock(m_mutex);
	auto found = m_groups.find(group);
	if (found != m_groups.end())
	{
		result.assign(found->second.cbegin(), found->second.cend());
	}
	return std::move(result);
}

template <typename Key, typename T>
std::vector<Key> Groups<Key, T>::getGroups(const T& element) const
{
	std::vector<Key> result;
	SharedLock lock(m_mutex);
	auto found = m_membership.find(element);
	if (found != m_membership.end())
	{
		result.assign(found->second.cbegin(), found->second.cend());
	}
	return std::move(result);
}

template <typename Key, typename T>
std::vector<Key> Groups<Key, T>::getGroups() const
{
	std::vector<Key> result;
	{
		SharedLock lock(m_mutex);
		std::for_each(m_groups.cbegin(), m_groups.cend(),
					  [&result](const std::pair<Key, std::unordered_set<T>>& value)
		{
			result.emplace(result.end(), value.first);
		});
	}
	return std::move(result);
}

template <typename Key, typename T>
bool Groups<Key, T>::remove(const T& element)
{
	bool result = false;
	UniqueLock lock(m_mutex);
	if (m_membership.find(element) != m_membership.end())
	{
		for (auto& group : m_membership[element])
		{
			m_groups[group].erase(element);
		}
		m_membership.erase(element);
		result = true;
	}
	return result;
}

template <typename Key, typename T>
std::vector<T> Groups<Key, T>::operator[](const Key& group) const
{
	// Get member does the locking, not needed here
	return getMembers(group);
}

}
}

#endif
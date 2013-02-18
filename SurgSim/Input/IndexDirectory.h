// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_INPUT_INDEX_DIRECTORY_H
#define SURGSIM_INPUT_INDEX_DIRECTORY_H

#include <string>
#include <vector>
#include <unordered_map>

namespace SurgSim
{
namespace Input
{

/// A simple bidirectional mapping between names (strings) and distinct consecutive non-negative indices.
///
/// Access to this class is thread-safe if all of the threads are only performing const operations, i.e. reading the names and indices.
class IndexDirectory
{
public:
	/// Create an empty directory object.
	IndexDirectory() {};

	/// Create a directory object initialized to a list of names.
	IndexDirectory(const std::vector<std::string>& names)
	{
		for (auto it = names.cbegin();  it != names.cend();  ++it)
		{
			addEntry(*it);
		}
	}

	/// Given a name, return the index (or -1).
	int getIndex(const std::string& name) const
	{
		if (name.length() == 0)
		{
			return -1;
		}
		auto entry = m_indices.find(name);
		if (entry == m_indices.cend())
		{
			return -1;
		}
		else
		{
			return entry->second;
		}
	}

	/// Given an index, return the name (or "").
	std::string getName(int index) const
	{
		if ((index < 0) || (index >= static_cast<int>(m_names.size())))
		{
			return "";
		}
		else
		{
			return m_names[index];
		}
	}

	/// Get a list of all the names available from the index directory.
	const std::vector<std::string>& getAllNames() const
	{
		return m_names;
	}

	/// Check whether the specified name exists in the directory.
	bool hasEntry(const std::string& name) const
	{
		return ((name.length() > 0) && (m_indices.count(name) > 0));
	}

	/// Check the number of existing entries in the directory.
	size_t size() const
	{
		return m_names.size();
	}

	/// Check the number of existing entries in the directory.
	int getNumEntries() const
	{
		return static_cast<int>(m_names.size());
	}

protected:
	template <typename T>
	friend class NamedDataBuilder;
	friend class DataGroupBuilder;

	/// Copy constructor.
	/// Not directly accessible by external code, but but could be used by subclasses.
	IndexDirectory(const IndexDirectory& directory)
		: m_names(directory.m_names), m_indices(directory.m_indices)
	{
	}

	/// Assignment operator.
	/// Not directly accessible by external code, but could be used by subclasses.
	IndexDirectory& operator =(const IndexDirectory& directory)
	{
		m_names = directory.m_names;
		m_indices = directory.m_indices;
		return *this;
	}

	/// Create a new entry for the specified name.
	int addEntry(const std::string& name)
	{
		if ((name.length() == 0) || (m_indices.count(name) > 0))
		{
			return -1;
		}
		int index = static_cast<int>(m_names.size());
		m_names.push_back(name);
		m_indices[name] = index;
		return index;
	}

private:
	/// The array of entry names, in index order.
	std::vector<std::string> m_names;

	/// A mapping of entry names to indices.
	std::unordered_map<std::string, int> m_indices;
};

};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_INPUT_INDEX_DIRECTORY_H

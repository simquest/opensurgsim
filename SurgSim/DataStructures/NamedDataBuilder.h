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

#ifndef SURGSIM_DATA_STRUCTURES_NAMED_DATA_BUILDER_H
#define SURGSIM_DATA_STRUCTURES_NAMED_DATA_BUILDER_H

#include <memory>

#include "SurgSim/DataStructures/NamedData.h"

namespace SurgSim
{
namespace DataStructures
{

/// A class that allows you to build a \ref NamedData structure.
///
/// Since the data layout of a \ref NamedData object cannot be modified, this class can be helpful in initially
/// setting up the names and their corresponding indices.  You can add entries to the builder using \ref addEntry
/// and \ref addEntriesFrom calls, then create the NamedData instance with createData() or createSharedData().
///
/// \sa NamedData
template <typename T>
class NamedDataBuilder
{
public:
	/// Constructs an empty builder object.
	NamedDataBuilder() {};

	/// Produces a \ref NamedData object with an immutable set of names and indices.
	/// None of the values will contain any current data.
	/// \return the NamedData object *by value*.
	NamedData<T> createData() const
	{
		// NB: can't use copy construction in the std::make_shared call, because access is protected.
		std::shared_ptr<IndexDirectory> dir = std::make_shared<IndexDirectory>();
		*dir = m_directory;
		return NamedData<T>(dir);
	}

	/// Produces a shared pointer to an empty \ref NamedData object with an immutable set of names and indices.
	/// None of the values will contain any current data.
	/// \return a shared pointer to the NamedData object.
	std::shared_ptr<NamedData<T>> createSharedData() const
	{
		return std::make_shared<NamedData<T>>(createData());
	}

	/// Creates a new entry for the specified name.
	///
	/// \param name The name, which should be non-empty and should not already exist in the data.
	/// \return the index of the created entry, or -1 if the entry could not be added.
	int addEntry(const std::string& name)
	{
		return m_directory.addEntry(name);
	}

	/// Create new entries from a vector of names.
	/// \param names The names.
	void addEntriesFrom(const std::vector<std::string>& names)
	{
		for (auto it = names.cbegin();  it != names.cend();  ++it)
		{
			addEntry(*it);
		}
	}

	/// Create new entries from another NamedDataBuilder.
	/// \tparam typename U The data type of the other NamedDataBuilder.
	/// \param builder The other builder.
	template <typename U>
	void addEntriesFrom(const NamedDataBuilder<U>& builder)
	{
		addEntriesFrom(builder.getAllNames());
	}

	/// Create new entries from an already initialized NamedData.
	/// \tparam typename U The data type of the NamedData.
	/// \param data The data object.
	template <typename U>
	void addEntriesFrom(const NamedData<U>& data)
	{
		addEntriesFrom(data.getDirectory()->getAllNames());
	}

	/// Create new entries from an IndexDirectory.
	/// \param directory The index directory object.
	void addEntriesFrom(const IndexDirectory& directory)
	{
		addEntriesFrom(directory.getAllNames());
	}

	/// Given a name, return the corresponding index (or -1).
	/// \param name The name.
	/// \return the index for that name if one exists; -1 otherwise.
	int getIndex(const std::string& name) const
	{
		return m_directory.getIndex(name);
	}

	/// Given an index, return the corresponding name (or "").
	/// \param index The index.
	/// \return the name for that index if one exists; an empty string otherwise.
	std::string getName(int index) const
	{
		return m_directory.getName(index);
	}

	/// Get a list of all the names available in the builder.
	/// \return all the names.
	const std::vector<std::string>& getAllNames() const
	{
		return m_directory.getAllNames();
	}

	/// Check whether the specified name exists in the builder.
	///
	/// \param name The name.
	/// \return true if the entry exists.
	bool hasEntry(const std::string& name) const
	{
		return m_directory.hasEntry(name);
	}

	/// Check the number of existing entries in the builder.
	/// \return the number of entries.
	/// \sa getNumEntries()
	size_t size() const
	{
		return m_directory.size();
	}

	/// Check the number of existing entries in the builder.
	/// \return the number of entries.
	/// \sa size()
	int getNumEntries() const
	{
		return m_directory.getNumEntries();
	}

private:
	/// The mapping between names and indices that will be used to create the NamedData instance.
	IndexDirectory m_directory;
};

};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_NAMED_DATA_BUILDER_H

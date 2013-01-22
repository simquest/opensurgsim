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

#ifndef SURGSIM_INPUT_NAMED_DATA_BUILDER_H
#define SURGSIM_INPUT_NAMED_DATA_BUILDER_H

#include <memory>

#include "SurgSim/Input/NamedData.h"

namespace SurgSim
{
namespace Input
{

/// A class that allows you to build an IndexDirectory structure.
/// Read-only access to this class is thread-safe if all of the threads are only performing read (const) operations.
template <typename T>
class NamedDataBuilder : protected IndexDirectory
{
public:
	/// Create an empty directory object.
	NamedDataBuilder() {};

	/// Produce an empty \ref NamedData object with an immutable directory.
	/// \return the NamedData object *by value*.
	NamedData<T> createData() const
	{
		// NB: can't use copy construction in the std::make_shared call, because access is protected.
		std::shared_ptr<IndexDirectory> dir = std::make_shared<IndexDirectory>();
		*dir = *this;
		return NamedData<T>(dir);
	}

	/// Produce a shared pointer to an empty \ref NamedData object with an immutable directory.
	/// \return a shared pointer to the NamedData object.
	std::shared_ptr<NamedData<T>> createSharedData() const
	{
		return std::make_shared<NamedData<T>>(createData());
	}

	/// Create a new entry for the specified name.
	int addEntry(const std::string& name)
	{
		return IndexDirectory::addEntry(name);
	}

	/// Create new entries from a vector of strings.
	void addEntries(const std::vector<std::string>& names)
	{
		for (auto iter = names.cbegin();  iter != names.cend();  ++iter)
		{
			addEntry(*iter);
		}
	}

	/// Create new entries from an NamedDataBuilder.
	template <typename U>
	void addEntries(const NamedDataBuilder<U>& builder)
	{
		addEntries(builder.getAllNames());
	}

	/// Create new entries from an IndexDirectory.
	void addEntries(const IndexDirectory& directory)
	{
		addEntries(directory.getAllNames());
	}

	// The forwarding proxies below are needed because we don't inherit from IndexBuilder publicly.
	// We can just inherit their Doxygen documentation from the base class.

	int getIndex(const std::string& name) const
	{
		return IndexDirectory::getIndex(name);
	}

	std::string getName(int index) const
	{
		return IndexDirectory::getName(index);
	}

	const std::vector<std::string>& getAllNames() const
	{
		return IndexDirectory::getAllNames();
	}

	bool hasEntry(const std::string& name) const
	{
		return IndexDirectory::hasEntry(name);
	}

	int getNumEntries() const
	{
		return IndexDirectory::getNumEntries();
	}
};

};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_INPUT_NAMED_DATA_BUILDER_H

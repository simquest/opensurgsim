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

#ifndef SURGSIM_INPUT_NAMED_DATA_H
#define SURGSIM_INPUT_NAMED_DATA_H

#include <memory>
#include <string>

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Input/IndexDirectory.h>

namespace SurgSim
{
namespace Input
{

/// An array of value entries that can be accessed by name or index.
/// Each entry can also be marked as currently valid or missing.
template <typename T>
class NamedData
{
public:
	/// Create an empty object, with no associated directory yet.
	NamedData() {};

	/// Create an object containing items from an index directory.
	/// You should probably use \ref NamedDataBuilder or copy construction/assignment instead.
	NamedData(std::shared_ptr<const IndexDirectory> directory)
		: m_directory(directory)
	{
		m_data.resize(m_directory->getNumEntries());
		m_isCurrent.resize(m_directory->getNumEntries(), false);
		SURGSIM_ASSERT(isValid());
	}

	/// Create an object and copy the data from another object.
	NamedData(const NamedData& namedData)
		: m_directory(namedData.m_directory),
		  m_data(namedData.m_data),
		  m_isCurrent(namedData.m_isCurrent)
	{
		SURGSIM_ASSERT(isValid());
	}

	/// Copy the data from another object.
	NamedData& operator=(const NamedData& namedData)
	{
		SURGSIM_ASSERT(namedData.isValid()) <<
			"Can't use an invalid (empty) NamedData on the right-hand side of an assignment!";

		if (! isValid())
		{
			m_directory = namedData.m_directory;
		}
		else
		{
			SURGSIM_ASSERT(m_directory == namedData.m_directory) << "Incompatible NamedData contents in assignment!";
		}

		m_data = namedData.m_data;
		m_isCurrent = namedData.m_isCurrent;

		SURGSIM_ASSERT(isValid()) << "NamedData isn't valid after assignment!";
		SURGSIM_ASSERT(m_data.size() == m_directory->size() && m_isCurrent.size() == m_directory->size()) <<
			"NamedData isn't correctly sized after assignment!";

		return *this;
	}

	/// Create an object and move the data from another object.
	NamedData(NamedData&& namedData)
		: m_directory(std::move(namedData.m_directory)),
		  m_data(std::move(namedData.m_data)),
		  m_isCurrent(std::move(namedData.m_isCurrent))
	{
	}

	/// Move the data from another object.
	NamedData& operator=(NamedData&& namedData)
	{
		SURGSIM_ASSERT(namedData.isValid()) <<
			"Can't use an invalid (empty) NamedData on the right-hand side of an assignment!";

		if (! isValid())
		{
			m_directory = std::move(namedData.m_directory);
		}
		else
		{
			SURGSIM_ASSERT(m_directory == namedData.m_directory) << "Incompatible NamedData contents in assignment!";
		}
		
		m_data = std::move(namedData.m_data);
		m_isCurrent = std::move(namedData.m_isCurrent);

		SURGSIM_ASSERT(isValid()) << "NamedData isn't valid after assignment!";
		SURGSIM_ASSERT(m_data.size() == m_directory->size() && m_isCurrent.size() == m_directory->size()) <<
			"NamedData isn't correctly sized after assignment!";
		
		return *this;
	}

	/// Check if the object is valid, meaning it has a valid directory.
	/// If the object isn't valid, it can become valid on assignment from a valid object.
	bool isValid() const
	{
		return static_cast<bool>(m_directory);
	}

	/// Return the object's directory.
	std::shared_ptr<const IndexDirectory> getDirectory() const
	{
		return m_directory;
	}

	/// The object contains an entry with the specified index.
	bool hasEntry(int index) const
	{
		return ((index >= 0) && (index < static_cast<int>(m_data.size())));
	}

	/// The object contains an entry with the specified name.
	bool hasEntry(const std::string& name) const
	{
		return m_directory->hasEntry(name);
	}

	/// The object contains current data for the entry with the specified index.
	bool hasCurrentData(int index) const
	{
		return hasEntry(index) && m_isCurrent[index];
	}

	/// The object contains current data for the entry with the specified name.
	bool hasCurrentData(const std::string& name) const
	{
		int index =  m_directory->getIndex(name);
		if (index < 0)
		{
			return false;
		}
		else
		{
			SURGSIM_ASSERT(hasEntry(index));
			return m_isCurrent[index];
		}
	}

	/// Given an index, return the corresponding value.
	bool get(int index, T& value) const
	{
		if (! hasCurrentData(index))
		{
			return false;
		}
		else
		{
			value = m_data[index];
			return true;
		}
	}

	/// Given a name, return the corresponding value.
	bool get(const std::string& name, T& value) const
	{
		int index =  m_directory->getIndex(name);
		if ((index < 0) || ! m_isCurrent[index])
		{
			return false;
		}
		else
		{
			// XXX assert(hasEntry(index));
			value = m_data[index];
			return true;
		}
	}

	/// Record the data for an entry specified by an index.
	bool put(int index, const T& value)
	{
		if (! hasEntry(index))
		{
			return false;
		}
		else
		{
			m_data[index] = value;
			m_isCurrent[index] = true;
			return true;
		}
	}

	/// Record the data for an entry specified by a name.
	bool put(const std::string& name, const T& value)
	{
		int index =  m_directory->getIndex(name);
		if (index < 0)
		{
			return false;
		}
		else
		{
			// XXX assert(hasEntry(index));
			m_data[index] = value;
			m_isCurrent[index] = true;
			return true;
		}
	}

	/// Mark all data as not current.
	void reset()
	{
		m_isCurrent.assign(m_data.size(), false);
	}

private:
	/// The mapping between names and indices.
	std::shared_ptr<const IndexDirectory> m_directory;

	/// The array of values.
	std::vector<T> m_data;

	/// The array storing whether the data is current.
	std::vector<bool> m_isCurrent;
};

};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_INPUT_NAMED_DATA_H

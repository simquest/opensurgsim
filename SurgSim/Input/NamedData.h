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
#include <vector>

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Input/IndexDirectory.h>

namespace SurgSim
{
namespace Input
{

/// A collection of value entries that can be accessed by name or index.
///
/// A NamedData object contains a collection of values of type \a T.  Each entry in the collection can be
/// accessed by using either its unique name (a std::string) or its unique index (a non-negative integer).
/// Access by name is more convenient, but also less efficient.
///
/// A NamedData object constructed by the default constructor starts out empty, meaning it has not yet been
/// associated with a set of names and indices.  A <i>non</i>-empty object contains a fixed set of value entries;
/// entries <b>cannot be added or removed</b>, and names and indices of existing entries
/// <b>cannot be changed</b>.  A non-empty object also cannot ever become empty again.  These properties ensure
/// that a stable data layout is available to the code using this class so that it can, for example, record
/// entry indices and use them to retrieve the same entries later on.
///
/// However, each entry can be marked as not currently valid, i.e. missing.  Its entry still remains in the
/// collection, but for the moment has no value associated with it.
///
/// The set of names and indices within a NamedData object object cannot be modified, but it can be initialized
/// by passing in a vector of names to the constructor.  Alternately, you can initialize the data layout using
/// the \ref NamedDataBuilder class.  After doing that, you can create other objects with the same layout by
/// copy construction, or by assigning the initialized value to an empty (default-constructed) NamedData object.
///
/// \tparam T the data type used for values contained in this collection.
template <typename T>
class NamedData
{
public:
	/// Create an empty object, with no associated names and indices yet.
	NamedData() {};

	/// Create an object containing items from an index directory.
	/// You should probably use \ref NamedDataBuilder or copy construction/assignment instead.
	///
	/// \param directory The IndexDirectory object describing the names and indices to be used.
	NamedData(std::shared_ptr<const IndexDirectory> directory)
		: m_directory(directory)
	{
		m_data.resize(m_directory->getNumEntries());
		m_isCurrent.resize(m_directory->getNumEntries(), false);
		SURGSIM_ASSERT(isValid());
	}

	/// Construct an object, using the names from a vector of strings.
	/// The indices corresponding to each name's entry will be the same as that name's index in the vector.
	///
	/// \param names The names, which should be unique.
	NamedData(const std::vector<std::string>& names)
		: m_directory(std::make_shared<const IndexDirectory>(names))
	{
		m_data.resize(m_directory->getNumEntries());
		m_isCurrent.resize(m_directory->getNumEntries(), false);
		SURGSIM_ASSERT(isValid());
	}

	/// Construct an object as a copy of the data from another object.
	/// \param namedData The object to copy from.
	NamedData(const NamedData& namedData)
		: m_directory(namedData.m_directory),
		  m_data(namedData.m_data),
		  m_isCurrent(namedData.m_isCurrent)
	{
		SURGSIM_ASSERT(isValid());
	}

	/// Copy the data from another object.
	///
	/// The object being assigned into must either be empty (not yet associated with a set of names and indices), or
	/// the two objects must share the same data layout, resulting from earlier copy construction or assignment.
	/// ~~~~~
	/// DataGroup initial;
	/// // ...initialize "initial" to some non-empty value...
	/// NamedData copyConstructed(initial);  // Layout is shared with initial
	/// copyConstructed = initial            // OK, using the same layout
	/// NamedData another;                   // Object is empty (no layout)
	/// another = initial;                   // OK, layout is now shared with initial
	/// another = initial                    // OK, using the same layout
	/// ~~~~~
	///
	/// Note that the data layout must be the same, i.e. related to one another by object assignment or copy
	/// construction.  Objects that merely contain entries with the same names and indices are not acceptable!
	/// (Otherwise, we'd need to inefficiently compare layout contents each time we assign.)
	/// ~~~~~
	/// std::vector<std::string> names // = ...initialized to some value...;
	/// NamedData first(names);   // Layout of entries created from names
	/// NamedData second(names);  // Another layout of entries created from names; names and indices match
	/// second = first;           // ERROR at run-time, layouts were created separately!
	/// ~~~~~
	///
	/// \param namedData The object to copy from.
	/// \return The object that was assigned into.
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
	///
	/// \param [in,out] namedData The object to copy from, which will be left in an ununsable state.
	NamedData(NamedData&& namedData)
		: m_directory(std::move(namedData.m_directory)),
		  m_data(std::move(namedData.m_data)),
		  m_isCurrent(std::move(namedData.m_isCurrent))
	{
	}

	/// Move the data from another object.
	///
	/// The same restrictions on object compatibility apply as in the case of the copy assignment operator=(const NamedData&).
	///
	/// \param [in,out] namedData The object to copy from, which will be left in an ununsable state.
	/// \return The object that was assigned into.
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

	/// Check if the object is valid (non-empty), meaning it is associated with a set of names and indices.
	/// If the object is empty, it can become valid on assignment from a valid object.
	///
	/// \return true if valid, false if empty.
	bool isValid() const
	{
		return static_cast<bool>(m_directory);
	}

	/// Return the object's layout directory, which is its collection of names and indices.
	/// In most cases, you should use direct assignment instead of doing things via the directory.
	/// \return The IndexDirectory object containing the names and indices of entries.
	std::shared_ptr<const IndexDirectory> getDirectory() const
	{
		return m_directory;
	}

	/// Check whether the object contains an entry with the specified index.
	///
	/// \param index The index corresponding to the entry.
	/// \return true if that entry exists, false if not.
	bool hasEntry(int index) const
	{
		return ((index >= 0) && (index < static_cast<int>(m_data.size())));
	}

	/// Check whether the object contains an entry with the specified name.
	///
	/// \param name The name corresponding to the entry.
	/// \return true if that entry exists, false if not.
	bool hasEntry(const std::string& name) const
	{
		return m_directory->hasEntry(name);
	}

	/// Check whether the object contains current data for the entry with the specified index.
	///
	/// \param index The index of the entry.
	/// \return true if that entry exists and contains current data.
	bool hasCurrentData(int index) const
	{
		return hasEntry(index) && m_isCurrent[index];
	}

	/// Check whether the object contains current data for the entry with the specified name.
	///
	/// \param name The name of the entry.
	/// \return true if that entry exists and contains current data.
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

	/// Given an index, get the corresponding value.
	///
	/// \param index The index of the entry.
	/// \param [out] value The retrieved value.
	/// \return true if a current value is available and was written to \a value.
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

	/// Given a name, get the corresponding value.
	///
	/// \param name The name of the entry.
	/// \param [out] value The retrieved value.
	/// \return true if a current value is available and was written to \a value.
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
	/// The entry will also be marked as containing current data.
	///
	/// \param index The index of the entry.
	/// \param value The value to be set.
	/// \return true if successful.
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
	/// The entry will also be marked as containing current data.
	///
	/// \param name The name of the entry.
	/// \param value The value to be set.
	/// \return true if successful.
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

	/// Mark all of the data as not current.
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

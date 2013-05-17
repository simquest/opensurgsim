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

#ifndef SURGSIM_DATASTRUCTURES_NAMEDDATA_H
#define SURGSIM_DATASTRUCTURES_NAMEDDATA_H

#include <memory>
#include <string>
#include <vector>

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/DataStructures/IndexDirectory.h>

namespace SurgSim
{
namespace DataStructures
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
/// However, each entry can be marked as not valid right now, i.e. missing.  Its entry still remains in the
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
	inline NamedData();

	/// Create an object containing items from an index directory.
	/// You should probably use \ref NamedDataBuilder or copy construction/assignment instead.
	///
	/// \param directory The IndexDirectory object describing the names and indices to be used.
	inline explicit NamedData(std::shared_ptr<const IndexDirectory> directory);

	/// Construct an object, using the names from a vector of strings.
	/// The indices corresponding to each name's entry will be the same as that name's index in the vector.
	///
	/// \param names The names, which should be unique.
	inline explicit NamedData(const std::vector<std::string>& names);

	/// Construct an object as a copy of the data from another object.
	/// \param namedData The object to copy from.
	inline NamedData(const NamedData& namedData);

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
	inline NamedData& operator=(const NamedData& namedData);

	/// Create an object and move the data from another object.
	///
	/// \param [in,out] namedData The object to copy from, which will be left in an unusable state.
	inline NamedData(NamedData&& namedData);

	/// Move the data from another object.
	///
	/// The same restrictions on object compatibility apply as in the case of the copy assignment
	/// operator=(const NamedData&).
	///
	/// \param [in,out] namedData The object to copy from, which will be left in an unusable state.
	/// \return The object that was assigned into.
	inline NamedData& operator=(NamedData&& namedData);

	/// Check if the object is valid (non-empty), meaning it is associated with a set of names and indices.
	/// If the object is empty, it can become valid on assignment from a valid object.
	///
	/// \return true if valid, false if empty.
	inline bool isValid() const;

	/// Return the object's layout directory, which is its collection of names and indices.
	/// In most cases, you should use direct assignment instead of doing things via the directory.
	/// \return The IndexDirectory object containing the names and indices of entries.
	inline std::shared_ptr<const IndexDirectory> getDirectory() const;

	/// Given a name, return the corresponding index (or -1).
	/// \param name The name.
	/// \return the index for that name if one exists; -1 otherwise.
	inline int getIndex(const std::string& name) const;

	/// Given an index, return the corresponding name (or "").
	/// \param index The index.
	/// \return the name for that index if one exists; an empty string otherwise.
	inline std::string getName(int index) const;

	/// Check whether the object contains an entry with the specified index.
	/// Logically equivalent to <code>getName(index) != ""</code>.
	///
	/// \param index The index corresponding to the entry.
	/// \return true if that entry exists, false if not.
	inline bool hasEntry(int index) const;

	/// Check whether the object contains an entry with the specified name.
	/// Logically equivalent to <code>getIndex(name) != -1</code>.
	///
	/// \param name The name corresponding to the entry.
	/// \return true if that entry exists, false if not.
	inline bool hasEntry(const std::string& name) const;

	/// Check whether the entry with the specified index contains valid data.
	/// The check verifies that the entry's data was %set using set(int, const T&) or
	/// set(const std::string&, const T&), without being subsequently invalidated by reset(int)
	/// or reset(const std::string&).
	///
	/// \param index The index of the entry.
	/// \return true if that entry exists and contains valid data.
	inline bool hasData(int index) const;

	/// Check whether the entry with the specified name contains valid data.
	/// The check verifies that the entry's data was %set using set(int, const T&) or
	/// set(const std::string&, const T&), without being subsequently invalidated by reset(int)
	/// or reset(const std::string&).
	///
	/// \param name The name of the entry.
	/// \return true if that entry exists and contains valid data.
	inline bool hasData(const std::string& name) const;

	/// Given an index, get the corresponding value.
	/// It's only possible to get the value if the data was %set using set(int, const T&) or
	/// set(const std::string&, const T&), without being subsequently invalidated by reset(int)
	/// or reset(const std::string&).  In other words, get returns the same value as hasData would return.
	///
	/// \param index The index of the entry.
	/// \param [out] value The location for the retrieved value.  Must not be null.
	/// \return true if a valid value is available and was written to \a value.
	inline bool get(int index, T* value) const;

	/// Given a name, get the corresponding value.
	/// It's only possible to get the value if the data was %set using set(int, const T&) or
	/// set(const std::string&, const T&), without being subsequently invalidated by reset(int)
	/// or reset(const std::string&).  In other words, get returns the same value as hasData would return.
	///
	/// \param name The name of the entry.
	/// \param [out] value The location for the retrieved value.  Must not be null.
	/// \return true if a valid value is available and was written to \a value.
	inline bool get(const std::string& name, T* value) const;

	/// Record the data for an entry specified by an index.
	/// The entry will also be marked as containing valid data.
	///
	/// \param index The index of the entry.
	/// \param value The value to be set.
	/// \return true if successful.
	inline bool set(int index, const T& value);

	/// Record the data for an entry specified by a name.
	/// The entry will also be marked as containing valid data.
	///
	/// \param name The name of the entry.
	/// \param value The value to be set.
	/// \return true if successful.
	inline bool set(const std::string& name, const T& value);

	/// Invalidate an entry&mdash; mark it as not containing any valid data.
	///
	/// \param index The index of the entry.
	/// \return true if successful.
	inline bool reset(int index);

	/// Invalidate an entry&mdash; mark it as not containing any valid data.
	///
	/// \param name The name of the entry.
	/// \return true if successful.
	inline bool reset(const std::string& name);

	/// Invalidate all entries&mdash; mark everything as not containing any valid data.
	inline void resetAll();

	/// Check the number of existing entries.
	/// \return the size of the data collection.
	/// \sa getNumEntries()
	inline size_t size() const;

	/// Check the number of existing entries.
	/// \return the size of the data collection.
	/// \sa size()
	inline int getNumEntries() const;

private:
	/// The mapping between names and indices.
	std::shared_ptr<const IndexDirectory> m_directory;

	/// The array of values.
	std::vector<T> m_data;

	/// The array storing whether the data is currently valid.
	std::vector<bool> m_isDataValid;
};


};  // namespace Input
};  // namespace SurgSim


#include <SurgSim/DataStructures/NamedData-inl.h>


#endif  // SURGSIM_DATASTRUCTURES_NAMEDDATA_H

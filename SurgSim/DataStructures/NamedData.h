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

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/DataStructures/IndexDirectory.h"

namespace SurgSim
{
namespace DataStructures
{
/// The type used for copying values between two NamedData objects that cannot assign to each other. The keys are
/// the source indices.  The values are the target indices.
typedef std::unordered_map<int, int> NamedDataCopyMap;

/// Common strings for NamedData.

namespace Names
{
static const char* const BUTTON_0 = "button0";
static const char* const BUTTON_1 = "button1";
static const char* const BUTTON_2 = "button2";
static const char* const BUTTON_3 = "button3";
static const char* const BUTTON_4 = "button4";
static const char* const TOOLDOF = "toolDof";

static const char* const POSE = "pose";
static const char* const INPUT_POSE = "inputPose";

static const char* const ANGULAR_VELOCITY = "angularVelocity";
static const char* const LINEAR_VELOCITY = "linearVelocity";
static const char* const INPUT_ANGULAR_VELOCITY = "inputAngularVelocity";
static const char* const INPUT_LINEAR_VELOCITY = "inputLinearVelocity";

static const char* const FORCE = "force";
static const char* const TORQUE = "torque";

static const char* const DAMPER_JACOBIAN = "damperJacobian";
static const char* const SPRING_JACOBIAN = "springJacobian";

static const char* const IS_HOMED = "isHomed";
static const char* const IS_ORIENTATION_HOMED = "isOrientationHomed";
static const char* const IS_POSITION_HOMED = "isPositionHomed";

static const char* const DIGITAL_INPUT_PREFIX = "digitalInput";
static const char* const DIGITAL_OUTPUT_PREFIX = "digitalOutput";
static const char* const TIMER_INPUT_PREFIX = "timerInput";
static const char* const TIMER_OUTPUT_PREFIX = "timerOutput";
static const char* const ANALOG_INPUT_PREFIX = "analogInput";
static const char* const ANALOG_OUTPUT_PREFIX = "analogOutput";

static const char* const PROJECTION_MATRIX = "projectionMatix";
static const char* const LEFT_PROJECTION_MATRIX = "leftProjectionMatix";
static const char* const RIGHT_PROJECTION_MATRIX = "rightProjectionMatix";

static const char* const KEY = "key";
};

/// A templated dictionary in which data can be accessed by name or index, with immutable names & indices.
///
/// A NamedData object consists of a collection of entries of type \a T.  The data value for each entry can be accessed
/// by either the entry's unique name (a std::string) or the entry's unique index (a non-negative integer).  Access by
/// name is more convenient, but less efficient.
///
/// A NamedData object constructed by the default constructor has no entries, meaning it has not been associated with a
/// set of names and indices, and is called <i>invalid</i> or <i>empty</i>.
///
/// An <i>non</i>-empty object contains an immutable collection of entries.  For a non-empty object: entries cannot be
/// added or removed, and the entries' names and indices <b>cannot be changed</b>.  Further, a non-empty object cannot
/// become empty. These properties ensure that a stable data layout is available to the code using this class so
/// that it can, for example, record entry indices and use them to retrieve the same entries later on.
///
/// The data associated with an entry (e.g., the true or false associated with a particular name and index in a
/// NamedData<bool>) can be changed, and each entry can be reset to a "missing" state.  A reset entry remains in the
/// collection, but has no associated data.
///
/// The entries (i.e., names & indices) in a NamedData object can be set by passing a vector of names to the
/// constructor, or by using the \ref NamedDataBuilder class.  Given one non-empty object, other objects with the same
/// entries can be created via copy construction or assignment of the non-empty object to an empty
/// (default-constructed) object.
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
	/// This is used in the NamedVariantData copy constructor.
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

	/// Check if the object has been initialized, which means it has a set of entries (i.e., names, indices, and the
	/// map between them). If the object has not been initialized, it can become initialized on assignment from an
	/// initialized object.
	///
	/// \return true if initialized.
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

	/// Record the data for an entry specified by an index.
	/// This version accepts rvalues, and the data will be moved
	/// The entry will also be marked as containing valid data.
	///
	/// \param index The index of the entry.
	/// \param value The value to be set.
	/// \return true if successful.
	inline bool set(int index, T&& value);

	/// Record the data for an entry specified by a name.
	/// The entry will also be marked as containing valid data.
	///
	/// \param name The name of the entry.
	/// \param value The value to be set.
	/// \return true if successful.
	inline bool set(const std::string& name, const T& value);

	/// Record the data for an entry specified by a name.
	/// This version accepts rvalues, and the data will be moved
	/// The entry will also be marked as containing valid data.
	///
	/// \param name The name of the entry.
	/// \param value The value to be set.
	/// \return true if successful.
	inline bool set(const std::string& name, T&& value);

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

	/// Copy the data from another NamedData, based on a map of indices. Resets entries that are reset in the other.
	/// \param source The source NamedData.
	/// \param map The map of indices.
	/// \exception Asserts if the objects do not have the same template type.
	template <typename N>
	void copy(const NamedData<N>& source, const NamedDataCopyMap& map);

	/// Caches an entry's index if it is not already cached. An index is considered already cached if it is >= 0.
	/// \param name The name of the entry.
	/// \param [in,out] index The cached index.
	void cacheIndex(const std::string& name, int* index) const;

private:
	/// The mapping between names and indices.
	std::shared_ptr<const IndexDirectory> m_directory;

	/// The array of values.
	std::vector<T> m_data;

	/// The array storing whether the data is currently valid.
	std::vector<bool> m_isDataValid;
};

};  // namespace DataStructures
};  // namespace SurgSim


#include "SurgSim/DataStructures/NamedData-inl.h"


#endif  // SURGSIM_DATASTRUCTURES_NAMEDDATA_H

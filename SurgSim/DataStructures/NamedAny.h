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

#ifndef SURGSIM_DATASTRUCTURES_NAMEDANY_H
#define SURGSIM_DATASTRUCTURES_NAMEDANY_H

#include <boost/any.hpp>
#include <SurgSim/DataStructures/NamedData.h>
#include <SurgSim/DataStructures/NamedDataBuilder.h>

namespace SurgSim
{
namespace DataStructures
{

typedef NamedDataBuilder<boost::any> NamedAnyBuilder;

/// A NamedData collection of variant data type.
///
/// A NamedAny collection is a collection variant datatypes. Each entry in the collection can be
/// accessed by using either its unique name (a std::string) or its unique index (a non-negative integer).
/// Access by name is more convenient, but also less efficient.
///
/// This sub-class of NameData encapsulates the boost::any variant type, and adds two get functions that
/// provide typed access to the contained data.

class NamedAny : public NamedData<boost::any>
{
public:
	NamedAny() {};

	inline NamedAny(const NamedData<boost::any>& namedData):
		NamedData<boost::any>(namedData)
	{
	}

	/// Given an index, get the corresponding value.
	/// It's only possible to get the value if the data was %set using set(int, const T&) or
	/// set(const std::string&, const T&), without being subsequently invalidated by reset(int)
	/// or reset(const std::string&).  In other words, get returns the same value as hasData would return.
	///
	/// \tparam T the data type used for the value at the given index.
	/// \param index The index of the entry.
	/// \param [out] value The location for the retrieved value.  Must not be null.
	/// \return true if a valid value is available, was written to \a value, and value is the correct type.
	template <typename T>
	inline bool get(int index, T* value) const
	{
		boost::any a;
		if (!NamedData::get(index, &a))
			return false;
		try
		{
			*value = boost::any_cast<T>(a);
		}
		catch(const boost::bad_any_cast &)
		{
			return false;
		}
		return true;
	}

	/// Given a name, get the corresponding value.
	/// It's only possible to get the value if the data was %set using set(int, const T&) or
	/// set(const std::string&, const T&), without being subsequently invalidated by reset(int)
	/// or reset(const std::string&).  In other words, get returns the same value as hasData would return.
	///
	/// \tparam T the data type used for the value with the given name.
	/// \param name The name of the entry.
	/// \param [out] value The location for the retrieved value.  Must not be null.
	/// \return true if a valid value is available, was written to \a value, and value is the correct type.
	template <typename T>
	inline bool get(const std::string& name, T* value) const
	{
		boost::any a;
		if (!NamedData::get(name, &a))
			return false;
		try
		{
			*value = boost::any_cast<T>(a);
		}
		catch(const boost::bad_any_cast &)
		{
			return false;
		}
		return true;
	}
};

}; // namespace DataStructures
}; // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_NAMEDANY_H

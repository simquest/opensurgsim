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

#ifndef SURGSIM_DATASTRUCTURES_OPTIONALVALUE_H
#define SURGSIM_DATASTRUCTURES_OPTIONALVALUE_H

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace DataStructures
{

template <class T>
/// Container class that can indicate whether the object has been assigned a value.
/// \tparam Class of the value that this object contains
class OptionalValue
{
public:

	/// Default Constructor, no value.
	OptionalValue()  : m_hasValue(false)
	{
	}

	/// Constructor that assigns a value.
	/// \param value The value that should be used.
	explicit OptionalValue(const T& value) : m_hasValue(true), m_value(value)
	{
	}

	/// Copy constructor
	/// \param other The value used for copying.
	OptionalValue(const OptionalValue& other) : m_hasValue(other.m_hasValue)
	{
		if (m_hasValue)
		{
			m_value = other.m_value;
		}
	}

	/// Query if this object has been assigned a value.
	/// \return	true if yes, false if not.
	bool hasValue() const
	{
		return m_hasValue;
	}

	/// Mark this object as invalid
	void invalidate()
	{
		m_hasValue = false;
	}

	/// Set the value of this object, and mark it as valid
	/// \param val The value  of the object
	void setValue(const T& val)
	{
		m_hasValue = true;
		m_value = val;
	}

	/// Gets the value.
	/// \return	The assigned value if set, excepts if no value was set.
	const T& getValue() const
	{
		SURGSIM_ASSERT(m_hasValue) << "Tried to fetch a value from an invalid OptionalValue";
		return m_value;
	}

	/// Gets the value
	/// \note do not implement T& operator*(), because *optionalValue = X; would not be able to set
	///       the hasValue() property properly.
	/// \return the contained value, excpets if no value was set.
	const T& operator*() const
	{
		SURGSIM_ASSERT(m_hasValue) << "Tried to fetch a value from an invalid OptionalValue";
		return m_value;
	}

	/// Equality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const OptionalValue<T>& rhs) const
	{
		if (m_hasValue == true && rhs.m_hasValue == true)
		{
			return m_value == rhs.m_value;
		}
		else
		{
			return m_hasValue == rhs.m_hasValue;
		}
	}


	/// Equality operator.
	/// \param	rhs	The right hand side with the specific template type.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const T& rhs) const
	{
		if (m_hasValue)
		{
			return m_value == rhs;
		}
		else
		{
			return false;
		}
	}

	/// Inequality operator
	/// \param rhs the right hand side.
	/// \return true if the parameters are not considered equivalent
	bool operator!=(const OptionalValue<T>& rhs) const
	{
		return !(*this == rhs);
	}

	/// Inequality operator
	/// \param rhs the right hand side.
	/// \return true if the parameters are not considered equivalent
	bool operator!=(const T& rhs) const
	{
		return !(*this == rhs);
	}

	/// Assignment operator.
	/// \param rhs The right hand side of the operator.
	OptionalValue& operator=(const OptionalValue& rhs)
	{
		m_hasValue = rhs.m_hasValue;
		if (m_hasValue)
		{
			m_value = rhs.m_value;
		}
		return *this;
	}

	/// Assignment operator from template type, after this hasValue() is true even if the
	/// right and side was not initialized
	/// \param the value to be assigned to this optional value
	OptionalValue& operator=(const T& rhs)
	{
		setValue(rhs);
		return *this;
	}

private:
	bool m_hasValue;
	T m_value;
};

}; // Datastructures
}; // SurgSim

#endif

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

#ifndef SURGSIM_DATASTRUCTURES_TESTABLEVALUE_H
#define SURGSIM_DATASTRUCTURES_TESTABLEVALUE_H

#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{
namespace DataStructures
{

template <class T> 
/// Container class that can indicate wether the object has been assigned a value.
/// \tparam Class of the value that this object contains
class OptionalValue
{
public:
	OptionalValue()  : m_hasValue(false)
	{
	}

	explicit OptionalValue(const T& value) : m_hasValue(true), m_value(value)
	{
	}

	/// Query if this object has been assigned a value.
	/// \return	true if yes, false if not.
	bool hasValue() const 
	{
		return m_hasValue;
	}

	/// Mark this object as invalid
	void invalidate() {m_hasValue = false;}

	/// Set the value of this object, and mark it as valid
	/// \param val The value  of the object
	void setValue(const T& val) {
		m_hasValue = true;
		m_value = val;
	}

	/// Gets the value.
	/// \return	The assigned value if set, excepts if no value was set.
	const T& getValue() const
	{
		SURGSIM_ASSERT(m_hasValue) << "Tried to fetch a value from an invalid TestableValue";
		return m_value;
	}

private:
	bool m_hasValue;
	T m_value;
};

}; // Datastructures
}; // SurgSim

#endif
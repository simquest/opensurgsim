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

#ifndef SURGSIM_GRAPHICS_UNIFORM_H
#define SURGSIM_GRAPHICS_UNIFORM_H

#include "SurgSim/Graphics/UniformBase.h"

#include <vector>

namespace SurgSim
{

namespace Graphics
{

/// Base class for a graphics uniform with a value of type T.
/// \tparam	Value type
template <class T>
class Uniform : public virtual UniformBase
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	Uniform()
	{
		SURGSIM_ADD_RW_PROPERTY(Uniform, T, Value, get, set);
	}

	/// Sets the value of the uniform
	virtual void set(const T& value) = 0;

	/// Returns the value of the uniform
	virtual const T& get() const = 0;
};

/// Specialization of Uniform for vectors of values.
/// \tparam	Value type stored in the vector
template <class T>
class Uniform<std::vector<T>> : public virtual UniformBase
{
public:
	/// Returns the number of elements
	virtual size_t getNumElements() const = 0;

	/// Sets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \param	value	Value to set
	virtual void setElement(size_t index, const T& value) = 0;

	/// Sets the value of all of the uniform's elements
	/// \param	value	Vector of values
	virtual void set(const std::vector<T>& value) = 0;

	/// Gets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \return	Value of the element
	virtual typename std::vector<T>::const_reference getElement(size_t index) const = 0;

	/// Gets the value of all of the uniform's elements
	/// \return	Vector of values
	virtual const std::vector<T>& get() const = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_UNIFORM_H
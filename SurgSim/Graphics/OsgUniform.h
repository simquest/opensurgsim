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

#ifndef SURGSIM_GRAPHICS_OSGUNIFORM_H
#define SURGSIM_GRAPHICS_OSGUNIFORM_H

#include "SurgSim/Graphics/OsgUniformBase.h"
#include "SurgSim/Graphics/Uniform.h"

namespace YAML
{
class Node;
}

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of graphics uniform with a value of type T.
/// \tparam	Value the value type of the uniform
template <class T>
class OsgUniform : public Uniform<T>, public OsgUniformBase
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgUniform(const std::string& name);

	/// Sets the value of the uniform
	/// \param value the value for this uniform
	virtual void set(const T& value);

	/// Sets the value of the uniform from a YAML Node doing the correct conversion
	/// \param node the node that contains the value for this uniform
	virtual void set(const YAML::Node& node);

	/// \return the value of the uniform
	virtual const T& get() const;

	const std::string getGlslType() const override;

private:
	/// Value of the uniform
	T m_value;
};

/// Specialization of OsgUniform for vectors of values.
/// \tparam	Value type stored in the vector
template <class T>
class OsgUniform<std::vector<T>> : public Uniform<std::vector<T>>, public OsgUniformBase
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	/// \param	numElements	Number of elements
	OsgUniform(const std::string& name, size_t numElements);

	/// \return the number of elements in the uniform
	virtual size_t getNumElements() const;

	/// Sets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \param	value	Value to set
	virtual void setElement(size_t index, const T& value);

	/// Sets the value of all of the uniform's elements
	/// \param	value	Array of values
	virtual void set(const std::vector<T>& value);

	/// Sets the value of the uniform from a YAML Node doing the correct conversion
	/// \param node that contains the values for this uniform (needs to be sequence type)
	/// \throws if the node is not of sequence type
	virtual void set(const YAML::Node& node);

	/// Gets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \return	Value of the element
	virtual typename std::vector<T>::const_reference getElement(size_t index) const;

	/// Gets the value of all of the uniform's elements
	/// \return	Vector of values
	virtual const std::vector<T>& get() const;

	const std::string getGlslType() const override;

private:
	/// Vector containing the values of the uniform's elements
	std::vector<T> m_value;
};

};  // namespace Graphics

};  // namespace SurgSim

#include "SurgSim/Graphics/OsgUniform-inl.h"

/// \note HS-2013-jul-30 If OsgUniform.h is included by itself without OsgTextureUniform.h we have a
/// 	  state where the specializations that are implemented in OsgTextureUniform.h will not be found
/// 	  by the compiler, the code will compile but not work correctly, if you remove this include
/// 	  make sure that OsgUniform.h AND OsgTextureUniform.h are included at the same time.
#include "SurgSim/Graphics/OsgTextureUniform.h"
#include "SurgSim/Graphics/OsgTextureUniform-inl.h"

#endif  // SURGSIM_GRAPHICS_OSGUNIFORM_H

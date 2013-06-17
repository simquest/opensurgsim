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

#include <SurgSim/Graphics/Uniform.h>

#include <osg/Uniform>

#include <array>

namespace SurgSim
{

namespace Graphics
{

/// Base OSG implementation of graphics uniforms.
///
/// Wraps an osg::Uniform.
/// \note
/// SurgSim::Graphics::OsgUniform is templated on the type of value, so this base class allows a pointer to any type of
/// OSG Uniform.
class OsgUniformBase : public virtual UniformBase
{
public:
	/// Constructor
	/// \param	name	Name of the uniform
	/// \param	shaderName	Name used in shaders to reference this uniform
	OsgUniformBase(const std::string& name, const std::string& shaderName);

	/// Returns the name used in shaders to use the uniform
	const std::string& getShaderName() const;

	/// Returns the OSG uniform node
	osg::ref_ptr<osg::Uniform> getOsgUniform() const;

protected:
	/// OSG uniform node
	osg::ref_ptr<osg::Uniform> m_uniform;
};

/// OSG implementation of graphics uniform with a value of type T.
/// \tparam	Value type
template <class T>
class OsgUniform : public Uniform<T>, public OsgUniformBase
{
public:
	/// Constructor
	/// \param	name	Name of the uniform
	/// \param	shaderName	Name used in shaders to reference this uniform
	OsgUniform(const std::string& name, const std::string& shaderName);

	/// Sets the value of the uniform
	virtual void set(const T& value);

	/// Returns the value of the uniform
	virtual const T& get() const;

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
	/// \param	name	Name of the uniform
	/// \param	shaderName	Name used in shaders to reference this uniform
	/// \param	numElements	Number of elements
	OsgUniform(const std::string& name, const std::string& shaderName, unsigned int numElements);

	/// Returns the number of elements
	virtual unsigned int getNumElements() const;

	/// Sets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \param	value	Value to set
	virtual void setElement(unsigned int index, const T& value);

	/// Sets the value of all of the uniform's elements
	/// \param	value	Array of values
	virtual void set(const std::vector<T>& value);

	/// Gets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \return	Value of the element
	virtual const T& getElement(unsigned int index) const;

	/// Gets the value of all of the uniform's elements
	/// \return	Vector of values
	virtual const std::vector<T>& get() const;

private:
	/// Vector containing the values of the uniform's elements
	std::vector<T> m_value;
};

/// Specialization of OsgUniform for vector of bool values.
///
/// \note
/// This is necessary because std::vector<bool> has been specialized to optimize storage, and in result
/// std::vector::operator[] does not return a reference.
template <>
class OsgUniform<std::vector<bool>> : public Uniform<std::vector<bool>>, public OsgUniformBase
{
public:
	/// Constructor
	/// \param	name	Name of the uniform
	/// \param	shaderName	Name used in shaders to reference this uniform
	/// \param	numElements	Number of elements
	OsgUniform(const std::string& name, const std::string& shaderName, unsigned int numElements);

	/// Returns the number of elements
	virtual unsigned int getNumElements() const;

	/// Sets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \param	value	Value to set
	virtual void setElement(unsigned int index, bool value);

	/// Sets the value of all of the uniform's elements
	/// \param	value	Array of values
	virtual void set(const std::vector<bool>& value);

	/// Gets the value of one of the uniform's elements
	/// \param	index	Index of the element
	/// \return	Value of the element
	virtual bool getElement(unsigned int index) const;

	/// Gets the value of all of the uniform's elements
	/// \return	Vector of values
	virtual const std::vector<bool>& get() const;

private:
	/// Vector containing the values of the uniform's elements
	std::vector<bool> m_value;
};

};  // namespace Graphics

};  // namespace SurgSim

#include <SurgSim/Graphics/OsgUniform-inl.h>

#endif  // SURGSIM_GRAPHICS_OSGUNIFORM_H

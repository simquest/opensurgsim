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

#ifndef SURGSIM_GRAPHICS_OSGUNIFORM_INL_H
#define SURGSIM_GRAPHICS_OSGUNIFORM_INL_H

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgUniformTypes.h"


namespace SurgSim
{

namespace Graphics
{

/// Default type conversion to OSG
/// \tparam	T	Type
/// \note
/// The input value is just returned. This allows this conversion method to be used with scalar types such as int and
/// float, which are the same in Eigen and OSG.
template <typename T>
const T& toOsg(const T& value)
{
	return value;
}

template <class T>
OsgUniform<T>::OsgUniform(const std::string& name) :
	UniformBase(), Uniform<T>(), OsgUniformBase(name)
{
	osg::Uniform::Type osgUniformType = getOsgUniformType<T>();
	SURGSIM_ASSERT(osgUniformType != osg::Uniform::UNDEFINED) << "Failed to get OSG uniform type!";
	SURGSIM_ASSERT(m_uniform->setType(osgUniformType)) << "Failed to set OSG uniform type!";
	m_uniform->setNumElements(1);
}

template <class T>
void OsgUniform<T>::set(const T& value)
{
	SURGSIM_ASSERT(m_uniform->set(toOsg(value))) << "Failed to set OSG uniform value!" <<
			" Uniform: " << getName() << " value: " << value;
	m_value = value;
}

template <class T>
void OsgUniform<T>::set(const YAML::Node& node)
{
	set(node.as<T>());
}

template <class T>
const T& OsgUniform<T>::get() const
{
	return m_value;
}

template <class T>
OsgUniform<std::vector<T>>::OsgUniform(const std::string& name, size_t numElements) :
							UniformBase(), Uniform<std::vector<T>>(), OsgUniformBase(name)
{
	osg::Uniform::Type osgUniformType = getOsgUniformType<T>();
	SURGSIM_ASSERT(osgUniformType != osg::Uniform::UNDEFINED) << "Failed to get OSG uniform type!";
	SURGSIM_ASSERT(m_uniform->setType(osgUniformType)) << "Failed to set OSG uniform type!";
	m_value.resize(numElements);
	m_uniform->setNumElements(numElements);
}

template <class T>
size_t OsgUniform<std::vector<T>>::getNumElements() const
{
	return m_uniform->getNumElements();
}

template <class T>
void OsgUniform<std::vector<T>>::setElement(size_t index, const T& value)
{
	SURGSIM_ASSERT(m_uniform->setElement(index, toOsg(value))) << "Failed to set OSG uniform value!" <<
			" Uniform: " << getName() << " index: " << index << " value: " << value;
	m_value[index] = value;
}

template <class T>
void OsgUniform<std::vector<T>>::set(const std::vector<T>& value)
{
	SURGSIM_ASSERT(value.size() == m_uniform->getNumElements()) <<
			"Number of elements (" << value.size() << ") must match uniform's number of elements (" <<
			m_uniform->getNumElements() << ")! Uniform: " << getName();
	for (size_t i = 0; i < value.size(); ++i)
	{
		setElement(i, value[i]);
	}
}

template <class T>
typename std::vector<T>::const_reference OsgUniform<std::vector<T>>::getElement(size_t index) const
{
	return m_value[index];
}

template <class T>
const std::vector<T>& OsgUniform<std::vector<T>>::get() const
{
	return m_value;
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGUNIFORM_INL_H

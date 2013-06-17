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

#include <SurgSim/Graphics/OsgUniform.h>

using SurgSim::Graphics::OsgUniform;
using SurgSim::Graphics::toOsg;

OsgUniform<std::vector<bool>>::OsgUniform(const std::string& name, const std::string& shaderName, unsigned int numElements) : 
	UniformBase(name), Uniform(name), OsgUniformBase(name, shaderName)
{
	osg::Uniform::Type osgUniformType = SurgSim::Graphics::getOsgUniformType<bool>();
	SURGSIM_ASSERT(osgUniformType != osg::Uniform::UNDEFINED) << "Failed to get OSG uniform type!";
	SURGSIM_ASSERT(m_uniform->setType(osgUniformType)) << "Failed to set OSG uniform type!";
	m_value.resize(numElements);
	m_uniform->setNumElements(numElements);
}

unsigned int OsgUniform<std::vector<bool>>::getNumElements() const
{
	return m_uniform->getNumElements();
}

void OsgUniform<std::vector<bool>>::setElement(unsigned int index, bool value)
{
	SURGSIM_ASSERT(m_uniform->setElement(index, toOsg(value))) << "Failed to set OSG uniform value!" <<
		" Uniform: " << getName() << " index: " << index << " value: " << value;
	m_value[index] = value;
}

void OsgUniform<std::vector<bool>>::set(const std::vector<bool>& value)
{
	SURGSIM_ASSERT(value.size() == m_uniform->getNumElements()) << 
		"Number of elements (" << value.size() << ") must match uniform's number of elements (" << 
		m_uniform->getNumElements() << ")! Uniform: " << getName();
	for (unsigned int i = 0; i < value.size(); ++i)
	{
		setElement(i, value[i]);
	}
}

bool OsgUniform<std::vector<bool>>::getElement(unsigned int index) const
{
	return m_value[index];
}

const std::vector<bool>& OsgUniform<std::vector<bool>>::get() const
{
	return m_value;
}

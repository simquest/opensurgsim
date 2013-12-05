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

#include "SurgSim/Graphics/OsgMaterial.h"

#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgUniformBase.h"

#include <algorithm>

using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgUniformBase;

namespace SurgSim
{

namespace Graphics
{

OsgMaterial::OsgMaterial() : Material(),
	m_stateSet(new osg::StateSet())
{
}

bool OsgMaterial::addUniform(std::shared_ptr<UniformBase> uniform)
{
	bool didSucceed = false;

	std::shared_ptr<OsgUniformBase> osgUniform = std::dynamic_pointer_cast<OsgUniformBase>(uniform);
	if (osgUniform)
	{
		osgUniform->addToStateSet(m_stateSet);
		m_uniforms.push_back(osgUniform);
		didSucceed = true;
	}

	return didSucceed;
}

bool OsgMaterial::removeUniform(std::shared_ptr<UniformBase> uniform)
{
	std::shared_ptr<OsgUniformBase> osgUniform = std::dynamic_pointer_cast<OsgUniformBase>(uniform);

	bool didSucceed = false;

	if (osgUniform)
	{
		auto it = std::find(m_uniforms.begin(), m_uniforms.end(), osgUniform);

		if (it != m_uniforms.end())
		{
			osgUniform->removeFromStateSet(m_stateSet);
			m_uniforms.erase(it);
			didSucceed = true;
		}
	}

	return didSucceed;
}

unsigned int OsgMaterial::getNumUniforms() const
{
	return m_uniforms.size();
}

std::shared_ptr<UniformBase> OsgMaterial::getUniform(unsigned int index) const
{
	return m_uniforms[index];
}

std::shared_ptr<UniformBase>
OsgMaterial::getUniform(const std::string& name) const
{
	std::shared_ptr<UniformBase> result;
	auto it = std::find_if(
		std::begin(m_uniforms),
		std::end(m_uniforms),
		[&name] (const std::shared_ptr<OsgUniformBase>& uniform) {return uniform->getName() == name;});
	if (it != std::end(m_uniforms))
	{
		result = *it;
	}
	return result;
}

bool OsgMaterial::removeUniform(const std::string& name)
{
	bool result = false;
	auto it = std::find_if(
		std::begin(m_uniforms),
		std::end(m_uniforms),
		[&name] (const std::shared_ptr<OsgUniformBase>& uniform) {return uniform->getName() == name;});
	if (it != std::end(m_uniforms))
	{
		result = removeUniform(*it);
	}
	return result;
}

bool OsgMaterial::hasUniform(const std::string& name) const
{
	return (getUniform(name) != nullptr);
}

bool OsgMaterial::setShader(std::shared_ptr<Shader> shader)
{
	bool didSucceed = false;

	std::shared_ptr<OsgShader> osgShader = std::dynamic_pointer_cast<OsgShader>(shader);
	if (osgShader)
	{
		if (m_shader)
		{
			m_shader->removeFromStateSet(m_stateSet.get());
		}
		osgShader->addToStateSet(m_stateSet);
		m_shader = osgShader;
		didSucceed = true;
	}

	return didSucceed;
}

std::shared_ptr<Shader> OsgMaterial::getShader() const
{
	return m_shader;
}

void OsgMaterial::clearShader()
{
	if (m_shader)
	{
		m_shader->removeFromStateSet(m_stateSet.get());
	}
	m_shader = nullptr;
}

}; // namespace Graphics

}; // namespace SurgSim

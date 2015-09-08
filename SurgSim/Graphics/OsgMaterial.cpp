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

#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgProgram.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgUniformFactory.h"

#include <algorithm>
#include <functional>

#include <boost/any.hpp>


using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgUniformBase;

namespace SurgSim
{

namespace Graphics
{
OsgMaterial::OsgMaterial(const std::string& name)  :
	Material(name),
	m_stateSet(new osg::StateSet())
{

}

bool OsgMaterial::addUniform(std::shared_ptr<UniformBase> uniform)
{
	bool didSucceed = false;

	std::shared_ptr<OsgUniformBase> osgUniform = std::dynamic_pointer_cast<OsgUniformBase>(uniform);
	if (osgUniform != nullptr)
	{
		if (isInitialized())
		{
			osgUniform->addToStateSet(m_stateSet);
		}
		m_uniforms.push_back(osgUniform);

		// add a property to Material, that carries the uniform name and forwards to the value of the uniform
		// This exposes the non-shared pointer to the uniform in the function table, the entry in the function
		// table will be removed when the uniform is removed from the material. The material holds a shared
		// pointer to the uniform, keeping the uniform alive during the lifetime of the material.
		forwardProperty(osgUniform->getName(), *osgUniform.get(), "Value");
		didSucceed = true;
	}
	return didSucceed;
}

bool OsgMaterial::addUniform(const std::string& type, const std::string& name)
{
	static OsgUniformFactory factory;

	bool result = false;
	if (factory.isRegistered(type))
	{
		result = addUniform(factory.create(type, name));
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Type " << type << " not supported.";
	}
	return result;
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
			if (isInitialized())
			{
				osgUniform->removeFromStateSet(m_stateSet);
			}
			m_uniforms.erase(it);
			didSucceed = true;
		}

		removeAccessors(osgUniform->getName());

	}

	return didSucceed;
}

size_t OsgMaterial::getNumUniforms() const
{
	return m_uniforms.size();
}

std::shared_ptr<UniformBase> OsgMaterial::getUniform(size_t index) const
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
				  [&name](const std::shared_ptr<OsgUniformBase>& uniform)
	{
		return uniform->getName() == name;
	});
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
				  [&name](const std::shared_ptr<OsgUniformBase>& uniform)
	{
		return uniform->getName() == name;
	});
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

bool OsgMaterial::setProgram(std::shared_ptr<Program> program)
{
	bool didSucceed = false;

	std::shared_ptr<OsgProgram> osgProgram = std::dynamic_pointer_cast<OsgProgram>(program);
	if (osgProgram)
	{
		if (m_program)
		{
			m_program->removeFromStateSet(m_stateSet.get());
		}
		osgProgram->addToStateSet(m_stateSet);
		m_program = osgProgram;
		didSucceed = true;
	}

	return didSucceed;
}

std::shared_ptr<Program> OsgMaterial::getProgram() const
{
	return m_program;
}

void OsgMaterial::clearProgram()
{
	if (m_program)
	{
		m_program->removeFromStateSet(m_stateSet.get());
	}
	m_program = nullptr;
}

bool OsgMaterial::doInitialize()
{
	for (auto it = m_uniforms.begin(); it != m_uniforms.end(); ++it)
	{
		(*it)->addToStateSet(m_stateSet);
	}
	return true;
}

bool OsgMaterial::doWakeUp()
{
	return true;
}

osg::ref_ptr<osg::StateSet> OsgMaterial::getOsgStateSet() const
{
	return m_stateSet;
}

std::shared_ptr<OsgMaterial> buildMaterial(const std::string& vertexShaderName, const std::string& fragmentShaderName)
{
	bool result = true;

	std::shared_ptr<OsgMaterial> material;

	auto program = std::make_shared<OsgProgram>();
	std::string fileName;
	fileName = SurgSim::Framework::Runtime::getApplicationData()->findFile(vertexShaderName);
	if (!program->loadVertexShader(fileName))
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics"))
				<< "Shader " << vertexShaderName << ", could not "
				<< ((fileName == "") ? "find shader file" : "compile " + fileName) << ".";
		result = false;
	}

	fileName = SurgSim::Framework::Runtime::getApplicationData()->findFile(fragmentShaderName);
	if (!program->loadFragmentShader(fileName))
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics"))
				<< "Shader " << fragmentShaderName << " , could not "
				<< ((fileName == "") ? "find shader file" : "compile " + fileName) << ".";
		result = false;
	}

	if (result)
	{
		material = std::make_shared<OsgMaterial>("material");
		material->setProgram(program);
	}

	return material;
}

}; // namespace Graphics

}; // namespace SurgSim

// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Graphics/OsgProgram.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"

#include <boost/algorithm/string.hpp>

namespace
{
static const osg::Shader::Type OsgShaderTypes[3] = {osg::Shader::VERTEX, osg::Shader::FRAGMENT, osg::Shader::GEOMETRY};
}



namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Graphics::OsgProgram, OsgProgram);


OsgProgram::OsgProgram() : Program(),
	m_program(new osg::Program()),
	m_globalScope(false)
{

}

void OsgProgram::addToStateSet(osg::StateSet* stateSet)
{
	if (stateSet != nullptr)
	{
		int attribute = osg::StateAttribute::ON | ((m_globalScope) ? osg::StateAttribute::OVERRIDE : 0);
		stateSet->setAttributeAndModes(m_program, attribute);
	}
}

void OsgProgram::removeFromStateSet(osg::StateSet* stateSet)
{
	if (stateSet != nullptr)
	{
		stateSet->removeAttribute(m_program);
	}
}

bool OsgProgram:: hasVertexShader() const
{
	return hasShader(SHADER_TYPE_VERTEX);
}

void OsgProgram::clearVertexShader()
{
	clearShader(SHADER_TYPE_VERTEX);
}

bool OsgProgram::loadVertexShader(const std::string& filePath)
{
	return loadShaderSource(filePath, SHADER_TYPE_VERTEX);
}

void OsgProgram::setVertexShaderSource(const std::string& source)
{
	setShaderSource(source, SHADER_TYPE_VERTEX);
}

bool OsgProgram::getVertexShaderSource(std::string* source) const
{
	return getShaderSource(SHADER_TYPE_VERTEX, source);
}

bool OsgProgram::hasGeometryShader() const
{
	return hasShader(SHADER_TYPE_GEOMETRY);
}

void OsgProgram::clearGeometryShader()
{
	return clearShader(SHADER_TYPE_GEOMETRY);
}

bool OsgProgram::loadGeometryShader(const std::string& filePath)
{
	return loadShaderSource(filePath, SHADER_TYPE_GEOMETRY);
}

void OsgProgram::setGeometryShaderSource(const std::string& source)
{
	return setShaderSource(source, SHADER_TYPE_GEOMETRY);
}

bool OsgProgram::getGeometryShaderSource(std::string* source) const
{
	return getShaderSource(SHADER_TYPE_GEOMETRY, source);
}

bool OsgProgram::hasFragmentShader() const
{
	return hasShader(SHADER_TYPE_FRAGMENT);
}

void OsgProgram::clearFragmentShader()
{
	clearShader(SHADER_TYPE_FRAGMENT);
}

bool OsgProgram::loadFragmentShader(const std::string& filePath)
{
	return loadShaderSource(filePath, SHADER_TYPE_FRAGMENT);
}

void OsgProgram::setFragmentShaderSource(const std::string& source)
{
	setShaderSource(source, SHADER_TYPE_FRAGMENT);
}

bool OsgProgram::getFragmentShaderSource(std::string* source) const
{
	return getShaderSource(SHADER_TYPE_FRAGMENT, source);
}

osg::ref_ptr<osg::Program> OsgProgram::getOsgProgram() const
{
	return m_program;
}

void OsgProgram::setGlobalScope(bool val)
{
	m_globalScope = val;
	osg::StateAttribute::ParentList parents = m_program->getParents();
	for (auto it = std::begin(parents); it != std::end(parents); ++it)
	{
		addToStateSet(*it);
	}
}

bool OsgProgram::isGlobalScope() const
{
	return m_globalScope;
}

bool OsgProgram::hasShader(int shaderType) const
{
	bool result = true;
	if (shaderType < SHADER_TYPE_COUNT)
	{
		result = m_osgShaders[shaderType].valid();
	}
	return result;
}

void OsgProgram::clearShader(int shaderType)
{
	if (hasShader(shaderType))
	{
		m_program->removeShader(m_osgShaders[shaderType]);
		m_osgShaders[shaderType] = nullptr;
	}
}

bool OsgProgram::loadShaderSource(const std::string& filePath, int shaderType)
{
	bool result = false;
	auto shader = getOrCreateOsgShader(shaderType);
	if (shader->loadShaderSourceFromFile(filePath))
	{
		shader->setName(filePath);
		result = true;
	}

	return result;
}

void OsgProgram::setShaderSource(const std::string& source, int shaderType)
{
	auto shader = getOrCreateOsgShader(shaderType);
	shader->setShaderSource(source);
}

bool OsgProgram::getShaderSource(int shaderType, std::string* source) const
{
	if (hasShader(shaderType))
	{
		*source = m_osgShaders[shaderType]->getShaderSource();
		return true;
	}
	else
	{
		*source = "";
		return false;
	}
}

osg::ref_ptr<osg::Shader> OsgProgram::getOrCreateOsgShader(int shaderType)
{
	osg::ref_ptr<osg::Shader> result;
	if (!hasShader(shaderType))
	{
		result = new osg::Shader(OsgShaderTypes[shaderType]);
		m_program->addShader(result);
		m_osgShaders[shaderType] = result;
	}
	else
	{
		result = m_osgShaders[shaderType];
	}
	return result;
}

bool OsgProgram::doLoad(const std::string& filePath)
{
	YAML::Node node;
	try
	{
		node = YAML::LoadFile(filePath);
	}
	catch (YAML::ParserException e)
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Graphics/OsgProgram"))
				<< "Could not parse YAML File at " << filePath
				<< " due to " << e.msg << " at line " << e.mark.line << " column " << e.mark.column;
		return false;
	}

	auto map = node.as<std::unordered_map<std::string, std::string>>();

	if (!map["VertexShaderSource"].empty())
	{
		setVertexShaderSource(map["VertexShaderSource"]);
	}
	if (!map["FragmentShaderSource"].empty())
	{
		setVertexShaderSource(map["FragmentShaderSource"]);
	}
	if (!map["GeometryShaderSource"].empty())
	{
		setVertexShaderSource(map["GeometryShaderSource"]);
	}

	return true;
}

std::shared_ptr<OsgProgram> loadProgram(const SurgSim::Framework::ApplicationData& data, const std::string& name)
{
	return loadProgram(data, name + ".vert", name + ".frag");
}

std::shared_ptr<OsgProgram> loadProgram(const SurgSim::Framework::ApplicationData& data,
										const std::string& vertexShaderName, const std::string& fragmentShaderName)
{
	std::string filename;

	auto program(std::make_shared<SurgSim::Graphics::OsgProgram>());
	bool success = true;
	filename = data.findFile(vertexShaderName);
	if (filename == "")
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not find vertex shader " << vertexShaderName;
		success = false;
	}
	else if (!program->loadVertexShader(filename))
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not load vertex shader " << vertexShaderName;
		success = false;
	}

	filename = data.findFile(fragmentShaderName);
	if (filename == "")
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not find fragment shader " << fragmentShaderName;
		success = false;
	}
	if (!program->loadFragmentShader(filename))
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not load fragment shader " << fragmentShaderName;
		success = false;
	}

	if (!success)
	{
		program = nullptr;
	}

	return program;
}

}
}

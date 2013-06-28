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

#include <SurgSim/Graphics/OsgShader.h>

using SurgSim::Graphics::OsgShader;

OsgShader::OsgShader() : SurgSim::Graphics::Shader(),
	m_program(new osg::Program())
{

}

void OsgShader::addToStateSet(osg::StateSet* stateSet)
{
	stateSet->setAttributeAndModes(m_program, osg::StateAttribute::ON);
}

void OsgShader::removeFromStateSet(osg::StateSet* stateSet)
{
	stateSet->removeAttribute(m_program);
}

bool OsgShader:: hasVertexShader()
{
	return m_vertexShader.valid();
}

void OsgShader::clearVertexShader()
{
	if (m_vertexShader)
	{
		m_program->removeShader(m_vertexShader);
		m_vertexShader = nullptr;
	}
}

bool OsgShader::loadVertexShaderSource(const std::string& filePath)
{
	if (! m_vertexShader.valid())
	{
		m_vertexShader = new osg::Shader(osg::Shader::VERTEX);
		m_program->addShader(m_vertexShader);
	}
	return m_vertexShader->loadShaderSourceFromFile(filePath);
}

void OsgShader::setVertexShaderSource(const std::string& source)
{
	if (! m_vertexShader.valid())
	{
		m_vertexShader = new osg::Shader(osg::Shader::VERTEX);
		m_program->addShader(m_vertexShader);
	}
	m_vertexShader->setShaderSource(source);
}

bool OsgShader::getVertexShaderSource(std::string* source) const
{
	if (m_vertexShader)
	{
		*source = m_vertexShader->getShaderSource();
		return true;
	}
	else
	{
		*source = "";
		return false;
	}
}

bool OsgShader::hasGeometryShader() const
{
	return m_geometryShader.valid();
}

void OsgShader::clearGeometryShader()
{
	if (m_geometryShader)
	{
		m_program->removeShader(m_geometryShader);
		m_geometryShader = nullptr;
	}
}

bool OsgShader::loadGeometryShaderSource(const std::string& filePath)
{
	if (! m_geometryShader.valid())
	{
		m_geometryShader = new osg::Shader(osg::Shader::GEOMETRY);
		m_program->addShader(m_geometryShader);
	}
	return m_geometryShader->loadShaderSourceFromFile(filePath);
}

void OsgShader::setGeometryShaderSource(const std::string& source)
{
	if (! m_geometryShader.valid())
	{
		m_geometryShader = new osg::Shader(osg::Shader::GEOMETRY);
		m_program->addShader(m_geometryShader);
	}
	m_geometryShader->setShaderSource(source);
}

bool OsgShader::getGeometryShaderSource(std::string* source) const
{
	if (m_geometryShader)
	{
		*source = m_geometryShader->getShaderSource();
		return true;
	}
	else
	{
		*source = "";
		return false;
	}
}

bool OsgShader::hasFragmentShader() const
{
	return m_fragmentShader.valid();
}

void OsgShader::clearFragmentShader()
{
	if (m_fragmentShader)
	{
		m_program->removeShader(m_fragmentShader);
		m_fragmentShader = nullptr;
	}
}

bool OsgShader::loadFragmentShaderSource(const std::string& filePath)
{
	if (! m_fragmentShader.valid())
	{
		m_fragmentShader = new osg::Shader(osg::Shader::FRAGMENT);
		m_program->addShader(m_fragmentShader);
	}
	return m_fragmentShader->loadShaderSourceFromFile(filePath);
}

void OsgShader::setFragmentShaderSource(const std::string& source)
{
	if (! m_fragmentShader.valid())
	{
		m_fragmentShader = new osg::Shader(osg::Shader::FRAGMENT);
		m_program->addShader(m_fragmentShader);
	}
	m_fragmentShader->setShaderSource(source);
}

bool OsgShader::getFragmentShaderSource(std::string* source) const
{
	if (m_fragmentShader)
	{
		*source = m_fragmentShader->getShaderSource();
		return true;
	}
	else
	{
		*source = "";
		return false;
	}
}
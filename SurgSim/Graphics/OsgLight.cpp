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



#include "SurgSim/Graphics/OsgLight.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgConversions.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"


using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

#include <osg/Uniform>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/Node>

using osg::Uniform;

namespace SurgSim
{
namespace Graphics
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgLight, OsgLight);

/// \note HS-2013-sep-09 Right now we are implementing all the shader uniforms as floats, this
/// 	  means that they all have to be downconverted from double, i don't know what the hit
/// 	  of going to double in the shaders would be
OsgLight::OsgLight(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	Light(name),
	m_diffuseColor(1.0, 1.0, 1.0, 1.0),
	m_specularColor(1.0, 1.0, 1.0, 1.0),
	m_constantAttenuation(1.0),
	m_linearAttenuation(0.0),
	m_quadraticAttenuation(0.0)
{
	std::string prefix = "lightSource.";

	m_light = new osg::Light();
	m_light->setName(name);
	m_light->setLightNum(0);
	m_lightSource = new osg::LightSource();
	m_lightSource->setDataVariance(osg::Object::DYNAMIC);
	m_lightSource->setLight(m_light);

	m_switch->addChild(m_lightSource);

	m_uniforms[POSITION] = new osg::Uniform(osg::Uniform::FLOAT_VEC4, prefix + "position");

	m_uniforms[DIFFUSE_COLOR] = new osg::Uniform(osg::Uniform::FLOAT_VEC4, prefix + "diffuse");
	setDiffuseColor(m_diffuseColor);

	m_uniforms[SPECULAR_COLOR] = new osg::Uniform(osg::Uniform::FLOAT_VEC4, prefix + "specular");
	setSpecularColor(m_specularColor);

	m_uniforms[CONSTANT_ATTENUATION] = new osg::Uniform(osg::Uniform::FLOAT, prefix + "constantAttenuation");
	setConstantAttenuation(m_constantAttenuation);

	m_uniforms[LINEAR_ATTENUATION] = new osg::Uniform(osg::Uniform::FLOAT, prefix + "linearAttenuation");
	setLinearAttenuation(m_linearAttenuation);

	m_uniforms[QUADRATIC_ATTENUATION] = new osg::Uniform(osg::Uniform::FLOAT, prefix + "quadraticAttenuation");
	setQuadraticAttenuation(m_quadraticAttenuation);

	// By default light the main group
	setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

}

OsgLight::~OsgLight()
{

}



bool OsgLight::setGroup(std::shared_ptr<SurgSim::Graphics::Group> group)
{
	std::shared_ptr<OsgGroup> newGroup = std::dynamic_pointer_cast<OsgGroup>(group);

	if (group != nullptr && newGroup == nullptr)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics"))
				<< "OsgLight::setGroup() called with a group that is not an OsgGroup.";
	}

	bool clearGroup = m_group != nullptr && (newGroup != nullptr || group == nullptr);
	bool setGroup = (newGroup != nullptr);
	bool success  = (group != nullptr && newGroup != nullptr) || group == nullptr;

	if (clearGroup)
	{
		osg::ref_ptr<osg::StateSet> stateSet = m_group->getOsgGroup()->getOrCreateStateSet();
		remove(stateSet);
		m_group = nullptr;
	}

	if (setGroup)
	{
		osg::ref_ptr<osg::StateSet> stateSet = newGroup->getOsgGroup()->getOrCreateStateSet();
		apply(stateSet);
		m_group = newGroup;
	}

	return success;
}

std::shared_ptr<SurgSim::Graphics::Group> OsgLight::getGroup()
{
	return m_group;
}

void OsgLight::setDiffuseColor(const SurgSim::Math::UnalignedVector4d& color)
{
	m_diffuseColor = color;
	SurgSim::Math::Vector4f floatColor = color.cast<float>();
	osg::Vec4f osgVec = toOsg(floatColor);
	m_uniforms[DIFFUSE_COLOR]->set(osgVec);
	m_light->setDiffuse(osgVec);
}

SurgSim::Math::UnalignedVector4d OsgLight::getDiffuseColor()
{
	return m_diffuseColor;
}

void OsgLight::setSpecularColor(const SurgSim::Math::UnalignedVector4d& color)
{
	m_specularColor = color;
	SurgSim::Math::Vector4f floatColor = color.cast<float>();
	osg::Vec4f osgVec = toOsg(floatColor);
	m_uniforms[SPECULAR_COLOR]->set(osgVec);
	m_light->setSpecular(osgVec);
}

SurgSim::Math::UnalignedVector4d OsgLight::getSpecularColor()
{
	return m_specularColor;
}

void OsgLight::setConstantAttenuation(double val)
{
	m_constantAttenuation = val;
	m_uniforms[CONSTANT_ATTENUATION]->set(static_cast<float>(val));
	m_light->setConstantAttenuation(val);
}

double OsgLight::getConstantAttenuation()
{
	return m_constantAttenuation;
}

void OsgLight::setLinearAttenuation(double val)
{
	m_linearAttenuation = val;
	m_uniforms[LINEAR_ATTENUATION]->set(static_cast<float>(val));
	m_light->setLinearAttenuation(val);
}

double OsgLight::getLinearAttenuation()
{
	return m_linearAttenuation;
}

void OsgLight::setQuadraticAttenuation(double val)
{
	m_quadraticAttenuation = val;
	m_uniforms[QUADRATIC_ATTENUATION]->set(static_cast<float>(val));
	m_light->setQuadraticAttenuation(val);
}

double OsgLight::getQuadraticAttenuation()
{
	return m_quadraticAttenuation;
}

void OsgLight::doUpdate(double dt)
{
	SurgSim::Math::Vector3f position = getPose().translation().cast<float>();
	osg::Vec4f osgVec(osg::Vec4f(toOsg(position), 1.0));
	m_uniforms[POSITION]->set(osgVec);
	m_light->setPosition(osgVec);
}

void OsgLight::apply(osg::ref_ptr<osg::StateSet> stateSet)
{
	for (auto it = std::begin(m_uniforms); it != std::end(m_uniforms); ++it)
	{
		stateSet->addUniform(it->second);
	}
}

void OsgLight::remove(osg::ref_ptr<osg::StateSet> stateSet)
{
	for (auto it = std::begin(m_uniforms); it != std::end(m_uniforms); ++it)
	{
		stateSet->removeUniform(it->second);
	}
}

void OsgLight::setLightGroupReference(const std::string& name)
{
	m_groupReference = name;
	removeGroupReference(name);
}

std::string OsgLight::getLightGroupReference()
{
	return m_groupReference;
}

}; // Graphics
}; // SurgSim

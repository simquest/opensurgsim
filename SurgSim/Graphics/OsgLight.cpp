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



#include <SurgSim/Graphics/OsgLight.h>

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgConversions.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>


using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

#include <osg/Uniform>

using osg::Uniform;

namespace SurgSim
{
namespace Graphics
{

/// \note HS-2013-sep-09 Right now we are implementing all the shader uniforms as floats, this
/// 	  means that they all have to be downconverted from double, i don't know what the hit
/// 	  of going to double in the shaders would be
OsgLight::OsgLight(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	Light(name),
	m_ambientColor(0.0,0.0,0.0,1.0),
	m_diffuseColor(1.0,1.0,1.0,1.0),
	m_specularColor(1.0,1.0,1.0,1.0),
	m_constantAttenuation(1.0),
	m_linearAttenuation(0.0),
	m_quadraticAttenuation(0.0)
{
	m_uniforms[POSITION] = new osg::Uniform(Uniform::FLOAT_VEC3,"ossLightPosition");
	setPose(getPose());

	m_uniforms[AMBIENT_COLOR] = new osg::Uniform(Uniform::FLOAT_VEC4, "ossLightAmbientColor");
	setAmbientColor(m_ambientColor);

	m_uniforms[DIFFUSE_COLOR] = new osg::Uniform(Uniform::FLOAT_VEC4, "ossLightDiffuseColor");
	setDiffuseColor(m_diffuseColor);

	m_uniforms[SPECULAR_COLOR] = new osg::Uniform(Uniform::FLOAT_VEC4, "ossLightSpecularColor");
	setSpecularColor(m_specularColor);

	m_uniforms[CONSTANT_ATTENUATION] = new osg::Uniform(Uniform::FLOAT, "ossLightConstantAttenuation");
	setConstantAttenuation(m_constantAttenuation);

	m_uniforms[LINEAR_ATTENUATION] = new osg::Uniform(Uniform::FLOAT, "ossLightLinearAttenuation");
	setLinearAttenuation(m_linearAttenuation);

	m_uniforms[QUADRATIC_ATTENUATION] = new osg::Uniform(Uniform::FLOAT, "ossLightQuadraticAttenuation");
	setQuadraticAttenuation(m_quadraticAttenuation);
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

void OsgLight::setAmbientColor(const SurgSim::Math::Vector4d& color)
{
	m_ambientColor = color;
	SurgSim::Math::Vector4f floatColor = color.cast<float>();
	m_uniforms[AMBIENT_COLOR]->set(toOsg(floatColor));
}

SurgSim::Math::Vector4d OsgLight::getAmbientColor()
{
	return m_ambientColor;
}

void OsgLight::setDiffuseColor(const SurgSim::Math::Vector4d& color)
{
	m_diffuseColor = color;
	SurgSim::Math::Vector4f floatColor = color.cast<float>();
	m_uniforms[DIFFUSE_COLOR]->set(toOsg(floatColor));
}

SurgSim::Math::Vector4d OsgLight::getDiffuseColor()
{
	return m_diffuseColor;
}

void OsgLight::setSpecularColor(const SurgSim::Math::Vector4d& color)
{
	m_specularColor = color;
	SurgSim::Math::Vector4f floatColor = color.cast<float>();
	m_uniforms[SPECULAR_COLOR]->set(toOsg(floatColor));
}

SurgSim::Math::Vector4d OsgLight::getSpecularColor()
{
	return m_specularColor;
}

void OsgLight::setConstantAttenuation(double val)
{
	m_constantAttenuation = val;
	m_uniforms[CONSTANT_ATTENUATION]->set(static_cast<float>(val));
}

double OsgLight::getConstantAttenuation()
{
	return m_constantAttenuation;
}

void OsgLight::setLinearAttenuation(double val)
{
	m_linearAttenuation = val;
	m_uniforms[LINEAR_ATTENUATION]->set(static_cast<float>(val));
}

double OsgLight::getLinearAttenuation()
{
	return m_linearAttenuation;
}

void OsgLight::setQuadraticAttenuation(double val)
{
	m_quadraticAttenuation = val;
	m_uniforms[QUADRATIC_ATTENUATION]->set(static_cast<float>(val));
}

double OsgLight::getQuadraticAttenuation()
{
	return m_quadraticAttenuation;
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

void OsgLight::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	OsgRepresentation::setPose(pose);
	SurgSim::Math::Vector3f position = pose.translation().cast<float>();
	m_uniforms[POSITION]->set(toOsg(position));
}



}; // Graphics
}; // SurgSim

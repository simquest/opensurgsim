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

#ifndef SURGSIM_GRAPHICS_OSGLIGHT_H
#define SURGSIM_GRAPHICS_OSGLIGHT_H

#include <memory>
#include <string>
#include <unordered_map>

#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/Light.h"

#include <osg/ref_ptr>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{

class Uniform;
class StateSet;
class Light;
class LightSource;

}

namespace SurgSim
{
namespace Graphics
{

class OsgGroup;

SURGSIM_STATIC_REGISTRATION(OsgLight);

/// OpenScenegraph implementation for the Light interface
class OsgLight : public OsgRepresentation, public Light
{
public:

	/// If we use the uniforms map, to check for all the uniforms that should be set in the stateset, then
	/// we can reduce the breakability of the unittest
	friend class OsgLightTests;

	/// Constructor
	explicit OsgLight(const std::string& name);
	virtual ~OsgLight();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgLight);

	bool setGroup(std::shared_ptr<SurgSim::Graphics::Group> group) override;

	void setLightGroupReference(const std::string& name) override;

	std::string getLightGroupReference() override;

	std::shared_ptr<SurgSim::Graphics::Group> getGroup() override;

	void setDiffuseColor(const SurgSim::Math::UnalignedVector4d& color) override;

	SurgSim::Math::UnalignedVector4d getDiffuseColor() override;

	void setSpecularColor(const SurgSim::Math::UnalignedVector4d& color) override;

	SurgSim::Math::UnalignedVector4d getSpecularColor() override;

	void setConstantAttenuation(double val) override;

	double getConstantAttenuation() override;

	void setLinearAttenuation(double val) override;

	double getLinearAttenuation() override;

	void setQuadraticAttenuation(double val) override;

	double getQuadraticAttenuation() override;


private:
	void doUpdate(double dt) override;

	/// Applies all the lights variables to the given StateSet
	void apply(osg::ref_ptr<osg::StateSet> stateSet);

	/// Removes all the lights variable from the given StateSet
	void remove(osg::ref_ptr<osg::StateSet> stateSet);

	/// Internal for managing uniforms
	enum UniformType
	{
		POSITION = 0,
		DIFFUSE_COLOR,
		SPECULAR_COLOR,
		CONSTANT_ATTENUATION,
		LINEAR_ATTENUATION,
		QUADRATIC_ATTENUATION
	};

	/// The group for this light
	std::shared_ptr<OsgGroup> m_group;

	/// Map for managing all uniforms that this object owns
	std::unordered_map<int, osg::ref_ptr<osg::Uniform>> m_uniforms;

	SurgSim::Math::UnalignedVector4d m_diffuseColor;		///< The actual diffuse color that was set
	SurgSim::Math::UnalignedVector4d m_specularColor;	///< The actual specular color that was set

	double m_constantAttenuation;				///< The actual constant attenuation value that was set
	double m_linearAttenuation;					///< The actual linear attenuation value that was set
	double m_quadraticAttenuation;				///< The actual quadratic attenuation value that was set

	///@{
	/// Osg instances to manage the light
	osg::ref_ptr<osg::Light> m_light;
	osg::ref_ptr<osg::LightSource> m_lightSource;
	///@}

	std::string m_groupReference; ///< Name of the group that this light should shine on...
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#endif

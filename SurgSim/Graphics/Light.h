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

#ifndef SURGSIM_GRAPHICS_LIGHT_H
#define SURGSIM_GRAPHICS_LIGHT_H

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{

class Group;

/// Abstract interface for a light, a light needs to be assigned to a group to be active, only the members of this
/// group will be considered to be lit by this light. Currently this light implements a pointlight. It will have to
/// be extended for a directional and spot lights. The class should provide the following uniform values. The position
/// is set by using the representations setPose() call.
/// \code
/// struct LightSource {
/// 	vec4 ambient;
/// 	vec4 diffuse;
/// 	vec4 specular;
/// 	vec4 position;
/// 	float constantAttenuation;
/// 	float linearAttenuation;
/// 	float quadraticAttenuation;
/// };
///
/// uniform LightSource lightSource;
/// \endcode
///
class Light : public virtual Representation
{
public:

	/// Constructor
	explicit Light(const std::string& name) : Representation(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(Light, SurgSim::Math::Vector4d, DiffuseColor,
											getDiffuseColor, setDiffuseColor);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(Light, SurgSim::Math::Vector4d, SpecularColor,
											getSpecularColor, setSpecularColor);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(Light, double, ConstantAttenuation,
											getConstantAttenuation, setConstantAttenuation);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(Light, double, LinearAttenuation,
											getLinearAttenuation, setLinearAttenuation);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(Light, double, QuadraticAttenuation,
											getQuadraticAttenuation, setQuadraticAttenuation);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(Light, std::string, LightGroupReference,
											getLightGroupReference, setLightGroupReference);
	}

	virtual ~Light()
	{
	}

	/// Sets the group for this light, setting nullptr here will remove the light from its current group
	/// \param	group	The group.
	/// \return	true if it succeeds, false if the group is not an OsgGroup.
	virtual bool setGroup(std::shared_ptr<SurgSim::Graphics::Group> group) = 0;

	/// Gets the group that this light has been assigned to.
	/// \return	The group or nullptr if no group has been set.
	virtual std::shared_ptr<SurgSim::Graphics::Group> getGroup() = 0;

	/// Sets diffuse color of this light.
	/// \param	color	The color.
	virtual void setDiffuseColor(const SurgSim::Math::Vector4d& color) = 0;

	/// Gets diffuse color.
	/// \return	The diffuse color.
	virtual SurgSim::Math::Vector4d getDiffuseColor() = 0;

	/// Sets specular color of this light.
	/// \param	color	The color.
	virtual void setSpecularColor(const SurgSim::Math::Vector4d& color) = 0;

	/// Gets specular color.
	/// \return	The specular color.
	virtual SurgSim::Math::Vector4d getSpecularColor() = 0;

	/// Sets constant attenuation.
	/// \param	val	The value.
	virtual void setConstantAttenuation(double val) = 0;

	/// Gets constant attenuation.
	/// \return	The constant attenuation.
	virtual double getConstantAttenuation() = 0;

	/// Sets linear attenuation.
	/// \param	val	The value.
	virtual void setLinearAttenuation(double val) = 0;

	/// Gets linear attenuation.
	/// \return	The linear attenuation.
	virtual double getLinearAttenuation() = 0;

	/// Sets quadratic attenuation.
	/// \param	val	The value.
	virtual void setQuadraticAttenuation(double val) = 0;

	/// Gets quadratic attenuation.
	/// \return	The quadratic attenuation.
	virtual double getQuadraticAttenuation() = 0;

	/// Sets the name of the group that this light should work on
	/// \param name The name of the group to light
	virtual void setLightGroupReference(const std::string& name) = 0;

	/// Gets the name of the group this light should operate on
	/// \return the name of the group for this light
	virtual std::string getLightGroupReference() = 0;
};

}; // Graphics
}; // SurgSim

#endif
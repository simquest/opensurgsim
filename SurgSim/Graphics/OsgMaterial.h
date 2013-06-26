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

#ifndef SURGSIM_GRAPHICS_OSGMATERIAL_H
#define SURGSIM_GRAPHICS_OSGMATERIAL_H

#include <SurgSim/Graphics/Material.h>

#include <osg/Material>
#include <osg/StateSet>

#include <set>

namespace SurgSim
{

namespace Graphics
{

class MaterialFace;
class OsgShader;
class OsgUniformBase;

/// OSG-based implementation of a graphics material.
///
/// Wraps an osg::StateSet which is applied to the osg::Node of the Representation that is assigned this material.
///
/// \note
/// Only uniforms that subclass OsgUniformBase and shaders that subclass OsgShader can be assigned to this material.
/// \sa OsgUniformBase
/// \sa OsgShader
class OsgMaterial : public Material
{
public:
	/// Constructor
	OsgMaterial();

	/// Adds a uniform to this material
	/// \param	uniform	Uniform to add
	/// \return	True if uniform was added successfully, otherwise false
	/// \note	OsgMaterial only accepts subclasses of OsgUniformBase
	virtual bool addUniform(std::shared_ptr<UniformBase> uniform);

	/// Removes a uniform from this material
	/// \param	uniform	Uniform to remove
	/// \return True if uniform was removed successfully, otherwise false
	/// \note	OsgMaterial only accepts subclasses of OsgUniformBase
	virtual bool removeUniform(std::shared_ptr<UniformBase> uniform);

	/// Returns the number of uniforms in this material
	virtual unsigned int getNumUniforms() const;

	/// Gets a uniform in this material
	/// \param	index	Index of the uniform in the material's list of uniforms
	/// \return	Uniform at the index
	virtual std::shared_ptr<UniformBase> getUniform(unsigned int index) const;

	/// Sets the shader used by this material
	/// \param	shader	Shader program
	/// \return	True if shader was set successfully, otherwise false
	/// \note	OsgMaterial only accepts subclasses of OsgShader
	virtual bool setShader(std::shared_ptr<Shader> shader);

	/// Gets the shader used by this material
	/// \return	Shader program
	virtual std::shared_ptr<Shader> getShader() const;

	/// Returns the OSG state set with the material properties
	osg::ref_ptr<osg::StateSet> getOsgStateSet() const
	{
		return m_stateSet;
	}

private:
	/// OSG state set which provides material properties in the scenegraph
	osg::ref_ptr<osg::StateSet> m_stateSet;

	/// Uniforms used by this material
	std::vector<std::shared_ptr<OsgUniformBase>> m_uniforms;
	/// Shader used by this material
	std::shared_ptr<OsgShader> m_shader;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGMATERIAL_H
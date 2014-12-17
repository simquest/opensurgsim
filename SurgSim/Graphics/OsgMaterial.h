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

#include "SurgSim/Graphics/Material.h"

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
	/// \post	The material has no uniforms and no shader.
	explicit OsgMaterial(const std::string& name);

	/// Adds a uniform to this material
	/// \param	uniform	Uniform to add
	/// \return	True if uniform was added successfully, otherwise false
	/// \note	OsgMaterial only accepts subclasses of OsgUniformBase
	virtual bool addUniform(std::shared_ptr<UniformBase> uniform) override;

	virtual bool addUniform(const std::string& type, const std::string& name) override;

	/// Removes a uniform from this material
	/// \param	uniform	Uniform to remove
	/// \return True if uniform was removed successfully, otherwise false
	/// \note	OsgMaterial only accepts subclasses of OsgUniformBase
	virtual bool removeUniform(std::shared_ptr<UniformBase> uniform) override;

	virtual bool removeUniform(const std::string& name) override;

	virtual size_t getNumUniforms() const override;

	virtual std::shared_ptr<UniformBase> getUniform(size_t index) const override;

	virtual std::shared_ptr<UniformBase> getUniform(const std::string& name) const override;

	virtual bool hasUniform(const std::string& name) const override;

	/// Sets the shader used by this material
	/// \param	shader	Shader program
	/// \return	True if shader was set successfully, otherwise false
	/// \note	OsgMaterial only accepts subclasses of OsgShader
	virtual bool setShader(std::shared_ptr<Shader> shader) override;

	virtual std::shared_ptr<Shader> getShader() const override;

	virtual void clearShader() override;

	/// \return the OSG state set with the material properties
	osg::ref_ptr<osg::StateSet> getOsgStateSet() const;

	virtual bool doInitialize() override;

	virtual bool doWakeUp() override;

private:
	/// OSG state set which provides material properties in the scenegraph
	osg::ref_ptr<osg::StateSet> m_stateSet;

	/// Uniforms used by this material
	std::vector<std::shared_ptr<OsgUniformBase>> m_uniforms;

	/// Shader used by this material
	std::shared_ptr<OsgShader> m_shader;
};

/// Utility function to build the material.
/// \param vertexShaderName name of the vertex shader to be used, needs to be available on the path.
/// \param fragmentShaderName name of the fragment shader to be used, needs to be available on the path.
/// \return a valid material if all the shaders are found, nullptr otherwise
std::shared_ptr<OsgMaterial> buildMaterial(
	const std::string& vertexShaderName,
	const std::string& fragmentShaderName);

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGMATERIAL_H
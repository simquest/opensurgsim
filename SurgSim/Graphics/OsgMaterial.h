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

#ifndef SURGSIM_GRAPHICS_OSGMATERIAL_H
#define SURGSIM_GRAPHICS_OSGMATERIAL_H

#include <boost/any.hpp>
#include <osg/Material>
#include <osg/StateSet>
#include <set>

#include "SurgSim/Graphics/Material.h"


namespace SurgSim
{

namespace Graphics
{

class MaterialFace;
class OsgProgram;
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

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgMaterial);

	/// Adds a uniform to this material
	/// \param	uniform	Uniform to add
	/// \note	OsgMaterial only accepts subclasses of OsgUniformBase
	void addUniform(std::shared_ptr<UniformBase> uniform) override;

	void addUniform(const std::string& type, const std::string& name) override;

	/// Adds and a uniform to this material and set its value
	/// \param type the type of the uniform
	/// \param name Name used in shader code to access this uniform
	/// \param value The value for this uniform
	void addUniform(const std::string& type, const std::string& name, const boost::any& value);

	/// Removes a uniform from this material
	/// \param	uniform	Uniform to remove
	/// \return True if uniform was removed successfully, otherwise false
	/// \note	OsgMaterial only accepts subclasses of OsgUniformBase
	bool removeUniform(std::shared_ptr<UniformBase> uniform) override;

	bool removeUniform(const std::string& name) override;

	size_t getNumUniforms() const override;

	std::shared_ptr<UniformBase> getUniform(size_t index) const override;

	std::shared_ptr<UniformBase> getUniform(const std::string& name) const override;

	bool hasUniform(const std::string& name) const override;

	/// Sets the shader used by this material
	/// \param	program	Shader program
	/// \return	True if program was set successfully, otherwise false
	/// \note OsgMaterial only accepts subclasses of OsgProgram
	bool setProgram(std::shared_ptr<Program> program) override;

	std::shared_ptr<Program> getProgram() const override;

	void clearProgram() override;

	/// \return the OSG state set with the material properties
	osg::ref_ptr<osg::StateSet> getOsgStateSet() const;

	bool doInitialize() override;

	bool doWakeUp() override;

private:
	/// OSG state set which provides material properties in the scenegraph
	osg::ref_ptr<osg::StateSet> m_stateSet;

	/// Uniforms used by this material
	std::vector<std::shared_ptr<OsgUniformBase>> m_uniforms;

	/// Shader used by this material
	std::shared_ptr<OsgProgram> m_program;
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

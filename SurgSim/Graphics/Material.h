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

#ifndef SURGSIM_GRAPHICS_MATERIAL_H
#define SURGSIM_GRAPHICS_MATERIAL_H

#include <memory>
#include <set>

namespace SurgSim
{

namespace Graphics
{

class UniformBase;
class Shader;

/// Base class that defines the interface for graphics materials.
///
/// Graphics materials define the visual appearance of graphics representations and are composed of the uniforms and
/// shaders applied used when rendering the geometry.
/// \sa	UniformBase
/// \sa Shader
class Material
{
public:
	/// Destructor.
	//  (Note that Visual Studio does not support "= default" yet.)
	virtual ~Material()
	{
	}

	/// Adds a uniform to this material
	/// \param	uniform	Uniform to add
	/// \return	True if uniform was added successfully, otherwise false
	virtual bool addUniform(std::shared_ptr<UniformBase> uniform) = 0;

	/// Removes a uniform from this material
	/// \param	uniform	Uniform to remove
	/// \return True if uniform was removed successfully, otherwise false
	virtual bool removeUniform(std::shared_ptr<UniformBase> uniform) = 0;

	/// Returns the number of uniforms in this material
	virtual unsigned int getNumUniforms() const = 0;

	/// Gets a uniform in this material
	/// \param	index	Index of the uniform in the material's list of uniforms
	/// \return	Uniform at the index
	virtual std::shared_ptr<UniformBase> getUniform(unsigned int index) const = 0;

	/// Sets the shader used by this material
	/// \param	shader	Shader program
	/// \return	True if shader was set successfully, otherwise false
	virtual bool setShader(std::shared_ptr<Shader> shader) = 0;

	/// Gets the shader used by this material
	/// \return	Shader program
	virtual std::shared_ptr<Shader> getShader() const = 0;

	/// Removes the shader from the material, falling back to fixed-function pipeline
	virtual void clearShader() = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_MATERIAL_H
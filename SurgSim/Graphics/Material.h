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

#ifndef SURGSIM_GRAPHICS_MATERIAL_H
#define SURGSIM_GRAPHICS_MATERIAL_H

#include <memory>
#include <set>

#include "SurgSim/Framework/Component.h"

namespace SurgSim
{

namespace Graphics
{

class UniformBase;
class Program;

/// Base class that defines the interface for graphics materials.
///
/// Graphics materials define the visual appearance of graphics representations and are composed of the uniforms and
/// shaders applied used when rendering the geometry.
/// \sa	UniformBase
/// \sa Shader
class Material : public SurgSim::Framework::Component
{
public:

	/// Constructor
	explicit Material(const std::string& name) : Component(name) {}

	/// Destructor.
	//  (Note that Visual Studio does not support "= default" yet.)
	virtual ~Material()
	{
	}

	/// Adds a uniform to this material.
	/// \param	uniform	Uniform to add.
	virtual void addUniform(std::shared_ptr<UniformBase> uniform) = 0;

	/// Adds a GLSL typed uniform to this material
	/// \param type the type of the uniform
	/// \param name the name that this uniform should have
	virtual void addUniform(const std::string& type, const std::string& name) = 0;

	/// Removes a uniform from this material.
	/// \param	uniform	Uniform to remove.
	/// \return True if uniform was removed successfully, otherwise false.
	virtual bool removeUniform(std::shared_ptr<UniformBase> uniform) = 0;

	/// Removes a uniform from this material.
	/// \param	name Uniform to remove.
	/// \return True if uniform was removed successfully, otherwise false.
	virtual bool removeUniform(const std::string& name) = 0;

	/// Gets a uniform in this material.
	/// \param	name	The name of the Uniform to fetch.
	/// \return	The uniform, nullptr if the uniform does not exist.
	virtual std::shared_ptr<UniformBase> getUniform(const std::string& name) const = 0;

	/// Gets a uniform in this material
	/// \param	index	Index of the uniform in the material's list of uniforms
	/// \return	Uniform at the index
	virtual std::shared_ptr<UniformBase> getUniform(size_t index) const = 0;

	/// Checks if this material has a uniform with the given name.
	/// \param	name	The name of the Uniform to check.
	/// \return	true if the uniform is in the material, false otherwise.
	virtual bool hasUniform(const std::string& name) const = 0;

	/// Returns the number of uniforms in this material.
	virtual size_t getNumUniforms() const = 0;

	virtual void loadProgram(const std::string& nameTuple) = 0;


	/// Sets the program used by this material.
	/// \param	program	Shader program.
	/// \return	True if program was set successfully, otherwise false.
	virtual bool setProgram(std::shared_ptr<Program> program) = 0;

	/// Gets the program used by this material.
	/// \return	Shader program.
	virtual std::shared_ptr<Program> getProgram() const = 0;

	/// Removes the shader from the material, falling back to fixed-function pipeline.
	virtual void clearProgram() = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_MATERIAL_H

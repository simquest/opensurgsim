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

#ifndef SURGSIM_GRAPHICS_OSGSHADER_H
#define SURGSIM_GRAPHICS_OSGSHADER_H

#include "SurgSim/Graphics/Program.h"

#include <osg/Program>
#include <osg/StateSet>

#include <string>
#include <memory>
#include <array>

namespace SurgSim
{
namespace Framework
{
class ApplicationData;
}

namespace Graphics
{

/// OSG-based implementation of a graphics shader.
///
/// Wraps an osg::Program which manages the geometry, vertex, and fragment shaders.
/// The osg::Program is added to the osg::StateSet of an osg::Node to use the shaders for the rendering of that
/// node's geometry.
class OsgProgram : public Program
{
public:
	/// Constructor
	/// \post	No shader code is set, so the fixed-function pipeline is used.
	OsgProgram();

	bool hasVertexShader() const override;

	void clearVertexShader() override;

	bool loadVertexShader(const std::string& filePath) override;

	void setVertexShaderSource(const std::string& source) override;

	bool getVertexShaderSource(std::string* source) const override;

	bool hasGeometryShader() const override;

	void clearGeometryShader() override;

	bool loadGeometryShader(const std::string& filePath) override;

	void setGeometryShaderSource(const std::string& source) override;

	bool getGeometryShaderSource(std::string* source) const override;

	bool hasFragmentShader() const override;

	void clearFragmentShader() override;

	bool loadFragmentShader(const std::string& filePath) override;

	void setFragmentShaderSource(const std::string& source) override;

	bool getFragmentShaderSource(std::string* source) const override;

	void setGlobalScope(bool val) override;

	bool isGlobalScope() const override;

	/// \return the OSG program attribute
	osg::ref_ptr<osg::Program> getOsgProgram() const;

	/// Adds this shader to the OSG state set
	/// \param	stateSet	OSG state set
	void addToStateSet(osg::StateSet* stateSet);

	/// Removes this uniform from the OSG state set
	/// \param	stateSet	OSG state set
	void removeFromStateSet(osg::StateSet* stateSet);

private:
	/// OSG program attribute
	osg::ref_ptr<osg::Program> m_program;

	// type of shader, internal use only
	enum ShaderType
	{
		SHADER_TYPE_VERTEX = 0,
		SHADER_TYPE_FRAGMENT,
		SHADER_TYPE_GEOMETRY,
		SHADER_TYPE_COUNT
	};

	/// Storage of the osg objects
	std::array<osg::ref_ptr<osg::Shader>, SHADER_TYPE_COUNT> m_osgShaders;

	/// Check whether there is a shader in use for the given type
	/// \param shaderType Type of the shader
	/// \return true if the shader has been set, otherwise false.
	bool hasShader(int shaderType) const;

	/// Removes the geometry shader, returning that portion of the shader program to fixed-function.
	/// \param shaderType Type of the shader
	void clearShader(int shaderType);

	/// Loads the shader source code from a file
	/// \param	filePath	Path to file containing shader source code
	/// \param shaderType Type of the shader
	/// \return	True if the source is successfully loaded, otherwise false.
	bool loadShaderSource(const std::string& filePath, int shaderType);

	/// Set the shader source code
	/// \param	source Shader source code
	/// \param shaderType Type of the shader
	virtual void setShaderSource(const std::string& source, int shaderType);

	/// Gets the shader source code
	/// \return	Shader source code
	virtual bool getShaderSource(int shaderType, std::string* source) const;

	/// Fetches the appropriate shader if it exists, creates it otherwise
	/// \param shaderType Type of the shader
	/// \return the shader with the given type
	osg::ref_ptr<osg::Shader> getOrCreateOsgShader(int shaderType);

	/// Is the shader supposed to be used globally
	bool m_globalScope;

};

/// Utility function, load a program from a set of shader files
/// \param data Application data object
/// \param name the base name of the shader files to be used '.vert' and '.frag' will be added automatically
std::shared_ptr<SurgSim::Graphics::OsgProgram> loadProgram(const SurgSim::Framework::ApplicationData& data,
		const std::string& name);

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGSHADER_H

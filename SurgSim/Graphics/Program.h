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

#ifndef SURGSIM_GRAPHICS_PROGRAM_H
#define SURGSIM_GRAPHICS_PROGRAM_H

#include <string>
#include "SurgSim/Framework/Accessible.h"

namespace SurgSim
{

namespace Framework
{
class Asset;
}

namespace Graphics
{

/// Base class that defines the interface for graphics programs.
///
/// A program in OSS is a collection of 'shaders' that are usually executed in sequence  (Geometry,
/// Vertex and Fragment). Each shader itself is software that is executed on the GPU, shaders are usually introduced
/// as source. The concept is similar to the use of program in
/// OpenGl see https://www.opengl.org/sdk/docs/man4/html/glCreateProgram.xhtml
class Program
{
public:

	/// Destructor
	virtual ~Program() = 0;

	/// \return true if the vertex shader has been set, otherwise false.
	virtual bool hasVertexShader() const = 0;

	/// Removes the vertex shader, returning that portion of the shader program to fixed-function.
	virtual void clearVertexShader() = 0;

	/// Loads the vertex shader source code from a file
	/// \param	filePath	Path to file containing shader source code
	/// \return	True if the source is successfully loaded, otherwise false.
	virtual bool loadVertexShader(const std::string& filePath) = 0;

	/// Set the vertex shader source code
	/// \param	source	Shader source code
	virtual void setVertexShaderSource(const std::string& source) = 0;

	/// Gets the vertex shader source code
	/// \return	Shader source code
	virtual bool getVertexShaderSource(std::string* source) const = 0;

	/// \return true if the geometry shader has been set, otherwise false.
	virtual bool hasGeometryShader() const = 0;

	/// Removes the geometry shader, returning that portion of the shader program to fixed-function.
	virtual void clearGeometryShader() = 0;

	/// Loads the geometry shader source code from a file
	/// \param	filePath	Path to file containing shader source code
	/// \return	True if the source is successfully loaded, otherwise false.
	virtual bool loadGeometryShader(const std::string& filePath) = 0;

	/// Set the geometry shader source code
	/// \param	source	Shader source code
	virtual void setGeometryShaderSource(const std::string& source) = 0;

	/// Gets the geometry shader source code
	/// \return	Shader source code
	virtual bool getGeometryShaderSource(std::string* source) const = 0;


	/// \return true if the fragment shader has been set, otherwise false.
	virtual bool hasFragmentShader() const = 0;

	/// \return the fragment shader, returning that portion of the shader program to fixed-function.
	virtual void clearFragmentShader() = 0;

	/// Loads the fragment shader source code from a file
	/// \param	filePath	Path to file containing shader source code
	/// \return	True if the source is successfully loaded, otherwise false.
	virtual bool loadFragmentShader(const std::string& filePath) = 0;

	/// Set the fragment shader source code
	/// \param	source	Shader source code
	virtual void setFragmentShaderSource(const std::string& source) = 0;

	/// Gets the fragment shader source code
	/// \return	Shader source code
	virtual bool getFragmentShaderSource(std::string* source) const = 0;

	/// Clears the entire shader, returning to fixed-function pipeline.
	virtual void clear()
	{
		clearVertexShader();
		clearGeometryShader();
		clearFragmentShader();
	}

	/// When this is set to true, this shader should be used instead of other shaders that might apply, depending
	/// on the hierarchy that is set out. E.g if this shader is on a camera, the shaders that occur in a group
	/// attached to that camera will be overridden.
	/// This will usually be used in conjunction with \sa RenderPass.
	/// \param	val	If true the shader should override shaders in lower levels.
	virtual void setGlobalScope(bool val) = 0;

	/// Query if this shader is of global scope.
	/// \return	true if global scope, false if not.
	virtual bool isGlobalScope() const = 0;

};

inline Program::~Program()
{
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_SHADER_H

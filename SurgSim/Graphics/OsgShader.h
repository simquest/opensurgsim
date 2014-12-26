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

#include "SurgSim/Graphics/Shader.h"

#include <osg/Program>
#include <osg/StateSet>

#include <string>
#include <memory>

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
class OsgShader : public Shader
{
public:
	/// Constructor
	/// \post	No shader code is set, so the fixed-function pipeline is used.
	OsgShader();

	/// Adds this shader to the OSG state set
	/// \param	stateSet	OSG state set
	virtual void addToStateSet(osg::StateSet* stateSet);

	/// Removes this uniform from the OSG state set
	/// \param	stateSet	OSG state set
	virtual void removeFromStateSet(osg::StateSet* stateSet);

	/// Returns true if the vertex shader has been set, otherwise false.
	virtual bool hasVertexShader() const;

	/// Removes the vertex shader, returning that portion of the shader program to fixed-function.
	virtual void clearVertexShader();

	/// Loads the vertex shader source code from a file
	/// \param	filePath	Path to file containing shader source code
	/// \return	True if the source is successfully loaded, otherwise false.
	virtual bool loadVertexShaderSource(const std::string& filePath);

	/// Set the vertex shader source code
	/// \param	source	Shader source code
	virtual void setVertexShaderSource(const std::string& source);

	/// Gets the vertex shader source code
	/// \return	Shader source code
	virtual bool getVertexShaderSource(std::string* source) const;

	/// Returns true if the geometry shader has been set, otherwise false.
	virtual bool hasGeometryShader() const;

	/// Removes the geometry shader, returning that portion of the shader program to fixed-function.
	virtual void clearGeometryShader();

	/// Loads the geometry shader source code from a file
	/// \param	filePath	Path to file containing shader source code
	/// \return	True if the source is successfully loaded, otherwise false.
	virtual bool loadGeometryShaderSource(const std::string& filePath);

	/// Set the geometry shader source code
	/// \param	source	Shader source code
	virtual void setGeometryShaderSource(const std::string& source);

	/// Gets the geometry shader source code
	/// \return	Shader source code
	virtual bool getGeometryShaderSource(std::string* source) const;


	/// Returns true if the fragment shader has been set, otherwise false.
	virtual bool hasFragmentShader() const;

	/// Removes the fragment shader, returning that portion of the shader program to fixed-function.
	virtual void clearFragmentShader();

	/// Loads the fragment shader source code from a file
	/// \param	filePath	Path to file containing shader source code
	/// \return	True if the source is successfully loaded, otherwise false.
	virtual bool loadFragmentShaderSource(const std::string& filePath);

	/// Set the fragment shader source code
	/// \param	source	Shader source code
	virtual void setFragmentShaderSource(const std::string& source);

	/// Gets the fragment shader source code
	/// \return	Shader source code
	virtual bool getFragmentShaderSource(std::string* source) const;

	/// Returns the OSG program attribute
	osg::ref_ptr<osg::Program> getOsgProgram() const;

	/// Enables the shader to override other material shaders
	/// \param	val	if true the shader will replace other shaders in a lower hierarchy.
	void setGlobalScope(bool val) override;

	/// Query if this object is global scope and overrides other lower level shaders.
	/// \return	true if global scope, false if not.
	bool isGlobalScope() const override;


private:
	/// OSG program attribute
	osg::ref_ptr<osg::Program> m_program;

	/// OSG vertex shader
	osg::ref_ptr<osg::Shader> m_vertexShader;
	/// OSG geometry shader
	osg::ref_ptr<osg::Shader> m_geometryShader;
	/// OSG fragment shader
	osg::ref_ptr<osg::Shader> m_fragmentShader;

	/// Is the shader supposed to be used globally
	bool m_globalScope;
};

std::shared_ptr<SurgSim::Graphics::OsgShader> loadShader(const SurgSim::Framework::ApplicationData& data,
		const std::string& name);


};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGSHADER_H

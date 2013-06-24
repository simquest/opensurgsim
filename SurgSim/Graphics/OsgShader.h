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

#include <SurgSim/Graphics/Shader.h>

#include <osg/Program>
#include <osg/StateSet>

namespace SurgSim
{

namespace Graphics
{

/// OSG-based implementation of a graphics shader.
///
/// Wraps an osg::Program which manages the geometry, vertex, and fragment shaders.
/// The osg::Program is added to the osg::StateSet of an osg::Node to use the shaders for the rendering of that
/// node's geometry.
/// \todo	Implement loading of the geometry/vertex/fragment shaders from files/strings.
class OsgShader : public Shader
{
public:
	/// Constructor
	OsgShader();

	/// Adds this shader to the OSG state set
	/// \param	stateSet	OSG state set
	virtual void addToStateSet(osg::StateSet* stateSet);

	/// Removes this uniform from the OSG state set
	/// \param	stateSet	OSG state set
	virtual void removeFromStateSet(osg::StateSet* stateSet);

	/// Returns the OSG program attribute
	osg::ref_ptr<osg::Program> getOsgProgram() const
	{
		return m_program;
	}

private:
	/// OSG program attribute
	osg::ref_ptr<osg::Program> m_program;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGSHADER_H

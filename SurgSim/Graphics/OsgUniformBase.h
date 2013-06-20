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

#ifndef SURGSIM_GRAPHICS_OSGUNIFORMBASE_H
#define SURGSIM_GRAPHICS_OSGUNIFORMBASE_H

#include <SurgSim/Graphics/Uniform.h>

#include <osg/Uniform>

namespace SurgSim
{

namespace Graphics
{

/// Base OSG implementation of graphics uniforms.
///
/// Wraps an osg::Uniform.
/// \note
/// SurgSim::Graphics::OsgUniform is templated on the type of value, so this base class allows a pointer to any type of
/// OSG Uniform.
class OsgUniformBase : public virtual UniformBase
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgUniformBase(const std::string& name) : UniformBase(),
		m_uniform(new osg::Uniform())
	{
		m_uniform->setName(name);
	}

	/// Returns the name used in shader code to access this uniform
	const std::string& getName() const
	{
		return m_uniform->getName();
	}

	/// Returns the OSG uniform node
	osg::ref_ptr<osg::Uniform> getOsgUniform() const
	{
		return m_uniform;
	}

protected:
	/// OSG uniform node
	osg::ref_ptr<osg::Uniform> m_uniform;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGUNIFORMBASE_H

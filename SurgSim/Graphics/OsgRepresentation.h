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

#ifndef SURGSIM_GRAPHICS_OSGACTOR_H
#define SURGSIM_GRAPHICS_OSGACTOR_H

#include <SurgSim/Graphics/Representation.h>

#include <osg/Node>

namespace SurgSim
{

namespace Graphics
{

/// Base OSG implementation of a graphics representation.
///
/// A Graphics::OsgRepresentation wraps a osg::Node that serves as the root node for this representation in the OSG scenegraph.
class OsgRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \param	node	Root OSG node of the representation
	explicit OsgRepresentation(const std::string& name, osg::Node* node) : Representation(name),
		m_node(node)
	{
	}

	/// Returns the root OSG node of the representation
	osg::ref_ptr<osg::Node> getOsgNode() const
	{
		return m_node;
	}

private:
	/// Root OSG node of the representation
	osg::ref_ptr<osg::Node> m_node;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGACTOR_H

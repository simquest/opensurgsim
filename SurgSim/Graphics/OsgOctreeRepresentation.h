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

#ifndef SURGSIM_GRAPHICS_OSGOCTREEREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGOCTREEREPRESENTATION_H

#include <memory>
#include <string>
#include <unordered_map>

#include <osg/ref_ptr>
#include <osg/Group>

#include "SurgSim/Graphics/OctreeRepresentation.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Math/OctreeShape.h"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif


namespace SurgSim
{
namespace Graphics
{

class OsgUnitBox;

/// OSG octree representation, implements an OctreeRepresenation using OSG.
class OsgOctreeRepresentation : public OctreeRepresentation, public OsgRepresentation
{
public:
	/// Constructor
	/// \param name Name of OsgOctreeRepresentation
	explicit OsgOctreeRepresentation(const std::string& name);

	/// Destructor
	~OsgOctreeRepresentation();

	/// Executes the update operation
	/// \param	dt	The time step
	virtual void doUpdate(double dt) override;

	/// Get the Octree contained by this representation
	/// \return	The octree contained by this representation.
	virtual std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> getOctree() const override;

	/// Set the Octree of this representation. The Octree is retrieved from a Math::OctreeShape.
	/// \param octreeShape The OctreeShape from which the octree is retrieved.
	virtual void setOctree(std::shared_ptr<SurgSim::Math::OctreeShape> octreeShape) override;

private:
	/// Draw the Octree associated with this OSG representation.
	/// \param thisTranform The osg::PositionAttitudeTransform.
	/// \param octreeNode The OctreeNode to be drawn.
	/// \param level The level at which octreeNode is.
	/// \param parentIndex The index of octreeNodes' parent.
	/// \param index The index of octreeNode.
	void draw(osg::ref_ptr<osg::Group> thisTransform, std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octreeNode,
			  unsigned level, unsigned parentIndex, unsigned index);

	/// The Octree represented by this representation
	std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> m_octree;

	/// Shared unit box, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitBox> m_sharedUnitBox;
	/// Returns the shared unit box
	static std::shared_ptr<OsgUnitBox> getSharedUnitBox();

	/// Dummy osg::Node used to subsititute the shared osg box when an OctreeNode is not active.
	osg::ref_ptr<osg::Node> m_dummy;

	/// A hash table recording if an OctreeNode (with an ID) has been added to the scene graph tree.
	std::unordered_map<unsigned, bool> m_nodeAdded;

	/// Determine the index of an OctreeNode in the corresponding scene graph tree.
	/// At a given level, the 2nd child of an OctreeNode may be added to the corresponding scene graph tree as its 5th
	/// child.
	std::unordered_map<unsigned, unsigned> m_nodeIndex;
};

}; // Graphics
}; // SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif // SURGSIM_GRAPHICS_OSGOCTREEREPRESENTATION_H
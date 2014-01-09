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

#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/OctreeRepresentation.h"

namespace SurgSim
{
namespace Graphics
{

class OsgUnitBox;

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

/// OSG octree representation, implements an OctreeRepresenation using OSG.
template <class Data>
class OsgOctreeRepresentation : public OctreeRepresentation<Data>, public OsgRepresentation
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

	/// Get the Octree of this representation
	/// \return	The octree used by this representation.
	virtual std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> getOctree() const override;

	/// Set the Octree of this representation
	/// \param The Octree to be used in this representation.
	virtual void setOctree(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> octree) override;

private:

	/// To draw the given Octree Node
	/// \param octreeNode Octree node to be drawn
	/// \return An osg::PositionAttitudeTransform containing the OSG representatoin of the Octree node
	osg::ref_ptr<osg::PositionAttitudeTransform> draw(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> octreeNode);

	/// The Octree represented by this representation
	std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> m_octree;

	/// Shared unit box, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitBox> m_sharedUnitBox;
	/// Returns the shared unit box
	static std::shared_ptr<OsgUnitBox> getSharedUnitBox();
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#include "SurgSim/Graphics/OsgOctreeRepresentation-inl.h"

#endif // SURGSIM_GRAPHICS_OSGOCTREEREPRESENTATION_H
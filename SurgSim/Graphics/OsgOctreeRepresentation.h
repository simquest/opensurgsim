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
/// Given a OctreeShape, this representation will copy the Octree instead of sharing the Octree (with the OctreeShape).
/// Wake up call on this representation will fail if no octree is held.
/// That is to say, setOctree() method MUST be called before WakeUp() to make this representation work properly.
/// The OSG tree corresponds to the Octree will be built only once at wake up and can not be changed once awake.
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

	/// Wake up this representation, build the OSG corresponds to the Octree it holds.
	/// \return True if OSG tree is built successfully.
	virtual bool doWakeUp() override;

	/// Get the Octree contained by this representation
	/// \return	The octree contained by this representation.
	virtual std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> getOctree() const override;

	/// Set the Octree of this representation. The Octree is retrieved from a Math::OctreeShape.
	/// \param octreeShape The OctreeShape from which the octree is retrieved.
	virtual void setOctree(const SurgSim::Math::OctreeShape& octreeShape) override;

private:
	/// Draw the Octree associated with this OSG representation.
	/// \param parentTransformNode The osg::PositionAttitudeTransform node under which either the octreeNode or its
	///                            children will be drawn.
	/// \param octreeNode The OctreeNode to be drawn.
	void buildOctree(osg::ref_ptr<osg::Group> parentTransformNode,
					 std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree);

	/// The Octree represented by this representation
	std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> m_octree;

	/// Shared unit box, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitBox> m_sharedUnitBox;
	/// Returns the shared unit box
	static std::shared_ptr<OsgUnitBox> getSharedUnitBox();
};

}; // Graphics
}; // SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif // SURGSIM_GRAPHICS_OSGOCTREEREPRESENTATION_H
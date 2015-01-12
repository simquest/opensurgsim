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

#include "SurgSim/Framework/ObjectFactory.h"
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

SURGSIM_STATIC_REGISTRATION(OsgOctreeRepresentation);

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

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgOctreeRepresentation);

	/// Executes the update operation
	/// \param	dt	The time step
	void doUpdate(double dt) override;

	void setOctreeShape(const std::shared_ptr<SurgSim::Math::Shape>& shape) override;
	std::shared_ptr<SurgSim::Math::OctreeShape> getOctreeShape() const override;

	/// Mark the OctreeNode visible/invisible in the given a OctreePath (typedef-ed in OctreeNode.h).
	/// \param path An OctreePath, giving the path leads to the OctreeNode whose visibility to be changed.
	/// \param visibility Whether or not the OctreeNode specified by 'path' is visible or not.
	void setNodeVisible(const SurgSim::DataStructures::OctreePath& path, bool visibility) override;

private:
	/// Draw the Octree associated with this OSG representation.
	/// \param parentTransformNode The osg::PositionAttitudeTransform node under which either the octreeNode or its
	///                            children will be drawn.
	/// \param octree The octree to be drawn.
	void buildOctree(osg::ref_ptr<osg::PositionAttitudeTransform> parentTransformNode,
					 std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree);

	/// Shared unit box, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitBox> m_sharedUnitBox;

	/// The OctreeShape whose Octree will be visualized.
	std::shared_ptr<SurgSim::Math::OctreeShape> m_octreeShape;

	/// Returns the shared unit box
	static std::shared_ptr<OsgUnitBox> getSharedUnitBox();
};

}; // Graphics
}; // SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif // SURGSIM_GRAPHICS_OSGOCTREEREPRESENTATION_H
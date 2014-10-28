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

#include <osg/PositionAttitudeTransform>

#include "SurgSim/Graphics/OsgOctreeRepresentation.h"

#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgUnitBox.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgOctreeRepresentation, OsgOctreeRepresentation);

using SurgSim::Math::Vector3d;

OsgOctreeRepresentation::OsgOctreeRepresentation(const std::string& name) :
	Representation(name),
	OctreeRepresentation(name),
	OsgRepresentation(name),
	m_sharedUnitBox(getSharedUnitBox())
{
}

OsgOctreeRepresentation::~OsgOctreeRepresentation()
{
}

void OsgOctreeRepresentation::doUpdate(double dt)
{
}

// An Octree(Node) is traversed in following order (the 2nd OctreeNode, i.e. OctreeNode with "1" is now shown):
/*
				________
			   /3  /  7/|
			  /-------/ |
			 /2__/_6_/| |
			|   |   | |/|
			|___|___|/|5|
			|   |   | |/
			|0__|__4|/
*/
void OsgOctreeRepresentation::buildOctree(osg::ref_ptr<osg::PositionAttitudeTransform> transformNode,
		std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree)
{
	SURGSIM_ASSERT(!isAwake()) << "OsgOctreeRepresentation::buildOctree() should be called before wake up.";
	osg::ref_ptr<osg::PositionAttitudeTransform> osgTransform = new osg::PositionAttitudeTransform();
	transformNode->addChild(osgTransform);

	if (octree->hasChildren())
	{
		auto octreeChildren = octree->getChildren();
		for (int i = 0; i < 8; ++i)
		{
			buildOctree(osgTransform, octreeChildren[i]);
		}
	}
	else
	{
		osgTransform->addChild(m_sharedUnitBox->getNode());
		osgTransform->setPosition(toOsg(static_cast<Vector3d>(octree->getBoundingBox().center())));
		osgTransform->setScale(toOsg(static_cast<Vector3d>(octree->getBoundingBox().sizes())));

		int nodeMask = octree->isActive() ? 0xffffffff : 0;
		osgTransform->setNodeMask(nodeMask);
	}
}

void OsgOctreeRepresentation::setOctreeShape(const std::shared_ptr<SurgSim::Math::Shape>& shape)
{
	SURGSIM_ASSERT(!isAwake()) << "OsgOctreeRepresentation::setOctree() should be called before wake up.";

	auto octreeShape = std::dynamic_pointer_cast<SurgSim::Math::OctreeShape>(shape);
	SURGSIM_ASSERT(octreeShape != nullptr) << "OsgOctreeRepresentation can only accept an OctreeShape.";
	m_octreeShape = octreeShape;

	buildOctree(m_transform, m_octreeShape->getOctree());
}

std::shared_ptr<SurgSim::Math::OctreeShape> OsgOctreeRepresentation::getOctreeShape() const
{
	return m_octreeShape;
}

void OsgOctreeRepresentation::setNodeVisible(const SurgSim::DataStructures::OctreePath& path, bool visibility)
{
	SURGSIM_ASSERT(0 != m_transform->getNumChildren()) << "No Octree held by OsgOctreeRepresentation";

	osg::ref_ptr<osg::Group> result = m_transform->getChild(0)->asGroup();
	for (auto index = std::begin(path); index != std::end(path); ++index)
	{
		SURGSIM_ASSERT(result->getNumChildren() > 1) <<
				"OsgOctreeRepresentation::setNodeVisible(): Invalid OctreePath";

		result = result->getChild(*index)->asGroup();
	}
	result->setNodeMask(visibility ? 0xffffffff : 0);
}

std::shared_ptr<SurgSim::Graphics::OsgUnitBox> OsgOctreeRepresentation::getSharedUnitBox()
{
	static SurgSim::Framework::SharedInstance<OsgUnitBox> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim

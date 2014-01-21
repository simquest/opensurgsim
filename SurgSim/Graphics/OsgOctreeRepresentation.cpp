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
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgUnitBox.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{

using SurgSim::Math::Vector3d;

OsgOctreeRepresentation::OsgOctreeRepresentation(const std::string& name) :
	Representation(name),
	OctreeRepresentation(name),
	OsgRepresentation(name),
	m_octree(nullptr),
	m_sharedUnitBox(getSharedUnitBox()),
	m_dummy(new osg::Node())
{
	m_transform->addChild(new osg::PositionAttitudeTransform());
}


OsgOctreeRepresentation::~OsgOctreeRepresentation()
{
}


void OsgOctreeRepresentation::doUpdate(double dt)
{
	SURGSIM_ASSERT(m_octree) << "OsgOctreeRepresentation::doUpdate(): No Octree attached.";

	// Traverse the Octree and the corresponding OSG tree.
	// A new node will be added to the OSG tree if it is not present.
	// Draw the OSG node if the corresponding OctreeNode is active, i.e. leaf node with data.
	draw(m_transform->getChild(0)->asGroup(), m_octree, 0, 0, 0);
}

void OsgOctreeRepresentation::draw
	(osg::ref_ptr<osg::Group> thisTransform, std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree,
	 unsigned level, unsigned parentIndex, unsigned index)
{
	// The IDs of children in a Octree are as follows (ID 1 is not shown):
	/*
				     _______ 
				   /3  /  7/|
				  /-------/ |
				 /2__/_6_/| |
				|   |   | |/|
				|___|___|/|5|
				|   |   | |/
				|0__|__4|/
	*/
	// Get ID for this OctreeNode
	int base = 0;
	for (int n = static_cast<int>(level) - 1; n >= 0; --n)
	{
		base += static_cast<int>(pow(8.0, n));
	}
	unsigned nodeID = static_cast<unsigned>(base) + 8 * (parentIndex) + index;


	if (m_nodeAdded[nodeID])
	{
		auto transformNode = thisTransform->getChild(m_nodeIndex[nodeID])->asGroup();
		if (octree->hasChildren())
		{
			// Scale and position is controlled by leaf node.
			// If a node has child, its scale and position will be set to 1 and the origin, respectively.
			transformNode->replaceChild(m_sharedUnitBox->getNode(), m_dummy);
			transformNode->asTransform()->asPositionAttitudeTransform()->setScale(osg::Vec3d(1.0, 1.0, 1.0));
			transformNode->asTransform()->asPositionAttitudeTransform()->setPosition(osg::Vec3d(0.0, 0.0, 0.0));

			auto octreeChildren = octree->getChildren();
			for(int i = 0; i < 8; ++i)
			{
				draw(transformNode, octreeChildren[i], level + 1, index, i);
			}
		}
		else if (octree->isActive())
		{
			transformNode->replaceChild(m_dummy, m_sharedUnitBox->getNode());
		}
	}
	else
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> osgTransform = new osg::PositionAttitudeTransform();
		osgTransform->addChild(m_dummy);

		Vector3d distance = octree->getBoundingBox().center();
		osgTransform->setPosition(toOsg(distance));
		Vector3d size = octree->getBoundingBox().sizes();
		osgTransform->setScale(toOsg(size));

		thisTransform->addChild(osgTransform);
		// Hash this OctreeNode's location in the corresponding OSG node.
		m_nodeIndex[nodeID] = thisTransform->getNumChildren() - 1;
		m_nodeAdded[nodeID] = true;

		// After add a OSG node for this OctreeNode, need to draw this node or its descendant.
		draw(thisTransform, octree, level, parentIndex, index);
	}
}


std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> OsgOctreeRepresentation::getOctree() const
{
	return m_octree;
}


void SurgSim::Graphics::OsgOctreeRepresentation::setOctree(std::shared_ptr<SurgSim::Math::OctreeShape> octreeShape)
{
	m_octree = octreeShape->getRootNode();
	doUpdate(0.0);
}


std::shared_ptr<SurgSim::Graphics::OsgUnitBox> OsgOctreeRepresentation::getSharedUnitBox()
{
	static SurgSim::Framework::SharedInstance<OsgUnitBox> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim

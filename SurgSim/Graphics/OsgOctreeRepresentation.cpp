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
	draw(m_transform->getChild(0)->asGroup(), m_octree);
}


// An Octree(Node) is traversed in following order (the 2nd OctreeNode, i.e. OctreeNode with "1" is now shown): 
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
void OsgOctreeRepresentation::draw
	(osg::ref_ptr<osg::Group> parentTransformNode, std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree)
{
	const Vector3d key = octree->getBoundingBox().center();
	auto result = find_if(m_nodeMap.cbegin(), m_nodeMap.cend(), 
						  [&key](const std::pair<SurgSim::Math::Vector3d, unsigned>& item){ return item.first == key; }
						 );

	if (m_nodeMap.cend() != result)
	{
		auto thisTransformNode = parentTransformNode->getChild(result->second)->asGroup();
		if (octree->hasChildren())
		{
			// Scale and position is controlled by leaf node.
			// If a node has child, its scale and position will be set to 1 and the origin, respectively.
			thisTransformNode->replaceChild(m_sharedUnitBox->getNode(), m_dummy);
			thisTransformNode->asTransform()->asPositionAttitudeTransform()->setScale(osg::Vec3d(1.0, 1.0, 1.0));
			thisTransformNode->asTransform()->asPositionAttitudeTransform()->setPosition(osg::Vec3d(0.0, 0.0, 0.0));

			auto octreeChildren = octree->getChildren();
			for(int i = 0; i < 8; ++i)
			{
				draw(thisTransformNode, octreeChildren[i]);
			}
		}
		else if (octree->isActive())
		{
			thisTransformNode->replaceChild(m_dummy, m_sharedUnitBox->getNode());
		}
	}
	else
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> osgTransform = new osg::PositionAttitudeTransform();
		osgTransform->addChild(m_dummy);
		osgTransform->setPosition(toOsg(static_cast<Vector3d>(octree->getBoundingBox().center())));
		osgTransform->setScale(toOsg(static_cast<Vector3d>(octree->getBoundingBox().sizes())));

		parentTransformNode->addChild(osgTransform);
		m_nodeMap.emplace_back(key, parentTransformNode->getNumChildren() - 1);

		// After add a OSG node for this OctreeNode, need to draw this node or its descendant.
		draw(parentTransformNode, octree);
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

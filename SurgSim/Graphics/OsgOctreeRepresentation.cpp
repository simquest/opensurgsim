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
}


OsgOctreeRepresentation::~OsgOctreeRepresentation()
{
}


void OsgOctreeRepresentation::doUpdate(double dt)
{
	SURGSIM_ASSERT(m_octree) << "OsgOctreeRepresentation::doUpdate(): No Octree attached.";
	
	draw(m_transform, m_octree->getBoundingBox().center(), m_octree, 0, 0, 0);
}

void OsgOctreeRepresentation::draw
	(osg::ref_ptr<osg::Group> thisTransform, SurgSim::Math::Vector3d parentCenter, std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree, unsigned level, unsigned parentIndex, unsigned index)
{
	int base = 0;
	for (int n = (int)level - 1; n >= 0; --n)
	{
		base += (int)pow(8.0, n);
	}
	unsigned nodeID = base + 8 * (parentIndex) + index;


	if (m_nodeAdded[nodeID])
	{
		auto transformNode = thisTransform->getChild(m_nodeIndex[nodeID])->asGroup();
		if (octree->hasChildren())
		{
			transformNode->replaceChild(m_sharedUnitBox->getNode(), m_dummy);

			auto octreeChildren = octree->getChildren();
			for(int i = 0; i < 8; ++i)
			{
				draw(transformNode, octree->getBoundingBox().center(), octreeChildren[i], level + 1, index, i);
			}
		}
		else if (octree->isActive())
		{
			transformNode->replaceChild(m_dummy, m_sharedUnitBox->getNode());
		}
	}
	else
	{
		addNode(thisTransform, octree, nodeID, parentCenter);
		draw(thisTransform, parentCenter, octree, level, parentIndex, index);
	}
}


std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> OsgOctreeRepresentation::getOctree() const
{
	return m_octree;
}


void SurgSim::Graphics::OsgOctreeRepresentation::setOctree(std::shared_ptr<SurgSim::Math::OctreeShape> octreeShape)
{
	m_octree = octreeShape->getRootNode();
	addNode(m_transform, m_octree, 0, m_octree->getBoundingBox().center());
}

void SurgSim::Graphics::OsgOctreeRepresentation::addNode(osg::ref_ptr<osg::Group> thisTransform, std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree, unsigned nodeID, Vector3d parentCenter)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> osgTransform = new osg::PositionAttitudeTransform();
	if (octree->isActive())
	{
		osgTransform->addChild(m_sharedUnitBox->getNode());
	}
	else
	{
		osgTransform->addChild(m_dummy);
	}

	
	Vector3d distance = octree->getBoundingBox().center() - parentCenter;
	osgTransform->setPosition(toOsg(distance));
	Vector3d size = octree->getBoundingBox().sizes();
	osgTransform->setScale(toOsg(size));

	thisTransform->asTransform()->asPositionAttitudeTransform()->setScale(osg::Vec3d(1.0, 1.0, 1.0));
	thisTransform->addChild(osgTransform);
	m_nodeIndex[nodeID] = thisTransform->getNumChildren() - 1;
	m_nodeAdded[nodeID] = true;
}


std::shared_ptr<SurgSim::Graphics::OsgUnitBox> OsgOctreeRepresentation::getSharedUnitBox()
{
	static SurgSim::Framework::SharedInstance<OsgUnitBox> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim

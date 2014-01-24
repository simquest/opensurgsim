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

using SurgSim::Math::Vector3d;

OsgOctreeRepresentation::OsgOctreeRepresentation(const std::string& name) :
	Representation(name),
	OctreeRepresentation(name),
	OsgRepresentation(name),
	m_octree(nullptr),
	m_sharedUnitBox(getSharedUnitBox())
{
}


OsgOctreeRepresentation::~OsgOctreeRepresentation()
{
}


void OsgOctreeRepresentation::doUpdate(double dt)
{
}


bool OsgOctreeRepresentation::doWakeUp()
{
	if (!m_octree)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
			<< "OsgOctreeRepresentation::doWakeUp(): No Octree held when waking up.";
		return false;
	}

	buildOctree(m_transform, m_octree);
	return true;
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
void OsgOctreeRepresentation::buildOctree
	(osg::ref_ptr<osg::Group> transformNode, std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> octree)
{
	SURGSIM_ASSERT(!isAwake()) << "OsgOctreeRepresentation::buildOctree() should be called before wake up.";
	if (octree->isActive())
	{
		if (octree->hasChildren())
		{
			auto octreeChildren = octree->getChildren();
			for(int i = 0; i < 8; ++i)
			{
				buildOctree(transformNode, octreeChildren[i]);
			}
		}
		else
		{
			osg::ref_ptr<osg::PositionAttitudeTransform> osgTransform = new osg::PositionAttitudeTransform();
			osgTransform->addChild(m_sharedUnitBox->getNode());
			osgTransform->setPosition(toOsg(static_cast<Vector3d>(octree->getBoundingBox().center())));
			osgTransform->setScale(toOsg(static_cast<Vector3d>(octree->getBoundingBox().sizes())));
			transformNode->addChild(osgTransform);
		}
	}
}

void OsgOctreeRepresentation::setOctree(const std::shared_ptr<SurgSim::Math::OctreeShape>& octreeShape)
{
	SURGSIM_ASSERT(!isAwake()) << "OsgOctreeRepresentation::setOctree() should be called before wake up.";
	m_octree = std::make_shared<SurgSim::Math::OctreeShape::NodeType>(*(octreeShape)->getRootNode());
}

std::shared_ptr<SurgSim::Math::OctreeShape::NodeType> OsgOctreeRepresentation::getOctree() const
{
	return m_octree;
}

std::shared_ptr<SurgSim::Graphics::OsgUnitBox> OsgOctreeRepresentation::getSharedUnitBox()
{
	static SurgSim::Framework::SharedInstance<OsgUnitBox> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim

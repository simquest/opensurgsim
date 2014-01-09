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

#include <osg/PositionAttitudeTransform>

#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgUnitBox.h"


namespace SurgSim
{
namespace Graphics
{

template<class Data>
OsgOctreeRepresentation<Data>::OsgOctreeRepresentation(const std::string& name) :
	Representation(name),
	OctreeRepresentation<Data>(name),
	OsgRepresentation(name),
	m_sharedUnitBox(getSharedUnitBox())
{
	// Center the unit boundingBox at (0.0, 0.0, 0.0)
	Eigen::AlignedBox<double, 3> boundingBox;
	boundingBox.min() = Vector3d::Ones() * -0.5;
	boundingBox.max() = Vector3d::Ones() * 0.5;
	m_octree = std::make_shared<SurgSim::DataStructures::OctreeNode<Data>>(boundingBox);
	m_transform->addChild(draw(m_octree));
}

template<class Data>
OsgOctreeRepresentation<Data>::~OsgOctreeRepresentation()
{
}

template<class Data>
void OsgOctreeRepresentation<Data>::doUpdate(double dt)
{
	m_transform->removeChildren(0, m_transform->getNumChildren());
	m_transform->addChild(draw(m_octree));
}

template<class Data>
osg::ref_ptr<osg::PositionAttitudeTransform> OsgOctreeRepresentation<Data>::draw(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> octree)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> osgTransform = new osg::PositionAttitudeTransform();
	if (octree->hasChildren())
	{
		auto octreeChildren = octree->getChildren();
		for(int index = 0; index < 8; ++index)
		{
			osgTransform->addChild(draw(octreeChildren[index]));
		}
	}
	else if (octree->isActive())
	{
		osgTransform->addChild(m_sharedUnitBox->getNode());
		auto boundingBox = octree->getBoundingBox();

		// Translate the corresponding UnitBox of Octree node to its position
		Vector3d translation = (boundingBox.max() + boundingBox.min()) / 2.0;
		osgTransform->setPosition(toOsg(translation));

		// Scale the UnitBox based on the size of Octree node
		Vector3d octreeSize = boundingBox.max() - boundingBox.min();
		osgTransform->setScale(toOsg(octreeSize));
	}
	return osgTransform;
}

template<class Data>
std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> OsgOctreeRepresentation<Data>::getOctree() const
{
	return m_octree;
}


template <class Data>
void SurgSim::Graphics::OsgOctreeRepresentation<Data>::setOctree(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> octree)
{
	m_octree = octree;
}


template<class Data>
std::shared_ptr<SurgSim::Graphics::OsgUnitBox> OsgOctreeRepresentation<Data>::getSharedUnitBox()
{
	static SurgSim::Framework::SharedInstance<OsgUnitBox> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim